// SPDX-License-Identifier: Apache-2.0
// IDF 6's public PSA hash API selects Espressif's locked SHA peripheral driver.
// Preserve libsrtp's HMAC-SHA1 wire tags, including incremental RTP/ROC input.
#include "auth.h"
#include "alloc.h"
#include "err.h"
#include "auth_test_cases.h"
#include "psa/crypto.h"
#include <string.h>

typedef struct {
    psa_hash_operation_t inner;
    psa_hash_operation_t outer;
    psa_hash_operation_t work;
} gea_hmac_state_t;

typedef struct {
    srtp_auth_t auth;
    gea_hmac_state_t state;
} gea_hmac_t;

extern const srtp_auth_type_t srtp_hmac;
srtp_debug_module_t srtp_mod_hmac = { false, "hmac sha-1 psa" };

static srtp_err_status_t gea_hmac_alloc(srtp_auth_t **auth, size_t key_len, size_t out_len) {
    if (key_len > 20 || out_len > 20) return srtp_err_status_bad_param;
    gea_hmac_t *instance = srtp_crypto_alloc(sizeof(*instance));
    if (!instance) return srtp_err_status_alloc_fail;
    instance->state.inner = psa_hash_operation_init();
    instance->state.outer = psa_hash_operation_init();
    instance->state.work = psa_hash_operation_init();
    instance->auth.type = &srtp_hmac;
    instance->auth.state = &instance->state;
    instance->auth.key_len = key_len;
    instance->auth.out_len = out_len;
    instance->auth.prefix_len = 0;
    *auth = &instance->auth;
    return srtp_err_status_ok;
}

static srtp_err_status_t gea_hmac_dealloc(srtp_auth_t *auth) {
    gea_hmac_t *instance = (gea_hmac_t *)auth;
    psa_hash_abort(&instance->state.work);
    psa_hash_abort(&instance->state.inner);
    psa_hash_abort(&instance->state.outer);
    octet_string_set_to_zero(instance, sizeof(*instance));
    srtp_crypto_free(instance);
    return srtp_err_status_ok;
}

static srtp_err_status_t gea_hmac_start(void *context) {
    gea_hmac_state_t *state = context;
    if (psa_hash_abort(&state->work) != PSA_SUCCESS ||
        psa_hash_clone(&state->inner, &state->work) != PSA_SUCCESS)
        return srtp_err_status_auth_fail;
    return srtp_err_status_ok;
}

static srtp_err_status_t gea_hmac_init(void *context, const uint8_t *key, size_t key_len) {
    if (key_len > 20 || (key_len && !key)) return srtp_err_status_bad_param;
    gea_hmac_state_t *state = context;
    uint8_t inner_pad[64], outer_pad[64];
    memset(inner_pad, 0x36, sizeof(inner_pad));
    memset(outer_pad, 0x5c, sizeof(outer_pad));
    for (size_t i = 0; i < key_len; ++i) {
        inner_pad[i] ^= key[i];
        outer_pad[i] ^= key[i];
    }
    psa_hash_abort(&state->work);
    psa_hash_abort(&state->inner);
    psa_hash_abort(&state->outer);
    const int ok = psa_crypto_init() == PSA_SUCCESS &&
        psa_hash_setup(&state->inner, PSA_ALG_SHA_1) == PSA_SUCCESS &&
        psa_hash_update(&state->inner, inner_pad, sizeof(inner_pad)) == PSA_SUCCESS &&
        psa_hash_setup(&state->outer, PSA_ALG_SHA_1) == PSA_SUCCESS &&
        psa_hash_update(&state->outer, outer_pad, sizeof(outer_pad)) == PSA_SUCCESS;
    octet_string_set_to_zero(inner_pad, sizeof(inner_pad));
    octet_string_set_to_zero(outer_pad, sizeof(outer_pad));
    if (!ok) return srtp_err_status_auth_fail;
    return gea_hmac_start(state);
}

static srtp_err_status_t gea_hmac_update(void *context, const uint8_t *message, size_t size) {
    gea_hmac_state_t *state = context;
    if (size && !message) return srtp_err_status_bad_param;
    if (size && psa_hash_update(&state->work, message, size) != PSA_SUCCESS)
        return srtp_err_status_auth_fail;
    return srtp_err_status_ok;
}

static srtp_err_status_t gea_hmac_compute(void *context, const uint8_t *message, size_t size,
                                        size_t tag_len, uint8_t *tag) {
    gea_hmac_state_t *state = context;
    if (tag_len > 20 || (tag_len && !tag)) return srtp_err_status_bad_param;
    uint8_t inner_digest[20] = {0}, digest[20] = {0};
    size_t length = 0;
    const int ok = gea_hmac_update(state, message, size) == srtp_err_status_ok &&
        psa_hash_finish(&state->work, inner_digest, sizeof(inner_digest), &length) == PSA_SUCCESS &&
        length == sizeof(inner_digest) &&
        psa_hash_clone(&state->outer, &state->work) == PSA_SUCCESS &&
        psa_hash_update(&state->work, inner_digest, sizeof(inner_digest)) == PSA_SUCCESS &&
        psa_hash_finish(&state->work, digest, sizeof(digest), &length) == PSA_SUCCESS &&
        length == sizeof(digest);
    if (ok && tag_len) memcpy(tag, digest, tag_len);
    octet_string_set_to_zero(inner_digest, sizeof(inner_digest));
    octet_string_set_to_zero(digest, sizeof(digest));
    if (!ok) {
        psa_hash_abort(&state->work);
        return srtp_err_status_auth_fail;
    }
    return srtp_err_status_ok;
}

const srtp_auth_type_t srtp_hmac = {
    gea_hmac_alloc, gea_hmac_dealloc, gea_hmac_init, gea_hmac_compute,
    gea_hmac_update, gea_hmac_start, "HMAC-SHA1 through ESP-IDF PSA",
    &srtp_hmac_test_case_0, SRTP_HMAC_SHA1
};
