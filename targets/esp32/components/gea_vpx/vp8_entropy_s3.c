// SPDX-License-Identifier: BSD-3-Clause
// Differential verification of the S3 entropy optimization against libvpx's
// original arithmetic-bit algorithm (libvpx 1.16.0, BSD-licensed).
#include "vp8_s3.h"
#include "vp8/decoder/dboolhuff.h"
#include "esp_timer.h"
#include <string.h>

static __attribute__((noinline)) int reference_bit(BOOL_DECODER *br, int probability) {
  const unsigned split = 1 + (((br->range - 1) * probability) >> 8);
  if (br->count < 0) vp8dx_bool_decoder_fill(br);
  VP8_BD_VALUE value = br->value;
  int count = br->count;
  const VP8_BD_VALUE bigsplit = (VP8_BD_VALUE)split << (VP8_BD_VALUE_SIZE - 8);
  unsigned range = split;
  int bit = 0;
  if (value >= bigsplit) {
    range = br->range - split;
    value -= bigsplit;
    bit = 1;
  }
  const unsigned shift = vp8_norm[(unsigned char)range];
  br->range = range << shift;
  br->value = value << shift;
  br->count = count - shift;
  return bit;
}

static void decrypt(void *opaque, const unsigned char *input, unsigned char *output, int size) {
  const unsigned char key = *(const unsigned char *)opaque;
  for (int i = 0; i < size; ++i) output[i] = input[i] ^ key;
}

int gea_vp8_entropy_check_s3(struct gea_vp8_entropy_result *result) {
  memset(result, 0, sizeof(*result));
  // Normalization is defined only for positive ranges. Every range reachable
  // from either arithmetic branch is covered, including probability 0/255.
  for (unsigned range = 1; range <= 255; ++range)
    if (vp8_norm[range] != __builtin_clz(range) - 24) return 0;
  uint32_t random = 0xa154f63du;
  unsigned char data[128], probabilities[256], key = 0x53;
  for (unsigned test = 0; test < 4096; ++test) {
    for (unsigned i = 0; i < sizeof(probabilities); ++i) {
      random ^= random << 13; random ^= random >> 17; random ^= random << 5;
      probabilities[i] = (unsigned char)random;
      if (i < sizeof(data)) data[i] = (unsigned char)(random >> 8);
    }
    // Includes zero-length, truncated and encrypted inputs. Compare the
    // entire arithmetic state after every bit, not only the decoded output.
    BOOL_DECODER scalar, native;
    vpx_decrypt_cb callback = test & 1 ? decrypt : NULL;
    const unsigned length = test % (sizeof(data) + 1);
    vp8dx_start_decode(&scalar, data, length, callback, &key);
    native = scalar;
    for (unsigned i = 0; i < sizeof(probabilities); ++i) {
      const int expected = reference_bit(&scalar, probabilities[i]);
      const int actual = vp8dx_decode_bool(&native, probabilities[i]);
      if (actual != expected || native.value != scalar.value ||
          native.range != scalar.range || native.count != scalar.count ||
          native.user_buffer != scalar.user_buffer ||
          vp8dx_bool_error(&native) != vp8dx_bool_error(&scalar)) return 0;
    }
    ++result->cases;
    // Time batches, excluding comparison and per-bit timer overhead.
    vp8dx_start_decode(&scalar, data, sizeof(data), NULL, NULL);
    native = scalar;
    int64_t started = esp_timer_get_time();
    for (unsigned i = 0; i < sizeof(probabilities); ++i) reference_bit(&scalar, probabilities[i]);
    result->scalar_us += esp_timer_get_time() - started;
    started = esp_timer_get_time();
    for (unsigned i = 0; i < sizeof(probabilities); ++i) vp8dx_decode_bool(&native, probabilities[i]);
    result->native_us += esp_timer_get_time() - started;
    if (native.value != scalar.value || native.count != scalar.count || native.range != scalar.range) return 0;
  }
  return 1;
}
