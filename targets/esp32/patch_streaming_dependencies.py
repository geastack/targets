"""Generate narrowly patched SDK sources in the normal build output.

Registry sources stay untouched. Reject source drift instead of guessing where
allocation/lifetime fixes belong. Espressif WebSocket 1.8 / codec-dev 1.6.2,
ESP-IDF 6.0.2 WebSocket transport.
"""
from pathlib import Path
import sys


def replace_once(source, old, new):
    if source.count(old) != 1:
        raise ValueError(f"SDK source drift at {old[:80]!r}")
    return source.replace(old, new, 1)


def patch_websocket(source):
    source = replace_once(source, '#include "freertos/task.h"',
        '#include "freertos/task.h"\n#include "freertos/idf_additions.h"\n#include "esp_heap_caps.h"')
    source = replace_once(source,
        '    destroy_and_free_resources(client);\n    return ESP_OK;\n}',
        '''    // STOPPED_BIT is published after the last client access by its task.
    // Reap the suspended task here; never free a running external stack.
    if (client->task_handle) {
        while (eTaskGetState(client->task_handle) != eSuspended) {
            vTaskDelay(1);
        }
        vTaskDeleteWithCaps(client->task_handle);
        client->task_handle = NULL;
    }
    destroy_and_free_resources(client);
    return ESP_OK;
}''')
    source = replace_once(source,
        '''    if (client->selected_for_destroying == true) {
        destroy_and_free_resources(client);
    } else {
        xEventGroupSetBits(client->status_bits, STOPPED_BIT);
    }
    vTaskDelete(NULL);''',
        '''    if (client->selected_for_destroying == true) {
        destroy_and_free_resources(client);
        vTaskDeleteWithCaps(NULL);
    } else {
        xEventGroupSetBits(client->status_bits, STOPPED_BIT);
        vTaskSuspend(NULL); // destroy/start joins and frees this PSRAM stack.
    }''')
    source = replace_once(source, '    client->transport = client->config->ext_transport;',
        '''    if (client->task_handle) {
        while (eTaskGetState(client->task_handle) != eSuspended) {
            vTaskDelay(1);
        }
        vTaskDeleteWithCaps(client->task_handle);
        client->task_handle = NULL;
    }
    client->transport = client->config->ext_transport;''')
    source = replace_once(source, '    if (xTaskCreatePinnedToCore(esp_websocket_client_task,',
        '''    const UBaseType_t stack_caps = MALLOC_CAP_8BIT |
        (heap_caps_get_total_size(MALLOC_CAP_SPIRAM) ? MALLOC_CAP_SPIRAM : MALLOC_CAP_INTERNAL);
    if (xTaskCreatePinnedToCoreWithCaps(esp_websocket_client_task,''')
    source = replace_once(source, '&client->task_handle, client->config->task_core_id) != pdTRUE)',
        '&client->task_handle, client->config->task_core_id, stack_caps) != pdTRUE)')
    source = replace_once(source,
        '''                    // Now acquire tx_lock with timeout (consistent with PONG handling)
                    if (xSemaphoreTakeRecursive(client->tx_lock, WEBSOCKET_TX_LOCK_TIMEOUT_MS) != pdPASS) {
                        ESP_LOGE(TAG, "Could not lock ws-client within %d timeout for PING", WEBSOCKET_TX_LOCK_TIMEOUT_MS);''',
        '''                    // Optional heartbeats must not suspend incoming media behind
                    // a busy sender. The next ping interval retries; PONG/CLOSE
                    // retain their required handshake behavior.
                    if (xSemaphoreTakeRecursive(client->tx_lock, 0) != pdPASS) {
                        ESP_LOGD(TAG, "Skipping PING while transmitter is busy");''')
    return source


def patch_ws_transport(source):
    source = replace_once(source, '#include "esp_tls_crypto.h"',
        '#include "esp_tls_crypto.h"\n#include "esp_timer.h"\n#include "esp_heap_caps.h"')
    source = replace_once(source, '#define MAX_WEBSOCKET_HEADER_SIZE   16',
        '#define MAX_WEBSOCKET_HEADER_SIZE   16\n#define GEA_WS_TX_BUFFER_SIZE        4096')
    source = replace_once(source, '    size_t response_header_len;\n} transport_ws_t;',
        '''    size_t response_header_len;
    char *gea_tx_buffer;      /*!< Separate from handshake/RX storage; writers are serialized by the client. */
} transport_ws_t;''')
    source = replace_once(source, '    free(ws->redir_host);\n    free(ws->path);',
        '    free(ws->gea_tx_buffer);\n    free(ws->redir_host);\n    free(ws->path);')
    source = replace_once(source,
        '''    char *buffer = (char *)b;
    char ws_header[MAX_WEBSOCKET_HEADER_SIZE];
    char *mask;
    int header_len = 0, i;

    int poll_write;
    if ((poll_write = esp_transport_poll_write(ws->parent, timeout_ms)) <= 0) {
        ESP_LOGE(TAG, "Error transport_poll_write(%d)", poll_write);
        return poll_write;
    }''',
        '''    char ws_header[MAX_WEBSOCKET_HEADER_SIZE];
    int header_len = 0;
    if (len < 0 || (len && !b)) {
        return -1;
    }
    const int64_t deadline_us = timeout_ms < 0 ? 0 :
        esp_timer_get_time() + (int64_t)timeout_ms * 1000;
    if (!ws->gea_tx_buffer) {
        // Keep the bounded staging buffer out of scarce DMA/internal RAM
        // when PSRAM is available. Non-PSRAM boards retain malloc fallback.
        ws->gea_tx_buffer = heap_caps_malloc(GEA_WS_TX_BUFFER_SIZE, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
        if (!ws->gea_tx_buffer) {
            ws->gea_tx_buffer = malloc(GEA_WS_TX_BUFFER_SIZE);
        }
        if (!ws->gea_tx_buffer) {
            ESP_LOGE(TAG, "Cannot allocate WebSocket transmit buffer");
            return -1;
        }
    }''')
    source = replace_once(source,
        '''    if (mask_flag) {
        mask = &ws_header[header_len];
        ssize_t rc;
        if ((rc = getrandom(ws_header + header_len, 4, 0)) < 0) {
            ESP_LOGD(TAG, "getrandom() returned %zd", rc);
            return -1;
        }
        header_len += 4;

        for (i = 0; i < len; ++i) {
            buffer[i] = (buffer[i] ^ mask[i % 4]);
        }
    }

    if (esp_transport_write(ws->parent, ws_header, header_len, timeout_ms) != header_len) {
        ESP_LOGE(TAG, "Error write header");
        return -1;
    }
    if (len == 0) {
        return 0;
    }

    int ret = esp_transport_write(ws->parent, buffer, len, timeout_ms);
    // in case of masked transport we have to revert back to the original data, as ws layer
    // does not create its own copy of data to be sent
    if (mask_flag) {
        mask = &ws_header[header_len - 4];
        for (i = 0; i < len; ++i) {
            buffer[i] = (buffer[i] ^ mask[i % 4]);
        }
    }
    return ret;''',
        '''    if (mask_flag) {
        const ssize_t rc = getrandom(ws_header + header_len, 4, 0);
        if (rc != 4) {
            ESP_LOGD(TAG, "getrandom() returned %zd", rc);
            return -1;
        }
        header_len += 4;
    }

    // A standalone WS header becomes a tiny TLS record, followed by another
    // socket-writability wait before its payload. Coalesce both; common audio
    // frames fit in one TLS write. Larger frames keep the same WS framing and
    // masking position while streaming through this bounded staging buffer.
    memcpy(ws->gea_tx_buffer, ws_header, header_len);
    int prefix = header_len;
    int offset = 0;
    do {
        const int capacity = GEA_WS_TX_BUFFER_SIZE - prefix;
        const int count = len - offset < capacity ? len - offset : capacity;
        for (int i = 0; i < count; ++i) {
            const char mask = mask_flag ? ws_header[header_len - 4 + ((offset + i) & 3)] : 0;
            ws->gea_tx_buffer[prefix + i] = b[offset + i] ^ mask;
        }
        const int chunk_len = prefix + count;
        int written = 0;
        while (written < chunk_len) {
            int remaining_ms = timeout_ms;
            if (timeout_ms >= 0) {
                const int64_t left_us = deadline_us - esp_timer_get_time();
                remaining_ms = left_us > 0 ? (int)((left_us + 999) / 1000) : 0;
            }
            const int ret = esp_transport_write(ws->parent, ws->gea_tx_buffer + written,
                                                chunk_len - written, remaining_ms);
            if (ret <= 0 || ret > chunk_len - written) {
                // Once any frame bytes went out, retrying as a fresh WS frame
                // would corrupt the stream. Let the client abort the connection.
                ESP_LOGE(TAG, "Error write WebSocket frame (%d)", ret);
                return -1;
            }
            written += ret;
        }
        offset += count;
        prefix = 0;
    } while (offset < len);
    return len;''')
    return source


def patch_codec(source):
    return replace_once(source,
        '''        ret = set_drv_fs(channel, true, bits_per_sample, i2s_data->clk_src, fs);
        _i2s_drv_enable(paired, true, true);''',
        '''        ret = set_drv_fs(channel, true, bits_per_sample, i2s_data->clk_src, fs);
        // Reconfiguration may fail after releasing DMA. Enabling that channel
        // dereferences a null DMA descriptor in ESP-IDF; propagate the failure.
        if (ret != ESP_CODEC_DEV_OK) {
            return ret;
        }
        ret = _i2s_drv_enable(paired, true, true);''')


def patch_srtp_aes(source):
    # IDF 6 removed mbedtls/aes.h, but exposes the same CTR contract through
    # its native, internally locked AES peripheral driver. Keep libsrtp's
    # counter/salt construction and error handling unchanged.
    source = replace_once(source, '#include <mbedtls/aes.h>', '#include "aes/esp_aes.h"')
    source = replace_once(source, '#include "aes_icm_ext.h"', '#include "gea_srtp_aes_icm.h"')
    source = source.replace('mbedtls_aes_setkey_enc', 'esp_aes_setkey')
    return source.replace('mbedtls_aes_', 'esp_aes_')


def patch_srtp_aes_header(source):
    source = replace_once(source, '#include <mbedtls/aes.h>', '#include "aes/esp_aes.h"')
    return replace_once(source, 'mbedtls_aes_context *ctx;', 'esp_aes_context *ctx;')


if __name__ == '__main__':
    kind, source_file, destination_file = sys.argv[1:]
    source = Path(source_file).read_text()
    patched = {'websocket': patch_websocket, 'ws_transport': patch_ws_transport,
               'codec': patch_codec, 'srtp_aes': patch_srtp_aes,
               'srtp_aes_header': patch_srtp_aes_header}[kind](source)
    destination = Path(destination_file)
    if not destination.exists() or destination.read_text() != patched:
        destination.write_text(patched)
