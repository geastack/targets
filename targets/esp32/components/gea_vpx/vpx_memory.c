// SPDX-License-Identifier: Apache-2.0
// libvpx's allocation API, using external memory for codec state and reference
// images. Keep internal RAM available for I2S DMA, Wi-Fi and the DTLS stack.
#include <stddef.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>

#ifdef ESP_PLATFORM
#include "esp_heap_caps.h"
#endif

void *vpx_memalign(size_t alignment, size_t size) {
    if (!alignment || (alignment & (alignment - 1)) || size > SIZE_MAX - alignment) return NULL;
    if (alignment < sizeof(void *)) alignment = sizeof(void *);
#ifdef ESP_PLATFORM
    return heap_caps_aligned_alloc(alignment, size, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
#else
    // The standalone build also exercises the same allocation contract on a
    // desktop or cross-compiles the decoder without needing the whole app.
    void *memory = NULL;
    return posix_memalign(&memory, alignment, size) == 0 ? memory : NULL;
#endif
}

void *vpx_malloc(size_t size) { return vpx_memalign(32, size); }

void *vpx_calloc(size_t count, size_t size) {
    if (count && size > SIZE_MAX / count) return NULL;
    void *memory = vpx_malloc(count * size);
    if (memory) memset(memory, 0, count * size);
    return memory;
}

void vpx_free(void *memory) {
#ifdef ESP_PLATFORM
    heap_caps_free(memory);
#else
    free(memory);
#endif
}
