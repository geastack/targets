// SPDX-License-Identifier: Apache-2.0
#include "esp_heap_caps.h"
#include <stddef.h>

void* gea_juice_malloc(size_t size) {
    return heap_caps_malloc(size, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
}

void* gea_juice_calloc(size_t count, size_t size) {
    return heap_caps_calloc(count, size, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
}

void* gea_juice_realloc(void* pointer, size_t size) {
    return heap_caps_realloc(pointer, size, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
}

void gea_juice_free(void* pointer) {
    heap_caps_free(pointer);
}
