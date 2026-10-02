// SPDX-License-Identifier: Apache-2.0
#pragma once
#include <stdlib.h>
#include <string.h>

// Force-included only for libjuice translation units. Do not change allocation
// policy for lwIP, mbedTLS, the application, or the existing audio peer.
void* gea_juice_malloc(size_t size);
void* gea_juice_calloc(size_t count, size_t size);
void* gea_juice_realloc(void* pointer, size_t size);
void gea_juice_free(void* pointer);

#define malloc(size) gea_juice_malloc(size)
#define calloc(count, size) gea_juice_calloc(count, size)
#define realloc(pointer, size) gea_juice_realloc(pointer, size)
#define free(pointer) gea_juice_free(pointer)
