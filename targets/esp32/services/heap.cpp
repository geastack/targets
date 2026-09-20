// ESP firmware heap policy for C++ runtime allocations.
//
// geatsc-generated apps use std::function, std::shared_ptr, std::string,
// vectors, and record storage during reactive updates. ESP-IDF's default
// global new routes those allocations to normal internal RAM, which is the
// scarcest heap on this target. Prefer PSRAM for C++ heap traffic and fall
// back to internal RAM only when needed.

#include "memory.h"

#include "esp_heap_caps.h"
#include "esp_rom_sys.h"

#include <cstddef>
#include <cstdint>
#include <cstdlib>
#include <new>

namespace {

class EspHeapAllocator {
public:
  static void *allocatePreferSpiram(std::size_t size, std::size_t alignment = alignof(std::max_align_t))
  {
    if (size == 0) size = 1;

    void *ptr = allocateWithCaps(size, alignment, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    if (!ptr) ptr = allocateWithCaps(size, alignment, MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
    if (!ptr) ptr = allocateWithCaps(size, alignment, MALLOC_CAP_8BIT);
    return ptr;
  }

  static void *allocateOrAbort(std::size_t size, std::size_t alignment = alignof(std::max_align_t))
  {
    void *ptr = allocatePreferSpiram(size, alignment);
    if (!ptr) {
      // esp_rom_printf goes through the UART ROM driver — no FreeRTOS log
      // mutex, no queue, no allocations. Safe to call from a failed-new path
      // that would otherwise overflow the gea_init stack via the regular
      // ESP_LOG* machinery.
      esp_rom_printf("[gea_heap] operator new failed: size=%u align=%u "
                     "internal_free=%u internal_largest=%u internal_min=%u "
                     "psram_free=%u psram_largest=%u\n",
                     static_cast<unsigned>(size),
                     static_cast<unsigned>(alignment),
                     static_cast<unsigned>(heap_caps_get_free_size(MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT)),
                     static_cast<unsigned>(heap_caps_get_largest_free_block(MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT)),
                     static_cast<unsigned>(heap_caps_get_minimum_free_size(MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT)),
                     static_cast<unsigned>(heap_caps_get_free_size(MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT)),
                     static_cast<unsigned>(heap_caps_get_largest_free_block(MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT)));
      std::abort();
    }
    return ptr;
  }

private:
  static void *allocateWithCaps(std::size_t size, std::size_t alignment, std::uint32_t caps)
  {
    if (alignment > alignof(std::max_align_t)) {
      return heap_caps_aligned_alloc(alignment, size, caps);
    }
    return heap_caps_malloc(size, caps);
  }
};

}  // namespace

void *gea::framework::memory::Allocator::allocatePreferSpiram(std::size_t size, std::size_t alignment) {
  return EspHeapAllocator::allocatePreferSpiram(size, alignment);
}

void *gea::framework::memory::Allocator::reallocatePreferSpiram(void *ptr, std::size_t size) {
  if (size == 0) {
    free(ptr);
    return nullptr;
  }

  void *next = heap_caps_realloc(ptr, size, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
  if (!next) next = heap_caps_realloc(ptr, size, MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
  if (!next) next = heap_caps_realloc(ptr, size, MALLOC_CAP_8BIT);
  return next;
}

void gea::framework::memory::Allocator::free(void *ptr) noexcept {
  if (ptr) heap_caps_free(ptr);
}

void *operator new(std::size_t size) { return EspHeapAllocator::allocateOrAbort(size); }
void *operator new[](std::size_t size) { return EspHeapAllocator::allocateOrAbort(size); }

void *operator new(std::size_t size, const std::nothrow_t &) noexcept {
  return EspHeapAllocator::allocatePreferSpiram(size);
}

void *operator new[](std::size_t size, const std::nothrow_t &) noexcept {
  return EspHeapAllocator::allocatePreferSpiram(size);
}

void operator delete(void *ptr) noexcept { gea::framework::memory::Allocator::free(ptr); }
void operator delete[](void *ptr) noexcept { gea::framework::memory::Allocator::free(ptr); }
void operator delete(void *ptr, std::size_t) noexcept { gea::framework::memory::Allocator::free(ptr); }
void operator delete[](void *ptr, std::size_t) noexcept { gea::framework::memory::Allocator::free(ptr); }
void operator delete(void *ptr, const std::nothrow_t &) noexcept { gea::framework::memory::Allocator::free(ptr); }
void operator delete[](void *ptr, const std::nothrow_t &) noexcept { gea::framework::memory::Allocator::free(ptr); }

#if defined(__cpp_aligned_new)
void *operator new(std::size_t size, std::align_val_t alignment) {
  return EspHeapAllocator::allocateOrAbort(size, static_cast<std::size_t>(alignment));
}

void *operator new[](std::size_t size, std::align_val_t alignment) {
  return EspHeapAllocator::allocateOrAbort(size, static_cast<std::size_t>(alignment));
}

void *operator new(std::size_t size, std::align_val_t alignment, const std::nothrow_t &) noexcept {
  return EspHeapAllocator::allocatePreferSpiram(size, static_cast<std::size_t>(alignment));
}

void *operator new[](std::size_t size, std::align_val_t alignment, const std::nothrow_t &) noexcept {
  return EspHeapAllocator::allocatePreferSpiram(size, static_cast<std::size_t>(alignment));
}

void operator delete(void *ptr, std::align_val_t) noexcept { gea::framework::memory::Allocator::free(ptr); }
void operator delete[](void *ptr, std::align_val_t) noexcept { gea::framework::memory::Allocator::free(ptr); }
void operator delete(void *ptr, std::size_t, std::align_val_t) noexcept { gea::framework::memory::Allocator::free(ptr); }
void operator delete[](void *ptr, std::size_t, std::align_val_t) noexcept { gea::framework::memory::Allocator::free(ptr); }
void operator delete(void *ptr, std::align_val_t, const std::nothrow_t &) noexcept { gea::framework::memory::Allocator::free(ptr); }
void operator delete[](void *ptr, std::align_val_t, const std::nothrow_t &) noexcept { gea::framework::memory::Allocator::free(ptr); }
#endif
