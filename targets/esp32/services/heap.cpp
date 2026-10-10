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
#include "freertos/FreeRTOS.h"
#include <cstdio>

#include <cstddef>
#include <cstdint>
#include <cstdlib>
#include <new>

namespace {

#if defined(GEA_HEAP_TRACE)
// Diagnosis only: live bytes of C++ heap traffic by allocation size (log2
// buckets), printed by the runtime's cycle trace (gea_heap_trace_print), so
// a board whose heap drains names the size class that grows.
std::uint32_t g_traceBucket[24];
std::uint32_t g_traceLive = 0;

// Live bytes per call site (allocations >= 512 B): ptr -> (site, size) in an
// open-addressed table, site -> live bytes in a second one.
struct SiteSlot { std::uintptr_t key; std::uintptr_t site; std::uint32_t size; };
constexpr unsigned kPtrSlots = 16384, kSiteSlots = 1024;
SiteSlot *g_ptrTable = nullptr;
std::uintptr_t g_siteKey[kSiteSlots];
std::int32_t g_siteLive[kSiteSlots];
portMUX_TYPE g_traceLock = portMUX_INITIALIZER_UNLOCKED;
thread_local std::uintptr_t t_site = 0;

void traceSite(void *ptr, std::uint32_t size, bool add)
{
  if (!g_ptrTable) {
    if (!add) return;
    g_ptrTable = static_cast<SiteSlot *>(heap_caps_calloc(kPtrSlots, sizeof(SiteSlot), MALLOC_CAP_SPIRAM));
    if (!g_ptrTable) return;
  }
  const std::uintptr_t key = reinterpret_cast<std::uintptr_t>(ptr);
  portENTER_CRITICAL(&g_traceLock);
  unsigned h = static_cast<unsigned>((key >> 3) * 2654435761u) & (kPtrSlots - 1);
  if (add) {
    if (size >= 512 && t_site) {
      for (unsigned i = 0; i < kPtrSlots; i++, h = (h + 1) & (kPtrSlots - 1)) {
        if (g_ptrTable[h].key == 0 || g_ptrTable[h].key == 1) {
          g_ptrTable[h] = {key, t_site, size};
          unsigned s = static_cast<unsigned>((t_site >> 1) * 2654435761u) & (kSiteSlots - 1);
          for (unsigned j = 0; j < kSiteSlots; j++, s = (s + 1) & (kSiteSlots - 1)) {
            if (g_siteKey[s] == t_site || g_siteKey[s] == 0) { g_siteKey[s] = t_site; g_siteLive[s] += size; break; }
          }
          break;
        }
      }
    }
  } else {
    for (unsigned i = 0; i < kPtrSlots; i++, h = (h + 1) & (kPtrSlots - 1)) {
      if (g_ptrTable[h].key == 0) break;
      if (g_ptrTable[h].key == key) {
        const std::uintptr_t site = g_ptrTable[h].site;
        unsigned s = static_cast<unsigned>((site >> 1) * 2654435761u) & (kSiteSlots - 1);
        for (unsigned j = 0; j < kSiteSlots; j++, s = (s + 1) & (kSiteSlots - 1)) {
          if (g_siteKey[s] == site) { g_siteLive[s] -= g_ptrTable[h].size; break; }
          if (g_siteKey[s] == 0) break;
        }
        g_ptrTable[h].key = 1;  // tombstone
        break;
      }
    }
  }
  portEXIT_CRITICAL(&g_traceLock);
}

void traceHeap(void *ptr, bool add)
{
  if (!ptr) return;
  const std::uint32_t size = static_cast<std::uint32_t>(heap_caps_get_allocated_size(ptr));
  traceSite(ptr, size, add);
  t_site = 0;
  unsigned bucket = 0;
  while (bucket < 23 && (std::uint32_t{16} << bucket) < size) bucket++;
  if (add) {
    g_traceBucket[bucket] += size;
    g_traceLive += size;
  } else {
    g_traceBucket[bucket] -= g_traceBucket[bucket] < size ? g_traceBucket[bucket] : size;
    g_traceLive -= g_traceLive < size ? g_traceLive : size;
    return;
  }
}
#else
inline void traceHeap(void *, bool) {}
#endif

class EspHeapAllocator {
public:
  static void *allocatePreferSpiram(std::size_t size, std::size_t alignment = alignof(std::max_align_t))
  {
    if (size == 0) size = 1;

    void *ptr = allocateWithCaps(size, alignment, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    if (!ptr) ptr = allocateWithCaps(size, alignment, MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
    if (!ptr) ptr = allocateWithCaps(size, alignment, MALLOC_CAP_8BIT);
    traceHeap(ptr, true);
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

#if defined(GEA_HEAP_TRACE)
extern "C" void gea_heap_trace_print()
{
  // Top call sites by live bytes (and their change since the last print).
  static std::int32_t lastLive[kSiteSlots];
  unsigned top[12] = {};
  int n = 0;
  for (unsigned i = 0; i < kSiteSlots; i++) {
    if (!g_siteKey[i] || g_siteLive[i] < 8 * 1024) continue;
    int at = n < 12 ? n++ : 11;
    if (at == 11 && n == 12 && g_siteLive[top[11]] >= g_siteLive[i]) continue;
    top[at] = i;
    while (at > 0 && g_siteLive[top[at - 1]] < g_siteLive[top[at]]) { unsigned t = top[at]; top[at] = top[at - 1]; top[at - 1] = t; at--; }
  }
  std::printf("[heaps]");
  for (int k = 0; k < n; k++) {
    const unsigned i = top[k];
    std::printf(" %08x:%dK(%+d)", static_cast<unsigned>(g_siteKey[i]), static_cast<int>(g_siteLive[i] >> 10), static_cast<int>((g_siteLive[i] - lastLive[i]) >> 10));
  }
  for (unsigned i = 0; i < kSiteSlots; i++) lastLive[i] = g_siteLive[i];
  std::printf("\n");
  std::printf("[heapt] live=%uKB psram_free=%uKB", static_cast<unsigned>(g_traceLive >> 10),
              static_cast<unsigned>(heap_caps_get_free_size(MALLOC_CAP_SPIRAM) >> 10));
  for (unsigned i = 0; i < 24; i++)
    if (g_traceBucket[i] >= 16 * 1024) std::printf(" %u:%uK", static_cast<unsigned>(16u << i), static_cast<unsigned>(g_traceBucket[i] >> 10));
  std::printf("\n");
}
#endif

void *gea::framework::memory::Allocator::allocatePreferSpiram(std::size_t size, std::size_t alignment) {
#if defined(GEA_HEAP_TRACE)
  t_site = reinterpret_cast<std::uintptr_t>(__builtin_return_address(0));
#endif
  return EspHeapAllocator::allocatePreferSpiram(size, alignment);
}

void *gea::framework::memory::Allocator::reallocatePreferSpiram(void *ptr, std::size_t size) {
  if (size == 0) {
    free(ptr);
    return nullptr;
  }

  traceHeap(ptr, false);
#if defined(GEA_HEAP_TRACE)
  t_site = reinterpret_cast<std::uintptr_t>(__builtin_return_address(0));
#endif
  void *next = heap_caps_realloc(ptr, size, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
  if (!next) next = heap_caps_realloc(ptr, size, MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
  if (!next) next = heap_caps_realloc(ptr, size, MALLOC_CAP_8BIT);
  traceHeap(next ? next : ptr, true);
  return next;
}

void gea::framework::memory::Allocator::free(void *ptr) noexcept {
  traceHeap(ptr, false);
  if (ptr) heap_caps_free(ptr);
}

#if defined(GEA_HEAP_TRACE)
void *operator new(std::size_t size) { t_site = reinterpret_cast<std::uintptr_t>(__builtin_return_address(0)); return EspHeapAllocator::allocateOrAbort(size); }
void *operator new[](std::size_t size) { t_site = reinterpret_cast<std::uintptr_t>(__builtin_return_address(0)); return EspHeapAllocator::allocateOrAbort(size); }
#else
void *operator new(std::size_t size) { return EspHeapAllocator::allocateOrAbort(size); }
void *operator new[](std::size_t size) { return EspHeapAllocator::allocateOrAbort(size); }
#endif

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
