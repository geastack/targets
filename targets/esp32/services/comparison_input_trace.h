// SPDX-License-Identifier: Apache-2.0
#pragma once

#include "services/comparison_benchmark.h"

#if GEA_EMBEDDED_COMPARISON_BENCHMARK
namespace gea::platform::comparison::input {

struct Entry {
  std::int64_t timestampUs;
  int phase, touching, x, y, pointerId, handlerX, handlerY;
};

struct Snapshot {
  Entry* entries = nullptr;
  unsigned count = 0, dropped = 0;
};

inline std::mutex traceMutex;
inline std::atomic<bool> recording{false};
inline Entry* entries = nullptr;
inline unsigned capacity = 0, count = 0, dropped = 0;

// The caller owns storage. END detaches it while locked, so it can be printed
// and freed without blocking input dispatch or racing the next consumed event.
inline bool begin(Entry* storage, unsigned size) {
  const std::lock_guard<std::mutex> guard(traceMutex);
  if (!storage || !size || entries || enabled.load(std::memory_order_acquire)) {
    return false;
  }
  entries = storage;
  capacity = size;
  count = dropped = 0;
  recording.store(true, std::memory_order_release);
  return true;
}

inline void consumed(std::int64_t timestampUs, int phase, bool touching, int x, int y,
                     int pointerId, int handlerX, int handlerY) {
  if (!recording.load(std::memory_order_acquire) || enabled.load(std::memory_order_acquire)) {
    return;
  }
  const std::lock_guard<std::mutex> guard(traceMutex);
  if (!recording.load(std::memory_order_relaxed) || enabled.load(std::memory_order_relaxed)) {
    return;
  }
  if (count < capacity) {
    entries[count++] = {timestampUs, phase, touching ? 1 : 0, x, y, pointerId, handlerX, handlerY};
  } else {
    ++dropped;
  }
}

inline Snapshot end() {
  const std::lock_guard<std::mutex> guard(traceMutex);
  recording.store(false, std::memory_order_release);
  const Snapshot result{entries, count, dropped};
  entries = nullptr;
  capacity = count = dropped = 0;
  return result;
}

} // namespace gea::platform::comparison::input
#endif
