// SPDX-License-Identifier: Apache-2.0
#pragma once

#include "services/comparison_benchmark.h"

#if GEA_EMBEDDED_COMPARISON_BENCHMARK
#include <cstring>

namespace gea::platform::comparison::upload {

struct Snapshot {
  std::uint16_t* pixels = nullptr;
  int width = 0, height = 0;
  std::uint64_t submittedPixels = 0;
  unsigned covered = 0, errors = 0;
};

inline std::mutex captureMutex;
inline std::atomic<bool> armed{false};
inline Snapshot state;
inline std::uint8_t* coverage = nullptr;
inline int x0 = 0, y0 = 0, x1 = -1, y1 = -1;
inline std::size_t cursor = 0;

inline bool begin(std::uint16_t* storage, int width, int height) {
  const std::lock_guard<std::mutex> guard(captureMutex);
  if (!storage || width <= 0 || height <= 0 || state.pixels ||
      enabled.load(std::memory_order_acquire)) {
    return false;
  }
  state = {storage, width, height, 0, 0, 0};
  coverage = reinterpret_cast<std::uint8_t*>(storage + std::size_t(width) * height);
  std::memset(coverage, 0, (std::size_t(width) * height + 7) / 8);
  x0 = y0 = 0;
  x1 = y1 = -1;
  cursor = 0;
  armed.store(true, std::memory_order_release);
  return true;
}

inline void window(int left, int top, int right, int bottom) {
  if (!armed.load(std::memory_order_acquire)) {
    return;
  }
  const std::lock_guard<std::mutex> guard(captureMutex);
  if (!armed.load(std::memory_order_relaxed)) {
    return;
  }
  x0 = left;
  y0 = top;
  x1 = right;
  y1 = bottom;
  cursor = 0;
  if (left < 0 || top < 0 || right >= state.width || bottom >= state.height || right < left ||
      bottom < top) {
    ++state.errors;
    x0 = y0 = 0;
    x1 = y1 = -1;
  }
}

inline void restart() {
  if (!armed.load(std::memory_order_acquire)) {
    return;
  }
  const std::lock_guard<std::mutex> guard(captureMutex);
  cursor = 0;
}

// Input bytes are exactly the submitted CO5300 big-endian RGB565 wire payload.
// Keep canonical RGB565 words for the existing little-endian RLE exporter.
inline void submitted(const void* buffer, std::size_t bytes) {
  if (!armed.load(std::memory_order_acquire)) {
    return;
  }
  const std::lock_guard<std::mutex> guard(captureMutex);
  if (!armed.load(std::memory_order_relaxed)) {
    return;
  }
  if (!buffer || (bytes & 1) || x1 < x0 || y1 < y0) {
    ++state.errors;
    return;
  }
  const std::size_t width = x1 - x0 + 1;
  const std::size_t count = width * (y1 - y0 + 1);
  if (bytes / 2 > count - std::min(cursor, count)) {
    ++state.errors;
    return;
  }
  const auto* source = static_cast<const std::uint8_t*>(buffer);
  for (std::size_t index = 0; index < bytes / 2; ++index, ++cursor) {
    const std::size_t pixel = (y0 + cursor / width) * state.width + x0 + cursor % width;
    state.pixels[pixel] = (std::uint16_t(source[index * 2]) << 8) | source[index * 2 + 1];
    const auto mask = std::uint8_t(1u << (pixel & 7));
    if (!(coverage[pixel / 8] & mask)) {
      coverage[pixel / 8] |= mask;
      ++state.covered;
    }
  }
  state.submittedPixels += bytes / 2;
}

inline void failed() {
  if (!armed.load(std::memory_order_acquire)) {
    return;
  }
  const std::lock_guard<std::mutex> guard(captureMutex);
  if (armed.load(std::memory_order_relaxed)) {
    ++state.errors;
  }
}

// Caller first blocks app rendering and waits for DMA completion. Detaching
// under this lock stops future submissions touching the immutable export.
inline Snapshot end() {
  const std::lock_guard<std::mutex> guard(captureMutex);
  armed.store(false, std::memory_order_release);
  const auto result = state;
  state = {};
  coverage = nullptr;
  return result;
}

} // namespace gea::platform::comparison::upload
#endif
