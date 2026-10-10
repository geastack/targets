// SPDX-License-Identifier: Apache-2.0
#pragma once

#ifndef GEA_EMBEDDED_COMPARISON_BENCHMARK
#define GEA_EMBEDDED_COMPARISON_BENCHMARK 0
#endif

#if GEA_EMBEDDED_COMPARISON_BENCHMARK
#include <algorithm>
#include <array>
#include <atomic>
#include <cstdint>
#include <cstdio>
#include <mutex>

namespace gea::platform::comparison {

struct Distribution {
  static constexpr unsigned kBinUs = 1000;
  std::array<std::uint32_t, 256> bins{};
  std::uint64_t count = 0, sum = 0, maximum = 0, over16667 = 0;

  void add(std::uint64_t us) {
    ++count;
    sum += us;
    maximum = std::max(maximum, us);
    if (us > 16667) {
      ++over16667;
    }
    ++bins[std::min<std::uint64_t>(us / kBinUs, bins.size() - 1)];
  }

  // -1 means no samples or an unbounded overflow bin, never a fabricated bound.
  std::int64_t percentileUpper(unsigned percent) const {
    std::uint64_t cumulative = 0;
    if (!count) {
      return -1;
    }
    for (unsigned bin = 0; bin < bins.size(); ++bin) {
      cumulative += bins[bin];
      if (cumulative * 100 >= count * percent) {
        return bin + 1 == bins.size() ? std::int64_t{-1}
                                      : static_cast<std::int64_t>((bin + 1) * kBinUs);
      }
    }
    return -1;
  }
};

struct Sample {
  char runId[64]{}, scenario[64]{};
  std::int64_t startUs = 0, endUs = 0, previousDoneUs = 0, previousPresentedUs = 0;
  std::uint64_t schedulerFrames = 0, renderedFrames = 0, presentedFrames = 0;
  std::uint64_t presentChunks = 0, presentPixels = 0;
  std::uint64_t completionWaitSum = 0, completionWaitMax = 0;
  std::uint32_t completionFailures = 0;
  bool completionFence = true;
  Distribution work, cadence, presentedCadence;
};

struct Token {
  std::uint32_t generation = 0;
  bool active = false;
};

inline std::mutex mutex;
inline std::atomic<bool> enabled{false};
inline std::atomic<bool> completionFence{true};
// Static captures only; monotonic frame/input clocks remain untouched.
inline std::atomic<std::int64_t> frozenEpochSeconds{0};
inline std::uint32_t generation = 0;
inline Sample sample;

inline void begin(const char* runId, const char* scenario, std::int64_t nowUs) {
  const std::lock_guard<std::mutex> guard(mutex);
  enabled.store(false, std::memory_order_release);
  sample = {};
  std::snprintf(sample.runId, sizeof(sample.runId), "%s", runId);
  std::snprintf(sample.scenario, sizeof(sample.scenario), "%s", scenario);
  sample.startUs = nowUs;
  sample.completionFence = completionFence.load(std::memory_order_relaxed);
  ++generation;
  enabled.store(true, std::memory_order_release);
}

inline Token frameBegin() {
  if (!enabled.load(std::memory_order_acquire)) {
    return {};
  }
  const std::lock_guard<std::mutex> guard(mutex);
  return {generation, enabled.load(std::memory_order_relaxed)};
}

inline void frameDone(Token token, std::int64_t startUs, std::int64_t doneUs,
                      std::uint32_t flushCalls, std::uint64_t pixels, std::uint32_t chunks,
                      std::uint64_t completionWaitUs = 0, bool completionSucceeded = true) {
  if (!token.active || !enabled.load(std::memory_order_acquire)) {
    return;
  }
  const std::lock_guard<std::mutex> guard(mutex);
  if (!enabled.load(std::memory_order_relaxed) || token.generation != generation) {
    return;
  }
  ++sample.schedulerFrames;
  if (flushCalls && pixels) {
    ++sample.renderedFrames;
  }
  sample.completionWaitSum += completionWaitUs;
  sample.completionWaitMax = std::max(sample.completionWaitMax, completionWaitUs);
  if (sample.completionFence && !completionSucceeded) {
    ++sample.completionFailures;
  }
  if (chunks && flushCalls && pixels && sample.completionFence && completionSucceeded) {
    ++sample.presentedFrames;
    if (sample.previousPresentedUs) {
      sample.presentedCadence.add(doneUs - sample.previousPresentedUs);
    }
    sample.previousPresentedUs = doneUs;
  }
  sample.presentChunks += chunks;
  sample.presentPixels += pixels;
  sample.work.add(static_cast<std::uint64_t>(std::max<std::int64_t>(0, doneUs - startUs)));
  if (sample.previousDoneUs) {
    sample.cadence.add(doneUs - sample.previousDoneUs);
  }
  sample.previousDoneUs = doneUs;
}

inline Sample end(std::int64_t nowUs) {
  const std::lock_guard<std::mutex> guard(mutex);
  enabled.store(false, std::memory_order_release);
  sample.endUs = nowUs;
  return sample;
}

} // namespace gea::platform::comparison
#endif
