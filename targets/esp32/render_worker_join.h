#pragma once

#include <atomic>
#include <cstdint>
#include <cstdlib>

namespace gea::platform::esp32::display::detail {

// A submitted raster job borrows its caller's stack and framebuffer. A slow
// worker cannot be abandoned: returning would release both while it still draws.
template <typename Park, typename Clock, typename Report>
void joinRenderWorker(const std::atomic<bool> &done, Park park, Clock now, Report report) {
  const std::int64_t started = now();
  bool reportedSlow = false;
  while (!done.load(std::memory_order_acquire)) {
    park();
    const std::int64_t elapsed = now() - started;
    if (done.load(std::memory_order_acquire))
      break;
    if (elapsed >= 5000000) {
      report(elapsed, true);
      // A genuinely stuck worker must stop the app, never return a live job.
      std::abort();
    }
    if (!reportedSlow && elapsed >= 100000) {
      report(elapsed, false);
      reportedSlow = true;
    }
  }
}

} // namespace gea::platform::esp32::display::detail
