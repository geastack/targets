#include "../render_worker_join.h"

#include <cassert>
#include <condition_variable>
#include <mutex>
#include <thread>

int main() {
  using gea::platform::esp32::display::detail::joinRenderWorker;
  // Hold a real worker past the former 100ms deadline. It still owns borrowed
  // frame data; join must remain parked until the worker writes it and releases it.
  std::atomic<bool> done{false}, joined{false};
  std::atomic<std::int64_t> clock{0};
  std::mutex mutex;
  std::condition_variable condition;
  bool reported = false, release = false;
  int borrowedFrame = 0, reports = 0;
  std::thread worker([&] {
    std::unique_lock<std::mutex> lock(mutex);
    condition.wait(lock, [&] { return release; });
    borrowedFrame = 42;
    done.store(true, std::memory_order_release);
    condition.notify_all();
  });
  std::thread caller([&] {
    joinRenderWorker(done,
      [&] {
        clock.fetch_add(25000);
        std::unique_lock<std::mutex> lock(mutex);
        if (reported) condition.wait(lock, [&] { return done.load(); });
      },
      [&] { return clock.load(); },
      [&](std::int64_t elapsed, bool stuck) {
        assert(elapsed >= 100000 && !stuck);
        std::lock_guard<std::mutex> lock(mutex);
        ++reports;
        reported = true;
        condition.notify_all();
      });
    assert(borrowedFrame == 42);
    joined.store(true);
  });
  {
    std::unique_lock<std::mutex> lock(mutex);
    condition.wait(lock, [&] { return reported; });
    assert(!joined.load());
    release = true;
    condition.notify_all();
  }
  caller.join();
  worker.join();
  assert(joined && reports == 1);
  // An already-completed job never parks or reports a delay.
  joinRenderWorker(done, [] { assert(false); }, [] { return 0; },
                   [](std::int64_t, bool) { assert(false); });
}
