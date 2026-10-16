#pragma once

// Force-included into every C++ file. The bare-metal libstdc++ is built with
// the "single" thread model, so <mutex> and <condition_variable> declare no
// std::mutex or std::condition_variable. gea's runtime locks only around
// SharedArrayBuffer and Atomics; with one thread there is nothing to exclude
// and nobody to wake a waiter, so these stand-ins lock nothing and a wait
// returns as soon as it checks its predicate.

#if defined(__cplusplus)
#include <chrono>
#include <condition_variable>
#include <mutex>

#if !defined(_GLIBCXX_HAS_GTHREADS)
namespace std {

class mutex {
public:
	constexpr mutex() noexcept = default;
	mutex(const mutex &) = delete;
	mutex &operator=(const mutex &) = delete;

	void lock() {}
	bool try_lock() { return true; }
	void unlock() {}
};

class condition_variable {
public:
	void notify_one() noexcept {}
	void notify_all() noexcept {}

	template <typename Lock, typename Predicate>
	void wait(Lock &, Predicate predicate) {
		(void)predicate();
	}

	template <typename Lock, typename Rep, typename Period, typename Predicate>
	bool wait_for(Lock &, const chrono::duration<Rep, Period> &, Predicate predicate) {
		return predicate();
	}
};

}  // namespace std
#endif
#endif
