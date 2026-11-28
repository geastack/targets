#pragma once

// The arm-none-eabi libstdc++ is built without gthreads, so <mutex> provides
// lock_guard/unique_lock/once_flag but no std::mutex or std::recursive_mutex.
// The runtime uses both, and core1 runs the render worker, so they must hold
// across cores: gea_std_mutex.cpp implements them over an RP2350 hardware
// spinlock (valid for objects in SRAM or PSRAM, no exclusive monitor needed).
// Force-included into every RP2350 C++ translation unit.

#ifdef __cplusplus
#include <mutex>

#if !defined(_GLIBCXX_HAS_GTHREADS)
namespace std {

class mutex {
public:
	constexpr mutex() noexcept = default;
	mutex(const mutex &) = delete;
	mutex &operator=(const mutex &) = delete;
	void lock() noexcept;
	bool try_lock() noexcept;
	void unlock() noexcept;

private:
	volatile bool locked_ = false;
};

class recursive_mutex {
public:
	constexpr recursive_mutex() noexcept = default;
	recursive_mutex(const recursive_mutex &) = delete;
	recursive_mutex &operator=(const recursive_mutex &) = delete;
	void lock() noexcept;
	bool try_lock() noexcept;
	void unlock() noexcept;

private:
	volatile unsigned owner_ = 0;  // core number + 1; 0 = unowned
	volatile unsigned depth_ = 0;
};

}  // namespace std
#endif
#endif
