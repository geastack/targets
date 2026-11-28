#include "gea_std_mutex.h"

#if !defined(_GLIBCXX_HAS_GTHREADS)
#include "hardware/sync.h"
#include "pico/platform.h"

namespace {

// PICO_SPINLOCK_ID_OS1 is reserved for an OS; this firmware runs none.
spin_lock_t *lockWord() { return spin_lock_instance(PICO_SPINLOCK_ID_OS1); }

}  // namespace

namespace std {

bool mutex::try_lock() noexcept
{
	const uint32_t saved = spin_lock_blocking(lockWord());
	const bool acquired = !locked_;
	if (acquired) locked_ = true;
	spin_unlock(lockWord(), saved);
	return acquired;
}

void mutex::lock() noexcept
{
	while (!try_lock()) tight_loop_contents();
}

void mutex::unlock() noexcept
{
	const uint32_t saved = spin_lock_blocking(lockWord());
	locked_ = false;
	spin_unlock(lockWord(), saved);
}

bool recursive_mutex::try_lock() noexcept
{
	const unsigned self = get_core_num() + 1;
	const uint32_t saved = spin_lock_blocking(lockWord());
	bool acquired = false;
	if (owner_ == 0 || owner_ == self) {
		owner_ = self;
		depth_ = depth_ + 1;
		acquired = true;
	}
	spin_unlock(lockWord(), saved);
	return acquired;
}

void recursive_mutex::lock() noexcept
{
	while (!try_lock()) tight_loop_contents();
}

void recursive_mutex::unlock() noexcept
{
	const uint32_t saved = spin_lock_blocking(lockWord());
	if (depth_ > 0 && --depth_ == 0) owner_ = 0;
	spin_unlock(lockWord(), saved);
}

}  // namespace std
#endif
