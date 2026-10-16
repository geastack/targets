// newlib's system layer for a bare-metal board: malloc grows through the
// SDRAM heap the linker script leaves after the framebuffers, stdout/stderr
// go to the serial console, wall time comes from the RTC, and there is no
// filesystem.

#include "console.h"
#include "rtc_clock.h"
#include "stm32f7xx.h"

#include <cerrno>
#include <cstddef>
#include <dirent.h>
#include <sys/stat.h>
#include <sys/time.h>

using namespace gea::stm32;

extern "C" {

extern char __heap_start[];
extern char __heap_end[];

void *_sbrk(std::ptrdiff_t increment) {
	static char *brk = __heap_start;
	if (brk + increment > __heap_end || brk + increment < __heap_start) {
		errno = ENOMEM;
		return reinterpret_cast<void *>(-1);
	}

	char *previous = brk;
	brk += increment;
	return previous;
}

int _write(int fd, const char *data, int length) {
	if (fd != 1 && fd != 2) {
		errno = EBADF;
		return -1;
	}

	console::write(data, static_cast<std::size_t>(length));
	return length;
}

int _read(int, char *, int) {
	return 0;
}

int _close(int) {
	return -1;
}

int _fstat(int, struct stat *st) {
	st->st_mode = S_IFCHR;
	return 0;
}

int _isatty(int fd) {
	return fd <= 2;
}

int _lseek(int, int, int) {
	return 0;
}

int _getpid() {
	return 1;
}

int _kill(int, int) {
	errno = EINVAL;
	return -1;
}

int _gettimeofday(struct timeval *tv, void *) {
	const std::int64_t now = rtc_clock::epochMicros();
	tv->tv_sec = static_cast<time_t>(now / 1'000'000);
	tv->tv_usec = static_cast<suseconds_t>(now % 1'000'000);
	return 0;
}

int mkdir(const char *, mode_t) {
	errno = ENOSYS;
	return -1;
}

DIR *opendir(const char *) {
	errno = ENOSYS;
	return nullptr;
}

struct dirent *readdir(DIR *) {
	return nullptr;
}

int closedir(DIR *) {
	errno = EBADF;
	return -1;
}

[[noreturn]] void _exit(int) {
	NVIC_SystemReset();
}

}  // extern "C"
