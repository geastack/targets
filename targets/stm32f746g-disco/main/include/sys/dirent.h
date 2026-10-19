#pragma once

// newlib's <dirent.h> expects the platform to supply <sys/dirent.h>. The board
// has no filesystem yet, so the types exist for portable code to compile and
// opendir() fails with ENOSYS (newlib_syscalls.cpp).

#include <sys/types.h>

#define DT_UNKNOWN 0
#define DT_DIR 4
#define DT_REG 8

struct dirent {
	ino_t d_ino;
	unsigned char d_type;
	char d_name[256];
};

typedef struct gea_stm32_dir DIR;
