// SPDX-License-Identifier: GPL-3.0-only
// The boot flash's "apps" FAT partition, served through a proxy VFS.
//
// Writing (and reading) the boot flash disables the cache for the duration,
// and on the ESP32-S3 that cache also fronts PSRAM: an ESP-IDF flash call
// made from a task whose stack lives in PSRAM aborts
// (esp_task_stack_is_sane_cache_disabled). The runtime task's stack is in
// PSRAM on most boards, and so may be an app's. So the FAT volume is mounted
// at a private path and every file operation on /apps is run by one worker
// task with an internal-RAM stack; callers block until it is done. FAT
// serializes its volume anyway, so one worker costs nothing in throughput.
#include "sdkconfig.h"

#if __has_include("esp_vfs_fat.h")

#include <dirent.h>
#include <errno.h>
#include <fcntl.h>
#include <stdbool.h>
#include <stdio.h>
#include <string.h>
#include <sys/stat.h>
#include <unistd.h>
#include <utime.h>

#include "esp_heap_caps.h"
#include "esp_log.h"
#include "esp_partition.h"
#include "esp_vfs.h"
#include "esp_vfs_fat.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"
#include "freertos/task.h"

static const char *TAG = "geapp_flash";

#define PRIVATE_ROOT "/.appsfat"
#define WORKER_STACK 6144
#define MAX_PATH 300

typedef struct Call {
	void (*fn)(struct Call *);
	const char *path;
	const char *path2;
	int fd;
	int flags;
	int mode;
	const void *in;
	void *out;
	size_t size;
	off_t offset;
	DIR *dir;
	struct dirent *entry;
	struct stat *st;
	long result;
	int error;
} Call;

static QueueHandle_t s_queue;
static SemaphoreHandle_t s_done;
static SemaphoreHandle_t s_lock;
static TaskHandle_t s_worker;
static char s_base[32];

typedef struct {
	DIR base;  // dd_vfs_idx is the proxy's: readdir and closedir come back here
	DIR *real;
	struct dirent entry;
} ProxyDir;

static void worker(void *arg)
{
	(void)arg;
	for (;;) {
		Call *call = NULL;
		if (xQueueReceive(s_queue, &call, portMAX_DELAY) != pdTRUE || !call) continue;
		errno = 0;
		call->fn(call);
		call->error = errno;
		xSemaphoreGive(s_done);
	}
}

// Runs `call` on the worker and waits; sets the caller's errno from it.
static long run(Call *call)
{
	if (xTaskGetCurrentTaskHandle() == s_worker) {
		call->fn(call);
		return call->result;
	}
	xSemaphoreTake(s_lock, portMAX_DELAY);
	Call *pointer = call;
	xQueueSend(s_queue, &pointer, portMAX_DELAY);
	xSemaphoreTake(s_done, portMAX_DELAY);
	xSemaphoreGive(s_lock);
	if (call->result < 0 || call->error) errno = call->error;
	return call->result;
}

// "/x/y" under the proxy -> "/.appsfat/x/y"; false if too long.
static bool privatePath(const char *path, char *out)
{
	const int n = snprintf(out, MAX_PATH, PRIVATE_ROOT "%s", path ? path : "");
	return n > 0 && n < MAX_PATH;
}

#define PATH_OR_FAIL(var, from)          \
	char var[MAX_PATH];                    \
	if (!privatePath((from), var)) {       \
		errno = ENAMETOOLONG;                \
		return -1;                           \
	}

static void doOpen(Call *c) { c->result = open(c->path, c->flags, c->mode); }
static void doClose(Call *c) { c->result = close(c->fd); }
static void doRead(Call *c) { c->result = read(c->fd, c->out, c->size); }
static void doWrite(Call *c) { c->result = write(c->fd, c->in, c->size); }
static void doLseek(Call *c) { c->result = lseek(c->fd, c->offset, c->mode); }
static void doPread(Call *c) { c->result = pread(c->fd, c->out, c->size, c->offset); }
static void doPwrite(Call *c) { c->result = pwrite(c->fd, c->in, c->size, c->offset); }
static void doFstat(Call *c) { c->result = fstat(c->fd, c->st); }
static void doFsync(Call *c) { c->result = fsync(c->fd); }
static void doStat(Call *c) { c->result = stat(c->path, c->st); }
static void doUnlink(Call *c) { c->result = unlink(c->path); }
static void doRename(Call *c) { c->result = rename(c->path, c->path2); }
static void doMkdir(Call *c) { c->result = mkdir(c->path, (mode_t)c->mode); }
static void doRmdir(Call *c) { c->result = rmdir(c->path); }
static void doAccess(Call *c) { c->result = access(c->path, c->mode); }
static void doTruncate(Call *c) { c->result = truncate(c->path, c->offset); }
static void doFtruncate(Call *c) { c->result = ftruncate(c->fd, c->offset); }
static void doOpendir(Call *c)
{
	c->dir = opendir(c->path);
	c->result = c->dir ? 0 : -1;
}
static void doReaddir(Call *c)
{
	c->entry = readdir(c->dir);
	c->result = c->entry ? 0 : -1;
}
static void doClosedir(Call *c) { c->result = closedir(c->dir); }

static int proxyOpen(void *ctx, const char *path, int flags, int mode)
{
	(void)ctx;
	PATH_OR_FAIL(real, path);
	Call c = {.fn = doOpen, .path = real, .flags = flags, .mode = mode};
	return (int)run(&c);
}
static int proxyClose(void *ctx, int fd)
{
	(void)ctx;
	Call c = {.fn = doClose, .fd = fd};
	return (int)run(&c);
}
static ssize_t proxyRead(void *ctx, int fd, void *dst, size_t size)
{
	(void)ctx;
	Call c = {.fn = doRead, .fd = fd, .out = dst, .size = size};
	return (ssize_t)run(&c);
}
static ssize_t proxyWrite(void *ctx, int fd, const void *data, size_t size)
{
	(void)ctx;
	Call c = {.fn = doWrite, .fd = fd, .in = data, .size = size};
	return (ssize_t)run(&c);
}
static off_t proxyLseek(void *ctx, int fd, off_t offset, int whence)
{
	(void)ctx;
	Call c = {.fn = doLseek, .fd = fd, .offset = offset, .mode = whence};
	return (off_t)run(&c);
}
static ssize_t proxyPread(void *ctx, int fd, void *dst, size_t size, off_t offset)
{
	(void)ctx;
	Call c = {.fn = doPread, .fd = fd, .out = dst, .size = size, .offset = offset};
	return (ssize_t)run(&c);
}
static ssize_t proxyPwrite(void *ctx, int fd, const void *src, size_t size, off_t offset)
{
	(void)ctx;
	Call c = {.fn = doPwrite, .fd = fd, .in = src, .size = size, .offset = offset};
	return (ssize_t)run(&c);
}
static int proxyFstat(void *ctx, int fd, struct stat *st)
{
	(void)ctx;
	Call c = {.fn = doFstat, .fd = fd, .st = st};
	return (int)run(&c);
}
static int proxyFsync(void *ctx, int fd)
{
	(void)ctx;
	Call c = {.fn = doFsync, .fd = fd};
	return (int)run(&c);
}
static int proxyStat(void *ctx, const char *path, struct stat *st)
{
	(void)ctx;
	PATH_OR_FAIL(real, path);
	Call c = {.fn = doStat, .path = real, .st = st};
	return (int)run(&c);
}
static int proxyUnlink(void *ctx, const char *path)
{
	(void)ctx;
	PATH_OR_FAIL(real, path);
	Call c = {.fn = doUnlink, .path = real};
	return (int)run(&c);
}
static int proxyRename(void *ctx, const char *from, const char *to)
{
	(void)ctx;
	PATH_OR_FAIL(realFrom, from);
	PATH_OR_FAIL(realTo, to);
	Call c = {.fn = doRename, .path = realFrom, .path2 = realTo};
	return (int)run(&c);
}
static int proxyMkdir(void *ctx, const char *path, mode_t mode)
{
	(void)ctx;
	PATH_OR_FAIL(real, path);
	Call c = {.fn = doMkdir, .path = real, .mode = (int)mode};
	return (int)run(&c);
}
static int proxyRmdir(void *ctx, const char *path)
{
	(void)ctx;
	PATH_OR_FAIL(real, path);
	Call c = {.fn = doRmdir, .path = real};
	return (int)run(&c);
}
static int proxyAccess(void *ctx, const char *path, int amode)
{
	(void)ctx;
	PATH_OR_FAIL(real, path);
	Call c = {.fn = doAccess, .path = real, .mode = amode};
	return (int)run(&c);
}
static int proxyTruncate(void *ctx, const char *path, off_t length)
{
	(void)ctx;
	PATH_OR_FAIL(real, path);
	Call c = {.fn = doTruncate, .path = real, .offset = length};
	return (int)run(&c);
}
static int proxyFtruncate(void *ctx, int fd, off_t length)
{
	(void)ctx;
	Call c = {.fn = doFtruncate, .fd = fd, .offset = length};
	return (int)run(&c);
}
static DIR *proxyOpendir(void *ctx, const char *path)
{
	(void)ctx;
	char real[MAX_PATH];
	if (!privatePath(path, real)) {
		errno = ENAMETOOLONG;
		return NULL;
	}
	ProxyDir *proxy = calloc(1, sizeof(ProxyDir));
	if (!proxy) {
		errno = ENOMEM;
		return NULL;
	}
	Call c = {.fn = doOpendir, .path = real};
	if (run(&c) < 0) {
		free(proxy);
		return NULL;
	}
	proxy->real = c.dir;
	return &proxy->base;
}
static struct dirent *proxyReaddir(void *ctx, DIR *dir)
{
	(void)ctx;
	ProxyDir *proxy = (ProxyDir *)dir;
	Call c = {.fn = doReaddir, .dir = proxy->real};
	if (run(&c) < 0 || !c.entry) return NULL;
	proxy->entry = *c.entry;  // copied while the worker waits for the next call
	return &proxy->entry;
}
static int proxyClosedir(void *ctx, DIR *dir)
{
	(void)ctx;
	ProxyDir *proxy = (ProxyDir *)dir;
	Call c = {.fn = doClosedir, .dir = proxy->real};
	const int result = (int)run(&c);
	free(proxy);
	return result;
}

static const esp_vfs_dir_ops_t s_dirOps = {
	.stat_p = proxyStat,
	.unlink_p = proxyUnlink,
	.rename_p = proxyRename,
	.opendir_p = proxyOpendir,
	.readdir_p = proxyReaddir,
	.closedir_p = proxyClosedir,
	.mkdir_p = proxyMkdir,
	.rmdir_p = proxyRmdir,
	.access_p = proxyAccess,
	.truncate_p = proxyTruncate,
	.ftruncate_p = proxyFtruncate,
};

static const esp_vfs_fs_ops_t s_ops = {
	.write_p = proxyWrite,
	.lseek_p = proxyLseek,
	.read_p = proxyRead,
	.pread_p = proxyPread,
	.pwrite_p = proxyPwrite,
	.open_p = proxyOpen,
	.close_p = proxyClose,
	.fstat_p = proxyFstat,
	.fsync_p = proxyFsync,
	.dir = &s_dirOps,
};

typedef struct {
	const char *label;
	esp_err_t err;
	unsigned long long total;
	unsigned long long free;
} MountCall;

static wl_handle_t s_wl = WL_INVALID_HANDLE;

static void doMount(Call *c)
{
	MountCall *m = (MountCall *)c->out;
	esp_vfs_fat_mount_config_t mount = {
		.format_if_mount_failed = true,
		.max_files = 6,
		.allocation_unit_size = 4096,
	};
	m->err = esp_vfs_fat_spiflash_mount_rw_wl(PRIVATE_ROOT, m->label, &mount, &s_wl);
	if (m->err == ESP_OK) {
		uint64_t total = 0, free = 0;
		esp_vfs_fat_info(PRIVATE_ROOT, &total, &free);
		m->total = total;
		m->free = free;
	}
	c->result = m->err == ESP_OK ? 0 : -1;
}

static void doInfo(Call *c)
{
	MountCall *m = (MountCall *)c->out;
	uint64_t total = 0, free = 0;
	m->err = esp_vfs_fat_info(PRIVATE_ROOT, &total, &free);
	m->total = total;
	m->free = free;
	c->result = m->err == ESP_OK ? 0 : -1;
}

static bool s_mounted = false;
static char s_status[96] = "not mounted yet";

const char *gea_geapp_flash_vfs_status(void) { return s_status; }

bool gea_geapp_flash_vfs_mount(const char *base, const char *label)
{
	if (s_mounted) return true;
	if (!esp_partition_find_first(ESP_PARTITION_TYPE_DATA, ESP_PARTITION_SUBTYPE_DATA_FAT, label)) {
		snprintf(s_status, sizeof(s_status), "no FAT partition labelled %s", label);
		return false;
	}
	if (!s_worker) {
		if (!s_queue) s_queue = xQueueCreate(1, sizeof(Call *));
		if (!s_done) s_done = xSemaphoreCreateBinary();
		if (!s_lock) s_lock = xSemaphoreCreateMutex();
		// The stack must be internal RAM: that is the whole point.
		if (!s_queue || !s_done || !s_lock ||
		    xTaskCreatePinnedToCoreWithCaps(worker, "geapp_flash", WORKER_STACK, NULL, 6, &s_worker, tskNO_AFFINITY,
		                                    MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT) != pdPASS) {
			s_worker = NULL;
			snprintf(s_status, sizeof(s_status), "no internal RAM for the flash worker (%u free)",
			         (unsigned)heap_caps_get_free_size(MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT));
			ESP_LOGE(TAG, "%s", s_status);
			return false;
		}
	}
	MountCall m = {.label = label};
	Call c = {.fn = doMount, .out = &m};
	if (run(&c) < 0) {
		snprintf(s_status, sizeof(s_status), "mount of %s failed: %s", label, esp_err_to_name(m.err));
		ESP_LOGE(TAG, "%s", s_status);
		return false;
	}
	snprintf(s_base, sizeof(s_base), "%s", base);
	const esp_err_t err = esp_vfs_register_fs(s_base, &s_ops, ESP_VFS_FLAG_STATIC | ESP_VFS_FLAG_CONTEXT_PTR, NULL);
	if (err != ESP_OK) {
		snprintf(s_status, sizeof(s_status), "registering %s failed: %s", s_base, esp_err_to_name(err));
		ESP_LOGE(TAG, "%s", s_status);
		return false;
	}
	s_mounted = true;
	snprintf(s_status, sizeof(s_status), "mounted %s at %s: %llu KB, %llu KB free", label, s_base, m.total / 1024, m.free / 1024);
	ESP_LOGI(TAG, "%s", s_status);
	return true;
}

bool gea_geapp_flash_vfs_free_bytes(const char *path, unsigned long long *out)
{
	if (!s_worker || !path) return false;
	const size_t n = strlen(s_base);
	if (strncmp(path, s_base, n) != 0 || (path[n] != '\0' && path[n] != '/')) return false;
	MountCall m = {0};
	Call c = {.fn = doInfo, .out = &m};
	if (run(&c) < 0) return false;
	*out = m.free;
	return true;
}

// GeaOS asks this before an install (geaos_fs_posix.cpp). A board with its own
// storage defines the strong one and answers for its volumes.
__attribute__((weak)) bool gea_geaos_free_bytes(const char *path, unsigned long long *out)
{
	return gea_geapp_flash_vfs_free_bytes(path, out);
}

#else

#include <stdbool.h>

const char *gea_geapp_flash_vfs_status(void) { return "this firmware has no FAT support"; }

bool gea_geapp_flash_vfs_mount(const char *base, const char *label)
{
	(void)base;
	(void)label;
	return false;
}

bool gea_geapp_flash_vfs_free_bytes(const char *path, unsigned long long *out)
{
	(void)path;
	(void)out;
	return false;
}

#endif
