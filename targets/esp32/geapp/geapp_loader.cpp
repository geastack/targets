// SPDX-License-Identifier: GPL-3.0-only
// The .geapp program loader.
//
// A program is a RISC-V ELF shared object linked with -fPIC -shared -nostdlib:
// its code addresses its own data PC-relatively and reaches everything else --
// the runtime, libstdc++, libc, ESP-IDF -- through its GOT and PLT. Loading is
// therefore: copy the PT_LOAD segments into one PSRAM block at any address,
// fill the GOT/PLT from the firmware's export table, and the code runs where it
// landed. The only relocations such an object carries are R_RISCV_RELATIVE,
// R_RISCV_32 and R_RISCV_JUMP_SLOT.
//
// Binding follows ELF symbol interposition with the firmware first: a symbol
// the firmware exports wins over the program's own copy. That is what keeps one
// instance of every runtime singleton -- a header-inline function's static, a
// template's static member, a vtable or typeinfo -- shared by the firmware and
// the program. The firmware's export table already leaves out what must stay
// the program's own (anonymous-namespace entities and the program hooks).
//
// PSRAM is executable when the firmware does not pre-configure PSRAM memory
// protection (CONFIG_SPIRAM_PRE_CONFIGURE_MEMORY_PROTECTION=n): the PMA entry
// for external RAM is RWX and the PMP window stays RWX.

#include "geapp_loader.h"

#include "geapp_exports.h"
#include "geapp_host.h"
#include "geaos_owned.h"

#include <cstdarg>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <new>
#include <vector>

#include <dirent.h>
#include <fcntl.h>
#include <unistd.h>

#include "driver/gpio.h"
#include "esp_cache.h"
#include "esp_heap_caps.h"
#include "sdkconfig.h"
#include "soc/soc.h"

#if CONFIG_IDF_TARGET_ARCH_XTENSA
#if !CONFIG_IDF_TARGET_ESP32S3
#error "geapp programs on Xtensa need the ESP32-S3: its instruction and data buses share one MMU"
#endif
#define GEAPP_XTENSA 1
#include "esp32s3/rom/cache.h"
#else
#define GEAPP_XTENSA 0
#endif
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/idf_additions.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"
#include "freertos/task.h"
#include "freertos/timers.h"

// lwIP, when the firmware links it (a build without networking does not).
struct sockaddr;
extern "C" int lwip_socket(int domain, int type, int protocol) __attribute__((weak));
extern "C" int lwip_accept(int fd, struct sockaddr *address, unsigned *length) __attribute__((weak));
extern "C" int lwip_close(int fd) __attribute__((weak));

extern "C" void __register_frame_info(const void *begin, void *object);
extern "C" void *__deregister_frame_info(const void *begin);
extern "C" int __cxa_atexit(void (*destructor)(void *), void *object, void *dso);

namespace gea::platform::geapp {

namespace {

constexpr char kTag[] = "geapp";

// ELF32 subset.
constexpr unsigned char kElfMagic[4] = {0x7f, 'E', 'L', 'F'};
constexpr int kClass32 = 1;
constexpr int kDataLsb = 1;
constexpr std::uint16_t kTypeDyn = 3;

#if GEAPP_XTENSA
constexpr std::uint16_t kMachine = 94;  // EM_XTENSA
constexpr char kMachineName[] = "Xtensa";
// The instruction-bus window over the same MMU pages as the data-bus window
// the heap hands out (the ESP32-S3 MMU is shared by both buses): code runs
// at its data address plus this.
constexpr std::uintptr_t kCodeOffset = SOC_IROM_LOW - SOC_DROM_LOW;
#else
constexpr std::uint16_t kMachine = 243;  // EM_RISCV
constexpr char kMachineName[] = "RISC-V";
constexpr std::uint32_t kFlagFloatAbiMask = 0x6;
constexpr std::uintptr_t kCodeOffset = 0;
#endif

constexpr std::uint32_t kPfExecute = 1;
constexpr std::uint32_t kPtLoad = 1;
constexpr std::uint32_t kPtDynamic = 2;
constexpr std::uint32_t kPtGnuEhFrame = 0x6474e550;

constexpr std::int32_t kDtNull = 0;
constexpr std::int32_t kDtNeeded = 1;
constexpr std::int32_t kDtPltRelSz = 2;
constexpr std::int32_t kDtHash = 4;
constexpr std::int32_t kDtStrTab = 5;
constexpr std::int32_t kDtSymTab = 6;
constexpr std::int32_t kDtRela = 7;
constexpr std::int32_t kDtRelaSz = 8;
constexpr std::int32_t kDtTextRel = 22;
constexpr std::int32_t kDtJmpRel = 23;
constexpr std::int32_t kDtInitArray = 25;
constexpr std::int32_t kDtFiniArray = 26;
constexpr std::int32_t kDtInitArraySz = 27;
constexpr std::int32_t kDtFiniArraySz = 28;

#if GEAPP_XTENSA
// Xtensa adds RELATIVE and 32 to what the field holds; RTLD names the lazy
// resolver's own slots, unused when everything binds at load.
constexpr std::uint32_t kRelNone = 0;
constexpr std::uint32_t kRel32 = 1;
constexpr std::uint32_t kRelRtld = 2;
constexpr std::uint32_t kRelGlobDat = 3;
constexpr std::uint32_t kRelJumpSlot = 4;
constexpr std::uint32_t kRelRelative = 5;
constexpr bool kRelInPlace = true;
#else
constexpr std::uint32_t kRelNone = 0;
constexpr std::uint32_t kRel32 = 1;
constexpr std::uint32_t kRelRtld = 0x100;     // none on RISC-V
constexpr std::uint32_t kRelGlobDat = 0x101;  // none on RISC-V
constexpr std::uint32_t kRelRelative = 3;
constexpr std::uint32_t kRelJumpSlot = 5;
constexpr bool kRelInPlace = false;
#endif

constexpr std::uint16_t kShnUndef = 0;
constexpr unsigned char kBindWeak = 2;

struct ElfHeader {
	unsigned char ident[16];
	std::uint16_t type;
	std::uint16_t machine;
	std::uint32_t version;
	std::uint32_t entry;
	std::uint32_t phoff;
	std::uint32_t shoff;
	std::uint32_t flags;
	std::uint16_t ehsize;
	std::uint16_t phentsize;
	std::uint16_t phnum;
	std::uint16_t shentsize;
	std::uint16_t shnum;
	std::uint16_t shstrndx;
};

struct ProgramHeader {
	std::uint32_t type;
	std::uint32_t offset;
	std::uint32_t vaddr;
	std::uint32_t paddr;
	std::uint32_t filesz;
	std::uint32_t memsz;
	std::uint32_t flags;
	std::uint32_t align;
};

struct Dynamic {
	std::int32_t tag;
	std::uint32_t value;
};

struct Symbol {
	std::uint32_t name;
	std::uint32_t value;
	std::uint32_t size;
	unsigned char info;
	unsigned char other;
	std::uint16_t shndx;
};

struct Rela {
	std::uint32_t offset;
	std::uint32_t info;
	std::int32_t addend;
};

struct AtExit {
	void (*destructor)(void *);
	void *object;
};

// An RTOS task a program created, deleted with the program if still alive.
struct OwnedTask {
	TaskHandle_t handle;
	bool withCaps;
};

// libgcc's `struct object` is seven words on this target; leave room.
struct FrameObject {
	void *words[16];
};

}  // namespace

struct Module {
	std::uint8_t *image = nullptr;
	std::size_t size = 0;
	// The executable segment's image offsets: an address there is code, run
	// at image + offset + kCodeOffset.
	std::uint32_t codeStart = 0;
	std::uint32_t codeEnd = 0;
	const Symbol *symbols = nullptr;
	std::uint32_t symbolCount = 0;
	const char *strings = nullptr;
	std::uint32_t initArray = 0;
	std::uint32_t initArraySize = 0;
	std::uint32_t finiArray = 0;
	std::uint32_t finiArraySize = 0;
	const void *ehFrame = nullptr;
	FrameObject frameObject{};
	bool framesRegistered = false;
	std::vector<AtExit> atExit;
	std::vector<OwnedTask> tasks;
	std::vector<QueueHandle_t> queues;
	// What the program's own code allocated or opened and has not given back
	// (geapp_owned below): taken back when it unloads.
	gea::geaos::OwnedSet blocks;
	gea::geaos::OwnedSet::Slot *blockSlots = nullptr;
	std::vector<esp_timer_handle_t> espTimers;
	std::vector<TimerHandle_t> rtosTimers;
	std::vector<FILE *> files;
	std::vector<int> fds;
	std::vector<DIR *> dirs;
	std::vector<int> isrPins;
};

namespace {

// The module whose constructors are running: its __cxa_atexit registrations are
// its own, to run when it unloads rather than never (the firmware never exits).
Module *g_constructing = nullptr;
std::vector<Module *> g_modules;

bool inCode(const Module &module, std::uint32_t offset)
{
	return offset >= module.codeStart && offset < module.codeEnd;
}

// The runtime address of an image offset: code through the instruction bus,
// everything else through the data bus.
std::uintptr_t addressAt(const Module &module, std::uint32_t offset)
{
	return reinterpret_cast<std::uintptr_t>(module.image) + offset + (inCode(module, offset) ? kCodeOffset : 0);
}

Module *moduleContaining(const void *address)
{
	for (Module *module : g_modules)
		if (contains(module, address)) return module;
	if (g_constructing && contains(g_constructing, address)) return g_constructing;
	return nullptr;
}

// Bound in place of __cxa_atexit for every program. A destructor for an object
// in a program's own image runs when that program unloads; anything else (a
// firmware singleton a program's inline code happened to construct first) is
// the firmware's for life.
extern "C" int geapp_cxa_atexit(void (*destructor)(void *), void *object, void *dso)
{
	if (Module *module = moduleContaining(object)) {
		module->atExit.push_back({destructor, object});
		return 0;
	}
	return __cxa_atexit(destructor, object, dso);
}

// Tasks and queues (semaphores and mutexes are queues) a program creates are
// its own: its code is about to be unmapped, so a task left behind would run
// freed memory the next time it woke, and a queue would never be returned.
// Bound in place of the creating and deleting calls for every program; the
// owner is the module the caller's code lies in.
Module *callerModule(const void *returnAddress) { return moduleContaining(returnAddress); }

template <typename Handle>
void forget(std::vector<Handle> &handles, Handle handle)
{
	for (auto it = handles.begin(); it != handles.end(); ++it)
		if (*it == handle) {
			handles.erase(it);
			return;
		}
}

extern "C" BaseType_t geapp_xTaskCreatePinnedToCore(TaskFunction_t code, const char *const name, const configSTACK_DEPTH_TYPE depth,
                                                    void *const parameters, UBaseType_t priority, TaskHandle_t *const created, const BaseType_t core)
{
	TaskHandle_t handle = nullptr;
	const BaseType_t ok = xTaskCreatePinnedToCore(code, name, depth, parameters, priority, &handle, core);
	if (ok == pdPASS && handle) {
		if (Module *module = callerModule(__builtin_return_address(0))) module->tasks.push_back({handle, false});
	}
	if (created) *created = handle;
	return ok;
}

extern "C" BaseType_t geapp_xTaskCreatePinnedToCoreWithCaps(TaskFunction_t code, const char *const name, const configSTACK_DEPTH_TYPE depth,
                                                            void *const parameters, UBaseType_t priority, TaskHandle_t *const created,
                                                            const BaseType_t core, UBaseType_t caps)
{
	TaskHandle_t handle = nullptr;
	const BaseType_t ok = xTaskCreatePinnedToCoreWithCaps(code, name, depth, parameters, priority, &handle, core, caps);
	if (ok == pdPASS && handle) {
		if (Module *module = callerModule(__builtin_return_address(0))) module->tasks.push_back({handle, true});
	}
	if (created) *created = handle;
	return ok;
}

void forgetTask(TaskHandle_t handle)
{
	if (!handle) handle = xTaskGetCurrentTaskHandle();
	for (Module *module : g_modules)
		for (auto it = module->tasks.begin(); it != module->tasks.end(); ++it)
			if (it->handle == handle) {
				module->tasks.erase(it);
				return;
			}
}

extern "C" void geapp_vTaskDelete(TaskHandle_t handle)
{
	forgetTask(handle);
	vTaskDelete(handle);
}

extern "C" void geapp_vTaskDeleteWithCaps(TaskHandle_t handle)
{
	forgetTask(handle);
	vTaskDeleteWithCaps(handle);
}

extern "C" QueueHandle_t geapp_xQueueGenericCreate(const UBaseType_t length, const UBaseType_t itemSize, const uint8_t type)
{
	QueueHandle_t queue = xQueueGenericCreate(length, itemSize, type);
	if (queue) {
		if (Module *module = callerModule(__builtin_return_address(0))) module->queues.push_back(queue);
	}
	return queue;
}

extern "C" QueueHandle_t geapp_xQueueCreateMutex(const uint8_t type)
{
	QueueHandle_t queue = xQueueCreateMutex(type);
	if (queue) {
		if (Module *module = callerModule(__builtin_return_address(0))) module->queues.push_back(queue);
	}
	return queue;
}

extern "C" QueueHandle_t geapp_xQueueCreateCountingSemaphore(const UBaseType_t maxCount, const UBaseType_t initialCount)
{
	QueueHandle_t queue = xQueueCreateCountingSemaphore(maxCount, initialCount);
	if (queue) {
		if (Module *module = callerModule(__builtin_return_address(0))) module->queues.push_back(queue);
	}
	return queue;
}

extern "C" void geapp_vQueueDelete(QueueHandle_t queue)
{
	for (Module *module : g_modules) forget(module->queues, queue);
	vQueueDelete(queue);
}

// ---- What a program owns ----------------------------------------------------
//
// A program's heap blocks, timers, files, sockets and GPIO interrupts are its
// own, taken back when it unloads whether or not it gave them back. The calls
// that create them are bound to the wrappers below for every program; the
// owner is the module the calling code lies in, exactly as for tasks.
//
// Heap blocks are recorded when program code allocates them and forgotten
// when anything frees them: a block the program allocated is often grown or
// freed later by firmware code (a runtime container's buffer), so the record
// follows every free in the system through the heap's free hook
// (CONFIG_HEAP_USE_HOOKS). Without the hooks nothing is recorded: reclaiming
// from a record that misses frees would free blocks twice.

// Room for 28K live blocks (the table refuses at 7/8). A program that holds
// more is reclaimed for what was recorded: every block on the record is
// still its own, so freeing those is safe; the rest leak, and the overflow
// is logged.
constexpr std::size_t kBlockSlots = 32768;

// Recursive: a wrapper's own bookkeeping (a vector growing) allocates.
SemaphoreHandle_t ownLock()
{
	static StaticSemaphore_t storage;
	static SemaphoreHandle_t lock = xSemaphoreCreateRecursiveMutexStatic(&storage);
	return lock;
}

struct OwnGuard {
	OwnGuard() { xSemaphoreTakeRecursive(ownLock(), portMAX_DELAY); }
	~OwnGuard() { xSemaphoreGiveRecursive(ownLock()); }
};

#if CONFIG_HEAP_USE_HOOKS
portMUX_TYPE g_heapMux = portMUX_INITIALIZER_UNLOCKED;
// The modules whose blocks the free hook keeps current (at most the one
// running and the one being switched in).
Module *volatile g_tracked[2] = {nullptr, nullptr};
volatile bool g_tracking = false;

void trackBlocks(Module *module, bool on)
{
	portENTER_CRITICAL(&g_heapMux);
	for (auto &slot : g_tracked) {
		if (on && !slot) {
			slot = module;
			break;
		}
		if (!on && slot == module) slot = nullptr;
	}
	g_tracking = g_tracked[0] || g_tracked[1];
	portEXIT_CRITICAL(&g_heapMux);
}

void recordBlock(const void *returnAddress, void *block, std::size_t size)
{
	if (!block || !g_tracking) return;
	Module *module = moduleContaining(returnAddress);
	if (!module || !module->blockSlots) return;
	portENTER_CRITICAL(&g_heapMux);
	if (g_tracked[0] == module || g_tracked[1] == module)
		module->blocks.add(reinterpret_cast<std::uintptr_t>(block), static_cast<std::uint32_t>(size));
	portEXIT_CRITICAL(&g_heapMux);
}
#else
[[maybe_unused]] void trackBlocks(Module *, bool) {}
void recordBlock(const void *, void *, std::size_t) {}
#endif

}  // namespace
}  // namespace gea::platform::geapp

#if CONFIG_HEAP_USE_HOOKS
// Every free in the system, from any task: forget the block if a program
// owned it. Short and allocation-free, as the heap requires of its hooks.
extern "C" void esp_heap_trace_free_hook(void *ptr)
{
	using namespace gea::platform::geapp;
	if (!g_tracking || !ptr) return;
	portENTER_CRITICAL_SAFE(&g_heapMux);
	for (Module *module : g_tracked)
		if (module && module->blocks.remove(reinterpret_cast<std::uintptr_t>(ptr))) break;
	portEXIT_CRITICAL_SAFE(&g_heapMux);
}
#endif

namespace gea::platform::geapp {
namespace {

#define GEAPP_RECORD(expr, size)                                                   \
	do {                                                                           \
		void *const block_ = (expr);                                               \
		recordBlock(__builtin_return_address(0), block_, (size));                  \
		return block_;                                                             \
	} while (0)

extern "C" void *geapp_malloc(std::size_t size) { GEAPP_RECORD(std::malloc(size), size); }
extern "C" void *geapp_calloc(std::size_t count, std::size_t size) { GEAPP_RECORD(std::calloc(count, size), count * size); }
extern "C" void *geapp_realloc(void *old, std::size_t size) { GEAPP_RECORD(std::realloc(old, size), size); }
extern "C" void *geapp_heap_caps_malloc(std::size_t size, std::uint32_t caps) { GEAPP_RECORD(heap_caps_malloc(size, caps), size); }
extern "C" void *geapp_heap_caps_calloc(std::size_t count, std::size_t size, std::uint32_t caps)
{
	GEAPP_RECORD(heap_caps_calloc(count, size, caps), count * size);
}
extern "C" void *geapp_heap_caps_realloc(void *old, std::size_t size, std::uint32_t caps) { GEAPP_RECORD(heap_caps_realloc(old, size, caps), size); }
extern "C" void *geapp_heap_caps_aligned_alloc(std::size_t alignment, std::size_t size, std::uint32_t caps)
{
	GEAPP_RECORD(heap_caps_aligned_alloc(alignment, size, caps), size);
}
extern "C" void *geapp_heap_caps_aligned_calloc(std::size_t alignment, std::size_t count, std::size_t size, std::uint32_t caps)
{
	GEAPP_RECORD(heap_caps_aligned_calloc(alignment, count, size, caps), count * size);
}
extern "C" void *geapp_aligned_alloc(std::size_t alignment, std::size_t size) { GEAPP_RECORD(aligned_alloc(alignment, size), size); }
extern "C" char *geapp_strdup(const char *text)
{
	char *copy = strdup(text);
	recordBlock(__builtin_return_address(0), copy, copy ? std::strlen(copy) + 1 : 0);
	return copy;
}
extern "C" char *geapp_strndup(const char *text, std::size_t limit)
{
	char *copy = strndup(text, limit);
	recordBlock(__builtin_return_address(0), copy, copy ? std::strlen(copy) + 1 : 0);
	return copy;
}
// operator new and new[] (size_t is unsigned int here): the generated code's
// objects and every inline container's storage.
extern "C" void *geapp_new(std::size_t size) { GEAPP_RECORD(::operator new(size), size); }
extern "C" void *geapp_new_array(std::size_t size) { GEAPP_RECORD(::operator new[](size), size); }
extern "C" void *geapp_new_nothrow(std::size_t size, const std::nothrow_t &tag) { GEAPP_RECORD(::operator new(size, tag), size); }
extern "C" void *geapp_new_array_nothrow(std::size_t size, const std::nothrow_t &tag) { GEAPP_RECORD(::operator new[](size, tag), size); }
extern "C" void *geapp_new_aligned(std::size_t size, std::align_val_t alignment) { GEAPP_RECORD(::operator new(size, alignment), size); }
extern "C" void *geapp_new_array_aligned(std::size_t size, std::align_val_t alignment) { GEAPP_RECORD(::operator new[](size, alignment), size); }

#undef GEAPP_RECORD

extern "C" esp_err_t geapp_esp_timer_create(const esp_timer_create_args_t *args, esp_timer_handle_t *out)
{
	const esp_err_t err = esp_timer_create(args, out);
	if (err == ESP_OK && out && *out) {
		OwnGuard guard;
		if (Module *module = callerModule(__builtin_return_address(0))) module->espTimers.push_back(*out);
	}
	return err;
}

extern "C" esp_err_t geapp_esp_timer_delete(esp_timer_handle_t timer)
{
	{
		OwnGuard guard;
		for (Module *module : g_modules) forget(module->espTimers, timer);
	}
	return esp_timer_delete(timer);
}

extern "C" TimerHandle_t geapp_xTimerCreate(const char *const name, const TickType_t period, const BaseType_t reload, void *const id,
                                           TimerCallbackFunction_t callback)
{
	TimerHandle_t timer = xTimerCreate(name, period, reload, id, callback);
	if (timer) {
		OwnGuard guard;
		if (Module *module = callerModule(__builtin_return_address(0))) module->rtosTimers.push_back(timer);
	}
	return timer;
}

extern "C" TimerHandle_t geapp_xTimerCreateStatic(const char *const name, const TickType_t period, const BaseType_t reload, void *const id,
                                                 TimerCallbackFunction_t callback, StaticTimer_t *buffer)
{
	TimerHandle_t timer = xTimerCreateStatic(name, period, reload, id, callback, buffer);
	if (timer) {
		OwnGuard guard;
		if (Module *module = callerModule(__builtin_return_address(0))) module->rtosTimers.push_back(timer);
	}
	return timer;
}

// xTimerDelete is this call with tmrCOMMAND_DELETE.
extern "C" BaseType_t geapp_xTimerGenericCommand(TimerHandle_t timer, const BaseType_t command, const TickType_t value, BaseType_t *const woken,
                                                const TickType_t wait)
{
	if (command == tmrCOMMAND_DELETE) {
		OwnGuard guard;
		for (Module *module : g_modules) forget(module->rtosTimers, timer);
	}
	return xTimerGenericCommand(timer, command, value, woken, wait);
}

extern "C" FILE *geapp_fopen(const char *path, const char *mode)
{
	FILE *file = std::fopen(path, mode);
	if (file) {
		OwnGuard guard;
		if (Module *module = callerModule(__builtin_return_address(0))) module->files.push_back(file);
	}
	return file;
}

extern "C" int geapp_fclose(FILE *file)
{
	{
		OwnGuard guard;
		for (Module *module : g_modules) forget(module->files, file);
	}
	return std::fclose(file);
}

extern "C" int geapp_open(const char *path, int flags, ...)
{
	int mode = 0;
	if (flags & O_CREAT) {
		va_list args;
		va_start(args, flags);
		mode = va_arg(args, int);
		va_end(args);
	}
	const int fd = ::open(path, flags, mode);
	if (fd >= 0) {
		OwnGuard guard;
		if (Module *module = callerModule(__builtin_return_address(0))) module->fds.push_back(fd);
	}
	return fd;
}

void recordFd(const void *returnAddress, int fd)
{
	if (fd < 0) return;
	OwnGuard guard;
	if (Module *module = callerModule(returnAddress)) module->fds.push_back(fd);
}

void forgetFd(int fd)
{
	OwnGuard guard;
	for (Module *module : g_modules) forget(module->fds, fd);
}

extern "C" int geapp_close(int fd)
{
	forgetFd(fd);
	return ::close(fd);
}

extern "C" int geapp_lwip_socket(int domain, int type, int protocol)
{
	const int fd = lwip_socket ? lwip_socket(domain, type, protocol) : -1;
	recordFd(__builtin_return_address(0), fd);
	return fd;
}

extern "C" int geapp_lwip_accept(int fd, struct sockaddr *address, unsigned *length)
{
	const int accepted = lwip_accept ? lwip_accept(fd, address, length) : -1;
	recordFd(__builtin_return_address(0), accepted);
	return accepted;
}

extern "C" int geapp_lwip_close(int fd)
{
	forgetFd(fd);
	return lwip_close ? lwip_close(fd) : ::close(fd);
}

extern "C" DIR *geapp_opendir(const char *path)
{
	DIR *dir = ::opendir(path);
	if (dir) {
		OwnGuard guard;
		if (Module *module = callerModule(__builtin_return_address(0))) module->dirs.push_back(dir);
	}
	return dir;
}

extern "C" int geapp_closedir(DIR *dir)
{
	{
		OwnGuard guard;
		for (Module *module : g_modules) forget(module->dirs, dir);
	}
	return ::closedir(dir);
}

extern "C" esp_err_t geapp_gpio_isr_handler_add(gpio_num_t pin, gpio_isr_t handler, void *arg)
{
	const esp_err_t err = gpio_isr_handler_add(pin, handler, arg);
	if (err == ESP_OK) {
		OwnGuard guard;
		if (Module *module = callerModule(__builtin_return_address(0))) module->isrPins.push_back(static_cast<int>(pin));
	}
	return err;
}

extern "C" esp_err_t geapp_gpio_isr_handler_remove(gpio_num_t pin)
{
	{
		OwnGuard guard;
		for (Module *module : g_modules) forget(module->isrPins, static_cast<int>(pin));
	}
	return gpio_isr_handler_remove(pin);
}

// Waits until the timer service task has run every command queued before
// this call (the deletes just sent), so no timer callback is mid-flight in
// the program's code when its image goes.
void drainTimerCommands()
{
	SemaphoreHandle_t done = xSemaphoreCreateBinary();
	if (!done) return;
	if (xTimerPendFunctionCall([](void *semaphore, std::uint32_t) { xSemaphoreGive(static_cast<SemaphoreHandle_t>(semaphore)); }, done, 0,
	                           pdMS_TO_TICKS(1000)) == pdPASS)
		xSemaphoreTake(done, pdMS_TO_TICKS(1000));
	vSemaphoreDelete(done);
}

void fail(char *error, std::size_t errorSize, const char *format, ...)
{
	if (!error || errorSize == 0) return;
	va_list args;
	va_start(args, format);
	std::vsnprintf(error, errorSize, format, args);
	va_end(args);
}

// The image's bytes: a whole file, or one entry of a .geapp package.
struct Source {
	FILE *file;
	std::uint32_t base;
	std::uint32_t length;
};

bool readAt(const Source &source, std::uint32_t offset, void *out, std::size_t size)
{
	if (offset > source.length || size > source.length - offset) return false;
	if (std::fseek(source.file, static_cast<long>(source.base + offset), SEEK_SET) != 0) return false;
	return std::fread(out, 1, size, source.file) == size;
}

std::uintptr_t specialSymbol(const char *name)
{
	struct Special {
		const char *name;
		std::uintptr_t address;
	};
	static const Special specials[] = {
		{"__cxa_atexit", reinterpret_cast<std::uintptr_t>(&geapp_cxa_atexit)},
		{"xTaskCreatePinnedToCore", reinterpret_cast<std::uintptr_t>(&geapp_xTaskCreatePinnedToCore)},
		{"xTaskCreatePinnedToCoreWithCaps", reinterpret_cast<std::uintptr_t>(&geapp_xTaskCreatePinnedToCoreWithCaps)},
		{"vTaskDelete", reinterpret_cast<std::uintptr_t>(&geapp_vTaskDelete)},
		{"vTaskDeleteWithCaps", reinterpret_cast<std::uintptr_t>(&geapp_vTaskDeleteWithCaps)},
		{"xQueueGenericCreate", reinterpret_cast<std::uintptr_t>(&geapp_xQueueGenericCreate)},
		{"xQueueCreateMutex", reinterpret_cast<std::uintptr_t>(&geapp_xQueueCreateMutex)},
		{"xQueueCreateCountingSemaphore", reinterpret_cast<std::uintptr_t>(&geapp_xQueueCreateCountingSemaphore)},
		{"vQueueDelete", reinterpret_cast<std::uintptr_t>(&geapp_vQueueDelete)},
		{"malloc", reinterpret_cast<std::uintptr_t>(&geapp_malloc)},
		{"calloc", reinterpret_cast<std::uintptr_t>(&geapp_calloc)},
		{"realloc", reinterpret_cast<std::uintptr_t>(&geapp_realloc)},
		{"heap_caps_malloc", reinterpret_cast<std::uintptr_t>(&geapp_heap_caps_malloc)},
		{"heap_caps_calloc", reinterpret_cast<std::uintptr_t>(&geapp_heap_caps_calloc)},
		{"heap_caps_realloc", reinterpret_cast<std::uintptr_t>(&geapp_heap_caps_realloc)},
		{"heap_caps_aligned_alloc", reinterpret_cast<std::uintptr_t>(&geapp_heap_caps_aligned_alloc)},
		{"heap_caps_aligned_calloc", reinterpret_cast<std::uintptr_t>(&geapp_heap_caps_aligned_calloc)},
		{"aligned_alloc", reinterpret_cast<std::uintptr_t>(&geapp_aligned_alloc)},
		{"strdup", reinterpret_cast<std::uintptr_t>(&geapp_strdup)},
		{"strndup", reinterpret_cast<std::uintptr_t>(&geapp_strndup)},
		{"_Znwj", reinterpret_cast<std::uintptr_t>(&geapp_new)},
		{"_Znaj", reinterpret_cast<std::uintptr_t>(&geapp_new_array)},
		{"_ZnwjRKSt9nothrow_t", reinterpret_cast<std::uintptr_t>(&geapp_new_nothrow)},
		{"_ZnajRKSt9nothrow_t", reinterpret_cast<std::uintptr_t>(&geapp_new_array_nothrow)},
		{"_ZnwjSt11align_val_t", reinterpret_cast<std::uintptr_t>(&geapp_new_aligned)},
		{"_ZnajSt11align_val_t", reinterpret_cast<std::uintptr_t>(&geapp_new_array_aligned)},
		{"esp_timer_create", reinterpret_cast<std::uintptr_t>(&geapp_esp_timer_create)},
		{"esp_timer_delete", reinterpret_cast<std::uintptr_t>(&geapp_esp_timer_delete)},
		{"xTimerCreate", reinterpret_cast<std::uintptr_t>(&geapp_xTimerCreate)},
		{"xTimerCreateStatic", reinterpret_cast<std::uintptr_t>(&geapp_xTimerCreateStatic)},
		{"xTimerGenericCommand", reinterpret_cast<std::uintptr_t>(&geapp_xTimerGenericCommand)},
		{"fopen", reinterpret_cast<std::uintptr_t>(&geapp_fopen)},
		{"fclose", reinterpret_cast<std::uintptr_t>(&geapp_fclose)},
		{"open", reinterpret_cast<std::uintptr_t>(&geapp_open)},
		{"close", reinterpret_cast<std::uintptr_t>(&geapp_close)},
		{"opendir", reinterpret_cast<std::uintptr_t>(&geapp_opendir)},
		{"closedir", reinterpret_cast<std::uintptr_t>(&geapp_closedir)},
		{"lwip_socket", reinterpret_cast<std::uintptr_t>(&geapp_lwip_socket)},
		{"lwip_accept", reinterpret_cast<std::uintptr_t>(&geapp_lwip_accept)},
		{"lwip_close", reinterpret_cast<std::uintptr_t>(&geapp_lwip_close)},
		{"gpio_isr_handler_add", reinterpret_cast<std::uintptr_t>(&geapp_gpio_isr_handler_add)},
		{"gpio_isr_handler_remove", reinterpret_cast<std::uintptr_t>(&geapp_gpio_isr_handler_remove)},
	};
	for (const Special &special : specials)
		if (std::strcmp(name, special.name) == 0) return special.address;
	return 0;
}

// The address a relocation's symbol binds to: the firmware's export first, then
// the module's own definition, then 0 for an undefined weak reference.
bool resolve(Module &module, std::uint32_t index, std::uintptr_t &out, char *error, std::size_t errorSize)
{
	if (index == 0) {
		out = 0;
		return true;
	}
	if (index >= module.symbolCount) {
		fail(error, errorSize, "symbol index %u out of range", static_cast<unsigned>(index));
		return false;
	}
	const Symbol &symbol = module.symbols[index];
	const char *name = module.strings + symbol.name;
	if (const std::uintptr_t special = specialSymbol(name)) {
		out = special;
		return true;
	}
	if (const std::uintptr_t exported = findExport(name)) {
		out = exported;
		return true;
	}
	if (symbol.shndx != kShnUndef) {
		out = addressAt(module, symbol.value);
		return true;
	}
	if ((symbol.info >> 4) == kBindWeak) {
		out = 0;
		return true;
	}
	fail(error, errorSize, "unresolved symbol %s", name);
	return false;
}

bool relocate(Module &module, std::uint32_t tableOffset, std::uint32_t tableSize, char *error, std::size_t errorSize)
{
	const auto *relocations = reinterpret_cast<const Rela *>(module.image + tableOffset);
	const std::size_t count = tableSize / sizeof(Rela);
	for (std::size_t i = 0; i < count; ++i) {
		const Rela &rela = relocations[i];
		if (rela.offset + sizeof(std::uint32_t) > module.size) {
			fail(error, errorSize, "relocation at 0x%x outside the image", static_cast<unsigned>(rela.offset));
			return false;
		}
		auto *target = reinterpret_cast<std::uint32_t *>(module.image + rela.offset);
		const std::uint32_t type = rela.info & 0xff;
		const std::uint32_t symbolIndex = rela.info >> 8;
		const std::uint32_t held = kRelInPlace ? *target : 0;
		switch (type) {
		case kRelNone:
		case kRelRtld:
			break;
		case kRelRelative: {
			// An image offset: code if it lands in the executable segment.
			const std::uint32_t offset = held + static_cast<std::uint32_t>(rela.addend);
			*target = static_cast<std::uint32_t>(addressAt(module, offset));
			break;
		}
		case kRel32:
		case kRelGlobDat:
		case kRelJumpSlot: {
			std::uintptr_t value = 0;
			if (!resolve(module, symbolIndex, value, error, errorSize)) return false;
			// A jump slot holds the lazy resolver's stub until bound: overwrite it.
			const std::uint32_t addend = type == kRelJumpSlot ? (kRelInPlace ? rela.addend : 0) : held + rela.addend;
			*target = static_cast<std::uint32_t>(value + addend);
			break;
		}
		default:
			fail(error, errorSize, "unsupported relocation type %u", static_cast<unsigned>(type));
			return false;
		}
	}
	return true;
}

// .eh_frame_hdr: version, eh_frame_ptr encoding, fde count encoding, table
// encoding, then the encoded eh_frame pointer.
const void *ehFrameFromHeader(const std::uint8_t *header)
{
	if (header[0] != 1) return nullptr;
	const std::uint8_t encoding = header[1];
	const std::uint8_t *field = header + 4;
	std::int32_t value;
	std::memcpy(&value, field, sizeof(value));
	switch (encoding) {
	case 0x1b:  // DW_EH_PE_pcrel | DW_EH_PE_sdata4
		return field + value;
	case 0x3b:  // DW_EH_PE_datarel | DW_EH_PE_sdata4: relative to the header
		return header + value;
	default:
		return nullptr;
	}
}

std::uint64_t readUleb(const std::uint8_t *&p, const std::uint8_t *end)
{
	std::uint64_t value = 0;
	unsigned shift = 0;
	while (p < end) {
		const std::uint8_t byte = *p++;
		if (shift < 64) value |= static_cast<std::uint64_t>(byte & 0x7f) << shift;
		shift += 7;
		if (!(byte & 0x80)) break;
	}
	return value;
}

// Adds kCodeOffset to every FDE's pc_begin that is encoded PC-relative (the
// only encoding a -fPIC link writes): the field is relative to its own data
// address, and the code it names runs kCodeOffset higher. CIEs are read for
// each FDE's pointer encoding; anything not understood is left alone and
// reported.
void shiftFrameCode(const std::uint8_t *frames, const std::uint8_t *imageEnd)
{
	struct Cie {
		const std::uint8_t *at;
		std::uint8_t encoding;
	};
	Cie cies[8];
	unsigned cieCount = 0;
	unsigned shifted = 0, skipped = 0;
	const std::uint8_t *p = frames;
	while (p + 4 <= imageEnd) {
		std::uint32_t length;
		std::memcpy(&length, p, 4);
		if (length == 0 || length == 0xffffffff) break;
		const std::uint8_t *entry = p + 4;
		const std::uint8_t *next = entry + length;
		if (next > imageEnd) break;
		std::uint32_t id;
		std::memcpy(&id, entry, 4);
		if (id == 0) {
			// CIE: version, augmentation, code/data alignment, return register,
			// then the augmentation data the string describes.
			const std::uint8_t *q = entry + 4;
			const std::uint8_t version = *q++;
			const char *augmentation = reinterpret_cast<const char *>(q);
			q += std::strlen(augmentation) + 1;
			readUleb(q, next);
			readUleb(q, next);  // data alignment (sleb; skipped the same way)
			if (version == 1) ++q;
			else readUleb(q, next);
			std::uint8_t encoding = 0;  // DW_EH_PE_absptr
			if (augmentation[0] == 'z') {
				readUleb(q, next);
				for (const char *a = augmentation + 1; *a && q < next; ++a) {
					if (*a == 'R') encoding = *q++;
					else if (*a == 'L') ++q;
					else if (*a == 'P') {
						const std::uint8_t personality = *q++;
						// Only the sdata4/udata4 forms a 32-bit link writes.
						q += (personality & 0x0f) == 0x0b || (personality & 0x0f) == 0x03 || (personality & 0x0f) == 0 ? 4 : 0;
					} else if (*a == 'S' || *a == 'B') {
					} else break;
				}
			}
			if (cieCount < sizeof(cies) / sizeof(cies[0])) cies[cieCount++] = {p, encoding};
		} else {
			const std::uint8_t *cie = entry - id;
			std::uint8_t encoding = 0xff;
			for (unsigned i = 0; i < cieCount; ++i)
				if (cies[i].at == cie) encoding = cies[i].encoding;
			auto *field = const_cast<std::uint8_t *>(entry + 4);
			// DW_EH_PE_pcrel with a 4-byte value (sdata4 or udata4).
			if ((encoding & 0x70) == 0x10 && ((encoding & 0x0f) == 0x0b || (encoding & 0x0f) == 0x03)) {
				std::int32_t delta;
				std::memcpy(&delta, field, 4);
				delta += static_cast<std::int32_t>(kCodeOffset);
				std::memcpy(field, &delta, 4);
				++shifted;
			} else {
				++skipped;
			}
		}
		p = next;
	}
	if (skipped) ESP_LOGW(kTag, "%u frame description(s) use an encoding the loader does not move; exceptions through them will not unwind", skipped);
	ESP_LOGD(kTag, "moved %u frame description(s) to the instruction bus", shifted);
}

void destroy(Module *module)
{
	if (!module) return;
	if (module->framesRegistered) __deregister_frame_info(module->ehFrame);
	if (module->image) heap_caps_free(module->image);
	if (module->blockSlots) heap_caps_free(module->blockSlots);
	delete module;
}

}  // namespace

Module *map(const char *path, char *error, std::size_t errorSize)
{
	return map(path, 0, UINT32_MAX, error, errorSize);
}

Module *map(const char *path, std::uint32_t offset, std::uint32_t length, char *error, std::size_t errorSize)
{
	if (exportCount() == 0) {
		fail(error, errorSize, "this firmware has no export table");
		return nullptr;
	}
	FILE *file = std::fopen(path, "rb");
	if (!file) {
		fail(error, errorSize, "cannot open %s", path);
		return nullptr;
	}
	const Source source{file, offset, length};
	ElfHeader header{};
	if (!readAt(source, 0, &header, sizeof(header)) || std::memcmp(header.ident, kElfMagic, 4) != 0 ||
	    header.ident[4] != kClass32 || header.ident[5] != kDataLsb) {
		std::fclose(file);
		fail(error, errorSize, "%s is not a 32-bit ELF", path);
		return nullptr;
	}
	if (header.type != kTypeDyn || header.machine != kMachine) {
		std::fclose(file);
		fail(error, errorSize, "%s is not a%s %s shared object", path, GEAPP_XTENSA ? "n" : "", kMachineName);
		return nullptr;
	}
#if !GEAPP_XTENSA
	// Single-precision hard float, as the firmware is built.
	if ((header.flags & kFlagFloatAbiMask) != 0x2) {
		std::fclose(file);
		fail(error, errorSize, "%s was built for a different float ABI (e_flags 0x%x)", path, static_cast<unsigned>(header.flags));
		return nullptr;
	}
#endif
	if (header.phentsize != sizeof(ProgramHeader) || header.phnum == 0 || header.phnum > 32) {
		std::fclose(file);
		fail(error, errorSize, "%s has malformed program headers", path);
		return nullptr;
	}
	ProgramHeader programs[32];
	if (!readAt(source, header.phoff, programs, sizeof(ProgramHeader) * header.phnum)) {
		std::fclose(file);
		fail(error, errorSize, "cannot read program headers of %s", path);
		return nullptr;
	}

	std::uint32_t span = 0, codeStart = 0, codeEnd = 0;
	const ProgramHeader *dynamic = nullptr;
	const ProgramHeader *ehFrameHeader = nullptr;
	for (int i = 0; i < header.phnum; ++i) {
		const ProgramHeader &program = programs[i];
		if (program.type == kPtLoad) {
			if (program.vaddr + program.memsz > span) span = program.vaddr + program.memsz;
			if (program.flags & kPfExecute) {
				if (codeEnd) {
					std::fclose(file);
					fail(error, errorSize, "%s has more than one executable segment", path);
					return nullptr;
				}
				codeStart = program.vaddr;
				codeEnd = program.vaddr + program.memsz;
			}
		} else if (program.type == kPtDynamic) {
			dynamic = &program;
		} else if (program.type == kPtGnuEhFrame) {
			ehFrameHeader = &program;
		}
	}
	if (span == 0 || !dynamic) {
		std::fclose(file);
		fail(error, errorSize, "%s has no loadable image or no dynamic section", path);
		return nullptr;
	}
	if (GEA_GEAPP_MAX_PROGRAM_BYTES && span > GEA_GEAPP_MAX_PROGRAM_BYTES) {
		std::fclose(file);
		fail(error, errorSize, "the app needs %u KB of memory; this device runs apps up to %u KB",
		     static_cast<unsigned>((span + 1023) / 1024), static_cast<unsigned>((GEA_GEAPP_MAX_PROGRAM_BYTES + 1023) / 1024));
		return nullptr;
	}

	auto *module = new (std::nothrow) Module();
	if (!module) {
		std::fclose(file);
		fail(error, errorSize, "out of memory");
		return nullptr;
	}
	module->codeStart = codeStart;
	module->codeEnd = codeEnd;
	// Whole instruction-cache lines, so the invalidate below covers the image exactly.
	constexpr std::size_t kLine = 128;
	module->size = (span + kLine - 1) & ~(kLine - 1);
	module->image = static_cast<std::uint8_t *>(heap_caps_aligned_calloc(kLine, 1, module->size, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT));
	if (!module->image) {
		std::fclose(file);
		fail(error, errorSize, "no PSRAM for a %u-byte image", static_cast<unsigned>(module->size));
		destroy(module);
		return nullptr;
	}
	for (int i = 0; i < header.phnum; ++i) {
		const ProgramHeader &program = programs[i];
		if (program.type != kPtLoad || program.filesz == 0) continue;
		if (!readAt(source, program.offset, module->image + program.vaddr, program.filesz)) {
			std::fclose(file);
			fail(error, errorSize, "cannot read segment at 0x%x of %s", static_cast<unsigned>(program.offset), path);
			destroy(module);
			return nullptr;
		}
	}
	std::fclose(file);

	std::uint32_t symbolTable = 0, stringTable = 0, hashTable = 0;
	std::uint32_t rela = 0, relaSize = 0, jmpRel = 0, pltRelSize = 0;
	const auto *entries = reinterpret_cast<const Dynamic *>(module->image + dynamic->vaddr);
	for (std::size_t i = 0; i * sizeof(Dynamic) < dynamic->memsz && entries[i].tag != kDtNull; ++i) {
		const Dynamic &entry = entries[i];
		switch (entry.tag) {
		case kDtNeeded:
			fail(error, errorSize, "%s needs another shared object; programs link only against the firmware", path);
			destroy(module);
			return nullptr;
		case kDtTextRel:
			fail(error, errorSize, "%s has text relocations; build it with -fPIC", path);
			destroy(module);
			return nullptr;
		case kDtSymTab: symbolTable = entry.value; break;
		case kDtStrTab: stringTable = entry.value; break;
		case kDtHash: hashTable = entry.value; break;
		case kDtRela: rela = entry.value; break;
		case kDtRelaSz: relaSize = entry.value; break;
		case kDtJmpRel: jmpRel = entry.value; break;
		case kDtPltRelSz: pltRelSize = entry.value; break;
		case kDtInitArray: module->initArray = entry.value; break;
		case kDtInitArraySz: module->initArraySize = entry.value; break;
		case kDtFiniArray: module->finiArray = entry.value; break;
		case kDtFiniArraySz: module->finiArraySize = entry.value; break;
		default: break;
		}
	}
	if (!symbolTable || !stringTable || !hashTable) {
		fail(error, errorSize, "%s has no dynamic symbol table (link it with --hash-style=sysv)", path);
		destroy(module);
		return nullptr;
	}
	module->symbols = reinterpret_cast<const Symbol *>(module->image + symbolTable);
	module->strings = reinterpret_cast<const char *>(module->image + stringTable);
	// DT_HASH: nbucket, nchain; nchain is the symbol count.
	module->symbolCount = reinterpret_cast<const std::uint32_t *>(module->image + hashTable)[1];

	if ((relaSize && !relocate(*module, rela, relaSize, error, errorSize)) ||
	    (pltRelSize && !relocate(*module, jmpRel, pltRelSize, error, errorSize))) {
		destroy(module);
		return nullptr;
	}

	if (ehFrameHeader) {
		module->ehFrame = ehFrameFromHeader(module->image + ehFrameHeader->vaddr);
		// Frame descriptions name code PC-relatively from the data bus; the
		// unwinder compares them with instruction-bus PCs.
		if (module->ehFrame && kCodeOffset) shiftFrameCode(static_cast<const std::uint8_t *>(module->ehFrame), module->image + module->size);
	}

	// The image was written through the data cache; fetch must see it.
	esp_cache_msync(module->image, module->size, ESP_CACHE_MSYNC_FLAG_DIR_C2M | ESP_CACHE_MSYNC_FLAG_UNALIGNED);
#if GEAPP_XTENSA
	// The instruction cache knows these pages by their instruction-bus
	// addresses, which may hold lines of an earlier program loaded here.
	Cache_Invalidate_ICache_All();
	asm volatile("isync" ::: "memory");
#else
	esp_cache_msync(module->image, module->size, ESP_CACHE_MSYNC_FLAG_DIR_M2C | ESP_CACHE_MSYNC_FLAG_TYPE_INST);
	asm volatile("fence.i" ::: "memory");
#endif

	if (ehFrameHeader) {
		if (module->ehFrame) {
			__register_frame_info(module->ehFrame, &module->frameObject);
			module->framesRegistered = true;
		} else {
			ESP_LOGW(kTag, "%s: unrecognised .eh_frame_hdr encoding; exceptions will not unwind through it", path);
		}
	}

	ESP_LOGI(kTag, "mapped %s at %p (%u bytes, %u symbols)", path, module->image,
		static_cast<unsigned>(module->size), static_cast<unsigned>(module->symbolCount));
	return module;
}

void construct(Module *module)
{
	// map() may have run on the other core: this hart's pipeline must not hold
	// anything it fetched from these addresses before the image was written.
#if GEAPP_XTENSA
	asm volatile("isync" ::: "memory");
#else
	asm volatile("fence.i" ::: "memory");
#endif
#if CONFIG_HEAP_USE_HOOKS
	// The record lives in PSRAM: 8 bytes a block.
	if (!module->blockSlots) {
		module->blockSlots = static_cast<gea::geaos::OwnedSet::Slot *>(
			heap_caps_calloc(kBlockSlots, sizeof(gea::geaos::OwnedSet::Slot), MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT));
		if (module->blockSlots) module->blocks.init(module->blockSlots, kBlockSlots);
		else ESP_LOGW(kTag, "no room for the heap record: this program's blocks are not reclaimed");
	}
	if (module->blockSlots) trackBlocks(module, true);
#endif
	g_constructing = module;
	const auto *constructors = reinterpret_cast<void (*const *)()>(module->image + module->initArray);
	for (std::uint32_t i = 0; i < module->initArraySize / sizeof(void *); ++i)
		if (constructors[i]) constructors[i]();
	g_constructing = nullptr;
	g_modules.push_back(module);
}

void discard(Module *module)
{
	destroy(module);
}

Module *load(const char *path, char *error, std::size_t errorSize)
{
	Module *module = map(path, error, errorSize);
	if (module) construct(module);
	return module;
}

void *symbol(const Module *module, const char *name)
{
	if (!module) return nullptr;
	for (std::uint32_t i = 1; i < module->symbolCount; ++i) {
		const Symbol &entry = module->symbols[i];
		if (entry.shndx == kShnUndef) continue;
		if (std::strcmp(module->strings + entry.name, name) == 0) return reinterpret_cast<void *>(addressAt(*module, entry.value));
	}
	return nullptr;
}

namespace {
ReleaseReport g_report;
ReleaseReport g_pending;  // filled by finalize(), published by release()
}  // namespace

const ReleaseReport &lastRelease() { return g_report; }

void finalize(Module *module)
{
	if (!module) return;
	g_pending = ReleaseReport{};
	g_pending.tasks = static_cast<unsigned>(module->tasks.size());
	// Its tasks go first: none may run the program's code while its statics
	// are destroyed, nor after. Its queues outlive the destructors, which may
	// still signal or take them.
	for (const OwnedTask &task : module->tasks) {
		ESP_LOGI(kTag, "deleting program task %s", pcTaskGetName(task.handle));
		if (task.withCaps) vTaskDeleteWithCaps(task.handle);
		else vTaskDelete(task.handle);
	}
	module->tasks.clear();
	// Its interrupts and timers next: each would call into code that is
	// about to be unmapped.
	{
		OwnGuard guard;
		g_pending.isrs = static_cast<unsigned>(module->isrPins.size());
		g_pending.espTimers = static_cast<unsigned>(module->espTimers.size());
		g_pending.rtosTimers = static_cast<unsigned>(module->rtosTimers.size());
		for (int pin : module->isrPins) gpio_isr_handler_remove(static_cast<gpio_num_t>(pin));
		if (!module->isrPins.empty()) ESP_LOGI(kTag, "removed %u program GPIO interrupt(s)", static_cast<unsigned>(module->isrPins.size()));
		module->isrPins.clear();
		for (esp_timer_handle_t timer : module->espTimers) {
			esp_timer_stop(timer);
			esp_timer_delete(timer);
		}
		if (!module->espTimers.empty()) ESP_LOGI(kTag, "deleted %u program esp_timer(s)", static_cast<unsigned>(module->espTimers.size()));
		module->espTimers.clear();
		for (TimerHandle_t timer : module->rtosTimers) xTimerDelete(timer, portMAX_DELAY);
		if (!module->rtosTimers.empty()) {
			ESP_LOGI(kTag, "deleted %u program FreeRTOS timer(s)", static_cast<unsigned>(module->rtosTimers.size()));
			drainTimerCommands();
		}
		module->rtosTimers.clear();
	}
	for (auto it = module->atExit.rbegin(); it != module->atExit.rend(); ++it) it->destructor(it->object);
	module->atExit.clear();
	const auto *destructors = reinterpret_cast<void (*const *)()>(module->image + module->finiArray);
	for (std::uint32_t i = module->finiArraySize / sizeof(void *); i > 0; --i)
		if (destructors[i - 1]) destructors[i - 1]();
	module->finiArraySize = 0;
	g_pending.queues = static_cast<unsigned>(module->queues.size());
	for (QueueHandle_t queue : module->queues) vQueueDelete(queue);
	module->queues.clear();
	// Files, directories and sockets the destructors did not close.
	OwnGuard guard;
	const unsigned files = static_cast<unsigned>(module->files.size() + module->dirs.size() + module->fds.size());
	g_pending.files = files;
	for (FILE *file : module->files) std::fclose(file);
	module->files.clear();
	for (DIR *dir : module->dirs) ::closedir(dir);
	module->dirs.clear();
	for (int fd : module->fds) ::close(fd);
	module->fds.clear();
	if (files) ESP_LOGI(kTag, "closed %u file(s), director(ies) or socket(s) the program left open", files);
}

namespace {

// Frees what the program allocated and nothing freed: after the runtime let
// go of the program's objects and its destructors ran, a block still on the
// record is one nobody can reach any more.
void reclaimBlocks(Module *module)
{
#if CONFIG_HEAP_USE_HOOKS
	if (!module->blockSlots) return;
	trackBlocks(module, false);
	// The free hook no longer touches this record: free from it freely.
	const std::size_t count = module->blocks.count();
	const std::uint64_t bytes = module->blocks.total();
	if (module->blocks.overflowed()) ESP_LOGW(kTag, "the heap record overflowed: some program blocks were not recorded");
	std::uintptr_t key = 0;
	std::uint32_t size = 0;
	while (module->blocks.takeAny(key, size)) heap_caps_free(reinterpret_cast<void *>(key));
	g_pending.blocks = static_cast<unsigned>(count);
	g_pending.blockBytes = bytes;
	g_pending.recordOverflowed = module->blocks.overflowed();
	if (count) ESP_LOGI(kTag, "reclaimed %u heap block(s), %u bytes, the program left allocated", static_cast<unsigned>(count), static_cast<unsigned>(bytes));
#else
	(void)module;
#endif
}

}  // namespace

void release(Module *module)
{
	if (!module) return;
	for (auto it = g_modules.begin(); it != g_modules.end(); ++it) {
		if (*it == module) {
			g_modules.erase(it);
			break;
		}
	}
	reclaimBlocks(module);
	g_pending.releases = g_report.releases + 1;
	g_report = g_pending;
	g_pending = ReleaseReport{};
	ESP_LOGI(kTag, "released image at %p (%u bytes)", module->image, static_cast<unsigned>(module->size));
	destroy(module);
}

void unload(Module *module)
{
	finalize(module);
	release(module);
}

std::uintptr_t firmwareSymbol(const char *name)
{
	return findExport(name);
}

bool contains(const Module *module, const void *address)
{
	if (!module || !module->image) return false;
	const auto *byte = static_cast<const std::uint8_t *>(address);
	if (byte >= module->image && byte < module->image + module->size) return true;
	// Code addresses (a return address, a callback) are on the instruction bus.
	const std::uintptr_t code = reinterpret_cast<std::uintptr_t>(address) - kCodeOffset;
	return kCodeOffset && code >= reinterpret_cast<std::uintptr_t>(module->image) + module->codeStart &&
	       code < reinterpret_cast<std::uintptr_t>(module->image) + module->codeEnd;
}

std::uintptr_t base(const Module *module)
{
	return module ? reinterpret_cast<std::uintptr_t>(module->image) : 0;
}

std::size_t imageSize(const Module *module)
{
	return module ? module->size : 0;
}

std::uintptr_t codeOffset()
{
	return kCodeOffset;
}

}  // namespace gea::platform::geapp
