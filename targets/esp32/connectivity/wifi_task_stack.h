// The Wi-Fi driver task's stack size, which ESP-IDF does not let you configure.
//
// The closed-source Wi-Fi blob creates its own "wifi" task with a hardcoded
// 3584-byte stack. That is too small for WPA2 auth on this firmware -- it
// overflows in `scan_inter_channel_timeout_process` on the init->auth
// transition -- and no public API exposes the size: there is no `stack_size`
// field on `wifi_init_config_t` and no `CONFIG_ESP_WIFI_TASK_STACK_SIZE` in
// 5.5.x or 6.0.x. The blob does create its tasks through the function pointers
// in `wifi_init_config_t.osi_funcs`, so those can be wrapped.
//
// Every firmware that calls `esp_wifi_init()` needs this, not only the ones
// that use gea's own Wi-Fi service: an app may declare no network capability
// -- so `connectivity/wifi.cpp` is not compiled into its image -- and still
// drive `esp_wifi` itself from its native sources. Call this on the config you
// are about to hand to `esp_wifi_init`:
//
//     wifi_init_config_t config = WIFI_INIT_CONFIG_DEFAULT();
//     gea::targets::esp32::wifi::applyTaskStackOverride(&config);
//     esp_wifi_init(&config);
//
// Header-only on purpose. A .cpp would have to be registered by a component,
// and the only components that can see `esp_wifi.h` are the ones whose REQUIRES
// already name `esp_wifi` -- which is exactly the set that varies per board and
// per app capability. Inline, the adapter compiles only where it is used, and
// `--gc-sections` never sees a reference to the blob's `g_wifi_osi_funcs` in an
// image that does not call this.
//
// It is a no-op on targets where Wi-Fi runs on a co-processor (esp32p4 with
// esp_wifi_remote/esp_hosted): the blob internals do not exist there and the
// host task is not the auth task.
#pragma once

#include "esp_wifi.h"

#if CONFIG_ESP_WIFI_ENABLED && !CONFIG_ESP_HOST_WIFI_ENABLED
#include <cstring>

#include "esp_log.h"
#include "esp_private/wifi_os_adapter.h"
#endif

namespace gea::targets::esp32::wifi {

#if CONFIG_ESP_WIFI_ENABLED && !CONFIG_ESP_HOST_WIFI_ENABLED

namespace task_stack_detail {

// 8192: the stock 3584-byte stack overflowed during WPA2 auth, and 6144
// survived association but stopped servicing packets during a sustained OTA
// upload.
constexpr uint32_t kOverrideBytes = 8192;

// Function-local statics in an inline function are one object across the whole
// program, which is what makes the header-only form safe: the saved originals
// and the patched table are shared no matter how many callers include this.
struct Adapter {
	int32_t (*originalPinned)(void *, const char *, uint32_t, void *, uint32_t, void *,
	                          uint32_t){};
	int32_t (*original)(void *, const char *, uint32_t, void *, uint32_t, void *){};
	wifi_osi_funcs_t funcs{};
};

inline Adapter &adapter()
{
	static Adapter instance;
	return instance;
}

inline uint32_t bumped(const char *const name, const uint32_t stackDepth)
{
	if (name && std::strcmp(name, "wifi") == 0 && stackDepth < kOverrideBytes) {
		ESP_LOGI("wifi", "wifi task stack bumped: %u -> %u bytes",
		         static_cast<unsigned>(stackDepth), static_cast<unsigned>(kOverrideBytes));
		return kOverrideBytes;
	}
	return stackDepth;
}

inline int32_t createPinned(void *taskFunc, const char *name, uint32_t stackDepth, void *param,
                            uint32_t prio, void *taskHandle, uint32_t coreId)
{
	return adapter().originalPinned(taskFunc, name, bumped(name, stackDepth), param, prio,
	                                taskHandle, coreId);
}

inline int32_t create(void *taskFunc, const char *name, uint32_t stackDepth, void *param,
                      uint32_t prio, void *taskHandle)
{
	return adapter().original(taskFunc, name, bumped(name, stackDepth), param, prio, taskHandle);
}

}  // namespace task_stack_detail

inline void applyTaskStackOverride(wifi_init_config_t *const config)
{
	if (config == nullptr)
		return;
	task_stack_detail::Adapter &adapter = task_stack_detail::adapter();
	// Patch once. Copying an already-patched table onto itself would leave the
	// forwarding pointers calling themselves.
	if (adapter.originalPinned == nullptr) {
		adapter.funcs = g_wifi_osi_funcs;
		adapter.originalPinned = g_wifi_osi_funcs._task_create_pinned_to_core;
		adapter.original = g_wifi_osi_funcs._task_create;
		adapter.funcs._task_create_pinned_to_core = task_stack_detail::createPinned;
		adapter.funcs._task_create = task_stack_detail::create;
	}
	config->osi_funcs = &adapter.funcs;
}

#else

inline void applyTaskStackOverride(wifi_init_config_t *)
{
}

#endif  // CONFIG_ESP_WIFI_ENABLED && !CONFIG_ESP_HOST_WIFI_ENABLED

}  // namespace gea::targets::esp32::wifi
