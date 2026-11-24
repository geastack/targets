#include "apps/app_manager.h"
#include "host/display_orientation.h"
#include "lights.h"
#include "runtime.h"
#include "services/device_control.h"
#ifndef GEA_EMBEDDED_BLE_DISABLED
#include "connectivity/ble_hid.h"
#endif
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

namespace gea::platform::elecrow::buttons { void startButtonsTask(); }
namespace gea::targets::esp32::wifi { void registerDriver(); }

extern "C" void app_main()
{
  gea::platform::elecrow::lights::enableBoardPower();
  gea::platform::elecrow::buttons::startButtonsTask();
  (void)gea::platform::esp32::apps::launcherPlatform();
#ifndef GEA_EMBEDDED_WIFI_DISABLED
  gea::targets::esp32::wifi::registerDriver();
#endif
#ifndef GEA_EMBEDDED_BLE_DISABLED
  gea::targets::esp32::ble::registerHidDriver();
#endif
  vTaskPrioritySet(nullptr, 5);
  gea::platform::esp32::services::startDeviceControlTask();

  using gea::framework::display::detail::DisplayOrientationState;
  gea::framework::RuntimeOptions options{};
  options.width = DisplayOrientationState::width();
  options.height = DisplayOrientationState::height();
  gea::framework::Runtime::run(options);
}
