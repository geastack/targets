// ESP-Mosaico board bring-up: the VCC_3V3 rail and the USB console.
//
// Everything on the panel side -- the CO5300, its touch controller, the codec
// supply on V1.0 and the expansion slots' 3V3 -- hangs off VCC_3V3, a high-side
// switch whose enable is GPIO60, active low. Until that rail is open the panel
// does not answer, and a QSPI write reports success either way, so a board that
// skips this boots with every log line saying the display came up and a dark
// screen. The BSP soft-starts the rail by fading the enable through LEDC over
// 50 ms (the switch's gate follows the average of the pin), which bounds the
// inrush when the board runs from its 65 mAh cell; this does the same.
//
// GPIO57 is the power-off request to the board's SAM8108 power controller:
// open drain, asserted low. It must be released for the board to stay on.
//
// The USB-C port is the S31's high-speed OTG controller, not USB-Serial/JTAG
// (that sits on GPIO33/34, routed to the left expansion header), so there is no
// ROM console on it: logs reach the host through a TinyUSB CDC interface that
// reports the chip's MAC as its USB serial -- the same "AA:BB:CC:DD:EE:FF"
// spelling the S3 boards' USB-Serial/JTAG uses, so `--board` resolves a
// Mosaico by serial exactly as it does every other board. UART0 keeps the
// early boot output and remains the fallback when USB does not come up.
//
// That console is also how apps are installed: `gea flash` streams the image to
// the running app (GEADEV OTA, `usbAppUpdate: "geadev-ota"` in targets.json),
// which writes the next OTA slot and restarts into it. The OTG port has no reset
// lines and V1.2 ships with automatic download disabled, so the ROM downloader
// is for first flash and recovery only (BOOT held through a power cycle). A new
// app confirms itself after it has run for a few seconds; one that crashes
// first is rolled back by the bootloader, and the app that comes back reports
// the crash on the console.

#include "board.h"

#include <algorithm>
#include <cstddef>
#include <cstdint>
#include <cstdio>

#include "driver/gpio.h"
#include "driver/ledc.h"
#include "esp_attr.h"
#include "esp_check.h"
#include "esp_err.h"
#include "esp_heap_caps.h"
#include "esp_log.h"
#include "esp_mac.h"
#include "esp_ota_ops.h"
#include "esp_private/panic_internal.h"
#include "esp_private/startup_internal.h"
#include "esp_rom_sys.h"
#include "esp_system.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"
#include "hal/uart_hal.h"
#include "riscv/rvruntime-frames.h"
#include "soc/lp_system_reg.h"
#include "soc/soc.h"

#if GEA_BOARD_USB_CONSOLE
#include "tinyusb.h"
#include "tinyusb_cdc_acm.h"
#include "tinyusb_console.h"
#include "tinyusb_default_config.h"
#include "vfs_tinyusb.h"
#endif

namespace
{
  constexpr char kTag[] = "mosaico";

  constexpr int kVccOnLevel = 0;
  constexpr int kVccOffLevel = 1;
  constexpr int kVccRampMs = 50;
  // Settling after the ramp before anything talks to the panel. The BSP's
  // codec path waits 10 ms; the panel takes its own 600 ms sleep-out later.
  constexpr int kVccSettleMs = 10;

  constexpr ledc_mode_t kRampMode = LEDC_LOW_SPEED_MODE;
  constexpr ledc_timer_t kRampTimer = LEDC_TIMER_1;
  constexpr ledc_channel_t kRampChannel = LEDC_CHANNEL_1;
  // 8-bit keeps the carrier well above the gate RC pole (the BSP's choice).
  constexpr ledc_timer_bit_t kRampResolution = LEDC_TIMER_8_BIT;
  constexpr std::uint32_t kRampFrequencyHz = 100000;

  StaticSemaphore_t powerLockStorage;
  SemaphoreHandle_t powerLock = nullptr;
  bool railsOn = false;

  esp_err_t configureOutput(gpio_num_t pin, gpio_mode_t mode, int level)
  {
    gpio_config_t config = {};
    config.pin_bit_mask = 1ULL << pin;
    config.mode = mode;
    config.pull_up_en = GPIO_PULLUP_DISABLE;
    config.pull_down_en = GPIO_PULLDOWN_DISABLE;
    config.intr_type = GPIO_INTR_DISABLE;
    // Preset the level so the pad never glitches through the wrong state.
    ESP_RETURN_ON_ERROR(gpio_set_level(pin, level), kTag, "preset GPIO%d", pin);
    return gpio_config(&config);
  }

  esp_err_t rampVccOn()
  {
    constexpr gpio_num_t pin = gea::platform::board::power.vcc3v3Enable;
    ledc_timer_config_t timer = {};
    timer.speed_mode = kRampMode;
    timer.duty_resolution = kRampResolution;
    timer.timer_num = kRampTimer;
    timer.freq_hz = kRampFrequencyHz;
    timer.clk_cfg = LEDC_AUTO_CLK;
    ESP_RETURN_ON_ERROR(ledc_timer_config(&timer), kTag, "ramp timer");

    constexpr std::uint32_t maxDuty = (1U << kRampResolution) - 1U;
    ledc_channel_config_t channel = {};
    channel.gpio_num = pin;
    channel.speed_mode = kRampMode;
    channel.channel = kRampChannel;
    channel.timer_sel = kRampTimer;
    channel.duty = kVccOffLevel ? maxDuty : 0;
    channel.hpoint = 0;
    ESP_RETURN_ON_ERROR(ledc_channel_config(&channel), kTag, "ramp channel");

    const esp_err_t installed = ledc_fade_func_install(0);
    esp_err_t err = installed == ESP_ERR_INVALID_STATE ? ESP_OK : installed;
    if (err == ESP_OK)
      err = ledc_set_fade_with_time(kRampMode, kRampChannel, kVccOnLevel ? maxDuty : 0, kVccRampMs);
    if (err == ESP_OK)
      err = ledc_fade_start(kRampMode, kRampChannel, LEDC_FADE_WAIT_DONE);
    if (installed == ESP_OK)
      ledc_fade_func_uninstall();

    // Hand the pad back to plain GPIO at its final level; a failed ramp leaves
    // the rail off rather than half driven.
    const int level = err == ESP_OK ? kVccOnLevel : kVccOffLevel;
    ledc_stop(kRampMode, kRampChannel, level);
    const esp_err_t restored = configureOutput(pin, GPIO_MODE_OUTPUT, level);
    return err != ESP_OK ? err : restored;
  }

  void powerOnRailsLocked()
  {
    using gea::platform::board::power;
    // Keep the board on before anything else: release the power-off request.
    gpio_hold_dis(power.powerOffRequest);
    esp_err_t err = configureOutput(power.powerOffRequest, GPIO_MODE_OUTPUT_OD, 1);
    if (err != ESP_OK)
      ESP_LOGE(kTag, "PWR_SW GPIO%d release failed: %s", power.powerOffRequest, esp_err_to_name(err));

    gpio_hold_dis(power.vcc3v3Enable);
    err = rampVccOn();
    if (err != ESP_OK)
    {
      // Better a hard switch-on than a dark panel: the ramp only bounds inrush.
      ESP_LOGW(kTag, "VCC_3V3 soft start failed (%s); switching it on directly", esp_err_to_name(err));
      err = configureOutput(power.vcc3v3Enable, GPIO_MODE_OUTPUT, kVccOnLevel);
    }
    if (err != ESP_OK)
    {
      ESP_LOGE(kTag, "VCC_3V3 enable GPIO%d failed: %s", power.vcc3v3Enable, esp_err_to_name(err));
      return;
    }
    vTaskDelay(pdMS_TO_TICKS(kVccSettleMs));
    railsOn = true;
    ESP_LOGI(kTag, "VCC_3V3 on (GPIO%d ramped low over %d ms), PWR_SW GPIO%d released",
             power.vcc3v3Enable, kVccRampMs, power.powerOffRequest);
  }

#if GEA_BOARD_USB_CONSOLE
  char usbSerial[18] = "000000000000";
  const char *usbStrings[] = {
      "\x09\x04",            // 0: English (0x0409)
      "Espressif",           // 1: Manufacturer
      "ESP-Mosaico",         // 2: Product
      usbSerial,             // 3: Serial -- the chip MAC, filled in at start
      "ESP-Mosaico Console", // 4: CDC interface
  };

  void startUsbConsole()
  {
    std::uint8_t mac[6] = {};
    if (esp_read_mac(mac, ESP_MAC_BASE) == ESP_OK)
    {
      std::snprintf(usbSerial, sizeof(usbSerial), "%02X:%02X:%02X:%02X:%02X:%02X",
                    mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
    }

    tinyusb_config_t config = TINYUSB_DEFAULT_CONFIG();
    config.descriptor.string = usbStrings;
    config.descriptor.string_count = sizeof(usbStrings) / sizeof(usbStrings[0]);
    esp_err_t err = tinyusb_driver_install(&config);
    if (err != ESP_OK)
    {
      ESP_LOGE(kTag, "TinyUSB install failed: %s; console stays on UART0", esp_err_to_name(err));
      return;
    }

    tinyusb_config_cdcacm_t cdc = {};
    cdc.cdc_port = TINYUSB_CDC_ACM_0;
    err = tinyusb_cdcacm_init(&cdc);
    if (err != ESP_OK)
    {
      ESP_LOGE(kTag, "USB CDC init failed: %s; console stays on UART0", esp_err_to_name(err));
      return;
    }

    err = tinyusb_console_init(TINYUSB_CDC_ACM_0);
    if (err != ESP_OK)
    {
      ESP_LOGE(kTag, "USB console redirect failed: %s; console stays on UART0", esp_err_to_name(err));
      return;
    }
    // Pass console input through untouched. The libc default (CR -> LF) is for
    // a typed terminal; GEADEV's parser takes either line ending, and GEADEV
    // OTA/PUSH stream raw binary that the conversion would corrupt.
    esp_vfs_tusb_cdc_set_rx_line_endings(ESP_LINE_ENDINGS_LF);
    ESP_LOGI(kTag, "USB CDC console up (serial %s)", usbSerial);
  }
#endif

  // The panic dump only reaches UART0, which nothing on this board exposes, and
  // a crash on boot comes before the USB console. The panic handler is wrapped
  // (-Wl,--wrap=esp_panic_handler) to leave the crash registers in LP_STORE10-14,
  // and the dump itself plus esp_rom_printf output land in an LP RAM ring; both
  // survive the reset. The next app to boot -- after a rollback, the previous
  // one -- prints them (GEADEV:CRASH, GEADEV:CRASHLOG) when a host opens the
  // console.
  constexpr std::uint32_t kPanicMagic = 0x9EA0DEADU;
  constexpr std::uint32_t kCrashLogMagic = 0x9EA01060U;
  constexpr std::size_t kCrashLogBytes = 12 * 1024;
  RTC_NOINIT_ATTR std::uint32_t crashLogMagic;
  RTC_NOINIT_ATTR std::uint32_t crashLogHead;
  RTC_NOINIT_ATTR char crashLog[kCrashLogBytes];
  char *previousCrashLog = nullptr;
  std::size_t previousCrashLogBytes = 0;

  IRAM_ATTR void crashLogPutc(char c)
  {
    crashLog[crashLogHead % kCrashLogBytes] = c;
    crashLogHead = crashLogHead + 1;
  }

  bool previousBootCrashed()
  {
    const esp_reset_reason_t reason = esp_reset_reason();
    return REG_READ(LP_SYSTEM_REG_LP_STORE10_REG) == kPanicMagic || reason == ESP_RST_PANIC ||
           reason == ESP_RST_INT_WDT || reason == ESP_RST_TASK_WDT || reason == ESP_RST_WDT ||
           reason == ESP_RST_BROWNOUT;
  }

  __attribute__((constructor)) void startCrashLog()
  {
    if (crashLogMagic == kCrashLogMagic && previousBootCrashed())
    {
      const std::size_t bytes = crashLogHead < kCrashLogBytes ? crashLogHead : kCrashLogBytes;
      previousCrashLog = static_cast<char *>(heap_caps_malloc(bytes + 1, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT));
      if (previousCrashLog)
      {
        const std::size_t start = crashLogHead < kCrashLogBytes ? 0 : crashLogHead % kCrashLogBytes;
        for (std::size_t i = 0; i < bytes; ++i)
          previousCrashLog[i] = crashLog[(start + i) % kCrashLogBytes];
        previousCrashLog[bytes] = 0;
        previousCrashLogBytes = bytes;
      }
    }
    crashLogMagic = kCrashLogMagic;
    crashLogHead = 0;
    esp_rom_install_channel_putc(2, crashLogPutc);
  }

#if GEA_BOARD_USB_CONSOLE
  void reportPreviousPanic()
  {
    const bool captured = REG_READ(LP_SYSTEM_REG_LP_STORE10_REG) == kPanicMagic;
    const std::uint32_t mepc = REG_READ(LP_SYSTEM_REG_LP_STORE11_REG);
    const std::uint32_t ra = REG_READ(LP_SYSTEM_REG_LP_STORE12_REG);
    const std::uint32_t mcause = REG_READ(LP_SYSTEM_REG_LP_STORE13_REG);
    const std::uint32_t mtval = REG_READ(LP_SYSTEM_REG_LP_STORE14_REG);
    REG_WRITE(LP_SYSTEM_REG_LP_STORE10_REG, 0);
    // Shown each time a host opens the console, for three minutes.
    const TickType_t until = xTaskGetTickCount() + pdMS_TO_TICKS(180000);
    while (xTaskGetTickCount() < until)
    {
      if (!tud_cdc_n_connected(0))
      {
        vTaskDelay(pdMS_TO_TICKS(100));
        continue;
      }
      vTaskDelay(pdMS_TO_TICKS(1500));
      if (captured)
        std::printf("GEADEV:CRASH reset=%d mepc=0x%08lx ra=0x%08lx mcause=0x%08lx mtval=0x%08lx\n",
                    static_cast<int>(esp_reset_reason()), static_cast<unsigned long>(mepc),
                    static_cast<unsigned long>(ra), static_cast<unsigned long>(mcause),
                    static_cast<unsigned long>(mtval));
      else
        std::printf("GEADEV:CRASH reset=%d (no panic)\n", static_cast<int>(esp_reset_reason()));
      std::printf("GEADEV:CRASHLOG BEGIN bytes=%u\n", static_cast<unsigned>(previousCrashLogBytes));
      for (std::size_t off = 0; off < previousCrashLogBytes; off += 256)
      {
        std::fwrite(previousCrashLog + off, 1, std::min<std::size_t>(256, previousCrashLogBytes - off), stdout);
        std::fflush(stdout);
      }
      std::printf("\nGEADEV:CRASHLOG END\n");
      std::fflush(stdout);
      while (tud_cdc_n_connected(0) && xTaskGetTickCount() < until)
        vTaskDelay(pdMS_TO_TICKS(100));
    }
  }

  void panicReportTask(void *)
  {
    reportPreviousPanic();
    vTaskDelete(nullptr);
  }
#endif

#if CONFIG_BOOTLOADER_APP_ROLLBACK_CONFIRM_BY_APP
  // A new app (installed over USB into the other OTA slot) is confirmed once it
  // has run this long. One that crashes or hangs-and-resets first is rolled
  // back by the bootloader on the next boot, and the board stays reachable.
  constexpr std::uint32_t kConfirmAfterMs = 8000;

  void confirmRunningApp()
  {
    esp_ota_img_states_t state;
    if (esp_ota_get_state_partition(esp_ota_get_running_partition(), &state) != ESP_OK ||
        (state != ESP_OTA_IMG_PENDING_VERIFY && state != ESP_OTA_IMG_NEW))
      return;
    vTaskDelay(pdMS_TO_TICKS(kConfirmAfterMs));
    const esp_err_t err = esp_ota_mark_app_valid_cancel_rollback();
    if (err == ESP_OK)
      ESP_LOGI(kTag, "app confirmed after %u ms; rollback cancelled", static_cast<unsigned>(kConfirmAfterMs));
    else
      ESP_LOGE(kTag, "app confirm failed: %s", esp_err_to_name(err));
  }
#endif

  void boardStartTask(void *)
  {
    gea::platform::board::powerOnRails();
#if GEA_BOARD_USB_CONSOLE
    startUsbConsole();
    if (previousBootCrashed())
      xTaskCreatePinnedToCore(panicReportTask, "mosaico_panic", 3072, nullptr, 5, nullptr, 0);
#endif
#if CONFIG_BOOTLOADER_APP_ROLLBACK_CONFIRM_BY_APP
    confirmRunningApp();
#endif
    vTaskDelete(nullptr);
  }
}  // namespace

namespace gea::platform::board
{
  void powerOnRails()
  {
    // Either the start task or display init gets here first; the other waits
    // for the rail rather than talking to a panel that is still unpowered.
    xSemaphoreTake(powerLock, portMAX_DELAY);
    if (!railsOn)
      powerOnRailsLocked();
    xSemaphoreGive(powerLock);
  }

  // Display init calls this just before the panel takes its init sequence
  // (GEA_BOARD_PREPARE_DISPLAY_PANEL). Referencing it from display.cpp is also
  // what keeps this translation unit -- and the start hook below -- linked.
  void prepareDisplayPanel()
  {
    powerOnRails();
  }
}  // namespace gea::platform::board

// Runs before app_main, before the scheduler. It only creates the start task:
// the LEDC fade and TinyUSB both need a running scheduler.
ESP_SYSTEM_INIT_FN(gea_mosaico_board_start, SECONDARY, BIT(0), 999)
{
  powerLock = xSemaphoreCreateMutexStatic(&powerLockStorage);
  if (xTaskCreatePinnedToCore(boardStartTask, "mosaico_start", 4096, nullptr, 10, nullptr, 0) != pdPASS)
    ESP_EARLY_LOGW(kTag, "board start task not created; display init will open VCC_3V3");
  return ESP_OK;
}

extern "C" void __real_esp_panic_handler(panic_info_t *info);

extern "C" void __wrap_esp_panic_handler(panic_info_t *info)
{
  const auto *frame = info ? static_cast<const RvExcFrame *>(info->frame) : nullptr;
  REG_WRITE(LP_SYSTEM_REG_LP_STORE11_REG, frame ? static_cast<std::uint32_t>(frame->mepc) : 0);
  REG_WRITE(LP_SYSTEM_REG_LP_STORE12_REG, frame ? static_cast<std::uint32_t>(frame->ra) : 0);
  REG_WRITE(LP_SYSTEM_REG_LP_STORE13_REG, frame ? static_cast<std::uint32_t>(frame->mcause) : 0);
  REG_WRITE(LP_SYSTEM_REG_LP_STORE14_REG, frame ? static_cast<std::uint32_t>(frame->mtval) : 0);
  REG_WRITE(LP_SYSTEM_REG_LP_STORE10_REG, kPanicMagic);
  __real_esp_panic_handler(info);
}

#if GEA_BOARD_USB_CONSOLE
// The TinyUSB console drops whatever does not fit the TX FIFO, so under a log
// flood a GEADEV reply (OTA READY, PONG) is lost and `gea flash` times out.
// Wait for the host to drain the FIFO instead, while a host is reading; give up
// after 100 ms and stop waiting until the FIFO takes a byte again, so a host
// that holds the port open without reading cannot stall the app.
extern "C" size_t __real_tinyusb_cdcacm_write_queue_char(tinyusb_cdcacm_itf_t itf, char ch);

extern "C" size_t __wrap_tinyusb_cdcacm_write_queue_char(tinyusb_cdcacm_itf_t itf, char ch)
{
  static bool txStalled = false;
  size_t queued = __real_tinyusb_cdcacm_write_queue_char(itf, ch);
  if (queued || txStalled || xPortInIsrContext() || xTaskGetSchedulerState() != taskSCHEDULER_RUNNING ||
      !tud_cdc_n_connected(itf))
  {
    if (queued)
      txStalled = false;
    return queued;
  }
  const TickType_t start = xTaskGetTickCount();
  while (xTaskGetTickCount() - start < pdMS_TO_TICKS(100))
  {
    tud_cdc_n_write_flush(itf);
    vTaskDelay(1);
    queued = __real_tinyusb_cdcacm_write_queue_char(itf, ch);
    if (queued)
      return queued;
  }
  txStalled = true;
  return 0;
}
#endif

// The panic dump goes out through the UART HAL; keep a copy in the crash log.
extern "C" void __real_uart_hal_write_txfifo(uart_hal_context_t *hal, const uint8_t *buf, uint32_t size,
                                             uint32_t *written);

extern "C" IRAM_ATTR void __wrap_uart_hal_write_txfifo(uart_hal_context_t *hal, const uint8_t *buf,
                                                       uint32_t size, uint32_t *written)
{
  __real_uart_hal_write_txfifo(hal, buf, size, written);
  for (uint32_t i = 0; i < *written; ++i)
    crashLogPutc(static_cast<char>(buf[i]));
}
