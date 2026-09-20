#pragma once

#include <cstddef>

#include "driver/spi_master.h"
#include "esp_err.h"
#include "esp_lcd_types.h"

namespace gea::platform::esp32::chip_bindings::displays
{

  struct QspiPanelConfig
  {
    spi_host_device_t spiHost;
    int pinCs;
    int pinPclk;
    int pinData0;
    int pinData1;
    int pinData2;
    int pinData3;
    int pinReset;
    int qspiPclkHz;
    int maxTransferBytes;
    int transactionQueueDepth;
    esp_lcd_panel_io_color_trans_done_cb_t onColorTransferDone;
    void *callbackContext;
  };

  class QspiPanel
  {
  public:
    virtual ~QspiPanel() = default;

    virtual bool initialized() const = 0;
    virtual esp_err_t init(const QspiPanelConfig &config) = 0;
    virtual esp_err_t setDisplayOn(bool enabled) = 0;
    virtual esp_err_t setBrightness(int brightnessPercent) = 0;
    // High-brightness mode: a sunlight-readability boost that drives the panel
    // above its normal-mode ceiling. Not every controller has one, so the
    // default reports "unsupported" rather than being pure-virtual.
    virtual esp_err_t setHighBrightnessMode(bool /*enabled*/) { return ESP_ERR_NOT_SUPPORTED; }
    virtual esp_err_t drawBitmap(int x0, int y0, int x1Exclusive, int y1Exclusive, const void *pixels) = 0;
    virtual esp_err_t setWindow(int x0, int y0, int x1, int y1) = 0;
    virtual esp_err_t txColor(int command, const void *color, std::size_t colorSize) = 0;
    virtual esp_err_t beginColorStream(int command) = 0;
    virtual esp_err_t queueColorStreamData(const void *color, std::size_t colorSize, bool keepCsActiveAfter, bool signalCompletion) = 0;
    virtual void endColorStream() = 0;
    // Autonomous (no bus-acquire) color stream: ISR-driven DMA so the wire overlaps
    // CPU work on another core. Default = the acquired-bus stream, so a binding
    // without an override behaves exactly as before; co5300 overrides it.
    virtual esp_err_t beginColorStreamAsync(int command) { return beginColorStream(command); }
    virtual void endColorStreamAsync() { endColorStream(); }
    // Program the scanline at which the panel emits its TE (vsync) pulse. Firing
    // it a few lines before the frame end gives the host lead time to write the
    // next frame's top rows into GRAM before scanout wraps to them. Default
    // no-op for panels/bindings without tear-scanline support.
    virtual esp_err_t setTearScanline(int /*line*/) { return ESP_OK; }
  };

  // Each board target links exactly one chip binding (co5300, sh8601, …) that
  // provides these symbols. display.cpp talks only to this interface.
  QspiPanel &qspiPanel();
  const char *qspiPanelDriverName();

} // namespace gea::platform::esp32::chip_bindings::displays
