#pragma once

#include "chip_bindings/displays/qspi_panel.h"

namespace gea::platform::esp32::chip_bindings::co5300
{

  using PanelConfig = displays::QspiPanelConfig;

  class Panel final : public displays::QspiPanel
  {
  public:
    static Panel &instance();

    bool initialized() const override;
    esp_err_t init(const PanelConfig &config) override;
    esp_err_t setDisplayOn(bool enabled) override;
    esp_err_t setBrightness(int brightnessPercent) override;
    esp_err_t setHighBrightnessMode(bool enabled) override;
    esp_err_t drawBitmap(int x0, int y0, int x1Exclusive, int y1Exclusive, const void *pixels) override;
    esp_err_t setWindow(int x0, int y0, int x1, int y1) override;
    esp_err_t txColor(int command, const void *color, std::size_t colorSize) override;
    esp_err_t beginColorStream(int command) override;
    esp_err_t queueColorStreamData(const void *color, std::size_t colorSize, bool keepCsActiveAfter, bool signalCompletion) override;
    void endColorStream() override;
    // Autonomous (no acquire_bus) variant — ISR-driven DMA so the wire streams
    // while the CPU rasters the next chunk. gea3d half-res present only; see
    // co5300_async_stream.inc. Data uses the same queueColorStreamData.
    esp_err_t beginColorStreamAsync(int command) override;
    void endColorStreamAsync() override;
    esp_err_t setTearScanline(int line) override;

  private:
    Panel() = default;

    esp_err_t txParam(int command, const void *param, std::size_t paramSize);

    void *panel_ = nullptr;
    void *panelIo_ = nullptr;
    bool colorStreamActive_ = false;
    // Persistent home for the async stream's opening RAMWR command word: the
    // ISR-driven DMA reads it after beginColorStreamAsync returns, so it can't
    // live on the stack. Alignment matches spi_transaction_t tx_buffer needs.
    alignas(4) int asyncCmdWord_ = 0;
  };

} // namespace gea::platform::esp32::chip_bindings::co5300
