#pragma once

#include "chip_bindings/displays/qspi_panel.h"

namespace gea::platform::esp32::chip_bindings::sh8601
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
    esp_err_t drawBitmap(int x0, int y0, int x1Exclusive, int y1Exclusive, const void *pixels) override;
    esp_err_t setWindow(int x0, int y0, int x1, int y1) override;
    esp_err_t txColor(int command, const void *color, std::size_t colorSize) override;
    esp_err_t beginColorStream(int command) override;
    esp_err_t queueColorStreamData(const void *color, std::size_t colorSize, bool keepCsActiveAfter, bool signalCompletion) override;
    void endColorStream() override;

  private:
    Panel() = default;

    esp_err_t txParam(int command, const void *param, std::size_t paramSize);

    void *panel_ = nullptr;
    void *panelIo_ = nullptr;
    bool colorStreamActive_ = false;
  };

} // namespace gea::platform::esp32::chip_bindings::sh8601
