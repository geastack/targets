#include "chip_bindings/displays/sh8601.h"

#include "displays/sh8601/sh8601.h"
#if GEA_EMBEDDED_RM690B0_PANEL
#include "displays/rm690b0/rm690b0.h"
#endif

#include <cstddef>
#include <cstdint>
#include <cstring>

#include "driver/spi_master.h"
#include "esp_check.h"
#include "esp_err.h"
#include "freertos/FreeRTOS.h"
#include "esp_lcd_panel_commands.h"
#include "esp_lcd_panel_io.h"
#include "esp_lcd_panel_io_interface.h"
#include "esp_lcd_panel_ops.h"
#if GEA_EMBEDDED_AXS15231B_PANEL
#include "driver/ledc.h"
#include "esp_lcd_axs15231b.h"
#else
#include "esp_lcd_sh8601.h"
#endif
#include "esp_log.h"
#include "esp_memory_utils.h"

#ifndef GEA_EMBEDDED_SH8601_SPI_MANUAL_DMA_ALIGN_FLAG
#define GEA_EMBEDDED_SH8601_SPI_MANUAL_DMA_ALIGN_FLAG 0
#endif

#ifndef GEA_EMBEDDED_SH8601_SPI_DMA_ALIGN_BYTES
#define GEA_EMBEDDED_SH8601_SPI_DMA_ALIGN_BYTES 64
#endif

namespace gea::platform::esp32::chip_bindings::sh8601
{

  namespace
  {

    constexpr const char *kTag =
#if GEA_EMBEDDED_AXS15231B_PANEL
        "axs15231b";
#else
        "sh8601";
#endif

#if GEA_EMBEDDED_AXS15231B_PANEL
    // Match the T-Display-S3 Long vendor sequence exactly. In particular, the
    // panel must leave sleep mode and select its AXS-specific RGB565 interface
    // format (0x05). The generic esp_lcd component writes 0x55 and our earlier
    // abbreviated override omitted SLPOUT; both leave controller timing/state
    // dependent on whatever survived reset, which is not safe for continuous
    // partial updates.
    constexpr std::uint8_t kInterfacePixelFormat = 0x05;
    constexpr std::uint8_t kControlDisplay = 0x28;
    constexpr std::uint8_t kPanelBrightness = 0x00;
    constexpr std::uint8_t kContrastOff = 0x00;
    const axs15231b_lcd_init_cmd_t kLilygoInitCommands[] = {
        {LCD_CMD_SLPOUT, nullptr, 0, 120},
        {0x13, nullptr, 0, 0},
        {0x20, nullptr, 0, 0},
        {LCD_CMD_COLMOD, &kInterfacePixelFormat, 1, 0},
        {0x29, nullptr, 0, 0},
        {0x53, &kControlDisplay, 1, 0},
        {0x51, &kPanelBrightness, 1, 0},
        {0x58, &kContrastOff, 1, 10},
    };

    constexpr ledc_mode_t kBacklightSpeedMode = LEDC_LOW_SPEED_MODE;
    constexpr ledc_timer_t kBacklightTimer = LEDC_TIMER_0;
    constexpr ledc_channel_t kBacklightChannel = LEDC_CHANNEL_0;

    esp_err_t configureBacklight()
    {
      ledc_timer_config_t timer = {};
      timer.speed_mode = kBacklightSpeedMode;
      timer.duty_resolution = LEDC_TIMER_8_BIT;
      timer.timer_num = kBacklightTimer;
      timer.freq_hz = 2000;
      timer.clk_cfg = LEDC_AUTO_CLK;
      ESP_RETURN_ON_ERROR(ledc_timer_config(&timer), kTag, "configure backlight timer failed");

      ledc_channel_config_t channel = {};
      channel.gpio_num = GEA_EMBEDDED_AXS15231B_BACKLIGHT_GPIO;
      channel.speed_mode = kBacklightSpeedMode;
      channel.channel = kBacklightChannel;
      channel.timer_sel = kBacklightTimer;
      channel.duty = 0;
      channel.hpoint = 0;
      return ledc_channel_config(&channel);
    }

    esp_err_t setBacklightDuty(int brightnessPercent)
    {
      const std::uint32_t duty = static_cast<std::uint32_t>((brightnessPercent * 255) / 100);
      ESP_RETURN_ON_ERROR(ledc_set_duty(kBacklightSpeedMode, kBacklightChannel, duty), kTag, "set backlight duty failed");
      return ledc_update_duty(kBacklightSpeedMode, kBacklightChannel);
    }
#elif GEA_EMBEDDED_RM690B0_PANEL
    // The RM69080 speaks the SH8601 command set and so rides this same driver,
    // but it does not come up on the SH8601 panel's sequence: it needs its
    // register page selected (0xFE) and two panel settings written before the
    // usual colour-mode / sleep-out / display-on run. Without them every
    // command still reports success -- QSPI has no read-back -- and the glass
    // stays dark. Taken from the vendor's own ESP-IDF example for this board.
    const sh8601_lcd_init_cmd_t kRm69080InitCommands[] = {
        {0xFE, (uint8_t[]){0x20}, 1, 0},   // select register page 0x20
        {0x26, (uint8_t[]){0x0A}, 1, 0},
        {0x24, (uint8_t[]){0x80}, 1, 0},
        {0xFE, (uint8_t[]){0x00}, 1, 0},   // back to the command page
        {0x3A, (uint8_t[]){0x55}, 1, 0},   // 16 bits per pixel
        {0xC2, (uint8_t[]){0x00}, 1, 10},
        {0x35, (uint8_t[]){0x00}, 0, 0},   // tearing-effect line on
        {0x51, (uint8_t[]){0x00}, 1, 10},  // dark while it wakes
        {0x11, (uint8_t[]){0x00}, 0, 80},  // sleep out
        {0x2A, (uint8_t[]){0x00, 0x10, 0x00, 0xD1}, 4, 0},
        {0x2B, (uint8_t[]){0x00, 0x00, 0x00, 0x57}, 4, 0},
        {0x29, (uint8_t[]){0x00}, 0, 10},  // display on
        {0x51, (uint8_t[]){0xFF}, 1, 0},   // full brightness
    };
#else
    const sh8601_lcd_init_cmd_t kWaveshareInitCommands[] = {
        {0x11, (uint8_t[]){0x00}, 0, 120},
        {0x44, (uint8_t[]){0x01, 0xD1}, 2, 0},
        {0x35, (uint8_t[]){0x00}, 1, 0},
        {0x53, (uint8_t[]){0x20}, 1, 10},
        {0x2A, (uint8_t[]){0x00, 0x00, 0x01, 0x6F}, 4, 0},
        {0x2B, (uint8_t[]){0x00, 0x00, 0x01, 0xBF}, 4, 0},
        {0x51, (uint8_t[]){0x00}, 1, 10},
        {0x29, (uint8_t[]){0x00}, 0, 10},
        {0x51, (uint8_t[]){0xFF}, 1, 0},
    };
#endif

    // Each panel carries its own visible-area offset; the RM69080's first
    // column sits 16 pixels in, which is why the vendor's CASET starts at 0x10.
#if GEA_EMBEDDED_RM690B0_PANEL
    constexpr int kPanelXGap = gea::chips::rm690b0::kPanelXGap;
    constexpr int kPanelYGap = gea::chips::rm690b0::kPanelYGap;
#else
    constexpr int kPanelXGap = gea::chips::sh8601::kPanelXGap;
    constexpr int kPanelYGap = gea::chips::sh8601::kPanelYGap;
#endif

    esp_lcd_panel_handle_t panelHandle(void *panel)
    {
      return static_cast<esp_lcd_panel_handle_t>(panel);
    }

    esp_lcd_panel_io_handle_t panelIoHandle(void *panelIo)
    {
      return static_cast<esp_lcd_panel_io_handle_t>(panelIo);
    }

    struct LcdSpiTransDescriptor
    {
      spi_transaction_t base;
      struct
      {
        unsigned int dc_gpio_level : 1;
        unsigned int en_trans_done_cb : 1;
      } flags;
    };

    struct EspLcdPanelIoSpiPrivate
    {
      esp_lcd_panel_io_t base;
      spi_device_handle_t spiDev;
      std::size_t spiTransMaxBytes;
      int dcGpioNum;
      esp_lcd_panel_io_color_trans_done_cb_t onColorTransDone;
      void *userCtx;
      std::size_t queueSize;
      std::size_t numTransInflight;
      int lcdCmdBits;
      int lcdParamBits;
      std::uint8_t csEnaPretrans;
      std::uint8_t csEnaPosttrans;
      struct
      {
        unsigned int dcCmdLevel : 1;
        unsigned int dcDataLevel : 1;
        unsigned int dcParamLevel : 1;
        unsigned int octalMode : 1;
        unsigned int quadMode : 1;
      } flags;
      LcdSpiTransDescriptor transPool[1];
    };

    static_assert(offsetof(LcdSpiTransDescriptor, base) == 0,
                  "LCD SPI transaction descriptor must begin with spi_transaction_t");
    static_assert(offsetof(EspLcdPanelIoSpiPrivate, base) == 0,
                  "ESP LCD SPI private IO must begin with esp_lcd_panel_io_t");

    EspLcdPanelIoSpiPrivate *spiPanelIo(void *panelIo)
    {
      return reinterpret_cast<EspLcdPanelIoSpiPrivate *>(panelIoHandle(panelIo));
    }

    LcdSpiTransDescriptor *transDescriptorAt(EspLcdPanelIoSpiPrivate *io, std::size_t index)
    {
      auto *pool = reinterpret_cast<LcdSpiTransDescriptor *>(
          reinterpret_cast<std::uint8_t *>(io) + offsetof(EspLcdPanelIoSpiPrivate, transPool));
      return &pool[index];
    }

    void prepareCommandBuffer(int &command, int commandBits)
    {
      if (commandBits <= 8)
        return;
      auto *bytes = reinterpret_cast<std::uint8_t *>(&command);
      const int byteCount = commandBits / 8;
      for (int left = 0, right = byteCount - 1; left < right; left++, right--)
      {
        const std::uint8_t tmp = bytes[left];
        bytes[left] = bytes[right];
        bytes[right] = tmp;
      }
    }

    esp_err_t drainInflight(EspLcdPanelIoSpiPrivate *io)
    {
      spi_transaction_t *spiTrans = nullptr;
      const std::size_t inflight = io->numTransInflight;
      for (std::size_t i = 0; i < inflight; i++)
      {
        const esp_err_t ret = spi_device_get_trans_result(io->spiDev, &spiTrans, portMAX_DELAY);
        if (ret != ESP_OK)
          return ret;
        io->numTransInflight--;
      }
      return ESP_OK;
    }

    LcdSpiTransDescriptor *nextQueuedDescriptor(EspLcdPanelIoSpiPrivate *io, esp_err_t &ret)
    {
      if (io->numTransInflight < io->queueSize)
      {
        ret = ESP_OK;
        return transDescriptorAt(io, io->numTransInflight);
      }

      spi_transaction_t *spiTrans = nullptr;
      ret = spi_device_get_trans_result(io->spiDev, &spiTrans, portMAX_DELAY);
      if (ret != ESP_OK)
        return nullptr;
      io->numTransInflight--;
      return reinterpret_cast<LcdSpiTransDescriptor *>(spiTrans);
    }

  } // namespace

  Panel &Panel::instance()
  {
    static Panel panel;
    return panel;
  }

  bool Panel::initialized() const
  {
    return panel_ && panelIo_;
  }

  esp_err_t Panel::init(const PanelConfig &config)
  {
    if (initialized())
      return ESP_OK;

    spi_bus_config_t busConfig = {};
    busConfig.data0_io_num = config.pinData0;
    busConfig.data1_io_num = config.pinData1;
    busConfig.sclk_io_num = config.pinPclk;
    busConfig.data2_io_num = config.pinData2;
    busConfig.data3_io_num = config.pinData3;
    busConfig.data4_io_num = -1;
    busConfig.data5_io_num = -1;
    busConfig.data6_io_num = -1;
    busConfig.data7_io_num = -1;
    busConfig.max_transfer_sz = config.maxTransferBytes;

    const esp_err_t busErr = spi_bus_initialize(config.spiHost, &busConfig, SPI_DMA_CH_AUTO);
    if (busErr != ESP_OK && busErr != ESP_ERR_INVALID_STATE)
    {
      ESP_LOGE(kTag, "SPI bus init failed: %s", esp_err_to_name(busErr));
      return busErr;
    }

    esp_lcd_panel_io_handle_t ioHandle = nullptr;
    esp_lcd_panel_io_spi_config_t ioConfig = {};
    ioConfig.cs_gpio_num = static_cast<gpio_num_t>(config.pinCs);
    ioConfig.dc_gpio_num = GPIO_NUM_NC;
    ioConfig.spi_mode = 0;
    ioConfig.pclk_hz = config.qspiPclkHz;
    ioConfig.trans_queue_depth = config.transactionQueueDepth;
    ioConfig.on_color_trans_done = config.onColorTransferDone;
    ioConfig.user_ctx = config.callbackContext;
    ioConfig.lcd_cmd_bits = 32;
    ioConfig.lcd_param_bits = 8;
    ioConfig.flags.quad_mode = true;
    ESP_RETURN_ON_ERROR(esp_lcd_new_panel_io_spi(config.spiHost, &ioConfig, &ioHandle), kTag, "create panel io failed");
    panelIo_ = ioHandle;

#if GEA_EMBEDDED_AXS15231B_PANEL
    ESP_RETURN_ON_ERROR(configureBacklight(), kTag, "backlight setup failed");
    axs15231b_vendor_config_t vendorConfig = {};
    vendorConfig.init_cmds = kLilygoInitCommands;
    vendorConfig.init_cmds_size = sizeof(kLilygoInitCommands) / sizeof(kLilygoInitCommands[0]);
#elif GEA_EMBEDDED_RM690B0_PANEL
    sh8601_vendor_config_t vendorConfig = {};
    vendorConfig.init_cmds = kRm69080InitCommands;
    vendorConfig.init_cmds_size = sizeof(kRm69080InitCommands) / sizeof(kRm69080InitCommands[0]);
#else
    sh8601_vendor_config_t vendorConfig = {};
    vendorConfig.init_cmds = kWaveshareInitCommands;
    vendorConfig.init_cmds_size = sizeof(kWaveshareInitCommands) / sizeof(kWaveshareInitCommands[0]);
#endif
    vendorConfig.flags.use_qspi_interface = 1;

    esp_lcd_panel_dev_config_t panelConfig = {};
    panelConfig.reset_gpio_num = config.pinReset >= 0 ? static_cast<gpio_num_t>(config.pinReset) : GPIO_NUM_NC;
    panelConfig.rgb_ele_order = LCD_RGB_ELEMENT_ORDER_RGB;
    panelConfig.bits_per_pixel = 16;
    panelConfig.vendor_config = &vendorConfig;

    esp_lcd_panel_handle_t panelHandleValue = nullptr;
#if GEA_EMBEDDED_AXS15231B_PANEL
    ESP_RETURN_ON_ERROR(esp_lcd_new_panel_axs15231b(ioHandle, &panelConfig, &panelHandleValue), kTag, "create panel failed");
#else
    ESP_RETURN_ON_ERROR(esp_lcd_new_panel_sh8601(ioHandle, &panelConfig, &panelHandleValue), kTag, "create panel failed");
#endif
    panel_ = panelHandleValue;

    ESP_RETURN_ON_ERROR(esp_lcd_panel_reset(panelHandle(panel_)), kTag, "panel reset failed");
    ESP_RETURN_ON_ERROR(esp_lcd_panel_init(panelHandle(panel_)), kTag, "panel init failed");
    ESP_RETURN_ON_ERROR(esp_lcd_panel_set_gap(panelHandle(panel_), kPanelXGap, kPanelYGap),
                        kTag,
                        "panel gap failed");
#if GEA_EMBEDDED_AXS15231B_PANEL
    // esp_lcd_axs15231b 2.1.0 wires its legacy `disp_off(bool off)`
    // implementation into IDF 6's `disp_on_off(bool on)` slot without
    // adapting the polarity. Calling esp_lcd_panel_disp_on_off(..., true)
    // therefore sends DISPOFF. Use the panel's public DCS command directly.
    ESP_RETURN_ON_ERROR(txParam(LCD_CMD_DISPON, nullptr, 0), kTag, "panel display on failed");
#else
    ESP_RETURN_ON_ERROR(esp_lcd_panel_disp_on_off(panelHandle(panel_), true), kTag, "panel display on failed");
#endif
#if GEA_EMBEDDED_AXS15231B_PANEL
    ESP_RETURN_ON_ERROR(setBacklightDuty(100), kTag, "enable backlight failed");
#endif
    return ESP_OK;
  }

  esp_err_t Panel::setDisplayOn(bool enabled)
  {
    if (!panel_)
      return ESP_ERR_INVALID_STATE;
#if GEA_EMBEDDED_AXS15231B_PANEL
    return txParam(enabled ? LCD_CMD_DISPON : LCD_CMD_DISPOFF, nullptr, 0);
#else
    return esp_lcd_panel_disp_on_off(panelHandle(panel_), enabled);
#endif
  }

  esp_err_t Panel::setBrightness(int brightnessPercent)
  {
    if (!panelIo_)
      return ESP_ERR_INVALID_STATE;
    if (brightnessPercent < 0)
      brightnessPercent = 0;
    if (brightnessPercent > 100)
      brightnessPercent = 100;
#if GEA_EMBEDDED_AXS15231B_PANEL
    return setBacklightDuty(brightnessPercent);
#else
    const std::uint8_t brightness = static_cast<std::uint8_t>((brightnessPercent * 255) / 100);
    const int lcdCommand = gea::chips::sh8601::CommandSet::qspiParameterCommand(0x51);
    return esp_lcd_panel_io_tx_param(panelIoHandle(panelIo_), lcdCommand, &brightness, 1);
#endif
  }

  esp_err_t Panel::drawBitmap(int x0, int y0, int x1Exclusive, int y1Exclusive, const void *pixels)
  {
    if (!panel_)
      return ESP_ERR_INVALID_STATE;
#if GEA_EMBEDDED_AXS15231B_PANEL
    if (!pixels || x1Exclusive <= x0 || y1Exclusive <= y0)
      return ESP_ERR_INVALID_ARG;
    // The component's QSPI draw function omits RASET and is only correct for
    // its sequential full-frame path. Gea submits dirty rectangles, so send
    // both axes explicitly as LILYGO's reference driver does.
    ESP_RETURN_ON_ERROR(setWindow(x0, y0, x1Exclusive - 1, y1Exclusive - 1), kTag, "set bitmap window failed");
    const std::size_t colorSize = static_cast<std::size_t>(x1Exclusive - x0) *
                                  static_cast<std::size_t>(y1Exclusive - y0) * sizeof(std::uint16_t);
    return txColor(LCD_CMD_RAMWR, pixels, colorSize);
#else
    return esp_lcd_panel_draw_bitmap(panelHandle(panel_), x0, y0, x1Exclusive, y1Exclusive, pixels);
#endif
  }

  esp_err_t Panel::setWindow(int x0, int y0, int x1, int y1)
  {
    const auto window = gea::chips::sh8601::CommandSet::addressWindow(x0, y0, x1, y1);
    ESP_RETURN_ON_ERROR(txParam(LCD_CMD_CASET, window.columns.data(), window.columns.size()), kTag, "send CASET failed");
    ESP_RETURN_ON_ERROR(txParam(LCD_CMD_RASET, window.rows.data(), window.rows.size()), kTag, "send RASET failed");
    return ESP_OK;
  }

  esp_err_t Panel::txColor(int command, const void *color, std::size_t colorSize)
  {
    if (!panelIo_)
      return ESP_ERR_INVALID_STATE;
#if GEA_EMBEDDED_AXS15231B_PANEL
    // The AXS15231B QSPI protocol does not accept pixel payload directly on
    // the 0x32/0x2C color phase. LILYGO's reference bus first issues RAMWR as
    // a command transaction (0x02/0x2C), then transfers pixels through RAMWRC
    // (0x32/0x3C). Skipping that split works for an initial full-screen upload
    // but repeated partial windows intermittently lose their horizontal write
    // position.
    if (command == LCD_CMD_RAMWR)
    {
      ESP_RETURN_ON_ERROR(txParam(LCD_CMD_RAMWR, nullptr, 0), kTag, "prime AXS15231B RAM write failed");
      command = LCD_CMD_RAMWRC;
    }
#endif
    const int lcdCommand = gea::chips::sh8601::CommandSet::qspiColorCommand(command);
    return esp_lcd_panel_io_tx_color(panelIoHandle(panelIo_), lcdCommand, color, colorSize);
  }

  esp_err_t Panel::beginColorStream(int command)
  {
    if (!panelIo_ || colorStreamActive_)
      return ESP_ERR_INVALID_STATE;

#if GEA_EMBEDDED_AXS15231B_PANEL
    if (command == LCD_CMD_RAMWR)
    {
      ESP_RETURN_ON_ERROR(txParam(LCD_CMD_RAMWR, nullptr, 0), kTag, "prime AXS15231B color stream failed");
      command = LCD_CMD_RAMWRC;
    }
#endif

    auto *io = spiPanelIo(panelIo_);
    ESP_RETURN_ON_ERROR(spi_device_acquire_bus(io->spiDev, portMAX_DELAY), kTag, "acquire SPI bus failed");

    esp_err_t ret = drainInflight(io);
    if (ret != ESP_OK)
    {
      spi_device_release_bus(io->spiDev);
      return ret;
    }

    auto *lcdTrans = transDescriptorAt(io, 0);
    std::memset(lcdTrans, 0, sizeof(LcdSpiTransDescriptor));

    int lcdCommand = gea::chips::sh8601::CommandSet::qspiColorCommand(command);
    prepareCommandBuffer(lcdCommand, io->lcdCmdBits);
    lcdTrans->base.user = io;
    lcdTrans->flags.dc_gpio_level = io->flags.dcCmdLevel;
    lcdTrans->base.length = io->lcdCmdBits;
    lcdTrans->base.tx_buffer = &lcdCommand;
    lcdTrans->base.flags |= SPI_TRANS_CS_KEEP_ACTIVE;
    if (io->flags.octalMode)
    {
      lcdTrans->base.flags |= SPI_TRANS_MULTILINE_CMD | SPI_TRANS_MULTILINE_ADDR | SPI_TRANS_MODE_OCT;
    }

    ret = spi_device_polling_transmit(io->spiDev, &lcdTrans->base);
    if (ret != ESP_OK)
    {
      spi_device_release_bus(io->spiDev);
      return ret;
    }

    colorStreamActive_ = true;
    return ESP_OK;
  }

  esp_err_t Panel::queueColorStreamData(const void *color, std::size_t colorSize, bool keepCsActiveAfter, bool signalCompletion)
  {
    if (!panelIo_ || !colorStreamActive_)
      return ESP_ERR_INVALID_STATE;
    if (!color || colorSize == 0)
      return ESP_ERR_INVALID_ARG;

    auto *io = spiPanelIo(panelIo_);
    const auto *bytes = static_cast<const std::uint8_t *>(color);
    std::size_t remaining = colorSize;

    while (remaining > 0)
    {
      std::size_t chunkSize = remaining;
      if (chunkSize > io->spiTransMaxBytes)
        chunkSize = io->spiTransMaxBytes;
      const bool isLastChunk = chunkSize == remaining;

      esp_err_t ret = ESP_OK;
      auto *lcdTrans = nextQueuedDescriptor(io, ret);
      if (ret != ESP_OK)
        return ret;
      if (!lcdTrans)
        return ESP_ERR_INVALID_STATE;
      std::memset(lcdTrans, 0, sizeof(LcdSpiTransDescriptor));

      lcdTrans->base.user = io;
      lcdTrans->flags.dc_gpio_level = io->flags.dcDataLevel;
      lcdTrans->flags.en_trans_done_cb = signalCompletion && isLastChunk;
      lcdTrans->base.length = chunkSize * 8;
      lcdTrans->base.tx_buffer = bytes;
#if GEA_EMBEDDED_SH8601_SPI_MANUAL_DMA_ALIGN_FLAG
      if (esp_ptr_external_ram(bytes))
      {
        const auto address = reinterpret_cast<std::uintptr_t>(bytes);
        constexpr std::size_t kDmaAlignBytes = GEA_EMBEDDED_SH8601_SPI_DMA_ALIGN_BYTES;
        if (((address | chunkSize) & (kDmaAlignBytes - 1)) != 0)
          return ESP_ERR_INVALID_ARG;
        lcdTrans->base.flags |= SPI_TRANS_DMA_BUFFER_ALIGN_MANUAL;
      }
#endif
      if (!isLastChunk || keepCsActiveAfter)
        lcdTrans->base.flags |= SPI_TRANS_CS_KEEP_ACTIVE;
      if (io->flags.octalMode)
      {
        lcdTrans->base.flags |= SPI_TRANS_MULTILINE_CMD | SPI_TRANS_MULTILINE_ADDR | SPI_TRANS_MODE_OCT;
      }
      else if (io->flags.quadMode)
      {
        lcdTrans->base.flags |= SPI_TRANS_MODE_QIO;
      }

      ret = spi_device_queue_trans(io->spiDev, &lcdTrans->base, portMAX_DELAY);
      if (ret != ESP_OK)
        return ret;
      io->numTransInflight++;

      bytes += chunkSize;
      remaining -= chunkSize;
    }

    return ESP_OK;
  }

  void Panel::endColorStream()
  {
    if (!panelIo_ || !colorStreamActive_)
      return;
    auto *io = spiPanelIo(panelIo_);
    spi_device_release_bus(io->spiDev);
    colorStreamActive_ = false;
  }

  esp_err_t Panel::txParam(int command, const void *param, std::size_t paramSize)
  {
    if (!panelIo_)
      return ESP_ERR_INVALID_STATE;
    return esp_lcd_panel_io_tx_param(panelIoHandle(panelIo_),
                                     gea::chips::sh8601::CommandSet::qspiParameterCommand(command),
                                     param,
                                     paramSize);
  }

} // namespace gea::platform::esp32::chip_bindings::sh8601

namespace gea::platform::esp32::chip_bindings::displays
{

  QspiPanel &qspiPanel()
  {
    return sh8601::Panel::instance();
  }

  const char *qspiPanelDriverName()
  {
#if GEA_EMBEDDED_AXS15231B_PANEL
    return "AXS15231B";
#else
    return "SH8601";
#endif
  }

} // namespace gea::platform::esp32::chip_bindings::displays
