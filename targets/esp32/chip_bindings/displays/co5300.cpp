#include "chip_bindings/displays/co5300.h"

#include "displays/co5300/co5300.h"

#include <cstddef>
#include <cstdint>
#include <cstring>

#include "driver/spi_master.h"
#include "esp_check.h"
#include "esp_err.h"
// IDF 6.0 no longer pulls FreeRTOS transitively through the esp_lcd/spi headers,
// so portMAX_DELAY (used in the SPI transaction calls below) needs an explicit
// include.
#include "freertos/FreeRTOS.h"
#include "esp_lcd_co5300.h"
#include "esp_lcd_panel_io_interface.h"
#include "esp_lcd_panel_commands.h"
#include "esp_lcd_panel_io.h"
#include "esp_lcd_panel_ops.h"
#include "esp_log.h"
#include "esp_memory_utils.h"

#ifndef GEA_EMBEDDED_CO5300_SPI_MANUAL_DMA_ALIGN_FLAG
#define GEA_EMBEDDED_CO5300_SPI_MANUAL_DMA_ALIGN_FLAG 0
#endif

#ifndef GEA_EMBEDDED_CO5300_SPI_DMA_ALIGN_BYTES
#define GEA_EMBEDDED_CO5300_SPI_DMA_ALIGN_BYTES 64
#endif

namespace gea::platform::esp32::chip_bindings::co5300
{

  namespace
  {

    constexpr const char *kTag = "co5300";

#if GEA_EMBEDDED_CO5300_WAVESHARE_175_INIT
    static const std::uint8_t kCmdFe20[] = {0x20};
    static const std::uint8_t kCmd19[] = {0x10};
    static const std::uint8_t kCmd1c[] = {0xA0};
    static const std::uint8_t kCmdFe00[] = {0x00};
    static const std::uint8_t kCmdC4[] = {0x80};
    static const std::uint8_t kCmd3a[] = {0x55};
    static const std::uint8_t kCmd35[] = {0x00};
    static const std::uint8_t kCmd53[] = {0x20};
    static const std::uint8_t kCmd51[] = {0xFF};
    static const std::uint8_t kCmd63[] = {0xFF};
    static const std::uint8_t kCmd2a[] = {0x00, 0x06, 0x01, 0xD7};
    static const std::uint8_t kCmd2b[] = {0x00, 0x00, 0x01, 0xD1};

    static const co5300_lcd_init_cmd_t kWaveshare175InitCommands[] = {
        {0xFE, kCmdFe20, sizeof(kCmdFe20), 0},
        {0x19, kCmd19, sizeof(kCmd19), 0},
        {0x1C, kCmd1c, sizeof(kCmd1c), 0},
        {0xFE, kCmdFe00, sizeof(kCmdFe00), 0},
        {0xC4, kCmdC4, sizeof(kCmdC4), 0},
        {0x3A, kCmd3a, sizeof(kCmd3a), 0},
        {0x35, kCmd35, sizeof(kCmd35), 0},
        {0x53, kCmd53, sizeof(kCmd53), 0},
        {0x51, kCmd51, sizeof(kCmd51), 0},
        {0x63, kCmd63, sizeof(kCmd63), 0},
        {0x2A, kCmd2a, sizeof(kCmd2a), 0},
        {0x2B, kCmd2b, sizeof(kCmd2b), 600},
        {0x11, nullptr, 0, 600},
        {0x29, nullptr, 0, 0},
    };
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
    // Open-code CO5300_PANEL_IO_QSPI_CONFIG: under IDF 6.0 esp_lcd_panel_io_spi_config_t's
    // cs_gpio_num/dc_gpio_num are gpio_num_t, but the component's macro hard-codes
    // `.dc_gpio_num = -1` (an int literal) which is an invalid int->gpio_num_t
    // conversion in C++. Build it by hand with GPIO_NUM_NC instead.
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

    {
      // The requested pclk is not what the SPI driver necessarily achieves —
      // log the negotiated clock so link-throughput math uses reality.
      auto *io = spiPanelIo(panelIo_);
      int actualFreqKhz = 0;
      if (spi_device_get_actual_freq(io->spiDev, &actualFreqKhz) == ESP_OK)
        ESP_LOGI(kTag, "SPI actual clock: %d kHz (requested %d), trans max %u bytes, queue %u",
                 actualFreqKhz, config.qspiPclkHz, static_cast<unsigned>(io->spiTransMaxBytes),
                 static_cast<unsigned>(io->queueSize));
    }

    co5300_vendor_config_t vendorConfig = {};
#if GEA_EMBEDDED_CO5300_WAVESHARE_175_INIT
    vendorConfig.init_cmds = kWaveshare175InitCommands;
    vendorConfig.init_cmds_size = sizeof(kWaveshare175InitCommands) / sizeof(kWaveshare175InitCommands[0]);
#endif
    vendorConfig.flags.use_qspi_interface = 1;

    esp_lcd_panel_dev_config_t panelConfig = {};
    panelConfig.reset_gpio_num = static_cast<gpio_num_t>(config.pinReset);
    panelConfig.rgb_ele_order = LCD_RGB_ELEMENT_ORDER_RGB;
    panelConfig.bits_per_pixel = 16;
    panelConfig.vendor_config = &vendorConfig;

    esp_lcd_panel_handle_t panelHandleValue = nullptr;
    ESP_RETURN_ON_ERROR(esp_lcd_new_panel_co5300(ioHandle, &panelConfig, &panelHandleValue), kTag, "create panel failed");
    panel_ = panelHandleValue;

    ESP_RETURN_ON_ERROR(esp_lcd_panel_reset(panelHandle(panel_)), kTag, "panel reset failed");
    ESP_RETURN_ON_ERROR(esp_lcd_panel_init(panelHandle(panel_)), kTag, "panel init failed");
    ESP_RETURN_ON_ERROR(esp_lcd_panel_set_gap(panelHandle(panel_), gea::chips::co5300::kPanelXGap, gea::chips::co5300::kPanelYGap),
                        kTag,
                        "panel gap failed");
    return ESP_OK;
  }

  esp_err_t Panel::setDisplayOn(bool enabled)
  {
    if (!panel_)
      return ESP_ERR_INVALID_STATE;
    return esp_lcd_panel_disp_on_off(panelHandle(panel_), enabled);
  }

  esp_err_t Panel::setBrightness(int brightnessPercent)
  {
    if (!panel_)
      return ESP_ERR_INVALID_STATE;
    if (brightnessPercent < 0)
      brightnessPercent = 0;
    if (brightnessPercent > 100)
      brightnessPercent = 100;
    return esp_lcd_panel_co5300_set_brightness(panelHandle(panel_), static_cast<std::uint8_t>(brightnessPercent));
  }

  // HBM is a SEPARATE command from the brightness registers, which is why
  // loading 0x63 alone changes nothing: 0x51 holds the normal-mode brightness,
  // 0x63 holds the value used WHILE HBM is active, and 0x66 (HBMEN, datasheet
  // 7.5.49) is the only thing that switches between them. Bit D[1] is the enable;
  // every other bit is reserved. Note that 0x53 (WRCTRLD) has no HBM bit at all
  // -- only BCTRL (D[5]) and DD (D[3]) -- so "write 0x53 with a bigger value" is
  // a dead end on this controller.
  //
  // The panel ships an Automatic Current Limit block (0x55/0x56) whose stated
  // purpose is to protect AMOLED lifetime, and the datasheet documents no duty
  // cycle for HBM, so treat this as a sunlight boost to switch on and off, not
  // as a brightness setting to leave on.
  esp_err_t Panel::setHighBrightnessMode(bool enabled)
  {
    if (!panelIo_)
      return ESP_ERR_INVALID_STATE;
    const std::uint8_t hbm[] = {static_cast<std::uint8_t>(enabled ? 0x02 : 0x00)};
    return txParam(0x66, hbm, sizeof(hbm));
  }

  esp_err_t Panel::drawBitmap(int x0, int y0, int x1Exclusive, int y1Exclusive, const void *pixels)
  {
    if (!panel_)
      return ESP_ERR_INVALID_STATE;
    return esp_lcd_panel_draw_bitmap(panelHandle(panel_), x0, y0, x1Exclusive, y1Exclusive, pixels);
  }

  esp_err_t Panel::setWindow(int x0, int y0, int x1, int y1)
  {
    const auto window = gea::chips::co5300::CommandSet::addressWindow(x0, y0, x1, y1);
    ESP_RETURN_ON_ERROR(txParam(LCD_CMD_CASET, window.columns.data(), window.columns.size()), kTag, "send CASET failed");
    ESP_RETURN_ON_ERROR(txParam(LCD_CMD_RASET, window.rows.data(), window.rows.size()), kTag, "send RASET failed");
    return ESP_OK;
  }

  esp_err_t Panel::txColor(int command, const void *color, std::size_t colorSize)
  {
    if (!panelIo_)
      return ESP_ERR_INVALID_STATE;
    const int lcdCommand = gea::chips::co5300::CommandSet::qspiColorCommand(command);
    return esp_lcd_panel_io_tx_color(panelIoHandle(panelIo_), lcdCommand, color, colorSize);
  }

  esp_err_t Panel::beginColorStream(int command)
  {
    if (!panelIo_ || colorStreamActive_)
      return ESP_ERR_INVALID_STATE;

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

    int lcdCommand = gea::chips::co5300::CommandSet::qspiColorCommand(command);
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
#if GEA_EMBEDDED_CO5300_SPI_MANUAL_DMA_ALIGN_FLAG
      if (esp_ptr_external_ram(bytes))
      {
        static bool warnedExternalDmaCapability = false;
        if (!warnedExternalDmaCapability)
        {
          ESP_LOGW(kTag,
                   "external LCD DMA source: ptr=%p len=%zu dma_capable=%d",
                   bytes,
                   chunkSize,
                   esp_ptr_dma_capable(bytes) ? 1 : 0);
          warnedExternalDmaCapability = true;
        }
        const auto address = reinterpret_cast<std::uintptr_t>(bytes);
        constexpr std::size_t kDmaAlignBytes = GEA_EMBEDDED_CO5300_SPI_DMA_ALIGN_BYTES;
        if (((address | chunkSize) & (kDmaAlignBytes - 1)) != 0)
        {
          ESP_LOGE(kTag,
                   "external LCD DMA chunk is not %zu-byte aligned: ptr=%p len=%zu",
                   kDmaAlignBytes,
                   bytes,
                   chunkSize);
          return ESP_ERR_INVALID_ARG;
        }
        // External PSRAM color buffers are already cache-synced by the display
        // backend. Force the SPI driver to reject, rather than silently bounce,
        // any external transaction that no longer satisfies DMA alignment.
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

// Autonomous (no-acquire) color-stream methods for the gea3d present. Kept in a
// separate file, #included here so it reaches the panel-IO internals above.
#include "chip_bindings/displays/co5300_async_stream.inc"

  esp_err_t Panel::txParam(int command, const void *param, std::size_t paramSize)
  {
    if (!panelIo_)
      return ESP_ERR_INVALID_STATE;
    return esp_lcd_panel_io_tx_param(panelIoHandle(panelIo_),
                                     gea::chips::co5300::CommandSet::qspiParameterCommand(command),
                                     param,
                                     paramSize);
  }

  esp_err_t Panel::setTearScanline(int line)
  {
    if (!panelIo_)
      return ESP_ERR_INVALID_STATE;
    if (line < 0)
      line = 0;
    // TEON mode 0 (TE at the programmed line) then set_tear_scanline (0x44). The
    // init already enables TE; re-asserting the mode here is idempotent and makes
    // the scanline take effect.
    const std::uint8_t teon[] = {0x00};
    ESP_RETURN_ON_ERROR(txParam(0x35, teon, sizeof(teon)), kTag, "TEON failed");
    const std::uint8_t scanline[] = {static_cast<std::uint8_t>((line >> 8) & 0xFF),
                                     static_cast<std::uint8_t>(line & 0xFF)};
    return txParam(0x44, scanline, sizeof(scanline));
  }

} // namespace gea::platform::esp32::chip_bindings::co5300

namespace gea::platform::esp32::chip_bindings::displays
{

  QspiPanel &qspiPanel()
  {
    return co5300::Panel::instance();
  }

  const char *qspiPanelDriverName()
  {
    return "CO5300";
  }

} // namespace gea::platform::esp32::chip_bindings::displays
