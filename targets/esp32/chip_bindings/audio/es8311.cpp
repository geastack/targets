#define GEA_AUDIO_DRIVER_INTERNAL 1
#include "audio.h"
#include "board.h"
#include "audio/es8311/es8311.h"
#include "i2c.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <mutex>
#include <vector>

#include "driver/gpio.h"
#include "driver/i2c_master.h"
#include "driver/i2s_std.h"
#include "esp_codec_dev.h"
#include "esp_codec_dev_defaults.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "host/media.h"

// AFE (acoustic echo cancellation) integration deferred — esp-sr 2.4.4's API
// requires afe_config_init("MR", models, AFE_TYPE_VC, AFE_MODE_LOW_COST) and
// esp_afe_handle_from_config(). Phase 5.0 ships raw mic mix; AEC is a Phase 5.1
// follow-up once the basic call flow is on-device verified.

namespace gea::platform::esp32::chip_bindings::es8311 {

// IDF 6.0: i2s_port_t is gone; i2s_chan_config_t.id (and I2S_CHANNEL_DEFAULT_CONFIG) take a plain int.
constexpr int kI2sPort = gea::platform::board::audio.i2sPort;
static_assert(kI2sPort == I2S_NUM_AUTO || kI2sPort >= 0,
	"board audio.i2sPort must name a controller or I2S_NUM_AUTO");
constexpr gpio_num_t kMclkPin = gea::platform::board::audio.mclk;
constexpr gpio_num_t kBclkPin = gea::platform::board::audio.bclk;
constexpr gpio_num_t kWsPin = gea::platform::board::audio.ws;
constexpr gpio_num_t kDoutPin = gea::platform::board::audio.dout;
constexpr gpio_num_t kDinPin = gea::platform::board::audio.din;
constexpr gpio_num_t kPaPin = gea::platform::board::audio.powerAmplifier;
constexpr int kCodecDataPort = 0;

class AudioOutputDriver {
public:
  static AudioOutputDriver &instance() {
    // Lazily heap-allocated so an app with no audio reserves ZERO RAM for this
    // ~12 KB driver (it used to sit in .bss for every app, audio or not). The
    // storage is created on the first instance() call; never freed.
    static AudioOutputDriver *const driver = new AudioOutputDriver();
    return *driver;
  }

  bool open(int sampleRate, int channels, int bitsPerSample) {
    std::lock_guard<std::mutex> lock(writeMutex_);
    const gea::chips::es8311::OutputFormat format(sampleRate, channels, bitsPerSample);
    if (!format.isPcm16()) return false;

    if (speakerOpen_ &&
        speakerSampleRate_ == sampleRate &&
        speakerChannels_ == channels &&
        speakerBitsPerSample_ == bitsPerSample) {
      applySpeakerVolumeLocked();
      return true;
    }

    if (recordOpen_) closeRecorderLocked();
    if (speakerOpen_) closeSpeakerLocked();

    const esp_err_t initErr = initSpeaker(sampleRate);
    if (initErr != ESP_OK) {
      ESP_LOGE(kTag, "Speaker init failed: %s", esp_err_to_name(initErr));
      return false;
    }

    esp_codec_dev_sample_info_t sampleInfo = {};
    sampleInfo.bits_per_sample = static_cast<std::uint8_t>(bitsPerSample);
    sampleInfo.channel = static_cast<std::uint8_t>(channels);
    sampleInfo.sample_rate = static_cast<std::uint32_t>(sampleRate);
    sampleInfo.mclk_multiple = I2S_MCLK_MULTIPLE_256;
    const esp_err_t openErr = esp_codec_dev_open(speakerCodec_, &sampleInfo);
    if (openErr != ESP_OK) {
      ESP_LOGE(kTag, "Speaker open failed: %s", esp_err_to_name(openErr));
      return false;
    }

    applySpeakerVolumeLocked();
    speakerOpen_ = true;
    speakerSampleRate_ = sampleRate;
    speakerChannels_ = channels;
    speakerBitsPerSample_ = bitsPerSample;
    i2sSampleRate_ = sampleRate;
    ESP_LOGI(kTag,
             "Speaker ready sample_rate=%d channels=%d bits=%d volume=%d codec_volume=%d pa_gpio=%d",
             sampleRate,
             channels,
             bitsPerSample,
             speakerVolume_,
             codecVolumeFromUserPercent(speakerVolume_),
             static_cast<int>(kPaPin));
    return true;
  }

  bool write(const std::int16_t *pcm, std::size_t sampleCount, int timeoutMs) {
    (void)timeoutMs;
    if (!speakerCodec_ || !speakerOpen_ || !pcm || sampleCount == 0) return false;

    std::lock_guard<std::mutex> lock(writeMutex_);
    auto *cursor = reinterpret_cast<const std::uint8_t *>(pcm);
    std::size_t remaining = sampleCount * sizeof(std::int16_t);
    while (remaining > 0) {
      const int bytesToWrite = static_cast<int>(std::min<std::size_t>(remaining, 4096));
      const int err = esp_codec_dev_write(
          speakerCodec_,
          const_cast<std::uint8_t *>(cursor),
          bytesToWrite);
      if (err != ESP_OK) {
        ESP_LOGW(kTag, "Speaker write failed: %s", esp_err_to_name(err));
        return false;
      }
      cursor += bytesToWrite;
      remaining -= static_cast<std::size_t>(bytesToWrite);
    }
    return true;
  }

  void close() {
    std::lock_guard<std::mutex> lock(writeMutex_);
    closeSpeakerLocked();
  }

  int volume() const {
    return speakerVolume_;
  }

  void setVolume(int volumePercent) {
    if (volumePercent < 0) volumePercent = 0;
    if (volumePercent > 100) volumePercent = 100;
    speakerVolume_ = volumePercent;
    if (speakerOpen_ && speakerCodec_) {
      std::lock_guard<std::mutex> lock(writeMutex_);
      applySpeakerVolumeLocked();
    }
  }

private:
  AudioOutputDriver() = default;

  static int codecVolumeFromUserPercent(int volumePercent) {
    if (volumePercent <= 0) return 0;
    if (volumePercent >= 100) return 100;

    const double amplitude = static_cast<double>(volumePercent) / 100.0;
    const double db = 20.0 * std::log10(amplitude);
    int codecVolume = static_cast<int>(std::lround(((db + 50.0) * 100.0) / 50.0));
    if (codecVolume < 1) codecVolume = 1;
    if (codecVolume > 100) codecVolume = 100;
    return codecVolume;
  }

  void applySpeakerVolumeLocked() {
    if (!speakerCodec_) return;
    if (kPaPin != GPIO_NUM_NC) gpio_set_level(kPaPin, 1);
    const int codecVolume = codecVolumeFromUserPercent(speakerVolume_);
    const int err = esp_codec_dev_set_out_vol(speakerCodec_, codecVolume);
    if (err != ESP_CODEC_DEV_OK) {
      ESP_LOGW(kTag, "Speaker volume set failed volume=%d codec_volume=%d err=%d", speakerVolume_, codecVolume, err);
    }
  }

  void closeSpeakerLocked() {
    if (speakerCodec_ && speakerOpen_) {
      esp_codec_dev_close(speakerCodec_);
      i2sChannelEnabled_ = false;
    }
    if (kPaPin != GPIO_NUM_NC) gpio_set_level(kPaPin, 0);
    speakerOpen_ = false;
    speakerSampleRate_ = 0;
    speakerChannels_ = 0;
    speakerBitsPerSample_ = 0;
  }

  void closeRecorderLocked() {
    if (recordCodec_ && recordOpen_) {
      esp_codec_dev_close(recordCodec_);
      rxChannelEnabled_ = false;
    }
    recordOpen_ = false;
  }

  esp_err_t disableI2sChannel(i2s_chan_handle_t channel, bool &enabled, const char *label) {
    if (!channel || !enabled) return ESP_OK;
    esp_err_t err = i2s_channel_disable(channel);
    if (err == ESP_ERR_INVALID_STATE) {
      ESP_LOGW(kTag, "I2S %s channel was already disabled", label);
      err = ESP_OK;
    }
    if (err != ESP_OK) {
      ESP_LOGE(kTag, "I2S %s disable failed: %s", label, esp_err_to_name(err));
      return err;
    }
    enabled = false;
    return ESP_OK;
  }

  void releaseI2s() {
    if (txChannel_) {
      (void)disableI2sChannel(txChannel_, i2sChannelEnabled_, "TX");
      i2s_del_channel(txChannel_);
      txChannel_ = nullptr;
    }
    if (rxChannel_) {
      (void)disableI2sChannel(rxChannel_, rxChannelEnabled_, "RX");
      i2s_del_channel(rxChannel_);
      rxChannel_ = nullptr;
    }
    i2sDataIf_ = nullptr;
  }

  esp_err_t initI2s(int sampleRate) {
    if (txChannel_ && i2sDataIf_) {
      if (i2sSampleRate_ == sampleRate) return ESP_OK;
      return reconfigureI2sClock(sampleRate);
    }

    i2s_chan_config_t channelConfig = I2S_CHANNEL_DEFAULT_CONFIG(kI2sPort, I2S_ROLE_MASTER);
    channelConfig.auto_clear = true;
    // Match the pala_note reference's DMA buffering (the IDF default 6 x 240
    // frames ~= 90 ms at 16 kHz). Two descriptors is effectively single-
    // buffered: on RX a slow SD flush in the capture task overruns and drops
    // mic samples (recording plays back "sped up"); on TX any scheduling jitter
    // starves the codec and the playback clicks/underruns. Six descriptors give
    // the headroom that makes pala_note's record/playback smooth on this codec.
    channelConfig.dma_desc_num = 6;
    channelConfig.dma_frame_num = 240;
    esp_err_t err = i2s_new_channel(&channelConfig, &txChannel_, &rxChannel_);
    if (err != ESP_OK) {
      ESP_LOGE(kTag, "I2S channel creation failed: %s", esp_err_to_name(err));
      return err;
    }

    i2s_std_config_t stdConfig = {};
    stdConfig.clk_cfg = I2S_STD_CLK_DEFAULT_CONFIG(static_cast<std::uint32_t>(sampleRate));
    stdConfig.slot_cfg = I2S_STD_MSB_SLOT_DEFAULT_CONFIG(I2S_DATA_BIT_WIDTH_32BIT, I2S_SLOT_MODE_STEREO);
    stdConfig.gpio_cfg.mclk = kMclkPin;
    stdConfig.gpio_cfg.bclk = kBclkPin;
    stdConfig.gpio_cfg.ws = kWsPin;
    stdConfig.gpio_cfg.dout = kDoutPin;
    stdConfig.gpio_cfg.din = kDinPin;
    stdConfig.gpio_cfg.invert_flags.mclk_inv = false;
    stdConfig.gpio_cfg.invert_flags.bclk_inv = false;
    stdConfig.gpio_cfg.invert_flags.ws_inv = false;

    err = i2s_channel_init_std_mode(txChannel_, &stdConfig);
    if (err != ESP_OK) {
      ESP_LOGE(kTag, "I2S std init failed: %s", esp_err_to_name(err));
      releaseI2s();
      return err;
    }

    err = i2s_channel_enable(txChannel_);
    if (err != ESP_OK) {
      ESP_LOGE(kTag, "I2S enable failed: %s", esp_err_to_name(err));
      releaseI2s();
      return err;
    }
    i2sChannelEnabled_ = true;

    err = i2s_channel_init_std_mode(rxChannel_, &stdConfig);
    if (err != ESP_OK) {
      ESP_LOGE(kTag, "I2S RX std init failed: %s", esp_err_to_name(err));
      releaseI2s();
      return err;
    }
    err = i2s_channel_enable(rxChannel_);
    if (err != ESP_OK) {
      ESP_LOGE(kTag, "I2S RX enable failed: %s", esp_err_to_name(err));
      releaseI2s();
      return err;
    }
    rxChannelEnabled_ = true;

    audio_codec_i2s_cfg_t i2sConfig = {};
    i2sConfig.port = kCodecDataPort;
    i2sConfig.rx_handle = rxChannel_;
    i2sConfig.tx_handle = txChannel_;
    i2sDataIf_ = audio_codec_new_i2s_data(&i2sConfig);
    if (!i2sDataIf_) {
      ESP_LOGE(kTag, "I2S codec data interface creation failed");
      releaseI2s();
      return ESP_ERR_NO_MEM;
    }

    i2sSampleRate_ = sampleRate;
    return ESP_OK;
  }

  esp_err_t reconfigureI2sClock(int sampleRate) {
    if (!txChannel_ || !rxChannel_) return ESP_ERR_INVALID_STATE;

    esp_err_t err = disableI2sChannel(txChannel_, i2sChannelEnabled_, "TX");
    if (err != ESP_OK) return err;
    err = disableI2sChannel(rxChannel_, rxChannelEnabled_, "RX");
    if (err != ESP_OK) return err;

    i2s_std_clk_config_t clockConfig = I2S_STD_CLK_DEFAULT_CONFIG(static_cast<std::uint32_t>(sampleRate));
    err = i2s_channel_reconfig_std_clock(txChannel_, &clockConfig);
    if (err != ESP_OK) {
      ESP_LOGE(kTag, "I2S TX clock reconfig failed: %s", esp_err_to_name(err));
      return err;
    }
    err = i2s_channel_reconfig_std_clock(rxChannel_, &clockConfig);
    if (err != ESP_OK) {
      ESP_LOGE(kTag, "I2S RX clock reconfig failed: %s", esp_err_to_name(err));
      return err;
    }

    err = i2s_channel_enable(txChannel_);
    if (err != ESP_OK) {
      ESP_LOGE(kTag, "I2S TX re-enable failed: %s", esp_err_to_name(err));
      return err;
    }
    i2sChannelEnabled_ = true;
    err = i2s_channel_enable(rxChannel_);
    if (err != ESP_OK) {
      ESP_LOGE(kTag, "I2S RX re-enable failed: %s", esp_err_to_name(err));
      return err;
    }
    rxChannelEnabled_ = true;
    i2sSampleRate_ = sampleRate;
    return ESP_OK;
  }

  esp_err_t initSpeaker(int sampleRate) {
    return initCodecDevice(sampleRate, ESP_CODEC_DEV_TYPE_OUT, speakerCodec_);
  }

  esp_err_t initRecorder(int sampleRate) {
    if (recordOpen_ && i2sSampleRate_ == sampleRate) return ESP_OK;
    if (recordOpen_) closeRecorderLocked();
    if (speakerOpen_) closeSpeakerLocked();

    esp_err_t err = initCodecDevice(sampleRate, ESP_CODEC_DEV_TYPE_IN, recordCodec_);
    if (err != ESP_OK) return err;

    esp_codec_dev_sample_info_t sampleInfo = {};
    sampleInfo.bits_per_sample = 16;
    sampleInfo.channel = 2;
    sampleInfo.sample_rate = static_cast<std::uint32_t>(sampleRate);
    sampleInfo.mclk_multiple = I2S_MCLK_MULTIPLE_256;
    err = esp_codec_dev_open(recordCodec_, &sampleInfo);
    if (err != ESP_OK) {
      ESP_LOGE(kTag, "Recorder open failed: %s", esp_err_to_name(err));
      return err;
    }
    esp_codec_dev_set_in_gain(recordCodec_, 45.0f);
    recordOpen_ = true;
    i2sSampleRate_ = sampleRate;
    return ESP_OK;
  }

  esp_err_t initCodecDevice(int sampleRate, esp_codec_dev_type_t deviceType, esp_codec_dev_handle_t &device) {
    esp_err_t err = initI2s(sampleRate);
    if (err != ESP_OK) return err;
    if (device) return ESP_OK;

    if constexpr (kPaPin != GPIO_NUM_NC) {
      gpio_config_t paConfig = {};
      paConfig.pin_bit_mask = 1ULL << kPaPin;
      paConfig.mode = GPIO_MODE_OUTPUT;
      gpio_config(&paConfig);
      gpio_set_level(kPaPin, 1);
    }

    if (!codecIf_) {
      auto i2cBus = gea::platform::i2c::Bus::primary();
      if (!i2cBus.available()) {
        ESP_LOGE(kTag, "I2C bus is not ready for ES8311");
        return ESP_ERR_INVALID_STATE;
      }

      audio_codec_i2c_cfg_t i2cConfig = {};
      i2cConfig.addr = ES8311_CODEC_DEFAULT_ADDR;
      i2cConfig.bus_handle = static_cast<i2c_master_bus_handle_t>(i2cBus.nativeHandle());
      i2cCtrlIf_ = audio_codec_new_i2c_ctrl(&i2cConfig);
      if (!i2cCtrlIf_) return ESP_ERR_NO_MEM;

      gpioIf_ = audio_codec_new_gpio();
      if (!gpioIf_) return ESP_ERR_NO_MEM;

      esp_codec_dev_hw_gain_t gain = {};
      gain.pa_voltage = 5.0;
      gain.codec_dac_voltage = 3.3;
      gain.pa_gain = 6.0;

      es8311_codec_cfg_t es8311Config = {};
      es8311Config.ctrl_if = i2cCtrlIf_;
      es8311Config.gpio_if = gpioIf_;
      es8311Config.codec_mode = ESP_CODEC_DEV_WORK_MODE_BOTH;
      es8311Config.pa_pin = kPaPin;
      es8311Config.pa_reverted = false;
      es8311Config.master_mode = false;
      es8311Config.use_mclk = true;
      es8311Config.digital_mic = false;
      es8311Config.invert_mclk = false;
      es8311Config.invert_sclk = false;
      es8311Config.hw_gain = gain;
      es8311Config.no_dac_ref = false;
      es8311Config.mclk_div = 256;

      codecIf_ = es8311_codec_new(&es8311Config);
      if (!codecIf_) return ESP_ERR_NO_MEM;
    }

    esp_codec_dev_cfg_t codecConfig = {};
    codecConfig.dev_type = deviceType;
    codecConfig.codec_if = codecIf_;
    codecConfig.data_if = i2sDataIf_;
    device = esp_codec_dev_new(&codecConfig);
    if (!device) return ESP_ERR_NO_MEM;
    esp_codec_set_disable_when_closed(device, false);
    return ESP_OK;
  }

  static constexpr const char *kTag = "audio";

  i2s_chan_handle_t txChannel_ = nullptr;
  i2s_chan_handle_t rxChannel_ = nullptr;
  const audio_codec_data_if_t *i2sDataIf_ = nullptr;
  const audio_codec_ctrl_if_t *i2cCtrlIf_ = nullptr;
  const audio_codec_gpio_if_t *gpioIf_ = nullptr;
  const audio_codec_if_t *codecIf_ = nullptr;
  esp_codec_dev_handle_t speakerCodec_ = nullptr;
  esp_codec_dev_handle_t recordCodec_ = nullptr;
  bool i2sChannelEnabled_ = false;
  bool rxChannelEnabled_ = false;
  bool speakerOpen_ = false;
  bool recordOpen_ = false;
  int i2sSampleRate_ = 0;
  int speakerSampleRate_ = 0;
  int speakerChannels_ = 0;
  int speakerBitsPerSample_ = 0;
  int speakerVolume_ = 100;
  std::mutex writeMutex_;

  TaskHandle_t captureTask_ = nullptr;
  std::vector<gea::host::NativeMediaTrackHandle> attachedTracks_;
  std::vector<gea::host::NativeMediaTrackHandle> captureSnapshot_;
  std::mutex attachedMutex_;
  static constexpr std::size_t kCaptureFrameSamples = 2048;
  std::array<std::int16_t, kCaptureFrameSamples * 2> stereoFrame_{};
  std::array<std::int16_t, kCaptureFrameSamples> monoFrame_{};

  static void captureTaskTrampoline(void *arg) {
    static_cast<AudioOutputDriver *>(arg)->runCaptureTask();
  }

  void runCaptureTask() {
    int consecutiveReadErrors = 0;

    while (true) {
      {
        std::lock_guard<std::mutex> lock(attachedMutex_);
        captureSnapshot_ = attachedTracks_;
      }

      if (captureSnapshot_.empty()) {
        {
          std::lock_guard<std::mutex> lock(writeMutex_);
          closeRecorderLocked();
        }
        vTaskDelay(pdMS_TO_TICKS(50));
        continue;
      }

      int err = ESP_OK;
      {
        std::lock_guard<std::mutex> lock(writeMutex_);
        err = initRecorder(16000);
        if (err == ESP_OK && recordCodec_) {
          err = esp_codec_dev_read(recordCodec_, stereoFrame_.data(), stereoFrame_.size() * sizeof(std::int16_t));
        }
      }
      if (err != ESP_OK) {
        if (consecutiveReadErrors == 0 || consecutiveReadErrors % 100 == 0) {
          ESP_LOGW(kTag, "Mic read failed err=%s snapshot_tracks=%u", esp_err_to_name(err), static_cast<unsigned>(captureSnapshot_.size()));
        }
        ++consecutiveReadErrors;
        vTaskDelay(pdMS_TO_TICKS(10));
        continue;
      }
      consecutiveReadErrors = 0;
      const std::size_t framesRead = kCaptureFrameSamples;

      // ES8311 input is interleaved as [mic, ref]. For local voice notes, keep
      // the real mic channel only; mixing in the ref slot makes speech metallic.
      for (std::size_t i = 0; i < framesRead; ++i) {
        monoFrame_[i] = stereoFrame_[2 * i];
      }
      for (auto handle : captureSnapshot_) {
        gea::host::media::track_inject_pcm(handle, monoFrame_.data(), framesRead);
      }
    }
  }

public:
  void attachTrack(gea::host::NativeMediaTrackHandle handle) {
    {
      std::lock_guard<std::mutex> lock(attachedMutex_);
      attachedTracks_.push_back(handle);
    }
    if (captureTask_ == nullptr) {
      xTaskCreatePinnedToCore(captureTaskTrampoline, "gea_mic", 8192, this, 5, &captureTask_, 1);
    }
  }

  void detachTrack(gea::host::NativeMediaTrackHandle handle) {
    std::lock_guard<std::mutex> lock(attachedMutex_);
    attachedTracks_.erase(std::remove(attachedTracks_.begin(), attachedTracks_.end(), handle), attachedTracks_.end());
  }
};

}  // namespace gea::platform::esp32::chip_bindings::es8311

bool gea::platform::audio::OutputDriver::open(int sampleRate, int channels, int bitsPerSample) {
  return gea::platform::esp32::chip_bindings::es8311::AudioOutputDriver::instance().open(sampleRate, channels, bitsPerSample);
}

bool gea::platform::audio::OutputDriver::write(const std::int16_t *pcm, std::size_t sampleCount, int timeoutMs) {
  return gea::platform::esp32::chip_bindings::es8311::AudioOutputDriver::instance().write(pcm, sampleCount, timeoutMs);
}

void gea::platform::audio::OutputDriver::close() {
  gea::platform::esp32::chip_bindings::es8311::AudioOutputDriver::instance().close();
}

int gea::platform::audio::OutputDriver::volume() {
  return gea::platform::esp32::chip_bindings::es8311::AudioOutputDriver::instance().volume();
}

void gea::platform::audio::OutputDriver::setVolume(int volumePercent) {
  gea::platform::esp32::chip_bindings::es8311::AudioOutputDriver::instance().setVolume(volumePercent);
}

namespace gea::host::media {

void platform_attach_track(NativeMediaTrackHandle handle) {
  gea::platform::esp32::chip_bindings::es8311::AudioOutputDriver::instance().attachTrack(handle);
}

void platform_detach_track(NativeMediaTrackHandle handle) {
  gea::platform::esp32::chip_bindings::es8311::AudioOutputDriver::instance().detachTrack(handle);
}

}  // namespace gea::host::media
