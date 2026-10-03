#define GEA_AUDIO_DRIVER_INTERNAL 1
#include "audio.h"
#include "board.h"
#if GEA_BOARD_HAS_EXPANDER
#include "chip_bindings/expanders/io_expander.h"
#endif
#include "audio/es8311/es8311.h"

#include "i2c.h"

#include <algorithm>
#include <array>
#include <atomic>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <mutex>
#include <stdexcept>
#include <vector>

#include "driver/gpio.h"
#include "driver/i2c_master.h"
#include "driver/i2s_std.h"
#include "driver/i2s_tdm.h"
#include "esp_codec_dev.h"
#include "esp_codec_dev_defaults.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "host/media.h"

// Full duplex is an app opt-in. Both directions share a clock, so a live
// capture stream cannot coexist with playback at a different sample rate.
#ifndef GEA_AUDIO_FULL_DUPLEX
#define GEA_AUDIO_FULL_DUPLEX 0
#endif
static_assert(GEA_AUDIO_FULL_DUPLEX == 0 || GEA_AUDIO_FULL_DUPLEX == 1);

#ifndef GEA_AUDIO_ECHO_CANCELLATION
#define GEA_AUDIO_ECHO_CANCELLATION 0
#endif
#if GEA_AUDIO_ECHO_CANCELLATION
static_assert(GEA_AUDIO_FULL_DUPLEX, "Echo cancellation requires full duplex");
#include "echo_cancellation.h"
#include "esp_heap_caps.h"
#endif

// Applications with a continuous low-latency source can use smaller DMA queues.
#ifndef GEA_AUDIO_DMA_DESCRIPTORS
#define GEA_AUDIO_DMA_DESCRIPTORS 6
#endif
#ifndef GEA_AUDIO_DMA_FRAMES
#define GEA_AUDIO_DMA_FRAMES 240
#endif
static_assert(GEA_AUDIO_DMA_DESCRIPTORS >= 2);
static_assert(GEA_AUDIO_DMA_FRAMES >= 8 && GEA_AUDIO_DMA_FRAMES <= 511);

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

esp_err_t setAmplifierEnabled(bool enabled) {
#if GEA_BOARD_SPEAKER_POWER
  return gea::platform::board::setSpeakerPower(enabled) ? ESP_OK : ESP_FAIL;
#endif
  if constexpr (kPaPin != GPIO_NUM_NC) return gpio_set_level(kPaPin, enabled);
#if GEA_BOARD_HAS_EXPANDER
  constexpr int pin = gea::platform::board::expander.powerAmplifier;
  if constexpr (pin >= 0) {
    auto &io = gea::platform::esp32::chip_bindings::expanders::ioExpander();
    return io.writePin(pin, enabled) && io.setInput(pin, false) ? ESP_OK : ESP_FAIL;
  }
#endif
  return ESP_OK;
}

constexpr int kCodecDataPort = 0;

// esp_codec_dev takes the 8-bit address. CE low answers at 7-bit 0x18, the
// component default; a board that straps CE high (the ESP-Mosaico, 0x19)
// names its 7-bit address with GEA_BOARD_ES8311_I2C_ADDRESS.
#ifdef GEA_BOARD_ES8311_I2C_ADDRESS
constexpr std::uint8_t kCodecI2cAddress = static_cast<std::uint8_t>(GEA_BOARD_ES8311_I2C_ADDRESS << 1);
#else
constexpr std::uint8_t kCodecI2cAddress = ES8311_CODEC_DEFAULT_ADDR;
#endif

// Boards whose microphones sit on an ES7210 ADC sharing this I2S bus (Waveshare AMOLED 2.06) name its
// 7-bit I2C address as `audio.es7210Address`; everywhere else the ES8311's own ADC records.
template <typename AudioConfig>
constexpr int micAdcAddressOf(const AudioConfig &audio) {
  if constexpr (requires { audio.es7210Address; }) {
    return audio.es7210Address;
  } else {
    return 0;
  }
}
constexpr int kMicAdcAddress = micAdcAddressOf(gea::platform::board::audio);
template <typename AudioConfig>
constexpr int referenceMicOf(const AudioConfig &audio) {
  if constexpr (requires { audio.es7210ReferenceMic; }) return audio.es7210ReferenceMic;
  return 0;
}
constexpr bool kHardwareAecReference = GEA_AUDIO_ECHO_CANCELLATION && kMicAdcAddress != 0 &&
    referenceMicOf(gea::platform::board::audio) == 3;

template <typename AudioConfig>
constexpr float aecMicGainOf(const AudioConfig &audio) {
  if constexpr (requires { audio.aecMicGainDb; }) return audio.aecMicGainDb;
  return 12.0f;
}
template <typename AudioConfig>
constexpr float aecOutputGainOf(const AudioConfig &audio) {
  if constexpr (requires { audio.aecOutputGain; }) return audio.aecOutputGain;
  return 1.0f;
}
constexpr float kAecMicGain = aecMicGainOf(gea::platform::board::audio);
#ifndef GEA_AUDIO_AEC_OUTPUT_GAIN_SCALE
#define GEA_AUDIO_AEC_OUTPUT_GAIN_SCALE 1.0f
#endif
static_assert(GEA_AUDIO_AEC_OUTPUT_GAIN_SCALE >= 0.0f && GEA_AUDIO_AEC_OUTPUT_GAIN_SCALE <= 64.0f,
              "AEC output gain scale must be between 0 and 64");
// Apply before the AEC output is clamped to int16. Scaling the published
// microphone packets afterward cannot recover samples already clipped here.
constexpr float kAecOutputGain = aecOutputGainOf(gea::platform::board::audio) *
    GEA_AUDIO_AEC_OUTPUT_GAIN_SCALE;
template <typename AudioConfig>
constexpr bool aecAggressiveNlpOf(const AudioConfig &audio) {
  if constexpr (requires { audio.aecAggressiveNlp; }) return audio.aecAggressiveNlp;
  return true;
}
constexpr bool kAecAggressiveNlp = aecAggressiveNlpOf(gea::platform::board::audio);
template <typename AudioConfig>
constexpr bool codecReferenceOf(const AudioConfig &audio) {
  if constexpr (requires { audio.aecCodecReference; }) return audio.aecCodecReference;
  return false;
}
// ES8311's ADCL + DACR mode returns mic and DAC reference in one RX frame.
// This changes AEC pairing, not the ES7210-specific TDM/clock configuration.
constexpr bool kPairedAecReference = kHardwareAecReference ||
    (GEA_AUDIO_ECHO_CANCELLATION && codecReferenceOf(gea::platform::board::audio));

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
    std::lock_guard<std::mutex> captureLock(readMutex_);
    const gea::chips::es8311::OutputFormat format(sampleRate, channels, bitsPerSample);
    if (!format.isPcm16()) return false;
#if GEA_AUDIO_ECHO_CANCELLATION
    if (sampleRate != 16000 || (channels != 1 && channels != 2)) return false;
#endif

    if (speakerOpen_ &&
        speakerSampleRate_ == sampleRate &&
        speakerChannels_ == channels &&
        speakerBitsPerSample_ == bitsPerSample) {
      applySpeakerVolumeLocked();
      return true;
    }

    if constexpr (GEA_AUDIO_FULL_DUPLEX) {
      if (recordOpen_ && i2sSampleRate_ != sampleRate) return false;
    } else {
      if (recordOpen_) closeRecorderLocked();
    }
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
#if GEA_AUDIO_ECHO_CANCELLATION
    aecSpeakerActive_.store(true);
#endif
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
    if (!pcm || sampleCount == 0) return false;

    // Checked under the lock, so idle-capture cleanup cannot delete the codec between check and write.
    std::lock_guard<std::mutex> lock(writeMutex_);
    if (!speakerCodec_ || !speakerOpen_) return false;

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
    std::lock_guard<std::mutex> captureLock(readMutex_);
    closeSpeakerLocked();
    // An idle capture task leaves the audio path up while the speaker plays; free it once neither uses it.
    if (!micInUse()) releaseAudioLocked();
  }

  int volume() const {
    return speakerVolume_;
  }
  bool flush() {
    std::lock_guard<std::mutex> lock(writeMutex_);
    if (!speakerOpen_ || !txChannel_) return true;
    i2s_chan_info_t info{};
    if (i2s_channel_get_info(txChannel_, &info) != ESP_OK) return false;
    // Reset TX descriptors only; retain RX, both codecs, AEC and its history.
    if (i2s_channel_disable(txChannel_) != ESP_OK) return false;
    static constexpr std::array<std::uint8_t, 256> silence{};
    std::size_t remaining = info.total_dma_buf_size;
    bool cleared = true;
    while (remaining) {
      std::size_t loaded = 0;
      const auto count = std::min(remaining, silence.size());
      if (i2s_channel_preload_data(txChannel_, silence.data(), count, &loaded) != ESP_OK || loaded != count) {
        cleared = false;
        break;
      }
      remaining -= loaded;
    }
    const auto restarted = i2s_channel_enable(txChannel_) == ESP_OK;
    ESP_LOGI(kTag, "Speaker DMA cancelled bytes=%u cleared=%d restarted=%d", unsigned(info.total_dma_buf_size), int(cleared), int(restarted));
    return cleared && restarted;
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
    setAmplifierEnabled(speakerVolume_ > 0);
    const int codecVolume = codecVolumeFromUserPercent(speakerVolume_);
    const int err = esp_codec_dev_set_out_vol(speakerCodec_, codecVolume);
    if (err != ESP_CODEC_DEV_OK) {
      ESP_LOGW(kTag, "Speaker volume set failed volume=%d codec_volume=%d err=%d", speakerVolume_, codecVolume, err);
    }
  }

  void closeSpeakerLocked() {
#if GEA_AUDIO_ECHO_CANCELLATION
    aecSpeakerActive_.store(false);
#endif
    if (speakerCodec_ && speakerOpen_) {
      esp_codec_dev_close(speakerCodec_);
    }
    setAmplifierEnabled(false);
    speakerOpen_ = false;
    speakerSampleRate_ = 0;
    speakerChannels_ = 0;
    speakerBitsPerSample_ = 0;
  }

  void closeRecorderLocked() {
    if (recordCodec_ && recordOpen_) {
      esp_codec_dev_close(recordCodec_);
    }
    recordOpen_ = false;
#if GEA_AUDIO_ECHO_CANCELLATION
    geaAudioAecStop();
#endif
  }

  esp_err_t disableI2sChannel(i2s_chan_handle_t channel, const char *label) {
    if (!channel) return ESP_OK;
    esp_err_t err = i2s_channel_disable(channel);
    if (err == ESP_ERR_INVALID_STATE) {
      err = ESP_OK;
    }
    if (err != ESP_OK) {
      ESP_LOGE(kTag, "I2S %s disable failed: %s", label, esp_err_to_name(err));
      return err;
    }
    return ESP_OK;
  }

  void releaseI2s() {
    // esp_codec_dev may enable TX implicitly while opening RX for its clock.
    // A cached enabled flag cannot observe that; consult the actual driver by
    // disabling each channel before deletion. Keep a handle if teardown fails.
    const auto release = [&](i2s_chan_handle_t &channel, const char *label) {
      if (!channel || disableI2sChannel(channel, label) != ESP_OK) return;
      const esp_err_t err = i2s_del_channel(channel);
      if (err != ESP_OK) {
        ESP_LOGE(kTag, "I2S %s deletion failed: %s", label, esp_err_to_name(err));
        return;
      }
      channel = nullptr;
    };
    release(txChannel_, "TX");
    release(rxChannel_, "RX");
    i2sDataIf_ = nullptr;
  }

  // Frees everything the audio path holds in internal RAM: the codec devices, the I2S data
  // interface, and both I2S channels with their DMA buffers. A recording app that also uses the
  // network needs this RAM back once the mic stops (esp_wifi_init fails with ESP_ERR_NO_MEM
  // otherwise). initCodecDevice() rebuilds it all on the next open. Call with writeMutex_ held.
  void releaseAudioLocked() {
    closeRecorderLocked();
    closeSpeakerLocked();
    if (speakerCodec_) {
      esp_codec_dev_delete(speakerCodec_);
      speakerCodec_ = nullptr;
    }
    if (recordCodec_) {
      esp_codec_dev_delete(recordCodec_);
      recordCodec_ = nullptr;
    }
    if (i2sDataIf_) audio_codec_delete_data_if(i2sDataIf_);
    releaseI2s();
    i2sSampleRate_ = 0;
  }

#if GEA_AUDIO_ECHO_CANCELLATION
  std::atomic<bool> aecSpeakerActive_{false};

  static bool txSent(i2s_chan_handle_t, i2s_event_data_t *event, void *context) {
    const auto *driver = static_cast<AudioOutputDriver *>(context);
    static const std::array<int16_t, 240> silence{};
    if (driver->aecSpeakerActive_.load(std::memory_order_relaxed)) {
      geaAudioAecReference(static_cast<const int16_t *>(event->dma_buf), event->size / sizeof(int16_t));
    } else {
      geaAudioAecReference(silence.data(), silence.size());
    }
    return false;
  }

  static bool rxReceived(i2s_chan_handle_t, i2s_event_data_t *event, void *) {
    geaAudioAecReceive(static_cast<const int16_t *>(event->dma_buf), event->size / sizeof(int16_t));
    return false;
  }
#endif

  esp_err_t initI2s(int sampleRate) {
    if (txChannel_ && i2sDataIf_) {
      if (i2sSampleRate_ == sampleRate) return ESP_OK;
      return reconfigureI2sClock(sampleRate);
    }

    if (txChannel_ || rxChannel_) {
      releaseI2s();
      if (txChannel_ || rxChannel_) return ESP_ERR_INVALID_STATE;
    }

    i2s_chan_config_t channelConfig = I2S_CHANNEL_DEFAULT_CONFIG(kI2sPort, I2S_ROLE_MASTER);
    channelConfig.auto_clear = true;
    // Match the pala_note reference's DMA buffering (the IDF default 6 x 240
    // frames ~= 90 ms at 16 kHz). Two descriptors is effectively single-
    // buffered: on RX a slow SD flush in the capture task overruns and drops
    // mic samples (recording plays back "sped up"); on TX any scheduling jitter
    // starves the codec and the playback clicks/underruns. Six descriptors give
    // the headroom that makes pala_note's record/playback smooth on this codec.
    channelConfig.dma_desc_num = GEA_AUDIO_DMA_DESCRIPTORS;
    channelConfig.dma_frame_num = GEA_AUDIO_DMA_FRAMES;
    esp_err_t err = i2s_new_channel(&channelConfig, &txChannel_, &rxChannel_);
    if (err != ESP_OK) {
      ESP_LOGE(kTag, "I2S channel creation failed: %s", esp_err_to_name(err));
      return err;
    }

    // Start with the codec's actual 16-bit stereo slots. A 32-bit placeholder
    // doubles DMA storage and forces allocation again during codec open.
    i2s_std_config_t stdConfig = {};
    stdConfig.clk_cfg = I2S_STD_CLK_DEFAULT_CONFIG(static_cast<std::uint32_t>(sampleRate));
    stdConfig.slot_cfg = I2S_STD_PHILIPS_SLOT_DEFAULT_CONFIG(I2S_DATA_BIT_WIDTH_16BIT, I2S_SLOT_MODE_STEREO);
    if constexpr (kHardwareAecReference) {
      // Four 16-bit ADC slots share a 64-clock frame with two 32-bit DAC slots.
      stdConfig.slot_cfg.slot_bit_width = I2S_SLOT_BIT_WIDTH_32BIT;
      stdConfig.slot_cfg.ws_width = 32;
    }
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

#if GEA_AUDIO_ECHO_CANCELLATION
    i2s_event_callbacks_t txCallbacks{};
    txCallbacks.on_sent = txSent;
    err = kPairedAecReference ? ESP_OK : i2s_channel_register_event_callback(txChannel_, &txCallbacks, this);
    if (err != ESP_OK) { releaseI2s(); return err; }
#endif
    err = i2s_channel_enable(txChannel_);
    if (err != ESP_OK) {
      ESP_LOGE(kTag, "I2S enable failed: %s", esp_err_to_name(err));
      releaseI2s();
      return err;
    }

    if constexpr (kHardwareAecReference) {
      i2s_tdm_config_t rxConfig{};
      rxConfig.clk_cfg = I2S_TDM_CLK_DEFAULT_CONFIG(static_cast<uint32_t>(sampleRate));
      rxConfig.slot_cfg = I2S_TDM_PHILIPS_SLOT_DEFAULT_CONFIG(
          I2S_DATA_BIT_WIDTH_16BIT, I2S_SLOT_MODE_STEREO,
          static_cast<i2s_tdm_slot_mask_t>(I2S_TDM_SLOT0 | I2S_TDM_SLOT1));
      // ES7210 TDM order is MIC1, MIC3, MIC2, MIC4. Capture only the first
      // microphone and hardware reference; unselected slots consume no DMA RAM.
      rxConfig.slot_cfg.total_slot = 4;
      rxConfig.slot_cfg.ws_width = 32;
      rxConfig.gpio_cfg.mclk = kMclkPin;
      rxConfig.gpio_cfg.bclk = kBclkPin;
      rxConfig.gpio_cfg.ws = kWsPin;
      rxConfig.gpio_cfg.dout = I2S_GPIO_UNUSED;
      rxConfig.gpio_cfg.din = kDinPin;
      err = i2s_channel_init_tdm_mode(rxChannel_, &rxConfig);
    } else {
      err = i2s_channel_init_std_mode(rxChannel_, &stdConfig);
    }
    if (err != ESP_OK) {
      ESP_LOGE(kTag, "I2S RX std init failed: %s", esp_err_to_name(err));
      releaseI2s();
      return err;
    }
#if GEA_AUDIO_ECHO_CANCELLATION
    i2s_event_callbacks_t rxCallbacks{};
    rxCallbacks.on_recv = rxReceived;
    err = i2s_channel_register_event_callback(rxChannel_, &rxCallbacks, this);
    if (err != ESP_OK) { releaseI2s(); return err; }
#endif
    err = i2s_channel_enable(rxChannel_);
    if (err != ESP_OK) {
      ESP_LOGE(kTag, "I2S RX enable failed: %s", esp_err_to_name(err));
      releaseI2s();
      return err;
    }

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

    esp_err_t err = disableI2sChannel(txChannel_, "TX");
    if (err != ESP_OK) return err;
    err = disableI2sChannel(rxChannel_, "RX");
    if (err != ESP_OK) return err;

    i2s_std_clk_config_t clockConfig = I2S_STD_CLK_DEFAULT_CONFIG(static_cast<std::uint32_t>(sampleRate));
    err = i2s_channel_reconfig_std_clock(txChannel_, &clockConfig);
    if (err != ESP_OK) {
      ESP_LOGE(kTag, "I2S TX clock reconfig failed: %s", esp_err_to_name(err));
      return err;
    }
    if constexpr (kHardwareAecReference) {
      i2s_tdm_clk_config_t rxClock = I2S_TDM_CLK_DEFAULT_CONFIG(static_cast<uint32_t>(sampleRate));
      err = i2s_channel_reconfig_tdm_clock(rxChannel_, &rxClock);
    } else {
      err = i2s_channel_reconfig_std_clock(rxChannel_, &clockConfig);
    }
    if (err != ESP_OK) {
      ESP_LOGE(kTag, "I2S RX clock reconfig failed: %s", esp_err_to_name(err));
      return err;
    }

    err = i2s_channel_enable(txChannel_);
    if (err != ESP_OK) {
      ESP_LOGE(kTag, "I2S TX re-enable failed: %s", esp_err_to_name(err));
      return err;
    }
    err = i2s_channel_enable(rxChannel_);
    if (err != ESP_OK) {
      ESP_LOGE(kTag, "I2S RX re-enable failed: %s", esp_err_to_name(err));
      return err;
    }
    i2sSampleRate_ = sampleRate;
    return ESP_OK;
  }

  esp_err_t initSpeaker(int sampleRate) {
    return initCodecDevice(sampleRate, ESP_CODEC_DEV_TYPE_OUT, speakerCodec_);
  }

  esp_err_t initRecorder(int sampleRate) {
    if (recordOpen_ && i2sSampleRate_ == sampleRate) return ESP_OK;
    if constexpr (GEA_AUDIO_FULL_DUPLEX) {
      if (speakerOpen_ && speakerSampleRate_ != sampleRate) return ESP_ERR_NOT_SUPPORTED;
    }
    if (recordOpen_) closeRecorderLocked();
    if constexpr (!GEA_AUDIO_FULL_DUPLEX) {
      if (speakerOpen_) {
        // A successful write only queues PCM in TX DMA. Reclaiming I2S for
        // capture immediately can truncate the final speech samples. Writing
        // one complete DMA ring of silence waits for every earlier descriptor
        // to finish; this follows the actual driver capacity, not a sleep.
        i2s_chan_info_t info{};
        esp_err_t drain = i2s_channel_get_info(txChannel_, &info);
        if (drain != ESP_OK) return drain;
        static constexpr std::array<std::uint8_t, 256> silence{};
        std::size_t remaining = info.total_dma_buf_size;
        while (remaining) {
          const auto bytes = std::min(remaining, silence.size());
          drain = esp_codec_dev_write(speakerCodec_, const_cast<std::uint8_t*>(silence.data()), bytes);
          if (drain != ESP_OK) return drain;
          remaining -= bytes;
        }
        ESP_LOGI(kTag, "TX DMA drained before microphone: %u bytes", static_cast<unsigned>(info.total_dma_buf_size));
        closeSpeakerLocked();
      }
    }

    esp_err_t err = initCodecDevice(sampleRate, ESP_CODEC_DEV_TYPE_IN, recordCodec_);
    if (err != ESP_OK) return err;

    esp_codec_dev_sample_info_t sampleInfo = {};
    sampleInfo.bits_per_sample = 16;
    sampleInfo.channel = kHardwareAecReference ? 4 : 2;
    if constexpr (kHardwareAecReference) sampleInfo.channel_mask =
        ESP_CODEC_DEV_MAKE_CHANNEL_MASK(0) | ESP_CODEC_DEV_MAKE_CHANNEL_MASK(1);
    sampleInfo.sample_rate = static_cast<std::uint32_t>(sampleRate);
    sampleInfo.mclk_multiple = I2S_MCLK_MULTIPLE_256;
    err = esp_codec_dev_open(recordCodec_, &sampleInfo);
    if (err != ESP_OK) {
      ESP_LOGE(kTag, "Recorder open failed: %s", esp_err_to_name(err));
      return err;
    }
    recordOpen_ = true;
    // Sensitivity is board-specific: the ES7210 loopback calibration is not
    // the ES8311 analog microphone's calibration. Keep gain before AEC low
    // enough for ADC headroom, then restore its voice level after cancellation.
    esp_codec_dev_set_in_gain(recordCodec_, GEA_AUDIO_ECHO_CANCELLATION ? kAecMicGain : 45.0f);
    if constexpr (kHardwareAecReference) {
      // MIC1 needs sufficient near-end sensitivity at unity AEC output. An
      // 18 dB PGA setting adds 6 dB over the former headroom-only setting,
      // below the previously clipping 24 dB setting. MIC3 stays independent.
      err = esp_codec_dev_set_in_channel_gain(recordCodec_, ESP_CODEC_DEV_MAKE_CHANNEL_MASK(0), 18.0f);
      if (err != ESP_OK) { closeRecorderLocked(); return err; }
      // Codec gain masks use physical MIC numbering, not TDM slot numbering.
      // The board attenuates the analog loopback before MIC3. At 0 dB its
      // measured reference was only 12–114 RMS against 257–1573 RMS echo;
      // use the codec's nominal 30 dB input gain for this attenuated line.
      err = esp_codec_dev_set_in_channel_gain(recordCodec_, ESP_CODEC_DEV_MAKE_CHANNEL_MASK(2), 30.0f);
      if (err != ESP_OK) { closeRecorderLocked(); return err; }
    }
#if GEA_AUDIO_ECHO_CANCELLATION
    if (!geaAudioAecStart(kPairedAecReference, kAecOutputGain, kAecAggressiveNlp)) { closeRecorderLocked(); return ESP_ERR_NO_MEM; }
    ESP_LOGI(kTag, "Microphone AEC gain: ADC=%.1f dB output=%.3f", kAecMicGain, kAecOutputGain);
#endif
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
    }
    err = setAmplifierEnabled(speakerVolume_ > 0);
    if (err != ESP_OK) return err;

    if (!codecIf_) {
      auto i2cBus = gea::platform::i2c::Bus::primary();
      if (!i2cBus.available()) {
        ESP_LOGE(kTag, "I2C bus is not ready for ES8311");
        return ESP_ERR_INVALID_STATE;
      }

      audio_codec_i2c_cfg_t i2cConfig = {};
      i2cConfig.addr = kCodecI2cAddress;
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
    if (deviceType == ESP_CODEC_DEV_TYPE_IN && kMicAdcAddress != 0) {
      codecConfig.codec_if = micAdcIf();
      if (!codecConfig.codec_if) return ESP_ERR_NO_MEM;
    }
    codecConfig.data_if = i2sDataIf_;
    device = esp_codec_dev_new(&codecConfig);
    if (!device) return ESP_ERR_NO_MEM;
    esp_codec_set_disable_when_closed(device, false);
    return ESP_OK;
  }

  // The ES7210 codec interface, created on first use: slave mode on the shared bus, mics 1 and 2 as
  // the stereo pair (the capture task keeps mic 1).
  const audio_codec_if_t *micAdcIf() {
    if (micAdcIf_) return micAdcIf_;

    auto i2cBus = gea::platform::i2c::Bus::primary();
    audio_codec_i2c_cfg_t i2cConfig = {};
    i2cConfig.addr = static_cast<std::uint8_t>(kMicAdcAddress << 1);
    i2cConfig.bus_handle = static_cast<i2c_master_bus_handle_t>(i2cBus.nativeHandle());
    micAdcCtrlIf_ = audio_codec_new_i2c_ctrl(&i2cConfig);
    if (!micAdcCtrlIf_) return nullptr;

    es7210_codec_cfg_t es7210Config = {};
    es7210Config.ctrl_if = micAdcCtrlIf_;
    es7210Config.master_mode = false;
    es7210Config.mic_selected = ES7210_SEL_MIC1 | ES7210_SEL_MIC2;
    if constexpr (kHardwareAecReference) es7210Config.mic_selected |= ES7210_SEL_MIC3 | ES7210_SEL_MIC4;
    es7210Config.mclk_src = ES7210_MCLK_FROM_PAD;
    es7210Config.mclk_div = 256;
    micAdcIf_ = es7210_codec_new(&es7210Config);
    if (!micAdcIf_) {
      audio_codec_delete_ctrl_if(micAdcCtrlIf_);
      micAdcCtrlIf_ = nullptr;
    }
    return micAdcIf_;
  }

  static constexpr const char *kTag = "audio";

  i2s_chan_handle_t txChannel_ = nullptr;
  i2s_chan_handle_t rxChannel_ = nullptr;
  const audio_codec_data_if_t *i2sDataIf_ = nullptr;
  const audio_codec_ctrl_if_t *i2cCtrlIf_ = nullptr;
  const audio_codec_gpio_if_t *gpioIf_ = nullptr;
  const audio_codec_if_t *codecIf_ = nullptr;
  const audio_codec_ctrl_if_t *micAdcCtrlIf_ = nullptr;
  const audio_codec_if_t *micAdcIf_ = nullptr;
  esp_codec_dev_handle_t speakerCodec_ = nullptr;
  esp_codec_dev_handle_t recordCodec_ = nullptr;
  bool speakerOpen_ = false;
  bool recordOpen_ = false;
  int i2sSampleRate_ = 0;
  int speakerSampleRate_ = 0;
  int speakerChannels_ = 0;
  int speakerBitsPerSample_ = 0;
  int speakerVolume_ = 100;
  // Configuration/destruction takes write then read. A duplex read releases
  // write before blocking on RX, allowing TX to run at the same time.
  std::mutex writeMutex_;
  std::mutex readMutex_;

  TaskHandle_t captureTask_ = nullptr;
  std::vector<gea::host::NativeMediaTrackHandle> attachedTracks_;
  std::vector<gea::host::NativeMediaTrackHandle> captureSnapshot_;
  std::mutex attachedMutex_;
  // Match the 20 ms Opus input frame in either duplex mode. A 2048-sample
  // half-duplex read batches 128 ms of speech before the encoder can see it.
  static constexpr std::size_t kCaptureFrameSamples = 320;
  std::array<std::int16_t, kCaptureFrameSamples * 2> stereoFrame_{};
  std::array<std::int16_t, kCaptureFrameSamples> monoFrame_{};

  bool micInUse() {
    std::lock_guard<std::mutex> lock(attachedMutex_);
    return !attachedTracks_.empty();
  }

  static void captureTaskTrampoline(void *arg) {
    static_cast<AudioOutputDriver *>(arg)->runCaptureTask();
  }

  void runCaptureTask() {
    int consecutiveReadErrors = 0;

    while (true) {
      bool idle = false;
      {
        std::lock_guard<std::mutex> lock(attachedMutex_);
        captureSnapshot_ = attachedTracks_;
        idle = captureSnapshot_.empty();
        // Cleared under the same lock attachTrack() checks, so a track attached from here on
        // starts a fresh capture task instead of relying on this one.
        if (idle) captureTask_ = nullptr;
      }

      // The last track detached: close the mic and, unless the speaker is still playing, free the
      // I2S channels and codec devices, then end this task so its stack is freed too.
      if (idle) {
        {
          std::lock_guard<std::mutex> lock(writeMutex_);
          std::lock_guard<std::mutex> captureLock(readMutex_);
          // A track attached since the check started a new capture task, which now owns the audio path.
          if (!micInUse()) {
            closeRecorderLocked();
            if (!speakerOpen_) releaseAudioLocked();
          }
        }
#if GEA_AUDIO_ECHO_CANCELLATION
        vTaskDeleteWithCaps(nullptr);
#else
        vTaskDelete(nullptr);
#endif
        return;
      }

      int err = ESP_OK;
#if GEA_AUDIO_ECHO_CANCELLATION
      {
        // A running RX uses DMA callbacks, so TX's blocking write never owns
        // the microphone read path. Take both locks only for configuration.
        std::unique_lock<std::mutex> captureLock(readMutex_);
        if (!recordOpen_) {
          captureLock.unlock();
          std::lock_guard<std::mutex> configLock(writeMutex_);
          captureLock.lock();
          err = initRecorder(16000);
        }
        if (err == ESP_OK) {
          bool ready = false;
          for (int retry = 0; retry < 500 && !ready; ++retry) {
            ready = geaAudioAecRead(monoFrame_.data(), kCaptureFrameSamples);
            if (!ready) vTaskDelay(1);
          }
          err = ready ? ESP_OK : ESP_ERR_TIMEOUT;
        }
      }
#else
      {
        std::unique_lock<std::mutex> lock(writeMutex_);
        std::lock_guard<std::mutex> captureLock(readMutex_);
        err = initRecorder(16000);
        if (err == ESP_OK && recordCodec_) {
          if constexpr (GEA_AUDIO_FULL_DUPLEX) lock.unlock();
          err = esp_codec_dev_read(recordCodec_, stereoFrame_.data(), stereoFrame_.size() * sizeof(std::int16_t));
        }
      }
#endif
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

#if !GEA_AUDIO_ECHO_CANCELLATION
      // ES8311 input is interleaved as [mic, ref]. For local voice notes, keep
      // the real mic channel only; mixing in the ref slot makes speech metallic.
      for (std::size_t i = 0; i < framesRead; ++i) {
        monoFrame_[i] = stereoFrame_[2 * i];
      }
#endif
      const int16_t *captured = monoFrame_.data();
      size_t capturedCount = framesRead;
#if GEA_AUDIO_ECHO_CANCELLATION
      capturedCount = geaAudioAecProcess(captured, framesRead, &captured);
#endif
      for (auto handle : captureSnapshot_) {
        if (capturedCount) gea::host::media::track_inject_pcm(handle, captured, capturedCount);
      }
    }
  }

public:
  void attachTrack(gea::host::NativeMediaTrackHandle handle) {
    std::lock_guard<std::mutex> lock(attachedMutex_);
    attachedTracks_.push_back(handle);
    // Checked and set under the lock the capture task clears it with, so only one capture task runs.
    if (captureTask_ == nullptr) {
#if GEA_AUDIO_ECHO_CANCELLATION
      // Capture DSP and speaker feeding share the highest audio priority.
      // Rendering may share this core at priority 23: using that priority here
      // time-sliced DSP with rendering and missed the 32 ms frame deadline.
      const auto created = xTaskCreatePinnedToCoreWithCaps(
          captureTaskTrampoline, "gea_mic", 8192, this,
          configMAX_PRIORITIES - 1, &captureTask_, 0,
          MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
      if (created != pdPASS) {
        attachedTracks_.pop_back();
        ESP_LOGE(kTag, "Microphone task allocation failed");
        throw std::runtime_error("Microphone task allocation failed");
      }
#else
      xTaskCreatePinnedToCore(captureTaskTrampoline, "gea_mic", 8192, this, 5, &captureTask_, 1);
#endif
    }
  }

  void detachTrack(gea::host::NativeMediaTrackHandle handle) {
    std::lock_guard<std::mutex> lock(attachedMutex_);
    attachedTracks_.erase(std::remove(attachedTracks_.begin(), attachedTracks_.end(), handle), attachedTracks_.end());
  }
};

}  // namespace gea::platform::esp32::chip_bindings::es8311

double gea::platform::audio::OutputDriver::outputLatency(int sampleRate) {
  return sampleRate > 0 ? double(GEA_AUDIO_DMA_DESCRIPTORS * GEA_AUDIO_DMA_FRAMES) / sampleRate : 0;
}

bool gea::platform::audio::OutputDriver::open(int sampleRate, int channels, int bitsPerSample) {
  return gea::platform::esp32::chip_bindings::es8311::AudioOutputDriver::instance().open(sampleRate, channels, bitsPerSample);
}

bool gea::platform::audio::OutputDriver::write(const std::int16_t *pcm, std::size_t sampleCount, int timeoutMs) {
  return gea::platform::esp32::chip_bindings::es8311::AudioOutputDriver::instance().write(pcm, sampleCount, timeoutMs);
}

void gea::platform::audio::OutputDriver::close() {
  gea::platform::esp32::chip_bindings::es8311::AudioOutputDriver::instance().close();
}
bool gea::platform::audio::OutputDriver::flush() {
  return gea::platform::esp32::chip_bindings::es8311::AudioOutputDriver::instance().flush();
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
