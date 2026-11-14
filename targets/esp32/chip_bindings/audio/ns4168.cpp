// NS4168 I2S class-D amplifier, plus an optional separate I2S MEMS microphone
// (INMP441-style: 24-bit left-justified samples in a 32-bit left slot).
//
// There is no codec to configure: the S3 is the I2S master and streams PCM to
// the amplifier, which decodes it itself. So the speaker path is one TX
// channel, the volume is applied to the samples in software, and the only
// control lines are the amplifier's shutdown and mute, which a board may route
// through its I/O expander (board::audio.expanderShutdown/expanderMute: high
// shuts down / mutes, as on the Elecrow CrowPanel Advance). The microphone,
// when the board has one, runs on the other I2S controller and its own pins, so
// capture and playback never contend for a clock. It is a standard-I2S MEMS
// part or a PDM one (board::audio.micFormat); the S3 receives PDM only on I2S0,
// so the speaker always takes I2S1. Format 2 asks the board controller, for
// boards whose revisions differ only in the microphone (CrowPanel V1.0 has an
// INMP441, V1.3+ a PDM part).

#define GEA_AUDIO_DRIVER_INTERNAL 1
#include "audio.h"
#include "board.h"
#if GEA_BOARD_HAS_EXPANDER
#include "chip_bindings/expanders/io_expander.h"
#endif

#include <algorithm>
#include <array>
#include <cstddef>
#include <cstdint>
#include <mutex>
#include <string_view>
#include <vector>

#include "driver/i2s_pdm.h"
#include "driver/i2s_std.h"
#include "esp_heap_caps.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "host/media.h"

#ifndef GEA_AUDIO_DMA_DESCRIPTORS
#define GEA_AUDIO_DMA_DESCRIPTORS 6
#endif
#ifndef GEA_AUDIO_DMA_FRAMES
#define GEA_AUDIO_DMA_FRAMES 240
#endif
static_assert(GEA_AUDIO_DMA_DESCRIPTORS >= 2);
static_assert(GEA_AUDIO_DMA_FRAMES >= 8 && GEA_AUDIO_DMA_FRAMES <= 511);

namespace gea::platform::esp32::chip_bindings::ns4168 {
namespace {

constexpr char kTag[] = "ns4168";
constexpr auto &kAudio = gea::platform::board::audio;
constexpr bool kHasMicrophone = kAudio.micBclk != GPIO_NUM_NC && kAudio.micDin != GPIO_NUM_NC;
constexpr i2s_port_t kSpeakerPort = I2S_NUM_1;
constexpr i2s_port_t kMicPort = I2S_NUM_0;
// The INMP441's 24 significant bits sit at the top of the 32-bit slot; keeping
// bits 29..14 gives ~12 dB of headroom over the raw top 16 bits, which is where
// speech from arm's length lands for this part.
constexpr int kMicShift = 14;
constexpr std::size_t kMicFrames = 256;

bool micIsPdm() {
	if (kAudio.micFormat == 1) return true;
	if (kAudio.micFormat == 0) return false;
#if GEA_BOARD_HAS_EXPANDER
	auto &expander = gea::platform::esp32::chip_bindings::expanders::ioExpander();
	expander.init();
	return std::string_view(gea::platform::esp32::chip_bindings::expanders::ioExpanderDriverName()) == "elecrow-stc8h";
#else
	return false;
#endif
}

bool setExpanderLine(int pin, bool high) {
#if GEA_BOARD_HAS_EXPANDER
	if (pin < 0) return true;
	return gea::platform::esp32::chip_bindings::expanders::ioExpander().writePin(pin, high);
#else
	(void)pin;
	(void)high;
	return true;
#endif
}

class AudioDriver {
public:
	static AudioDriver &instance() {
		// Heap-allocated on first use, so an app that never plays sound pays nothing.
		static AudioDriver *const driver = new AudioDriver();
		return *driver;
	}

	bool open(int sampleRate, int channels, int bitsPerSample) {
		std::lock_guard<std::mutex> lock(writeMutex_);
		if (bitsPerSample != 16 || (channels != 1 && channels != 2) || sampleRate < 8000 || sampleRate > 48000) return false;
		if (tx_ && sampleRate == sampleRate_ && channels == channels_) return true;
		closeLocked();

		i2s_chan_config_t channelConfig = I2S_CHANNEL_DEFAULT_CONFIG(kSpeakerPort, I2S_ROLE_MASTER);
		channelConfig.auto_clear = true;  // an underrun plays silence, not the last buffer again
		channelConfig.dma_desc_num = GEA_AUDIO_DMA_DESCRIPTORS;
		channelConfig.dma_frame_num = GEA_AUDIO_DMA_FRAMES;
		esp_err_t err = i2s_new_channel(&channelConfig, &tx_, nullptr);
		if (err != ESP_OK) {
			ESP_LOGE(kTag, "I2S TX channel: %s", esp_err_to_name(err));
			tx_ = nullptr;
			return false;
		}
		// Mono PCM goes out on both slots, so the amplifier plays it whichever
		// channel its CTRL strap selects.
		i2s_std_config_t config{};
		config.clk_cfg = I2S_STD_CLK_DEFAULT_CONFIG(static_cast<std::uint32_t>(sampleRate));
		config.slot_cfg = I2S_STD_PHILIPS_SLOT_DEFAULT_CONFIG(I2S_DATA_BIT_WIDTH_16BIT,
		                                                      channels == 1 ? I2S_SLOT_MODE_MONO : I2S_SLOT_MODE_STEREO);
		config.gpio_cfg.mclk = GPIO_NUM_NC;
		config.gpio_cfg.bclk = kAudio.bclk;
		config.gpio_cfg.ws = kAudio.ws;
		config.gpio_cfg.dout = kAudio.dout;
		config.gpio_cfg.din = GPIO_NUM_NC;
		err = i2s_channel_init_std_mode(tx_, &config);
		if (err == ESP_OK) err = i2s_channel_enable(tx_);
		if (err != ESP_OK) {
			ESP_LOGE(kTag, "I2S TX init: %s", esp_err_to_name(err));
			i2s_del_channel(tx_);
			tx_ = nullptr;
			return false;
		}
		setExpanderLine(kAudio.expanderMute, false);
		setExpanderLine(kAudio.expanderShutdown, false);
		sampleRate_ = sampleRate;
		channels_ = channels;
		ESP_LOGI(kTag, "speaker ready sample_rate=%d channels=%d volume=%d", sampleRate, channels, volume_);
		return true;
	}

	bool write(const std::int16_t *pcm, std::size_t sampleCount, int timeoutMs) {
		if (!pcm || sampleCount == 0) return false;
		std::lock_guard<std::mutex> lock(writeMutex_);
		if (!tx_) return false;
		const TickType_t timeout = timeoutMs < 0 ? portMAX_DELAY : pdMS_TO_TICKS(timeoutMs);
		const int gain = gainQ15();
		while (sampleCount > 0) {
			const std::size_t count = std::min(sampleCount, scaled_.size());
			for (std::size_t i = 0; i < count; ++i) scaled_[i] = static_cast<std::int16_t>((pcm[i] * gain) >> 15);
			std::size_t written = 0;
			if (i2s_channel_write(tx_, scaled_.data(), count * sizeof(std::int16_t), &written, timeout) != ESP_OK) return false;
			const std::size_t samples = written / sizeof(std::int16_t);
			if (samples == 0) return false;
			pcm += samples;
			sampleCount -= samples;
		}
		return true;
	}

	void close() {
		std::lock_guard<std::mutex> lock(writeMutex_);
		closeLocked();
	}

	bool flush() {
		std::lock_guard<std::mutex> lock(writeMutex_);
		if (!tx_) return true;
		// Disabling drops what is queued; re-enabling starts from silence
		// (auto_clear), which is what a seek or stop wants.
		if (i2s_channel_disable(tx_) != ESP_OK) return false;
		return i2s_channel_enable(tx_) == ESP_OK;
	}

	int volume() const { return volume_; }
	void setVolume(int percent) { volume_ = std::clamp(percent, 0, 100); }

	void attachTrack(gea::host::NativeMediaTrackHandle handle) {
		if constexpr (!kHasMicrophone) {
			(void)handle;
			return;
		}
		std::lock_guard<std::mutex> lock(trackMutex_);
		tracks_.push_back(handle);
		if (captureTask_) return;
		if (xTaskCreatePinnedToCoreWithCaps(captureEntry, "gea_mic", 6144, this, 5, &captureTask_, 1,
		                                    MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT) != pdPASS) {
			captureTask_ = nullptr;
			tracks_.pop_back();
			ESP_LOGE(kTag, "microphone task allocation failed");
		}
	}

	void detachTrack(gea::host::NativeMediaTrackHandle handle) {
		std::lock_guard<std::mutex> lock(trackMutex_);
		tracks_.erase(std::remove(tracks_.begin(), tracks_.end(), handle), tracks_.end());
	}

private:
	AudioDriver() = default;

	// Perceptual volume: 50% is about -12 dB, 0% is silent.
	int gainQ15() const {
		const int v = volume_;
		return (v * v * 32767) / 10000;
	}

	void closeLocked() {
		if (!tx_) return;
		setExpanderLine(kAudio.expanderShutdown, true);
		i2s_channel_disable(tx_);
		i2s_del_channel(tx_);
		tx_ = nullptr;
		sampleRate_ = 0;
		channels_ = 0;
	}

	static void captureEntry(void *arg) { static_cast<AudioDriver *>(arg)->runCapture(); }

	void runCapture() {
		i2s_chan_handle_t rx = nullptr;
		i2s_chan_config_t channelConfig = I2S_CHANNEL_DEFAULT_CONFIG(kMicPort, I2S_ROLE_MASTER);
		channelConfig.dma_desc_num = GEA_AUDIO_DMA_DESCRIPTORS;
		channelConfig.dma_frame_num = GEA_AUDIO_DMA_FRAMES;
		const bool pdm = micIsPdm();
		esp_err_t err = i2s_new_channel(&channelConfig, nullptr, &rx);
		if (err == ESP_OK && pdm) {
			// PDM: the S3 converts to 16-bit PCM in hardware.
			i2s_pdm_rx_config_t config{};
			config.clk_cfg = I2S_PDM_RX_CLK_DEFAULT_CONFIG(static_cast<std::uint32_t>(gea::platform::audio::deviceSampleRate));
			config.slot_cfg = I2S_PDM_RX_SLOT_DEFAULT_CONFIG(I2S_DATA_BIT_WIDTH_16BIT, I2S_SLOT_MODE_MONO);
			config.gpio_cfg.clk = kAudio.micBclk;
			config.gpio_cfg.din = kAudio.micDin;
			err = i2s_channel_init_pdm_rx_mode(rx, &config);
			if (err == ESP_OK) err = i2s_channel_enable(rx);
		} else if (err == ESP_OK) {
			i2s_std_config_t config{};
			config.clk_cfg = I2S_STD_CLK_DEFAULT_CONFIG(static_cast<std::uint32_t>(gea::platform::audio::deviceSampleRate));
			config.slot_cfg = I2S_STD_PHILIPS_SLOT_DEFAULT_CONFIG(I2S_DATA_BIT_WIDTH_32BIT, I2S_SLOT_MODE_MONO);
			config.slot_cfg.slot_mask = I2S_STD_SLOT_LEFT;  // L/R strapped low
			config.gpio_cfg.mclk = GPIO_NUM_NC;
			config.gpio_cfg.bclk = kAudio.micBclk;
			config.gpio_cfg.ws = kAudio.micWs;
			config.gpio_cfg.dout = GPIO_NUM_NC;
			config.gpio_cfg.din = kAudio.micDin;
			err = i2s_channel_init_std_mode(rx, &config);
			if (err == ESP_OK) err = i2s_channel_enable(rx);
		}
		if (err != ESP_OK) ESP_LOGE(kTag, "microphone I2S init: %s", esp_err_to_name(err));
		else ESP_LOGI(kTag, "microphone ready (%s) sample_rate=%d", pdm ? "PDM" : "I2S", gea::platform::audio::deviceSampleRate);

		std::array<std::int32_t, kMicFrames> raw{};
		std::array<std::int16_t, kMicFrames> pcm{};
		std::vector<gea::host::NativeMediaTrackHandle> snapshot;
		while (err == ESP_OK) {
			{
				std::lock_guard<std::mutex> lock(trackMutex_);
				if (tracks_.empty()) break;
				snapshot = tracks_;
			}
			std::size_t bytes = 0;
			std::size_t frames = 0;
			if (pdm) {
				if (i2s_channel_read(rx, pcm.data(), sizeof(pcm), &bytes, pdMS_TO_TICKS(200)) != ESP_OK) continue;
				frames = bytes / sizeof(std::int16_t);
			} else {
				if (i2s_channel_read(rx, raw.data(), sizeof(raw), &bytes, pdMS_TO_TICKS(200)) != ESP_OK) continue;
				frames = bytes / sizeof(std::int32_t);
				for (std::size_t i = 0; i < frames; ++i) {
					const std::int32_t sample = raw[i] >> kMicShift;
					pcm[i] = static_cast<std::int16_t>(std::clamp<std::int32_t>(sample, -32768, 32767));
				}
			}
			for (auto handle : snapshot) gea::host::media::track_inject_pcm(handle, pcm.data(), frames);
		}
		if (rx) {
			i2s_channel_disable(rx);
			i2s_del_channel(rx);
		}
		{
			std::lock_guard<std::mutex> lock(trackMutex_);
			captureTask_ = nullptr;
			// A track attached after the loop saw none starts its own task.
			if (!tracks_.empty()) attachTrackRestart();
		}
		vTaskDeleteWithCaps(nullptr);
	}

	void attachTrackRestart() {
		if (xTaskCreatePinnedToCoreWithCaps(captureEntry, "gea_mic", 6144, this, 5, &captureTask_, 1,
		                                    MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT) != pdPASS) {
			captureTask_ = nullptr;
		}
	}

	std::mutex writeMutex_;
	i2s_chan_handle_t tx_ = nullptr;
	int sampleRate_ = 0;
	int channels_ = 0;
	int volume_ = 70;
	std::array<std::int16_t, 512> scaled_{};

	std::mutex trackMutex_;
	std::vector<gea::host::NativeMediaTrackHandle> tracks_;
	TaskHandle_t captureTask_ = nullptr;
};

}  // namespace
}  // namespace gea::platform::esp32::chip_bindings::ns4168

using gea::platform::esp32::chip_bindings::ns4168::AudioDriver;

double gea::platform::audio::OutputDriver::outputLatency(int sampleRate) {
	return sampleRate > 0 ? double(GEA_AUDIO_DMA_DESCRIPTORS * GEA_AUDIO_DMA_FRAMES) / sampleRate : 0;
}
bool gea::platform::audio::OutputDriver::open(int sampleRate, int channels, int bitsPerSample) {
	return AudioDriver::instance().open(sampleRate, channels, bitsPerSample);
}
bool gea::platform::audio::OutputDriver::write(const std::int16_t *pcm, std::size_t sampleCount, int timeoutMs) {
	return AudioDriver::instance().write(pcm, sampleCount, timeoutMs);
}
void gea::platform::audio::OutputDriver::close() { AudioDriver::instance().close(); }
bool gea::platform::audio::OutputDriver::flush() { return AudioDriver::instance().flush(); }
int gea::platform::audio::OutputDriver::volume() { return AudioDriver::instance().volume(); }
void gea::platform::audio::OutputDriver::setVolume(int volumePercent) { AudioDriver::instance().setVolume(volumePercent); }

namespace gea::host::media {
void platform_attach_track(NativeMediaTrackHandle handle) { AudioDriver::instance().attachTrack(handle); }
void platform_detach_track(NativeMediaTrackHandle handle) { AudioDriver::instance().detachTrack(handle); }
}  // namespace gea::host::media
