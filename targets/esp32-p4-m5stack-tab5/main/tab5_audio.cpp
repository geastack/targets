#define GEA_AUDIO_DRIVER_INTERNAL 1
#include "audio.h"
#include "board.h"
#include "i2c.h"
#include "tab5_drivers.h"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <mutex>

#include "driver/i2c_master.h"
#include "driver/i2s_std.h"
#include "esp_codec_dev.h"
#include "esp_codec_dev_defaults.h"
#include "esp_log.h"

namespace {

constexpr const char *kTag = "tab5_audio";
constexpr int kI2sPort = gea::platform::board::audio.i2sPort;
constexpr int kCodecDataPort = 0;
constexpr gpio_num_t kMclkPin = gea::platform::board::audio.mclk;
constexpr gpio_num_t kBclkPin = gea::platform::board::audio.bclk;
constexpr gpio_num_t kWsPin = gea::platform::board::audio.ws;
constexpr gpio_num_t kDoutPin = gea::platform::board::audio.dout;
constexpr gpio_num_t kDinPin = gea::platform::board::audio.din;
constexpr gpio_num_t kPaPin = gea::platform::board::audio.powerAmplifier;

class Tab5AudioOutputDriver {
public:
	static Tab5AudioOutputDriver &instance()
	{
		static Tab5AudioOutputDriver driver;
		return driver;
	}

	bool open(int sampleRate, int channels, int bitsPerSample)
	{
		std::lock_guard<std::mutex> lock(mutex_);
		if (!isSupportedFormat(channels, bitsPerSample)) {
			ESP_LOGE(kTag, "Unsupported output format sample_rate=%d channels=%d bits=%d", sampleRate, channels, bitsPerSample);
			return false;
		}

		if (speakerOpen_ &&
			speakerSampleRate_ == sampleRate &&
			speakerChannels_ == channels &&
			speakerBitsPerSample_ == bitsPerSample) {
			applyVolumeLocked();
			return true;
		}

		if (speakerOpen_) closeSpeakerLocked();

		esp_err_t err = initSpeakerLocked();
		if (err != ESP_OK) {
			ESP_LOGE(kTag, "Speaker init failed: %s", esp_err_to_name(err));
			return false;
		}

		esp_codec_dev_sample_info_t sampleInfo = {};
		sampleInfo.bits_per_sample = static_cast<std::uint8_t>(bitsPerSample);
		sampleInfo.channel = static_cast<std::uint8_t>(channels);
		sampleInfo.sample_rate = static_cast<std::uint32_t>(sampleRate);
		sampleInfo.mclk_multiple = I2S_MCLK_MULTIPLE_256;
		err = esp_codec_dev_open(speakerCodec_, &sampleInfo);
		if (err != ESP_OK) {
			ESP_LOGE(kTag, "Speaker open failed: %s", esp_err_to_name(err));
			return false;
		}

		if (!gea::platform::tab5::setSpeakerEnabled(true)) {
			ESP_LOGW(kTag, "Speaker enable via IO expander failed");
		}
		applyVolumeLocked();
		speakerOpen_ = true;
		speakerSampleRate_ = sampleRate;
		speakerChannels_ = channels;
		speakerBitsPerSample_ = bitsPerSample;

		ESP_LOGI(kTag,
			"Speaker ready sample_rate=%d channels=%d bits=%d volume=%d codec_volume=%d",
			sampleRate,
			channels,
			bitsPerSample,
			speakerVolume_,
			codecVolumeFromUserPercent(speakerVolume_));
		return true;
	}

	bool write(const std::int16_t *pcm, std::size_t sampleCount, int timeoutMs)
	{
		(void)timeoutMs;
		if (!pcm || sampleCount == 0) return false;

		std::lock_guard<std::mutex> lock(mutex_);
		if (!speakerCodec_ || !speakerOpen_) return false;

		auto *cursor = reinterpret_cast<const std::uint8_t *>(pcm);
		std::size_t remaining = sampleCount * sizeof(std::int16_t);
		while (remaining > 0) {
			const int bytesToWrite = static_cast<int>(std::min<std::size_t>(remaining, 4096));
			const int err = esp_codec_dev_write(speakerCodec_, const_cast<std::uint8_t *>(cursor), bytesToWrite);
			if (err != ESP_OK) {
				ESP_LOGW(kTag, "Speaker write failed: %s", esp_err_to_name(err));
				return false;
			}
			cursor += bytesToWrite;
			remaining -= static_cast<std::size_t>(bytesToWrite);
		}
		return true;
	}

	void close()
	{
		std::lock_guard<std::mutex> lock(mutex_);
		closeSpeakerLocked();
	}

	int volume() const
	{
		return speakerVolume_;
	}

	void setVolume(int volumePercent)
	{
		if (volumePercent < 0) volumePercent = 0;
		if (volumePercent > 100) volumePercent = 100;

		std::lock_guard<std::mutex> lock(mutex_);
		speakerVolume_ = volumePercent;
		applyVolumeLocked();
	}

private:
	static bool isSupportedFormat(int channels, int bitsPerSample)
	{
		return (channels == 1 || channels == 2) && bitsPerSample == 16;
	}

	static int codecVolumeFromUserPercent(int volumePercent)
	{
		if (volumePercent <= 0) return 0;
		if (volumePercent >= 100) return 100;

		const double amplitude = static_cast<double>(volumePercent) / 100.0;
		const double db = 20.0 * std::log10(amplitude);
		int codecVolume = static_cast<int>(std::lround(((db + 50.0) * 100.0) / 50.0));
		if (codecVolume < 1) codecVolume = 1;
		if (codecVolume > 100) codecVolume = 100;
		return codecVolume;
	}

	void applyVolumeLocked()
	{
		if (!speakerCodec_) return;
		const int codecVolume = codecVolumeFromUserPercent(speakerVolume_);
		const int err = esp_codec_dev_set_out_vol(speakerCodec_, codecVolume);
		if (err != ESP_CODEC_DEV_OK) {
			ESP_LOGW(kTag, "Speaker volume set failed volume=%d codec_volume=%d err=%d", speakerVolume_, codecVolume, err);
		}
	}

	void closeSpeakerLocked()
	{
		if (speakerCodec_ && speakerOpen_) {
			esp_codec_dev_close(speakerCodec_);
		}
		(void)gea::platform::tab5::setSpeakerEnabled(false);
		speakerOpen_ = false;
		speakerSampleRate_ = 0;
		speakerChannels_ = 0;
		speakerBitsPerSample_ = 0;
	}

	void releaseI2sLocked()
	{
		if (txChannel_) {
			i2s_channel_disable(txChannel_);
			i2s_del_channel(txChannel_);
			txChannel_ = nullptr;
		}
		i2sDataIf_ = nullptr;
	}

	esp_err_t initI2sLocked()
	{
		if (txChannel_ && i2sDataIf_) return ESP_OK;

		i2s_chan_config_t channelConfig = I2S_CHANNEL_DEFAULT_CONFIG(kI2sPort, I2S_ROLE_MASTER);
		channelConfig.auto_clear = true;
		channelConfig.dma_desc_num = 6;
		channelConfig.dma_frame_num = 240;
		esp_err_t err = i2s_new_channel(&channelConfig, &txChannel_, nullptr);
		if (err != ESP_OK) {
			ESP_LOGE(kTag, "I2S channel creation failed: %s", esp_err_to_name(err));
			return err;
		}

		i2s_std_config_t stdConfig = {};
		stdConfig.clk_cfg = I2S_STD_CLK_DEFAULT_CONFIG(16000);
		stdConfig.clk_cfg.mclk_multiple = I2S_MCLK_MULTIPLE_256;
		stdConfig.slot_cfg = I2S_STD_PHILIPS_SLOT_DEFAULT_CONFIG(I2S_DATA_BIT_WIDTH_16BIT, I2S_SLOT_MODE_STEREO);
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
			releaseI2sLocked();
			return err;
		}

		err = i2s_channel_enable(txChannel_);
		if (err != ESP_OK) {
			ESP_LOGE(kTag, "I2S enable failed: %s", esp_err_to_name(err));
			releaseI2sLocked();
			return err;
		}

		audio_codec_i2s_cfg_t i2sConfig = {};
		i2sConfig.port = kCodecDataPort;
		i2sConfig.tx_handle = txChannel_;
		i2sDataIf_ = audio_codec_new_i2s_data(&i2sConfig);
		if (!i2sDataIf_) {
			ESP_LOGE(kTag, "I2S codec data interface creation failed");
			releaseI2sLocked();
			return ESP_ERR_NO_MEM;
		}

		return ESP_OK;
	}

	esp_err_t initCodecIfLocked()
	{
		if (codecIf_) return ESP_OK;

		if (!gea::platform::tab5::initIoExpanders()) {
			ESP_LOGE(kTag, "IO expander init failed");
			return ESP_ERR_INVALID_STATE;
		}

		auto i2cBus = gea::platform::i2c::Bus::primary();
		if (!i2cBus.available()) {
			ESP_LOGE(kTag, "I2C bus is not ready for ES8388");
			return ESP_ERR_INVALID_STATE;
		}

		audio_codec_i2c_cfg_t i2cConfig = {};
		i2cConfig.addr = ES8388_CODEC_DEFAULT_ADDR;
		i2cConfig.bus_handle = static_cast<i2c_master_bus_handle_t>(i2cBus.nativeHandle());
		i2cCtrlIf_ = audio_codec_new_i2c_ctrl(&i2cConfig);
		if (!i2cCtrlIf_) return ESP_ERR_NO_MEM;

		gpioIf_ = audio_codec_new_gpio();
		if (!gpioIf_) return ESP_ERR_NO_MEM;

		esp_codec_dev_hw_gain_t gain = {};
		gain.pa_voltage = 5.0;
		gain.codec_dac_voltage = 3.3;

		es8388_codec_cfg_t codecConfig = {};
		codecConfig.ctrl_if = i2cCtrlIf_;
		codecConfig.gpio_if = gpioIf_;
		codecConfig.codec_mode = ESP_CODEC_DEV_WORK_MODE_DAC;
		codecConfig.pa_pin = kPaPin;
		codecConfig.pa_reverted = false;
		codecConfig.master_mode = false;
		codecConfig.hw_gain = gain;

		codecIf_ = es8388_codec_new(&codecConfig);
		if (!codecIf_) return ESP_ERR_NO_MEM;
		return ESP_OK;
	}

	esp_err_t initSpeakerLocked()
	{
		esp_err_t err = initI2sLocked();
		if (err != ESP_OK) return err;
		err = initCodecIfLocked();
		if (err != ESP_OK) return err;
		if (speakerCodec_) return ESP_OK;

		esp_codec_dev_cfg_t codecDevConfig = {};
		codecDevConfig.dev_type = ESP_CODEC_DEV_TYPE_OUT;
		codecDevConfig.codec_if = codecIf_;
		codecDevConfig.data_if = i2sDataIf_;
		speakerCodec_ = esp_codec_dev_new(&codecDevConfig);
		if (!speakerCodec_) return ESP_ERR_NO_MEM;
		esp_codec_set_disable_when_closed(speakerCodec_, true);
		return ESP_OK;
	}

	i2s_chan_handle_t txChannel_ = nullptr;
	const audio_codec_data_if_t *i2sDataIf_ = nullptr;
	const audio_codec_ctrl_if_t *i2cCtrlIf_ = nullptr;
	const audio_codec_gpio_if_t *gpioIf_ = nullptr;
	const audio_codec_if_t *codecIf_ = nullptr;
	esp_codec_dev_handle_t speakerCodec_ = nullptr;
	bool speakerOpen_ = false;
	int speakerSampleRate_ = 0;
	int speakerChannels_ = 0;
	int speakerBitsPerSample_ = 0;
	int speakerVolume_ = 100;
	std::mutex mutex_;
};

}  // namespace

namespace gea::platform::audio {

bool OutputDriver::open(int sampleRate, int channels, int bitsPerSample)
{
	return Tab5AudioOutputDriver::instance().open(sampleRate, channels, bitsPerSample);
}

bool OutputDriver::write(const std::int16_t *pcm, std::size_t sampleCount, int timeoutMs)
{
	return Tab5AudioOutputDriver::instance().write(pcm, sampleCount, timeoutMs);
}

void OutputDriver::close()
{
	Tab5AudioOutputDriver::instance().close();
}

int OutputDriver::volume()
{
	return Tab5AudioOutputDriver::instance().volume();
}

void OutputDriver::setVolume(int volumePercent)
{
	Tab5AudioOutputDriver::instance().setVolume(volumePercent);
}

}  // namespace gea::platform::audio
