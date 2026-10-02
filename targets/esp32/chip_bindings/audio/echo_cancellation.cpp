// Official Espressif ESP-SR AEC; adapter only, no custom cancellation DSP.
#include "echo_cancellation.h"
#include "esp_aec.h"
#include "esp_heap_caps.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include <algorithm>
#include <atomic>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <mutex>

namespace {
constexpr uint32_t kMaxFrame = 512, kDmaFrames = 240, kCaptureRing = 1024;
portMUX_TYPE dmaMux = portMUX_INITIALIZER_UNLOCKED;
int16_t dmaMic[kCaptureRing]{}, dmaRef[kCaptureRing]{};
int16_t previousMic[kDmaFrames]{}, latestTx[kDmaFrames]{}, captureRef[320]{};
uint32_t dmaWritten = 0, dmaRead = 0, dmaDrops = 0;
bool dmaActive = false, previousMicReady = false;
bool hardwareReference = false;
float outputGain = 1.0f;
std::mutex aecMutex;
aec_handle_t *aec = nullptr;
alignas(16) int16_t mic[kMaxFrame], ref[kMaxFrame], clean[kMaxFrame], linear[kMaxFrame],
    output[kMaxFrame * 2];
size_t fill = 0, frameSize = 0;

uint64_t rawEnergy = 0, cleanEnergy = 0, linearEnergy = 0, refEnergy = 0, totalSamples = 0;
int64_t logAt = 0, processUs = 0;
uint32_t processedFrames = 0, clippedSamples = 0, clippedReferenceSamples = 0;
int rawPeak = 0, cleanPeak = 0, referencePeak = 0;
} // namespace

bool geaAudioAecStart(bool useHardwareReference, float gain, bool aggressiveNlp) {
  std::lock_guard<std::mutex> lock(aecMutex);
  if (aec)
    return true;
  if (!std::isfinite(gain) || gain <= 0)
    return false;
  outputGain = gain;
  aec_config_t config{};
  config.mic_num = 1;
  config.ref_num = 1;
  config.out_num = 1;
  config.filter_length = 4;
  config.sample_rate = 16000;
  config.caps = MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT;
  config.mode = AEC_MODE_FD_HIGH_PERF;
  // Select residual echo suppression independently of the output gain.
  config.nlp_level = aggressiveNlp ? AEC_NLP_LEVEL_AGGR : AEC_NLP_LEVEL_NORMAL;
  aec = aec_create_from_config(&config);
  if (!aec) {
    ESP_LOGE("gea_aec", "ESP-SR AEC allocation failed");
    return false;
  }
  frameSize = aec_get_chunksize(aec);
  if (!frameSize || frameSize > kMaxFrame) {
    ESP_LOGE("gea_aec", "Unsupported AEC frame size %u", unsigned(frameSize));
    aec_destroy(aec);
    aec = nullptr;
    return false;
  }
  fill = 0;
  portENTER_CRITICAL(&dmaMux);
  dmaWritten = dmaRead = dmaDrops = 0;
  previousMicReady = false;
  hardwareReference = useHardwareReference;
  std::memset(latestTx, 0, sizeof(latestTx));
  dmaActive = true;
  portEXIT_CRITICAL(&dmaMux);
  rawEnergy = cleanEnergy = linearEnergy = refEnergy = totalSamples = 0;
  processUs = processedFrames = clippedSamples = clippedReferenceSamples = 0;
  rawPeak = cleanPeak = referencePeak = 0;
  logAt = esp_timer_get_time();
  ESP_LOGI("gea_aec",
           "ESP-SR full-duplex high-performance AEC ready: rate=16000 frame=%u NLP=%s "
           "reference=%s internal_free=%u",
           unsigned(frameSize), aggressiveNlp ? "aggressive" : "normal",
           hardwareReference ? "ADC" : "TX-DMA",
           unsigned(heap_caps_get_free_size(MALLOC_CAP_INTERNAL)));
  return true;
}

void geaAudioAecStop() {
  std::lock_guard<std::mutex> lock(aecMutex);
  portENTER_CRITICAL(&dmaMux);
  dmaActive = false;
  portEXIT_CRITICAL(&dmaMux);
  if (aec) {
    aec_destroy(aec);
    aec = nullptr;
  }
  fill = 0;
}

// The RX/TX callbacks run on the I2S clock, independently of task scheduling.
// Retain one RX block so the reference always precedes its acoustic echo.
void geaAudioAecReference(const int16_t *pcm, size_t count) {
  portENTER_CRITICAL_ISR(&dmaMux);
  if (dmaActive && count == kDmaFrames)
    std::memcpy(latestTx, pcm, sizeof(latestTx));
  else if (dmaActive && count == kDmaFrames * 2) {
    for (size_t i = 0; i < kDmaFrames; ++i)
      latestTx[i] = int16_t((int32_t(pcm[i * 2]) + pcm[i * 2 + 1]) / 2);
  }
  portEXIT_CRITICAL_ISR(&dmaMux);
}
void geaAudioAecReceive(const int16_t *stereo, size_t count) {
  portENTER_CRITICAL_ISR(&dmaMux);
  if (dmaActive && count == kDmaFrames * 2) {
    if (hardwareReference) {
      // ES7210 analog loopback or ES8311 ADCL + DACR arrive in the same RX
      // frame. Keep them paired across callback/task scheduling jitter.
      if (dmaWritten - dmaRead + kDmaFrames > kCaptureRing) {
        dmaRead = dmaWritten;
        ++dmaDrops;
      }
      for (size_t i = 0; i < kDmaFrames; ++i) {
        dmaMic[(dmaWritten + i) % kCaptureRing] = stereo[i * 2];
        dmaRef[(dmaWritten + i) % kCaptureRing] = stereo[i * 2 + 1];
      }
      dmaWritten += kDmaFrames;
      portEXIT_CRITICAL_ISR(&dmaMux);
      return;
    }
    if (previousMicReady) {
      if (dmaWritten - dmaRead + kDmaFrames > kCaptureRing) {
        dmaRead = dmaWritten;
        ++dmaDrops;
      }
      for (size_t i = 0; i < kDmaFrames; ++i) {
        dmaMic[(dmaWritten + i) % kCaptureRing] = previousMic[i];
        dmaRef[(dmaWritten + i) % kCaptureRing] = latestTx[i];
      }
      dmaWritten += kDmaFrames;
    }
    for (size_t i = 0; i < kDmaFrames; ++i)
      previousMic[i] = stereo[i * 2];
    previousMicReady = true;
  }
  portEXIT_CRITICAL_ISR(&dmaMux);
}
bool geaAudioAecRead(int16_t *pcm, size_t count) {
  if (count > 320)
    return false;
  portENTER_CRITICAL(&dmaMux);
  const bool ready = dmaActive && dmaWritten - dmaRead >= count;
  if (ready) {
    for (size_t i = 0; i < count; ++i) {
      pcm[i] = dmaMic[(dmaRead + i) % kCaptureRing];
      captureRef[i] = dmaRef[(dmaRead + i) % kCaptureRing];
    }
    dmaRead += count;
  }
  portEXIT_CRITICAL(&dmaMux);
  return ready;
}

size_t geaAudioAecProcess(const int16_t *pcm, size_t count,
                          const int16_t **result) {
  std::lock_guard<std::mutex> lock(aecMutex);
  *result = output;
  if (!aec)
    return 0; // Never silently forward uncancelled audio after an AEC failure.
  size_t produced = 0;
  for (size_t i = 0; i < count; ++i) {
    mic[fill] = pcm[i];
    ref[fill] = captureRef[i];
    if (++fill == frameSize) {
      for (size_t j = 0; j < frameSize; ++j) {
        rawPeak = std::max(rawPeak, std::abs(int(mic[j])));
        referencePeak = std::max(referencePeak, std::abs(int(ref[j])));
        clippedSamples += std::abs(int(mic[j])) >= 32760;
        clippedReferenceSamples += std::abs(int(ref[j])) >= 32760;
        rawEnergy += int64_t(mic[j]) * mic[j];
        refEnergy += int64_t(ref[j]) * ref[j];
      }
      auto before = esp_timer_get_time();
      // ESP-SR's documented two-stage API preserves both cancellation stages.
      // Snapshot the linear result so telemetry can identify suppression of
      // near-end speech separately from cancellation of the speaker echo.
      aec_linear_process(aec, mic, ref, clean);
      std::memcpy(linear, clean, frameSize * sizeof(int16_t));
      aec_nlp_process(aec, clean);
      processUs += esp_timer_get_time() - before;
      ++processedFrames;
      for (size_t j = 0; j < frameSize; ++j) {
        linearEnergy += int64_t(linear[j]) * linear[j];
        cleanEnergy += int64_t(clean[j]) * clean[j];
        cleanPeak = std::max(cleanPeak, std::abs(int(clean[j])));
      }
      totalSamples += frameSize;
      // Apply the board's calibrated voice gain after AEC, keeping the ADC
      // unclipped. Unity-gain boards retain the direct copy. Never gate speech.
      if (outputGain == 1.0f) {
        std::memcpy(output + produced, clean, frameSize * sizeof(int16_t));
      } else {
        for (size_t j = 0; j < frameSize; ++j) {
          const float amplified = clean[j] * outputGain;
          output[produced + j] = int16_t(std::lround(std::clamp(amplified, -32768.0f, 32767.0f)));
        }
      }
      produced += frameSize;
      fill = 0;
    }
  }
  const auto now = esp_timer_get_time();
  if (now - logAt >= 1000000 && totalSamples) {
    ESP_LOGI("gea_aec",
             "raw_rms=%.0f clean_rms=%.0f ref_rms=%.0f linear_rms=%.0f reduction_db=%.1f "
             "process_us=%lld dma_drops=%lu clipped=%lu ref_clipped=%lu raw_peak=%d clean_peak=%d ref_peak=%d",
             std::sqrt(double(rawEnergy) / totalSamples),
             std::sqrt(double(cleanEnergy) / totalSamples),
             std::sqrt(double(refEnergy) / totalSamples),
             std::sqrt(double(linearEnergy) / totalSamples),
             10 * std::log10(double(rawEnergy + 1) / double(cleanEnergy + 1)),
             (long long)(processUs / std::max<uint32_t>(1, processedFrames)),
             (unsigned long)dmaDrops, (unsigned long)clippedSamples,
             (unsigned long)clippedReferenceSamples, rawPeak, cleanPeak, referencePeak);
    rawEnergy = cleanEnergy = linearEnergy = refEnergy = totalSamples = 0;
    rawPeak = cleanPeak = referencePeak = 0;
    processUs = processedFrames = clippedSamples = clippedReferenceSamples = 0;
    logAt = now;
  }
  return produced;
}
