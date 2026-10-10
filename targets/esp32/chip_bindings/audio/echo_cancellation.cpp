// Official Espressif ESP-SR AEC; adapter only, no custom cancellation DSP.
#include "echo_cancellation.h"
#include "esp_aec.h"
#include "esp_agc.h"
#if GEA_AUDIO_EXPERIMENT
#include "esp_ns.h"
#endif
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
int16_t dmaMic[kCaptureRing]{}, dmaMic2[kCaptureRing]{}, dmaRef[kCaptureRing]{};
int16_t previousMic[kDmaFrames]{}, latestTx[kDmaFrames]{}, captureRef[320]{}, captureMic2[320]{};
uint32_t dmaWritten = 0, dmaRead = 0, dmaDrops = 0;
bool dmaActive = false, previousMicReady = false;
bool hardwareReference = false;
float outputGain = 1.0f;
unsigned microphoneCount = 1;
bool releaseQuietReference = false;
size_t quietReferenceSamples = 0;
std::mutex aecMutex;
aec_handle_t *aec = nullptr;
void *agc = nullptr;
constexpr size_t kAgcFrame = 160; // ESP-SR AGC requires 10 ms at 16 kHz.
constexpr int kOutputPeakLimit = 28000; // 1.37 dB below the PCM16 ceiling.
int16_t *agcInput = nullptr, *agcOutput = nullptr;
size_t agcFill = 0;
bool gainFailed = false;
#if GEA_AUDIO_EXPERIMENT
bool experimentAec = true, experimentNlp = true, experimentAgc = true;
bool experimentInitialized = false;
int experimentMic = 0, experimentNlpLevel = 0, experimentNsLevel = -1;
int experimentCompression = 12, experimentTarget = 6, experimentQuietMs = 128;
aec_config_t experimentAecConfig{};
ns_handle_t experimentNs = nullptr;
int16_t *experimentCapture = nullptr;
size_t captureCapacity = 0, captureFrames = 0;
bool captureRunning = false;
int64_t captureStartedUs = 0;
uint32_t captureDrops = 0, captureEndDrops = 0;
#endif
// Only the ISR capture ring needs internal RAM. Keep the aligned DSP scratch
// in PSRAM, leaving DMA-capable RAM available for I2S and TLS encryption.
int16_t *scratch = nullptr, *mic = nullptr, *secondMicFrame = nullptr, *ref = nullptr, *clean = nullptr,
        *linear = nullptr, *output = nullptr;
size_t fill = 0, frameSize = 0;

uint64_t rawEnergy = 0, cleanEnergy = 0, linearEnergy = 0, refEnergy = 0, totalSamples = 0;
int64_t logAt = 0, processUs = 0;
uint32_t processedFrames = 0, clippedSamples = 0, clippedReferenceSamples = 0;
int rawPeak = 0, cleanPeak = 0, referencePeak = 0;
uint32_t outputClipped = 0;
uint64_t secondMicEnergy = 0;
int secondMicPeak = 0;
uint64_t outputEnergy = 0, outputSamples = 0;
int outputPeak = 0;
uint32_t peakLimitedFrames = 0, agcFullScaleSamples = 0;

// AGC consumes 10 ms blocks while AEC produces 32 ms blocks. Retain the tail
// across calls, preserving sample order without padding or an extra audio task.
size_t applyOutputGain(const int16_t *samples, size_t count, int16_t *destination) {
  bool useAgc = agc != nullptr;
#if GEA_AUDIO_EXPERIMENT
  useAgc = useAgc && experimentAgc;
  if (!useAgc && !experimentNs) {
    int peak = 0;
    for (size_t j = 0; j < count; ++j) peak = std::max(peak, std::abs(int(samples[j])));
    const float gain = peak ? std::min(outputGain, float(kOutputPeakLimit) / peak) : outputGain;
    peakLimitedFrames += gain < outputGain;
    for (size_t j = 0; j < count; ++j) destination[j] = int16_t(std::lround(samples[j] * gain));
    return count;
  }
#else
  if (!useAgc) {
    for (size_t j = 0; j < count; ++j) {
      const float amplified = samples[j] * outputGain;
      outputClipped += amplified > 32767.0f || amplified < -32768.0f;
      destination[j] = int16_t(std::lround(std::clamp(amplified, -32768.0f, 32767.0f)));
    }
    return count;
  }
#endif
  int peak = 0;
  for (size_t j = 0; j < count; ++j) peak = std::max(peak, std::abs(int(samples[j])));
  // Protect the AGC input BEFORE converting amplified samples to PCM16.
  // Clamping first would irreversibly distort peaks before the limiter saw them.
  const float gain = peak ? std::min(outputGain, float(kOutputPeakLimit) / peak) : outputGain;
  peakLimitedFrames += gain < outputGain;
  size_t produced = 0;
  for (size_t j = 0; j < count; ++j) {
    agcInput[agcFill++] = int16_t(std::lround(samples[j] * gain));
    if (agcFill != kAgcFrame) continue;
#if GEA_AUDIO_EXPERIMENT
    if (experimentNs) {
      ns_process(experimentNs, agcInput, agcOutput);
      std::memcpy(agcInput, agcOutput, kAgcFrame * sizeof(int16_t));
    }
#endif
    int status = 0;
    if (useAgc) status = esp_agc_process(agc, agcInput, agcOutput, kAgcFrame, 16000);
    else std::memcpy(agcOutput, agcInput, kAgcFrame * sizeof(int16_t));
    agcFill = 0;
    if (status < 0) {
      ESP_LOGE("gea_aec", "Post-AEC AGC failed: %d", status);
      gainFailed = true;
      return 0;
    }
    int processedPeak = 0;
    for (size_t k = 0; k < kAgcFrame; ++k) {
      processedPeak = std::max(processedPeak, std::abs(int(agcOutput[k])));
      agcFullScaleSamples += agcOutput[k] == 32767 || agcOutput[k] == -32768;
    }
    // A final uniform block attenuation preserves waveform shape and leaves
    // headroom for transport resampling, including negative full-scale samples.
    const float protection = processedPeak > kOutputPeakLimit
        ? float(kOutputPeakLimit) / processedPeak : 1.0f;
    peakLimitedFrames += protection < 1.0f;
    for (size_t k = 0; k < kAgcFrame; ++k)
      destination[produced + k] = int16_t(std::lround(agcOutput[k] * protection));
    produced += kAgcFrame;
  }
  return produced;
}
} // namespace

bool geaAudioAecStart(bool useHardwareReference, float gain, bool aggressiveNlp, unsigned inputMicrophones, bool adaptiveGain) {
  std::lock_guard<std::mutex> lock(aecMutex);
  if (aec)
    return true;
  if (!std::isfinite(gain) || gain <= 0 || inputMicrophones < 1 || inputMicrophones > 2 ||
      (inputMicrophones == 2 && !useHardwareReference))
    return false;
  scratch = static_cast<int16_t *>(heap_caps_aligned_alloc(
      16, (7 * kMaxFrame + 2 * kAgcFrame) * sizeof(int16_t), MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT));
  if (!scratch) {
    ESP_LOGE("gea_aec", "AEC processing buffer allocation failed");
    return false;
  }
  mic = scratch;
  secondMicFrame = mic + kMaxFrame;
  ref = secondMicFrame + kMaxFrame;
  clean = ref + kMaxFrame;
  linear = clean + kMaxFrame;
  output = linear + kMaxFrame;
#if !GEA_AUDIO_EXPERIMENT
  outputGain = gain;
#endif
  microphoneCount = inputMicrophones;
#if GEA_AUDIO_EXPERIMENT
  releaseQuietReference = experimentNlpLevel == 0;
#else
  releaseQuietReference = !aggressiveNlp;
#endif
  quietReferenceSamples = 0;
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
#if GEA_AUDIO_EXPERIMENT
  config.nlp_level = static_cast<aec_nlp_level_t>(experimentNlpLevel);
  experimentAecConfig = config;
#endif
  aec = aec_create_from_config(&config);
  if (!aec) {
    ESP_LOGE("gea_aec", "ESP-SR AEC allocation failed");
    heap_caps_free(scratch);
    scratch = mic = secondMicFrame = ref = clean = linear = output = nullptr;
    return false;
  }
  frameSize = aec_get_chunksize(aec);
  if (!frameSize || frameSize > kMaxFrame) {
    ESP_LOGE("gea_aec", "Unsupported AEC frame size %u", unsigned(frameSize));
    aec_destroy(aec);
    aec = nullptr;
    heap_caps_free(scratch);
    scratch = mic = secondMicFrame = ref = clean = linear = output = nullptr;
    return false;
  }
  agcInput = output + 2 * kMaxFrame;
  agcOutput = agcInput + kAgcFrame;
  if (adaptiveGain) {
    agc = esp_agc_open(AGC_MODE_2, 16000);
    if (!agc) {
      ESP_LOGE("gea_aec", "Post-AEC AGC allocation failed");
      aec_destroy(aec);
      aec = nullptr;
      heap_caps_free(scratch);
      scratch = mic = secondMicFrame = ref = clean = linear = output = nullptr;
      agcInput = agcOutput = nullptr;
      return false;
    }
    // 12 dB compression gain, built-in limiter, -6 dBFS target.
#if GEA_AUDIO_EXPERIMENT
    set_agc_config(agc, experimentCompression, 1, experimentTarget);
#else
    set_agc_config(agc, 12, 1, 6);
#endif
  }
#if GEA_AUDIO_EXPERIMENT
  if (!experimentNs && experimentNsLevel >= 0) {
    experimentNs = ns_pro_create(10, experimentNsLevel, 16000);
    if (!experimentNs) {
      if (agc) esp_agc_close(agc);
      agc = nullptr;
      aec_destroy(aec); aec = nullptr;
      heap_caps_free(scratch);
      scratch = mic = secondMicFrame = ref = clean = linear = output = nullptr;
      agcInput = agcOutput = nullptr;
      return false;
    }
  }
#endif
  agcFill = 0;
  gainFailed = false;
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
  outputClipped = 0;
  secondMicEnergy = 0;
  secondMicPeak = 0;
  outputEnergy = outputSamples = 0;
  outputPeak = 0;
  peakLimitedFrames = agcFullScaleSamples = 0;
  logAt = esp_timer_get_time();
  ESP_LOGI("gea_aec",
           "ESP-SR AEC ready: rate=16000 frame=%u NLP=%s "
           "physical_mics=%u aec_channels=1 reference=%s adaptive_gain=%s internal_free=%u",
           unsigned(frameSize), aggressiveNlp ? "aggressive" : "normal",
           microphoneCount, hardwareReference ? "ADC" : "TX-DMA", agc ? "digital+limiter" : "off",
           unsigned(heap_caps_get_free_size(MALLOC_CAP_INTERNAL)));
  return true;
}

void geaAudioAecStop() {
  std::lock_guard<std::mutex> lock(aecMutex);
#if GEA_AUDIO_EXPERIMENT
  if (captureRunning) {
    portENTER_CRITICAL(&dmaMux);
    captureEndDrops = dmaDrops;
    portEXIT_CRITICAL(&dmaMux);
  }
  captureRunning = false; // Retain the bounded recording for silent USB export.
#endif
  portENTER_CRITICAL(&dmaMux);
  dmaActive = false;
  portEXIT_CRITICAL(&dmaMux);
  if (aec) {
    aec_destroy(aec);
    aec = nullptr;
  }
#if GEA_AUDIO_EXPERIMENT
  if (experimentNs) ns_destroy(experimentNs);
  experimentNs = nullptr;

#endif
  if (agc) esp_agc_close(agc);
  agc = nullptr;
  agcInput = agcOutput = nullptr;
  agcFill = 0;
  gainFailed = false;
  heap_caps_free(scratch);
  scratch = mic = secondMicFrame = ref = clean = linear = output = nullptr;
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
  const size_t stride = hardwareReference ? microphoneCount + 1 : 2;
  if (dmaActive && count == kDmaFrames * stride) {
    if (hardwareReference) {
      // ES7210 analog loopback or ES8311 ADCL + DACR arrive in the same RX
      // frame. Keep them paired across callback/task scheduling jitter.
      if (dmaWritten - dmaRead + kDmaFrames > kCaptureRing) {
        dmaRead = dmaWritten;
        ++dmaDrops;
      }
      for (size_t i = 0; i < kDmaFrames; ++i) {
        // ES7210 slots are MIC1, speaker reference, MIC2.
        dmaMic[(dmaWritten + i) % kCaptureRing] = stereo[i * stride];
        dmaRef[(dmaWritten + i) % kCaptureRing] = stereo[i * stride + 1];
        if (microphoneCount == 2)
          dmaMic2[(dmaWritten + i) % kCaptureRing] = stereo[i * stride + 2];
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
      if (microphoneCount == 2)
        captureMic2[i] = dmaMic2[(dmaRead + i) % kCaptureRing];
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
  if (!aec || gainFailed)
    return 0; // Never silently forward uncancelled audio after an AEC failure.
  size_t produced = 0;
  for (size_t i = 0; i < count; ++i) {
#if GEA_AUDIO_EXPERIMENT
    if (captureRunning && captureFrames < captureCapacity) {
      if (!captureFrames) captureStartedUs = esp_timer_get_time();
      experimentCapture[captureFrames * 3] = pcm[i];
      experimentCapture[captureFrames * 3 + 1] = microphoneCount == 2 ? captureMic2[i] : 0;
      experimentCapture[captureFrames * 3 + 2] = captureRef[i];
      if (++captureFrames == captureCapacity) {
        captureRunning = false;
        portENTER_CRITICAL(&dmaMux);
        captureEndDrops = dmaDrops;
        portEXIT_CRITICAL(&dmaMux);
      }
    }
#endif
    // Both microphones are synchronized by the same ADC/I2S frame. Mix
    // before cancellation so one AEC models their combined speaker echo path.
    // Averaging avoids doubling amplitude and retains the single-channel CPU cost.
    mic[fill] = microphoneCount == 2
        ? int16_t((int32_t(pcm[i]) + captureMic2[i]) / 2) : pcm[i];
#if GEA_AUDIO_EXPERIMENT
    if (microphoneCount == 2 && experimentMic == 1) mic[fill] = pcm[i];
    if (microphoneCount == 2 && experimentMic == 2) mic[fill] = captureMic2[i];
#endif
    if (microphoneCount == 2) secondMicFrame[fill] = captureMic2[i];
    ref[fill] = captureRef[i];
    if (++fill == frameSize) {
      uint64_t frameReferenceEnergy = 0;
      for (size_t j = 0; j < frameSize; ++j) {
        rawPeak = std::max(rawPeak, std::abs(int(mic[j])));
        referencePeak = std::max(referencePeak, std::abs(int(ref[j])));
        clippedSamples += std::abs(int(mic[j])) >= 32760;
        clippedReferenceSamples += std::abs(int(ref[j])) >= 32760;
        rawEnergy += int64_t(mic[j]) * mic[j];
        refEnergy += int64_t(ref[j]) * ref[j];
        frameReferenceEnergy += int64_t(ref[j]) * ref[j];
        if (microphoneCount == 2) {
          const int16_t second = secondMicFrame[j];
          secondMicEnergy += int64_t(second) * second;
          secondMicPeak = std::max(secondMicPeak, std::abs(int(second)));
        }
      }
      auto before = esp_timer_get_time();
      // ESP-SR's documented two-stage API preserves both cancellation stages.
      // Snapshot the linear result so telemetry can identify suppression of
      // near-end speech separately from cancellation of the speaker echo.
#if GEA_AUDIO_EXPERIMENT
      if (experimentAec) aec_linear_process(aec, mic, ref, clean);
      else std::memcpy(clean, mic, frameSize * sizeof(int16_t));
#else
      aec_linear_process(aec, mic, ref, clean);
#endif
      std::memcpy(linear, clean, frameSize * sizeof(int16_t));
#if GEA_AUDIO_EXPERIMENT
      if (experimentAec && experimentNlp) aec_nlp_process(aec, clean);
#else
      aec_nlp_process(aec, clean);
#endif
      ++processedFrames;
      for (size_t j = 0; j < frameSize; ++j) {
        linearEnergy += int64_t(linear[j]) * linear[j];
        cleanEnergy += int64_t(clean[j]) * clean[j];
        cleanPeak = std::max(cleanPeak, std::abs(int(clean[j])));
      }
      totalSamples += frameSize;
      // Keep NLP trained, but release its hangover once playback has been
      // quiet for 128 ms. A whole quiet interval covers the linear filter's
      // output delay and acoustic tail; one quiet frame inside a word does not.
      // Linear AEC remains active and removes residual room/DMA echo.
#if GEA_AUDIO_EXPERIMENT
      const size_t kQuietReferenceSamples = size_t(experimentQuietMs) * 16;
#else
      constexpr size_t kQuietReferenceSamples = 2048;
#endif
      quietReferenceSamples = frameReferenceEnergy < uint64_t(128 * 128) * frameSize
          ? std::min(kQuietReferenceSamples, quietReferenceSamples + frameSize) : 0;
      const bool released = releaseQuietReference && kQuietReferenceSamples > 0 && quietReferenceSamples >= kQuietReferenceSamples;
      const size_t gained = applyOutputGain(released ? linear : clean,
                                           frameSize, output + produced);
      fill = 0;
      if (gainFailed) return 0;
      for (size_t j = 0; j < gained; ++j) {
        const int16_t value = output[produced + j];
        outputEnergy += int64_t(value) * value;
        outputPeak = std::max(outputPeak, std::abs(int(value)));
      }
      outputSamples += gained;
      produced += gained;
      processUs += esp_timer_get_time() - before;
    }
  }
  const auto now = esp_timer_get_time();
  if (now - logAt >= 1000000 && totalSamples) {
    ESP_LOGI("gea_aec",
             "raw_rms=%.0f clean_rms=%.0f ref_rms=%.0f linear_rms=%.0f reduction_db=%.1f "
             "process_us=%lld dma_drops=%lu clipped=%lu ref_clipped=%lu raw_peak=%d clean_peak=%d ref_peak=%d output_clipped=%lu output_clipped_pct=%.4f output_gain=%.3f output_rms=%.0f output_peak=%d peak_limited_frames=%lu agc_full_scale_samples=%lu mic2_rms=%.0f mic2_peak=%d",
             std::sqrt(double(rawEnergy) / totalSamples),
             std::sqrt(double(cleanEnergy) / totalSamples),
             std::sqrt(double(refEnergy) / totalSamples),
             std::sqrt(double(linearEnergy) / totalSamples),
             10 * std::log10(double(rawEnergy + 1) / double(cleanEnergy + 1)),
             (long long)(processUs / std::max<uint32_t>(1, processedFrames)),
             (unsigned long)dmaDrops, (unsigned long)clippedSamples,
             (unsigned long)clippedReferenceSamples, rawPeak, cleanPeak, referencePeak,
             (unsigned long)outputClipped, 100.0 * outputClipped / totalSamples, outputGain,
             std::sqrt(double(outputEnergy) / std::max<uint64_t>(1, outputSamples)), outputPeak,
             (unsigned long)peakLimitedFrames, (unsigned long)agcFullScaleSamples,
             std::sqrt(double(secondMicEnergy) / totalSamples), secondMicPeak);
    rawEnergy = cleanEnergy = linearEnergy = refEnergy = totalSamples = 0;
    rawPeak = cleanPeak = referencePeak = 0;
    outputClipped = 0;
    secondMicEnergy = 0;
    secondMicPeak = 0;
    outputEnergy = outputSamples = 0;
    outputPeak = 0;
    peakLimitedFrames = agcFullScaleSamples = 0;
    processUs = processedFrames = clippedSamples = clippedReferenceSamples = 0;
    logAt = now;
  }
  return produced;
}

#if GEA_AUDIO_EXPERIMENT
void geaAudioAecExperimentDefaults(float gain, bool aggressive, unsigned microphones, bool adaptive) {
  std::lock_guard<std::mutex> lock(aecMutex);
  if (experimentInitialized) return;
  experimentInitialized = true;
  outputGain = gain;
  experimentNlpLevel = aggressive ? 1 : 0;
  experimentAgc = adaptive;
  microphoneCount = microphones;
}
bool geaAudioAecConfigure(const char *key, float value) {
  if (!key || !std::isfinite(value) || value < -1 || value > 1000) return false;
  std::lock_guard<std::mutex> lock(aecMutex);
  const int number = int(value);
  const bool integer = value == float(number);
  if (!std::strcmp(key, "capture") && integer && number >= 0 && number <= 8) {
    if (!number) {
      if (captureRunning) {
        portENTER_CRITICAL(&dmaMux);
        captureEndDrops = dmaDrops;
        portEXIT_CRITICAL(&dmaMux);
      }
      captureRunning = false;
      return true;
    }
    if (!aec || captureRunning) return false;
    const size_t capacity = size_t(number) * 16000;
    auto *replacement = static_cast<int16_t *>(heap_caps_malloc(capacity * 3 * sizeof(int16_t),
        MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT));
    if (!replacement) return false;
    heap_caps_free(experimentCapture);
    experimentCapture = replacement;
    captureCapacity = capacity;
    captureFrames = 0;
    captureStartedUs = 0;
    portENTER_CRITICAL(&dmaMux);
    captureDrops = captureEndDrops = dmaDrops;
    portEXIT_CRITICAL(&dmaMux);
    captureRunning = true;
    return true;
  }
  if (!std::strcmp(key, "gain") && value >= .01f && value <= 256) outputGain = value;
  else if (!std::strcmp(key, "aec") && integer && number >= 0 && number <= 1) experimentAec = number;
  else if (!std::strcmp(key, "nlp") && integer && number >= 0 && number <= 1) experimentNlp = number;
  else if (!std::strcmp(key, "mic") && integer && number >= 0 && number <= 2) {
    if (number == 2 && microphoneCount != 2) return false;
    experimentMic = number;
  } else if (!std::strcmp(key, "agc") && integer && number >= 0 && number <= 1) {
    if (number && !agc && aec) return false;
    experimentAgc = number;
  } else if (!std::strcmp(key, "agc_db") && integer && number >= 0 && number <= 30) {
    experimentCompression = number;
    if (agc) set_agc_config(agc, experimentCompression, 1, experimentTarget);
  } else if (!std::strcmp(key, "agc_target") && integer && number >= 1 && number <= 20) {
    experimentTarget = number;
    if (agc) set_agc_config(agc, experimentCompression, 1, experimentTarget);
  } else if (!std::strcmp(key, "quiet_ms") && integer && number >= 0 && number <= 1000) experimentQuietMs = number;
  else if (!std::strcmp(key, "nlp_level") && integer && number >= 0 && number <= 2) {
    if (number == experimentNlpLevel) return true;
    if (aec) {
      auto config = experimentAecConfig;
      config.nlp_level = static_cast<aec_nlp_level_t>(number);
      auto *replacement = aec_create_from_config(&config);
      if (!replacement) return false;
      if (size_t(aec_get_chunksize(replacement)) != frameSize) { aec_destroy(replacement); return false; }
      aec_destroy(aec);
      aec = replacement;
      experimentAecConfig = config;
      fill = 0;
    }
    experimentNlpLevel = number;
    releaseQuietReference = number == 0;
  } else if (!std::strcmp(key, "ns") && integer && number >= -1 && number <= 2) {
    if (number == experimentNsLevel) return true;
    ns_handle_t replacement = number < 0 ? nullptr : ns_pro_create(10, number, 16000);
    if (number >= 0 && !replacement) return false;
    if (experimentNs) ns_destroy(experimentNs);
    experimentNs = replacement;
    experimentNsLevel = number;
  } else return false;
  agcFill = 0;
  quietReferenceSamples = 0;
  return true;
}
bool geaAudioAecCaptureRead(size_t offset, int16_t *samples, size_t capacity,
                            size_t *frames, size_t *total, int64_t *startedUs,
                            uint32_t *drops) {
  std::lock_guard<std::mutex> lock(aecMutex);
  if (captureRunning || !experimentCapture || offset > captureFrames) return false;
  *frames = std::min(capacity, captureFrames - offset);
  *total = captureFrames;
  *startedUs = captureStartedUs;
  *drops = captureEndDrops - captureDrops;
  std::memcpy(samples, experimentCapture + offset * 3, *frames * 3 * sizeof(int16_t));
  return true;
}
void geaAudioAecDescribe(char *buffer, size_t capacity) {
  std::lock_guard<std::mutex> lock(aecMutex);
  std::snprintf(buffer, capacity,
    "aec=%d nlp=%d nlp_level=%d gain=%.3f agc=%d agc_db=%d agc_target=%d ns=%d mic=%d physical_mics=%u quiet_ms=%d ready=%d",
    experimentAec, experimentNlp, experimentNlpLevel, outputGain, experimentAgc,
    experimentCompression, experimentTarget, experimentNsLevel, experimentMic,
    microphoneCount, experimentQuietMs, aec != nullptr);
}
#endif
