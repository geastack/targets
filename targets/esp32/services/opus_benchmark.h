// Offline codec measurement. No microphone, speaker, signaling or cloud session.
#pragma once
#include <cstdio>

#if GEA_EMBEDDED_APP_USES_AUDIO && !defined(GEA_EMBEDDED_RTC_UNUSED) && __has_include("esp_opus_enc.h")
#include "host/rtc_opus.h"
#include "esp_timer.h"
#include "esp_heap_caps.h"
#include "freertos/idf_additions.h"
#include "freertos/semphr.h"

namespace gea::platform::esp32::services {
inline void benchmarkOpus() {
  struct Job { SemaphoreHandle_t done; } job{xSemaphoreCreateBinary()};
  if (!job.done) {
    std::printf("GEADEV:ERR OPUSBENCH semaphore-allocation\n");
    return;
  }
  const auto run = [](void* context) {
    auto& job = *static_cast<Job*>(context);
    try {
      using Codec = gea::host::rtc::OpusAudio;
      for (const auto mode : {Codec::Application::LowDelay, Codec::Application::Voice}) {
        Codec codec(mode);
        alignas(16) std::array<int16_t, Codec::frameSamples> pcm{};
        uint64_t encodeUs = 0, decodeUs = 0, maxEncodeUs = 0;
        uint32_t random = 1, totalBytes = 0;
        constexpr unsigned frames = 25;
        const auto started = esp_timer_get_time();
        for (unsigned frame = 0; frame < frames; ++frame) {
          // Periodic voiced signal plus deterministic low-level noise. Avoid
          // benchmarking only silence, whose codec cost is not representative.
          for (unsigned i = 0; i < pcm.size(); ++i) {
            random = random * 1664525u + 1013904223u;
            const int phase = (frame * pcm.size() + i) % 160;
            pcm[i] = int16_t((phase < 80 ? phase - 40 : 120 - phase) * 80
                            + int(random >> 24) - 128);
          }
          auto before = esp_timer_get_time();
          const auto size = codec.encode(pcm.data());
          const uint64_t elapsed = esp_timer_get_time() - before;
          encodeUs += elapsed;
          if (elapsed > maxEncodeUs) maxEncodeUs = elapsed;
          if (size <= 0) throw std::runtime_error("encode-failed");
          totalBytes += size;
          before = esp_timer_get_time();
          if (codec.decode(codec.packet.data(), size) != Codec::frameSamples)
            throw std::runtime_error("decode-failed");
          decodeUs += esp_timer_get_time() - before;
          vTaskDelay(1);
        }
        std::printf("GEADEV:OPUSBENCH mode=%s rate=%d bitrate=24000 frame_ms=%d complexity=0 frames=%u encode_avg_us=%llu encode_max_us=%llu decode_avg_us=%llu elapsed_us=%lld bytes=%u\n",
            mode == Codec::Application::LowDelay ? "lowdelay" : "voice", Codec::sampleRate,
            Codec::frameDurationMs, frames, (unsigned long long)(encodeUs / frames),
            (unsigned long long)maxEncodeUs, (unsigned long long)(decodeUs / frames),
            (long long)(esp_timer_get_time() - started), unsigned(totalBytes));
      }
      std::printf("GEADEV:OPUSBENCH END\n");
    } catch (const std::exception& error) {
      std::printf("GEADEV:ERR OPUSBENCH %s\n", error.what());
    }
    std::fflush(stdout);
    xSemaphoreGive(job.done);
    for (;;) vTaskSuspend(nullptr);
  };
  TaskHandle_t task = nullptr;
  if (xTaskCreatePinnedToCoreWithCaps(run, "gea_opus_bench", 64 * 1024, &job, 5, &task, 1,
                                    MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT) != pdPASS) {
    std::printf("GEADEV:ERR OPUSBENCH task-allocation\n");
  } else {
    xSemaphoreTake(job.done, portMAX_DELAY);
    while (eTaskGetState(task) != eSuspended) vTaskDelay(1);
    vTaskDeleteWithCaps(task);
  }
  vSemaphoreDelete(job.done);
}
} // namespace gea::platform::esp32::services
#else
namespace gea::platform::esp32::services {
inline void benchmarkOpus() { std::printf("GEADEV:ERR OPUSBENCH unavailable\n"); }
}
#endif
