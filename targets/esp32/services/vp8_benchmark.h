// Offline diagnostic; no signaling, microphone, audio playback or cloud room.
#pragma once
#include <cstdio>

#if defined(GEA_EMBEDDED_VP8_BENCHMARK) && GEA_EMBEDDED_VP8_BENCHMARK && __has_include(<vpx/vpx_decoder.h>)
#include "host/rtc_vp8_decoder.h"
#include "host/rtc_vp8_capture.h"
#include "esp_timer.h"
#include "freertos/idf_additions.h"
#include "freertos/semphr.h"
#include <cstdio>
#include <cstring>
#include <stdexcept>

namespace gea::platform::esp32::services {

inline void startVp8Capture() {
  if (!gea::host::rtc::vp8::capture().start())
    std::puts("GEADEV:ERR VP8CAPTURE stop-conversation-first");
  else std::puts("GEADEV:VP8CAPTURE READY max_frames=12 max_bytes=131072");
}

inline void saveVp8Capture() {
  auto& capture = gea::host::rtc::vp8::capture();
  if (!capture.save("/storage/gea-vp8-live-benchmark.ivf"))
    std::puts("GEADEV:ERR VP8CAPTURE save-failed-or-worker-active");
  else capture.report();
}

inline void cancelVp8Capture() {
  gea::host::rtc::vp8::capture().cancel();
  std::puts("GEADEV:VP8CAPTURE CANCELLED");
}

inline configRUN_TIME_COUNTER_TYPE vp8BenchmarkTaskCpu() {
#if CONFIG_FREERTOS_GENERATE_RUN_TIME_STATS
  TaskStatus_t status{};
  vTaskGetInfo(nullptr, &status, pdFALSE, eInvalid);
  return status.ulRunTimeCounter;
#else
  return 0;
#endif
}

// Runs synchronously from the diagnostic console, on a joined PSRAM worker
// with the same stack/core/priority as live video. Read flash from the console's
// internal stack first: SPIFFS disables caches and cannot run on a PSRAM stack.
inline void benchmarkVp8File(const char* path) {
  if (!path) {
    std::printf("GEADEV:ERR VP8BENCH usage=VP8BENCH_path.ivf\n");
    return;
  }
  std::vector<uint8_t, gea::host::media::VideoAllocator<uint8_t>> bytes;
  try {
    using File = std::unique_ptr<std::FILE, decltype(&std::fclose)>;
    File file(std::fopen(path, "rb"), std::fclose);
    if (!file || std::fseek(file.get(), 0, SEEK_END)) throw std::runtime_error("open-failed");
    const long size = std::ftell(file.get());
    if (size < 32 || size > 1024*1024 || std::fseek(file.get(), 0, SEEK_SET))
      throw std::runtime_error("invalid-file-size");
    bytes.resize(size);
    if (std::fread(bytes.data(), 1, bytes.size(), file.get()) != bytes.size())
      throw std::runtime_error("file-read-failed");
  } catch (const std::exception& error) {
    std::printf("GEADEV:ERR VP8BENCH %s\n", error.what());
    return;
  }
  struct Job { std::span<const uint8_t> bytes; SemaphoreHandle_t done; } job{bytes, xSemaphoreCreateBinary()};
  if (!job.done) {
    std::printf("GEADEV:ERR VP8BENCH semaphore-allocation\n");
    return;
  }
  TaskHandle_t task = nullptr;
  const auto run = [](void* context) {
    auto& job = *static_cast<Job*>(context);
    const auto work = [&] {
      using namespace gea::host;
      const auto* header = job.bytes.data();
      if (std::memcmp(header, "DKIF", 4) ||
          header[4] || header[5] || header[6] != 32 || header[7] || std::memcmp(header+8, "VP80", 4))
        throw std::runtime_error("invalid-IVF-header");
      rtc::vp8::Decoder decoder;
      size_t cursor = 32;
      const int64_t start = esp_timer_get_time();
      unsigned frames = 0;
      uint64_t decodeTotal = 0, convertTotal = 0;
      while (frames < 300 && esp_timer_get_time() - start < 15'000'000) {
        if (cursor == job.bytes.size()) break;
        if (job.bytes.size() - cursor < 12) throw std::runtime_error("truncated-frame-header");
        const auto* frameHeader = job.bytes.data() + cursor;
        cursor += 12;
        const uint32_t size = uint32_t(frameHeader[0]) | uint32_t(frameHeader[1]) << 8 |
                              uint32_t(frameHeader[2]) << 16 | uint32_t(frameHeader[3]) << 24;
        if (!size || size > rtc::vp8::Assembler::maxFrameBytes) throw std::runtime_error("invalid-frame-size");
        if (size > job.bytes.size() - cursor) throw std::runtime_error("truncated-frame");
        const auto encoded = job.bytes.subspan(cursor, size);
        cursor += size;
        const auto cpuStarted = vp8BenchmarkTaskCpu();
        const auto frame = decoder.decode(encoded, frames);
        const auto cpuUs = configRUN_TIME_COUNTER_TYPE(vp8BenchmarkTaskCpu() - cpuStarted);
        if (!frame) throw std::runtime_error("decode-failed");
        const auto timing = decoder.timing();
        decodeTotal += timing.decodeUs;
        convertTotal += timing.convertUs;
        std::printf("GEADEV:VP8BENCH FRAME n=%u key=%u bytes=%u width=%u height=%u decode_us=%llu convert_us=%llu allocate_us=%llu output_alignment=%u cpu_us=%llu\n",
                    frames++, unsigned(!(encoded[0]&1)), unsigned(size), unsigned(frame->width), unsigned(frame->height),
                    (unsigned long long)timing.decodeUs, (unsigned long long)timing.convertUs,
                    (unsigned long long)timing.allocateUs, unsigned(uintptr_t(frame->rgb565.data()) & 15),
                    (unsigned long long)cpuUs);
        if (timing.macroblocks) std::printf("GEADEV:VP8BENCH STAGES tokens_us=%llu reconstruction_us=%llu loopfilter_us=%llu other_us=%llu macroblocks=%u token_blocks=%u filter_rows=%u\n",
                    (unsigned long long)timing.tokenUs,
                    (unsigned long long)(timing.macroblockUs >= timing.tokenUs ? timing.macroblockUs - timing.tokenUs : 0),
                    (unsigned long long)timing.filterUs,
                    (unsigned long long)(timing.decodeUs >= timing.macroblockUs + timing.filterUs ? timing.decodeUs - timing.macroblockUs - timing.filterUs : 0),
                    timing.macroblocks, timing.tokenBlocks, timing.filterRows);
        vTaskDelay(1);
      }
      if (!frames) throw std::runtime_error("empty-IVF");
      std::printf("GEADEV:VP8BENCH END frames=%u elapsed_us=%lld decode_us=%llu convert_us=%llu\n",
                  frames, (long long)(esp_timer_get_time()-start),
                  (unsigned long long)decodeTotal, (unsigned long long)convertTotal);
    };
    try { work(); }
    catch (const std::exception& error) { std::printf("GEADEV:ERR VP8BENCH %s\n", error.what()); }
    std::fflush(stdout);
    xSemaphoreGive(job.done);
    vTaskSuspend(nullptr);
  };
  if (xTaskCreatePinnedToCoreWithCaps(run, "gea_vp8_bench", 64*1024, &job, 3, &task, 1,
                                    MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT) != pdPASS) {
    std::printf("GEADEV:ERR VP8BENCH task-allocation\n");
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
inline void startVp8Capture() { std::puts("GEADEV:ERR VP8CAPTURE unavailable"); }
inline void saveVp8Capture() { std::puts("GEADEV:ERR VP8CAPTURE unavailable"); }
inline void cancelVp8Capture() { std::puts("GEADEV:ERR VP8CAPTURE unavailable"); }
inline void benchmarkVp8File(const char*) { std::printf("GEADEV:ERR VP8BENCH unavailable\n"); }
}
#endif
