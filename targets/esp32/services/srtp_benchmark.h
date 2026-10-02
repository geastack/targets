// Offline authenticated RTP measurement. No sockets or cloud session.
#pragma once
#include <cstdio>

#if GEA_EMBEDDED_APP_USES_AUDIO && __has_include("srtp.h")
#include "srtp.h"
#include "esp_timer.h"
#include "esp_heap_caps.h"
#include "freertos/idf_additions.h"
#include "freertos/semphr.h"
#include <array>
#include <cstring>
#include <stdexcept>

namespace gea::platform::esp32::services {
inline void benchmarkSrtp() {
  struct Job { SemaphoreHandle_t done; } job{xSemaphoreCreateBinary()};
  if (!job.done) {
    std::printf("GEADEV:ERR SRTPBENCH semaphore-allocation\n");
    return;
  }
  const auto run = [](void* context) {
    auto& job = *static_cast<Job*>(context);
    try {
      struct Session {
        srtp_t value = nullptr;
        ~Session() { if (value) srtp_dealloc(value); }
      };
      if (srtp_init() != srtp_err_status_ok) throw std::runtime_error("init-failed");
      for (const unsigned payloadSize : {120u, 1200u}) {
        std::array<uint8_t, 30> key{};
        for (unsigned i = 0; i < key.size(); ++i) key[i] = uint8_t(i);
        srtp_policy_t policy{};
        srtp_crypto_policy_set_rtp_default(&policy.rtp);
        srtp_crypto_policy_set_rtcp_default(&policy.rtcp);
        policy.ssrc.type = ssrc_specific;
        policy.ssrc.value = 0x12345678;
        policy.key = key.data();
        policy.window_size = 128;
        Session sender, receiver;
        if (srtp_create(&sender.value, &policy) != srtp_err_status_ok ||
            srtp_create(&receiver.value, &policy) != srtp_err_status_ok)
          throw std::runtime_error("session-allocation");
        std::array<uint8_t, 1400> plain{}, protectedPacket{}, decoded{};
        const unsigned packetSize = payloadSize + 12;
        for (unsigned i = 12; i < packetSize; ++i) plain[i] = uint8_t(i * 17);
        plain[0] = 0x80; plain[1] = 101;
        plain[8] = 0x12; plain[9] = 0x34; plain[10] = 0x56; plain[11] = 0x78;
        uint64_t protectUs = 0, unprotectUs = 0;
        constexpr unsigned packets = 100;
        for (unsigned i = 0; i < packets; ++i) {
          plain[2] = uint8_t(i >> 8); plain[3] = uint8_t(i);
          size_t protectedSize = protectedPacket.size();
          auto before = esp_timer_get_time();
          if (srtp_protect(sender.value, plain.data(), packetSize, protectedPacket.data(),
                           &protectedSize, 0) != srtp_err_status_ok)
            throw std::runtime_error("protect-failed");
          protectUs += esp_timer_get_time() - before;
          size_t decodedSize = decoded.size();
          if (!i) {
            // Authentication failure must not consume the sequence number.
            protectedPacket[protectedSize - 1] ^= 1;
            if (srtp_unprotect(receiver.value, protectedPacket.data(), protectedSize,
                               decoded.data(), &decodedSize) != srtp_err_status_auth_fail)
              throw std::runtime_error("tampered-packet-accepted");
            protectedPacket[protectedSize - 1] ^= 1;
            decodedSize = decoded.size();
          }
          before = esp_timer_get_time();
          if (srtp_unprotect(receiver.value, protectedPacket.data(), protectedSize,
                             decoded.data(), &decodedSize) != srtp_err_status_ok)
            throw std::runtime_error("unprotect-failed");
          unprotectUs += esp_timer_get_time() - before;
          if (decodedSize != packetSize || std::memcmp(plain.data(), decoded.data(), packetSize))
            throw std::runtime_error("roundtrip-mismatch");
          vTaskDelay(1);
        }
        std::printf("GEADEV:SRTPBENCH payload_bytes=%u packets=%u protect_avg_us=%llu unprotect_avg_us=%llu roundtrip=ok tamper=rejected\n",
            payloadSize, packets, (unsigned long long)(protectUs / packets),
            (unsigned long long)(unprotectUs / packets));
      }
      std::printf("GEADEV:SRTPBENCH END\n");
    } catch (const std::exception& error) {
      std::printf("GEADEV:ERR SRTPBENCH %s\n", error.what());
    }
    std::fflush(stdout);
    xSemaphoreGive(job.done);
    for (;;) vTaskSuspend(nullptr);
  };
  TaskHandle_t task = nullptr;
  if (xTaskCreatePinnedToCoreWithCaps(run, "gea_srtp_bench", 24 * 1024, &job, 7, &task, 1,
                                    MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT) != pdPASS) {
    std::printf("GEADEV:ERR SRTPBENCH task-allocation\n");
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
inline void benchmarkSrtp() { std::printf("GEADEV:ERR SRTPBENCH unavailable\n"); }
}
#endif
