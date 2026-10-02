// SPDX-License-Identifier: Apache-2.0
#pragma once

#include "sdkconfig.h"
#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include "esp_timer.h"

#if defined(__XTENSA__) && defined(CONFIG_IDF_TARGET_ESP32S3) && CONFIG_IDF_TARGET_ESP32S3 && __has_include("gea_pcm.h")
#include "gea_pcm.h"

// Exercise the exact generated runtime artifact through the framework's
// normal app include directory. This performs no audio I/O or background work.
inline void checkPcmSimd() {
  alignas(16) float input[40];
  alignas(16) std::int16_t output[48];
  constexpr std::uint32_t boundaries[] = {
      0, 0x80000000, 1, 0x80000001, 0x007fffff, 0x807fffff,
      0x377fffff, 0x37800000, 0x37800001, 0xb77fffff, 0xb7800000, 0xb7800001,
      0x3f7fffff, 0x3f800000, 0x3f800001, 0xbf7fffff, 0xbf800000, 0xbf800001,
      0x7f800000, 0xff800000, 0x7f800001, 0xff800001, 0x7fc00000, 0xffffffff,
      // Half-integer PCM boundaries and adjacent Float32 encodings.
      0x3f0000ff, 0x3f000100, 0x3f000101, 0xbf00007f, 0xbf000080, 0xbf000081,
      0x3d8000ff, 0x3d800100, 0x3d800101, 0xbd8003ff, 0xbd800400, 0xbd800401};
  constexpr std::size_t lengths[] = {0, 1, 7, 8, 9, 15, 16, 17, 31, 32};
  std::size_t samples = 0;
  std::uint32_t random = 0x713ba549;
  const auto started = esp_timer_get_time();
  for (unsigned dataset = 0; dataset < 2; ++dataset) {
    for (std::size_t inOffset = 0; inOffset < 4; ++inOffset) {
      for (std::size_t outOffset = 0; outOffset < 8; ++outOffset) {
        for (auto count : lengths) {
          for (std::size_t i = 0; i < 40; ++i) {
            random ^= random << 13; random ^= random >> 17; random ^= random << 5;
            // Mix arbitrary IEEE encodings with values in the active audio
            // range so random cases do not mostly become zero/saturation.
            const auto bits = dataset == 0 ? boundaries[(i + outOffset) % 36] :
                (i & 1) ? random : (random & 0x807fffffU) | ((111U + ((random >> 24) & 15U)) << 23);
            std::memcpy(&input[i], &bits, sizeof(bits));
          }
          for (auto& value : output) value = 12345;
          gea::runtime::audio::float32ToPcm16(input + inOffset, output + outOffset, count);
          for (std::size_t i = 0; i < count; ++i) {
            const auto expected = gea::runtime::audio::detail::pcm16Sample(input[inOffset + i]);
            const auto actual = output[outOffset + i];
            ++samples;
            if (actual != expected) {
              std::uint32_t bits;
              std::memcpy(&bits, &input[inOffset + i], sizeof(bits));
              std::printf("GEADEV:PCMSIMD ERR bits=%08lx actual=%d expected=%d input_offset=%u output_offset=%u count=%u index=%u\n",
                          static_cast<unsigned long>(bits), int(actual), int(expected),
                          unsigned(inOffset), unsigned(outOffset), unsigned(count), unsigned(i));
              return;
            }
          }
          for (std::size_t i = 0; i < 48; ++i) {
            if ((i < outOffset || i >= outOffset + count) && output[i] != 12345) {
              std::printf("GEADEV:PCMSIMD ERR guard index=%u\n", unsigned(i));
              return;
            }
          }
        }
      }
    }
  }
  std::printf("GEADEV:PCMSIMD OK simd=1 samples=%u elapsed_us=%lld\n",
              unsigned(samples), static_cast<long long>(esp_timer_get_time() - started));
}
#else
inline void checkPcmSimd() { std::puts("GEADEV:PCMSIMD ERR unavailable"); }
#endif
