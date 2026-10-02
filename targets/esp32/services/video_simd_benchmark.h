// SPDX-License-Identifier: Apache-2.0
// Offline differential tests execute the actual S3 instructions, not emulation.
#pragma once
#include <cstdio>
#if defined(GEA_VIDEO_SIMD_ESP32S3) && GEA_VIDEO_SIMD_ESP32S3 && \
    defined(GEA_EMBEDDED_VP8_BENCHMARK) && GEA_EMBEDDED_VP8_BENCHMARK
#include "host/video_color.h"
#include "pixel.h"
#include "vp8_s3.h"
#include "esp_timer.h"
#include "esp_heap_caps.h"
#include "freertos/idf_additions.h"
#include "freertos/semphr.h"
#include <algorithm>
#include <cstring>
#include <stdexcept>

struct loop_filter_info {
  const unsigned char *mblim, *blim, *lim, *hev_thr;
};
extern "C" {
#define GEA_DECLARE_COPY_REFERENCE(size) \
  void vp8_copy_mem##size##_c(unsigned char*, int, unsigned char*, int);
GEA_DECLARE_COPY_REFERENCE(16x16)
GEA_DECLARE_COPY_REFERENCE(8x8)
GEA_DECLARE_COPY_REFERENCE(8x4)
#undef GEA_DECLARE_COPY_REFERENCE
#define GEA_DECLARE_REFERENCE(kind, size) \
  void vp8_##kind##_predict##size##_c(unsigned char*, int, int, int, unsigned char*, int);
GEA_DECLARE_REFERENCE(sixtap, 4x4)
GEA_DECLARE_REFERENCE(sixtap, 8x4)
GEA_DECLARE_REFERENCE(sixtap, 8x8)
GEA_DECLARE_REFERENCE(sixtap, 16x16)
GEA_DECLARE_REFERENCE(bilinear, 4x4)
GEA_DECLARE_REFERENCE(bilinear, 8x4)
GEA_DECLARE_REFERENCE(bilinear, 8x8)
GEA_DECLARE_REFERENCE(bilinear, 16x16)
#undef GEA_DECLARE_REFERENCE
void vpx_tm_predictor_4x4_c(uint8_t*, ptrdiff_t, const uint8_t*, const uint8_t*);
void vpx_tm_predictor_8x8_c(uint8_t*, ptrdiff_t, const uint8_t*, const uint8_t*);
void vpx_tm_predictor_16x16_c(uint8_t*, ptrdiff_t, const uint8_t*, const uint8_t*);
void vp8_short_idct4x4llm_c(short*, unsigned char*, int, unsigned char*, int);
void vp8_dequant_idct_add_c(short*, short*, unsigned char*, int);
void vp8_dequant_idct_add_y_block_c(short*, short*, unsigned char*, int, char*);
void vp8_dequant_idct_add_uv_block_c(short*, short*, unsigned char*, unsigned char*, int, char*);
void vp8_dc_only_idct_add_c(short, unsigned char*, int, unsigned char*, int);
void vp8_short_inv_walsh4x4_c(short*, short*);
#define GEA_DECLARE_LOOP_REFERENCE(name) \
  void vp8_loop_filter_##name##_c(unsigned char*, unsigned char*, unsigned char*, int, int, struct loop_filter_info*);
GEA_DECLARE_LOOP_REFERENCE(bh)
GEA_DECLARE_LOOP_REFERENCE(bv)
GEA_DECLARE_LOOP_REFERENCE(mbh)
GEA_DECLARE_LOOP_REFERENCE(mbv)
#undef GEA_DECLARE_LOOP_REFERENCE
void vp8_loop_filter_bhs_c(unsigned char*, int, const unsigned char*);
void vp8_loop_filter_bvs_c(unsigned char*, int, const unsigned char*);
void vp8_loop_filter_simple_horizontal_edge_c(unsigned char*, int, const unsigned char*);
void vp8_loop_filter_simple_vertical_edge_c(unsigned char*, int, const unsigned char*);
}

namespace gea::platform::esp32::services {
inline void benchmarkVideoSimd(unsigned mode = 0) {
  namespace pixel = gea::framework::graphics::pixel;
  struct Job { SemaphoreHandle_t done; unsigned mode; } job{xSemaphoreCreateBinary(), mode};
  if (!job.done) { std::puts("GEADEV:ERR SIMDBENCH semaphore-allocation"); return; }
  const auto run = [](void* context) {
    auto& job = *static_cast<Job*>(context);
    try {
      if (job.mode != 1) {
        alignas(16) pixel::native_t source[416], output[416];
        unsigned cases = 0;
        for (unsigned i = 0; i < 416; ++i) source[i] = pixel::native_t(i * 197u + 31u);
        for (unsigned sourceOffset = 0; sourceOffset < 8; ++sourceOffset)
          for (unsigned outputOffset = 0; outputOffset < 8; ++outputOffset)
            for (unsigned count = 0; count <= 396; ++count) {
              std::fill_n(output, 416, pixel::native_t(0xa55a));
              pixel::copyNative(output + outputOffset, source + sourceOffset, count);
              for (unsigned i = 0; i < 416; ++i) {
                const auto expected = i >= outputOffset && i < outputOffset + count
                    ? source[sourceOffset + i - outputOffset] : pixel::native_t(0xa55a);
                if (output[i] != expected) throw std::runtime_error("pixel-copy-or-guard");
              }
              ++cases;
              if (!(cases & 127u)) vTaskDelay(1);
            }
        uint64_t scalarUs = 0, simdUs = 0;
        for (unsigned round = 0; round < 128; ++round) {
          const auto scalarStarted = esp_timer_get_time();
          for (unsigned row = 0; row < 448; ++row) {
            std::memcpy(output, source + 14, 368 * sizeof(pixel::native_t));
            asm volatile("" : : "r"(output) : "memory");
          }
          scalarUs += esp_timer_get_time() - scalarStarted;
          const auto simdStarted = esp_timer_get_time();
          for (unsigned row = 0; row < 448; ++row) {
            pixel::copyNative(output, source + 14, 368);
            asm volatile("" : : "r"(output) : "memory");
          }
          simdUs += esp_timer_get_time() - simdStarted;
          vTaskDelay(1);
        }
        std::printf("GEADEV:SIMDBENCH pixel_copy_cases=%u scalar_us=%llu simd_us=%llu exact=1\n",
                    cases, (unsigned long long)scalarUs, (unsigned long long)simdUs);
      }
      struct WorkspaceCheck {
        uint8_t* allocation = static_cast<uint8_t*>(heap_caps_aligned_alloc(
            16, GEA_VP8_WORKSPACE_BYTES + 32, MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT));
        void* previous = nullptr;
        WorkspaceCheck() {
          if (!allocation) throw std::runtime_error("workspace-allocation");
          std::memset(allocation, 0xa5, GEA_VP8_WORKSPACE_BYTES + 32);
          previous = gea_vp8_bind_workspace_s3(allocation + 16);
        }
        ~WorkspaceCheck() {
          gea_vp8_bind_workspace_s3(previous);
          heap_caps_free(allocation);
        }
        void verify() {
          for (unsigned i = 0; i < 16; ++i)
            if (allocation[i] != 0xa5 || allocation[16 + GEA_VP8_WORKSPACE_BYTES + i] != 0xa5)
              throw std::runtime_error("workspace-guard");
        }
      } workspace;
      gea_vp8_entropy_result entropy{};
      if (!gea_vp8_entropy_check_s3(&entropy)) throw std::runtime_error("entropy-state-mismatch");
      std::printf("GEADEV:SIMDBENCH entropy_cases=%u exact=1 scalar_us=%llu native_us=%llu\n",
          entropy.cases, (unsigned long long)entropy.scalar_us, (unsigned long long)entropy.native_us);
      alignas(16) uint16_t swap[24];
      for (unsigned base = 0; base < 65536; base += 8) {
        std::fill_n(swap, 24, uint16_t(0xdead));
        for (unsigned i = 0; i < 8; ++i) swap[8 + i] = uint16_t(base + i);
        gea::host::media::rgb565SwapBytes(swap + 8, 8);
        for (unsigned i = 0; i < 8; ++i) {
          const auto value = uint16_t(base + i);
          if (swap[8 + i] != uint16_t((value << 8) | (value >> 8)) ||
              swap[i] != 0xdead || swap[16 + i] != 0xdead)
            throw std::runtime_error("rgb565-swap-mismatch");
        }
      }
      std::puts("GEADEV:SIMDBENCH swap_values=65536 exact=1");
      // Exercise both byte orders, multi-iteration SIMD, scalar tails, odd
      // heights/strides, unaligned fallback, distinct chroma and write guards.
      alignas(16) uint8_t packedY[32*3], packedU[16*2], packedV[16*2];
      alignas(16) uint16_t nativePixels[128], panelPixels[128];
      uint32_t colorState = 0x29f914adu;
      const auto colorReference = [](uint8_t y, uint8_t u, uint8_t v) {
        const int c = 298 * (int(y) - 16), d = int(u) - 128, e = int(v) - 128;
        const int r = std::clamp((c + 409*e + 128) >> 8, 0, 255);
        const int g = std::clamp((c - 100*d - 208*e + 128) >> 8, 0, 255);
        const int b = std::clamp((c + 516*d + 128) >> 8, 0, 255);
        return uint16_t((r >> 3) << 11 | (g >> 2) << 5 | (b >> 3));
      };
      for (unsigned test = 0; test < 4096; ++test) {
        for (auto* plane : {packedY, packedU, packedV}) {
          const unsigned length = plane == packedY ? sizeof(packedY) : sizeof(packedU);
          for (unsigned i = 0; i < length; ++i) {
            colorState ^= colorState << 13; colorState ^= colorState >> 17; colorState ^= colorState << 5;
            plane[i] = uint8_t(colorState);
          }
        }
        std::fill_n(nativePixels, 128, uint16_t(0xdead));
        std::fill_n(panelPixels, 128, uint16_t(0xdead));
        const unsigned width = 8 + test % 17, offset = test % 2;
        const unsigned ys = 32 - offset, us = 16 - offset;
        gea::host::media::i420ToRgb565(packedY + offset, ys, packedU + offset, us,
            packedV + offset, us, width, 3, nativePixels + 8 + offset);
        gea::host::media::i420ToRgb565(packedY + offset, ys, packedU + offset, us,
            packedV + offset, us, width, 3, panelPixels + 8 + offset, true);
        for (unsigned i = 0; i < 128; ++i) {
          const bool pixel = i >= 8 + offset && i < 8 + offset + width*3;
          const auto value = nativePixels[i];
          if (pixel) {
            const auto index = i - 8 - offset, row = index / width, column = index % width;
            if (value != colorReference(packedY[offset + row*ys + column],
                packedU[offset + (row/2)*us + column/2],
                packedV[offset + (row/2)*us + column/2]))
              throw std::runtime_error("color-reference-mismatch");
          } else if (value != 0xdead) {
            throw std::runtime_error("color-native-guard");
          }
          if (panelPixels[i] != (pixel ? uint16_t((value << 8) | (value >> 8)) : uint16_t(0xdead)))
            throw std::runtime_error("color-endian-mismatch");
        }
        if (!(test & 255)) vTaskDelay(1);
      }
      std::puts("GEADEV:SIMDBENCH color_endian_cases=4096 exact=1");
      if (job.mode != 1) {
      if (job.mode == 0) {
      const auto reference = [](uint8_t y, uint8_t u, uint8_t v) {
        const int c = 298 * (int(y) - 16), d = int(u) - 128, e = int(v) - 128;
        const int r = std::clamp((c + 409*e + 128) >> 8, 0, 255);
        const int g = std::clamp((c - 100*d - 208*e + 128) >> 8, 0, 255);
        const int b = std::clamp((c + 516*d + 128) >> 8, 0, 255);
        return uint16_t((r >> 3) << 11 | (g >> 2) << 5 | (b >> 3));
      };
      alignas(16) uint8_t y[8], u[4], v[4];
      alignas(16) uint16_t out[8];
      // All Y/U/V combinations, with distinct luma lanes to catch zip errors.
      for (unsigned uu = 0; uu < 256; ++uu) {
        std::memset(u, uu, sizeof(u));
        for (unsigned vv = 0; vv < 256; ++vv) {
          std::memset(v, vv, sizeof(v));
          for (unsigned yy = 0; yy < 256; yy += 8) {
            for (unsigned i = 0; i < 8; ++i) y[i] = yy+i;
            gea::host::media::gea_i420_rgb565_row_s3(y, u, v, out, 8);
            for (unsigned i = 0; i < 8; ++i)
              if (out[i] != reference(y[i], uu, vv)) {
                std::printf("GEADEV:SIMDBENCH mismatch y=%u u=%u v=%u actual=%04x expected=%04x\n",
                    y[i], uu, vv, out[i], reference(y[i], uu, vv));
                throw std::runtime_error("color-mismatch");
              }
          }
        }
        if (!(uu & 15)) vTaskDelay(1);
      }
      std::puts("GEADEV:SIMDBENCH color_combinations=16777216 exact=1");
      for (unsigned test = 0; test < 256; ++test) {
        for (unsigned i = 0; i < 8; ++i) y[i] = uint8_t(test + i*31);
        for (unsigned i = 0; i < 4; ++i) { u[i] = uint8_t(test + i*67); v[i] = uint8_t(test - i*47); }
        gea::host::media::gea_i420_rgb565_row_s3(y, u, v, out, 8);
        for (unsigned i = 0; i < 8; ++i)
          if (out[i] != reference(y[i], u[i/2], v[i/2])) throw std::runtime_error("color-lane-mismatch");
      }
      }
      using Filter = void (*)(unsigned char*, int, int, int, unsigned char*, int);
      struct Pair { Filter scalar, simd; };
#define GEA_FILTER_PAIR(kind, size) Pair{vp8_##kind##_predict##size##_c, vp8_##kind##_predict##size##_s3}
      const Pair pairs[]{GEA_FILTER_PAIR(sixtap, 4x4), GEA_FILTER_PAIR(sixtap, 8x4),
        GEA_FILTER_PAIR(sixtap, 8x8), GEA_FILTER_PAIR(sixtap, 16x16),
        GEA_FILTER_PAIR(bilinear, 4x4), GEA_FILTER_PAIR(bilinear, 8x4),
        GEA_FILTER_PAIR(bilinear, 8x8), GEA_FILTER_PAIR(bilinear, 16x16)};
#undef GEA_FILTER_PAIR
      alignas(16) uint8_t source[64*64], scalar[32*18], simd[32*18];
      uint32_t state = 0x581ffadu;
      for (auto& value : source) { state ^= state << 13; state ^= state >> 17; state ^= state << 5; value = state; }
      {
        using Copy = void (*)(unsigned char*, int, unsigned char*, int);
        const Copy copies[][2] = {
          {vp8_copy_mem16x16_c, vp8_copy_mem16x16_s3},
          {vp8_copy_mem8x8_c, vp8_copy_mem8x8_s3},
          {vp8_copy_mem8x4_c, vp8_copy_mem8x4_s3},
          {[](unsigned char* src, int ss, unsigned char* dst, int ds) {
             for (int y = 0; y < 4; ++y) std::memcpy(dst + y*ds, src + y*ss, 4);
           }, [](unsigned char* src, int ss, unsigned char* dst, int ds) {
             vp8_sixtap_predict4x4_s3(src, ss, 0, 0, dst, ds);
           }}
        };
        alignas(16) uint8_t expected[40*18], actual[40*18];
        uint64_t scalarCopyUs = 0, simdCopyUs = 0;
        unsigned copyCases = 0;
        for (const auto& pair : copies)
          for (int sourceStride : {37, 64}) for (int pitch : {37, 40})
            for (unsigned sourceOffset = 0; sourceOffset < 16; ++sourceOffset)
              for (unsigned destinationOffset = 0; destinationOffset < 16; ++destinationOffset) {
                std::memset(expected, 0xa5, sizeof(expected));
                std::memset(actual, 0xa5, sizeof(actual));
                auto* input = source + 8*sourceStride + 16 + sourceOffset;
                auto before = esp_timer_get_time();
                pair[0](input, sourceStride, expected + 32 + destinationOffset, pitch);
                scalarCopyUs += esp_timer_get_time() - before;
                before = esp_timer_get_time();
                pair[1](input, sourceStride, actual + 32 + destinationOffset, pitch);
                simdCopyUs += esp_timer_get_time() - before;
                if (std::memcmp(expected, actual, sizeof(expected)))
                  throw std::runtime_error("vp8-copy-mismatch");
                if (!(++copyCases & 255)) vTaskDelay(1);
              }
        std::printf("GEADEV:SIMDBENCH motion_copy_cases=%u scalar_us=%llu simd_us=%llu exact=1\n",
                    copyCases, (unsigned long long)scalarCopyUs, (unsigned long long)simdCopyUs);
      }
      uint64_t scalarUs = 0, simdUs = 0;
      unsigned cases = 0;
      for (const auto& pair : pairs) {
        for (unsigned offset = 0; offset < 16; ++offset)
          for (int yp = 0; yp < 8; ++yp) for (int xp = 0; xp < 8; ++xp) {
            // Upstream bilinear asserts that at least one axis is fractional.
            if (!(xp | yp)) continue;
            std::memset(scalar, 0xa5, sizeof(scalar));
            std::memset(simd, 0xa5, sizeof(simd));
            auto* input = source + 8*64 + 16 + offset;
            auto before = esp_timer_get_time();
            pair.scalar(input, 64, xp, yp, scalar + 32, 32);
            scalarUs += esp_timer_get_time() - before;
            before = esp_timer_get_time();
            pair.simd(input, 64, xp, yp, simd + 32, 32);
            simdUs += esp_timer_get_time() - before;
            if (std::memcmp(scalar, simd, sizeof(scalar)))
              throw std::runtime_error("filter-mismatch");
            ++cases;
          }
        vTaskDelay(1);
      }
      std::printf("GEADEV:SIMDBENCH filter_cases=%u scalar_us=%llu simd_us=%llu exact=1\n",
          cases, (unsigned long long)scalarUs, (unsigned long long)simdUs);
      using Intra = void (*)(uint8_t*, ptrdiff_t, const uint8_t*, const uint8_t*);
      const Intra intra[][2] = {
        {vpx_tm_predictor_4x4_c, vpx_tm_predictor_4x4_s3},
        {vpx_tm_predictor_8x8_c, vpx_tm_predictor_8x8_s3},
        {vpx_tm_predictor_16x16_c, vpx_tm_predictor_16x16_s3}
      };
      alignas(16) uint8_t above[48], left[16];
      uint64_t intraScalarUs = 0, intraSimdUs = 0;
      for (unsigned test = 0; test < 512; ++test) {
        for (auto& value : above) { state ^= state << 13; state ^= state >> 17; state ^= state << 5; value = state; }
        for (auto& value : left) { state ^= state << 13; state ^= state >> 17; state ^= state << 5; value = state; }
        for (unsigned size = 0; size < 3; ++size) {
          std::memset(scalar, 0xa5, sizeof(scalar));
          std::memset(simd, 0xa5, sizeof(simd));
          const unsigned offset = test % 16;
          auto before = esp_timer_get_time();
          intra[size][0](scalar + 32, 32, above + 16 + offset, left);
          intraScalarUs += esp_timer_get_time() - before;
          before = esp_timer_get_time();
          intra[size][1](simd + 32, 32, above + 16 + offset, left);
          intraSimdUs += esp_timer_get_time() - before;
          if (std::memcmp(scalar, simd, sizeof(scalar))) throw std::runtime_error("intra-tm-mismatch");
        }
        if (!(test & 63)) vTaskDelay(1);
      }
      std::printf("GEADEV:SIMDBENCH intra_cases=1536 scalar_us=%llu simd_us=%llu exact=1\n",
          (unsigned long long)intraScalarUs, (unsigned long long)intraSimdUs);
      alignas(16) short coefficients[16], quantizers[16], copy[16], walshScalar[256], walshSimd[256];
      scalarUs = simdUs = 0;
      for (unsigned test = 0; test < 4096; ++test) {
        for (unsigned i = 0; i < 16; ++i) {
          state ^= state << 13; state ^= state >> 17; state ^= state << 5;
          coefficients[i] = short(state); quantizers[i] = short(state >> 16);
        }
        if (test < 32) {
          std::memset(coefficients, 0, sizeof(coefficients));
          coefficients[test % 16] = test < 16 ? -32768 : 32767;
        }
        std::memset(walshScalar, 0x5a, sizeof(walshScalar));
        std::memset(walshSimd, 0x5a, sizeof(walshSimd));
        vp8_short_inv_walsh4x4_c(coefficients, walshScalar);
        vp8_short_inv_walsh4x4_s3(coefficients, walshSimd);
        if (std::memcmp(walshScalar, walshSimd, sizeof(walshScalar))) throw std::runtime_error("walsh-mismatch");
        std::memset(scalar, 0xa5, sizeof(scalar));
        std::memset(simd, 0xa5, sizeof(simd));
        auto before = esp_timer_get_time();
        vp8_short_idct4x4llm_c(coefficients, source + 32, 32, scalar + 32, 32);
        scalarUs += esp_timer_get_time() - before;
        before = esp_timer_get_time();
        vp8_short_idct4x4llm_s3(coefficients, source + 32, 32, simd + 32, 32);
        simdUs += esp_timer_get_time() - before;
        if (std::memcmp(scalar, simd, sizeof(scalar))) {
          std::printf("GEADEV:SIMDBENCH idct_mismatch case=%u coefficients=", test);
          for (const auto coefficient : coefficients) std::printf("%d,", coefficient);
          std::printf(" scalar=");
          for (unsigned i=0;i<16;++i) std::printf("%u,", scalar[32+(i/4)*32+i%4]);
          std::printf(" simd=");
          for (unsigned i=0;i<16;++i) std::printf("%u,", simd[32+(i/4)*32+i%4]);
          std::printf("\n");
          throw std::runtime_error("idct-mismatch");
        }
        std::memcpy(copy, coefficients, sizeof(copy));
        vp8_dequant_idct_add_c(coefficients, quantizers, scalar + 32, 32);
        vp8_dequant_idct_add_s3(copy, quantizers, simd + 32, 32);
        if (std::memcmp(scalar, simd, sizeof(scalar)) || std::memcmp(coefficients, copy, sizeof(copy)))
          throw std::runtime_error("dequant-mismatch");
        vp8_dc_only_idct_add_c(short(state), source + 32, 32, scalar + 32, 32);
        vp8_dc_only_idct_add_s3(short(state), source + 32, 32, simd + 32, 32);
        if (std::memcmp(scalar, simd, sizeof(scalar))) throw std::runtime_error("dc-idct-mismatch");
        if (!(test & 255)) vTaskDelay(1);
      }
      std::printf("GEADEV:SIMDBENCH idct_cases=4096 dequant_cases=4096 walsh_cases=4096 dc_cases=4096 scalar_us=%llu simd_us=%llu exact=1\n",
          (unsigned long long)scalarUs, (unsigned long long)simdUs);
      {
        alignas(16) short expectedQ[24*16], actualQ[24*16], dq[16];
        alignas(16) uint8_t expectedPixels[3*32*20], actualPixels[3*32*20];
        char eobs[24];
        uint64_t scalarBlocksUs = 0, simdBlocksUs = 0;
        for (unsigned test = 0; test < 1024; ++test) {
          for (auto& q : expectedQ) {
            state ^= state << 13; state ^= state >> 17; state ^= state << 5;
            q = short(state);
          }
          for (auto& d : dq) {
            state ^= state << 13; state ^= state >> 17; state ^= state << 5;
            d = short(state);
          }
          for (unsigned block = 0; block < 24; ++block) {
            eobs[block] = char((test + block) % 17);
            // Exercise skipped zero DC and short-wrap-to-zero as well as
            // mixed DC/full transforms and arbitrary extreme coefficients.
            if (test % 4 == 0) expectedQ[block*16] = 0;
            if (test % 4 == 1) { expectedQ[block*16] = -32768; dq[0] = 2; }
          }
          std::memcpy(actualQ, expectedQ, sizeof(expectedQ));
          for (auto& p : expectedPixels) p = uint8_t(state++);
          std::memcpy(actualPixels, expectedPixels, sizeof(expectedPixels));
          auto before = esp_timer_get_time();
          vp8_dequant_idct_add_y_block_c(expectedQ, dq, expectedPixels + 32, 32, eobs);
          vp8_dequant_idct_add_uv_block_c(expectedQ + 256, dq, expectedPixels + 32*21,
                                         expectedPixels + 32*41, 32, eobs + 16);
          scalarBlocksUs += esp_timer_get_time() - before;
          before = esp_timer_get_time();
          vp8_dequant_idct_add_y_block_s3(actualQ, dq, actualPixels + 32, 32, eobs);
          vp8_dequant_idct_add_uv_block_s3(actualQ + 256, dq, actualPixels + 32*21,
                                          actualPixels + 32*41, 32, eobs + 16);
          simdBlocksUs += esp_timer_get_time() - before;
          if (std::memcmp(expectedPixels, actualPixels, sizeof(expectedPixels)) ||
              std::memcmp(expectedQ, actualQ, sizeof(expectedQ)))
            throw std::runtime_error("macroblock-idct-mismatch");
          if (!(test & 63)) vTaskDelay(1);
        }
        std::printf("GEADEV:SIMDBENCH macroblock_cases=1024 scalar_us=%llu simd_us=%llu exact=1\n",
                    (unsigned long long)scalarBlocksUs, (unsigned long long)simdBlocksUs);
      }
      using Loop = void (*)(unsigned char*, unsigned char*, unsigned char*, int, int, loop_filter_info*);
      using Simple = void (*)(unsigned char*, int, const unsigned char*);
      const Loop loops[][2] = {{vp8_loop_filter_bh_c, vp8_loop_filter_bh_s3},
        {vp8_loop_filter_bv_c, vp8_loop_filter_bv_s3}, {vp8_loop_filter_mbh_c, vp8_loop_filter_mbh_s3},
        {vp8_loop_filter_mbv_c, vp8_loop_filter_mbv_s3}};
      const Simple simple[][2] = {{vp8_loop_filter_bhs_c, vp8_loop_filter_simple_bh_s3},
        {vp8_loop_filter_bvs_c, vp8_loop_filter_simple_bv_s3},
        {vp8_loop_filter_simple_horizontal_edge_c, vp8_loop_filter_simple_mbh_s3},
        {vp8_loop_filter_simple_vertical_edge_c, vp8_loop_filter_simple_mbv_s3}};
      alignas(16) uint8_t frameScalar[3*1024], frameSimd[3*1024];
      scalarUs = simdUs = 0;
      for (unsigned test = 0; test < 768; ++test) {
        uint8_t mblim = uint8_t(test / 2), blim = mblim, limit = uint8_t(test % 64), hev = uint8_t(test % 4);
        loop_filter_info info{&mblim, &blim, &limit, &hev};
        for (unsigned kind = 0; kind < 8; ++kind) {
          for (unsigned i = 0; i < sizeof(frameScalar); ++i) {
            state ^= state << 13; state ^= state >> 17; state ^= state << 5;
            // Smooth ramps and small discontinuities exercise accepted filters;
            // arbitrary noise exercises masks and extreme pixel values.
            frameScalar[i] = frameSimd[i] = test >= 512 ? uint8_t(test - 512) :
                uint8_t((test & 1) ? state : test + (i/32)*2 + (state & 7));
          }
          const unsigned offset = 8*32 + 8 + (test % 4);
          auto before = esp_timer_get_time();
          if (kind < 4) loops[kind][0](frameScalar+offset, frameScalar+1024+offset,
              frameScalar+2048+offset, 32, 32, &info);
          else simple[kind-4][0](frameScalar+offset, 32, &blim);
          scalarUs += esp_timer_get_time() - before;
          before = esp_timer_get_time();
          if (kind < 4) loops[kind][1](frameSimd+offset, frameSimd+1024+offset,
              frameSimd+2048+offset, 32, 32, &info);
          else simple[kind-4][1](frameSimd+offset, 32, &blim);
          simdUs += esp_timer_get_time() - before;
          if (std::memcmp(frameScalar, frameSimd, sizeof(frameScalar))) {
            std::printf("GEADEV:SIMDBENCH loop_mismatch case=%u kind=%u\n", test, kind);
            throw std::runtime_error("loop-filter-mismatch");
          }
        }
        if (!(test & 15)) vTaskDelay(1);
      }
      std::printf("GEADEV:SIMDBENCH loop_cases=6144 scalar_us=%llu simd_us=%llu exact=1\n",
          (unsigned long long)scalarUs, (unsigned long long)simdUs);
      }
      workspace.verify();
      std::printf("GEADEV:SIMDBENCH workspace_bytes=%u exact=1\n", GEA_VP8_WORKSPACE_BYTES);
      std::puts("GEADEV:SIMDBENCH END");
    } catch (const std::exception& error) { std::printf("GEADEV:ERR SIMDBENCH %s\n", error.what()); }
    std::fflush(stdout);
    xSemaphoreGive(job.done);
    vTaskSuspend(nullptr);
  };
  TaskHandle_t task = nullptr;
  if (xTaskCreatePinnedToCoreWithCaps(run, "gea_simd_bench", 32*1024, &job, 3, &task, 1,
                                    MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT) != pdPASS)
    std::puts("GEADEV:ERR SIMDBENCH task-allocation");
  else {
    xSemaphoreTake(job.done, portMAX_DELAY);
    while (eTaskGetState(task) != eSuspended) vTaskDelay(1);
    vTaskDeleteWithCaps(task);
  }
  vSemaphoreDelete(job.done);
}
}
#else
namespace gea::platform::esp32::services {
inline void benchmarkVideoSimd(unsigned = 0) { std::puts("GEADEV:ERR SIMDBENCH unavailable"); }
}
#endif
