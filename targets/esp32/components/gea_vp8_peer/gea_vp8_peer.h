// SPDX-License-Identifier: Apache-2.0
#pragma once
#include "esp_peer.h"
#include <atomic>

namespace gea::host::rtc::vp8 { struct EncodedFrame; }

// This value is local to this implementation, never passed to peer_default.
inline constexpr auto GEA_PEER_VIDEO_CODEC_VP8 = static_cast<esp_peer_video_codec_t>(3);

struct GeaVp8PeerOptions {
  std::atomic<bool>* running = nullptr;
  std::atomic<bool>* requestKeyframe = nullptr;
  void (*wake)(void*) = nullptr;
  void* wakeContext = nullptr;
  // Optional ownership transfer for the native decoder worker. The legacy
  // esp_peer callback borrows a pointer and therefore requires a frame copy.
  int (*onOwnedVideo)(gea::host::rtc::vp8::EncodedFrame&&, void*) = nullptr;
};

const esp_peer_ops_t* gea_vp8_peer_impl();
