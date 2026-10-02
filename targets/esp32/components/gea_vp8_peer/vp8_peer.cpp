// SPDX-License-Identifier: Apache-2.0
#include "gea_vp8_peer.h"
#include "host/rtc_receive_sdp.h"
#include "host/rtc_vp8_receiver.h"
#include "esp_heap_caps.h"
#include "esp_log.h"
#include "esp_random.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "juice/juice.h"
#include "mbedtls/version.h"
#include "mbedtls/md.h"
#include "dtls_srtp.h"
#include <array>
#include <atomic>
#include <cstring>
#include <mutex>
#include <new>
#include <optional>
#include <sys/time.h>

namespace {
using namespace gea::host::rtc;
constexpr const char* tag = "gea_vp8_peer";
uint64_t nowMs() { return uint64_t(esp_timer_get_time()) / 1000; }

struct Datagram {
  std::array<uint8_t, 4096> bytes;
  size_t size = 0;
};

// Embedded in the PSRAM-allocated peer. The ICE callback only copies bytes;
// crypto, packet assembly and media callbacks run on Gea's peer worker.
template<size_t Capacity>
struct Queue {
  std::array<Datagram, Capacity> packets;
  size_t read = 0, size = 0;
  std::mutex mutex;

  bool push(const char* bytes, size_t length) {
    if (!length || length > packets[0].bytes.size()) return false;
    std::lock_guard lock(mutex);
    if (size == Capacity) return false;
    auto& packet = packets[(read + size++) % Capacity];
    packet.size = length;
    std::memcpy(packet.bytes.data(), bytes, length);
    return true;
  }

  bool pop(Datagram& result) {
    std::lock_guard lock(mutex);
    if (!size) return false;
    auto& packet = packets[read];
    result.size = packet.size;
    std::memcpy(result.bytes.data(), packet.bytes.data(), result.size);
    read = (read + 1) % Capacity;
    --size;
    return true;
  }

  bool empty() {
    std::lock_guard lock(mutex);
    return !size;
  }
};

struct Peer {
  esp_peer_cfg_t cfg{};
  GeaVp8PeerOptions options;
  juice_agent_t* ice = nullptr;
  dtls_srtp_t* crypto = nullptr;
  Queue<8> dtls;
  Queue<8> media;
  using ReceiveQueue = Queue<64>;
  ReceiveQueue* receiveMedia = nullptr;
  Queue<8> outgoing;
  std::mutex signalingMutex;
  std::optional<receive::Offer> pending, offer;
  bool offerAccepted = false; // Guarded by signalingMutex.
  std::optional<vp8::Receiver> video;
  rtp::Reorder audio;
  receive::Track audioTrack, videoTrack;
  std::atomic<juice_state_t> iceState{JUICE_STATE_DISCONNECTED};
  std::atomic<bool> stopped{false}, overflow{false};
  std::atomic<bool> offerRequested{false}, sendReady{false};
  bool sending = false;
  uint16_t sendSequence = 0;
  uint32_t sentPackets = 0, sentOctets = 0, lastTimestamp = 0;
  uint64_t lastAudioMs = 0;
  uint64_t startedMs = 0, handshakeDeadlineMs = 0, lastReportMs = 0;
  uint64_t lastMediaLogMs = 0;
  uint32_t videoPackets = 0, videoFrames = 0, audioPackets = 0, unmatchedPackets = 0;
  uint32_t authenticationDrops = 0, videoSequenceGaps = 0, videoLatePackets = 0;
  uint32_t videoStarts = 0, videoKeyframes = 0, videoMarkers = 0;
  uint32_t videoNonReferences = 0;
  uint16_t nextVideoSequence = 0;
  std::atomic<uint32_t> receiveDrops{0};
  unsigned videoWidth = 0, videoHeight = 0;
  uint32_t localSsrc = 0;
  bool answered = false, authenticated = false, failed = false;
  bool certificateMatched = false;

  bool running() const { return !stopped && options.running && options.running->load(); }

  void wake() const { if (options.wake) options.wake(options.wakeContext); }

  void state(esp_peer_state_t value) {
    if (cfg.on_state) cfg.on_state(value, cfg.ctx);
  }

  int fail(const char* reason) {
    sendReady = false;
    if (!failed) { ESP_LOGE(tag, "%s", reason); state(ESP_PEER_STATE_CONNECT_FAILED); }
    failed = true;
    return ESP_PEER_ERR_FAIL;
  }

  static Peer* bioPeer(void* context) {
    return static_cast<Peer*>(static_cast<dtls_srtp_t*>(context)->ctx);
  }

  static int send(void* context, const unsigned char* bytes, size_t size) {
    auto* self = bioPeer(context);
    if (!self->running()) return MBEDTLS_ERR_SSL_INTERNAL_ERROR;
    const int result = juice_send(self->ice, reinterpret_cast<const char*>(bytes), size);
    return result == JUICE_ERR_SUCCESS ? int(size) : MBEDTLS_ERR_SSL_INTERNAL_ERROR;
  }

  static int recv(void* context, unsigned char* bytes, size_t size) {
    auto* self = bioPeer(context);
    Datagram packet;
    if (!self->running() || (self->handshakeDeadlineMs && nowMs() > self->handshakeDeadlineMs))
      return MBEDTLS_ERR_SSL_INTERNAL_ERROR;
    if (!self->dtls.pop(packet)) {
      vTaskDelay(1);
      return MBEDTLS_ERR_SSL_WANT_READ;
    }
    if (packet.size > size) return MBEDTLS_ERR_SSL_BUFFER_TOO_SMALL;
    std::memcpy(bytes, packet.bytes.data(), packet.size);
    return int(packet.size);
  }

  bool initialize() {
    localSsrc = esp_random();
    if (!localSsrc) localSsrc = 1;
    sendSequence = uint16_t(esp_random());
    startedMs = nowMs();
    dtls_srtp_cfg_t config{};
    config.role = DTLS_SRTP_ROLE_CLIENT;
    config.ctx = this;
    config.udp_send = send;
    config.udp_recv = recv;
    crypto = dtls_srtp_init(&config);
    if (!crypto) return false;
    juice_config_t iceConfig{};
    iceConfig.user_ptr = this;
    iceConfig.cb_state_changed = [](juice_agent_t*, juice_state_t value, void* ctx) {
      auto* self = static_cast<Peer*>(ctx);
      self->iceState = value;
      self->wake();
    };
    iceConfig.cb_recv = [](juice_agent_t*, const char* bytes, size_t size, void* ctx) {
      auto* self = static_cast<Peer*>(ctx);
      if (!self->running() || !size) return;
      const auto first = uint8_t(bytes[0]);
      if (first >= 20 && first <= 63) {
        if (!self->dtls.push(bytes, size)) self->overflow = true;
      } else if (first >= 128 && first <= 191) {
        const bool queued = self->receiveMedia ? self->receiveMedia->push(bytes, size) : self->media.push(bytes, size);
        if (!queued) { self->overflow = true; ++self->receiveDrops; }
      }
      self->wake();
    };
    // The SFU is publicly reachable ICE-lite. Connectivity checks create the
    // NAT mapping; the server learns the peer-reflexive address. No relay-only
    // claim or silently ignored TURN config is supported by this adapter yet.
    ice = juice_create(&iceConfig);
    if (!ice) return false;
    state(ESP_PEER_STATE_CANDIDATE_GATHERING);
    if (juice_set_ice_tcp_mode(ice, JUICE_ICE_TCP_MODE_ACTIVE) != JUICE_ERR_SUCCESS ||
        juice_gather_candidates(ice) != JUICE_ERR_SUCCESS)
      return false;
    return true;
  }

  bool start(receive::Offer remote) {
    offer = std::move(remote);
    if (!sending) {
      for (const auto& track : offer->tracks) {
        if (track.kind == "audio") audioTrack = track;
        else videoTrack = track;
      }
      if (videoTrack.ssrc) video.emplace(videoTrack.payloadType, videoTrack.ssrc);
      if (!initialize()) return false;
    }
    if (!ice || juice_set_remote_description(ice, offer->iceDescription.c_str()) != JUICE_ERR_SUCCESS)
      return false;
    juice_set_remote_gathering_done(ice);
    state(ESP_PEER_STATE_PAIRING);
    return true;
  }

  bool makeDescription() {
    std::array<char, JUICE_MAX_SDP_STRING_LEN> iceSdp{};
    if (juice_get_local_description(ice, iceSdp.data(), iceSdp.size()) != JUICE_ERR_SUCCESS) return false;
    const auto sdp = sending ?
        receive::microphoneOffer(iceSdp.data(), dtls_srtp_get_local_fingerprint(crypto), localSsrc) :
        receive::answer(*offer, iceSdp.data(), dtls_srtp_get_local_fingerprint(crypto));
    if (!sdp) return false;
    esp_peer_msg_t message{ESP_PEER_MSG_TYPE_SDP,
        reinterpret_cast<uint8_t*>(const_cast<char*>(sdp->data())), int(sdp->size())};
    if (cfg.on_msg && cfg.on_msg(&message, cfg.ctx)) return false;
    answered = true;
    return true;
  }

  bool authenticate() {
    state(ESP_PEER_STATE_CONNECTING);
    handshakeDeadlineMs = nowMs() + 10000;
    certificateMatched = false;
    // IDF discards peer certificates after handshake by default. Authenticate
    // the leaf while mbedTLS owns it; do not retain an entire certificate chain
    // or weaken the fingerprint check when get_peer_cert() returns null.
    mbedtls_ssl_set_verify(&crypto->ssl,
        [](void* context, mbedtls_x509_crt* certificate, int depth, uint32_t* flags) {
          auto* self = static_cast<Peer*>(context);
          if (depth) return 0;
          const auto* hash = mbedtls_md_info_from_type(MBEDTLS_MD_SHA256);
          std::array<uint8_t, 32> digest{};
          if (!certificate || !hash ||
              mbedtls_md(hash, certificate->raw.p, certificate->raw.len, digest.data()))
            return MBEDTLS_ERR_X509_CERT_VERIFY_FAILED;
          uint8_t difference = 0;
          for (size_t i = 0; i < digest.size(); ++i)
            difference |= digest[i] ^ self->offer->peerFingerprint[i];
          self->certificateMatched = difference == 0;
          if (difference) return MBEDTLS_ERR_X509_CERT_VERIFY_FAILED;
          // WebRTC authenticates a self-signed certificate through the SDP
          // fingerprint, not a public CA or hostname. Only an exact match
          // authorizes clearing the CA verification flags.
          *flags = 0;
          return 0;
        }, this);
    if (dtls_srtp_handshake(crypto) != 0 || !certificateMatched ||
        !crypto->srtp_in || !crypto->srtp_out) return false;
    handshakeDeadlineMs = 0;
    authenticated = true;
    sendReady = sending;
    state(ESP_PEER_STATE_CONNECTED);
    ESP_LOGI(tag, "%s SRTP authenticated", sending ? "Microphone" : "VP8/Opus");
    return true;
  }

  void deliverVideo(vp8::EncodedFrame frame) {
    ++videoFrames;
    if (options.onOwnedVideo) {
      if (options.onOwnedVideo(std::move(frame), cfg.ctx)) video->requireKeyframe();
      return;
    }
    esp_peer_video_frame_t output{frame.timestamp / 90, frame.bytes.data(), int(frame.bytes.size())};
    if (cfg.on_video_data && cfg.on_video_data(&output, cfg.ctx)) video->requireKeyframe();
  }

  void deliverAudio(const rtp::Packet& packet) {
    esp_peer_audio_frame_t output{packet.timestamp / 48,
        const_cast<uint8_t*>(packet.payload.data()), int(packet.payload.size())};
    if (cfg.on_audio_data) cfg.on_audio_data(&output, cfg.ctx);
  }

  void feedback(bool pli) {
    // Compound RR + SDES(CNAME), optionally PLI. This is valid whether or not
    // reduced-size RTCP was negotiated. SRTCP tag/index room follows the bytes.
    std::array<uint8_t, 96> bytes{0x80, 201, 0, 1};
    rtp::write32(bytes.data() + 4, localSsrc);
    int size = 8;
    if (sending && sentPackets) {
      timeval wall{};
      gettimeofday(&wall, nullptr);
      bytes[1] = 200; bytes[3] = 6;
      rtp::write32(bytes.data() + 8, uint32_t(uint64_t(wall.tv_sec) + 2208988800ULL));
      rtp::write32(bytes.data() + 12, uint32_t((uint64_t(wall.tv_usec) << 32) / 1000000));
      rtp::write32(bytes.data() + 16, lastTimestamp + uint32_t((nowMs() - lastAudioMs) * 48));
      rtp::write32(bytes.data() + 20, sentPackets);
      rtp::write32(bytes.data() + 24, sentOctets);
      size = 28;
    }
    bytes[size] = 0x81; bytes[size + 1] = 202; bytes[size + 3] = 4;
    rtp::write32(bytes.data() + size + 4, localSsrc);
    bytes[size + 8] = 1; bytes[size + 9] = 7;
    std::memcpy(bytes.data() + size + 10, sending ? "gea-mic" : "gea-vp8", 7);
    size += 20;
    if (pli) {
      const auto request = rtp::pictureLossIndication(localSsrc, videoTrack.ssrc);
      std::memcpy(bytes.data() + size, request.data(), request.size());
      size += int(request.size());
    }
    size_t protectedSize = bytes.size();
    if (srtp_protect_rtcp(crypto->srtp_out, bytes.data(), size, bytes.data(), &protectedSize, 0) == srtp_err_status_ok &&
        protectedSize > size_t(size) && protectedSize <= bytes.size())
      juice_send(ice, reinterpret_cast<const char*>(bytes.data()), protectedSize);
  }

  int enqueueAudio(esp_peer_audio_frame_t* frame) {
    if (!sending || !sendReady || !running()) return ESP_PEER_ERR_WRONG_STATE;
    if (!frame || !frame->data || frame->size <= 0 || frame->size > 1275)
      return ESP_PEER_ERR_INVALID_ARG;
    std::array<uint8_t, 1287> packet{};
    packet[0] = 0x80;
    packet[1] = 111;
    packet[2] = uint8_t(sendSequence >> 8);
    packet[3] = uint8_t(sendSequence);
    rtp::write32(packet.data() + 4, uint32_t(uint64_t(frame->pts) * 48));
    rtp::write32(packet.data() + 8, localSsrc);
    std::memcpy(packet.data() + 12, frame->data, frame->size);
    if (!outgoing.push(reinterpret_cast<const char*>(packet.data()), size_t(frame->size) + 12))
      return ESP_PEER_ERR_WOULD_BLOCK;
    ++sendSequence;
    wake();
    return 0;
  }

  bool sendAudio() {
    // Only the peer worker touches SRTP. The encoder enqueues bounded RTP
    // packets, avoiding concurrent crypto/report access from the two tasks.
    Datagram packet;
    while (outgoing.pop(packet)) {
      const auto payloadBytes = packet.size - 12;
      const auto timestamp = rtp::read32(packet.bytes.data() + 4);
      size_t protectedSize = packet.bytes.size();
      if (srtp_protect(crypto->srtp_out, packet.bytes.data(), int(packet.size),
            packet.bytes.data(), &protectedSize, 0) != srtp_err_status_ok ||
          protectedSize <= packet.size || protectedSize > packet.bytes.size() ||
          juice_send(ice, reinterpret_cast<const char*>(packet.bytes.data()), protectedSize) != JUICE_ERR_SUCCESS)
        return false;
      ++sentPackets;
      sentOctets += uint32_t(payloadBytes);
      lastTimestamp = timestamp;
      lastAudioMs = nowMs();
    }
    return true;
  }

  int loop() {
    if (!running() || failed) return 0;
    if (sending && !ice) {
      if (!offerRequested) return 0;
      if (!initialize() || !makeDescription()) return fail("Microphone local ICE/SDP generation failed");
    }
    if (!offer) {
      std::optional<receive::Offer> next;
      { std::lock_guard lock(signalingMutex); next = std::move(pending); pending.reset(); }
      if (!next) return 0;
      if (!start(std::move(*next))) return fail("VP8 ICE/DTLS initialization failed");
    }
    if (!answered && !makeDescription()) return fail("VP8 local SDP generation failed");
    const auto state = iceState.load();
    if (state == JUICE_STATE_FAILED) return fail("VP8 ICE connectivity failed");
    if (!authenticated) {
      if (nowMs() - startedMs > 15000) return fail("VP8 ICE connection timed out");
      if (state != JUICE_STATE_CONNECTED && state != JUICE_STATE_COMPLETED) return 0;
      if (!authenticate()) return fail("VP8 DTLS or certificate fingerprint verification failed");
    }
    if (sending) {
      if (!sendAudio()) return fail("Microphone SRTP send failed");
    } else if (overflow.exchange(false) || options.requestKeyframe->exchange(false)) {
      if (video) video->requireKeyframe();
    }
    if (!dtls.empty()) {
      std::array<uint8_t, 256> control{};
      if (dtls_srtp_read(crypto, control.data(), control.size()) == MBEDTLS_ERR_SSL_PEER_CLOSE_NOTIFY)
        return fail("VP8 remote DTLS peer closed");
    }
    Datagram packet;
    for (unsigned count = 0; count < 64 &&
        (receiveMedia ? receiveMedia->pop(packet) : media.pop(packet)); ++count) {
      int size = int(packet.size);
      if (size < 12) continue;
      const bool rtcp = packet.bytes[1] >= 192 && packet.bytes[1] <= 223;
      if (rtcp) {
        // Authenticate reports even though scheduling does not consume them yet.
        dtls_srtp_decrypt_rtcp_packet(crypto, packet.bytes.data(), &size);
        continue;
      }
      if (sending) continue;
      if (dtls_srtp_decrypt_rtp_packet(crypto, packet.bytes.data(), &size) != 0 || size <= 0) {
        ++authenticationDrops;
        continue;
      }
      const auto bytes = std::span(packet.bytes.data(), size_t(size));
      const auto parsed = rtp::parse(bytes);
      if (!parsed) continue;
      if (parsed->payloadType == audioTrack.payloadType && parsed->ssrc == audioTrack.ssrc) {
        ++audioPackets;
        audio.push(bytes, nowMs(), [&](const rtp::Packet& p) { deliverAudio(p); });
      } else if (video && parsed->payloadType == videoTrack.payloadType && parsed->ssrc == videoTrack.ssrc) {
        const auto distance = uint16_t(parsed->sequence - nextVideoSequence);
        if (!videoPackets || distance < 0x8000) {
          if (videoPackets) videoSequenceGaps += distance;
          nextVideoSequence = uint16_t(parsed->sequence + 1);
        } else ++videoLatePackets;
        ++videoPackets;
        if (parsed->marker) ++videoMarkers;
        // A lost keyframe tail must not hide the advertised stream size.
        const auto payload = vp8::parsePayload(parsed->payload);
        if (payload && payload->startsFrame) {
          ++videoStarts;
          if (parsed->payload[0] & 0x20) ++videoNonReferences;
          const auto header = payload->bytes;
          if (!(header[0] & 1)) ++videoKeyframes;
          if (header.size() >= 10 && !(header[0] & 1) && header[3] == 0x9d &&
              header[4] == 0x01 && header[5] == 0x2a) {
            const unsigned width = (header[6] | (unsigned(header[7]) << 8)) & 0x3fff;
            const unsigned height = (header[8] | (unsigned(header[9]) << 8)) & 0x3fff;
            if (width != videoWidth || height != videoHeight) {
              videoWidth = width; videoHeight = height;
              ESP_LOGI(tag, "VP8 RTP dimensions=%ux%u", width, height);
            }
          }
        }
        video->push(bytes, nowMs(), [&](vp8::EncodedFrame f) { deliverVideo(std::move(f)); });
      } else ++unmatchedPackets;
    }
    const auto now = nowMs();
    if (sending) {
      if (now - lastReportMs >= 1000) {
        feedback(false);
        ESP_LOGI(tag, "microphone transmitted packets=%u octets=%u", unsigned(sentPackets), unsigned(sentOctets));
        lastReportMs = now;
      }
      return 0;
    }
    audio.poll(now, [&](const rtp::Packet& p) { deliverAudio(p); });
    if (video) video->poll(now, [&](vp8::EncodedFrame f) { deliverVideo(std::move(f)); });
    const bool pli = video && video->takeKeyframeRequest(now);
    if (pli || now - lastReportMs >= 1000) { feedback(pli); lastReportMs = now; }
    if (now - lastMediaLogMs >= 2000) {
      ESP_LOGI(tag, "receive audio_packets=%u video_packets=%u video_frames=%u unmatched=%u queue_drops=%u needs_keyframe=%d",
          unsigned(audioPackets), unsigned(videoPackets), unsigned(videoFrames),
          unsigned(unmatchedPackets), unsigned(receiveDrops.load()), video && video->needsKeyframe());
      ESP_LOGI(tag, "video RTP starts=%u keyframes=%u markers=%u sequence_gaps=%u late=%u auth_drops=%u nonreferences=%u",
          unsigned(videoStarts), unsigned(videoKeyframes), unsigned(videoMarkers),
          unsigned(videoSequenceGaps), unsigned(videoLatePackets), unsigned(authenticationDrops),
          unsigned(videoNonReferences));
      lastMediaLogMs = now;
    }
    return 0;
  }

  ~Peer() {
    stopped = true;
    if (ice) juice_destroy(ice); // Joins callbacks before their context is freed.
    if (crypto) dtls_srtp_deinit(crypto);
    if (receiveMedia) {
      receiveMedia->~ReceiveQueue();
      heap_caps_free(receiveMedia);
    }
  }
};

int open(esp_peer_cfg_t* cfg, esp_peer_handle_t* handle) {
  if (!cfg || !handle || cfg->extra_size != sizeof(GeaVp8PeerOptions) || !cfg->extra_cfg)
    return ESP_PEER_ERR_INVALID_ARG;
  const bool sending = cfg->audio_dir == ESP_PEER_MEDIA_DIR_SEND_ONLY && cfg->video_dir == ESP_PEER_MEDIA_DIR_NONE;
  const bool receiving = cfg->audio_dir == ESP_PEER_MEDIA_DIR_RECV_ONLY &&
      (cfg->video_dir == ESP_PEER_MEDIA_DIR_NONE ||
       (cfg->video_dir == ESP_PEER_MEDIA_DIR_RECV_ONLY && cfg->video_info.codec == GEA_PEER_VIDEO_CODEC_VP8));
  if ((!sending && !receiving) || cfg->enable_data_channel ||
      cfg->audio_info.codec != ESP_PEER_AUDIO_CODEC_OPUS ||
      cfg->ice_trans_policy == ESP_PEER_ICE_TRANS_POLICY_RELAY) return ESP_PEER_ERR_NOT_SUPPORT;
  const auto options = *static_cast<GeaVp8PeerOptions*>(cfg->extra_cfg);
  if (!options.running || !options.requestKeyframe) return ESP_PEER_ERR_INVALID_ARG;
  void* allocation = heap_caps_malloc(sizeof(Peer), MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
  if (!allocation) return ESP_PEER_ERR_NO_MEM;
  auto* self = new(allocation) Peer;
  self->cfg = *cfg;
  self->options = options;
  self->sending = sending;
  if (!sending) {
    auto* queue = heap_caps_malloc(sizeof(Peer::ReceiveQueue), MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    if (!queue) { self->~Peer(); heap_caps_free(self); return ESP_PEER_ERR_NO_MEM; }
    self->receiveMedia = new(queue) Peer::ReceiveQueue;
  }
  *handle = self;
  return 0;
}

int message(esp_peer_handle_t handle, esp_peer_msg_t* msg) {
  auto* self = static_cast<Peer*>(handle);
  if (!msg || !msg->data || msg->size <= 0) return ESP_PEER_ERR_INVALID_ARG;
  if (msg->type != ESP_PEER_MSG_TYPE_SDP) return ESP_PEER_ERR_NOT_SUPPORT;
  const std::string_view sdp{reinterpret_cast<const char*>(msg->data), size_t(msg->size)};
  auto offer = self->sending ? receive::parseMicrophoneAnswer(sdp) : receive::parseOffer(sdp);
  if (!offer) return ESP_PEER_ERR_BAD_DATA;
  std::lock_guard lock(self->signalingMutex);
  // One initial offer only. ICE restart/renegotiation require a fresh peer.
  if (self->offerAccepted) return ESP_PEER_ERR_WRONG_STATE;
  self->offerAccepted = true;
  self->pending = std::move(offer);
  self->wake();
  return 0;
}
} // namespace

const esp_peer_ops_t* gea_vp8_peer_impl() {
  static const auto ops = [] {
    esp_peer_ops_t result{};
    result.open = open;
    result.send_msg = message;
    result.new_connection = [](esp_peer_handle_t handle) -> int {
      auto* peer = static_cast<Peer*>(handle);
      if (!peer->sending) return ESP_PEER_ERR_NOT_SUPPORT;
      if (peer->offerRequested.exchange(true)) return ESP_PEER_ERR_WRONG_STATE;
      peer->wake();
      return 0;
    };
    result.send_audio = [](esp_peer_handle_t handle, esp_peer_audio_frame_t* frame) {
      return static_cast<Peer*>(handle)->enqueueAudio(frame);
    };
    result.main_loop = [](esp_peer_handle_t handle) {
      auto* peer = static_cast<Peer*>(handle);
      try { return peer->loop(); }
      catch (...) { return peer->fail("VP8 peer allocation or media processing failed"); }
    };
    result.disconnect = [](esp_peer_handle_t handle) {
      static_cast<Peer*>(handle)->stopped = true;
      return 0;
    };
    result.close = [](esp_peer_handle_t handle) {
      auto* peer = static_cast<Peer*>(handle);
      peer->~Peer();
      heap_caps_free(peer);
      return 0;
    };
    return result;
  }();
  return &ops;
}
