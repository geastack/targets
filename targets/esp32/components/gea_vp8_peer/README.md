# Native VP8/Opus media peer

An `esp_peer_ops_t` implementation for bundled Opus/VP8 reception and a separate
audio-only Opus microphone sender. The shared Gea host selects reception from
SDP, or sending from an audio-only send transceiver with no configured ICE
servers (the hosted SFU supplies publicly reachable candidates). Other codec and data-channel
connections keep Espressif's default peer.

- libjuice supplies ICE, including connectivity checks and consent freshness.
- The pinned Espressif peer supplies its IDF-specific DTLS/SRTP implementation.
- SHA-256 of the remote DTLS certificate must match the authenticated signaling
  SDP before any media reaches a callback.
- Fixed packet queues live in the PSRAM peer allocation. Normal in-order media
  has no artificial packet wait; sequence gaps get at most a 30 ms reorder wait.
- The microphone encoder enqueues RTP packets; the peer worker performs SRTP
  and sends them immediately, without sharing crypto state across tasks. Its
  negotiated SSRC, Opus payload type and 48 kHz RTP clock match the local SDP.
  Sender reports include packet/octet counts and the corresponding RTP clock.
  Only a video receiver allocates the 64-packet incoming media queue.
- VP8 loss or decoder overflow triggers rate-limited PLI. RTX and generic NACK
  are not advertised. Encoded reference frames cannot be skipped silently.
- Closing sets the shared running flag, aborting a pending handshake; cleanup
  joins the ICE worker before releasing its callback context.

The receiver accepts one send-only Opus stream and one send-only VP8 stream in
one BUNDLE. The microphone sender offers a single send-only Opus stream and
requires a matching recv-only answer with passive DTLS and RTCP multiplexing.
It does not implement data channels, video sending,
relay-only policy, renegotiation or ICE restarts. The publicly reachable hosted
SFU is reached through outgoing ICE checks, including peer-reflexive NAT mapping.

RTP timestamps are carried into the existing audio/video callbacks, but received
RTCP sender-report clock synchronization is not implemented. No claim of measured
A/V synchronization or S3 frame rate is made by the portable packet tests.

Test the shared SDP, RTP, keyframe recovery and real VP8 decoding with
`bash packages/core/test/run-vp8-tests.sh` in the core repo. The transport
component cross-compiles against ESP-IDF 6.0.2; a full device conversation still
needs verification.
