# Gea ICE transport

Pinned ESP32 libjuice fork: `0x1abin/libjuice` at
`34bae17c9c361639f64347da689503d3b06c0803`, SHA-256 verified at configure.
Upstream MPL-2.0 source and license remain in CMake's dependency output.

This component supplies codec-independent ICE datagrams for the VP8 receiver.
It does not replace DTLS/SRTP, authenticate media, or decode video itself.
The existing Espressif audio peer is unaffected.

All libjuice heap allocations and its 16 KiB worker stack use PSRAM. Its
FreeRTOS thread adapter is patched at configure time to join, suspend, and
delete external-stack tasks correctly. The patch is idempotent and rejects
unexpected upstream source. Mutexes and kernel task metadata remain in the
memory required by FreeRTOS. No silent internal-RAM allocation fallback.

The shared network build includes this dependency for `gea_vp8_peer`. Its code
is only used for incoming VP8 offers; the default audio/H264 peer still uses
Espressif's existing ICE implementation. Hardware media verification remains
separate from compilation.
