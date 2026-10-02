# ESP32-S3 VP8 kernels

This component builds pinned upstream libvpx 1.16.0 with VP8 decoding only.
On ESP32-S3 it selects Gea's native PIE assembly at libvpx's generated dispatch
boundary. It does not require an application setting or fork the bitstream parser.
Other CPUs retain upstream C implementations.

Native kernels:

- I420 to RGB565: eight pixels, exact limited-range BT.601 coefficients,
  40-bit accumulators, clipping and packing; no intermediate RGB888 image.
- Bilinear and six-tap prediction: eight pixels, separate rounding/clipping
  for horizontal and vertical passes, including 4-pixel blocks. Integer-pixel
  axes skip their identity pass and intermediate buffer entirely.
  Odd six-tap phases omit the two zero outer coefficients, reducing both SIMD
  loads/multiplies and the vertical intermediate extent without changing pixels.
  The assembly call covers a complete column group across all block rows,
  retaining constants in PIE registers and using a hardware tap loop.
- Integer-motion block copies: complete 16×16, 8×8, 8×4 and 4×4 blocks with
  arbitrary padded source alignment and exact destination stores. Odd destination
  alignment retains the scalar path.
- True-motion intra prediction: 4×4, 8×8 and 16×16 blocks; eight signed lanes
  compute and clip the left/top predictor. Unaligned top edges are copied into
  a small aligned stack buffer; no plane or frame copy is needed.
- Full 4×4 IDCT and inverse Walsh transform: four parallel 32-bit lanes.
  First-pass short truncation and signed rounding match upstream. DC-only
  reconstruction uses bounded signed 16-bit lanes.
- Dequantization: eight signed 16-bit products, preserving short wraparound,
  fused into IDCT without writing/reloading an intermediate coefficient block.
  The whole Y/UV block wrappers explicitly call these native leaf kernels;
  upstream's scalar block functions bypass generated leaf dispatch. Zero DC
  residuals skip pixel arithmetic while preserving coefficient clearing.
- Normal, macroblock and simple deblocking: eight lanes, with native edge
  transpose, widening and packing. Zero corrections skip both pixel-update
  arithmetic and the scatter back into reference planes.
  Horizontal stores write only the affected rows (four normal, six macroblock,
  two simple), avoiding dirty/writeback traffic on unchanged reference rows.
- RGB565 panel byte order: fused into the I420 SIMD conversion before the
  single output write; eight pixels per instruction group. The independent
  in-place SIMD swap remains available for other RGB565 sources.
  AMOLED rendering retains the decoder buffer rather than expanding to RGB888
  and allocating another full-frame image.
  VP8 reserves a bounded three-buffer pixel pool, recycling storage only after
  all frame readers release it. Frame identities and weak-reference expiration
  remain independent of recycled storage; a slow sink cannot force pool growth.
  Paired luma rows reuse their widened, centered chroma registers. SIMD byte
  deinterleave/reinterleave packs panel endian output directly, replacing two
  mask loads and the per-vector shift/mask sequence. Scalar BT.601 comparisons
  cover both byte orders independently, including odd final rows and guards.

Entropy decoding is serial native C: S3 builds inline the arithmetic-bit loop,
use NSAU for normalization instead of a table lookup, and retain arithmetic
state locally per macroblock. Checked, idempotent patches apply to the pinned
upstream implementation at configure time. Parsing, control flow and directional
intra prediction remain upstream C. This is not a claim that every instruction
in VP8 is SIMD.

The codec kernels use libvpx's padded frame borders and aligned coefficient
blocks. The I420 kernel reads exactly its requested span: its C++ dispatcher
checks source/output alignment and handles odd dimensions and short tails.
Video allocations use 16-byte alignment so the native path is deterministic.
Opaque framebuffer painting also uses Gea's native PIE span-copy helper. It
handles the cropped 396-to-368-pixel rows without requiring matching source and
destination alignment, keeps vector loads within the requested span, and
preserves panel byte order. The on-board check covers 25,408 offset/length cases
with guards; a warm cropped-row microbenchmark measured 1.76× versus memcpy.
This is a row-copy speedup, not a claim about complete live FPS.
Assembly routines use caller-saved PIE state; ESP-IDF saves this state across
task switches. No interrupt masking or global kernel lock is used.

The decoder scopes a task-local 672-byte internal-RAM workspace around decode.
Interpolation and deblocking reuse it sequentially; reference planes, output
images and worker stacks remain in PSRAM. Nested bindings restore the previous
workspace. If the bounded internal allocation cannot be made, the unchanged
aligned stack path remains available; live startup reports the actual allocation.

## Verification

`GEADEV SIMDBENCH` runs on a joined PSRAM worker, with no microphone, speaker,
network session or cloud credits. It compares the actual S3 instructions with
the preserved upstream C functions:

- Every one of the 16,777,216 Y/U/V combinations, plus distinct chroma lanes.
- 4,096 entropy streams with full state comparisons after every bit, including
  all normalization ranges, probability extremes, truncated and encrypted input.
- All subpixel offsets and source alignments for all eight prediction kernels.
- 4,096 integer-motion block copies with every source/destination offset,
  odd and aligned strides, and complete destination guards.
- 1,536 true-motion cases covering all three block sizes, unaligned top edges,
  clipping and untouched destination borders.
- Every RGB565 value for the byte-order conversion, with surrounding guards.
- 4,096 fused-color cases comparing canonical and panel byte order, including
  multi-iteration vectors, mixed chroma, scalar tails and odd padded strides.
- 4,096 IDCT, dequantization, Walsh and DC cases, including extreme coefficients.
- 1,024 whole Y/UV macroblocks with mixed EOB values, extreme coefficients,
  zero DC and short-wrap-to-zero products, comparing all pixels and coefficients.
- 6,144 deblocking cases covering smooth edges, noise, constant planes at every
  byte value, all orientations, chroma planes, unaligned addresses and untouched
  surrounding pixels. Internal-workspace guards are also checked.

Only an `END` following every `exact=1` is a pass. Timing excludes the comparison
itself. `GEADEV VP8BENCH /storage/gea-vp8-benchmark.ivf` measures complete decoding
of a saved clip; its conversion time includes the separately reported buffer
allocation/initialization time. Neither diagnostic establishes live A/V quality.

`GEADEV VP8PROFILE ON` enables task-local stage clocks; `OFF` disables them.
They default off on boot and add no tasks, timers or interrupts. The saved-clip
and live tools accept `--profile-video` and disable profiling during cleanup.
Stage durations are wall time (including preemption). Token time is nested in
macroblock time; reports subtract it before labeling reconstruction time.

`GEADEV VP8CAPTURE START` optionally retains the first accepted keyframe and its
reference prefix by moving encoded buffers, capped at 12 frames / 128 KiB. It
defaults off and makes no live flash writes. After the decoder worker joins,
`GEADEV VP8CAPTURE SAVE` writes `/storage/gea-vp8-live-benchmark.ivf` and reports
each frame's live wall and CPU durations. `CANCEL` releases retained buffers.
Starting/saving while a video worker is running is rejected. Replay this file
with `VP8BENCH` to compare the same content and dimensions under live and offline
loads. Offline frame reports include `cpu_us`; CPU counters exclude preemption,
while wall durations include it. The app diagnostic tools handle capture cleanup.

`GEADEV SIMDBENCH kernels` omits the unchanged exhaustive color sweep and runs
the codec differential checks in a few seconds. `GEADEV SIMDBENCH swap` checks
only byte order. On the AMOLED 1.8, the kernel check measured filtering at
104,887 µs versus 215,804 µs scalar; true-motion at 3,529 versus 11,288 µs;
IDCT at 7,590 versus 13,832 µs; and deblocking at 58,491 versus 104,750 µs.
All comparisons were exact. These are isolated kernel timings, not live FPS.

The zero-correction/workspace build on AMOLED 1.8 measured 6,144 deblocking cases
at 65,458 µs versus 157,102 µs scalar (2.4×), and 8,064 interpolation cases at
92,977 µs versus 214,627 µs scalar (2.3×). The same saved 60-frame clip completed
in 3.333 seconds (18.0 decoded frames/s). A subsequent live check still failed
with UI watchdog stalls; none of these isolated timings establish live FPS.
