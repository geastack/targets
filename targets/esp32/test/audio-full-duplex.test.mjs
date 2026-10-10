// Compile production open/capture methods with codec calls replaced by fakes.
// RX deliberately blocks: opt-in TX must progress while default TX stays gated.
import assert from 'node:assert/strict'
import path from 'node:path'
import { readFileSync } from 'node:fs'
import { spawnSync } from 'node:child_process'
import { fileURLToPath } from 'node:url'

const source = readFileSync(new URL('../chip_bindings/audio/es8311.cpp', import.meta.url), 'utf8')
function section(start, end) {
  const begin = source.indexOf(start)
  const finish = source.indexOf(end, begin + start.length)
  assert.ok(begin >= 0 && finish > begin, start)
  return source.slice(begin, finish)
}
const output = process.env.GEA_TEST_BUILD_DIR
  ? path.join(process.env.GEA_TEST_BUILD_DIR, 'audio-duplex-test')
  : fileURLToPath(new URL('../../esp32-s3-touch-amoled-2.06/build/audio-duplex-test', import.meta.url))
for (const duplex of [0, 1]) {
  const program = `
#include <array>
#include <cassert>
#include <chrono>
#include <condition_variable>
#include <cstdint>
#include <future>
#include <mutex>
#include <thread>
#define GEA_AUDIO_FULL_DUPLEX ${duplex}
#define GEA_AUDIO_ECHO_CANCELLATION 0
${section("#ifndef GEA_AUDIO_RECORD_GAIN_DB", "#if GEA_AUDIO_ECHO_CANCELLATION")}
constexpr bool kHardwareAecReference=false;
constexpr unsigned kAecMicrophoneCount=1;
namespace gea::platform::audio { constexpr int deviceSampleRate=16000; }
constexpr bool kPairedAecReference=false;
constexpr bool kAecAggressiveNlp=true;
constexpr float kAecMicGain=12.0f, kAecOutputGain=1.0f;
#define ESP_CODEC_DEV_MAKE_CHANNEL_MASK(n) (1u << (n))
#define ESP_LOGE(...) ((void)0)
#define ESP_LOGI(...) ((void)0)
using esp_err_t=int;
constexpr int ESP_OK=0, ESP_ERR_NOT_SUPPORTED=1, ESP_ERR_INVALID_STATE=2, ESP_FAIL=3, ESP_CODEC_DEV_TYPE_IN=1, I2S_MCLK_MULTIPLE_256=256;
struct Channel { bool enabled=false, deleted=false, failDelete=false, failDisable=false; };
using i2s_chan_handle_t=Channel*;
int i2s_channel_disable(Channel* channel) {
  assert(!channel->deleted);
  if (channel->failDisable) return ESP_FAIL;
  if (!channel->enabled) return ESP_ERR_INVALID_STATE;
  channel->enabled=false; return ESP_OK;
}
int i2s_del_channel(Channel* channel) {
  assert(!channel->deleted);
  if (channel->enabled) return ESP_ERR_INVALID_STATE;
  if (channel->failDelete) return ESP_FAIL;
  channel->deleted=true; return ESP_OK;
}
struct i2s_chan_info_t { size_t total_dma_buf_size=5760; };
int i2s_channel_get_info(Channel*, i2s_chan_info_t*) { return ESP_OK; }
size_t drainedBytes=0;
int esp_codec_dev_write(int,void*,size_t count) { drainedBytes+=count; return ESP_OK; }
struct esp_codec_dev_sample_info_t { uint8_t bits_per_sample, channel; uint16_t channel_mask; uint32_t sample_rate; int mclk_multiple; };
int esp_codec_dev_open(int, esp_codec_dev_sample_info_t*) { return ESP_OK; }
void esp_codec_dev_set_in_gain(int,float) {}
int esp_codec_dev_set_in_channel_gain(int,uint16_t,float) { return ESP_OK; }
namespace gea::chips::es8311 {
struct OutputFormat { OutputFormat(int,int,int) {} bool isPcm16() const { return true; } };
}
std::mutex gate;
std::condition_variable changed;
bool reading=false, releaseRead=false;
int esp_codec_dev_read(int, void*, size_t) {
  std::unique_lock<std::mutex> lock(gate);
  reading=true; changed.notify_all();
  changed.wait(lock, [] { return releaseRead; });
  return ESP_OK;
}
struct Driver {
  Channel tx, rx;
  Channel *txChannel_=&tx, *rxChannel_=&rx;
  void* i2sDataIf_=nullptr;
  bool recordOpen_=false, speakerOpen_=false;
  int i2sSampleRate_=16000, speakerSampleRate_=16000, speakerChannels_=1, speakerBitsPerSample_=16;
  int speakerCodec_=1, recordCodec_=2;
  int micCloses=0, speakerCloses=0;
  std::mutex writeMutex_, readMutex_;
  std::array<int16_t,640> stereoFrame_{};
  void applySpeakerVolumeLocked() {}
  void closeRecorderLocked() { recordOpen_=false; ++micCloses; }
  void closeSpeakerLocked() { speakerOpen_=false; ++speakerCloses; }
  int initSpeaker(int) { return ESP_OK; }
  int initCodecDevice(int,int,int&) { return ESP_OK; }
${section('  bool open(int sampleRate', '  bool write(')}
${section('  esp_err_t initRecorder(', '  esp_err_t initCodecDevice(')}
${section('  esp_err_t disableI2sChannel(', '  // Frees everything')}
  void captureOnce() {
${section('      int err = ESP_OK;', '      if (err != ESP_OK) {\n        if (consecutiveReadErrors')}
    assert(err==ESP_OK);
  }
};
int main() {
  Driver cleanup;
  cleanup.tx.enabled=true; // The codec enabled TX implicitly for RX's clock.
  cleanup.releaseI2s();
  assert(cleanup.tx.deleted && cleanup.rx.deleted);
  assert(!cleanup.txChannel_ && !cleanup.rxChannel_);
  Driver deletionFailure;
  deletionFailure.tx.failDelete=true;
  deletionFailure.releaseI2s();
  assert(deletionFailure.txChannel_ && !deletionFailure.rxChannel_);
  deletionFailure.tx.failDelete=false;
  deletionFailure.releaseI2s();
  assert(!deletionFailure.txChannel_);
  Driver disableFailure;
  disableFailure.tx.enabled=true; disableFailure.tx.failDisable=true;
  disableFailure.releaseI2s();
  assert(disableFailure.txChannel_ && !disableFailure.tx.deleted);
  disableFailure.tx.failDisable=false;
  disableFailure.releaseI2s();
  assert(!disableFailure.txChannel_);
  Driver handoff;
  handoff.speakerOpen_=true;
  assert(handoff.initRecorder(16000)==ESP_OK);
  assert(drainedBytes==(GEA_AUDIO_FULL_DUPLEX ? 0 : 5760));
  Driver recorder; recorder.recordOpen_=true;
  assert(recorder.open(16000,1,16));
  assert(recorder.recordOpen_==bool(GEA_AUDIO_FULL_DUPLEX));
  assert(recorder.micCloses==(GEA_AUDIO_FULL_DUPLEX?0:1));
  Driver speaker; speaker.speakerOpen_=true;
  assert(speaker.initRecorder(16000)==ESP_OK);
  assert(speaker.speakerOpen_==bool(GEA_AUDIO_FULL_DUPLEX));
  assert(speaker.speakerCloses==(GEA_AUDIO_FULL_DUPLEX?0:1));
  if (GEA_AUDIO_FULL_DUPLEX) {
    Driver conflict; conflict.recordOpen_=true;
    assert(!conflict.open(24000,1,16));
    assert(conflict.recordOpen_ && conflict.micCloses==0);
    Driver conflict2; conflict2.speakerOpen_=true; conflict2.speakerSampleRate_=24000;
    assert(conflict2.initRecorder(16000)==ESP_ERR_NOT_SUPPORTED);
    assert(conflict2.speakerOpen_ && conflict2.speakerCloses==0);
  }
  Driver simultaneous;
  std::thread capture([&] { simultaneous.captureOnce(); });
  { std::unique_lock<std::mutex> lock(gate); changed.wait(lock, [] { return reading; }); }
  std::promise<void> attempted, wrote;
  auto attemptedFuture=attempted.get_future(), wroteFuture=wrote.get_future();
  std::thread writer([&] {
    attempted.set_value();
    std::lock_guard<std::mutex> lock(simultaneous.writeMutex_);
    wrote.set_value();
  });
  attemptedFuture.wait();
  const bool progressed=wroteFuture.wait_for(std::chrono::milliseconds(200))==std::future_status::ready;
  assert(progressed==bool(GEA_AUDIO_FULL_DUPLEX));
  { std::lock_guard<std::mutex> lock(gate); releaseRead=true; } changed.notify_all();
  capture.join(); writer.join();
}
`
  const compile = spawnSync(process.env.CXX || 'c++', ['-std=c++17', '-pthread', '-fsanitize=address,undefined', '-x', 'c++', '-', '-o', output], {input: program, encoding:'utf8'})
  assert.equal(compile.status, 0, compile.stderr)
  const run = spawnSync(output, [], {encoding:'utf8', timeout:10000})
  assert.equal(run.status, 0, run.stderr || String(run.error))
}
console.log('Audio half/full-duplex ownership, clock conflict, and concurrent RX/TX passed')
