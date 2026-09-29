// Compile production open/capture methods with codec calls replaced by fakes.
// RX deliberately blocks: opt-in TX must progress while default TX stays gated.
import assert from 'node:assert/strict'
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
const output = fileURLToPath(new URL('../../esp32-s3-touch-amoled-2.06/build/audio-duplex-test', import.meta.url))
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
#define ESP_LOGE(...) ((void)0)
#define ESP_LOGI(...) ((void)0)
using esp_err_t=int;
constexpr int ESP_OK=0, ESP_ERR_NOT_SUPPORTED=1, ESP_CODEC_DEV_TYPE_IN=1, I2S_MCLK_MULTIPLE_256=256;
struct esp_codec_dev_sample_info_t { uint8_t bits_per_sample, channel; uint32_t sample_rate; int mclk_multiple; };
int esp_codec_dev_open(int, esp_codec_dev_sample_info_t*) { return ESP_OK; }
void esp_codec_dev_set_in_gain(int,float) {}
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
  void captureOnce() {
${section('      int err = ESP_OK;\n      {', '      if (err != ESP_OK) {\n        if (consecutiveReadErrors')}
    assert(err==ESP_OK);
  }
};
int main() {
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
