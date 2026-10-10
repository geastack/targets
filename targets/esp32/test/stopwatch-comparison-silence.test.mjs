import assert from 'node:assert/strict'
import { readFileSync } from 'node:fs'
import { spawnSync } from 'node:child_process'
import { fileURLToPath } from 'node:url'

const audio = readFileSync(new URL('../chip_bindings/audio/es8311.cpp', import.meta.url), 'utf8')
const power = readFileSync(
  new URL('../../esp32-s3-m5stack-stopwatch/main/power.cpp', import.meta.url),
  'utf8',
)
const output = fileURLToPath(
  new URL('../../../build/stopwatch-comparison-silence-test', import.meta.url),
)

function method(source, signature) {
  const start = source.indexOf(signature)

  assert.notEqual(start, -1, `missing production method ${signature}`)
  let end = source.indexOf('{', start) + 1
  let depth = 1

  while (depth > 0 && end < source.length) {
    if (source[end] === '{') {
      depth++
    } else if (source[end] === '}') {
      depth--
    }

    end++
  }

  assert.equal(depth, 0)

  return source.slice(start, end)
}

// Execute the actual hardware-authority methods, without touching hardware.
// UI requests remain at 100 while each diagnostic output is physically off.
for (const enabled of [0, 1]) {
  const program = `
#include <algorithm>
#include <cassert>
#include <cmath>
#include <cstdint>
#include <mutex>
using std::int64_t;
using std::uint16_t;
using esp_err_t = int;
#define GEA_EMBEDDED_COMPARISON_BENCHMARK ${enabled}
#define GEA_BOARD_SPEAKER_POWER 1
#define GEA_BOARD_HAS_EXPANDER 0
#define ESP_LOGW(...)
#define pdMS_TO_TICKS(value) (value)
constexpr int ESP_OK = 0, ESP_FAIL = -1, ESP_CODEC_DEV_OK = 0;
constexpr int GPIO_NUM_NC = -1, GPIO_NUM_14 = 14, kPaPin = GPIO_NUM_NC;
std::mutex busMutex;
bool ready = true, amplifierPin = false;
int amplifierGpio = -1, codecOutput = -1, pwmOutput = -1;
int ioe = 0;
int64_t motorUntil = -1;
void pin(int index, bool value) { assert(index == 9); amplifierPin = value; }
int gpio_set_level(int index, int value) {
  assert(index == GPIO_NUM_14); amplifierGpio = value; return ESP_OK;
}
void vTaskDelay(int) {}
bool write16(int, int reg, uint16_t value) {
  assert(reg == 0x1B); pwmOutput = value; return true;
}
int64_t esp_timer_get_time() { return 1000; }
int esp_codec_dev_set_out_vol(void *, int value) { codecOutput = value; return ESP_OK; }
namespace gea::platform::board {
${method(power, 'bool setSpeakerPower(bool enabled)')}
}
${method(audio, 'esp_err_t setAmplifierEnabled(bool enabled)')}
class AudioOutputDriver {
public:
  void *speakerCodec_ = this;
  int speakerVolume_ = 100;
  ${method(audio, 'static int codecVolumeFromUserPercent(int volumePercent)')}
  ${method(audio, 'void applySpeakerVolumeLocked()')}
};
class Power { public: static bool vibrate(double duration, int strength); };
${method(power, 'bool Power::vibrate(double duration, int strength)')}
int main() {
  AudioOutputDriver driver;
  driver.applySpeakerVolumeLocked();
  assert(driver.speakerVolume_ == 100);
  assert(codecOutput == (${enabled} ? 0 : 100));
  assert(amplifierPin == !${enabled});
  assert(amplifierGpio == (${enabled} ? 0 : 1));
  // Direct board callers also cannot re-enable the amplifier in diagnostics.
  assert(gea::platform::board::setSpeakerPower(true));
  assert(amplifierPin == !${enabled});
  assert(amplifierGpio == (${enabled} ? 0 : 1));
  assert(Power::vibrate(200, 100));
  assert(pwmOutput == (${enabled} ? 0x8000 : 0x8FFF));
  assert(motorUntil == (${enabled} ? 0 : 201000));
  driver.speakerVolume_ = 0;
  driver.applySpeakerVolumeLocked();
  assert(codecOutput == 0 && !amplifierPin && amplifierGpio == 0);
}
`
  const compiled = spawnSync(
    process.env.CXX || 'clang++',
    [
      '-std=c++20',
      '-fsanitize=address,undefined',
      '-fno-omit-frame-pointer',
      '-x',
      'c++',
      '-',
      '-o',
      output,
    ],
    { input: program, encoding: 'utf8' },
  )

  assert.equal(compiled.status, 0, compiled.stderr)
  const result = spawnSync(output, [], { encoding: 'utf8' })

  assert.equal(result.status, 0, result.stderr)
}

console.log(
  'StopWatch comparison silence: production output and diagnostic speaker/motor clamps verified',
)
