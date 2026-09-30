// Exercise the actual board reset against a fake register bus. Unrelated
// expander pins must survive, and failed transactions must release the handle.
import assert from 'node:assert/strict'
import { readFileSync } from 'node:fs'
import { spawnSync } from 'node:child_process'
import path from 'node:path'
const source = readFileSync(new URL('../main/panel_reset.cpp', import.meta.url), 'utf8')
  .replace(/^#include .*\n/gm, '')
if (!process.env.TMPDIR) throw new Error('Set TMPDIR to existing build output')
const binary = path.join(process.env.TMPDIR, 'amoled-18-panel-reset-test')
const harness = `
#include <cstdint>
#include <cstdio>
#include <vector>
#include <cassert>
using esp_err_t = int;
constexpr int ESP_OK = 0, I2C_ADDR_BIT_LEN_7 = 7;
using i2c_master_bus_handle_t = void*;
using i2c_master_dev_handle_t = void*;
struct i2c_device_config_t { int dev_addr_length, device_address, scl_speed_hz; };
struct Fake { uint8_t output, direction; int calls{}, fail{}, removed{}, errors{}; std::vector<int> delays, writes; } fake;
#define ESP_LOGI(...) ((void)0)
#define ESP_LOGE(...) (++fake.errors)
const char* esp_err_to_name(int) { return "error"; }
int pdMS_TO_TICKS(int ms) { return ms; }
void vTaskDelay(int ms) { fake.delays.push_back(ms); }
namespace gea::platform::i2c {
struct Bus {
 static Bus primary() { return {}; }
 bool available() const { return true; }
 void* nativeHandle() const { return &fake; }
};
}
int i2c_master_bus_add_device(void* bus, const i2c_device_config_t* c, void** out) {
 assert(bus == &fake && c->device_address == 0x20); *out=&fake; return 0;
}
int i2c_master_transmit_receive(void*, const uint8_t* reg, int, uint8_t* out, int, int) {
 if (++fake.calls == fake.fail) return -1;
 assert(*reg==1 || *reg==3); *out=*reg==1 ? fake.output : fake.direction; return 0;
}
int i2c_master_transmit(void*, const uint8_t* bytes, unsigned long n, int) {
 if (++fake.calls == fake.fail) return -1;
 assert(n==2); fake.writes.push_back(bytes[0]);
 if (bytes[0]==1) fake.output=bytes[1]; else { assert(bytes[0]==3); fake.direction=bytes[1]; }
 return 0;
}
void i2c_master_bus_rm_device(void*) { ++fake.removed; }
${source}
int main() {
 for (unsigned output=0; output<256; ++output) for(unsigned direction=0; direction<256; ++direction) {
  fake={uint8_t(output),uint8_t(direction)};
  gea::platform::board::prepareDisplayPanel();
  assert(fake.output==(output|7) && fake.direction==(direction&~7));
  assert(fake.removed==1 && fake.errors==0);
  assert((fake.delays==std::vector<int>{20,50}));
  assert((fake.writes==std::vector<int>{1,3,1}));
 }
 for (int failure=1; failure<=5; ++failure) {
  fake={0xa8,0xff}; fake.fail=failure;
  gea::platform::board::prepareDisplayPanel();
  assert(fake.removed==1 && fake.errors==1 && fake.calls==failure);
  assert((fake.output&~7)==0xa8 && (fake.direction&~7)==0xf8);
 }
 puts("1.8 reset preserves other pins, orders the pulse, and cleans up all transaction failures");
}
`
const built = spawnSync(process.env.CXX || 'c++', ['-std=c++17', '-x', 'c++', '-', '-o', binary], { input: harness, encoding: 'utf8' })
assert.equal(built.status, 0, built.stderr || String(built.error))
const ran = spawnSync(binary, [], { encoding: 'utf8' })
assert.equal(ran.status, 0, ran.stderr || String(ran.error))
process.stdout.write(ran.stdout)
