#include "fake_magnetometer.h"
#include "gea_runtime.h"
#include "host/magnetometer.h"
#include <cassert>
#include <cstdio>
#include <limits>

namespace {
struct Fake {
  bool attached = false, failRead = false, failInit = false, failRemove = false;
  bool ready = true;
  bmm150_mag_data value{};
};
std::array<Fake, 2> devices;
int busToken = 1;
bool busReady = true;
unsigned attachments = 0, removals = 0, delays = 0, reads = 0;
std::int64_t now = 1000;
Fake &sensor(bmm150_dev *device) {
  assert(device && device->intf_ptr);
  return *static_cast<Fake *>(device->intf_ptr);
}
} // namespace
void gea::platform::board::powerOnRails() {}
gea::platform::i2c::Bus gea::platform::i2c::Bus::primary() { return {}; }
bool gea::platform::i2c::Bus::available() const { return busReady; }
void *gea::platform::i2c::Bus::nativeHandle() const { return &busToken; }
std::int64_t esp_timer_get_time() { return now; }
void vTaskDelay(unsigned ms) { delays += ms; }
int bmm150_interface_init_from_master_bus(bmm150_dev *device, void *bus,
                                          unsigned address,
                                          unsigned frequency) {
  assert(bus == &busToken && frequency == 400000);
  assert(address == 0x11 || address == 0x12);
  auto &fake = devices[address - 0x11];
  assert(!fake.attached);
  fake.attached = true;
  device->intf_ptr = &fake;
  ++attachments;
  return 0;
}
int bmm150_interface_deinit_device(bmm150_dev *device) {
  auto &fake = sensor(device);
  if (fake.failRemove)
    return -1;
  fake.attached = false;
  device->intf_ptr = nullptr;
  ++removals;
  return 0;
}
int bmm150_init(bmm150_dev *device) { return sensor(device).failInit ? -1 : 0; }
int bmm150_set_op_mode(bmm150_settings *settings, bmm150_dev *) {
  assert(settings->pwr_mode == BMM150_POWERMODE_NORMAL);
  return 0;
}
int bmm150_set_presetmode(bmm150_settings *settings, bmm150_dev *) {
  assert(settings->preset_mode == BMM150_PRESETMODE_REGULAR);
  return 0;
}
int bmm150_get_regs(unsigned reg, std::uint8_t *value, unsigned size,
                    bmm150_dev *device) {
  assert(reg == BMM150_REG_DATA_READY_STATUS && size == 1);
  *value = sensor(device).ready ? 1 : 0;
  return 0;
}
int bmm150_read_mag_data(bmm150_mag_data *out, bmm150_dev *device) {
  auto &fake = sensor(device);
  ++reads;
  now += 100; // back-to-back reads really complete at different times
  if (fake.failRead)
    return -1;
  *out = fake.value;
  return 0;
}

int main() {
  using gea::platform::sensors::Magnetometer;
  using gea::platform::sensors::MagnetometerReading;
  std::array<MagnetometerReading, 2> pair;
  assert(Magnetometer::count() == 2);
  assert(!Magnetometer::readAll(pair.data(), pair.size()));
  assert(!pair[0].valid && !pair[1].valid);
  busReady = false;
  assert(!Magnetometer::open() && attachments == 0);
  busReady = true;
  devices[1].failInit = true;
  assert(!Magnetometer::open());
  assert(devices[0].attached && !devices[1].attached && removals == 1);
  devices[1].failInit = false;
  assert(Magnetometer::open() && attachments == 3);
  assert(Magnetometer::open() && attachments == 3); // idempotent
  devices[0].value = {-32768, 140, 0};
  devices[1].value = {24, 32767, -32768};
  assert(Magnetometer::readAll(pair.data(), pair.size()));
  assert(pair[0].valid && pair[1].valid);
  assert(pair[0].x == -32768 && pair[0].saturatedAxes == 1);
  assert(pair[1].y == 32767 && pair[1].saturatedAxes == 6);
  assert(pair[0].timestampUs == static_cast<std::uint64_t>(now) &&
         pair[1].timestampUs == static_cast<std::uint64_t>(now));
  devices[0].failRead = true;
  assert(!Magnetometer::readAll(pair.data(), pair.size()));
  assert(!pair[0].valid && pair[0].x == 0 && pair[1].valid);
  devices[0].failRead = false;
  const auto before = reads;
  assert(!Magnetometer::readAll(pair.data(), 1) && reads == before);
  assert(!pair[0].valid);
  MagnetometerReading invalid{};
  assert(!Magnetometer::read(2, invalid) && !invalid.valid && reads == before);

  // Use the compiler's actual const Float64Array view, including its byte
  // offset, rather than a std::array proxy for the generated host boundary.
  auto storage = gea::makeRef<gea::TypedArray<double>>(9);
  auto view = storage->subarray(2, 7);
  const auto &hostView = gea::detail::hostTypedArrayArgument(view);
  assert(gea::host::Magnetometer.read(1, hostView));
  assert(storage->data()[1] == 0 && storage->data()[2] == 24);
  assert(storage->data()[3] == 32767 && storage->data()[5] == 6);
  assert(storage->data()[6] == static_cast<double>(now) &&
         storage->data()[7] == 0);
  for (double bad :
       {-1., 1.5, 4294967296., std::numeric_limits<double>::quiet_NaN()}) {
    const auto previous = reads;
    assert(!gea::host::Magnetometer.read(bad, hostView));
    assert(reads == previous && storage->data()[2] == 0 &&
           storage->data()[6] == 0);
  }
  std::array<double, 4> small{1, 2, 3, 4};
  assert(!gea::host::Magnetometer.read(0, small) && small[0] == 1);

  devices[1].failRemove = true;
  Magnetometer::close();
  assert(!devices[0].attached && devices[1].attached);
  assert(!Magnetometer::read(1, invalid));
  assert(!Magnetometer::open()); // failed remove must not lose its handle
  devices[1].failRemove = false;
  assert(Magnetometer::open());
  Magnetometer::close();
  assert(!devices[0].attached && !devices[1].attached);
  devices[0].ready = false;
  assert(!Magnetometer::open() && delays == 200 && !devices[0].attached);
  devices[0].ready = true;
  assert(Magnetometer::open());
  Magnetometer::close();
  std::puts("Magnetometer: shared bus, pair samples, sentinel ABI, lifecycle "
            "and typed views passed");
}
