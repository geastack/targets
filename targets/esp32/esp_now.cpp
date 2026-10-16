#include "sdkconfig.h"
#if CONFIG_ESP_WIFI_ENABLED && !CONFIG_ESP_HOST_WIFI_ENABLED &&                \
    __has_include("esp_now.h")
#include "connectivity/wifi_task_stack.h"
#include "esp_event.h"
#include "esp_heap_caps.h"
#include "esp_idf_version.h"
#include "esp_log.h"
#include "esp_now.h"
#include "esp_now_power.h"
#include "esp_now_rate.h"
#include "esp_now_tx_header.h"
#include "esp_timer.h"
#include "esp_wifi.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "platform/esp_now.h"
#include <atomic>
#include <cstring>
#include <new>

namespace {
using namespace gea::platform::espnow;
constexpr unsigned kQueue = 64;
struct State {
  Datagram rx[kQueue];
  std::atomic<unsigned> write{0}, read{0}, sent{0}, failed{0}, dropped{0},
      received{0}, maxSend{0};
  std::atomic<bool> busy{false}, open{false}, captureTxHeader{false};
  std::int64_t sentAt = 0;
  std::atomic<std::uint64_t> completedStart{0}, completedAt{0};
  std::atomic<unsigned> completedRate{0}, completedStatus{0};
  std::atomic<std::uint32_t> completedMacControl{0}, completedMacSequence{0};
  SemaphoreHandle_t event = nullptr;
  bool ownsRadio = false;
  wifi_ps_type_t previousPs = WIFI_PS_NONE;
  int rate = 54, channel = 0;
};
State *state = nullptr;
void received(const esp_now_recv_info_t *info, const std::uint8_t *bytes,
              int size) {
  auto *s = state;
  if (!s || !s->open.load(std::memory_order_acquire) || !info ||
      !info->src_addr || !bytes || size <= 0 || size > 1470)
    return;
  const unsigned w = s->write.load(std::memory_order_relaxed);
  if (w - s->read.load(std::memory_order_acquire) >= kQueue) {
    ++s->dropped;
    return;
  }
  auto &p = s->rx[w % kQueue];
  std::memcpy(p.source.data(), info->src_addr, 6);
  std::memcpy(p.bytes.data(), bytes, size);
  p.size = size;
  p.receivedUs = esp_timer_get_time();
  p.rssi = info->rx_ctrl ? info->rx_ctrl->rssi : 0;
  p.radioTimestampUs = info->rx_ctrl ? info->rx_ctrl->timestamp : 0;
  p.radioTimestampValid = info->rx_ctrl != nullptr;
  s->write.store(w + 1, std::memory_order_release);
  ++s->received;
  xSemaphoreGive(s->event);
}
#if ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(5, 5, 0)
void sent(const esp_now_send_info_t *info, esp_now_send_status_t status) {
#else
void sent(const std::uint8_t *, esp_now_send_status_t status) {
#endif
  auto *s = state;
  if (!s)
    return;
  const auto completed = esp_timer_get_time();
  const auto elapsed = static_cast<unsigned>(completed - s->sentAt);
  if (elapsed > s->maxSend.load())
    s->maxSend.store(elapsed);
  if (status == ESP_NOW_SEND_SUCCESS)
    ++s->sent;
  else
    ++s->failed;
  s->completedStart.store(s->sentAt, std::memory_order_relaxed);
  s->completedAt.store(completed, std::memory_order_relaxed);
#if ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(5, 5, 0)
  s->completedRate.store(info ? static_cast<unsigned>(info->rate) : 0,
                         std::memory_order_relaxed);
#endif
  gea::targets::esp32::espnow::TxMacHeader mac{};
#if ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(5, 5, 0)
  if (s->captureTxHeader.load(std::memory_order_acquire))
    mac = gea::targets::esp32::espnow::callbackTxMacHeader(info);
#endif
  s->completedMacControl.store(mac.control, std::memory_order_relaxed);
  s->completedMacSequence.store(mac.sequence, std::memory_order_relaxed);
  s->completedStatus.store(static_cast<unsigned>(status),
                           std::memory_order_relaxed);
  // The sole sender's canSend acquire observes this complete metadata before
  // it admits another datagram. Other statistics readers remain data-race free.
  s->busy.store(false, std::memory_order_release);
  xSemaphoreGive(s->event);
}
bool configure_rate(const std::uint8_t *mac) {
  return gea::targets::esp32::espnow::configureRate(mac, state->rate);
}
} // namespace
extern "C" bool gea_espnow_open(int channel, int rate) {
  esp_now_rate_config_t requested{};
  if (channel < 0 || channel > 13 ||
      !gea::targets::esp32::espnow::rateConfig(rate, requested))
    return false;
  if (state && state->open.load())
    return state->channel == channel && state->rate == rate;
  if (!state) {
    auto *memory =
        heap_caps_malloc(sizeof(State), MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    if (!memory)
      memory = heap_caps_malloc(sizeof(State), MALLOC_CAP_8BIT);
    if (!memory)
      return false;
    state = new (memory) State;
    state->event = xSemaphoreCreateBinary();
    if (!state->event) {
      state->~State();
      heap_caps_free(state);
      state = nullptr;
      return false;
    }
  }
  auto *s = state;
  s->rate = rate;
  s->channel = channel;
  wifi_mode_t mode{};
  const bool existing = esp_wifi_get_mode(&mode) == ESP_OK;
  if (existing && channel != 0)
    return false; // Do not commandeer an AP/station owner.
  if (!existing) {
    if (!channel)
      return false;
    const auto loop = esp_event_loop_create_default();
    if (loop != ESP_OK && loop != ESP_ERR_INVALID_STATE)
      return false;
    wifi_init_config_t config = WIFI_INIT_CONFIG_DEFAULT();
    config.wifi_task_core_id = 0;
    gea::targets::esp32::wifi::applyTaskStackOverride(&config);
    if (esp_wifi_init(&config) != ESP_OK)
      return false;
    s->ownsRadio = true;
    if (esp_wifi_set_storage(WIFI_STORAGE_RAM) != ESP_OK ||
        esp_wifi_set_mode(WIFI_MODE_STA) != ESP_OK ||
        esp_wifi_start() != ESP_OK ||
        esp_wifi_set_channel(channel, WIFI_SECOND_CHAN_NONE) != ESP_OK) {
      gea_espnow_close();
      return false;
    }
  }
  if (!gea::targets::esp32::espnow::rateProtocolReady(rate)) {
    if (s->ownsRadio)
      gea_espnow_close();
    return false;
  }
  esp_wifi_get_ps(&s->previousPs);
  if (esp_wifi_set_ps(WIFI_PS_NONE) != ESP_OK || esp_now_init() != ESP_OK) {
    gea_espnow_close();
    return false;
  }
  std::uint32_t version = 0;
  if (esp_now_get_version(&version) != ESP_OK || version < 2 ||
      esp_now_register_recv_cb(received) != ESP_OK ||
      esp_now_register_send_cb(sent) != ESP_OK) {
    gea_espnow_close();
    return false;
  }
  s->read.store(0);
  s->write.store(0);
  s->busy.store(false);
  s->sent.store(0);
  s->failed.store(0);
  s->received.store(0);
  s->dropped.store(0);
  s->maxSend.store(0);
  s->completedStart.store(0);
  s->completedAt.store(0);
  s->completedRate.store(0);
  s->completedStatus.store(0);
  s->completedMacControl.store(0);
  s->completedMacSequence.store(0);
  s->captureTxHeader.store(false);
  s->open.store(true, std::memory_order_release);
  ESP_LOGI("espnow", "v%lu channel=%d rate=%d Mbps RX capacity=%u",
           (unsigned long)version, channel, rate, kQueue);
  return true;
}
extern "C" void gea_espnow_close() {
  auto *s = state;
  if (!s)
    return;
  s->captureTxHeader.store(false, std::memory_order_release);
  s->open.store(false, std::memory_order_release);
  esp_now_unregister_recv_cb();
  esp_now_unregister_send_cb();
  esp_now_deinit();
  // Keep the bounded storage for subsequent opens. A callback already running
  // on another core cannot encounter freed memory.
  if (s->ownsRadio) {
    esp_wifi_stop();
    esp_wifi_deinit();
    s->ownsRadio = false;
  } else
    esp_wifi_set_ps(s->previousPs);
  s->busy.store(false);
}
extern "C" bool gea_espnow_is_open() {
  return state && state->open.load(std::memory_order_acquire);
}
extern "C" bool gea_espnow_can_send() {
  return gea_espnow_is_open() && !state->busy.load(std::memory_order_acquire);
}
extern "C" bool gea_espnow_capture_tx_header(bool enabled) {
  if (enabled && (!gea_espnow_is_open() || !state->ownsRadio))
    return false;
  if (state)
    state->captureTxHeader.store(enabled, std::memory_order_release);
  return true;
}
extern "C" bool gea_espnow_set_max_tx_power(int quarterDbm) {
  return gea::targets::esp32::espnow::setMaxTxPower(
      gea_espnow_is_open(), state && state->ownsRadio, quarterDbm);
}
extern "C" int gea_espnow_get_max_tx_power() {
  return gea::targets::esp32::espnow::getMaxTxPower(gea_espnow_is_open());
}
extern "C" bool gea_espnow_peer(const std::uint8_t *mac,
                                const std::uint8_t *key) {
  if (!gea_espnow_is_open() || !mac)
    return false;
  esp_now_peer_info_t peer{};
  std::memcpy(peer.peer_addr, mac, 6);
  peer.ifidx = WIFI_IF_STA;
  if (key) {
    std::memcpy(peer.lmk, key, 16);
    peer.encrypt = true;
  }
  const auto result = esp_now_is_peer_exist(mac) ? esp_now_mod_peer(&peer)
                                                 : esp_now_add_peer(&peer);
  return result == ESP_OK && configure_rate(mac);
}
extern "C" int gea_espnow_send(const std::uint8_t *mac, const void *bytes,
                               std::size_t size) {
  if (!gea_espnow_is_open() || !mac || !bytes || !size || size > 1470)
    return -1;
  auto *s = state;
  bool idle = false;
  if (!s->busy.compare_exchange_strong(idle, true, std::memory_order_acq_rel))
    return 0;
  s->sentAt = esp_timer_get_time();
  const auto result =
      esp_now_send(mac, static_cast<const std::uint8_t *>(bytes), size);
  if (result != ESP_OK) {
    s->busy.store(false, std::memory_order_release);
    if (result == ESP_ERR_ESPNOW_NO_MEM)
      return 0;
    ++s->failed;
    return -1;
  }
  return static_cast<int>(size);
}
extern "C" bool gea_espnow_receive(gea::platform::espnow::Datagram *packet) {
  if (!gea_espnow_is_open() || !packet)
    return false;
  auto *s = state;
  const auto r = s->read.load(std::memory_order_relaxed);
  if (r == s->write.load(std::memory_order_acquire))
    return false;
  *packet = s->rx[r % kQueue];
  s->read.store(r + 1, std::memory_order_release);
  return true;
}
extern "C" void gea_espnow_wake() {
  if (state)
    xSemaphoreGive(state->event);
}
extern "C" void gea_espnow_wait(unsigned ms) {
  if (state)
    xSemaphoreTake(state->event, pdMS_TO_TICKS(ms));
}
extern "C" void gea_espnow_statistics(gea::platform::espnow::Statistics *out) {
  if (!out || !state)
    return;
  const auto macControl = state->completedMacControl.load();
  const auto macSequence = state->completedMacSequence.load();
  *out = {state->sent.load(),           state->failed.load(),
          state->received.load(),       state->dropped.load(),
          state->maxSend.load(),        state->completedStart.load(),
          state->completedAt.load(),    state->completedRate.load(),
          state->completedStatus.load(),
          static_cast<std::uint16_t>(macControl),
          static_cast<std::uint16_t>(macControl >> 16),
          static_cast<std::uint16_t>(macSequence),
          (macSequence & 0x10000U) != 0};
}
#endif
