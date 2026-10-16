#pragma once
#include <cstdint>
constexpr int ESP_OK = 0;
int esp_wifi_set_max_tx_power(std::int8_t power);
int esp_wifi_get_max_tx_power(std::int8_t *power);
