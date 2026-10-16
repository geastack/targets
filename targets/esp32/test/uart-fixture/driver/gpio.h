#pragma once
using gpio_num_t = int;
constexpr int GPIO_NUM_MAX = 64;
#define GPIO_IS_VALID_GPIO(pin) ((pin) >= 0 && (pin) < 64)
#define GPIO_IS_VALID_OUTPUT_GPIO(pin) GPIO_IS_VALID_GPIO(pin)
int gpio_reset_pin(gpio_num_t pin);
