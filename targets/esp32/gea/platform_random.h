#pragma once

#include <cstdint>

#include "esp_random.h"

inline std::uint64_t gea_runtime_platform_random_u64()
{
  return (static_cast<std::uint64_t>(esp_random()) << 32) |
         static_cast<std::uint64_t>(esp_random());
}
