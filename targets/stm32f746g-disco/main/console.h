#pragma once

#include <cstddef>
#include <cstdint>

// USART1 on the ST-LINK/V2-1 virtual COM port. Writes block until the bytes
// are in the shift register; reads drain a DMA ring the UART fills on its own,
// so nothing is lost between polls.
namespace gea::stm32::console {

void init(std::uint32_t baud);
void write(const char *data, std::size_t length);
std::size_t read(char *data, std::size_t capacity);

}  // namespace gea::stm32::console
