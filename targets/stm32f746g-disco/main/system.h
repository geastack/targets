#pragma once

#include <cstdint>

namespace gea::stm32 {

// What SystemInit managed to bring up, kept in uninitialised DTCM so it
// survives the .data/.bss setup that runs after it.
struct BootInfo {
	bool clockReady;
	bool sdramReady;
	bool qspiReady;
	std::uint32_t qspiJedecId;
};

const BootInfo &bootInfo();

}  // namespace gea::stm32
