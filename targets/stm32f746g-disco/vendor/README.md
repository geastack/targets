# Vendored CMSIS

The files this target compiles from ST's STM32CubeF7, copied unchanged so a
build needs no Cube checkout. `cmsis/` mirrors the Cube's `Drivers/CMSIS`;
only what the build includes is here, and no HAL, BSP or middleware.

Source: [STM32CubeF7](https://github.com/STMicroelectronics/STM32CubeF7)
V1.17.4 (`package.xml`: FW.F7.1.17.0, patch FW.F7.1.17.4; commit `b609133`).

| Path | Upstream | Version | Licence |
|---|---|---|---|
| `cmsis/Core/Include/cmsis_compiler.h`, `cmsis_gcc.h`, `cmsis_version.h`, `core_cm7.h`, `mpu_armv7.h` | Arm CMSIS-Core(M), as shipped in STM32CubeF7 `Drivers/CMSIS/Core/Include` | CMSIS V5.4_CM7 (CMSIS-Core 5.1) | Apache-2.0, `cmsis/Core/LICENSE.txt` |
| `cmsis/Device/ST/STM32F7xx/Include/stm32f7xx.h`, `stm32f746xx.h`, `system_stm32f7xx.h` | [cmsis_device_f7](https://github.com/STMicroelectronics/cmsis_device_f7) | v1.2.10 (`2352e88`) | Apache-2.0, `cmsis/Device/ST/STM32F7xx/LICENSE.md` |
| `cmsis/Device/ST/STM32F7xx/Source/Templates/gcc/startup_stm32f746xx.s` | cmsis_device_f7 | v1.2.10 (`2352e88`) | Apache-2.0, `cmsis/Device/ST/STM32F7xx/LICENSE.md` |

STM32CubeF7 ships CMSIS-Core without a licence file (its headers carry
`SPDX-License-Identifier: Apache-2.0`), so `cmsis/Core/LICENSE.txt` is the
licence file of Arm's [CMSIS_5](https://github.com/ARM-software/CMSIS_5)
repository. `cmsis/Device/ST/STM32F7xx/LICENSE.md` is cmsis_device_f7's own.

Apache-2.0 code may be combined with this GPL-3.0 project; these files keep
their own licence and notices. Setting `STM32_CUBE_F7_PATH` to a Cube checkout
builds against its `Drivers/CMSIS` instead.
