# STM32WBA65 MCU library

Target: STM32WBA65RIV7 (STM32WBA65I-DK1 Discovery Kit).

`lib/ioc` is the initial Debug/Release Cortex-M33 library configuration
(`STM32WBA65xx`). It uses the shared IOsonata directory structure. Flash
and SRAM size do not define a separate library variant; applications select
their linker script. A different peripheral inventory or interrupt vector
layout requires a separate library configuration.

## Integration status

The IOC library now includes IOsonata `ResetEntry.c`, the WBA65 interrupt
vector table, and a shared HSI16 system initializer. The vector table follows
STM32WBA65xx interrupt order. The system initializer installs the vector base
but does not configure PLLs, flash latency or TrustZone.

**The MCU port is not ready for application builds.** Outstanding work:

- Verify startup and vector placement with the installed STM32WBA65 CMSIS
  headers, hardware, and security configuration
- Implement live RCC clock-frequency decoding before changing from HSI16
- Implement GPIO, interrupt dispatch, UART and Timer drivers
- Validate the application linker script against STM32WBA65 flash/SRAM banks
  and the configured secure/nonsecure partitions
- Integrate and test the STM32CubeWBA radio middleware

Use the WBA65 device definitions and interrupt mapping from a consistent
STM32CubeWBA release. Do not compile WBA5x startup or link with the WBA5x
1 MB/128 KB linker script. The STM32WBA Bluetooth adapter in
`../src/` additionally requires CubeWBA middleware; it is not yet
included in this library configuration.

The application linker layouts are now provided in `ldscript/`:

- `gcc_stm32wba65_i.ld`: 2 MB flash, 512 KB SRAM (STM32WBA65RIV7).
- `gcc_stm32wba65_g.ld`: 1 MB flash, 256 KB SRAM.

Each reserves two 16 KB flash NVM partitions at the upper end of its
variant's flash. The memory boundaries require verification against the
chosen ST secure/nonsecure image layout, option bytes, and radio firmware
reservation before flashing. These linker scripts are not evidence of a
successful boot or a working BLE stack.

The initial bring-up sequence is startup, GPIO, UART, Timer, then BLE.
Discovery Kit GPIO/USART wiring belongs in each example's `board.h`,
not in the library.

References: STM32WBA6 reference manual RM0515 and STM32CubeWBA:
https://github.com/STMicroelectronics/STM32CubeWBA

## Build dependency

The IOC build configuration expects the official CMSIS headers in a sibling
`external/STM32CubeWBA/Drivers/CMSIS/` checkout, including
`Device/ST/STM32WBAxx/Include/stm32wba65xx.h`. The project does not
vendor this device header. The shared initial system module runs from the
16 MHz reset HSI clock; clock configuration beyond that is not yet supported.

The existing `gcc_arm_flash.ld` calls IOsonata `ResetEntry` and places
`__Vectors` in `.ivector`. Do not link the STM32CubeWBA GCC startup module
in the same image or two startup/vector definitions will conflict.
