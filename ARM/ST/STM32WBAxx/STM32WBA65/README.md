# STM32WBA65 MCU library

Target: STM32WBA65RIV7 (STM32WBA65I-DK1 Discovery Kit).

`lib/ioc` is the initial Debug/Release Cortex-M33 library configuration
(`STM32WBA65xx`). It uses the shared IOsonata directory structure. Flash
and SRAM size do not define a separate library variant; applications select
their linker script. A different peripheral inventory or interrupt vector
layout requires a separate library configuration.

## Integration status

The project currently links generic IOsonata support sources and shared
STM32WBA headers. **It is not a complete buildable MCU library.** The
following target-specific components are not yet linked:

- ST CMSIS `stm32wba65xx.h` and WBA65 system/startup dependencies
- STM32WBA65RIV7 interrupt-vector source
- IOsonata WBA6 startup/system-clock and interrupt dispatch
- IOsonata WBA6 GPIO, UART and Timer drivers
- Verified vendor startup and MCU-specific system/interrupt integration

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
