# STM32WBA65 MCU library

Target: STM32WBA65RIV7 (STM32WBA65I-DK1 Discovery Kit).

`lib/ioc` is the initial Debug/Release Cortex-M33 library configuration
(`STM32WBA65xx`). It uses the shared IOsonata directory structure. Flash
and SRAM size do not define a separate library variant; applications select
their linker script. A different peripheral inventory or interrupt vector
layout requires a separate library configuration.

## Integration status

The IOC library now includes IOsonata `ResetEntry.c`, the WBA65 interrupt
vector table, a shared HSI16 system initializer, and shared GPIO configuration. The vector table follows
STM32WBA65xx interrupt order. The system initializer installs the vector base
but does not configure PLLs, flash latency or TrustZone.

**The MCU port is not ready for application builds.** Outstanding work:

- Verify startup and vector placement with the installed STM32WBA65 CMSIS
  headers, hardware, and security configuration
- Implement live RCC clock-frequency decoding before changing from HSI16
- Verify EXTI interrupt routing/dispatch on hardware and implement UART/Timer drivers
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

The shared GPIO implementation now supports input, output, alternate functions,
pull resistors, output type, pin speed, and atomic BSRR set/clear. GPIO EXTI0–EXTI15 now route callbacks by pin number; interrupt allocation uses
the fixed EXTI line matching that pin, and a line cannot be assigned to two
ports simultaneously. This implementation has not been hardware-validated.
GPIO register writes and peripheral clocks have not been hardware tested.

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

## External middleware checkout

Clone the ST Bluetooth middleware independently of STM32CubeWBA:

```sh
cd ~/swdev/external
git clone https://github.com/STMicroelectronics/stm32-mw-wpan.git
```

The IOC project uses `external/stm32-mw-wpan/ble/` and
`external/stm32-mw-wpan/link_layer/` for the BLE middleware headers.
STM32CubeWBA remains the source for CMSIS, HAL, and `Utilities/tim_serv`.
Use a WPAN revision compatible with the selected STM32CubeWBA version;
ST documents component compatibility in STM32CubeWBA release notes.

`host_stack_if.h` and `ll_sys_if.h` belong to the CubeWBA
target-specific BLE application configuration, not the standalone
middleware root. Their integration, and linking the BLE binary libraries,
remain outstanding. BLE adapter sources are currently excluded from
the IOC configurations.

## GPIO consolidation

All STM32 IOC libraries now use `ARM/ST/src/iopincfg_stm32.cpp` for
GPIO register configuration, RCC port enabling, and EXTI callback dispatch.
No family `gpio_stm32*.c` translation units are linked. F0, F4, and L4 EXTI
paths select their ST CMSIS register definitions at compile time.

STM32WBA65 EXTI is implemented with the STM32CubeWBA CMSIS EXTI_TypeDef:
RTSR1/FTSR1 select edges, IMR1 masks lines, RPR1/FPR1 report and clear
pending edges, and EXTI->EXTICR[] selects the GPIO port. No inferred register
offsets or replacement peripheral structures are used. This path still
requires hardware regression testing on STM32WBA65I-DK1.
