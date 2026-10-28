# SAM D11/D21 MCU port

Work in progress. This is an **MCU port**, not an Xplained Pro board
support package. Applications retain control of pin and oscillator
configuration.

## Targets

| MCU | Evaluation hardware | Flash | RAM | CPU |
| --- | --- | ---: | ---: | --- |
| ATSAMD21J18A | SAM D21 Xplained Pro | 256 KiB | 32 KiB | Cortex-M0+ |
| ATSAMD11D14A | SAM D11 Xplained Pro | 16 KiB | 4 KiB | Cortex-M0+ |

The linker scripts reuse `ARM/ldscript/gcc_arm_flash.ld`.
The linker search path must contain `ARM/ldscript`. Do not use
the SAMD21J18A linker script for other SAMD21 package/memory variants.

## Implementation status

- [x] MCU-specific flash/RAM linker memory maps
- [ ] Microchip device CMSIS headers and core/part definitions
- [x] Cortex-M0+ IRQ vector tables using IOsonata `ResetEntry` (source added; build and device validation pending)
- [ ] Clock source and GCLK setup, synchronization and clock reporting (reset-time 1 MHz clock plus selectable 8 MHz OSC8M/GCLK0 path added; channel enable/disable added; peripheral bus gates and 48 MHz setup pending)
- [ ] PORT GPIO, mux and external interrupts (basic GPIO/mux source added; interrupt API, device definitions and validation pending)
- [ ] SERCOM UART, SPI and I2C drivers
- [ ] TC/TCC hardware timers
- [ ] Embedded CDT managed static-library projects and generic examples
- [ ] Build and Xplained Pro hardware validation
- [ ] USB device support (separate milestone)

## Implementation rules

Follow the existing IOsonata interfaces and coding style; no independent
ASF/START/Harmony application architecture, new board abstraction, or
board-specific source files. Shared SAM D drivers belong in this family
directory when hardware behavior actually matches. MCU-specific definitions
and pin capabilities must remain distinct.

A minimal working port must pass the existing generic Blinky, UART PRBS and
timer examples on physical hardware before it is described as supported.

## Current clock limitation

The initial `system_samd.c` retains the reset-time OSC8M/8 configuration
(1 MHz CPU clock) unless the application selects OSC8M at 8 MHz using
`SystemCoreClockSelect(OSC_TYPE_RC, 8000000)`. The internal SamdGclkChannelEnable/Disable functions now select peripheral
GCLK channels, but peripheral bus clock gates, device-specific channel IDs,
and the 48 MHz clock source must still be integrated with the drivers. External oscillator selection, 48 MHz DFLL configuration and peripheral
clock routing is not yet integrated with SERCOM or timer drivers. Do not use this stage to validate
UART baud rate, high-speed timing or USB.
