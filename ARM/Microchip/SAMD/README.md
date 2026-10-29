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
- [ ] SERCOM UART, SPI and I2C drivers (D21 polling 8N1 UART source added; pad routing and hardware tests pending; enabled baud changes and RX error checks added; interrupts, DMA, SPI and I2C not implemented)
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

## SAM D21 SERCOM clocks

`Samd21SercomClockEnable(DevNo)` enables the APBC bus clock and the
SERCOM core GCLK channel for device 0..5, using generator 0. This is an
internal MCU helper; application source does not configure SERCOM clocks.
The helper is not a UART driver and has not been compiled or hardware-tested.
The SAM D11 peripheral clock mapping will be verified separately.

## SAM D21 UART bring-up

The initial `uart_samd21.cpp` is restricted to asynchronous 8N1 with polling
Rx/Tx, no DMA, no hardware flow control and no interrupt mode. It currently
uses USART RXPO=1 (PAD1) and TXPO=0 (PAD0); the application pin map must
select pins matching those SERCOM pad assignments. It is not a general
SERCOM pad-routing implementation. At 8 MHz, 16x oversampling limits the
maximum baud rate to 500000; the 1 MHz reset clock is insufficient for
common high baud rates. The driver still needs a target build and hardware
verification, including initialization failure paths and error flags.

## UART review status

The polling UART source now checks TXC before changing baud rate with the
USART enabled, waits for ENABLE synchronization, and checks received data
for framing, parity and buffer overflow errors. The device clock source and
register behavior have not been confirmed on hardware. Build, waveform and
UART PRBS tests remain required. No hardware results are claimed.

## CMSIS device selection

The managed projects select the exact Microchip device header through
`include/samdxxx.h`:

- SAMD11D14AM: `__SAMD11D14AM__` selects `samd11d14am.h`.
- SAMD21J18A: `__SAMD21J18A__` selects `samd21j18a.h`.

The SAMD11D14A name without the AM package suffix is not the device
identifier of the imported header. Drivers use the vendor CMSIS header
for register and interrupt definitions; no vendor HAL is linked.

## Xplained Pro Blinky hardware check

Managed Eclipse Embedded CDT applications:

- `SAMD11/exemples/Blinky/ioc` uses the shared `exemples/misc/blinky.c`.
  LED0 is PA16 (active low).
- `SAMD21/exemples/Blinky/ioc` uses the same application source.
  LED0 is PB30 (active low).

Import the matching `lib/ioc` project first, then the Blinky project.
Build the library and Blinky in the same configuration (Debug or Release).
The Blinky application uses the internal OSC8M clock at 8 MHz; no
external oscillator is required for this check.

Both boards include an EDBG CMSIS-DAP probe. With the Xplained Pro board
connected to the EDBG USB port, program the ELF output with OpenOCD.
The command below runs from the Blinky `ioc` folder after a Debug build:

```sh
openocd -f interface/cmsis-dap.cfg -f target/at91samdXX.cfg \
    -c "adapter speed 400; program Debug/Blinky_<PART>.elf verify reset exit"
```

Replace `<PART>` with `SAMD11D14AM` or `SAMD21J18A`.
Use the actual artifact filename shown by Eclipse if it differs.
The expected first result is a blinking yellow LED0. There is no serial
output in this GPIO smoke test.

These application projects have not yet been compiled or run on either board.
UART and hardware timer examples still require driver completion.
