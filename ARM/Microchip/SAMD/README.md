# SAM D11/D21 MCU port

Applications supply pin and oscillator configuration for these MCU targets.

## Targets

| MCU | Evaluation hardware | Flash | RAM | CPU |
| --- | --- | ---: | ---: | --- |
| ATSAMD21J18A | SAM D21 Xplained Pro | 256 KiB | 32 KiB | Cortex-M0+ |
| ATSAMD11D14A | SAM D11 Xplained Pro | 16 KiB | 4 KiB | Cortex-M0+ |

The linker scripts reuse `ARM/ldscript/gcc_arm_flash.ld`.
The linker search path must contain `ARM/ldscript`. Do not use
the SAMD21J18A linker script for other SAMD21 package/memory variants.

## Port limits

This port is incomplete. The supplied startup, GPIO and clock sources and
SAMD21 polling UART do not establish a hardware-validated port. Hardware
timers, SPI, I2C and USB device support are not implemented in this revision.

The clock source is OSC8M: reset-time division gives 1 MHz; applications can
select 8 MHz with `SystemCoreClockSelect(OSC_TYPE_RC, 8000000)`. External
oscillators and the 48 MHz DFLL path are not integrated. The SAMD21 UART is
restricted to polling 8N1, RX on PAD1 and TX on PAD0, without flow control,
interrupt or DMA operation. At 8 MHz, 16x oversampling limits its baud rate
to 500000. Select pins matching those pad assignments in the application.

## CMSIS device selection

The managed projects select the exact Microchip device header through
`include/samdxxx.h`:

- SAMD11D14AM: `__SAMD11D14AM__` selects `samd11d14am.h`.
- SAMD21J18A: `__SAMD21J18A__` selects `samd21j18a.h`.

The SAMD11D14A name without the AM package suffix is not the device
identifier of the imported header. Drivers use the vendor CMSIS header
for register and interrupt definitions; no vendor HAL is linked.

## Build and run Blinky

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
    -c "adapter speed 400; program Debug/Blinky.elf verify reset exit"
```

The Eclipse project and artifact are named `Blinky` on both targets, matching the existing IOsonata example convention.
The expected first result is a blinking yellow LED0. There is no serial
output in this GPIO smoke test.

These application projects have not yet been compiled or run on either board.
UART and hardware timer examples still require driver completion.
