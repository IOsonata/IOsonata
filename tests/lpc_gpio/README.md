# LPC11Uxx GPIO checks and pin-map migration

Run `python3 tests/lpc_gpio/run.py` on Linux with Python 3, gcc and g++.
The real driver is compiled as C and C++ at O0 and O2 with UBSan. Tests map
peripheral memory using the vendor CMSIS declarations and the UART tests' core
shim. They check GPIO/IOCON clock enable, UART board muxing, all IOCON addresses,
reset/debug GPIO selection, FUNC4, ADC/digital transitions, I2C reserved bits,
pulls, open drain, port writes, pin 31 and invalid inputs. The register checks
do not emulate pad electrical behavior, clock gating or interrupt timing.

LPC11Uxx now follows the generic constants in coredev/iopincfg.h, as LPC546xx,
STM32 and Microchip do. No generic API was changed. External LPC11Uxx board maps
using raw mux numbers need migration: use IOPINOP_GPIO for GPIO and
IOPINOP_FUNCn for hardware FUNC n. For example, UART function 1 uses
IOPINOP_FUNC1 (numeric value 2). GPIO on P0_0 and P0_10..15 automatically selects
hardware function 1; IOPINOP_FUNC0 still selects hardware function 0. Reassigning
reset/debug pins removes their reset/debug function. The repository's LPC11U35
Blinky, DFU and UART examples have been updated.

Configuration uses pointers to named vendor IOCON registers. LPC17xx PINCON
and LPC546xx IOCON differ, so these GPIO registers are not combined into one
shared driver. Shared legacy UART and SSP remain separate from pin setup.
LPC17xx GPIO is still an older implementation using raw mux values.

ADC functions select analog mode without digital input, pulls, hysteresis or
open drain; configuring GPIO or a digital alternate function restores digital
input. P0_4/5 are always open drain and need external pull-ups where a high level
is required. Their pull and output-type requests cannot change the pad hardware;
GPIO selects standard I/O mode and I2C selects standard/fast mode. Function
numbers must be valid for the chosen pin/package; this driver bounds the
three-bit field but does not validate each pin's alternate-function table.

Full port writes use PIN, not the per-pin B/W aliases. Narrow writes change the
low 8/16 output bits with separate CLR/SET writes, preserving upper pins; the
two writes are not a simultaneous port update. Fast control helpers require a
previously configured/enabled GPIO clock. IOPinDisable leaves a GPIO input with
no pull, not a disconnected input buffer. Pin interrupts remain unimplemented
and registration returns failure. Drive strength, speed and sense setters are
unsupported no-ops.

GPIO, SSP, UART and Cortex-M0 compiler-to-assembly checks pass. Firmware linking,
Blinky output, UART PRBS output, SPI chip-select timing and ADC/I2C electrical
behavior still require a target toolchain and board validation.

Hardware reference: NXP UM10462, chapters 7 (IOCON) and 9 (GPIO),
https://www.keil.com/dd/docs/datashts/nxp/lpc11uxx/um10462.pdf.
