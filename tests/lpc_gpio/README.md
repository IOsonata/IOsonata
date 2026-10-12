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
no pull, not a disconnected input buffer. Drive strength and speed setters are
unsupported no-ops.

Pin interrupts use eight channels (IntNo 0..7), connected to the existing
FLEX_INT0_IRQHandler through FLEX_INT7_IRQHandler vectors. Configure the pin as
a digital input first. Explicit registration rejects an occupied channel, invalid
pins, NULL callbacks, unsupported sense values and priorities outside 0..3.
Automatic allocation returns the first free channel or -1. Callbacks receive the
channel number and caller context in interrupt context.

IOPinSetSense supports rising, falling and both edges. IOPINSENSE_DISABLE pauses
the registered pin without releasing its channel; another sense value rearms it.
Changing sense clears stale edge and NVIC pending status. IOPinDisableInterrupt
masks detection, clears pending status and releases the channel. The ISR clears
only its channel before the callback, so an edge during the callback is retained.
Allocation and hook updates use short interrupt locks; callbacks run after the
lock is restored. Group/port interrupts and deep-sleep wake configuration are not
implemented by these functions.

PINTSEL uses selectors 0..23 for port 0 and 24..55 for port 1. The old bundled
CMSIS header incorrectly declared three NVIC priority bits; it now declares two,
as specified by UM10462 section 6.3. Use priorities 0..3 for LPC11Uxx. The Blinky
button definitions were updated accordingly.

The interrupt tests check all eight vector names, channel exhaustion/reuse,
occupied-channel rejection, port boundaries, priority bounds, edge enables,
pause/resume, callback context, release from a callback and a new edge during a
callback. A host NVIC model applies register set/clear effects at NVIC calls and
callbacks; it does not simulate asynchronous hardware or interrupt latency.

GPIO, SSP, UART and Cortex-M0 compiler-to-assembly checks pass. Firmware linking,
Blinky output, UART PRBS output, SPI chip-select timing and ADC/I2C electrical
behavior still require a target toolchain and board validation.

Hardware reference: NXP UM10462, chapters 7 (IOCON) and 9 (GPIO),
https://www.keil.com/dd/docs/datashts/nxp/lpc11uxx/um10462.pdf.
