# LPC GPIO checks and pin-map migration

Run `python3 tests/lpc/gpio/run.py` on Linux with Python 3, gcc and g++.
The LPC17xx control test checks byte, halfword and word port writes against
the vendor GPIO register declarations on all five ports. Halfword writes retain
all 16 supplied bits and preserve the upper 16 pins. P1.31 checks exercise
unsigned masks in direction, set, clear and toggle helpers. These tests run as
C and C++ at O0 and O2 with UBSan; mapped memory does not model FIOSET/FIOCLR
side effects, FIOMASK, pin muxing or electrical behavior.

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
LPC17xx now uses the same generic pin-function constants: GPIO and FUNC0
select hardware function 0, and FUNC1 through FUNC3 select functions 1 through
3. External LPC17xx maps using raw mux values must migrate to IOPINOP_FUNCn.
The LPC1769 DfuBoot UART map and MBEDPRESSO PWM definitions are migrated.
LPC17xx configuration retains volatile access through named vendor PINCON
registers and protects register read/modify/write operations with an interrupt
lock. Invalid port, pin-index and function ranges leave registers unchanged;
callers must still choose pins and alternate functions available on their part.
Tests cover register-bank boundaries, P1.31, all four mux and pull encodings,
open drain, direction, DfuBoot UART pins and the three I2C mux selections.

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
pins, NULL callbacks and unsupported sense values. Priorities select the closest
level in 0..3; negative values become 0 and values above 3 become 3.
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

PINTSEL uses selectors 0..23 for port 0 and 24..55 for port 1, except
reserved P1.30 (selector 54). The old bundled
CMSIS header incorrectly declared three NVIC priority bits; it now declares two,
as specified by UM10462 section 6.3. Use priorities 0..3 for LPC11Uxx. The Blinky
button definitions were updated accordingly.

The interrupt tests check all eight vector names, channel exhaustion/reuse,
occupied-channel rejection, port boundaries, priority clamping (including INT_MIN/INT_MAX), edge enables,
pause/resume, callback context, release from a callback and a new edge during a
callback. A host NVIC model applies register set/clear effects at NVIC calls and
callbacks; it does not simulate asynchronous hardware or interrupt latency.

GPIO, SSP, UART and Cortex-M0 compiler-to-assembly checks pass. Firmware linking,
Blinky output, UART PRBS output, SPI chip-select timing and ADC/I2C electrical
behavior still require a target toolchain and board validation.

P1.30 is rejected by GPIO configuration, per-pin control and pin-interrupt
registration. Its IOCON offset 0xD8 is reserved in UM10462 table 60, despite
having a PIO1_30 member in the bundled vendor header. The register lookup keeps
a NULL entry at that offset, preserving P1.31's address. Tests check that the
reserved slot is untouched, failed interrupt requests consume no channel, and
SSP initialization rejects P1.30 as a chip select.

Hardware reference: NXP UM10462, chapters 7 (IOCON) and 9 (GPIO),
https://www.keil.com/dd/docs/datashts/nxp/lpc11uxx/um10462.pdf.

LPC17xx IOPinSetSense configures rising, falling, both or neither edge on
P0.0-11/P0.15-30 and P2.0-13 through vendor GPIOINT registers. Package pin
availability remains the caller's responsibility. Reserved pins, other ports
and invalid sense values are ignored. Updates preserve unrelated enable bits
and pending events under the existing saved-interrupt-state helper. The caller
configures pin mux/direction separately. This API does not allocate callbacks
or enable NVIC; callback registration manages the shared interrupt.
C/C++ tests cover every pin and mode, invalid inputs and full register-block
preservation at O0/O2 with UBSan.

LPC17xx GPIO callback registration uses fixed IntNo values: P0.n uses n, and
P2.n uses 32+n. All 42 supported pins can be registered independently. Explicit
IOPinEnableInterrupt requires the matching IntNo; allocation returns it. A
second registration for the same pin fails until disabled. SetSense(DISABLE)
pauses detection while retaining the registration. Pin mux/direction must be
configured separately, and package pin availability remains the caller's job.

EINT3 is shared by all GPIO callbacks. The driver selects the most urgent
registered priority, clamped to 0..31, and disables NVIC only after the last
registration is released. It owns the EINT3 vector: the separate EINT3 pin
function and another EINT3 handler must not be enabled alongside this driver.
Pending GPIO status is cleared before calling a handler. Both edges on one pin
produce one callback per visit; a new edge during that callback remains pending.
Each next pin is checked live so callbacks may disable or reallocate other pins.
Latched status for disabled or unregistered pins is drained without callbacks.

Host tests cover both ports, simultaneous edges, priority clamping and shared
priority changes, registration collisions, every supported slot, release, pause,
callback cancellation and edges arriving during callbacks. The mmap fixture
emulates W1C at callback/NVIC boundaries; it does not model electrical timing
or interrupt preemption. C/C++ builds exercise the EINT3 symbol at O0/O2.

LPC17xx IOPinDisable returns a pin to GPIO input, no pull resistor and normal
(non-open-drain) mode where supported, following the LPC11 cleanup pattern. It does not promise
to disconnect the input buffer or release an interrupt registration. Callers
stop peripheral activity and release callbacks separately. Reconfiguration
restores the desired pin function. Tests cover low/high pin halves, all ports,
neighbor preservation, invalid indices and the generic IOPinDis batch helper.
The previous port fails linking these tests because IOPinDisable is missing.

LPC17xx direction changes and toggles save/restore interrupt state around
read-modify-write decisions. Toggle writes only the selected FIOSET/FIOCLR bit,
leaving unrelated output latches alone. Invalid port/pin indices do not access
registers; SetDir retains its no-change behavior for directions other than
INPUT/OUTPUT. Full-port writes remain explicit full-port operations.
The inline-control test asserts masked register access and exact mask restoration
for all pin indices on all five ports, both initial interrupt states and both
toggle levels. It compares the complete vendor GPIO blocks to detect unwanted
writes. It models critical-section calls, not asynchronous hardware timing.

All LPC17xx inline GPIO reads and writes validate port indices (0..4), and
single-pin operations validate pin indices (0..31) before shifting. Invalid
reads return zero and invalid writes leave registers untouched, matching the
LPC11 convention. These bounds describe register slots, not package pin
availability. Tests cover negative indices, upper boundaries, INT_MIN/INT_MAX,
all valid pin indices and existing byte/halfword/full-port write behavior.

IOPinSetStrength and IOPinSetSpeed are intentional no-ops on LPC17xx, as on
LPC11. There are no general per-pin drive-strength or slew-rate settings.
These functions supply generic API linkage without claiming hardware support
or changing the specialized I2C0 pad controls. The C/C++ fixture calls both
functions and verifies PINCON, including I2CPADCFG, remains unchanged.

LPC17xx P0.27/SDA0 and P0.28/SCL0 are permanently open drain with no internal
pull resistors; outputs require external pull-ups. Configuration skips their
reserved PINMODE/PINMODE_OD fields. GPIO/FUNC0 disables the I2C filter and slew
control through the corresponding I2CPADCFG bit; FUNC1 (I2C0) and FUNC2 (USB OTG
I2C) enable them. Drive settings remain owned by the I2C driver, and configuring
one pin preserves the companion pin. IOPinDisable restores GPIO input and GPIO
pad mode. Tests cover both pins, all 16 initial pad-control combinations, both
output types, every pull request, GPIO/I2C transitions and complete PINCON
preservation outside the selected mux/mode fields. Electrical behavior remains
subject to board testing. See NXP UM10360, PINMODE1, PINMODE_OD0 and I2CPADCFG.

LPC17xx USB pads P0.29/P0.30 have no configurable pull resistors. All pull
requests, including cleanup through IOPinDisable, leave their reserved
PINMODE1 fields unchanged; their valid open-drain controls still work.
Tests cover both pins, all pull requests, normal/open-drain transitions and
complete PINCON preservation outside the selected mux/open-drain fields.
These pads also share their physical GPIO direction: both FIO0DIR bits must
be set for outputs; clearing either makes both inputs. Callers must configure
the pair consistently. Per-pin APIs retain individual direction-bit writes
and do not silently configure the companion pin. See NXP UM10360 table 88
and section 9.5.1.
