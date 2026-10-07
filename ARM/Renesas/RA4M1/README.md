# RA4M1 startup foundation

Initial native MCU port under the existing `ARM/Renesas` tree. This change adds
system startup, clock reporting, Cortex-M4 vectors, and the normal-boot linker
layout. It does not add a board package, Arduino compatibility, FSP runtime,
peripheral drivers, or a USB controller implementation.

## IOsonata integration

The pattern is `ARM/Renesas/RE01`: an MCU include wrapper, a weak `g_McuOsc`,
`system_*.c`, `vectors_*.c`, `.vectors`, `__Vectors`, `__StackTop`, and
`IEL0_IRQHandler` through `IEL31_IRQHandler`.

### IOC library project

Import `ARM/Renesas/RA4M1/lib/ioc` with **Existing Projects into Workspace**.
The project is named `IOsonata_RA4M1` and provides `Debug` (`-O0`, `-g3`) and
`Release` (`-Os`) configurations. Both use Cortex-M4 Thumb, `fpv4-sp-d16`, and
hard-float ABI, with the normal Arm Cross GCC managed-build tools. Language
settings follow the Renesas library template: GNU C17 and GNU C++23, with C++
RTTI and exceptions disabled. Configure
the compiler installation through IOC's existing toolchain settings, not an
absolute path embedded in the project.

The outputs are `lib/ioc/Debug/libIOsonata_RA4M1.a` and
`lib/ioc/Release/libIOsonata_RA4M1.a`. The project links the two target startup
sources and the existing `ARM/src/ResetEntry.c`; it does not copy them. It uses
the production target/generic headers and `ARM/CMSIS/Core/Include`, never the
reduced headers in `tests/ra4m1`. CMSIS is taken from the existing repository
files; no new vendor SDK or submodule is required.

This is the library project for the **startup-only port increment**. It does
not yet compile peripheral drivers or the generic USB/application subsystem.
Future driver increments must add their sources to this same project. The
library does not link an executable or produce a flash image; the application
links it with `ldscript/gcc_ra4m1.ld` and the matching hard-float runtime. Do not
also compile the same startup sources into the application. The accompanying
`wizard.json` supplies the MCU, library and linker metadata using the existing
IOC wizard format; it does not add a board configuration.

### Source integration

Build these sources as C11 or later:

- `ARM/src/ResetEntry.c` (existing file, unchanged)
- `ARM/Renesas/RA4M1/src/system_ra4m1.c`
- `ARM/Renesas/RA4M1/src/vectors_ra4m1.c`

Use the RA4M1 `include` directory, IOsonata `include` and `ARM/include`, and the
project's existing CMSIS Core include directory. Link with
`ARM/Renesas/RA4M1/ldscript/gcc_ra4m1.ld`. Use the usual IOsonata/newlib runtime
and library settings; this linker script does not hard-code a C library group.
CPU flags are `-mcpu=cortex-m4 -mthumb`. When using hardware floating point,
use `-mfpu=fpv4-sp-d16` and the same `-mfloat-abi` in the application and every
linked library. Compile the vector object directly, or retain it from the
library through the existing ResetEntry reference to `__Vectors`.

The wrapper `ra4m1xxx.h` supplies the CMSIS core configuration and the 32 ICU
slot IRQ numbers. `src/ra4m1_startup_regs.h` is explicitly a **private startup
register subset**, not a fabricated complete Renesas device header. It keeps
this increment independent of FSP configuration headers. A complete peripheral
register header and the RA4M1 interrupt manager remain work for subsequent port
increments. Do not include both this core wrapper and an unrelated complete
vendor device header in the same translation unit.

The shared ResetEntry copies `.data` (including `.fastrun`), clears `.bss`,
calls SystemInit, updates the clock information, and enters its existing runtime
path. The port does not duplicate C runtime startup or change constructor/RTOS
entry behavior. SystemInit installs VTOR, enables CP10/CP11, leaves SysTick
untouched, disables/clears all 32 NVIC external interrupts, and clears their
IELSR routes. Peripheral/event routing is intentionally not fixed at startup.

## Default clocks

The weak oscillator descriptor selects internal HOCO at 48 MHz, internal LOCO
at 32.768 kHz, and `bUSBClk=false`. No crystal, GPIO map, LED, UART, or particular
development platform is assumed. Accuracy fields are zero (unspecified), not
an invented PPM rating. An application's statically initialized strong
`g_McuOsc` overrides the default, as on other IOsonata targets.

Expected default successful-startup state:

| Item | Value |
| --- | --- |
| `SystemCoreClock` / ICLK | 48,000,000 Hz |
| PCLKA / PCLKC / PCLKD | 48,000,000 Hz |
| PCLKB / FCLK | 24,000,000 Hz |
| `SCKSCR` | `0x00` (HOCO) |
| `HOCOCR2` | `0x20` (48 MHz) |
| `SCKDIVCR` | `0x10010100` |
| `OPCCR.OPCM` | `0` (High-speed mode) |
| `SOPCCR.SOPCM` | `0` |
| `MEMWAIT` | `1` |
| `FCACHEE` | `1`, after completed invalidation |
| `IELSR[0..31]` | `0` (unassigned) |

**The default 48 MHz operating point requires VCC of 2.7–5.5 V.** This initial
startup deliberately uses High-speed mode, not automatic supply-voltage
measurement or low-voltage power optimization. The hardware's wider supply
range does not permit 48 MHz across that entire range.

`SystemPeriphClockGet` indices are 0=PCLKA, 1=PCLKB, 2=PCLKC, 3=PCLKD.
`SystemFlashClockGet` reports FCLK. Queries decode the live source and divider
registers and never change Flash timing. External input frequency is necessarily
kept in software; software cannot discover a crystal's frequency from a selector.
The oscillator-stop fallback is accounted for: MOSC redirected to MOCO reports
MOCO, and a free-running PLL reports zero (unknown frequency).

## Other oscillator configurations

Core RC descriptors accept 8 MHz (MOCO), and 24, 32, or 48 MHz (HOCO).
A 64 MHz CPU request is rejected; the decoder can nevertheless identify the
64 MHz HOCO encoding if it reads an externally established configuration.

Main crystal/external-clock descriptors accept 1–20 MHz. Startup uses an exact
48 MHz PLL solution when the input is 4–12.5 MHz and a legal integer multiplier
and output divider exist. Otherwise it uses the main oscillator directly.
Examples: 4, 6, 8, and 12 MHz can supply the 48 MHz PLL plan; 16 MHz cannot use
this PLL input range and is therefore used directly. PLLCCR2 is an **8-bit**
register at `0x4001E02B`; the implementation does not copy RE01 PLL encodings.

`bUSBClk=true` requires a 48 MHz source and sets only the USB clock selector.
HOCO is a device-only USB source and requires zero HOCO user trim; PLL can
supply the clock for host or device. USBFS remains stopped/disabled. This is
clock preparation only: no claim of a working RA4M1 USB port is made.

Low-frequency descriptors accept internal RC at 32768 Hz or a 32768 Hz crystal.
The selected oscillator is started; RTC/AGT/LCD drivers own their individual
source-selection registers. A running SOSC and retained RTC are not reset.
A low-frequency TCXO or a 32000 Hz descriptor is rejected by this initial port.

External resonator settling remains a hardware requirement. `RA4M1_MOSC_WAIT`
defaults to code 9: 262144 **MOCO** cycles, nominally 32.768 ms. It is a masking
time, not a measurement proving oscillation. `RA4M1_SOSC_DRIVE` defaults to 0
(normal drive), and `RA4M1_SOSC_STARTUP_US` defaults to 2,000,000. The software
wait is conservative and can be appreciably longer; it is not a calibrated
timer and cannot detect a missing SOSC crystal. Validate settling and loading
with the actual resonator. No load-capacitance value is inferred from a board.

## Reconfiguration and failure handling

`SystemInit` is a cold-start entry, not a runtime reinitialization API.
`SystemCoreClockSelect` performs a checked switch through MOCO /16. Before
calling it at runtime the application must quiesce **all** clock-dependent
clients: peripheral transfers, DMA/DTC, timers/SysTick, trace/clock outputs,
RTOS timing, and any other oscillator consumers. USBFS SCKE must be zero.
The function masks CPU interrupts but cannot automatically quiesce DMA or an
application. It refuses Subosc-mode/LOCO/SOSC system-clock entry and refuses an
enabled or latched oscillator-stop detector. Low-frequency selection likewise
requires quiesced consumers and masks interrupts during its startup wait.
Independent bus-divider changes and low-power transition policy are deferred;
`SystemPeriphClockSet` succeeds only for the already-established frequency.

Protected writes preserve unrelated PRCR bits and restore the prior protection
and PRIMASK. Frequency and mode changes use readback/ready checks. Oscillator,
mode, or register-readback failure does not report the requested rate as if it
had been established. A failed runtime switch is **not transactional rollback**:
it can leave the CPU on MOCO /16 or on the requested clock with cache disabled.
Check the return value, `g_Ra4m1StartupError`, and live clock queries before
resuming clients. Cold-start failures stop before entering the application.
The enum in `system_ra4m1.h` names the debugger-visible failure stages.

The existing generic `nsDelay` factor starts at an uncalibrated value of 27;
`RA4M1_NSDELAY_FACTOR` is build-time overridable. Startup never uses that API.
The shared generic delay loops, low-frequency operation, and delay accuracy
still require target measurement; no timing-accuracy claim is made here.

## Boot options and linker layout

The normal-boot image contains OFS0/OFS1 at `0x400/0x404` and reserves the entire
option area through `0x43B`. Ordinary code starts after it, at or beyond `0x440`.
OFS0 is `0xFFFFFFFF`: IWDT does not auto-start, and WDT uses register-start mode.
OFS1 is `0xFFFF8EFF`: HOCO auto-starts at 24 MHz while reset execution still uses
MOCO /16. This keeps HOCO running in reset's Low-voltage mode. SystemInit waits
for it to stabilize, enters High-speed mode, and then changes its frequency.
LVD0 remains disabled by these initial option values; production voltage and
watchdog policy must be selected deliberately for the final product.
The security-MPU region is disabled. Debug-ID/access-window configuration at
`0x01010000` is not emitted or altered. Boot-swap/bootloader layouts are not
provided in this increment.

The linker reserves 2 KiB stack and 1 KiB heap by default. Override with
`--defsym=__STACK_SIZE=<bytes>` and `--defsym=__HEAP_SIZE=<bytes>` as appropriate.
Link assertions reject missing/duplicate options, the wrong vector-table size,
flash overflow, and data/heap/stack overlap. No common IOsonata linker is changed.

## Validation status

Run `python3 tests/ra4m1/run_validation.py` from the repository root.
The tests use reduced CMSIS/generic-contract **test shims**, clearly separated
from production includes. They run the actual startup implementation against a
register model and compile/link an ARM layout probe. They are not an FSP build,
a full IOsonata/newlib firmware build, a CPU emulator, or hardware validation.

The first physical check is cold boot and debugger entry to the application,
then the default register values above and a measured CPU/timer clock. Repeat
with software and external reset. No peripheral driver or evaluation-board
project is included in this starting increment.

## Sources

IOsonata patterns were read from `sam4l_usb_debug` commit
`721b361401b1db64ce7ccd2ebc532f88bbfb48b8`: the RE01 startup/vectors, shared
ResetEntry, generic clock/interrupt contracts, and common ARM linker script.

Hardware basis: Renesas RA4M1 Group User's Manual: Hardware,
**R01UH0887EJ0110, Rev. 1.10, September 29, 2023**. Relevant sections are chapter
6 (option-setting memory), chapter 8 (clocks), chapter 10 (operating modes),
chapter 12 (write protection), chapter 13 (ICU), chapter 44 (Flash/cache), and
chapter 48 (electrical limits). Register facts were checked against the manual;
Renesas FSP `v6.6.0` RA4M1 feature definitions were a secondary cross-check.
