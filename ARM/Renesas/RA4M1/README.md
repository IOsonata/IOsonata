# RA4M1 native MCU port

Native MCU port under the existing `ARM/Renesas` tree: system startup, clock
reporting, Cortex-M4 vectors, normal-boot linker layout, GPIO/pin control, and
CPU interrupt registration through the ICU, asynchronous SCI UART, and AGT/GPT timers.
See [TIMER.md](TIMER.md) for timer device mapping, operation and validation limits.
No board package, Arduino/FSP runtime, USB controller, DMA/DTC, or NMI
driver is included yet.

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
`lib/ioc/Release/libIOsonata_RA4M1.a`. The project links the six target sources, the existing `ARM/src/ResetEntry.c`,
and the existing `src/cfifo.c`, `src/device_intrf.cpp`, `src/coredev/uart.cpp`,
and `src/coredev/timer.cpp`;
it does not copy them. It uses
the production target/generic headers and `ARM/CMSIS/Core/Include`, never the
reduced headers in `tests/ra4m1`. CMSIS is taken from the existing repository
files; no new vendor SDK or submodule is required.

This project builds startup, GPIO, ICU, UART and timer support. It does not yet compile
the generic USB/application subsystem. Future driver
increments must add their sources to this same project. The
library does not link an executable or produce a flash image; the application
links it with `ldscript/gcc_ra4m1.ld` and the matching hard-float runtime. Do not
also compile the same startup sources into the application. The accompanying
`wizard.json` supplies the MCU, library and linker metadata using the existing
IOC wizard format; it does not add a board configuration.

### Source integration

Compile `.c` sources as C11 or later and `.cpp` sources as C++11 or later:

- `ARM/src/ResetEntry.c` (existing file, unchanged)
- `ARM/Renesas/RA4M1/src/system_ra4m1.c`
- `ARM/Renesas/RA4M1/src/vectors_ra4m1.c`
- `ARM/Renesas/RA4M1/src/interrupt_ra4m1.cpp`
- `ARM/Renesas/RA4M1/src/iopincfg_ra4m1.c`
- `ARM/Renesas/RA4M1/src/uart_ra4m1.cpp`
- `ARM/Renesas/RA4M1/src/timer_ra4m1.cpp`
- `src/cfifo.c`, `src/device_intrf.cpp`, `src/coredev/uart.cpp`, `src/coredev/timer.cpp` (existing, unchanged)

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
register subset**, not a complete Renesas device header. `ra4m1_ioregs.h`
adds the GPIO/ICU subset used by this port. These keep the implementation
independent of FSP configuration headers. Other peripheral definitions remain
work for subsequent increments. Do not include both this core wrapper and an
unrelated complete vendor device header in the same translation unit.

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

## GPIO and pin configuration

Use the existing `coredev/iopincfg.h` APIs and this target's `iopinctrl.h`.
Port/pin numbering is the MCU numbering: P105 is port 1, pin 5; P110 is port 1,
pin 10. No application wiring is compiled into the driver. A build may define
`RA4M1_PACKAGE_PINS` as 40, 48, 64, or 100 in both library and application.
Unspecified means the 100-pin group superset; the application is still responsible
for selecting a pin actually bonded on its part. Package configuration changes
require rebuilding the library as well as the application.

`IOPINOP_GPIO` selects GPIO; `IOPINOP_FUNC0` through `IOPINOP_FUNC30` encode
PSEL 1 through 31, following the RE01 convention. Only function codes implemented
somewhere on RA4M1 are accepted, and the caller must choose a valid per-pin
function from the manual's pin table. `IOPINOP_FUNC31` selects analog operation
on analog-capable pins. The driver disconnects PMR before changing the mux,
preserves the output latch, and restores the prior PWPR and PRIMASK state.

Digital input/output, pull-up, normal and supported N-channel open-drain modes
are implemented. Pull-down and FOLLOW requests are rejected (no register writes),
not silently converted to a different pull. Port 0 open-drain is not supported.
GPIO `IOPINDIR_BI` starts as input; alternate-function direction is controlled by
the peripheral. `IOPinSetDir` changes a configured digital GPIO between input
and output. Invalid requests use the generic API's existing failure convention:
void configuration calls leave the pin unchanged, interrupt calls return failure.
`IOPinSetSpeed` is an explicit no-op: no independent slew-rate field is exposed.
Regular/strong drive selects low/middle drive; P408's special highest-drive mode
is not exposed by this two-level generic API.

Set/clear and port writes use PCNTR3's atomic set/reset fields. Toggle snapshots
the output latch inside a short CPU critical section and changes only its pin.
Fast controls require the caller to establish GPIO operation first; they do not
configure muxes or initialize retained pin buffers. DMA/ELC consumers must be
quiesced before software reconfiguration. ELC-controlled output bits are masked
out of software output writes. P200/P214/P215 are input-only. Active main/subclock
pins are not reconfigured, and the paired P914/P915 USB/GPIO mode transition is
deferred to the USB port rather than attempted as independent per-pin writes.

For P402/P403/P404, configuration checks backup-domain readiness before reading
retained input/output controls and refuses pins retained for RTC/VBATWIO use.
It restores the previous power-switch/protection state and does not reset the
RTC, clear backup flags, or take over retained functions. `IOPinDisable` removes
an owned pin interrupt and returns the pin to input/no-pull; it is not a claim
of physical pad-power disconnection.

## ICU and GPIO interrupts

`Ra4m1RegisterIntHandler` / `Ra4m1UnregisterIntHandler` follow the existing RE01
registration interface. There are 32 CPU slots; peripheral event IDs and slot
numbers are different. Allocation validates priority/event, rejects duplicate
IELSR/DELSR routes, skips enabled/active/manually routed slots, and reserves slots
with strong application `IELn_IRQHandler` overrides. Callbacks receive the
allocated CPU slot and run synchronously in ISR context without holding a new
global interrupt mask across application code. Keep callbacks nonblocking.

Configure and negate the peripheral request before registration. For non-pin
sources the callback must negate/read back the source and allow two module-clock
synchronization cycles before returning; the dispatcher then clears IELSR.IR.
GPIO edge requests are acknowledged before the callback so a new callback-time
edge is retained. Normal ISR exit never clears NVIC pending state. Generation
checks prevent post-callback acknowledgement from modifying a released/replaced
registration. `Ra4m1AcknowledgeInt` lets a source-specific handler acknowledge
a negated/synchronized request before invoking application callbacks. It marks
that dispatch as already acknowledged, so the trailing dispatcher does not
erase an event raised during application code. It never clears NVIC pending. Quiesce the source before unregistering; the integer slot is not
a persistent handle. Do not change an allocated IELSR/DTCE behind this API.
Unregister does not wait for an already active callback: its code/context must
remain alive until the callback returns, including across higher-priority
preemption. NMI, DMA/DTC activation and general ELC routing are not implemented.

The event header records the manual's event IDs, not a promise to route every
ELC event to the CPU. This increment deliberately rejects the source IDs for
Snooze entry, USB/SSI FIFO transfers, and ELC software events. Reserved holes and
IDs outside the manual's implemented range are also rejected. Future transfer
and low-power drivers can extend that supported subset after validating their
activation/acknowledgement rules.

`IOPinEnableInterrupt(IntNo, ...)` strictly validates the fixed hardware line
against the pin. `IOPinAllocateInterrupt(...)` finds that line and returns it;
`IOPinDisableInterrupt(...)` accepts the returned line. The GPIO callback receives
this **fixed GPIO line**, not its dynamically allocated CPU slot. Lines 0–12, 14,
and 15 are supported where bonded; there is no IRQ13. The complete group map is:

| IRQ line | MCU pins |
| --- | --- |
| 0 | P105, P206, P400 |
| 1 | P101, P104, P205 |
| 2 | P002, P100, P213 |
| 3 | P004, P110, P212 |
| 4 | P111, P402, P411 |
| 5 | P302, P401, P410 |
| 6 | P000, P301, P409 |
| 7 | P001, P015, P408 |
| 8 | P305, P415 |
| 9 | P304, P414 |
| 10 | P005 |
| 11 | P501 |
| 12 | P502 |
| 14 | P505 |
| 15 | P011 |

Configure a digital GPIO input before enabling its interrupt. Falling, rising,
and both-edge sensing are implemented; IRQCR filtering is disabled in this
increment. Busy/invalid channels are rejected, allocation failure restores the
previous pin/IRQCR configuration, and configuration cannot implicitly release
another pin's interrupt. Existing WUPEN routes are not commandeered. GPIO
interrupt registration does not change low-power wake policy.

Like RE01, `IOPinSetSense` configures port-group EOF/EOR events, not the IRQCR
mode of a registered pin interrupt. It applies to ports 1–4 and requires the
consumer to disconnect its ELC route while changing sense. General ELC consumers
remain outside this increment.

The vector object now has unresolved references to the weak IEL entries supplied
by the interrupt manager. This ensures normal static-library extraction brings
in the dispatcher; it no longer silently resolves the IRQs to startup-only trap
aliases. Strong application vector overrides are still supported.

## SCI UART

The native `uart_ra4m1.cpp` implements the existing `UARTDEV`, `UARTCfg_t`,
`DevIntrf_t`, and `CFifo` interfaces. There is no second UART API, queue or FSP
runtime. Device numbering follows the compact RE01 convention:

| `UARTCfg_t.DevNo` | Hardware SCI | Clock | Module-stop bit |
| --- | --- | --- | --- |
| 0 | SCI0 | PCLKA | MSTPB31 |
| 1 | SCI1 | PCLKA | MSTPB30 |
| 2 | SCI2 | PCLKA | MSTPB29 |
| 3 | SCI9 | PCLKA | MSTPB22 |

Supply exactly two `IOPINCFG` entries in the usual **RX, TX** order. Pin functions
are the application's MCU pin-map choices, not a board default. RX must be input,
TX output, and each present pin must specify its valid SCI peripheral PSEL code
using the target's `IOPINOP_FUNCn` convention. The caller must select the correct
per-SCI/per-pin combination from the hardware manual; the generic pin configurator
does not resolve that mapping. To omit RX or TX, use `PortNo=PinNo=-1` in its
array slot. At least one direction must be present.

This increment supports asynchronous full-duplex UART, 7/8 data bits, no/odd/even
parity, and one/two stop bits. Both polling and interrupt mode are implemented.
Unsupported requests fail initialization: DMA, IrDA, hardware/software flow
control, half-duplex, synchronous/network mode, 9-bit data and automatic baud.
`UARTSetCtrlLineState` does not drive modem lines or transmit break in this
increment. Framing, parity, overrun and received-break indications are reported
through the existing counters/line-state callback; damaged bytes are discarded.

SCI0/1 hardware FIFOs are disabled deliberately, giving all four channels the
same non-FIFO register path. Each direction has a default 16-byte software FIFO,
overridable at library build with `RA4M1_UART_FIFO_SIZE`, or with the existing
caller-supplied, word-aligned FIFO memory in `UARTCfg_t`. Memory sizes include
`sizeof(CFifo_t)`. `bFifoBlocking=true` refuses insertion when full; RX still
drains the hardware and counts dropped new bytes, rather than blocking an ISR.
`false` retains the existing CFifo push-out policy and counts evicted bytes.
There is no hardware receive backpressure without flow control.

Interrupt TX copies accepted bytes into CFifo and primes TDR directly. TXI feeds
subsequent bytes; TEI marks final wire completion. Queuing during an in-flight
final byte re-arms TXI. `UART_EVT_TXREADY` is delivered when a full FIFO gains
space and when transmission drains; its length is the available FIFO space.
RX callbacks carry a null buffer and the queued byte count. Application callbacks
run outside newly acquired PRIMASK sections and may queue more data. The SCI
source is negated/synchronized and the ICU route acknowledged before application
code; a callback-time arrival is not cleared on dispatcher exit. Code/context
must remain valid until an already active callback returns after higher-priority
preemption, as with the ICU registration contract.

Polling allocates no ICU slots and writes accepted bytes directly to TDR, never
leaving accepted data in an undriven software TX queue. Reads return currently
available data; writes have a bounded retry count. Both modes can return a partial
byte count. The caller must retry the unaccepted suffix. Polling completion flags
are sampled on API return, not updated asynchronously without interrupts.

Baud selection uses PCLKA from `SystemPeriphClockGet(0)`, integer BRR/CKS and
optional MDDR modulation. The returned `Rate` is the rounded nominal hardware
rate, not the requested value or a measurement of oscillator accuracy. The
planner considers the supported sampling modes and rejects nominal divisor
error above `RA4M1_UART_MAX_ERROR_PPM` (default 20,000 = 2%). Include oscillator
and peer-clock tolerance in the application's total error budget. The legal
clock/divisor range is not a throughput guarantee for the non-FIFO ISR path.

`UARTSetRate` requires `UARTDisable` and a quiesced peer first; it returns zero
without changing a running interface. Changing system clocks also requires
quiescing UART, then setting its rate again before enabling. Disable aborts
in-flight TX and discards/counts queued TX while retaining RX FIFO data, register
configuration and module clock. Bytes already in the hardware pipeline may be
truncated; their exact wire progress is not counted as a known byte drop. Enable
restores operation without full initialization. Reset also flushes RX. PowerOff
releases ICU routes, restores prior pin configuration, gates the SCI module and
requires full UARTInit for reuse. Duplicate channel/device ownership, overlapping
RX/TX FIFO memory, invalid modes and partial initialization failures are rejected.

Run `python3 tests/ra4m1/run_uart_validation.py` from the repository root for the
UART model and the existing startup/GPIO/ICU regression suite. The UART test
executes the actual UART, GPIO and ICU sources, but its CMSIS and UART/DevIntrf
headers are reduced test contracts. Its default standalone run uses a CFifo test
double. In a full checkout, the runner additionally builds against the unchanged
production `src/cfifo.c`. The model checks register access sizes, frame/baud
configuration, all four SCI channels, FIFO overflow, partial transfers, TX restart,
callback-time RX, callback rearming/release, disable/rate/reset, allocation and
write failure cleanup, and simultaneous independent channel ownership. It does
not prove the production C++ ABI, interrupt latency, maximum sustainable baud or
physical serial operation. IOC uses production headers and the shared sources,
never these test shims.

## Validation status

Run `python3 tests/ra4m1/run_validation.py` from the repository root.
The tests use reduced CMSIS/generic-contract **test shims**, clearly separated
from production includes. They run the actual startup implementation against a
register model and compile/link an ARM layout probe. They are not an FSP build,
a full IOsonata/newlib firmware build, a CPU emulator, or hardware validation.

Automated checks run the 59 existing startup scenarios and 188 GPIO/ICU
assertions at O0/Os/O2 and with ASan/UBSan (sanitizer diagnostics are fatal).
They also check package masks, ARM compilation, ELF layout, ordinary archive
extraction, strong vector override, and IOC source links/metadata. These are
model/compiler/linker checks, not GPIO electrical or interrupt-latency measurements.

The first physical check is cold boot and debugger entry to the application,
then the default register values above and a measured CPU/timer clock. Repeat
with software and external reset. Then use an application-supplied GPIO input/output
pair to measure output transitions and falling/rising/both-edge callbacks. No
particular LED, button, test-pin pairing, or evaluation-board project is assumed.

## Sources

IOsonata patterns were read from `sam4l_usb_debug` commit
`721b361401b1db64ce7ccd2ebc532f88bbfb48b8`: the RE01 startup/vectors, shared
ResetEntry, generic clock/interrupt contracts, and common ARM linker script.

Hardware basis: Renesas RA4M1 Group User's Manual: Hardware,
**R01UH0887EJ0110, Rev. 1.10, September 29, 2023**. Relevant sections are chapter
6 (option-setting memory), chapter 8 (clocks), chapter 10 (operating modes),
chapter 12 (write protection), chapter 13 (ICU), chapter 44 (Flash/cache), and
chapter 48 (electrical limits). GPIO/ICU additionally uses sections 1.7 (pin
assignments), 13.2/13.3 (IRQCR, IELSR/DELSR and request handling), chapter 19
(I/O ports, package bonding, PFS/PWPR and usage notes), and chapter 11 (battery
backup and retained pin controls). The Renesas-authored manual text was read
through a public document mirror when the official PDF exceeded the fetch limit.
Renesas FSP `v6.6.0` RA4M1 event definitions and `r_ioport.c` were secondary
cross-checks, not a runtime dependency. GPIO API/project patterns were read from
IOsonata `main` at `eef99dc82cb3936e5c0f796563a3a58104e50ead`.

UART hardware basis: the same Rev.1.10 manual, chapter 28 (SCI), chapter 10
(module stop) and section 13.3 (interrupt request handling). Renesas FSP
`v6.6.0` `r_sci_uart.c` was a secondary sequencing cross-check only.
