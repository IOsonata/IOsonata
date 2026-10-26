# RA4M1 native MCU port

Native MCU port under the existing `ARM/Renesas` tree: system startup, clock
reporting, Cortex-M4 vectors, normal-boot linker layout, GPIO/pin control, and
CPU interrupt registration through the ICU, asynchronous SCI UART, AGT/GPT timers, and IIC master and slave.
See [TIMER.md](TIMER.md) for timer device mapping and validation limits, and
[INTERRUPTS.md](INTERRUPTS.md) for the ICU/NVIC routing architecture,
registration rules, and interrupt-maintenance invariants.
No board package, Arduino/FSP runtime, IIC DMA/DTC,
general DTC, or NMI driver is included yet. UART TX uses native DMAC.
The IIC master (polling and interrupt) and slave are rewritten and checked
against a host model of the IIC, not yet hardware-validated.

## IOsonata 0.13 release scope

RA4M1 I2C (IIC) fixes and master/slave hardware validation are deferred beyond
0.13 by maintainer decision on October 18, 2026. The unresolved hardware
issue remains open; the existing IIC implementation is not hardware validated
and is excluded from the validated functionality claimed for this release.
This deferral removes RA4M1 I2C from the 0.13 merge requirements. Startup,
GPIO, timer and UART hardware results remain unchanged.

## Implementation status (October 13, 2026)

Target: `prerelease_0.13`. The RA4M1 register-access migration uses Renesas
CMSIS peripheral structures rather than handwritten base addresses and
register-offset wrappers. The existing IOsonata interfaces remain unchanged.
This is an MCU-level implementation, not an EK-RA4M1 board support package.

The user has confirmed successful firmware compilation and execution on an
EK-RA4M1: **Blinky PASS** (startup and GPIO output) and **TimerDemo PASS**
(initialization and timer callback operation). A subsequent hardware log
exercised Timer DevNo 3 (GPT1), with four compare callbacks configured for
100 ms, 1000 ms, 250 ms and 500 ms. All four generated callbacks; sustained
T0/T1/T2 measured intervals largely matched their requested periods.
The printed T3 callback intervals varied; the test used J-Link `rdimon`
semihosting, which can halt the CPU and distort callback timestamps.
Therefore the log is **functional evidence, not a bound on GPT interrupt
latency or period accuracy**. For timing measurement, collect timestamps in
RAM without semihosting inside the measurement loop.

**Not yet confirmed on hardware:** the rewritten IIC master and slave paths, the RSPI master and slave (new, see RSPI master and slave), GPIO edge interrupts,
other AGT/GPT channels and all six GPT compare routes, long-run rollover/pause/resume,
stop/start recovery, low-power operation, general DMA/DTC and ELC integration.
See USBFS supply and validation below for current USB hardware results.
SCI UART is complete and hardware-tested. Apart from the UART test
(`run_uart_validation.py`, see SCI UART), the host-register-model test runners
under `tests/ra4m1` predate the CMSIS migration and require updates;
the old published model assertion counts are not post-migration results.
Neither a complete automated production-header compiler matrix nor updated
host regression results are claimed here.

### Interrupt architecture

The Cortex-M4 vector table has **16 ARM-defined system positions** (the first
stores the initial MSP) followed by **32 external NVIC vectors** IRQ0..IRQ31.
Renesas ICU `IELSR[0..31]` selects which peripheral event feeds each external
NVIC slot. The NVIC positions and independent priorities are fixed; their
peripheral event sources are programmable. Thus the peripheral events do
**not** all share one interrupt.

`interrupt_ra4m1.cpp` implements the shared **ICU registration and routing
infrastructure**: it selects an unused CPU slot, assigns its event ID, installs
a callback/context, sets NVIC priority, dispatches the corresponding
`IELn_IRQHandler`, acknowledges the ICU request and releases routes.
`vectors_ra4m1.c` supplies the hardware vector table. One timer overflow
event and each enabled GPT compare event may consume separate CPU slots;
Timer DevNo 3 with four compare triggers can use five slots. GPIO's fixed
external pin IRQ *line* is distinct from the dynamically allocated NVIC slot.
The ICU driver is not a replacement for the ARM NVIC.

### Remaining engineering work

Complete the RA4M1 coding-style pass (file headers, declaration placement,
one statement per line, explicit multi-line control-flow blocks), refresh the
CMSIS-based host regression models, run production-header Cortex-M4 Debug
and Release checks, and validate the IIC master, IIC slave and GPIO interrupts on hardware.
No merge into `main` is authorized by this documentation update.

## IOsonata integration

The pattern is `ARM/Renesas/RE01`: an MCU include wrapper, a weak `g_McuOsc`,
`system_*.c`, `vectors_*.c`, `.vectors`, `__Vectors`, `__StackTop`, and
`IEL0_IRQHandler` through `IEL31_IRQHandler`.

### IOC library project

Import `ARM/Renesas/RA4/RA4M1/lib/ioc` with **Existing Projects into Workspace**.
The project is named `IOsonata_RA4M1` and provides `Debug` (`-O0`, `-g3`) and
`Release` (`-Os`) configurations. Both use Cortex-M4 Thumb, `fpv4-sp-d16`, and
hard-float ABI, with the normal Arm Cross GCC managed-build tools. Language
settings follow the Renesas library template: GNU C17 and GNU C++23, with C++
RTTI and exceptions disabled. Configure
the compiler installation through IOC's existing toolchain settings, not an
absolute path embedded in the project.

The outputs are `lib/ioc/Debug/libIOsonata_RA4M1.a` and
`lib/ioc/Release/libIOsonata_RA4M1.a`. The project has the same layout and
the same common sources as the other IOsonata libraries (made from the
STM32G474 library project): the generic `src` and `include` trees, sensors,
storage, display, DFU, micro-ecc, littlefs and FatFs support, with the RA4M1
sources in place of the vendor ones: `system_ra4m1.c`, `vectors_ra4m1.c`,
`iopincfg_ra4m1.c`, `interrupt_ra4m1.cpp`, `uart_ra4m1.cpp`, `i2c_ra4m1.cpp`,
`spi_ra4m1.cpp`, `timer_ra4m1.cpp` and `nvm_ra4m1.cpp`, plus `nvm.cpp`,
`nvm_region.cpp`, `diskio_nvm.cpp` and `syslog.cpp`. All are linked, not
copied. The include paths are `ARM/Renesas/RA4/RA4M1/include`, `ARM/include`,
`ARM/CMSIS/Core/Include`, the repository `include`, littlefs, FatFs and
micro-ecc, never the reduced headers in `tests/ra4m1`. Release defines
`NDEBUG`. No vendor SDK or submodule is required.

The library does not link an executable or produce a flash image; the
application links it with `ldscript/gcc_ra4m1.ld` and the matching hard-float
runtime. Do not
also compile the same startup sources into the application. The accompanying
`wizard.json` supplies the MCU, library and linker metadata using the existing
IOC wizard format; it does not add a board configuration.

### Source integration

Compile `.c` sources as C11 or later and `.cpp` sources as C++11 or later:

- `ARM/src/ResetEntry.c` (existing file, unchanged)
- `ARM/Renesas/RA4/RA4M1/src/system_ra4m1.c`
- `ARM/Renesas/RA4/RA4M1/src/vectors_ra4m1.c`
- `ARM/Renesas/RA4/RA4M1/src/interrupt_ra4m1.cpp`
- `ARM/Renesas/RA4/RA4M1/src/iopincfg_ra4m1.c`
- `ARM/Renesas/RA4/RA4M1/src/uart_ra4m1.cpp`
- `ARM/Renesas/RA4/RA4M1/src/i2c_ra4m1.cpp`
- `ARM/Renesas/RA4/RA4M1/src/spi_ra4m1.cpp`
- `ARM/Renesas/RA4/RA4M1/src/timer_ra4m1.cpp`
- `src/cfifo.c`, `src/device_intrf.cpp`, `src/coredev/uart.cpp`, `src/coredev/i2c.cpp`, `src/coredev/spi.cpp`, `src/coredev/timer.cpp` (existing, unchanged)

Use the RA4M1 `include` directory, IOsonata `include` and `ARM/include`, and the
project's existing CMSIS Core include directory. Link with
`ARM/Renesas/RA4/RA4M1/ldscript/gcc_ra4m1.ld`. Use the usual IOsonata/newlib runtime
and library settings; this linker script does not hard-code a C library group.
CPU flags are `-mcpu=cortex-m4 -mthumb`. When using hardware floating point,
use `-mfpu=fpv4-sp-d16` and the same `-mfloat-abi` in the application and every
linked library. Compile the vector object directly, or retain it from the
library through the existing ResetEntry reference to `__Vectors`.

The wrapper `ra4m1xxx.h` supplies the CMSIS core configuration, the
32 ICU slot IRQ numbers, and includes the Renesas vendor device header
`include/R7FA4M1AB.h`. Production startup, GPIO/PFS, ICU, AGT/GPT, SCI and IIC
drivers access the vendor peripheral structures and register members directly
(`R_SYSTEM`, `R_PORTn`, `R_PFS`, `R_ICU`, `R_AGTn`, `R_GPTn`, `R_SCIn`).
The old private register-map headers `ra4m1_startup_regs.h`, `ra4m1_ioregs.h`,
`ra4m1_timer_regs.h` and `ra4m1_uart_regs.h` were removed. Pin/package
capability masks and peripheral control-bit values remain MCU-specific data,
not alternative register-address maps. No FSP runtime or vendor driver is used.
The Renesas header retains its vendor licensing.

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

**The default 48 MHz operating point requires VCC of 2.7-5.5 V.** This initial
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

Main crystal/external-clock descriptors accept 1-20 MHz. Startup uses an exact
48 MHz PLL solution when the input is 4-12.5 MHz and a legal integer multiplier
and output divider exist. Otherwise it uses the main oscillator directly.
Examples: 4, 6, 8, and 12 MHz can supply the 48 MHz PLL plan; 16 MHz cannot use
this PLL input range and is therefore used directly. PLLCCR2 is an **8-bit**
register at `0x4001E02B`; the implementation does not copy RE01 PLL encodings.

`bUSBClk=true` requires a 48 MHz source and sets only the USB clock selector.
HOCO is a device-only USB source and requires zero HOCO user trim; PLL can
supply the clock for host or device. USBFS remains stopped/disabled. This is
clock preparation only; the USBFS controller driver enables the peripheral.

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

## IIC master and slave

The native RA4M1 IIC driver exposes **DevNo 0 = IIC0** and **DevNo 1 = IIC1**
through the existing `coredev/i2c.h` interface: standard 7-bit I2C up to
400 kHz as master or as slave. DMA/DTC, SMBus, 10-bit address and Fast-mode
Plus requests are rejected by `I2CInit`.

Master. One service routine advances the transfer from the IIC flags
(START, TDRE/TEND, RDRF, NACKF, STOP, arbitration and timeout). Polling mode
loops on it, interrupt mode (`bIntEn`) calls it from the RXI, TXI, TEI and EEI
handlers, so both follow the same register sequence, the one of the Renesas
FSP `r_iic_master`: dummy read of the address phase, WAIT for the last bytes,
ACKBT set before the final byte, STOP requested before the last read, NACK
answered with STOP, repeated START with the timeout function off.
`StartTx`/`StartRx` only record the target. The bus is used by `TxData` and
`RxData`; a `TxSrData` transfer (the register address of a register-style
read) keeps the bus and `RxData` issues the repeated START. In interrupt mode
`TxData` copies up to 64 bytes, returns -1 and the end is reported with
`DEVINTRF_EVT_COMPLETED` and the byte count, `RxData` returns -1 and fills the
caller buffer before that event. A transfer that keeps the bus, or one larger
than 64 bytes, runs to the end inside the call. A NACK returns the number of
bytes acknowledged. Arbitration loss and the hardware or software timeout
reset the IIC and end the transfer, so the next one starts clean.

Slave (`bIntEn` required). One to three distinct 7-bit addresses of
0x08 to 0x77, one per slave address register. The first byte after an address
match is the address itself and gives the direction. `WRITE_RQST` or
`READ_RQST` is sent then, and the application installs the buffer with
`I2CSetWriteRqstBuffer` or `I2CSetReadRqstData`; the pointers are cleared
before each request, as in the other IOsonata slave backends. The event does
not carry the address index, so an application using several addresses sets
the buffer of every index. `READ_RQST` after a repeated START carries the byte
count of the write that came before. `COMPLETED` carries the byte count at
STOP, at a repeated START, or on an error. A write beyond the buffer is
refused with NACK from the byte after the last stored one. A read beyond the
data returns 0xFF and does not count. After a master NACK the slave does the
dummy ICDRR read that releases SCL.

IIC uses PCLKB. Bit-rate programming searches CKS/ICBRL/ICBRH values that meet
the standard/fast-mode minimum SCL low/high periods while keeping the nominal
rate at or below the requested value. The calculation assumes zero rise/fall
time, making the programmed timing conservative on the real open-drain bus.
The digital noise filter is disabled so its additional clock stages do not
make the rate model implicit. The count offsets used for ICBRL/ICBRH
(3 with CKS=0, 2 otherwise) are the ones to confirm with a scope.

The valid peripheral pin alternatives are IIC0 SCL on P204/P400/P408 with SDA
on P401/P407, and IIC1 SCL on P100/P205 with SDA on P101/P206. These require
PSEL 7, represented by `IOPINOP_FUNC6`. The application still selects one
legal pair and must provide suitable pull-ups for its bus capacitance and speed.
The driver configures both pins as `IOPINDIR_BI` whatever direction the pin map
lists. The IIC block controls the pin direction, and the internal pull-up
(`IOPINRES_PULLUP`) has no effect on a pin with PDR set, which an
`IOPINDIR_OUTPUT` entry for SCL would produce. The internal pull-up is weak
(tens of kOhm), so external pull-ups are advised above a few tens of kHz.

`DeviceIntrfReset` uses open-drain SCL/SDA, clocks a stuck slave up to nine
times, emits STOP when possible, and then restores the peripheral pin
configuration.

Validation: `python3 tests/ra4m1/run_i2c_validation.py` runs
`tests/ra4m1/i2c_test.cpp` against a register and bus model of the IIC, at
-O0, -Os, -O2 and with the address and undefined behavior sanitizers, and
compiles the driver for Cortex-M4 with the real headers. The model checks the
bus sequence (every start, address, byte, acknowledge and stop), the SCL
hold of WAIT and of a master NACK, ICU edge events with a settable handler
latency, and stuck-bus, arbitration-loss and stuck-slave recovery. It follows
the FSP flows and is not the silicon: whether START/STOP/NACKF flags set with
their interrupt disabled (polling), whether TXI and TEI are edge or level at
the ICU, and the exact bit-rate offsets are open until the hardware run.

## RSPI master and slave

`src/spi_ra4m1.cpp` implements `SPIInit` of `coredev/spi.h` for **DevNo 0 =
RSPI0** and **DevNo 1 = RSPI1**. It follows the RE01 RSPI master (polled,
clock synchronous mode, GPIO chip select) and the SAM4L slave event sequence
(`DEVINTRF_EVT_STATECHG`, then `DEVINTRF_EVT_COMPLETED`). The RSPI clock is
PCLKA. The generic `SPI` class and the C functions are unchanged. **Hardware
not tested.**

Master.
- Full duplex, SPI modes 0 to 3, MSB or LSB first, 8 to 16 bit frames. A frame
  of more than 8 bits uses two bytes in the buffer, low byte first, and the
  length must be a multiple of two.
- The RSPI runs in clock synchronous (3 wire) mode and the chip select is a
  GPIO from the pin map (index 3 and up, `IOPINOP_GPIO` output) driven by the
  driver with `SPICSEL_AUTO`; with `SPICSEL_MAN` the application drives it.
- Rate: `Rate` selects the closest supported SCK, PCLKA / (2 x (SPBR + 1) x
  2^BRDV), from PCLKA / 4096 to PCLKA / 2. A request outside that range gets
  the nearest end and the actual rate is returned. A request of 0 selects
  1 MHz.
- Polling by default. With `bIntEn`, a `TxData` or `RxData` of more than one
  frame runs from the receive interrupt, keeps two frames in the RSPI and
  reports the end with `DEVINTRF_EVT_COMPLETED` and the byte count (-1 on a
  hardware error). Those calls return -1. `TxData` copies up to 256 bytes into
  the driver, longer data and every `TxSrData` command phase run polled inside
  the call. `bDmaEn` is accepted and ignored; `DevIntrf_t::bDma` is false.

Slave (`bIntEn` required).
- 8 bit frames, modes 1 and 3 only. The RSPI slave does not support CPHA = 0
  (Renesas FSP refuses it too) and `SPIInit` returns false for it. The master
  has to release the chip select between frames.
- The select input is SSL0 of the unit and is pin map index 3. One slave
  device, `SlaveIdx` 0.
- The interrupt order is: STATECHG (the application calls `SetSlaveRxBuffer`
  and `SetSlaveTxData`, index 0), the master clocks the frame, COMPLETED with
  the byte count, or -1 after an overrun, underrun or when the frame is longer
  than the RX buffer. The driver arms again from the same interrupt, which
  discards the unused TX prefetch, so every frame starts at TX byte 0. TX
  bytes past the data length are `DummyByte`.
- The next TX frame must be in the RSPI before the master clocks it. The driver
  loads two frames when armed and writes frame k + 2 from the receive interrupt
  of frame k. The master SCK rate must therefore leave time for one interrupt
  per frame, including the interrupts of the other peripherals. Start at
  400 kHz or lower and raise it only after a hardware run. The limit was not
  measured.

Pins (LQFP100, function PSEL 6 = `IOPINOP_FUNC5`):

| Signal | RSPI0 | RSPI1 |
| --- | --- | --- |
| RSPCK | P412, P102 | P111, P204 |
| MISO | P410, P100 | P110, P202 |
| MOSI | P411, P101 | P109, P203 |
| SSL0 (slave select) | P413, P103 | P108, P112, P205 |

`SPIInit` returns false when a pin is not in the table for its unit. Master
chip select pins are plain GPIO outputs and are not checked against SSL pins.

Assumptions to confirm on the board:
- The slave frame ends when the RSPI goes idle (SPII, IDLNF falling) after the
  master releases the chip select. The manual text was not available when the
  driver was written; the sequence follows the FSP `r_spi` driver for the TX
  and RX parts. If COMPLETED never arrives, the slave end detection is the
  first place to look.
- A slave error that stops the unit (SPE cleared) ends the frame at once and
  arms the next one while the chip select may still be low.
- A slave idle event with no received byte is ignored.

Example: `exemples/SPIMasterSlave` builds the shared
`exemples/spi/spi_master_slave.cpp` with RSPI0 as master and RSPI1 as slave on
the EK-RA4M1 headers, see `exemples/README.md`.

Validation: `python3 tests/ra4m1/run_spi_validation.py` runs
`tests/ra4m1/spi_test.cpp` against a frame level model of two RSPI units wired
as master and slave, with the real `DeviceIntrf` and `SPI` generic code, at
-O0, -Os, -O2 and with the address and undefined behavior sanitizers, and
compiles the driver for Cortex-M4 with the real headers. The model has the
transmit buffer and shift register, SPRF/SPTEF/IDLNF, the error flags, the
SCKASE clock stop, SPRI/SPII/SPEI through a simple ICU and PRIMASK. It checks
the register sequence, the data of modes 1 and 3 at 1, 16 and 129 bytes in
polled and interrupt master mode, short frames followed by long frames,
overflow and re-arm, copy of interrupt TX data, busy flag and cancel,
re-initialization, pin and parameter rejection, and module stop and PRCR use.
The model CPU is infinitely fast: it does not show whether the slave keeps up
with a given SCK rate. The SPII behavior at the end of a slave frame is a model
assumption. It is not hardware validation.

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
this **fixed GPIO line**, not its dynamically allocated CPU slot. Lines 0-12, 14,
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
mode of a registered pin interrupt. It applies to ports 1-4 and requires the
consumer to disconnect its ELC route while changing sense. General ELC consumers
remain outside this increment.

The vector object now has unresolved references to the weak IEL entries supplied
by the interrupt manager. This ensures normal static-library extraction brings
in the dispatcher; it no longer silently resolves the IRQs to startup-only trap
aliases. Strong application vector overrides are still supported.

## SCI UART

`uart_ra4m1.cpp` implements the IOsonata UART (`UARTDev_t`, `UARTCfg_t`,
`DevIntrf_t` and the CFIFOs) with the same structure as the other UART drivers.

| `UARTCfg_t.DevNo` | SCI | Hardware FIFO | Module stop bit |
| --- | --- | --- | --- |
| 0 | SCI0 | 16 bytes RX and TX | MSTPB31 |
| 1 | SCI1 | 16 bytes RX and TX | MSTPB30 |
| 2 | SCI2 | none, RDR and TDR | MSTPB29 |
| 3 | SCI9 | none, RDR and TDR | MSTPB22 |

The pin map is RX then TX, with the PSEL function code of each pin in `PinOp`
(`IOPINOP_FUNCn`). The pin map is selected by the application from the hardware
manual, the driver does not check the SCI and pin combination.

Supported: asynchronous full duplex, 7 or 8 data bits, parity none, odd or even,
1 or 2 stop bits, polling, interrupt and TX DMA mode. UARTInit fails for flow
control, IrDA, half duplex, synchronous mode, 9 data bits, parity mark or space,
and DMA with a non-blocking FIFO. `UARTSetCtrlLineState` does nothing.

- TxData puts the data in the TX CFIFO, then starts TX if it is ready. RxData
  reads from the RX CFIFO. Copies to and from a CFIFO are done in pieces of at
  most 8 bytes with interrupts disabled.
- Interrupt mode: one handler for RXI, TXI and ERI, registered with
  `Ra4m1RegisterIntHandler` at `IntPrio`. TEI is not used. RX is serviced first
  whatever the event. TXI is enabled while the TX CFIFO has data, `bTxReady`
  is true when the TX CFIFO is empty. The ICU IR flag of RXI and TXI is cleared
  at the start of the handler, ERI at the end, as the Renesas FSP does. SCI2/9
  have no FIFO: RDR holds one character and the next one overruns it a
  character time later. Their RXI and ERI service RX only and run at
  `IntPrio`, TXI and the DMA transfer end service TX only and run one level
  lower (unless `IntPrio` is already the lowest), so that RXI can preempt
  them. RDR is read only with RDRF or an error flag set, by any of them:
  reading RDR clears RDRF, a pending RXI of a character ERI already read finds
  RDRF cleared and does not read it again. ERI leaves the RXI IR flag as it is.
  Callbacks from RX and TX interrupts can nest on SCI2/9. SCI2/9
  TX waits for a TXI only with TDR full: the CPU writes TDR while TDRE is set,
  two bytes in a row with the line idle (the first goes on to the shift
  register at once), and TxData sets TIE after filling TDR. The TXI then comes
  from the move of a full TDR at the end of a character. On the EK-RA4M1 the
  SCI2 interrupt mode loopback stopped with TIE set, TDR empty, the line idle
  and no IR flag: the TXI after a CPU write to an idle line with TIE already
  set did not come. With
  DMA, TxData starting a block on SCI2/9 locks out only that lower level with
  BASEPRI, RXI preempts it. TX is taken by an atomic exchange of `bTxReady`, a
  TxData from an interrupt finding it taken leaves its bytes in the TX CFIFO to
  the caller that has it.
- DMA mode (`bDMAMode`): while the channel is idle the CPU puts what the SCI
  takes, as without DMA (TX FIFO of SCI0/1, TDR of SCI2/9), which costs no
  interrupt. DMAC channel `DevNo` sends the rest in blocks straight from the
  TX CFIFO, one byte to FTDRL or TDR for each TXI, linked in `ICU.DELSR`. A
  block is the first contiguous run of the CFIFO, at most half of it, and its
  bytes are released when the channel is done. The DMAC transfer end interrupt
  (or TxData and RxData without `bIntMode`) fills the SCI again and starts the
  next block. With TX idle TIE is cleared, so TxData then costs about as much
  as polling without DMA. A TXI that arrived while the channel was stopped can
  be left pending in `ICU.DELSR`. It is cleared, `DELSR` written 0 then the TXI
  again as the Renesas FSP r_dmac stops a transfer: on SCI0/1 after the CPU
  wrote to the SCI at the end of a block and when TX goes idle, on SCI2/9
  before the channel is armed. SCI0/1 use TTRG 0 in DMA mode, TXI with the TX
  FIFO empty, and start a block by setting TIE again, as the Renesas FSP does
  with a transfer instance. SCI2/9 with the line idle (TDRE and TEND set) and
  more than one byte: the channel is armed for the bytes after the first, TIE
  is set, then the CPU writes the first byte to TDR, as the Renesas FSP starts
  a transfer without the FIFO. The move of that byte to the shift register
  gives the TXI for the channel. On the EK-RA4M1 SCI2 the channel got that TXI
  (UartLoopback interrupt DMA started its blocks that way after each late
  transfer end, without a stop), the CPU did not always in interrupt mode.
  With the first byte written by the CPU before arming, its move could come
  before the request was cleared and each such block went through the TDR
  empty path below: short blocks, about 74 kB/s instead of 96 in that
  loopback. Otherwise the CPU fills TDR as in interrupt mode, then TIE is set
  and the channel armed for the rest with TDR full, its move gives the TXI for
  the channel. TDR is read again: read empty with bytes left,
  the channel is stopped (a transfer it already took finishes, DMSTS.ACT),
  DMCRA and TDR are read: TDR full, the channel is armed again; TDR empty, no
  TXI is coming, the bytes the channel moved are released and the CPU fills
  TDR again. When the channel
  moved the last byte of the block, DTE is not set again: in normal mode a
  count of 0 is a transfer with no end, which would send the memory after the
  block. RX stays on the interrupt or polling: the SCI has no idle line
  event to move a DMA ring to the RX CFIFO after a short burst. The DMAC module
  (MSTPA22) is started and never stopped by the driver, other drivers may use
  it. UARTInit fails if channel `DevNo` is linked to another event.
- Polling mode: TxData and RxData move the data between the SCI and the CFIFOs.
  With SCI2 and SCI9 the application must call RxData or TxData at least once
  per character time or characters are lost to overrun. With DMA, RDR is also
  read at each step of a block start and after it.
- SCI0 and SCI1 FIFO mode: RXI at 8 received characters, or with fewer once the
  line is idle for 1.5 frames (receive data ready). TXI at 8 or fewer characters
  in the TX FIFO.
- Characters with a framing or parity error are dropped and counted
  (`FramErrCnt`, `ParErrCnt`, `RxDropCnt`). Overruns are counted in
  `RxOvrErrCnt`, the characters already received are kept. The line state
  callback gets the error bits, with `UART_LINESTATE_BRK` when RXD is still low
  after a framing error. A full blocking RX CFIFO drops the new character
  (`RxDropCnt`), a full non-blocking one drops the oldest.
- The default CFIFO is 16 bytes (`RA4M1_UART_CFIFO_SIZE`), `pRxMem` and `pTxMem`
  in `UARTCfg_t` give larger ones.
- Rate: PCLKA from `SystemPeriphClockGet(0)`. SetRate sets and returns the
  closest rate the SCI makes (BRR, CKS, SEMR base clock and MDDR modulation). A
  running UART is stopped for one bit time while the rate changes. 0 returns 0,
  there is no automatic rate.
- Disable clears TE and RE. Enable flushes the TX CFIFO, resets the hardware
  FIFOs and the error counters, the RX CFIFO is kept. PowerOff releases the
  interrupts, disconnects the pins and stops the SCI module, a new UARTInit is
  needed after it.

Host test: `python3 tests/ra4m1/run_uart_validation.py`. It compiles the driver
with a model of the SCI, DMAC, module stop, PRCR and ICU registers
(`tests/ra4m1/uart_include`) and the real CFIFO and DeviceIntrf code. RXI, TXI,
ERI and the DMAC transfer end are edge events that set the IR flag of their
ICU slot, and the line can move while a handler runs. DMA runs three times,
with a TXI arriving while DTE is cleared held in DELSR, with it lost, and held
with the channel moving each request it took one register access or line step
later (DMSTS.ACT set until then). A count of 0 written with DTE set, and
channel settings written during a transfer, fail the test. It covers
init, rate selection against every register setting, TX and RX in polling,
interrupt, polled DMA and interrupt DMA mode on the four SCIs, idle time between
DMA blocks, the end of a block seen late (the line moves up to 60 characters
between TxData calls, interrupts held off), the last byte of a SCI2/9 block
moved 1 to 3 register accesses after the channel took it, while the block
start checks TDR and stops the channel, one SCI2/9 overrun with the next
character arriving while it is handled (only the overrun character lost), a
SCI2/9 framing error with the next character arriving at each step of the ERI
handler in turn (received once, no read of RDR without RDRF or an error flag),
SCI2/9 TxData starting a block with only the TX level locked out (BASEPRI in
the model) while the line moves and RXI is taken in the middle of it, its
callback calling TxData too, no TX handler taken inside TxData, the SCI2/9
tests again without a TXI to the CPU from a CPU write to an idle line with TIE
set (the DMAC gets it; with and without a TXI when TIE is set with TDRE set),
and again with such a write going on to the shift register 3 register accesses
later,
an echo loop with bursts, receive
errors, overrun, full CFIFOs, callbacks, Disable/Enable and PowerOff. It also
compiles the driver with GNU Arm for the Cortex-M4. It does not test exact
timing or the electrical interface.

## Validation status

Run `python3 tests/ra4m1/run_validation.py` from the repository root.
The tests use simplified CMSIS and generic API **test headers**, kept separate
from production headers. They run the actual startup implementation against a
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
ResetEntry, generic clock and interrupt interfaces, and common ARM linker script.

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

## Data flash NVM

`nvm_ra4m1.cpp` exposes the 8 KiB data flash at 0x40100000 through the existing
`NvmIntrf` and `Nvm`. `NvmMcuCfg` reports byte programming, 64-byte transfer
chunks and 1 KiB erase blocks. Call `Nvm::Erase` before reusing a programmed
range. The driver uses the FACI blank-check command before programming and
rejects nonblank destinations. It does not emulate EEPROM or erase implicitly.
Code flash and option-setting memory are outside the permitted address range.

The IOC library project includes this port in `libIOsonata_RA4M1.a`.
Import `exemples/NvmDemo/ioc` and rebuild the library and example with matching
configurations. Output uses the existing UART0 pins (P100 RX, P101 TX) at
1000000 baud, 8N1, interrupt mode and no DMA in both Debug and Release.
The test erases/programs **0x40101400-0x40101FFF**, the last three data-flash
blocks. Two blocks are scratch; the last stores a boot stamp. Expect
`ALL PASS`, then power cycle without clearing data flash and check that the
previous boot stamp increases.

This port is polling-only; `NvmIntrf::Init(..., true)` rejects interrupt mode.
Registering an arbiter blocks accesses rather than bypassing its owner. Keep
FCLK between 4 and 32 MHz and unchanged during an operation. Interrupts remain
enabled during polling, but no ISR, DMA or other driver may read data flash or
use FACI while it is in program/erase mode. Code flash remains readable.
Timeout/error paths reset the sequencer and attempt to restore read mode;
if that fails, subsequent operations are refused. Failed writes may partially
program their range. Erase before retrying a failed program.

Debug/Release library and NvmDemo compile/link checks pass with GCC 13.2.1.
`tests/storage/data_nvm/run.py` executes the production ARM port and generic
Nvm code under a register model, including P/E address conversion, protected
mode writes, blank checks, erase boundaries, error/timeout recovery and bounds.
Data-flash hardware validation is pending. The model does not establish flash
timing, endurance or power-loss behavior.

Controller sequence reference: Renesas RA4M1 hardware manual, FACI LP data-flash
program/erase flowcharts, cross-checked against the RA4M1 MF3 paths in Renesas
FSP `ra/fsp/src/r_flash_lp/r_flash_lp.c`. The port uses the existing vendor
CMSIS registers directly and has no FSP runtime dependency.

## SCE5 crypto

`crypto_ra4m1.cpp` provides two engines on the SCE5 block: `CryptoRa4m1Rng`
(TRNG) and `CryptoRa4m1Aes` (AES-128/256 ECB, CBC and CTR in both directions;
CMAC, CCM and GCM composed over the block by `CryptoAesModes`). Both can be
built with the `CryptoRa4m1RngCreate` and `CryptoRa4m1AesCreate` factories
into caller storage of `CRYPTO_RA4M1_RNG_MEMSIZE` and
`CRYPTO_RA4M1_AES_MEMSIZE` bytes. `crypto_default_ra4.cpp` binds the generic
`Crypto` object to them; SHA-256, HMAC and P-256 keep the generic software
definitions. The two engines share one lock: the SCE5 runs one primitive at a
time and a random draw inside an AES operation is a resource conflict.

The SCE5 has no register description in the hardware manual. Its access
sequences, including the key information table the engine verifies at start,
are the Renesas plain-key primitives in `crypto_sce5.c` (BSD-3-Clause, notice
in `crypto_sce5-LICENSE.txt`, copied from FSP at the commit named in the file
header). Nothing else from FSP is built. An AES operation generates the key
index and initializes the engine once, runs every block through the update
primitive and finalizes at the end; a failed primitive marks the engine
faulted, wipes the request output and resets the block on the next `Enable`.

Host checks: `python3 tests/crypto/run_ra4_models.py` runs the generic
CryptoTest against an OpenSSL model of the primitive interface, and the
production primitive file against a register model with injected poll
failures. `exemples/CryptoTest` on the EK-RA4M1 (48 MHz): 101 of 101 checks.
An AES operation costs about 300 us of key index generation and engine
setup plus about 3.3 us per block (ECB, 1 KiB: 3.39 ms against 51.7 ms in
software); CMAC 1 KiB 6.1 ms, CCM 11.8 ms, GCM 25.0 ms (GHASH in software);
TRNG 16 bytes in 63 us. SHA-256 and P-256 run in software (P-256 key import
1.44 s).


## USBFS supply and validation

The USBFS driver defaults to an external 3.3 V supply on VCC_USB, as wired
on the EK-RA4M1 (E7/E8 closed). USBMC.VDCEN stays clear. For hardware wired
to use the internal USB regulator, compile the IOsonata library with
`RA4USB_USE_INTERNAL_REGULATOR=1`; VCC_USB_LDO must then be 3.8 to 5.5 V
and VCC_USB must be wired as the regulator output, not an external supply.
See the [EK-RA4M1 manual](https://www.renesas.com/en/document/mat/ek-ra4m1-v1-users-manual),
sections 5.6.5.3/5.6.5.4, and the
[RA4M1 datasheet](https://www.renesas.com/en/document/dst/ra4m1-group-datasheet),
table 2.2.

EK-RA4M1 enumeration was confirmed on hardware: the host completed the
configuration descriptor transfer, selected configuration 1, and read back
configuration 1; USBFS entered the configured state. Temporary semihosting
traces and debugger-only state have been removed from the controller.
Release hardware logs show CDC PRBS reception around 105.3 kB/s and CDC
loopback around 423 kB/s in each direction (846 kB/s combined), with zero
reported errors over the supplied approximately 50-second loopback excerpt.
The final 60-second summary was not supplied. Combo stress and unplug/replug
validation remain pending. RA4M1 USBFS operates at full speed (12 Mbit/s).

USBFS FIFO writes follow the Renesas FSP CPU transfer sequence: select the
FIFO, clear the buffer, recheck FRDY, write 16-bit little-endian pairs and an
8-bit odd tail, then read CFIFOCTR before setting BVAL for a short packet.
Source bytes are assembled without requiring aligned caller buffers. Reads
continue to use 8-bit access. Reference: Renesas FSP commit
`6e2675336b37fbd805f0c01d5134adfdab4bf294`, `r_usb_pcontrolrw.c`
(`usb_pstd_ctrl_read`), `r_usb_plibusbip.c` (`usb_pstd_write_data`), and
`r_usb_preg_abs.c` (`usb_pstd_write_fifo`) under
[`ra/fsp/src/r_usb_basic/src`](https://github.com/renesas/fsp/tree/6e2675336b37fbd805f0c01d5134adfdab4bf294/ra/fsp/src/r_usb_basic/src).
`python3 tests/ra4m1/usb_fifo_test.py` checks the production write functions
against delayed post-clear readiness, stuck readiness, byte order, unaligned
buffers, odd lengths, zero-length and full packets at O0/Os/O2. This model
checks software sequencing, not actual USBFS timing or enumeration.


USBFS acknowledges the ICU latch on ISR entry through
`Ra4m1AcknowledgeInt`, following FSP `usbfs_interrupt_handler` in
`ra/fsp/src/r_usb_basic/src/hw/r_usb_mcu.c`. The helper suppresses the
shared dispatcher's trailing acknowledgement. This preserves new requests
latched during the bounded 32-event USB batch. The live hardware trace
reached the full configuration descriptor, then stopped after exactly
32 handled events with INTSTS0=0x2ca2 still pending. The previous trailing
clear could discard that request. `python3 tests/ra4m1/usb_irq_test.py`
executes the production USB ISR, ICU acknowledgement and dispatcher with
injected requests across the batch boundary; it also verifies that the old
ISR loses the pending request. Hardware confirmed that the pending ICU request survives ISR exit, the ISR
re-enters, and enumeration completes.

OUT reception follows FSP `usb_pstd_ctrl_write`: select the FIFO, clear it,
enable BRDY, then set PID=BUF without waiting for FRDY. Waiting for received
data before permitting the host to send it can prevent CDC SET_LINE_CODING
from completing when a terminal opens. FIFO reads still require FRDY.
For IN pipes 1-5, BEMP completes a transfer only when PIPE_CTR.INBUFM is
clear, matching FSP `usb_pstd_bemp_pipe_process`. While it remains set,
the transfer and BEMP interrupt stay active; the caller cannot refill the
buffer prematurely. Interrupt pipes 6-9 retain their separate completion
behavior. Host tests `usb_out_test.py` and `usb_in_test.py` cover these
sequences; see the Release hardware results above for CDC traffic validation.

Data OUT endpoints explicitly request their first destination with the existing
DRDY callback when opened, then request another after each completion or
clear-halt. This follows the generic UsbIntrf reserve/commit flow; it does not
rely on NRDY occurring while a pipe is NAKed. A full software queue keeps the
pipe NAKed and UsbIntrf retries its pending receive after the application
consumes a packet. The pipe is NAKed before reading/clearing its receive FIFO.
`tests/ra4m1/usb_rx_lifecycle_test.py` checks first-buffer acquisition,
100 packet completions, full-queue backpressure, consumption/rearming and
callback teardown. Release CDC loopback hardware results are recorded above.


For concurrent CDC and ISO hardware testing, use
[`UsbCdcIsoStress`](exemples/UsbCdcIsoStress/README.md). It uses one CDC
loopback function and one bidirectional ISO function (five physical pipes).
The full dual-CDC/HID/INT/ISO combo exceeds RA4M1 USBFS pipe capacity.
CDC+ISO hardware validation remains pending.


CDC+ISO hardware testing reached CDC echo with zero errors, but ISO lost
alternating validation frames and reported 22 loopback queue drops. The
RA4M1 ISR now retires BEMP completions before BRDY receive callbacks and
SOF scheduling, so simultaneous events free the transmitted FIFO head before
queuing another echo and offering the next frame. ISO scheduler calls with
a null buffer service OUT without submitting a fabricated IN transfer;
a non-null zero-length packet remains a real application transfer. This
matches the existing SAM4L and Nordic distinction between idle service and
a queued frame. `tests/ra4m1/usb_iso_schedule_test.py` checks idle service,
queued ZLPs and 100 co-pending completion/receive/SOF intervals at O0/Os/O2.
These sequencing fixes require a hardware retest; ISO is not yet validated.
