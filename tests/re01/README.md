# RE01 port regression checks

Run the Linux host checks from the repository root:

```sh
make -C tests/re01 test
```

The test compiles the real RE01 port and generated register header. A small
CMSIS shim replaces CPU instructions and NVIC operations; anonymous memory
backs the peripheral addresses. It checks clock reporting and divider bounds,
ICU ownership and reserved events, IRQ8/9 routing, GPIO validation and failed
allocation cleanup, TMR resume, AGT reset/divider limits, comparator allocation
and reuse, UART polling and interrupt paths, SCI9 module control, repeated
initialization, RX drops, error clearing, and partial IRQ allocation cleanup.
The baud-rate check uses an independent exhaustive BRR/MDDR calculation.

Build and link Blinky, TimerDemo, UartPrbsTxTest, PulseTrain, UartRetargetDemo,
I2CMasterDemo and SPIMasterDemo for Cortex-M0+ with both normal linker scripts,
then inspect the resulting ELF sections and option bytes:

```sh
python3 tests/re01/build_examples.py
python3 tests/re01/build_examples.py --all-timers
```

Use `--tool-prefix /path/to/bin/arm-none-eabi-` if the compiler is not on PATH.
`--package DBN`, `--package CFB` or `--package CFP` selects the package define.
The check requires vectors below 0x400, 64 erased option bytes at 0x400,
code/data load addresses at or above 0x440, and heap/stack limits inside RAM.
`--timer-devno 0` through `--timer-devno 8` selects a TimerDemo device; the
default is GPT0 (3). `--all-timers` builds all nine selections. The script
resolves source and board paths from the actual example project links and uses
GNU C++23 for C++ sources. Console builds include the real UART retarget and
stdio device adapters.
These are direct compiler/linker checks of the example sources, not an
IOcomposer or Eclipse project build. The script uses newlib-nano and nosys.

The original example set was validated with GCC 14.3.Rel1 for all three
package defines: 90 image checks covered the seven examples, all nine TimerDemo
selections and both normal linker scripts. After linking the existing shared
bus demos, 24 I2CMasterDemo/SPIMasterDemo checks passed across Debug/Release
defines, all three packages and both normal linker scripts. Existing shared
`ARM/src/iatomic.c` builtin declaration warnings, I2C demo switch/unused-variable
warnings and UART PRBS unused-variable warnings remain.

The host test does not emulate FIFO side effects, oscillator stabilization, voltage
transitions, asynchronous AGT stop acknowledgement, interrupt timing, or flash
programming. Board validation is still required for normal/boost startup, PLL
fallback, SCI loopback and burst traffic, GPIO wake, timer rate/reset/resume,
and simultaneous AGT compare events. SPI/I2C checks are described below.

## General-purpose timers

The RE01 Timer port exposes nine devices through the existing C handle and
C++ wrapper. The original device numbers are preserved:

| DevNo | Hardware | Counter width | Compare triggers |
| --- | --- | --- | --- |
| 0 | AGT0 | 16 bits | 2 |
| 1 | AGT1 | 16 bits | 1 |
| 2 | TMR0/1 in cascade | 16 bits | 2 |
| 3 | GPT0 (GPT320) | 32 bits | 4 |
| 4 | GPT1 (GPT321) | 32 bits | 4 |
| 5 | GPT2 (GPT162) | 16 bits | 4 |
| 6 | GPT3 (GPT163) | 16 bits | 4 |
| 7 | GPT4 (GPT164) | 16 bits | 4 |
| 8 | GPT5 (GPT165) | 16 bits | 4 |

`TimerGetLowFreqDevCount()` returns 2, `TimerGetHighFreqDevCount()` returns 7,
and `TimerGetHighFreqDevNo()` returns 2. GPT compare triggers use A/B/C/D
with buffering disabled; E/F are not exposed because they have no independent
ICU interrupt events. GPT16 and GPT32 use 32-bit register accesses. Starting,
stopping or resetting a GPT channel does not change another channel or disable
their shared module clock. The library project includes the new GPT source.

AGT selects LOCO (LFRC), the subclock (LFXTAL), or PCLKB (HF/default fallback),
and divides by powers of two from 1 to 128. TMR uses the current PCLKB with
divisors 1/2/8/32/64/1024/8192. GPT uses PCLKA, equal to ICLK, with divisors
1/4/16/64/256/1024. These drivers do not switch the system oscillator or start
an unconfigured oscillator; the application must supply a running source.
Frequency requests select the nearest representable integer rate, and zero
selects the maximum. Rates below the slowest divider clamp to that rate.
Reinitialize the timer after changing the system/peripheral clocks.

Reset clears the extended count, peripheral/ICU pending flags and compare
deadlines while preserving running/disabled state. Disable/Enable retains the
counter, clock selection and triggers. Frequency changes reset and restart the
counter, recalculate active compare periods, and fail without changing the
configuration if an active period no longer fits. Trigger periods must round
to 3 through 65535 ticks for 16-bit counters, or 3 through 4294967295 ticks
for GPT32. Check the actual returned period; zero means the request failed.

One-shot completion releases its ICU route before the callback, permitting
immediate rearming. Continuous triggers advance from the previous deadline
and skip missed periods, rather than adding ISR latency to every period.
Callbacks run in interrupt context and must be short. Every initialized timer
needs an overflow route and each active compare needs another route; allocation
can fail because the ICU has only 32 lines with event-specific routing limits.
A failed allocation leaves existing trigger configuration intact. A timer has
one static owner; the same handle can reinitialize that device but cannot move
to another device while it owns the first. Keep initialized handles alive.

Extended counts account for one pending wrap before the overflow ISR runs,
including reads inside overflow callbacks. Interrupts must be serviced within
one full counter cycle; multiple unserviced wraps cannot be reconstructed.
Trigger periods must also allow enough time to service and rearm the compare.
AGT retains the maintainer's stop/update sequence and simultaneous-comparator
hardware caution: use compare B at a frequency no higher than compare A.
Its stop wait is bounded, and a failed stop in the compare ISR releases both
compare routes rather than leaking them.

Raw per-tick interrupts (`bTickInt = true`), external counter clocks and external
trigger inputs are unsupported and fail closed. Use compare triggers for
periodic callbacks. RTC, watchdogs, the low-speed clock timer (LST), low-speed
pulse generator (LPG) and clock correction circuit (CCC) are separate hardware
functions, outside this general-purpose Timer list. SysTick retains its
separate `TimerSysTick` implementation.

Register definitions, write-protection key, count direction, prescaler
encodings and buffer controls were checked against Renesas's official
[RE01 SVD](https://github.com/renesas/re-driver-package/blob/d67d8f1410421e33923e65a776add5401fbb11b8/SDK_RE01_1500KB/RE01_1500KB_DFP/SVD/RE01_1500KB.svd)
and generated device header. The SVD calls the GPT input clock PCLKD; this
RE01 device has PCLKA/PCLKB. The Renesas
[datasheet](https://docs.rs-online.com/5a08/A700000007228313.pdf),
R01DS0363EJ0110, section 6.3.6 identifies PCLKA for GPT timing; section 5
identifies PCLKB for GPT register access. The full hardware manual could not
be retrieved during this review.

Host regression checks exercise all nine devices, each GPT divider and A-D
event route, counter widths, pending wraps and callback reads, reset/resume,
one-shot release/rearming, continuous phase and missed periods, simultaneous
AGT flags, frequency changes, ownership, invalid requests, exhausted ICU routes
and the bounded AGT stop failure. Cortex-M0+ compile/link/layout checks cover
the examples with both linker scripts for DBN/CFB/CFP. These checks do not
validate GPT/AGT clock-domain timing, actual interrupt latency, low-power
operation or the reported AGT comparator hardware issue. Validate counter
rates, wrap/reset/resume and simultaneous compare callbacks on the board.

## SPI and I2C polling masters

The RE01 port now provides SPI0/1 and RIIC0/1 through the existing C handles,
C++ wrappers and `DeviceIntrf` function tables. The framework retains ownership
of the transfer busy flag and enable reference count. Each controller has one
static owner; another handle cannot claim it until `PowerOff()` releases it.
No shared interface API or application pin map is changed.

Set both `bIntEn` and `bDmaEn` to `false`. Unsupported configurations fail
initialization: SPI slave, multiplexed 3-wire, quad/octal, frames below 8 or
above 16 bits; I2C slave, SMBus, 10-bit addresses and requests above 400 kHz;
and interrupt or DMA operation on either bus. Polling callbacks are not issued.
The RE01 I2CMasterDemo and SPIMasterDemo projects link the existing shared
`i2c_master_demo.cpp` and `spi_master_demo.cpp` applications. All six RE01 I2C
configurations define `I2C_MASTER_DMA_ENABLE=false` and
`I2C_MASTER_INT_ENABLE=false`; the shared SPI example already selects polling.
Their wiring, peer protocol, console settings and project configurations are
documented in `ARM/Renesas/RE01/RE01_1500KB/exemples/README.md`.

SPI supports all four clock modes, MSB/LSB order, multiple active-low GPIO
chip selects (`SPICSEL_AUTO`), and application-controlled selects
(`SPICSEL_MAN`). An automatic select stays asserted across a command/read or
command/write transfer. Normal SPI requires SCK, MISO and MOSI pin entries,
followed by GPIO output entries for automatic selects. For 9-16-bit frames,
buffer lengths must be even; each frame occupies two little-endian bytes,
and unaligned buffers are supported. An odd command length aborts the entire
read. Receive clocks transmit `DummyByte`, repeated in both bytes for wide
frames, and TX always drains RX to avoid overrun. `FirstRdData` retains the
first received frame. Clock calculation uses ICLK/PCLKA, including the
special SPB=4 encoding for 8-bit frames and the proper byte access to SPDR.

I2C requires two distinct SDA/SCL peripheral pin entries of type
`IOPINTYPE_OPENDRAIN`, with external pull-ups (`IOPINRES_NONE`) or internal
pull-ups. P-channel/pull-down configurations are rejected. Register reads
use repeated START, and 1-, 2-, 3- and longer reads ACK intermediate bytes,
NACK the last byte and request STOP before its final register read. Timing
uses PCLKB and the two-stage digital filter; the selected low/high periods
meet Standard/Fast mode minima and the nominal clock never exceeds the
request. Reported rate excludes board rise/fall times and clock stretching.
Both rate setters return zero without changing configuration when a request
cannot be represented or a transfer is open. Reapply the rate after changing
the system/peripheral clock while the bus is idle.

Transfers return the number of bytes actually received or acknowledged.
Errors and finite polling timeouts release/reset the local engine; SPI
releases CS and I2C tries STOP if it still owns the bus. Arbitration loss never
requests STOP or toggles GPIO. A failed I2C command prefix cannot proceed to
the read phase. A STOP timeout resets the local engine; the returned byte
count still describes data already transferred, not bus-clear success.
Polling budgets are loop counts with at least a 50 ms allowance per wait,
not a precise wall-clock deadline.

For an explicit I2C bus clear, use the inherited `Reset()` operation (C:
`DeviceIntrfReset(&dev.DevIntrf)`). It uses open-drain GPIO, waits for SCL to
rise, emits up to nine recovery clocks, makes STOP when SCL is released,
restores the caller's pin functions and preserves enabled/disabled state.
Only request GPIO bus clear when this application controls bus recovery;
it can disturb another master. The generic `I2CBusReset()` helper remains
unchanged and uses push-pull GPIO; use the target reset operation for a
bus with clock stretching. RE01 pin configuration now preserves the output
latch so preloading CS/SDA/SCL high survives pin-mux changes.

Cross-build and exercise the actual Arm instructions with a serial register
model (the same tool-prefix/package options apply):

```sh
python3 -m pip install pyelftools unicorn
python3 tests/re01/build_serial.py
```

The model independently checks clock equations and timing minima, SPI access
widths and 8-16-bit frames in every mode, multiple/manual CS, command/read and
command/write transactions, RX buffer guards, controller ownership, generic
busy/reference counts, rate changes, unsupported modes and C++ base-class
dispatch. RIIC checks cover both controllers, read lengths 1/2/3/4/17,
dummy-read/WAIT/protected-ACK sequencing, repeated START and final-byte STOP.
Fault injection covers SPI overrun and stalled receive, I2C address/data NACK,
arbitration loss, stretched/stalled receive, busy bus and stalled STOP, followed
by successful retries. GPIO checks include PFS/output-latch aliases, external
pull-ups and explicit bus clear with SCL held low. All three package builds
and model runs pass with GCC 14.3.Rel1. Synthetic fixture pins are for the
model only; package bonding and valid peripheral pin functions belong in
the application's `board.h`.

Register sequences follow the official Renesas
[SPI source](https://github.com/renesas/re-driver-package/blob/d67d8f1410421e33923e65a776add5401fbb11b8/SDK_RE01_1500KB/RE01_1500KB_DFP/Device/Driver/Src/r_spi/r_spi_cmsis_api.c)
and
[I2C source](https://github.com/renesas/re-driver-package/blob/d67d8f1410421e33923e65a776add5401fbb11b8/SDK_RE01_1500KB/RE01_1500KB_DFP/Device/Driver/Src/r_i2c/r_i2c_cmsis_api.c).
The model checks register behavior, not analog timing, oscillator startup,
interrupt latency or actual devices. Hardware validation remains necessary:
SPI loopback/logic-analyzer captures for each mode/width and both controllers,
I2C 100/400 kHz captures with real pull-ups, short reads/repeated START, NACK,
clock stretching, recovery and concurrent-master arbitration.

## DFU

Build the real stage-0 boot example and a Blinky application with the DFU
linker scripts, then run the boot and target flash routines in Unicorn:

```sh
python3 -m pip install cryptography pyelftools unicorn
python3 tests/re01/build_dfu.py
```

The same `--tool-prefix` and `--package` options apply. `--build-only` skips
emulation. The script supplies an ephemeral P-256 public key and a 32 MHz
RC oscillator configuration to the boot example, independently signs an
MCUboot-format fixture, and verifies option memory, boot/application load
addresses, reserved recovery RAM, and placement of flash routines in RAM.
GCC 14.3.Rel1 builds both images for DBN, CFB and CFP (six images). Shared
atomic builtin warnings and the boot linker's RAM code RWX warning remain.

The 11 emulator scenarios cover empty, valid, tampered and legacy records;
payload writes from flash and unaligned RAM; program errors and retries;
all 384 erase-block addresses, neighbouring block isolation and all-ones
erase; address/length limits; exact clock bounds and MHz rounding; restoration
of protection, flash interrupts, VTOR and PRIMASK; DBFULL/command timeouts;
and RAM reset when forced stop or return to read mode fails. The model rejects
flash fetches/reads during P/E mode and checks RAM NMI/HardFault vectors.
SystemInit is skipped; this is a command model, not hardware validation of
flash timing, voltage, startup or asynchronous exception delivery.

Flash geometry and commands follow Renesas's official
[RE01 driver package](https://github.com/renesas/re-driver-package/tree/d67d8f1410421e33923e65a776add5401fbb11b8/SDK_RE01_1500KB/RE01_1500KB_DFP/Device/Driver):
`Include/r_flash_re01_1500kb.h` defines 384 uniform 4 KB blocks and 8/256-byte
programming; `Src/r_flash/r_flash_lowlevel.c` defines the FACI address,
command sequence and 1-32 MHz ICLK requirement. The accompanying driver
specification is R01AN4768EJ0103, revision 1.03. DFU uses 256-byte programming
and retains the existing boot, record and application addresses.

A DFU boot project must select ICLK between 1 and 32 MHz, for example by
overriding `g_McuOsc` with a 32 MHz RC configuration and `bUSBClk = false`.
The port's weak default is 48 MHz; flash erase/write calls reject that clock.
The target does not switch clocks during a flash operation.

The record magic now occupies 256 bytes (`__dfu_state_unit = 256`). Records
made with the previous 128-byte assumption are refused. After installing the
updated boot, reupload the application or regenerate its factory record with
the new state unit. Keep the recovery path available during this migration.
