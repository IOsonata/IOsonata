# MCU core power and clock ports

## Port rules status

State of each port against "Rules for an MCU port" in
[MCU core power, clock and timer lifecycle](system-clock-timer-lifecycle.md),
November 25, 2026.

| Port | No run time source selection | Clock code reads no device or driver state | `SystemSleep` leaves the drivers alone | Wake source set through the device driver |
| --- | --- | --- | --- | --- |
| SAM D11, SAM D21 | Yes | Yes | Yes | Yes |
| SAM4L | Yes, not run on hardware | Yes, not run on hardware | Yes, not run on hardware | No |
| SAM4E | No | No | Not reviewed | No |
| nRF52, nRF5340 App/Net, nRF54L15/LM20A/LM20B, nRF9160/91x1 | Yes | Yes | Yes | Yes |
| ST | No | Not reviewed | Not reviewed | No |
| NXP | No | Not reviewed | Not reviewed | No |
| Renesas | No | Not reviewed | Not reviewed | No |
| Espressif | No | Not reviewed | No | No |

The last column: the remaining older ports still define `SystemWakePin`, which is no
longer an application function. `CorePowerTest` enables the wake button
with `IOPinAllocateInterrupt`. Until the pin driver of a port makes its pin
interrupt wake the MCU, the button wake of `CorePowerTest` is not expected
to work on that port, and the `SystemWakePin` text in this document and in
the port documents describes code to be removed. On SAM D the
pin driver requests the low frequency clock for the EIC, so an enabled pin
interrupt wakes the MCU from Standby and `SystemSleep` calls no driver.

The Nordic clock code takes the sources from the board oscillator descriptor
when it starts an oscillator. The peripheral clock is the fixed 16 MHz
PCLK16M, there is no clock tree to configure. GPIO interrupt enable configures
GPIOTE and pin SENSE. Interrupt disable releases the pin. System ON sleep uses
the configured interrupt, and System OFF uses GPIO DETECT. The nRF5340
network core has no independent System OFF. Radio stack clock ownership,
modem shutdown checks, and network-core LFRC fallback still apply.

On the nRF52840 DK, CoreClockTest completed twice with zero failures.
CorePowerTest passed 134 checks across four boots. All 12 RTC Standby wakes
and 12 button Standby wakes preserved the CPU and peripheral clocks, UART
rate, and static and stack RAM. All three System OFF wakes reported reset
reason 0x00000100 and preserved the 16-byte RAM test pattern.

On the nRF52832 BLYST-NANO-DK, CoreClockTest completed twice with zero
failures. Two complete CorePowerTest runs passed 66 checks. All six RTC
Standby wakes and six button Standby wakes passed the clock, UART rate and
RAM checks. The System OFF wake reported reset reason 0x00000100 and
preserved the 16-byte RAM test pattern.

### Nordic hardware runs on the refactored branch

These runs used the Nordic port as merged in #150. After them, the startup
write of the low frequency source and the QSPI clock as peripheral clock 1
were removed: the source is written when LFCLK starts and the QSPI driver
sets its HFCLK192M divider, as before #150. Those two paths were not run
again on hardware.

| Target | CoreClockTest | CorePowerTest |
| --- | --- | --- |
| nRF5340 Audio DK, application core | Passed at 128 MHz and 64 MHz. | 38 checks passed after System OFF wake. The retained 32 KiB kept its pattern. None of the 8,192 words in the non-retained section kept theirs. |
| nRF5340 Audio DK, network core | Passed at 64 MHz. LF and HF timer requests returned their reference counts to the initial values. | 16 checks passed, including three timer wakes from Standby. The network core has no independent System OFF. |
| IBK-NRF54L15 | Passed at 128 MHz. | 36 checks passed after a System OFF pin wake. Retention and non-retention passed for separate 32 KiB regions. Two earlier restarts reported a debug reset (0x20). |
| nRF54LM20A DK | Passed at 128 MHz. | Three System OFF pin wakes reported 0x100. A complete post-wake summary showed 36 checks passed, including retention and non-retention. |
| nRF9160 DK | Passed at 64 MHz. | 32 checks passed at startup and 34 after each of three System OFF pin wakes. The 8 KiB retained region kept its pattern each time. |
| nRF9151 DK | Passed at 64 MHz with the DK pin map. | Three System OFF pin wakes reported 0x100. The post-wake run passed 34 checks including 8 KiB RAM retention. In a separate run following a software reset, the modem-power guard refused System OFF. |

The nRF54LM20B library, CoreClockTest and CorePowerTest built in Debug and
Release with zero errors and warnings. The LM20A and LM20B share the same core. The LM20A DK runs cover the
shared core power and clock implementation for both build targets.

During the SAM D refactor, `system_samd.c` went from 4,466 to
2,366 bytes at `-Os`, and the SAM D11 `CoreClockTest` Release image from
14,960 to 11,684 bytes of flash on a 16 KB part. The removed code was the
search for clock users, the run time divider setter, the frequency rebuilt
from the registers, the run time source selection and the driver calls of
`SystemSleep`. As of November 25, 2026 these SAM D results are from host
tests and a register model; the hardware run is not done.

## Microchip 0.14 port status, November 24

The SAM D11/D21, SAM4L and SAM4E ports report power capabilities and
implement the shared HF/LF clock requests. Timers hold their clock while
running. The system keeps the root clocks available for CPU and peripheral
users after the last explicit request. Runtime oscillator calibration returns
false on these ports.

| Family | Resumable sleep and seconds delay | System OFF | Running CPU mode |
| --- | --- | --- | --- |
| SAM D11/D21 | Standby with RTC or configured EIC wake. The seconds delay borrows an unassigned RTC. | Unavailable | Divides the CPU and APB clocks by two. Peripheral generic-clock frequencies are preserved. |
| SAM4L | RETENTION. Wake by an EIC pin, the AST timer or the UART receive wake armed with `UARTEnableWake`. The seconds delay borrows an unassigned AST. | Backup with wake pins and backup registers | Changes synchronous dividers together. Active peripherals can refuse the change. |
| SAM4E | CPU sleep keeps TC and MCK running for interrupt wake. A nonzero seconds delay is unavailable because the Timer driver exposes TC channels only. | Backup with wake pins and backup registers | Changes the MCK prescaler when peripheral clock users permit it. |

Arm GCC linked CoreClockTest and CorePowerTest in Debug and Release for
SAMD21, SAM4LCxC, SAM4LSxC and SAM4E16E using the project sources and
linker scripts. The SAMD11D14AM Release CoreClockTest fits with the board's timer and
delay tests disabled. Its full CoreClockTest and CorePowerTest exceed flash.
Register models cover clock requests, timer power-down and shared-clock
preservation. Board verification of the new running modes, delay accuracy
and wake behavior remains open.


## SAM4L implementation and board evidence, November 17

The SAM4L generic power/clock entry points are implemented for SAM4LC and
SAM4LS. This states source coverage, not validation of every hardware mode.

| Operation | Implementation |
| --- | --- |
| Core source | RC1M, RCFAST 4/8/12 MHz through DFLL; RC80M; OSC0 crystal/bypass with PLL where a valid ratio exists |
| Low-frequency source | RC32K, 32768 Hz crystal, external clock; active consumers prevent unsafe changes |
| Peripheral clocks | PBA/PBB/PBC/PBD dividers; PBB remains equal to CPU/HSB |
| Delay factors | Refreshed after core clock changes for both delay loops |
| SystemOscInit | Linkable no-op, matching SAMD; SystemInit/selectors perform configuration and crystal load capacitance is not programmable |
| SystemSleep | SLEEP0 for ordinary IRQs, RETENTION for asynchronous EIC/AST, WAIT for configured PM.AWEN peripheral sources |
| SystemPowerOff | Backup, exited by reset; EIC and AST wake routes supported |
| Reset/retention | Reset and wake reason decoding; four BSCIF backup registers; SRAM retention in Backup is physically unavailable |
| Watchdog | Disabled at startup; applications own enabling it |

AST previously enabled IMR without WER. The timer driver now synchronizes WER
writes, enables overflow/alarm wake while running, removes wake requests when
stopped, and clears wake flags through SCR. Single-shot completion disables the
alarm wake before delivering the callback. Setup clears retained wake state
and enables the BPM AST Backup route while preserving other routes. The
32000 Hz RC32K request alias now reports its actual nominal 32768 Hz source.

The maintainer's latest SAM4LC CoreClockTest run ended with zero failures: RC4/RC12,
PBA divisors 1/2/4/1, both delay functions, 25 timer interrupts per interval,
six OLED1 BUTTON1 RETENTION wakes, CPU/APB/LF descriptors, UART RX and APB
restoration passed. The 250 ms delays were approximately 251194..251600 us.
Earlier eight consecutive Backup pin wakes advanced the retained count 1..9.
ClockDisplayDemo and every interrupt/DMA combination of LCDConsoleDisplayDemo
also passed on this board.

Validation for this update: all 124 SAM4LC and 128 SAM4LS library sources
compiled in Debug and Release, followed by relinks after the final edits.
Sixteen SAM4L demo links passed: CoreClockTest, button/timer CorePowerTest on both
families and both display demos on SAM4LC. The edited C++ sources compile with
GNU++11. All fifteen public clock/power functions have one definition in each
archive. Existing timer, clock and power register tests passed. Shared demo
rebuilds also passed on D11, D21, C3 and C6.

The subsequent CoreClockTest AST run ended with zero failures. The visible automatic
wake intervals were 1000000 us, with timer wake interrupts, restored clocks,
both delay functions and UART RX passing. CorePowerTest then completed four
consecutive AST Backup wakes: reset reason 0x1000 and retained counts 1 through 5.

TWIS0/TWIS1 now configure their PM clock and asynchronous wake requests through
the I2C slave enable/disable lifecycle. The existing I2CSlaveDemo and D21
I2CMasterDemo have a selectable address-match sleep profile; see
[the test procedure](../../exemples/i2c/sleepwalking.md). Hardware execution is
deferred. This implements TWIS SleepWalking, not ADC/PEVC or other autonomous
peripheral paths. External oscillators, SAM4LS hardware and sleep currents
remain unmeasured.

## November 17 implementation review

SAMD11, SAMD21, SAM4L, ESP32-C3 and ESP32-C6 are not marked fully complete.
Earlier demo passes establish the configurations exercised, not every clock
source, divider or power mode. The remaining items below must not be hidden by
a successful build or by reclassifying a missing implementation as unsupported
hardware.

This revision adds:

- SAMD11/21 generic-clock divider selection, including each generator's divider
  width, binary division, active-consumer checks and CPU delay-factor updates.
  CoreClockTest now selects OSC8M again on D11 and checks 1/2/4/1 dividers, timer
  interrupts and both delay functions at each rate on both targets.
- SAM4L peripheral-clock setters for PBA/PBB/PBC/PBD. Rates cannot exceed the
  CPU rate; PBB must equal the CPU rate because it also supplies AHB modules.
  CoreClockTest checks the PBA dividers with the UART stopped and reinitialized.
- SAM4L SystemSleep selects RETENTION for asynchronous EIC/AST wake, WAIT for
  enabled PM SleepWalking wake sources, and SLEEP0 for ordinary interrupt wake.
  AST wake detection uses WER, independently of the interrupt mask. The caller
  still owns configuring and clearing the peripheral wake event. BPM, SCR,
  clock gates, PM interrupt ownership and the caller's interrupt mask are kept.
- C3 RTC fast SRAM (8 KB) and C6 LP SRAM (16 KB) retention requests, with a
  `.rtc.noinit` linker section excluded from normal startup initialization.
  HP SRAM retention and independently powering off RTC/LP SRAM are not claimed.
  ROM storage words remain reserved; they are not application retention registers.
- C3/C6 CorePowerTest checks retained data after pin wake, using the region above
  the first 1 KB reserved for wake-stub use. This demo region must not also hold
  application retained objects or LP firmware.
- C6 RC32K selection is removed following Espressif advisory AR2024-011. RC
  selection uses RC_SLOW. Sleep keeps the actual selected LP clock powered,
  using the hardware mux rather than inferring it from a frequency descriptor.

CoreClockTest now prints its small set of integer/string messages directly through
UART; LCDConsoleDisplayDemo remains the display/syslog demonstration. The
expanded D11 CoreClockTest uses 13,364 bytes of flash and 1,208 bytes of BSS in the
current size-optimized LTO build. Sleep checks on D11, D21 and SAM4LC now use
a peripheral-clock divisor of four and restore the original rate afterward.

Debug and Release library-source builds and CoreClockTest/CorePowerTest links passed
for D11, D21, SAM4LC, SAM4LS, C3 and C6. Both display demos also linked for D11
and D21. These are GNU command-line builds derived from the managed project
source lists; no new board execution was performed here.

Hardware validation still required for this revision:

- D11/D21 changed dividers, actual D11 source selection, and sleep at reduced
  clock rates, including UART receive after resume.
- SAM4LC TWIS WAIT/SleepWalking. AST, PBA dividers and OLED1 RETENTION
  wake have the passing board results recorded above.
- C3/C6 retained SRAM across repeated Deep-sleep pin wakes.
- Sleep-current measurements and external oscillator configurations.

The subsequent C3/C6 PLL implementation extends the existing 160 MHz build
option. Define ESP32C3_CPU_160MHZ or ESP32C6_CPU_160MHZ in both the library and
CoreClockTest compiler settings, then clean rebuild both. Startup initializes PLL
when required, and SystemCoreClockSelect(OSC_TYPE_XTAL, 40000000) returns to its
160 MHz CPU output after RC_FAST. Without the option, XTAL still selects 40 MHz.
The generic oscillator API is unchanged; it is not an arbitrary CPU-rate setter.

PLL configuration and settling execute from instruction RAM. C3 uses its ROM
analog-I2C routines and checks PLL lock; C6 uses the vendor analog-I2C workaround
and checks calibration completion. Failed setup keeps the previous CPU source.
An already running PLL is reused. C6 limits the MSPI PLL input to 80 MHz and
keeps AHB at 40 MHz when switching from a direct source. Application peripherals
must be stopped before runtime source changes, as in the direct-clock profile.
C3 APB is fixed at 80 MHz under PLL, so that profile does not request fractional
APB rates; the direct profile exercises the shared CPU/APB divider.

Default and 160 MHz profiles compile and link in Debug and Release. PLL startup,
RC-to-PLL return and sleep/resume at 160 MHz still require board execution.
Earlier sections describing PLL startup as unimplemented refer to prior code.
These additions are not a claim of complete hardware validation.

The maintainer also confirmed SAM4LC LCDConsoleDisplayDemo in all four
interrupt/DMA combinations and ClockDisplayDemo. Those results apply to the
existing display configuration and do not validate the new sleep modes.


The public functions are in `include/coredev/system_core_clock.h`.
Implementations belong to the MCU library. Applications use the same functions
and `exemples/misc/core_power_test.cpp`, with board pins in `board.h`.

Power off means a mode exited by reset, not an ordinary interrupt-resumable
sleep. Unsupported retention ranges and wake pins return false. An implemented
operation must not return success without programming or checking the hardware.
Clock selection follows the MCU clock tree. The Nordic HFXO request count is
appropriate for its on-demand crystal; it must not stop a clock supplying another
MCU's CPU, PLL, USB, or peripheral.

## Source coverage

This table records source implementation, not hardware validation or completion
of every clock mode. Existing clock code still needs a family-by-family review.

| Family | Generic power functions | Clock work remaining |
| --- | --- | --- |
| nRF52 | Existing dedicated implementation | Preserve SoftDevice/MPSL ownership and tested HFXO requests |
| nRF53, nRF54L, nRF91 | Existing shared implementation, with per-core restrictions | Verify per-core and SDK configurations |
| STM32 F0/F4/L1/L4/G4/H7 | Existing shared implementation | Audit selectors, ready waits and active clock users |
| STM32 WBA | WBA65I Standby, GPIO wake, reset reason, backup registers and bank-wide SRAM retention on working branch | Validate radio ownership, SRAM bank retention and target build |
| SAM4E | Reset, Backup mode, wake pins, GPBR; no SRAM retention in Backup | Slow-clock and main-clock/PLLA selection implemented; hardware validation pending |
| SAM4L | Reset, Backup mode, EIC wake pins, four backup registers; SLEEP0/WAIT/RETENTION resume implemented | SAM4LC clock/delay checks and eight consecutive Backup wakes passed; three Release SLEEP0 cycles passed with clocks, delays, timer IRQs and UART RX; six EIC RETENTION cycles and PBA dividers passed; AST RETENTION and four Backup timer wakes passed; TWIS WAIT hardware validation deferred |
| SAM D11/D21 | PM reset reasons and interrupt-resumable Standby; D11 SW0 and D21 SW0/OLED1 BUTTON1 repeated resume passed | D21 clocks, delays, timer IRQs, UART RX and SRAM samples passed; full SRAM continuity, LF switching and sleep current remain unverified |
| LPC11Uxx | Deep power-down, fixed wake pin, four retention words, reset causes and ordinary Sleep implemented | Core/LF selectors and enabled peripheral checks compiled; hardware pending |
| LPC17xx | Deep Power-down, reset reasons, five backup words and ordinary Sleep implemented | Runtime core/USB clock checks compiled; fixed RTC source has no selector; hardware pending |
| LPC546xx | Deep power-down, ordinary Sleep, reset causes and seven RTC backup words implemented | Runtime peripheral/PLL ownership and clock decoding checked; hardware pending |
| RA4M1 | Reset reporting, external-IRQ Software Standby and ordinary Sleep; no reset-on-wake power-off mode | Runtime module/CLKOUT checks added; protected clock models and firmware builds passed; hardware pending |
| R9A02G021 | Reset reporting, wake pins and resumable Sleep/Software Standby; no reset-on-wake power off | Clock supply type A supports reduced CPU speed. Timer and GPIO wake require board validation |
| RE01 | Reset reporting, external-IRQ Software Standby, ordinary Sleep and reset-on-wake Deep Software Standby | Startup oscillator setup, CPU divider modes and shared timer clocks implemented. Hardware validation pending |
| ESP32 C-series | C3 RTC and C6 PMU Light-sleep/Deep-sleep implemented; C6 repeated GPIO9 resume and GPIO3 reset-on-wake passed with UART output drained and Disable() called; C3 repeated Light-sleep and GPIO3 Deep-sleep wake passed | LF calibration, direct XTAL/RC CPU selection and APB rates implemented; C3/C6 combined sleep/clock checks passed; current measurements pending; optional 160 MHz PLL profile and RTC/LP SRAM retention implemented, hardware validation pending |
| nRF54H, Raspberry Pi ports | No generic power implementation found | Review port maturity and vendor startup integration first |

## SAM D11/D21 power and clock port (0.14)

SAM D is the first port converted to the port rules of
[system-clock-timer-lifecycle.md](system-clock-timer-lifecycle.md).

### Design, November 25, 2026

- The clock sources are fixed by `g_McuOsc` at startup. There is no run time
  source selection. `g_McuOsc` is not range checked: with a wrong setting
  the oscillator or the PLL does not start and the main clock stays on
  OSC8M. The 32 kHz oscillator selected for generator 1 is set to run in
  Standby when it is started, since the RTC and the EIC, the Standby wake
  sources, are clocked from it. The clock code stores the frequencies of generators 0
  and 1 when it sets them and reads no device or driver state.
- `SystemHighFreqClockEnable/Disable` and `SystemLowFreqClockEnable/Disable`
  count the users of a shared clock, so that one user's disable does not
  stop a clock another user needs. The counts refuse nothing.
- `SystemPeriphClockGet(0)` is the main generator and `(1)` the 32 kHz
  generator. `SystemPeriphClockSet` returns the current frequency: the
  dividers are not changed at run time.
- `SystemPowerModeSet` changes the CPU and bus dividers. The caller handles
  the device timing that depends on them, a SysTick on the CPU clock
  included.
- `SystemSleep` enters Standby and returns after an interrupt. It calls no
  device driver. The drivers have no sleep, pause or resume functions.
- A wake source is a device interrupt. A pin interrupt enabled with
  `IOPinEnableInterrupt` or `IOPinAllocateInterrupt` wakes the MCU: the EIC
  requests the low frequency clock with its first pin interrupt or event
  and releases it with the last one, and that clock runs in Standby. Pin
  edges are sampled at 32 kHz, the timer external trigger through an EIC
  event included. On D11, PA14 is the NMI input; its pin interrupt uses the
  NMI, whose handler sets the EIC interrupt pending.
- `SystemHighFreqClockCalibrate` and `SystemLowFreqClockCalibrate` return
  false: the port has no run time RC calibration.
- `SystemResetReason` reads PM RCAUSE. `SystemPowerOff` is not supported.

Sizes at -Os with Arm GNU 14.2.1, code and initialized data: the clock file
2,114 bytes, the sleep file 60, the pin driver 1,853 on D11 and 1,858 on D21.
Linked with the managed project options, D11 `CorePowerTest` uses 16,024 of
16,384 flash bytes and D11 `CoreClockTest` 15,928.

### Hardware, November 25, 2026

SAM D11 Xplained Pro: `CorePowerTest` passed 30 checks with no failure,
three Standby wakes from the RTC and three from SW0 (PA14), with the CPU and
peripheral clocks, the UART rate and the RAM samples checked after every
wake. `CoreClockTest` passed with no failure, with and without the delay
calibration; the measured delay loop cost agreed at 48 MHz and 24 MHz within
0.04 percent.

SAM D21 Xplained Pro: `CoreClockTest` passed with no failure. With the
calibration timer at 250 kHz the delay loop cost matches the D11, 13.0 and
3.0 CPU cycles per pass. `CorePowerTest` passed 30 checks with no failure,
three Standby wakes from the RTC and three from the button through its EIC
edge interrupt.

After the clock file was reduced to 2,114 bytes (the 32 kHz oscillator set to
run in Standby at startup, no range check of `g_McuOsc`), `CorePowerTest`
was run again on the D21 and on the D11 and passed the same 30 checks on
both.

Not run on hardware: Standby current with the EIC on
the low frequency clock, and the timer external trigger at the 32 kHz
sampling.

### Timer

Timer Enable refuses a source frequency different from its cached BaseFreq.
After the source frequency changed, call Frequency to select a rate using the
new source. This existing operation resets the counter, recalculates active trigger
periods and starts it. A missing source or failed reconfiguration returns zero.
A stop, setup, reset or start failure invalidates the cached source frequency,
so Enable refuses partial configuration even when the source is unchanged.
A successful Frequency retry restores the cached value. This does not claim
that hardware stopped when a stop synchronization failed. The final RTC read
synchronization after start or stop is also checked: failure suppresses timer
interrupt dispatch, invalidates the saved frequency and requires reconfiguration.
The period calculation uses an aligned local value before assigning the packed
64-bit trigger field. An unchanged source allows normal Disable/Enable resume.

### Earlier hardware results

These runs predate the November 25 change. They used the earlier code, with
run time clock selection and level sense wake, both since removed. They are
kept as a record of the reset cause, Standby, RAM and delay results.

Maintainer-reported reset-cause tests passed 128/128 combinations for
each MCU, and Arm GNU source compilation passed for D11/D21 including
the Standby source. On November 15, 2026, the maintainer reported five
consecutive SAMD11 SW0/PA14 Standby resumes, with counts 0x00000001 through
0x00000005 and another Standby entry. Subsequent D11 runs also passed the
128-byte static and 128-byte stack SRAM samples across five wakeups. This
is a sample of live RAM continuity, not a full SRAM test. SAMD21 subsequently
passed five Standby wakes with the same static and stack samples. Full SRAM
testing and sleep-current measurements remain pending.

On November 17, SAMD21 CoreClockTest passed three Standby cycles with SW0/PA15
and another three with OLED1 BUTTON1/PB04 on EXT1. CPU/APB reporting remained
48 MHz; the LF descriptor remained 32768 Hz. Both 250 ms delay checks measured
about 251 ms at the actual 750 kHz timer rate, with 25 interrupts. UART RX
passed before sleep and after every wake. LF switching was skipped.
ClockDisplayDemo output worked, and LCDConsoleDisplayDemo display syslog
passed all four combinations of SPI interrupt and DMA settings. These results
do not establish other CPU/APB rates or physical oscillator accuracy.

On November 17, 2026, CoreClockTest at `2de35f281` passed three OSC8M-reference
selections at 48 MHz. The shared fixed-point delay implementation measured
250094 us for msDelay(250) and 250107 us for nsDelay(250000000), with 25
interrupts each and zero failures. Short nanosecond-delay accuracy is not
established by this long-interval check. CoreClockTest now provides a selectable
SW0 Standby run that repeats clock, delay and timer checks after three wakes;
that combined run at `e017243bc` passed all three wakes with zero failures.
CPU/APB and LF descriptor checks passed. Post-wake msDelay measured
250097-250103 us and nsDelay 250108-250112 us, with 25 interrupts per sample.
A separate CorePowerTest run passed five wakes with both 128-byte SRAM samples.
The run at `d11b58001` also passed all four polling UART receive prompts:
one before sleep and one after each of three SW0 wakes. Baseline msDelay
measured 250093 us and nsDelay 250107 us; post-wake measurements were
250103-250104 us and 250114-250115 us respectively. Every sample had 25
interrupts. CPU/APB clocks, LF descriptors and APB restoration passed, with
zero failures. UART interrupt/DMA receive modes were not exercised.

## STM32WBA power port (0.14)

STM32WBA requires a family-specific power implementation. The WBA65 I
variant uses the `system_power_stm32wba.cpp` source linked into its Eclipse
managed MCU library. It implements reset and Standby wake reasons, GPIO
wakeup routing and polarity through PWR WUCR1/2/3, the TAMP backup registers,
and whole-bank SRAM1/SRAM2 Standby retention via PWR CR1 R1RSB1/R2RSB1.
The DK1 hardware reported 32 backup registers; a requested SRAM subrange
selects retention for its entire bank.

The dedicated CorePowerTest reserves the SRAM test regions in its linker
script. Hardware validation on STM32WBA65I-DK1 (MB2143),
November 14, 2026: two consecutive PC13/WKUP2 Standby wake resets
(`SYSTEM_RESET_WAKE_PIN`, 0x100) passed. Backup-register startup counts
advanced from 1 through 3. Each wake preserved all 8192/8192 words in
the retained 32 KB SRAM2 test region and yielded 0/8192 unchanged words
in the deliberately non-retained SRAM1 test region.

The example disables USART1 and its DMA before Standby. Earlier tests
with an active UART did not wake; the exact peripheral/DMA/clock contribution
was not isolated. The simplified post-cleanup build still requires a
regression run. Bluetooth/radio and USB activity must be stopped by their
owners; active-stack power transitions are not validated. WBA G memory
variants and WBA2/WBA5 need separate port review.

Reference: ST RM0515 (STM32WBA6), chapter 11 Power control. STM32WBA5
uses RM0493 and has different wakeup and memory layouts.

## Nordic validation (0.14)

The Nordic HFXO reference-count and ownership regression was verified on
nRF52832 hardware: bare-metal HfxoTest 5/5, S132 SoftDevice HfxoTestBle 5/5,
and SDC/MPSL HfxoTestBle 5/5. The source-level clock test
`python3 tests/nordic/check_clock_hfxo_source.py` passed 5 tests. These
results do not cover BLE clock handover during stack initialization.

On the I-SYST IBK-NRF54L15, the Release CorePowerTest was tested with
standalone System OFF / S1 GPIO wakeup (P1.08). RESETREAS mapped to
SYSTEM_RESET_WAKE_PIN (0x100), and GPREGRET boot counts increased across
successive wakeups. SRAM retention was verified in RAM1 for four successive
wakeups with 8192/8192 words preserved in the selected 32 KB section.
The subsequent non-retention comparison passed for three consecutive wakeups,
with 0/8192 words matching the original pattern in the other 32 KB section.
The two ranges start at 0x20020000 and 0x20028000.

System OFF validation must be done without an active debugger connection.
Repeated RESETREAS debug-interface (0x20) restarts were observed in a debug
setup; they are not counted as System OFF GPIO-wakeup validation, and their
precise trigger was not established. Run the Release firmware standalone and
monitor through UART. Do not add debug-reset suppression to the generic demo.

On the nRF54LM20 DK, Release CorePowerTest completed three consecutive
System OFF / button wakeup cycles. Reset reason was
SYSTEM_RESET_WAKE_PIN (0x100), with GPREGRET counter increasing from 3 to 6.
The 32 KB retained RAM1 section at 0x20040000 preserved 8192/8192 words
each cycle. The separate non-retained RAM1 section at 0x20048000 had
0/8192 original words remaining each cycle. These results validate the
specified sections only, not all RAM retention modes.

On the nRF5340 Audio DK (PCA10121), the application-core Release CorePowerTest
used UART RX P1.04, TX P1.05 and PLAY/PAUSE P0.04 for wakeup. Two
consecutive System OFF/wakeup cycles reported SYSTEM_RESET_WAKE_PIN (0x100).
The GPREGRET startup count advanced from 8 to 10. The 32 KB RAM1 region at
0x20040000 retained all 8192/8192 test words, and the separate 32 KB region
at 0x20048000 retained 0/8192 original words, after each wakeup. The example
uses P0.04 because the earlier P0.02 VOL- configuration read as asserted
when the button was not pressed. Temporary pin diagnostics were removed
after validation. The nRF5340 network-core power path was not exercised.

For nRF9151/nRF9161 the current CorePowerTest is a bare-metal GPIO/UART
application: it does not initialize the Nordic modem library, and its Eclipse
project does not link a modem library. Consequently no modem shutdown call
is needed for this example. Other applications which initialize the modem
must first stop cellular operation and shut down the modem through their
own modem interface before SystemPowerOff. The nRF91x1 DK demo uses UART0
RX P0.26 / TX P0.27 and wake button P0.08. It should be tested standalone
in Release without an active debug session.

The nRF91 RAM linker region is shared with application data, stack and
optional modem shared-memory sections; unlike the tested nRF54 targets
it does not reserve a separate free RAM1 region. Do not reuse the nRF54
fixed RAM test addresses on nRF91 without a dedicated linker reservation.

The nRF91x1 CorePowerTest was hardware validated on the reported DK setup:
three successive GPIO wakeups from System OFF, with reset reason
SYSTEM_RESET_WAKE_PIN (0x100) and GPREGRET start counter increasing from
3 to 6. This validates bare-metal System OFF, button wakeup and retention
registers for the tested configuration. It does not validate SRAM retention,
modem-active shutdown, or the independent nRF9160 variant.

For the nRF9160 DK, the IOsonata 0.14 CoreClockTest subsequently completed
two consecutive hardware UART runs with **0 failures** after the LFRC
measured-frequency RTC correction and Disable/Enable cache fix. Both
RTC0/RTC1 passed `msDelay` and `nsDelay` 250-ms checks using measured
~44.73–44.74-kHz LFRC rates, and crystal-based RTCs, HF timers, sDelay,
shared-clock lifecycles and timer triggers passed. This validates RTC
**timekeeping compensation**, not physical LFRC trimming or datasheet
frequency tolerance. The subsequent nRF9160 DK CorePowerTest
also completed four boots with zero failures (14 checks on the first boot
and 15 on each of the next three, 59 checks total). Three timer-based
System ON sleep/wake cycles passed on each boot, for 12/12 cycles. Clock,
UART and static/stack RAM continuity passed after each resume. Three
successive Button 1 wakes from System OFF reported reset reason 0x100
and passed retention marker and reset-on-wake checks. GPIO wake
configuration/disable/rearm and retention register operations also passed.
SRAM retention across System OFF was UNSUPPORTED because no dedicated
test region was provided; reduced CPU power mode is not supported.
The log showed SysLog evicted=0, UART pending=8 at snapshot and
66 unaccepted UART submissions per boot, a backpressure diagnostic
rather than evidence of lost output. Modem-active System OFF, current
consumption and nRF91x1 CoreClockTest/CorePowerTest remain unverified.

Outstanding family-specific build checks and the nRF5340 network-core
power limitations still require review.

## SAM4E

`system_power_sam4e.cpp` implements the power functions. `SystemInit` captures
reset/wake status before a clock read can clear SUPC status. Reset reasons are
cached. Wake pins use the WKUP0..15 assignments in datasheet tables 10-2/10-3.
The application must configure a pin as a GPIO input; analog or capture uses of
the same pin must be stopped first. Both high and low wake levels are supported.

Backup mode switches off the core regulator and all SRAM. It keeps 20 GPBR
words while VDDIO remains supplied. `SystemRamRetain` returns false because
SRAM retention is not available in that mode. GPBR writes are checked by readback;
tamper protection may prevent a write.

`system_clock_sam4e.cpp` implements slow-clock selection and oscillator
initialization. RC is 32 kHz; crystal/bypass is 32768 Hz. Switching to the external
source is irreversible until the backup supply resets. A request to switch back
to RC or change between crystal and bypass after selection returns false.
Startup waits are bounded. On timeout, the requested configuration is not
reported as active; a late oscillator start can still occur in hardware.

RTC/RTT drivers own their alarm configuration. Power off preserves it. The reset
reason reads their alarm status only for Backup exits with the corresponding
wake source enabled. No GPBR word is reserved by this implementation.

Reference: [Microchip SAM4E datasheet](https://ww1.microchip.com/downloads/en/DeviceDoc/Atmel-11157-32-bit-Cortex-M4-Microcontroller-SAM4E16-SAM4E8_Datasheet.pdf),
sections 10, 16, 18 and 19.

Run `python3 tests/sam4e/run_power.py` for register-model tests, production-header
syntax checks and Eclipse source-link checks. These do not measure power, execute
a target firmware image, or validate physical wakeup. Those checks remain to be
performed on hardware.

Arm GCC 15.2.1 target checks passed on November 15, 2026:

- `python3 tests/microchip/compile_power.py`: all six SAM4E variants compile
  with the production CMSIS headers; every typed peripheral address is checked.
- `python3 tests/sam4e/build_examples.py`: SAM4E16E Blinky, UartRetargetDemo
  and UartPrbsTxTest compile and link in Debug and Release (six builds).
  The check uses selected library sources and the managed projects' source
  links, defines and linker scripts. It verifies that each image includes the
  SAM4E clock and reset implementations. It does not run IOcomposer or hardware.

Main-clock selection in `system_sam4e.c` switches MCK to SLCK before changing
MAINCK or PLLA. Prescaler and source writes are separate and each waits for
MCKRDY. All ready waits are bounded. A failed oscillator or PLL start leaves
MCK on SLCK; a failed MCK transition returns false and the getter reports the
register-selected frequency, which cannot confirm that the hardware stabilized.
Flash remains at five wait states so a late transition cannot outrun flash.

RC requests choose the nearest 4, 8 or 12 MHz setting (ties choose the lower
rate). External crystal inputs must be 3..20 MHz; bypass inputs supported by
this PLL configuration are 3..50 MHz. The supplied frequency describes the
physical input. PLLA chooses the highest legal output up to 240 MHz and MCK
uses half that rate. When USB clock support is requested, PLLA must provide an
exact multiple of 48 MHz and its USB divider is configured accordingly.
The USB driver still owns enabling the USB clock.

Clock changes are refused while USB or a programmable clock is enabled, or PMC
write protection is active. Applications must stop other peripheral transfers
before a runtime change and reconfigure their baud rates and timer periods
afterward. Runtime selection does not reinitialize the watchdog or slow clock.
Peripheral clock index 0 is MCK; other indices return 0.
The main-clock register model checks repeated selections, transition ordering,
PLL limits, bypass, RC rounding, USB division and ready timeouts at O0 and O2.
See datasheet sections 28, 29 and 46 for the clock tree and electrical limits.

## SAM4L

SAM4LC CoreClockTest rerun passed all three RC4/RC12 selection cycles with readable
UART output. CPU/APB measurements were about 44.8 MHz with RC4 and 39.64 MHz
with RC12. Both 250 ms delays measured about 253420 us with 25 interrupts,
within the demo's 2 percent tolerance; the final result was zero failures.
The earlier SystemSleep implementation used SLEEP0, preserving BPM mode, SCR,
PBD clock gates and the caller's interrupt mask. Entry clears RET/BKUP and
SLEEPDEEP/SLEEPONEXIT, and returns without sleeping if VREG is not ready.
Peripherals retain their running clocks; enabled interrupts wake the CPU.
That tested revision did not implement WAIT or RETENTION. Its SAM4LC CoreClockTest selected
OLED1 BUTTON1/PC06 through GPIO interrupt group 8 and checks the wake callback,
clocks, both delays, timer IRQs and UART RX across three cycles. The Release
retest passed all checks with zero failures. Its 1260164 Hz timer measured
251320 to 251351 us for 250 ms delays, with 25 interrupts before and after
sleep. All four UART RX prompts passed. Backup wake continues to use the
separate EIC route.

SAM4LC CorePowerTest was clean-rebuilt and retested on November 17: eight
consecutive pin wakes reported reset reason 0x100, and the retained start
count advanced from 1 to 9. The intermittent failure did not recur in that
run. This validates the tested EIC8 wake and start-count retention; it does
not cover all backup registers or measure Backup current.

The SAM4LCxC CorePowerTest now selects OLED1 BUTTON1 on EXT2 (PC06 / EIC8) for
Backup wake. Hardware confirmed one BUTTON1 wake and retained start count
advancing from 1 to 2. The following re-arm exposed a stale ownership check:
EIC remained enabled across Backup but its SRAM flag was cleared. Wake-line
ownership now follows the retained Backup pin routing. Wake setup also enables
the EIC peripheral interrupt. The clean rebuild results above confirm repeated
wake with these corrections.

SAM4LC CoreClockTest hardware results on November 17 exposed UART corruption with
RCFAST 4 MHz selected and incorrect delay calibration: 250 ms measured as
205823 us through `msDelay` and 586699 us through `nsDelay`. The SAM4L port now
uses 14-cycle and 3-cycle loop factors, respectively, refreshed from the CPU
frequency at startup and after clock updates. The later Release run exposed
the incorrect 13-cycle microsecond factor; fixed loop alignment alone did
not correct it. The 14-cycle factor passed the hardware retest above.
CoreClockTest also acquires and stops AST before selecting clocks because AST
can survive reset after ClockDisplayDemo. The generic ARM delay loops use
fixed alignment without MCU-specific conditionals.

The DFLL output is measured with FREQM against the configured CLK32K reference
before selecting it. `SystemCoreClockGet` and `SystemPeriphClockGet` use this
measured rate with their hardware dividers. UART already calculates its baud
rate from `SystemPeriphClockGet(0)`; no UART-specific clock correction is used.
Measurement accuracy depends on the low-frequency reference. An internal
RC32K reference does not provide crystal accuracy. An active FREQM user blocks
core-clock changes. A failed measurement reports failure instead of accepting
the nominal DFLL frequency.

SAM4LC/SAM4LS Debug and Release CoreClockTest and CorePowerTest links pass, as do
existing SAM4L clock and power host checks. The SAM4LC hardware rerun above confirms UART output and delay timing.
SAM4LS hardware validation remains pending.

`system_power_sam4l.cpp` implements reset reasons, Backup entry, EIC wake pins
and four 32-bit backup registers. Backup powers down both SRAM banks, so
`SystemRamRetain` returns false. Ordinary Retention sleep is not used for
`SystemPowerOff`, since it resumes execution instead of resetting the core.
Stop DMA and peripheral transfers before calling power off. Existing AST and
other backup wake enables are preserved; their drivers own their configuration.

SAM4L startup leaves the watchdog disabled; applications enable it for their
own use case. Runtime core-clock selection preserves that application choice.
Low-frequency clock selection returns false on an oscillator-ready timeout,
restores the incoming interrupt mask, and updates the oscillator descriptor
only after success. Repeating the same request checks readiness again.
The descriptor records the last successful selection, not a measurement of
clock health. A failed external-mode change may leave that oscillator stopped.
Startup still halts when its required clock cannot be established. Runtime
main-clock selection now returns false on oscillator, PLL, DFLL, divider or
flash setup failure and attempts to leave the CPU on RCSYS. The clock getter
reports the register-selected source and divider after the attempt. A failed
hardware switch cannot be treated as a confirmed stable clock.

Runtime selection preserves the watchdog, low-frequency clock, cache settings
and voltage scale. It chooses dividers within the existing voltage scale
instead of requesting a new power transition. Startup retains its checked
power transition in SRAM and stops if its required clock setup fails.
The caller must still stop clock consumers before changing the main clock
and reconfigure their rates afterward. Runtime changes are refused while a
UART, SPI or I2C peripheral clock is enabled, USB is enabled (including suspended USB), or any PDCA channel is
enabled. USB or PDCA with its HSB clock enabled but register clock gated is
also refused. Application generic clocks must be disabled; GCLK0 remains the
system DFLL reference, and GCLK7 is treated as system-owned only when
`bUSBClk` is set and USB is disabled. Refusal leaves hardware registers and
oscillator descriptors untouched and restores the incoming interrupt mask.

Enabled analog, audio, LCD and other PBA peripheral clock gates also refuse
main-clock changes. The AST must be stopped with no synchronization pending.
For TC timers managed by IOsonata, the timer port reports running/unhealthy
state without reading TC status, so rollover and compare flags are preserved.
A TC block with no IOsonata timer owner is refused while its PBA clock is on.
Applications using TC registers directly must stop and gate that block before
changing the clock; mixing direct channel control with Timer objects in the
same block is not covered by the ownership query.

Disable every active timer before changing the clock. After a successful
change, TC `SetFrequency` reads the current PBA frequency and recalculates its
divider and trigger periods. It resets and starts the counter as before. The
optional timer query does not pull timer drivers into a clock-only image.
Core SysTick/scheduler timing remains the application's responsibility.
Low-frequency selection refuses a running or synchronizing AST, enabled
PICOUART, an LF-selected watchdog, enabled LCD/CATB clock gates, an active
frequency measurement, or any generic clock using OSC32K, RC32K or the 1 kHz
output. Both sampled brownout monitors are conservatively treated as busy;
continuous brownout detection and a watchdog using RCSYS may remain enabled.
The selector does not disable or reconfigure these users.

The BPM/BSCIF, AST, WDT, PICOUART, SCIF and FREQM register clocks must be
accessible for these checks. If one is gated, the request is refused rather
than temporarily enabling clocks or assuming its peripheral is stopped.
These gates are enabled by the reset configuration. Refused calls preserve
all registers, pending flags, oscillator descriptors and the interrupt mask.
After a permitted source change, AST SetFrequency refreshes its source rate
before recalculating its prescaler and periods.

Repeating a request rechecks the hardware setup; a matching descriptor alone
does not report success.
Run `python3 tests/sam4l/run_clock.py` for the Linux register-model checks.

Wake pins use asynchronous high/low level detection through the backup mux.
Configure the selected pin as a GPIO input first. Toggle detection is not
supported. An EIC line already enabled by another driver is refused. Wake
configuration does not install an ISR or enable NVIC delivery.

| Package | Backup wake pins |
| --- | --- |
| A | PA04, PA05, PA06, PA07 |
| B | A pins plus PB01 |
| C | B pins plus PC03, PC04, PC05, PC06 |

Both SAM4LC and SAM4LS use these mappings. Alternative EIC pin routes cannot
wake from Backup. Disabling a pin removes its backup mux and EIC enable while
preserving the shared EIC wake enable for other users.

The cause registers remain readable until the next reset/wakeup. SYSRESETREQ
is reported by hardware as OCDRST and maps to `SYSTEM_RESET_SOFT`; software
cannot distinguish a debugger-issued SYSRESETREQ. Backup EIC and AST wakeups
map to the generic pin and timer reasons. PICOUART wakeup has no generic reason
bit and is not misreported as USB or GPIO.

Run `python3 tests/sam4l/run_power.py`. It checks production headers for all
18 C/S, density and package combinations, runs host register tests for all six
package/family combinations at O0 and O2, and checks both Eclipse library links.
These are not Arm firmware builds or hardware power/wakeup validation.

Arm GCC 15.2.1 also compiled the power and clock sources for all 18 SAM4L
variants with `python3 tests/microchip/compile_power.py`. The SAM4LS8C
peripheral check linked 20 examples in each of Debug and Release, including
UsbCdcLoopbackTaktOS, using TaktOS commit
`86002a6c3fee280ac0e978283919630f99e3b53d` from
[IOsonata/TaktOS](https://github.com/IOsonata/TaktOS). Run it with
`python3 tests/sam4l/peripheral_build_test.py --taktos ../TaktOS`.
These 40 builds use selected library sources; hardware execution is unverified.
The 18-variant compilation and all 40 example links passed again after the
runtime failure-handling change. The existing clock register checks also passed
for SAM4LC8C and SAM4LS8C at O0 and O2, including failed oscillator/PLL/DFLL
startup, retry, flash readiness, interrupt-mask restoration and WDT preservation.
The low-frequency ownership checks cover every generic-clock index and verify
register preservation on refusal. The timer model checks AST and TC retiming.
All seven TimerDemo selections also compile and link in Debug and Release.

Reference: [Microchip SAM4L datasheet 42023H](https://ww1.microchip.com/downloads/aemDocuments/documents/OTH/ProductDocuments/DataSheets/Atmel-42023-ARM-Microcontroller-ATSAM4L-Low-Power-LCD_Datasheet.pdf),
sections 6.1.3, 10.7.15, 11.5, 11.6.9..11, 12.7.18 and 21.6.

## LPC11Uxx power and clock port

The LPC11U35 library implements the existing generic power functions using
UM10462 chapters 3 and 4. `SystemPowerOff` enters Deep power-down and discards
pending CPU interrupts. PIO0_16 wakes the chip on a falling edge and starts a
reset. The pin needs an external pull-up; the RESET pin is inactive in this
mode. `SystemWakePin` accepts only that pin and polarity. It returns false for
disable because the hardware wake input cannot be disabled. Applications must
not set the write-once PCON NODPD bit if they intend to use Deep power-down.

`SystemResetReason` caches the first read and clears the recorded flags.
DPDFLAG identifies wakeup rather than a cold power-on, even though wakeup
internally generates POR. Four full-width GPREG words survive Deep power-down
and software reset. GPREG4 is left to the application because it also contains
wake-pin hysteresis control. SRAM retention in Deep power-down is unsupported.
`SystemSleep` currently uses ordinary Sleep, preserves RAM and peripheral
clocks, and wakes from an enabled NVIC interrupt. Deep-sleep and Power-down
with clock restoration are not implemented. `SystemWakePin` selects the
Deep power-down input only; ordinary Sleep uses the normal interrupt drivers.

Runtime core-clock requests refuse enabled timer, SSP, UART, ADC, USB and WWDT
clock gates, an enabled I2C controller, or CLKOUT. Stopped timers whose gates
remain enabled are conservatively refused. These checks apply to the legacy
setters as well as `SystemCoreClockSelect`; startup configures the clocks before
drivers take ownership. Applications must stop and reconfigure SysTick and
scheduler timing around an accepted runtime change. Clock decoding reports
zero for powered-down sources or an unlocked PLL. The existing fixed 48 MHz
CPU target and bounded PLL recovery to 12 MHz IRC remain unchanged.

`SystemLowFreqClockSelect` selects the closest nominal watchdog-oscillator
rate for an RC request. Actual nominal frequency and 40-percent oscillator
accuracy are recorded in `g_McuOsc.LowPwrOsc`. A running WWDT, its enabled bus
clock, a main-clock WDO selection, or a WDO CLKOUT selection prevents changing
it. This does not enable or feed the watchdog. The default LF descriptor has
frequency zero, so startup leaves WDO unconfigured; an application-supplied
nonzero LF descriptor is applied during startup.

The managed `CorePowerTest` project links the existing generic demo, with UART
DMA disabled for LPC11U35. Validation includes all 115 enabled library source
files in Debug and Release, TimerDemo, UART PRBS TX and loopback, SPI loopback,
I2CScan and CorePowerTest links in both configurations, existing clock tests in
C and C++, and temporary power-register checks at O0/O2. These are compilation
and register-model results; sleep entry, wake reset, retention and current draw
still need hardware validation. D11 remains the current hardware test target.

## LPC17xx power and clock port

The LPC1769 managed library includes reset reporting, Deep Power-down,
ordinary interrupt-resumable Sleep and five full-width RTC backup registers.
RTC register access temporarily enables PCONP.PCRTC and restores the previous
gate state. Backup contents survive software reset and Deep Power-down while
the VBAT domain remains supplied. SRAM cannot be retained in Deep Power-down.
The watchdog is neither enabled nor fed by these functions.

Deep Power-down wakes through the RESET pin or an application-configured RTC
interrupt. `SystemPowerOff` preserves the RTC interrupt enable and pending
state, stops SysTick, and disables other CPU interrupts before entry. The
application must arm a future RTC event and handle any already-pending event
before entering. `SystemWakePin` returns false because there is no selectable
GPIO wake input for this mode. `SystemSleep` uses ordinary Sleep and enabled
NVIC interrupts; Deep Sleep and Power-down with PLL restoration are not yet
implemented.

`SystemResetReason` caches and clears RSID and DPDFLAG on the first call.
It reports power-on/brownout, external reset and watchdog reset. After Deep
Power-down it reports reset-pin wake when EXTR is set, otherwise RTC wake when
an RTC interrupt flag is present. It leaves RTC interrupt flags untouched.
RSID has no dedicated software-reset or lockup flag, so these are not inferred.
Read the reset reason before application RTC initialization clears wake flags.
With a separately supplied VBAT domain, retention and DPDFLAG can outlive a
loss of core supply; the registers do not identify every such power sequence.

Runtime core-clock changes refuse enabled peripheral gates except RTC and
GPIO, active PLL1, CLKOUT, a PCLK-clocked running watchdog, and the DAC counter.
The gate check is conservative: powered but idle peripherals must also be
released. Startup configures clocks before application drivers own them.
The divider/PLL search remains outside the interrupt lock; register changes
and ownership checks run under the lock. The closest-frequency calculation,
100/120 MHz device limits and bounded PLL failure handling remain in place.
Applications must stop and reconfigure SysTick and scheduler timing around a
runtime change. Peripheral-clock setters remain caller-controlled because
drivers use them while configuring their own peripherals.

The port refuses an oscillator-range change while MAIN feeds the CPU. Switch
to IRC first; changing the range of an unused oscillator stops and restarts it
with bounded ready checks. Other SCS fields are preserved. PLL1 configuration
refuses an enabled USB peripheral. The RTC has a fixed 32768 Hz external clock;
`SystemLowFreqClockSelect` returns false because there is no programmable LF
source selector. This does not stop or reset the RTC.

Validation covers all 124 enabled library sources in Debug and Release,
TimerDemo, CorePowerTest, UART PRBS TX/loopback, SPI loopback, I2CScan and the six
USB demos in both configurations. Clock register tests cover C/C++ callers,
LPC1758/1759/1768/1769 limits, oscillator/PLL failures and busy-user rejection.
Temporary power-register checks cover reset caching, backup access, interrupt
selection and sleep register restoration at O0/O2. USB project source links
and four C++ event initializers were corrected; USB_IRQHandler now has the C
linkage required by the vector table. Hardware sleep/wake, retention and current
measurements remain pending. D11 remains the current hardware test board.

## LPC546xx power and clock port

Both LPC54605 and LPC54628 managed libraries include the target power unit.
It caches and clears SYSRSTSTAT on the first reset-reason query, implements
ordinary interrupt-resumable Sleep, and enters reset-on-wake Deep power-down.
The latter follows NXP libpower's POWER_EnterDeepPowerDown sequence: SCR deep
sleep, PDRUNCFG1 = 0x300002AF, PDRUNCFG0 = 0xFEFFFFF7, then WFI. These full
register values include reserved bits required by the vendor sequence. No
vendor binary is linked. Reference: UM10912 power management and power API
chapters, and [NXP MCUXpresso LPC54628 libpower_hardabi.a](https://github.com/nxp-mcuxpresso/legacy-mcux-sdk/blob/main/devices/LPC54628/gcc/libpower_hardabi.a),
Git blob d58823378ee47b14321d21e662696dd6a7bc578a.

Deep power-down leaves only the RTC domain and RESET wake input. RTC CTRL
alarm/countdown wake configuration is preserved. Applications must arm a
future RTC event and handle existing flags before entry. CPU interrupts and
SysTick are stopped; ordinary Sleep preserves their configuration and restores
the incoming SCR and interrupt mask. Deep-sleep clock and voltage restoration
is not implemented. No watchdog is enabled or fed by these functions.

There is no selectable GPIO wake input for Deep power-down, so SystemWakePin
returns false. SRAM retention is unsupported. Seven full-width backup words
alias the RTC driver's GPREG0..6; applications must coordinate their use.
GPREG7 remains the RTC time-valid marker. Backup access temporarily enables the
RTC bus clock and restores its prior state without resetting the RTC.
Reset reporting uses only the hardware POR, BOD, EXTRST, WDT and SYSRST flags.
It does not infer a Deep power-down wake cause from a possibly stale RTC flag.

Runtime core selection refuses peripheral clock gates other than memory,
GPIO/pin routing and RTC, active audio/USB PLLs, or CLKOUT. This is conservative:
a stopped peripheral with its gate enabled must still release it. The PLL
search precedes the interrupt lock; the ownership check and transition are
atomic against interrupts. Startup uses the same setup without application
ownership checks. Applications must stop and reconfigure SysTick and scheduler
rates around a runtime change. An external oscillator is stopped before its
mode/range is changed. Failed PLL setup leaves the core on the 12 MHz FRO.

Clock decoding rejects powered-down FRO, system oscillator and PLL sources,
unlocked PLLs, disabled FRO high-frequency output, and invalid PLL divisors.
The RTC's system-clock output must be enabled to be decoded. This is register
state, not an oscillator-health measurement. The LF descriptor now describes
the fixed 32768 Hz crystal; SystemLowFreqClockSelect returns false, and the RTC
driver retains ownership of oscillator startup and verification.

The new LPC54628 CorePowerTest uses the existing generic example and VCOM UART.
It wakes through hardware RESET; P0.4 cannot wake Deep power-down. All 179 LPC54628 and 32 LPC54605 managed library sources compile in Debug
and Release. The LPC54628 timer, peripheral and USB checks pass 60 firmware
links across those configurations. Existing register tests cover clock ownership, failed PLL setup,
RTC, timers, transfers, storage, audio and other peripherals. Temporary power
register checks ran at O0/O2. These do not establish hardware wake behavior,
backup retention or current draw. D11 remains the current hardware test board.


## RA4M1 power and clock port

RA4M1 has Sleep, Software Standby and Snooze, but no Deep Software Standby.
The shared vendor header includes registers belonging to other RA variants;
the port does not use those as evidence of RA4M1 capabilities. There is no
SystemPowerOff implementation. The existing generic CorePowerTest uses its
resume path through SystemSleep, with P000/IRQ6 and SCI0 P100/P101.

SystemWakePin uses the existing GPIO interrupt allocator. Falling and rising
edges are supported, with the IRQ filter disabled. A pin, IRQ line or event
route already owned by GPIO, DMA/DTC or another wake source is refused.
Disabling the owned pin releases its WUPEN bit and interrupt route. Ordinary
GPIO interrupt release also removes a wake bit owned by this port. Unrelated
WUPEN bits and pin routes are preserved.

SystemSleep uses Software Standby when external IRQ wake is configured and
an associated NVIC interrupt is enabled, DMA/DTC are stopped, Snooze is off,
and oscillator-stop detection is disabled. Otherwise it uses ordinary Sleep.
Other Software Standby wake sources are not yet handled by this implementation.
Active SysTick/PendSV pending work returns without sleep so it can be serviced.
Applications must stop incompatible peripheral transfers before entry.

The Standby entry follows procedure B of Renesas technical update
[TN-RA*-A0146A/E](https://www.renesas.com/en/document/tcu/note-about-interruption-during-transition-low-power-modes-0):
disable SysTick interrupt generation and non-wake NVIC delivery, then DSB/WFI.
Event routes and pending IRQ flags are preserved. After resume, restore SBYCR,
PRCR, SCR, NVIC enables and SysTick control before restoring PRIMASK. SBYCR
bit 14 is always written as one. The port does not touch unsupported DPSBYCR.
Software Standby retains SRAM and resumes execution; it does not generate a
reset reason. Reset-on-wake RAM retention and generic retention words report
unsupported. Battery backup memory integration remains outside this port.

Reset reporting caches the first read of RSTSR0/1, then clears their recorded
flags. POR and voltage-monitor resets map to POWERON, WDT/IWDT to WDT, and
SWRF to SOFT. RES clears all reset flags, so an all-zero first read maps to PIN.
Read this API before other code clears reset flags. Memory/MPU/stack errors
have no exact generic reason bit and are not mislabeled as CPU lockup. RSTSR2
cold/warm history is left untouched. Startup option words keep WDT and IWDT
auto-start disabled; sleep and clock selection do not feed or start them.

Runtime core-clock changes now refuse enabled peripheral module gates,
DMAC/DTC's enabled module gate, and CLKOUT before unlocking protected registers.
The SRAM gates and MSTPC0 clock-domain policy are excluded from that test.
This is conservative: a stopped but ungated peripheral still prevents a change.
Startup bypasses the application-ownership check. The existing oscillator
plans, bounded ready waits and protection/interrupt restoration are retained.
LF selection starts LOCO or SOSC without stopping the other oscillator or
resetting the RTC; the peripheral chooses its own LF source. Callers still
own SysTick/scheduler retiming after a successful core-clock change. Module-gate
checks do not track a watchdog started directly through registers; applications
must not change its PCLK timing while it is running.

All 305 enabled library sources compile in Debug and Release. All 19 managed
demo projects link in both configurations, including CorePowerTest. GCC host
checks run the existing clock and GPIO models at O0/O2 with undefined-behavior
checks, including wake ownership and runtime busy refusal. The existing timer
model also passes at O0/O2. UART passes at O0/Os/O2 and with address/undefined-
behavior checks (1,167,463 interrupts per run). I2C and SPI models pass their optimization and
address/undefined-behavior checks, with leak detection disabled because the
workspace runs under ptrace. Temporary power
register checks cover reset caching, mode selection, pending work, interrupt
masking and register restoration at O0/O2. The original all-in-one validation
runner additionally requires Clang/LLD, which are not installed in this workspace;
its separate Clang ELF probes were not run. Hardware wake, current draw and
oscillator restoration remain pending; D11 is still the hardware test target.

Reference: RA4M1 hardware manual R01UH0887EJ0110, chapters 5, 8, 10 and 13,
and the RA4M1 feature definitions in Renesas FSP.


## R9A02G021 generic power and clock integration

SystemResetReason caches RSTSR0/1 before clearing recorded flags. Power-on and
voltage-monitor resets map to POWERON, watchdog resets to WDT and software
reset to SOFT. An all-zero status maps to PIN following the manual's reset
source flow. SRAM parity/ECC and bus faults have no matching generic reason
bit and are not reported as CPU lockup. RSTSR2 cold/warm history is preserved.

The GPIO interrupt driver arms the ICU wake bit on allocation and clears it
on release. An IRQ assigned to a Snooze request keeps that configuration.
On FPB-R9A02G021, SW1 uses P108/IRQ4. UART0 uses P303 (RX) and P302 (TX)
at 1 Mbaud.

Clock supply type A uses half the source frequency for PCLKB (peripheral
clock 0). LOW_POWER halves ICLK and leaves PCLKB unchanged. Type B keeps
ICLK and PCLKB equal and supports PERFORMANCE only. The delay functions
read the machine cycle counter and use the current ICLK frequency.

Timer 0 uses the machine timer clocked by LOCO. Timers 1 through 8 use TAU
channels 0 through 7. A disabled channel keeps its prescaler until PowerOff.
The last channel to power off restores the TAU module gate if the timer
driver started the module.

An armed machine timer uses ordinary Sleep. With an enabled ICU wake source
and Snooze off, SystemSleep can use Software Standby. The application prepares
its peripherals before the call. The port temporarily masks non-wake CLIC
interrupts for Software Standby and restores their enables on return, as
required by Renesas TN-RIS-A0001B/E. Explicit Snooze entry for UART, SPI and
ADC retains its driver hooks. There is no reset-on-wake System OFF mode.

## RE01

The RE01_1500KB library now includes `system_power_re01.c` in all six
DBN/CFB/CFP Debug/Release configurations. Reset reporting caches and clears
RSTSR0/RSTSR1 on the first call. Deep Software Standby wake flags identify
pin and CCC wake. Startup disables the previous boot's wake configuration
and clears the Deep Standby mode selection without clearing those wake flags.
No retention registers or reset-on-wake RAM retention are exposed.

`SystemSleep()` uses Software Standby only in normal ALLPWON mode, with Snooze
disabled, the DMAC/DTC module stopped, and an enabled external IRQ wake route.
It selects ALLPWON -> EXFPWON -> ALLPWON and retains all eight SRAM banks.
Boost, other power-domain modes, Snooze, enabled DMAC/DTC, or other wake policies
use ordinary Sleep. Both paths preserve the caller's interrupt mask, register
protection, sleep settings and pending interrupts. Drivers still own stopping
peripherals before Standby. The port does not enable a watchdog.

Configure a GPIO interrupt with `IOPinAllocateInterrupt()` or
`IOPinEnableInterrupt()`. Rising and falling edges on the four dedicated pins
also enable Deep Standby wake. Toggle sensing uses the ordinary IRQ. Disabling
the interrupt releases the wake setting that the GPIO driver enabled.

| Deep Standby source | Pin |
| --- | --- |
| IRQ0-DS | P4.11 |
| IRQ1-DS | P2.7 |
| IRQ2-DS | P4.10 |
| IRQ3-DS | P4.9 |

P4.11 and P2.7 share debug functions. Their dedicated Deep Standby wake
inputs remain disabled during debugging.

Startup selects the oscillator and sets ICLK and PCLKB. Peripheral clock
requests return the actual frequency of the shared domain. LOW_POWER divides
ICLK by two and keeps PCLKB unchanged. PERFORMANCE restores ICLK. A held HF
clock request prevents a divider transition. Both transitions update the delay
factors. Clock getters report zero for stopped sources and reserved dividers.

The timer driver numbers AGT0/1 first, followed by the paired TMR and six
GPT channels. Disable keeps the timer configuration. PowerOff releases its
interrupts and restores the module-stop bit when the last channel using the
module powers off. TimerTest exercises all nine devices and sDelay.

The new `CorePowerTest` has six package/configuration targets and uses the
shared resume-on-interrupt demo, including static and stack RAM samples.
Its application clock configuration selects the internal 32 MHz oscillator
at startup, avoiding Boost. The UART uses the existing P2.2/P2.3 SCI4 mapping
at 1 Mbaud. The wake pin is P4.10 (IRQ2/IRQ2-DS), connected to SW3 on the
EK-RE01 1500KB. Other boards can use a normally-open button from P4.10 to
ground. Check the board/package pin availability before use.

The demo defaults to resumable Standby with static and stack RAM samples.
Set `POWER_DEMO_RESUME_WAKE=0` in the application compiler definitions to
exercise reset-on-wake Deep Standby instead. Both forms use the same project
and shared demo source. Since this part has no exposed retention registers,
the reset-on-wake demo's start count is not retained.

`SystemPowerOff()` disables SysTick/NVIC delivery, exits Boost if needed, and
enters Deep Software Standby with I/O state retained. Dedicated wake sources
reset the MCU; the function never resumes the application. If debug is enabled
or a required transition fails, it stays stopped until reset; Deep Standby
is not entered. Debug-mode Boost transitions are also refused
by the clock helpers. The application owns quiescing its devices before power
off, as shown by the demo's UART drain and disable.

For early WS1 silicon, compile the RE01 library with `CHIP_VERSION_WS1`.
The vendor restrictions require an application-configured CCC wake source for
Deep Standby; without it, `SystemPowerOff()` stays stopped until reset. The
port does not enable CCC or a watchdog on behalf of the application.

The transition sequence follows Renesas' RE01 driver package:
[`r_lpm_api.c`](https://github.com/renesas/re-driver-package/blob/master/SDK_RE01_1500KB/RE01_1500KB_DFP/Device/Driver/Src/r_lpm/r_lpm_api.c)
and
[`r_system_api.c`](https://github.com/renesas/re-driver-package/blob/master/SDK_RE01_1500KB/RE01_1500KB_DFP/Device/Driver/Src/r_system/r_system_api.c).
Pin assignments follow Renesas datasheet R01DS0363EJ0110, tables 1.5-1.7.
Hardware validation remains pending for wake/reset behavior, I/O retention,
Boost transitions and sleep current.

## ESP32-C3/C6 CPU sleep and clock readout

This section records the earlier CPU-WFI-only batch, before the RTC/PMU
implementation. The current code and its unverified status are described
in "ESP32-C3/C6 RTC and PMU sleep implementation" below.

The C3 and C6 libraries now provide SystemSleep using CPU WFI. It keeps the
peripheral clocks, SRAM and stack intact. Interrupt delivery is masked around
WFI and restored to its previous state after wake. The GPIO driver implements
SystemWakePin with the existing interrupt registration and dispatch functions.
It accepts rising or falling transitions, refuses pins already owned by another
callback, and releases only its own callback. C6 pads assigned to LP IO are
refused. This mode does not configure the RTC or PMU sleep state machine.

SystemResetReason reads the last hardware reset cause without clearing it.
Power/brownout, software, watchdog and JTAG causes map to generic flags. C3's
power-on code also covers chip-level brownout and super-watchdog events; the
hardware code does not distinguish them. Unmapped reset causes return zero.
For a deep-sleep reset (0x05), pin and timer wake causes map to the generic
wake flags. C3 uses RTC wake status with timer bit 3; C6 uses PMU wake status
with timer bit 4. Multiple recognized causes are combined. Wake status is not
cleared, and a different reset reason does not report stale wake flags. This
decoding does not enable PMU sleep entry.

C3 clock decoding now applies PRE_DIV_CNT to XTAL and RC_FAST and returns zero
for reserved clock selections. Its peripheral clock 0 is the APB clock, limited
to 80 MHz. C6 peripheral clock 0 follows the root, AHB and APB dividers instead
of returning CPU frequency. C6 RC_FAST uses the nominal 20 MHz rate; a reserved
root returns zero without dividing by zero in the delay-period calculation.
Both ports reject invalid peripheral clock indices.

The C6 startup CPU-frequency option no longer requires ESP32_PLL_FREQ. When
ROM left PLL selected, it changes only the CPU divider, accounting for the
fixed root divide-by-three and the separate 120 MHz selection. AHB, APB, flash
and peripheral clock selections remain unchanged. A non-PLL boot configuration
is preserved. Default startup still preserves ROM clock selection. Watchdogs
remain disabled during startup and are not changed by SystemSleep.

CorePowerTest managed projects are provided for C3 and C6, using UART0 at 1 Mbaud
and the GPIO9 BOOT button. They use the generic resume path and static/stack SRAM
sample check. Release BOOT before resetting the board. The generic demo prints
"Standby"; on these targets the implemented mode is CPU WFI only.

Runtime core/low-frequency oscillator selection and peripheral-clock setters
still return failure. The GPIO driver also prepares separate RTC/LP deep-wake routing for
registered wake pins, with rollback after a rejected sleep. C3 supports
GPIO0-5; C6 supports GPIO0-7. Preparation checks GPIO direction, the
input-matrix consumers, GPIO0/1 when used by the 32 kHz crystal, and
C6 pins already assigned to LP IO. A driver cannot silently replace
a wake-owned interrupt callback. The preparation is not yet called by
a PMU sleep transition, so this routing is not a functional Deep-sleep
mode and must not be used as evidence that Deep-sleep is implemented.

SystemPowerOff is not implemented. No retention registers
are exposed because RTC scratch registers are used by ROM and clock setup;
SystemRamRetain returns false. PMU Light-sleep, Deep-sleep, RTC/LP wake sources,
retention and runtime oscillator ownership remain work to complete before
calling the ESP32 power and clock port complete.

Validation for this batch:

- All 200 C3 and 202 C6 managed-library translation units compile in both Debug
  and Release with RISC-V GCC 15.2.0.
- Both new CorePowerTest projects link in Debug and Release using their managed
  source and linker settings. Release text/data/bss: C3 11137/480/1420 bytes;
  C6 11395/480/1600 bytes.
- 24 existing ESP32 Python checks pass, including extended C3/C6 clock MMIO
  checks at O0 and Os.
- GPIO interrupt/wake ownership and input-routing checks pass for all five
  C-series variants at O0 and O2. C2/C5/C61 startup clock checks also pass.
- The existing serial-example build script links 80 Debug/Release example
  configurations across C2/C3/C5/C6/C61.
- Temporary C3/C6 MMIO checks verify reset mapping, repeated reads, unsupported
  retention results and preservation of the interrupt mask around CPU sleep.
- No board execution or sleep-current measurement was performed. Hardware
  testing remains on SAMD11 until the maintainer selects another board.

Register references are Espressif ESP-IDF v5.5.1 per-chip clk_tree_ll.h,
reset_reasons.h, system_reg.h, pcr_reg.h, rtc_cntl_reg.h and lp_clkrst_reg.h.

## ESP32-C3/C6 startup clock preservation and ROM timing

C3 startup previously selected PLL regardless of the root left by ROM. It now
changes only the CPU tap when PLL is already selected. XTAL, RC_FAST and their
dividers are preserved. The default PLL tap remains 80 MHz, with
ESP32C3_CPU_160MHZ selecting 160 MHz. Startup does not power up, calibrate or
stop a PLL.

Both C3 and C6 startup now read back the resulting CPU frequency and update
IOsonata's cycle-delay values immediately. They also call the target ROM
ets_update_cpu_frequency entry so later ROM delays use the same CPU MHz. The
C3/C6 linker scripts supply the respective ROM addresses as Esp32RomClockUpdate.
Invalid clock selections do not overwrite the ROM timing value. Clock getters
remain read-only with respect to ROM timing state. Fractional MHz values use
the nearest integer supported by the ROM interface.

This follows the existing startup ownership rule: ROM initializes the crystal,
PLL and flash interface; IOsonata changes the supported CPU tap without
reconfiguring the bus or peripheral roots. Watchdogs remain disabled by startup.

Validation:

- All 200 C3 and 202 C6 library sources compile in Debug and Release.
- The default and 160 MHz CorePowerTest variants link in both configurations
  for both chips, eight images total. Their ROM symbols resolve to 0x40000588
  on C3 and 0x40000048 on C6, matching the Espressif v5.5.1 ROM linker files.
- 24 ESP32 Python checks pass. The clock model covers C3 default/160 MHz
  startup, C6 CPU-divider choices, non-PLL entry, invalid source handling and
  the MHz value passed to the ROM function.
- C2/C5/C61 clock checks pass at O0 and Os. All 80 serial-example builds pass.
- These are compiler/linker and host-register checks, not board execution.

Runtime root-clock switching, low-frequency oscillator startup/calibration and
PMU sleep remain pending. The vendor review confirms that C3 Deep-sleep also
needs revision-dependent regulator settings and RTC-memory wake-stub handling;
C6 uses a different PMU sequence. CPU WFI must not be presented as either
Light-sleep or Deep-sleep.

References: Espressif ESP-IDF v5.5.1 components/esp_rom/esp32c3/ld/esp32c3.rom.ld,
components/esp_rom/esp32c6/ld/esp32c6.rom.ld, the per-chip rtc_clk.c and C3
rtc_sleep.c in components/esp_hw_support/port.

## ESP32-C3/C6 low-frequency clock selection

`SystemLowFreqClockSelect()` now uses the TIMG0 calibration unit to measure
low-frequency sources against the 40 MHz crystal before switching the RTC mux.
It follows the existing target-level oscillator selector and descriptor pattern;
the generic API is unchanged. `SystemOscInit()` applies the application oscillator
descriptor. Startup does not call it automatically. Call the boolean selector
when the application needs to handle a missing crystal or failed calibration.

Supported sources:

| Target | Internal RC | External crystal |
| --- | --- | --- |
| C3 | Nominal 136 kHz RC_SLOW | 32.768 kHz XTAL32K |
| C6 | Nominal 136 kHz RC_SLOW or 32.768 kHz RC32K | 32.768 kHz XTAL32K |

An RC request selects the nearest available nominal frequency; zero selects
RC_SLOW. The default C3/C6 descriptors now specify 136 kHz. A crystal request
must specify 32768 Hz or zero. TCXO input and C3 RC_FAST/256 are not implemented.
The RC descriptor reports the measured frequency. The crystal descriptor keeps
its nominal frequency, while the measured period is stored in the hardware's
Q13.19 calibration word for subsequent RTC timing.

This is an initialization operation: call before RTC timekeeping, LP-core
firmware, modem timing or low-power wake scheduling. GPIO0/1 must be reserved
for the crystal. Output, interrupt, GPIO input-matrix and C6 LP-pin assignments
are checked, but these checks do not constitute tracking of every possible
analog or direct-pin consumer. Active RTC/LP watchdogs, configured wake sources,
timer alarm interrupts and C6 modem clock use cause the call to fail. Existing
TIMG timer counters and watchdog configuration are not changed or reset.

One-shot calibration and application-configured continuous calibration are not
interrupted. The hardware-reset continuous configuration can be paused with a
bounded timeout and is restored afterward. Temporary bus gating, digital clock
routing and the C6 shared 32 kHz calibration selection are restored. A missing
or out-of-range oscillator leaves the previous RTC mux, calibration period and
oscillator descriptor unchanged, and restores oscillator configuration/power.
A successful selection leaves previously running oscillators enabled.

The call masks interrupts throughout. Its delay/polling budgets total less than
121 ms, including a 100 ms startup wait for a newly enabled crystal. It should
not be called from an interrupt or during latency-sensitive operation. It does
not enable a watchdog. PMU Light-sleep and Deep-sleep remain unimplemented.

Validation:

- All 201 C3 and 203 C6 library translation units compile in Debug and Release
  with RISC-V GCC 15.2.0, using the managed project source lists and settings.
- CorePowerTest links for both targets in Debug and Release, both normally and
  with SystemOscInit explicitly retained so the new code is checked by the
  linker. Release flash text increases by 976 bytes on C3 and 1204 bytes on C6
  in the latter case. Unused LF code is discarded.
- Temporary production-source MMIO checks pass at O0 and Os for both chips:
  RC/crystal selection, repeat selection, reset calibration pause, busy
  calibration rejection, hardware timeout, stuck polling, invalid readings,
  rollback, register preservation, clock/pin ownership and interrupt state.
- The existing 24 ESP32 Python checks, 20 GPIO checks and all 80 serial-example
  links pass. No new demo or test framework was added.
- No board execution was performed. Hardware testing remains on SAMD11 until
  the maintainer selects another board.

Register references: Espressif ESP-IDF v5.5.1 per-chip clk_tree_ll.h,
rtc_clk.c, rtc_time.c, timer_group_reg.h, rtc_cntl_reg.h, lp_clkrst_reg.h,
lp_aon_reg.h, pmu_reg.h, pcr_reg.h and the C6 modem_lpcon_reg.h/reg_base.h.

## ESP32-C3/C6 direct CPU source and APB selection

`SystemCoreClockSelect()` now selects the 40 MHz crystal directly or RC_FAST
(nominal 17.5 MHz on C3, 20 MHz on C6). XTAL accepts 40000000 or zero; RC
requests use the one available oscillator rate. External clock input is not
implemented. This follows the existing target-level source selector and
active-consumer checks, without changing the generic API.

The operation keeps existing PLL and flash clock settings. It does not start,
recalibrate or select PLL. A return to the PLL-based startup CPU rate is not
provided by this selector. C3/C6 crystal-based peripherals continue to use the
physical 40 MHz crystal even when the CPU descriptor reports RC_FAST. UART,
SPI, I2C, clock readout and LF calibration now account for that distinction.
C3 retains its nominal RC_FAST rate. C6 measures RC_FAST against XTAL on
each RC selection and uses the result in the descriptor, CPU/APB getters and
IOsonata cycle delays. Calibration failure leaves the prior selection unchanged.
There is no background drift tracking; ROM timing retains integer-MHz rounding.

`SystemPeriphClockSet(0, rate)` selects the nearest supported APB rate:

- C3 uses the shared CPU/APB divider on XTAL or RC_FAST, with a minimum CPU
  rate of 1 MHz for the existing delay/ROM timing implementation. While PLL
  feeds the CPU, APB remains fixed at 80 MHz and the call returns that rate.
- C6 selects AHB divided by 1, 2 or 4, preserving CPU and AHB dividers. Invalid
  root/divider configurations are rejected. Other indices and zero return 0.

A change updates IOsonata cycle delays and ROM CPU timing before restoring the
caller's interrupt mask. C3 also updates the ROM APB frequency storage word.
The root mux write/readback runs from instruction RAM. Failed power-enable,
mux or divider writes leave the prior configuration and descriptor in place.
The crystal must already be running; the function does not restart an external
oscillator that another owner has shut down. RC_FAST startup waits 100 us to
cover the vendor's 50 us minimum despite integer-MHz delay rounding. Calls
require the current CPU rate to be at least 1 MHz.

Call before starting modem or LP-core firmware. Enabled peripheral gates are
checked conservatively; SPI/I2C and other affected peripherals must be released.
TIMG counters and watchdogs must be stopped. UARTs using XTAL may remain on;
other UART sources block a change. No watchdog is enabled or fed by these APIs.
An unchanged APB request does not require stopping its users.

C3 Timer Group setup now calculates its divider from the actual APB rate.
Reinitialize an existing stopped C3 timer after changing APB, before restarting
it. C6 Timer Groups retain their independent 80 MHz PLL source. These calls do
not change their counters, alarms or watchdog configuration.

Validation:

- All 202 C3 and 204 C6 managed-library translation units compile in Debug
  and Release, including a full rebuild after the shared header change.
- Default CorePowerTest images and images retaining all three clock APIs link
  for both chips and configurations. The root mux helper is in instruction RAM;
  retaining it does not pull in the timer driver.
- Temporary production-source register models pass at O0 and Os for source
  changes, nearest rates, unsupported inputs, active consumers, UART crystal
  operation, invalid dividers, failed writes, rollback and interrupt state.
  LF calibration checks also pass with the CPU descriptor set to RC_FAST.
- Existing checks pass: 24 ESP32 Python checks; 10 timer, 10 SPI and 10 I2C
  model configurations; 30 C/C++ timer/demo links; 80 serial-example links.
  Existing timer/SPI checks were extended for changed CPU/APB rates.
- These clock-selection checks did not include board execution. C6 sleep
  hardware results are recorded below; runtime clock selection still needs
  hardware validation.

References: Espressif ESP-IDF v5.5.1 per-chip rtc_clk.c, clk_tree_ll.h,
system_reg.h, pcr_reg.h, rtc_cntl_reg.h, pmu_reg.h and UART register definitions.


## ESP32-C3/C6 RTC and PMU sleep status

The target libraries contain `system_sleep_esp32c3.c` and
`system_sleep_esp32c6.c`. These implement RTC/PMU sleep and supersede the
CPU-WFI-only implementation described in the earlier sections. The generic
API is unchanged.

### C6 hardware results (November 16, 2026)

| Mode | Wake input | Result |
| --- | --- | --- |
| Light-sleep, resume execution | BOOT/GPIO9 | Repeated wake passed with UART output drained and Disable() called; 128 static and 128 stack bytes matched after each wake |
| Deep-sleep, reset on wake | External GPIO3 switch to GND | Seven consecutive wake/restart cycles passed on DevKit N8 with UART output drained and Disable() called and serial diagnostics removed |

Every reported Deep-sleep restart identified reset reason `0x100` (pin wake).
`Start count 1, 0 retention registers` is expected: the port exposes no
retention registers. The counter is initialized again at startup.
The SRAM sample result applies to Light-sleep only and is not a full SRAM
or Deep-sleep retention test. Sleep current has not been measured. These runs
used empty UART Disable/Enable hooks: they drained output but did not gate UART
clocks. The new hooks gate serial and bus clocks, preserve configuration and
restore UART interrupt enables on resume. PowerOff additionally clears FIFOs,
releases pins and interrupt routing, and requires Init before reuse. The updated
CorePowerTest uses UART PowerOff before Deep-sleep. At `17d081bd1`, CoreClockTest
passed three cycles and nine BOOT wakes with these Disable/Enable hooks, and
CorePowerTest passed three consecutive GPIO3 wake/restart cycles after UART
PowerOff. Every Deep-sleep restart reported pin-wake reason 0x100. These results
validate both UART shutdown paths on the board; current remains unmeasured.

The cleaned C6 CorePowerTest defaults to GPIO3 Deep-sleep. To exercise the
validated Light-sleep path, set `WAKE_BUT_PIN` to 9 and
`POWER_DEMO_RESUME_WAKE` to 1 in its `board.h`. UART0 uses TX16/RX17 at
1 Mbaud through the board's USB-to-UART bridge. GPIO9 is not a Deep-sleep
wake pin. The demo waits for the switch to remain released before sleeping.

### C6 implementation

The Light-sleep entry preserves SRAM, the stack, crystal and PLL. It saves
PMU and clock state, switches to XTAL from instruction RAM with the flash
cache suspended, and restores clocks and cache before returning. The linker
keeps the application stack below ROM-reserved SRAM. GPIO wake preparation
preserves the pad configuration and restores it after resume.

Deep-sleep initializes the active, modem and sleep PMU states, hands regulator
control to the PMU and requests HP power down. Regulator bias uses chip eFuse
values where available. RC_FAST is measured before entry; PMU fast-clock waits,
including crystal startup, use that measured rate. LP IO routes GPIO3 through
EXT1. EXT1 remains enabled as a wake source without also rejecting the
non-returning Deep-sleep request. Wake restarts the application through ROM.

The earlier button checks, first-cycle WFI comparison, PMU serial stage output,
register dumps and HP reset experiments have been removed. Their earlier
failures are superseded by the repeated hardware results above.

C6 build validation after cleanup:

- All 206 managed-library source files compiled in Debug and Release with
  RISC-V GNU 15.2.0, using the source lists and settings from the Eclipse
  managed projects. These were command-line builds, not Eclipse invocations.
- CorePowerTest linked in both Deep-sleep and Light-sleep configurations.
- The instruction-RAM sleep entry was inspected in both builds; its calls
  during cache suspension resolve to ROM clock/cache functions.

| Configuration | text | data | bss |
| --- | ---: | ---: | ---: |
| Deep-sleep Debug | 44586 | 584 | 4312 |
| Deep-sleep Release | 21463 | 564 | 4044 |
| Light-sleep Debug | 37678 | 576 | 2484 |
| Light-sleep Release | 15335 | 480 | 1884 |

These results apply to the cleaned implementation at `f834cf5e0`.

### Supported behavior and remaining work

`SystemSleep()` attempts resumable Light-sleep using pins configured with
`SystemWakePin()`. If preparation cannot establish the required pin or
controller state, it falls back to CPU WFI. A wake input already active
prevents hardware sleep preparation. Wake-owned GPIO interrupt configuration
is restored on return.

`SystemPowerOff()` requests reset-on-wake Deep-sleep and never returns.
C3 accepts GPIO0-5 and C6 accepts GPIO0-7. GPIO0/1 are unavailable when used
by the 32 kHz crystal. Pins owned by another interrupt, peripheral or C6 LP IO
user are refused. A rejected Deep-sleep request remains in an instruction-RAM
wait loop; it does not resume flash execution after potentially powering down
flash or PLL.

Callers must drain and disable UART/DMA and stop incompatible peripheral,
modem and flash transactions. The power functions do not enable a watchdog.
RTC scratch registers remain reserved for ROM and clock control:
`SystemRetRegCount()` returns zero and `SystemRamRetain()` returns false.
No Deep-sleep retained-RAM section or startup exemption is implemented.
RTC timer wake is not configured by these power functions.

The November 17 C6 CorePowerTest retest recorded five consecutive Deep-sleep
wake/restart cycles, each reporting SYSTEM_RESET_WAKE_PIN (0x100). Start count
remained one with zero exposed retention registers, as expected. The initial
start reported zero; the submitted log did not include the raw reset register.

Review against ESP-IDF v5.5.1 C3/C6 ROM rtc.h found raw reset code 0x15
(USB_UART_CHIP_RESET) missing from the translation. It now maps to
SYSTEM_RESET_DEBUG alongside USB-JTAG reset 0x16. This describes a host reset
through the USB Serial/JTAG interface, not wake from USB power. It may explain
the initial zero, but that cause is not confirmed without the raw register.
No sleep entry, wake configuration or flasher reset sequence changed.

Both complete libraries rebuilt (204 C3 and 206 C6 sources per configuration),
and CorePowerTest linked in Debug/Release for Light-sleep and Deep-sleep: eight
links. Demo compilation used GNU C++11. Temporary production-function checks
at O0/Os compared every low-byte reset value and six-bit wake combination;
only USB-serial reset translation changed. Pin, timer and combined wake checks
passed. The new reset label has not yet been checked on hardware.

Remaining work is separate from the completed C6 GPIO wake validation:

- C6 CoreClockTest at `e5044c35f` passed three cycles of XTAL/RC_FAST switching,
  APB changes, LF selection, UART TX and timer interrupts with zero failures.
  Calibrated RC_FAST timing at `6d3947a30` also passed three cycles with zero
  failures: RC_FAST 16134719-16136308 Hz, 250 ms delay 249969-250037 us.
  Combined clock selection and Light-sleep resume at `b79ea4253` passed all
  three cycles and nine BOOT wakes with zero failures. CPU/APB restoration,
  LF descriptor preservation and post-wake timers passed for XTAL/RC_FAST and
  RC_SLOW/RC32K. Post-wake 250 ms samples were 250019-250078 us with 25 IRQs.
  The later run at `17d081bd1` passed all nine wakes with UART clock gating:
  post-wake delays were 250049-250180 us with 25 IRQs. One initial UART line
  was truncated before sleep; the cause remains unresolved.
  The November 17 receive-enabled run completed with zero failures after
  power-cycling the board. The submitted excerpt covers the end of cycle 2
  and all of cycle 3: five BOOT resumes and five polling UART RX checks pass.
  Both delay units pass, with 250 ms samples at 249979-250050 us and 24/25
  IRQs. Build configuration was not specified. Other peripheral operation,
  interrupt/DMA UART RX and short nanosecond delays remain untested.
  RESET had failed to start CoreClockTest, CorePowerTest and LedDemo after flashing;
  one power cycle restored execution and RESET then worked normally. The
  transient native-USB flashing/reset cause is unconfirmed; no recurring
  post-flash failure or flasher fix has been established.
- Measure Light-sleep and Deep-sleep current with board circuitry accounted for.
- C3 CorePowerTest passed six consecutive GPIO9 Light-sleep wakes with the
  128-byte static and 128-byte stack SRAM samples preserved. GPIO3 Deep-sleep
  passed four manual wake/restart cycles with reset reason 0x100; the board
  stayed asleep between manual wake events. RTC fast-memory retention is not
  validated or exposed as application retention registers.
- C3 CoreClockTest after `04d1522a8` passed all three cycles and nine Light-sleep
  wakes with zero failures. XTAL, calibrated RC_FAST, shared CPU/APB division,
  RC_SLOW, restored clock descriptors and timer IRQs passed. RC_FAST measured
  20805079-20810365 Hz; 250 ms samples measured 250065-252360 us with 25 IRQs.
  A later receive-enabled run on November 17, following the C++11 demo fix,
  passed three cycles, nine BOOT wakes and all ten polling UART RX prompts
  with zero failures. All 90 delay samples reported 25 interrupts: msDelay
  250018-250478 us and nsDelay 250018-250137 us. RC_FAST measured
  20811686-20815652 Hz; RC_SLOW measured 135730-136293 Hz. The build
  configuration was not identified in the submitted log. Sleep-current and
  interrupt/DMA UART RX measurements remain pending. Dots before startup
  output remain unexplained.
- Runtime PLL startup/selection and Deep-sleep RAM retention are unsupported;
  they must not be reported as completed capabilities.

Implementation references: Espressif ESP-IDF v5.5.1 per-chip RTC/PMU sleep,
PMU initialization, clock calibration and ROM register-I2C implementations.

Cross-target demo projects and build coverage are listed in
[Power and clock demos](power-clock-demo-matrix.md).
