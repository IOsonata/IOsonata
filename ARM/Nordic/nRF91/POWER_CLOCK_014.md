# nRF91 power and clock port — IOsonata 0.14

## Targets and scope

The nRF9160 and nRF91x1 libraries use the Nordic shared `system_clock_nrfx.cpp`,
`system_power_nrfx.cpp` and RTC/HF timer drivers. The 0.14 port is not
hardware-complete merely because these sources build.

- Core clock: fixed 64 MHz; PCLK16M is 16 MHz. Requests for unsupported
  clock divisors return the hardware frequency rather than changing it.
- HF clock and LF clock: the shared system Enable/Disable owners count requests.
  RTC timers request LFCLK through those APIs, not a private oscillator owner.
- System ON IDLE: `SystemSleep()` preserves SCR and the original interrupt mask,
  and returns after an interrupt. System OFF remains reset-on-wake.
- `sDelay()`: uses an available low-frequency RTC with WFE and returns it to
  the timer pool. On nRF91, the clock owner's observed cold-start polling
  duration is compensated, as on the nRF5340 application core.
- nRF91 LFRC has no CLOCK CAL task and can run far from 32768 Hz.
  The nRF91 RTC driver measures raw LFRC counter ticks over a calibrated
  100-ms high-frequency CPU delay on first RTC initialization per LFCLK
  selection. It caches the measured input rate until the next source
  selection, and uses that rate for RTC prescalers, timer timestamps,
  and trigger compare periods. Neither RTC ownership nor core
  delay-loop factors are changed by the measurement.
- A requested LFRC remains LFRC; there is no silent substitution with
  LFXO, synthesized LFCLK or mutation of `g_McuOsc`. The measurement is
  performed in foreground with interrupts enabled and uses no extra
  RTC/HF timer peripheral, heap, or watchdog. Failed/unreasonable samples
  fail RTC initialization rather than exposing fictitious timestamps.
- LFXO remains 32768 Hz and retains its accuracy advantage. The existing
  sDelay implementation continues to require LFXO on nRF91, because its
  low-power call must not insert a foreground RC measurement busy wait.
  An RC rate estimate has limited validity as temperature changes and
  inherits the uncertainty of the CPU high-frequency time reference;
  long-duration accurate timekeeping should use LFXO. `SystemLowFreqClockCalibrate()`
  still reports hardware trimming unsupported: measuring RC rate is not
  an oscillator trim.
- nRF91 LFCLK start completion is checked against the actual LFCLKSTAT
  source, not just the LFCLKSTARTED event. Existing modem ownership must
  be preserved; the modem requires LFXO.

## nRF9160 hardware observation (2026-11-23)

The first CoreClockTest transcript reports four LFRC-only timing failures:

| Timer | CPU delay | RTC-reported elapsed | Independent HF elapsed |
| --- | --- | --- | --- |
| RTC0 | msDelay 250 ms | 342224 us | 250215 us |
| RTC0 | nsDelay 250 ms | 342377 us | 250214 us |
| RTC1 | msDelay 250 ms | 342285 us | 250213 us |
| RTC1 | nsDelay 250 ms | 342346 us | 250212 us |

These imply about 44.83 kHz of physical RTC tick rate when the driver uses
a nominal 32768 Hz. The RC is documented by Nordic as having large
frequency variation (nominal 32768 Hz, 30% tolerance), and the observed
ratio is about 36.8% above nominal. The physical oscillator/source register
state and test conditions therefore require investigation; do not waive
these four failures or label them as PASS.

After restoring the crystal source, RTC0/RTC1 measured approximately
250152 us for a real 250156–250158 us. All three 16 MHz HF timers also
passed. CPU delay calibration and 64 MHz / 16 MHz clock reporting passed.
That transcript predated the new nRF91 `sDelay()` integration.

## nRF9160 DK CorePowerTest wake button

The nRF9160 DK's default hardware routing connects Button 1 to GPIO P0.06,
active low. The nRF9160 CorePowerTest board file now defines this button
as a wake input; the generic test applies the required pull-up resistor,
configures GPIO SENSE, and verifies wake after System OFF without a debugger.
Button routing depends on the nRF52840 board-controller firmware, so test
on the DK with its standard routing. No Thingy:91 button mapping is
assumed. The existing VCOM0 UART pins are unchanged.

Prior validation of three successive System OFF GPIO wakes on nRF91x1
applies to that target's earlier bare-metal power demo, not to the
updated nRF9160 CorePowerTest or to modem-active shutdown.

## nRF9160 delay divisors (2026-11-23)

CoreClockTest on the nRF9160 at 64 MHz measured `msDelay(100)` at
100,010 us and `nsDelay(100 ms)` at 100,011 us with the 16,000,000 and
3,000,000 divisors, 3,999,600 us loops/s and 21,330,986 ns loops/s.
`system_nrf91.c` now uses the measured divisors 16,001,600 and 3,000,330.
The file is shared by the nRF9160 and nRF91x1 libraries; the values come
from one nRF9160 run and are not yet measured on an nRF91x1. Rerun
CoreClockTest to confirm both delays at 100,000 us.

## nRF9160 CoreClockTest hardware sign-off

Two consecutive nRF9160 CoreClockTest executions from the latest hardware
UART log completed with **0 failures** after the LFRC Disable/Enable cache
correction in commit `220472dd`. This closes the previously recorded four,
then two, LFRC timer accuracy failures for this tested board and build.

| Check | First run | Second run |
| --- | --- | --- |
| Final CoreClockTest result | 0 failures | 0 failures |
| Measured/report LFRC timer rate | 44,730 Hz | 44,740 Hz |
| RTC0 LFRC msDelay/nsDelay, 250 ms | PASS/PASS | PASS/PASS |
| RTC1 LFRC msDelay/nsDelay, 250 ms | PASS/PASS | PASS/PASS |
| RTC0/RTC1 LFXO and HF timer matrix | PASS | PASS |
| sDelay cold/warm/reuse | PASS/PASS/PASS | PASS/PASS/PASS |
| HF/LF ownership and RTC trigger lifecycle | PASS | PASS |

Independent HF timer measurements of the running LFRC were approximately
44.75–44.80 kHz; the RTC timer reports an independently sampled ~44.73–44.74
kHz and both 250-ms delays now remain within the unchanged 2% accuracy
requirement. The configured LFXO measures approximately 32,768 Hz. The
64-MHz CPU, 16-MHz peripheral clock and delay-loop calibrations pass.

**Scope:** This demonstrates compensation of RTC elapsed time and trigger
periods under the measured hardware conditions; it does not trim the RC
oscillator, establish LFRC frequency compliance with the datasheet, or
prove long-term accuracy under temperature variation. A more accurate time
reference should continue to use LFXO. It is not evidence of modem-active
clock control, System ON wake, or System OFF GPIO wake.

## nRF9160 CorePowerTest hardware validation

Four consecutive nRF9160 DK CorePowerTest boots reported **0 failures**:
the first boot performed 14 checks and the next three each performed 15
checks, for 59 completed checks total. The source UART transcript confirms:

- Three resumable System ON sleep/timer-wake cycles per boot:
  **12/12 automatic wakes PASS**.
- After every wake, CPU/peripheral clock rate, UART rate, and
  static/stack RAM continuity checks PASS.
- Three subsequent Button 1 (P0.06) wakes from System OFF:
  reset reason `0x00000100`, retention marker and
  `System OFF reset-on-wake: PASS` on each restart.
- GPIO wake configuration, invalid-index rejection, disable and rearm,
  reset-reason stability, and GPREGRET read/write/invalid-index checks
  all PASS.
- The reported capability bitmap is `0x0000003f`. Reduced CPU
  power mode is UNSUPPORTED and a reserved System OFF SRAM retention
  test region is not configured, so SRAM retention across System OFF
  is **not validated**.
- Every run reports `LogDiag fifo=0, uartfifo=8, evicted=0,
  uartRejected=0x42`. The diagnostic snapshots output queues before
  the final report drains. The rejected UART submissions reflect
  backpressure, not independently verified dropped UART bytes; preserve
  this observation for UART/SysLog performance review.

This covers bare-metal System ON sleep/resume and System OFF/button
wake on the nRF9160 DK under the tested conditions. Modem-active
shutdown, current measurements, optional SRAM retention and the
nRF91x1 CorePowerTest are outside this result.

## Remaining validation

1. Run source tests: `python3 tests/nordic/check_nrf91_power_clock_014.py`.
2. Run Debug and Release managed builds for `nrf9160` and `nrf91x1`
   through `tests/nordic/build_managed_power_clock.py`.
3. nRF9160 CoreClockTest hardware run: **complete, 2/2 runs with
   zero failures**, including both LFRC RTCs, crystal-source RTCs,
   16-MHz timers, sDelay, shared clocks and the Disable/Enable lifecycle.
   Verify the LFCLKSTAT source explicitly if investigating physical LFRC
   datasheet tolerance; the UART timing result does not read it directly.
4. Run CoreClockTest on nRF91x1 hardware. The nRF9160 result does not
   establish that other nRF91 variants are validated.
5. nRF9160 CorePowerTest hardware run: **complete**, 4 boots and
   59 checks with zero failures; 12/12 automatic timer wakes and
   3/3 Button 1 System OFF resets passed. No reserved SRAM retention
   region was configured; modem-active System OFF remains untested.
6. Run nRF91x1 CorePowerTest and CoreClockTest on hardware. Revalidate
   the shared power and clock paths across its intended MCU variants,
   and verify Debug/Release managed configurations.
7. Preserve already validated nRF52, nRF5340 and nRF54L behavior and
   investigate UART TX backpressure separately if it affects throughput.

No default watchdog is enabled. No target-specific code belongs in the
shared CoreClockTest or CorePowerTest examples.


## LFRC rate compensation implementation

`system_clock_nrfx.cpp` exposes two nRF91-only internal helpers to the
RTC driver: a source-configuration generation counter and a read-only
physical LFRC status query. A successful `SystemLowFreqClockSelect()`
invalidates the previously measured RC rate. Neither function owns
or starts clocks.

`timer_lf_nrfx.cpp` starts a newly initialized RTC temporarily with
PRESCALER=0, measures the oscillator frequency using a 100-ms
foreground high-frequency delay, and restores the timer's requested
frequency. The input clock is read from actual LFCLKSTAT, not only
the board's nominal `g_McuOsc` descriptor. Subsequent RTCs reuse the
same measurement for that LFCLK configuration. After the measurement,
`TimerSetFrequency()` uses the measured base rate and reports the
achievable divided frequency. The existing generic Timer API converts
microseconds and compare periods using that reported frequency.

When a timer is kept initialized but disabled across a source change,
reinitializing that timer refreshes its effective frequency and
compare setup. Source switches remain disallowed while LFCLK is in use.
The 100-ms software observation cannot permanently stabilize LFRC:
thermal or supply drift after measurement still affects long-running
timekeeping. Production modem use must retain LFXO.


## LFRC Disable/Enable lifecycle correction

The nRF9160 CoreClockTest after the first LFRC rate-measurement patch
reproduced a precise state bug on both RTC0 and RTC1. The first 250-ms
measurement passed at 44720 Hz, but after TimerDisable stopped the last
LFCLK owner the next TimerSetFrequency reported 32768 Hz. RTC raw
ticks still measured approximately 44740-44755 Hz, causing two
remaining 250-ms timing failures. Both LFXO and HF timer paths passed.

The measured input rate must be associated with the **selected** LFRC
source and the selection generation, rather than the LFCLK running bit.
A stopped LFCLK does not mean its requested source changed. The
internal `nRFxLfclkUsesRc()` query returns true when RC is selected
and LFCLK is stopped or physically supplied by RC. If an external
owner is supplying running LFXO instead, the timer uses nominal
32768 Hz. `nRFxRtcBaseFrequency()` keeps the measured rate across
balanced TimerDisable/Enable calls and invalidates it only when the
clock configuration generation changes.

The generic CoreClockTest keeps the 2% assertion; no hard-coded
44720-Hz frequency, synthesized clock, or test waiver is introduced.
The physical LFRC frequency is not trimmed.


## nRF9160 timer lifecycle audit (0.14)

The nRF9160 implements five generic hardware timers: RTC0/RTC1 at
DevNo 0/1 and TIMER0/TIMER1/TIMER2 at DevNo 2/3/4. Two full
CoreClockTest runs with zero failures validated both RTCs, all three
16-MHz TIMERs, interrupt counts, single/continuous callbacks, trigger
reuse, clock ownership and accurate LFRC/LFXO time conversions.

The subsequent timer audit found additional **source-level defects**
not exercised in that hardware run. This batch fixes the following
on the nRF91 path:

- HF TIMER reinitialization: TimerInit accepts an explicitly selected
  timer after TimerDisable. A running timer rejects TimerInit. It checks PEI connections and
  counter mode before replacing the configuration.
- Frequency changes: for nRF91 HF TIMER and RTC, prevalidate all
  armed normal compares against the proposed frequency before
  touching hardware. Successful changes reset the counter origin
  and rephase one-shot/continuous compare periods to the new tick
  frequency. Existing PEI bindings reject frequency changes.
- RTC reset while LFCLK stopped: after changing frequency on a
  disabled RTC, defer counter reset and trigger rearm until the
  next Enable has started the selected LFCLK.
- Compare bounds: a period that exceeds the 32-bit TIMER or 24-bit
  RTC hardware compare range fails instead of silently truncating
  or clamping to a shorter interval.
- Clock source requests: nRF91 RTC explicit LFRC/LFXTAL requests
  follow the selected LFCLK source (not an immutable board default).
  HF TIMER has a fixed peripheral clock and rejects explicit
  per-timer oscillator source selection.

These are code changes awaiting a managed rebuild and hardware
regression. The two previously recorded zero-failure CoreClockTest
runs predate this batch and must not be used to sign off its new
edge cases.

**Open capabilities / limitations:**

- Extended 32-bit HF timer software rollover is folded on observed
  counter reads or IRQs; without either for a full counter period,
  a wrap may be missed. At 16 MHz that period is 268.44 seconds.
  The 24-bit RTC also requires servicing its overflow interrupt.
- The generic Timer-to-PEI convenience API currently returns
  unsupported for nRF91; DPPI routing and event-to-task tests have
  not been validated. No assumption is made that an nRF52 PPI
  backend is interchangeable with nRF91 DPPI.
- The existing shared `counter_nrfx.cpp` implementation is not
  linked into the nRF9160 managed IOsonata library; Counter-mode
  behavior has not been validated on this target.
- Modem-active peripheral ownership and Debug/Release build
  configurations remain separately unverified.

Do not change generic CoreClockTest source to add nRF91-only
conditionals. Use the new nRF91 source/model regressions and a
hardware follow-up for active compares, duplicate handles and
long interval rejection.

## CoreClockTest TIMER0 handle reuse regression

The nRF9160 DK previously reported a TIMER0 initialization failure
because the driver rejected an explicit TimerInit call when the
TimerDev_t address changed. The driver now accepts reinitialization
of an explicitly selected TIMER or RTC after TimerDisable. Automatic RTC selection still
requires a free RTC. Reinitialization on the DK awaits verification.

## TimerPowerOff and timer device assignment

TimerPowerOff() powers down the timer. TimerInit() is required to
start it again. TimerDisable() preserves the configuration for
TimerEnable(), including when TimerInit() selected the RTC automatically.
sDelay() powers off its temporary RTC when the delay finishes.

On nRF91, RTC0/RTC1 and TIMER0/1/2 support PowerOff. Disconnect
any PEI connections before powering down the timer.

The same generic CoreClockTest is used across every MCU. It tests
that the first HF timer can be powered off and initialized again
with the same TimerDev_t. The source file does not contain any
nRF91-specific code.

The earlier nRF9160 CoreClockTest and CorePowerTest runs predate
TimerPowerOff and cannot validate it. Rebuild the nRF9160 library
and repeat the hardware tests, including LFRC/LFXO timer timing,
HF/LF shared clock requests, sDelay, PowerOff followed by Init
on the same timer structure, and rejection of repeated PowerOff.

See docs/architecture/timer-power-off.md for the other MCU timer
drivers and the remaining hardware tests.
