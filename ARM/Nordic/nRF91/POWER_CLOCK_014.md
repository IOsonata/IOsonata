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

## Remaining validation

1. Run source tests: `python3 tests/nordic/check_nrf91_power_clock_014.py`.
2. Run Debug and Release managed builds for `nrf9160` and `nrf91x1`
   through `tests/nordic/build_managed_power_clock.py`.
3. Run CoreClockTest on nRF9160 using the updated code and inspect
   LFXO cold/warm `sDelay`, ownership and LFRC timing separately.
4. Validate the new nRF91 LFRC software rate measurement. Run both
   RTC0/RTC1 on LFRC in CoreClockTest, compare the reported calibrated
   timer frequency with its independent raw RTC tick-rate diagnostic,
   and require all four 250-ms/2%-tolerance checks to PASS. Confirm
   the active LFCLKSTAT source is RC during RC tests and LFXO after
   restoration. Repeated RC selections must remeasure the rate.
   The earlier four failures remain evidence until hardware retest.
5. Run CorePowerTest on nRF9160 and nRF91x1, including timer wake,
   GPIO/System OFF wake, retention, UART restoration, and reset reasons.
   A bare-metal System OFF test does not validate modem-active shutdown.
6. Preserve all previously validated nRF52, nRF5340 and nRF54L behavior.

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

