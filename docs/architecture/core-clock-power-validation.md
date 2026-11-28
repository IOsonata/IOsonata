# IOsonata 0.14 — Core Clock and Power hardware acceptance

The generic managed-project firmware sources are:

- `exemples/misc/core_clock_test.cpp` — CoreClockTest
- `exemples/misc/core_power_test.cpp` — CorePowerTest
- `exemples/timer/timer_demo.cpp` — TimerDemo (user-facing learning example)

Both tests are identical across MCU targets. The timer and oscillator implementations
remain in the appropriate MCU drivers. Do not add target-specific test code,
private timer allocation, DWT runtime delay code, or UART flush workarounds.

## Run order on each MCU/core

1. Build CoreClockTest using the existing IOcomposer Eclipse CDT managed
   project (both Debug and Release where supported). Use the normal UART
   at 1,000,000 baud. Run without a debugger attached for sleep tests.
2. Run CoreClockTest after a power cycle. Check its final
   `CoreClockTest complete: 0 failures` line.
3. Build CorePowerTest using the same managed-project workflow. Run it
   without an attached debugger.
4. Let all automatic Standby timer-wake cycles finish. Check the final
   `CorePowerTest complete: 0x00000000 failures` line.
5. If System OFF and a wake button are supported, press the wake button,
   verify the reset-on-wake reason and repeat. Do not treat reset-on-wake
   as an ordinary return from System OFF.

## CoreClockTest coverage

- Startup CPU frequency, CPU reduced-speed mode and performance restoration.
- Shared LF and HF clock enable and disable. For each available timer class,
  use one timer and check that it keeps counting after the test releases its
  clock requests.
- `usDelay`, `msDelay` and `nsDelay` of 100 ms at the highest and the lowest
  CPU frequency, timed by the HF timer with the interrupts off, each within
  2 percent (98,000 to 102,000 us). A port without a low power mode is
  checked at its one frequency.
- Numbered peripheral clock requests, actual rate, readback, restoration
  and invalid indices.
- One calibration request for each configured RC oscillator where supported.
- With `CORE_CLOCK_TEST_DELAY_CALIBRATION`, during porting: at both
  frequencies the delay factors are measured, with DWT where available or
  from the HF timer times. The delays are timed again with the measured
  factors, within 2 percent, before the factors, the divisors and the two
  `CalcDelayFactor` points are printed. The factors of the code are back
  after each measurement. The port puts the printed values in its code,
  and a second run without calibration checks them. A flash-limited MCU can
  leave calibration off.

Timer trigger modes, callbacks, rollover and device power-off belong in the
timer tests. On SAM D, run `python3 tests/samd/run_timer.py` for those drivers.

## TimerTest coverage

`exemples/misc/timer_test.cpp`, the TimerTest project of each target, tests
the timers and the Timer API on hardware. It uses the generic interfaces
only, board.h gives the UART pins. For every DevNo, the low power timers
first:

- Init at the highest rate, `TimerSetFrequency` to half of it gives the
  closest rate and `TimerGetFrequency` returns it, the highest rate again.
- Init of the same DevNo by another `TimerDev_t` is refused while the timer
  runs and while it is disabled.
- The count over 200 ms is within 2 percent of a reference timer, the first
  high frequency timer or another one when that is under test. At the
  highest rate a 16-bit counter wraps many times in the window.
- A single trigger of 10 ms fires once. A continuous trigger of 10 ms fires
  10 times, plus or minus one, in 105 ms, and no more once disabled.
- Enable after Disable counts again. After `TimerPowerOff`, Enable fails and
  another `TimerDev_t` can take the DevNo.

`sDelay(1)`, timed by the first high frequency timer, is within 2 percent:
as the first call after reset, with the low frequency clock held running by
the test, and with every low power timer held, so sDelay takes a high
frequency timer. Each line prints the time and its difference from 1 s.

TimerTest projects exist for the Nordic targets, SAMD21, SAM4LCxC,
SAM4LSxC and SAM4E16E. It uses `UARTprintf` and does not fit the 16 KB
SAMD11D14AM.

Hardware runs, Nov. 27, 2026, reported by the maintainer: validated on the
SAMD11 (CoreClockTest, there is no TimerTest project), SAMD21, SAM4L,
nRF52840 and nRF52832. On SAM D and SAM4L the TimerTest Disable and Enable
check found the TC and TCC counts lost across Disable and Enable, fixed in
3020ba575 and 8158ceee0 before these runs. The SAMD11 also passed separate
TC1, TC2 and TCC0 resume checks and the one- and two-second sDelay checks.

nRF52832 TimerTest, 0 failures, 3 RTC and 5 TIMER:

| sDelay(1) | Time | From 1 s |
| --- | --- | --- |
| First call, LFXO stopped | 1,002,612 us | +2,612 us |
| LF clock held running | 1,000,030 us | +30 us |
| Every RTC held, a TIMER used | 1,000,071 us | +71 us |

The first call was 1,262,331 us before the clock start time was subtracted.
The RTC counts over 200 ms were 200,012 to 200,043 us against TIMER0 at
199,999 us. The 10 ms continuous trigger fired 10 times in 105 ms on every
timer, with a period of 10,009 us on the RTC (the closest 32,768 Hz tick)
and 10,000 us on the TIMER.

### Additional Nordic hardware runs

Each run below finished with zero TimerTest failures. The last three columns
give the signed error in microseconds for a one-second delay. A positive
value means the delay ran long.

| Board and core | Commit | LF / HF timers | First call (us) | LF clock running (us) | All LF timers held (us) |
| --- | --- | --- | ---: | ---: | ---: |
| nRF5340 Audio DK, application | `4e5f27ac4` | 2 / 3 | +2,045 | -337 | +96 |
| nRF5340 Audio DK, network | `4e5f27ac4` | 2 / 3 | -276 | -283 | +192 |
| IBK-nRF54L15 | `4e5f27ac4` | 4 / 7 | -110 | -131 | +400 |
| nRF54LM20A DK | `4e5f27ac4` | 4 / 7 | +35 | +10 | +894 |
| nRF9160 DK | `3a69e3751` | 2 / 3 | +3,572 | +184 | +150 |
| nRF91x1 DK | `3a69e3751` | 2 / 3 | +4,206 | +186 | +150 |

On the IBK-nRF54L15, a separate 64 MHz CoreClockTest run verified the
compiled-in delay factors with calibration disabled. For 100 ms, usDelay
measured 99,994 us, msDelay 99,993 us and nsDelay 100,000 us. The run
finished with zero failures. Commit `3a69e3751` includes those factors.

The nRF54LM20B and SAM4E16E still lack board runs.

## TimerDemo

TimerDemo is a user-facing educational example for learning the generic
Timer API; it is not part of hardware acceptance. Its code is not a
substitute for automated, pass/fail checks in CoreClockTest.

## CorePowerTest coverage

- Reset reason stability and reset-on-wake verification, where available.
- Retention-register access and invalid-index rejection.
- Supported reduced running-power mode, with frequency restoration.
- Repeated interrupt-resumable Standby with **automatic timer wake**;
  CPU/peripheral frequencies, UART rate, and static/stack RAM checked
  following each wake.
- Wake pin configuration, invalid index, disable and rearm.
- System OFF button-wake reset loop on supported boards.

## Acceptance rules

- **PASS:** All supported checks report PASS; both test programs reach
  their zero-failures summary. Supported System OFF wakes produce the
  expected reset-on-wake result.
- **UNSUPPORTED:** Reported only for a capability genuinely absent,
  a missing board wake resource, or an allocation scenario the test
  cannot safely establish. Do not reinterpret a supported failure
  as UNSUPPORTED.
- **FAIL/BLOCKED:** A failure, hang, missing terminal summary, incorrect
  wake state, or compilation error must be recorded before signoff.
- Hardware runs establish behavior only for the tested target/core,
  configuration, and build. Do not generalize one MCU's calibrated
  delay factors or clock support to another MCU.
