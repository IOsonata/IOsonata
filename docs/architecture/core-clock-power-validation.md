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

- Startup CPU/peripheral frequencies, configured/RC source selection and
  restoration, peripheral-clock bounds and requests.
- Shared HF/LF clock Enable/Disable ownership and CPU reduced-speed mode
  where supported.
- DWT only as an optional **calibration measurement**; independent HF timer
  measurement validates real `msDelay` and `nsDelay` call sites, with
  interrupt masking during that timing. No DWT in runtime delays.
- Calibration factors at highest/lowest observed CPU speed, with an
  interpolated value when two distinct speeds are supported.
- `sDelay(1)` cold and warm, with end-to-end HF timer reference. A further
  `sDelay(1)` call after releasing the LF clock verifies temporary
  timer reuse; this is a separate timing sample.
- **All enumerated LF/HF timers:** initialize, set frequency, Enable/Disable,
  verify elapsed time, counter and periodic IRQs across LF clock selection
  and restoration. Independently compare LF timing against the HF reference.
- **Timer implementation:** verify an available trigger, single-shot callback,
  continuous callback, counter progression, stopping callbacks, and trigger
  slot reuse using the generic Timer API. These are automated assertions;
  TimerDemo is not a validation dependency.
- Independent timer clock ownership.
- **After** the timer matrix, only if every advertised LF timer was
  initialized successfully: verify `sDelay(0)` succeeds and that
  `sDelay(1)` refuses to take an initialized-but-disabled timer. Both
  disabled ownership and all-LF-timers-occupied results must be PASS.
  If this exact ownership condition cannot be established, print
  UNSUPPORTED rather than falsely passing it. The test is deliberately
  last: `TimerDisable` is not a generic deinitialization API.

The ownership test uses only the public `TimerInit`, `TimerDisable`,
`TimerGetLowFreqDevCount`, and `sDelay` interface. No Nordic RTC
registers or Claim/Release APIs appear in CoreClockTest.

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

Existing successful nRF52832 and nRF5340 CoreClockTest timings remain valid
for their reported scope. The expanded timer trigger checks have not yet
been run on hardware and must pass before signoff.
