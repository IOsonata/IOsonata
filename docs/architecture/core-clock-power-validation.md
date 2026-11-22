# IOsonata 0.14 — Core Clock and Power hardware acceptance

The generic managed-project firmware sources are:

- `exemples/misc/core_clock_test.cpp` — CoreClockTest
- `exemples/misc/core_power_test.cpp` — CorePowerTest
- `exemples/timer/timer_demo.cpp` — TimerDemo (timer-driver functional test)

Both are identical across MCU targets. The timer and oscillator implementations
remain in the appropriate MCU drivers. Do not add target-specific test code,
private timer allocation, DWT runtime delay code, or UART flush workarounds.

## Run order on each MCU/core

1. Build CoreClockTest using the existing IOcomposer Eclipse CDT managed
   project (both Debug and Release where supported). Use the normal UART
   at 1,000,000 baud. Run without a debugger attached for sleep tests.
2. Run CoreClockTest after a power cycle. Check its final
   `CoreClockTest complete: 0 failures` line.
3. Run the existing TimerDemo for the target's supported timer-device
   configurations. Verify timer Init/Enable/Disable, trigger callbacks,
   count and period reporting; it is the timer implementation test.
4. Build CorePowerTest using the same managed-project workflow. Run it
   without an attached debugger.
5. Let all automatic Standby timer-wake cycles finish. Check the final
   `CorePowerTest complete: 0x00000000 failures` line.
6. If System OFF and a wake button are supported, press the wake button,
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
- Clock-change integration with LF/HF timer devices: verify 250 ms
  delay timing and timer interrupt counts **after oscillator selection and
  restoration**, with independent HF reference for LF timers. These are
  clock integration checks, not a replacement for TimerDemo.
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

## TimerDemo coverage

TimerDemo is the existing generic timer-driver exercise. It tests the
configured timer's initialization, available triggers, callback periods,
count reporting and Enable/Disable behavior. Run the MCU's existing
managed TimerDemo project; do not replicate those driver functionality tests
in CoreClockTest or CorePowerTest.

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
for their reported scope. The newly added RTC-contention phase needs
a new hardware run before it can be signed off.
