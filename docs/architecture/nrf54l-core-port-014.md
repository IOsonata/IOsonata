# IOsonata 0.14 - nRF54L System Core clock/power port

Working branch: `port_nrf54l_power_clock_014`.
Baseline: squash commit `1398e64d5f2ba5a7f5972abda6feeacd589e2a85`.

## Target managed projects

- `ARM/Nordic/nRF54/nRF54L15/lib/ioc`
- `ARM/Nordic/nRF54/nRF54L15/exemples/CoreClockTest/ioc`
- `ARM/Nordic/nRF54/nRF54L15/exemples/CorePowerTest/ioc`
- `ARM/Nordic/nRF54/nRF54LM20x/lib/ioc`
- `ARM/Nordic/nRF54/nRF54LM20x/exemples/CoreClockTest/ioc`
- `ARM/Nordic/nRF54/nRF54LM20x/exemples/CorePowerTest/ioc`

Use IOcomposer Eclipse CDT managed builds. Clean the library, build it, then
clean/build each test for matching configurations. The shared test sources
are unchanged and MCU-independent. UART default is 1,000,000 baud.

## Implemented on this branch - awaiting builds and hardware testing

- The GRTC backend supports `TimerInit(DevNo = -1)` for an unused,
  application-available GRTC interrupt group. MPSL's final group and
  compare channels 7-11 are not automatically selected.
- Explicit GRTC Timer handles retain ownership after `TimerDisable`.
  An auto-selected temporary Timer returns its group on Disable. Selection
  and reservation are interrupt-safe; release disables its interrupt group,
  clears the group's compare channels/events, and invalidates the old handle.
  A generic test may sequentially reuse its handle for another group only
  after the previous timer is disabled and all its triggers are cleared.
  Sequential rebinding retires the old disabled group's reservation so
  no stale interrupt dispatch points to a handle whose DevNo has changed.
  Live timers or armed triggers cannot be rebound, and another Timer handle
  cannot take an occupied group.
- `sDelay()` uses that generic TimerInit/EnableTrigger/DisableTrigger/Disable
  sequence. GRTC runs at 1 MHz, independently of the nRF52/nRF53 LFCLK
  startup-compensation logic; no unsupported fixed LFCLK compensation is
  applied to this target.
- The nRF54L System Core backend implements resumable `SystemSleep` using
  existing NVIC wake sources and advertises `SYSTEM_POWER_CAP_SLEEP` plus
  `SYSTEM_POWER_CAP_TIMER_WAKE`.
- The existing System OFF, GPIO wake, retention-register and RAM retention
  paths remain in place and must be evaluated by CorePowerTest.

## nRF54L15 first hardware run

Initial nRF54L15 CoreClockTest exposed a reproducible 2x-fast msDelay:
`msDelay(100)` measured approximately 50,004 us at 128 MHz,
while `nsDelay(100 ms)` measured 100,004 us. All 11 timer devices
accordingly showed approximately 125 ms for the 250 ms msDelay test
and only two callbacks in the 110 ms continuous-trigger sample.

The measured 128 MHz nRF54L15 divisor is now 8,000,640 (us) and
3,000,120 (ns), in `ARM/Nordic/nRF54/src/system_nrf54l.c`.
It applies only to NRF54L15_XXAA at 128 MHz; 64 MHz and
nRF54LM20 remain pending separate hardware calibration.

The final `sDelay occupied disabled LF timers` failure revealed that
sequentially reusing the same TimerDev handle discarded the GRTC
reservation of earlier disabled groups. The driver now stores explicit
group reservations separately from the active interrupt-dispatch handle.
Automatic allocation must skip reserved groups.

Additional source review found stale-handle access paths in GRTC `Reset`, `EnableTrigger`,
`DisableTrigger`, `GetTickCount`, `GetMaxTrigger`, `SetFrequency` and
`FindAvailTrigger`. An automatically released timer retains its former
`DevNo`, which can subsequently refer to a group used by a different timer.
All seven operations now check the active group owner before accessing group
state. Mutating trigger/reset paths also hold the interrupt mask across the
ownership check and update. Source-level regression assertions were added.
These changes are not a replacement for the required hardware rerun.

The initial `sDelay` cold/warm/reuse results were about 990.5 ms,
which passed the existing 2% acceptance criterion. Do not make a speculative
correction until the 128 MHz msDelay fix and timer ownership change
have been rebuilt and rerun.

**The first hardware run is not accepted:** the described fixes are
committed but have not been validated by a subsequent hardware run.

## UART mode correction

The generic `CoreClockTest` now enables UART interrupts by default on every
MCU (`CORE_CLOCK_TEST_UART_INT=true`). Boards may opt out when they lack
interrupt support. DMA remains separately selectable and defaults to false;
all six Nordic CoreClockTest boards select `CORE_CLOCK_TEST_UART_DMA=true`,
without repeating the interrupt default. On nRF54L, UARTE requires EasyDMA.
The earlier nRF54L15 and LM20x configuration omitted both board macros,
so UARTE forced DMA while leaving interrupt mode disabled.

The generic test's existing `DEVINTRF_EVT_TX_FIFO_EMPTY` callback
continues to control the flush check; no direct UART hardware access was
added to the test. Build and real-board revalidation remain mandatory.

## nRF54L15 CoreClockTest output and UART PRBS

The nRF54L15 CoreClockTest (128 MHz) showed missing and concatenated UART
diagnostic characters despite reporting `CoreClockTest complete: 0 failures`.
That output alone does not establish a defective UART implementation. The
UART transmit implementation had already been tested with PRBS on hardware,
as confirmed by the maintainer. Its original interrupt/DMA transmit handling
has therefore been restored without the unverified STOP/TXSTOPPED state or
the additional `bTxStopPending` flag.

The generic CoreClockTest's separately committed formatter batching remains.
Any further output diagnosis must isolate the CoreClockTest print/host-capture
path from the PRBS-validated UART driver before changing the driver again.

The same run reported GRTC time around 250.4-250.6 ms against an HF timer
reference of about 250.05 ms in the LFRC phase, and around 252.46-252.50 ms
after LF-source restoration. This remains an independent clock-source issue.

**Hardware status:** PRBS validation was performed before this intervention,
not repeated in this session. Rebuild and rerun CoreClockTest to assess its
output; the existing GRTC timing discrepancy is still under investigation.

## Managed clean-build commands

The shared Nordic headless Eclipse/CDT build script now includes both nRF54L
managed targets, including their library and generic Core tests:

```sh
python3 tests/nordic/build_managed_power_clock.py \
	--target nrf54l15 --target nrf54lm20x \
	--config Debug --config Release
```

`Debug_SDC` / `Release_SDC` are not offered for these two Core
tests, which contain only Debug and Release configurations. Build/validate
BLE/MPSL coexistence separately. The command has not been executed in this
session and does not replace the board tests.

## Hardware acceptance - follow-up required

For **each** nRF54L15 and nRF54LM20x target, run:

1. `CoreClockTest`: CPU/peripheral clock restoration, HF/LF lifecycle,
   delay calibration across available CPU frequencies, cold/warm/reuse
   `sDelay`, all GRTC and HF timers, one-shot and continuous triggers,
   independent counter progression and timer ownership/contention.
   Require `CoreClockTest complete: 0 failures`.
2. `CorePowerTest` without a debugger attached: capability reporting,
   resumable standby timer wake (all three cycles), clock/UART/RAM
   restoration, wake-pin validation and System OFF button wake/reset, where
   the board exposes them. Require the complete zero-failure summary.
3. Reboot and repeat, paying attention to GRTC group ownership and BLE/MPSL
   coexistence. MPSL-enabled builds are a separate configuration from
   bare-metal acceptance.

The GRTC group ownership, startup, and interrupt operations have **not**
been tested on hardware for this port. Do not mark targets validated or
merge to main until managed builds and real-board acceptance pass.

## Source-level check

`python3 tests/nordic/check_nrf54l_core_port.py`

Source-level inspection of the nine main ownership, allocation, sleep and
managed-project invariants passed. The Python regression script has not been
executed in this session. Neither source inspection nor Python assertions are
a substitute for a managed build or hardware validation.
