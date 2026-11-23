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

## GRTC low-frequency source selection (November 23, 2026)

The nRF54L15 datasheet specifies that GRTC CLKCFG.CLKSEL must be
chosen **before** the GRTC is started. The SYSCOUNTER runs at a
nominal 1 MHz while awake; its low-frequency reference is used
during sleep. Previously the IOsonata GRTC driver started the counter
without explicitly choosing its LF source, allowing the reset-default
system LFCLK choice to remain.

The nRF54L GRTC driver now uses the board-described low-frequency
oscillator in the one-time, stopped-counter start path:

- XTAL: direct GRTC LFXO source
- RC: direct GRTC LFLPRC source
- Any other source: return failure rather than silently select an
  unsupported GRTC source

If the counter was already started (for example by MPSL/SoftDevice),
the driver preserves that clock source and its running counter.
This choice is intentionally fixed once GRTC starts; changing the
system LFCLK with the generic clock API must not reconfigure GRTC.

The next nRF54L15 CoreClockTest must compare all four GRTC groups to
an independent HF Timer both before and after system LF source
selection, and must verify bare-metal and separate BLE/MPSL startup.
Do not assume this change resolves the remaining timing discrepancy
until actual hardware output confirms it.

## nRF54L15 GRTC cold-start delay (November 23, 2026)

The subsequent two-run nRF54L15 CoreClockTest log showed a 1,434,775-us
cold sDelay on the second execution, followed by 1,000,047-us warm/reuse
calls. Nordic specifies a typical 430-ms LFXO oscillator startup, which
closely matches the 434,775-us startup excess. The first execution's
990,486-us cold call suggests the crystal was still warm after the test's
earlier system LFCLK lifecycle.

The shared Nordic sDelay previously assumed nRF54L GRTC always warm.
It now checks whether GRTC SYSCOUNTER is already started or the system
LFXO is running before the first GRTC TimerInit. This read-only check
does not start clocks or modify GRTC ownership. nRF54L15 alone gets the
provisional measured 434,775-tick cold compensation at its 1-MHz GRTC
frequency; warm calls and RC-configured boards use zero. nRF54LM20x
retains its existing zero-compensation defaults until measured.

The generic CoreClockTest now reports sDelay's measured overhead in
microseconds instead of incorrectly converting all MCU timer sources
using 32,768 Hz. It remains generic and does not infer clock hardware
or update any production delay factors.

**Hardware follow-up required:** test true cold power-on versus reset,
LFCLK recently released versus actively retained, RC board configuration,
and BLE/MPSL-owned GRTC. Startup time can vary with crystal, temperature
and load. The provisional constant must not be treated as validated for
production until these checks pass.

## nRF54L oscillator startup configuration (November 23, 2026)

The shared ARM ResetEntry copies initialized data, clears BSS and calls
SystemInit before entering the application. The nRF54L system initializes
FICR trims and the CPU clock, but did not invoke the already-existing
SystemOscInit function linked by both nRF54L15 and nRF54LM20x libraries.
Consequently the board-described 32-MHz/32.768-kHz crystal load-capacitor
calculations were not being applied by IOsonata startup.

SystemInit now calls SystemOscInit **after vendor trim initialization** and
before any application/GRTC timer initialization. This is limited to secure
normal clock initialization. SystemOscInit checks the present capacitor
registers and skips a write if the desired value is already present; this
avoids redundant writes when settings have been retained across a reset.
No oscillator is started or stopped by this call.

Nordic documents that updating XOSC32KI.INTCAP after GRTC has requested
LFXO can increase supply current. The configuration must precede the first
GRTC LFXO request. An inherited bootloader/MPSL clock that is already
running before the application SystemInit is a separate ownership case:
these changes do **not** claim to safely reconfigure an already-running
crystal if the retained capacitance differs.

**Board requirement:** The weak g_McuOsc default in osc_nrf54l.c describes
the Nordic DK. Production IBK-NRF54L15 and other boards must provide
correct board-specific oscillator load capacitances. Do not assume the
Nordic DK capacitor values apply to an IBK board.

To diagnose remaining GRTC timing differences, capture these actual
register values at the indicated checkpoints in a **separate debugger run**
(not during the timing-acceptance run):

- Immediately after SystemInit: OSCILLATORS.XOSC32KI.INTCAP,
  OSCILLATORS.XOSC32M.CONFIG.INTCAP, GRTC.MODE, GRTC.CLKCFG.
- After the first nRFxGrtcEnable: GRTC.MODE, GRTC.CLKCFG.CLKSEL,
  CLOCK.LFCLK.SRC, CLOCK.LFCLK.STAT.
- During the CoreClockTest LFRC phase and after LF restoration:
  GRTC.CLKCFG.CLKSEL, GRTC.MODE, CLOCK.LFCLK.SRC,
  CLOCK.LFCLK.STAT and CLOCK.LFCLK.SRCCOPY.

Read the register fields without modifying them. Debugger halts can
affect low-power timing, so compare the recorded configuration separately
from the UART timing logs. If the board selects direct GRTC LFXO,
system-LFCLK switching should not rewrite GRTC.CLKCFG.

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
