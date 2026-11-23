# IOsonata 0.14 - nRF54L System Core clock/power port

Working branch: `port_nrf54l_power_clock_014`.
Baseline: squash commit `1398e64d5f2ba5a7f5972abda6feeacd589e2a85`.

## Target managed projects

- `ARM/Nordic/nRF54/nRF54L15/lib/ioc`
- `ARM/Nordic/nRF54/nRF54L15/exemples/CoreClockTest/ioc`
- `ARM/Nordic/nRF54/nRF54L15/exemples/CorePowerTest/ioc`
- `ARM/Nordic/nRF54/nRF54LM20A/lib/ioc`
- `ARM/Nordic/nRF54/nRF54LM20A/exemples/CoreClockTest/ioc`
- `ARM/Nordic/nRF54/nRF54LM20A/exemples/CorePowerTest/ioc`
- `ARM/Nordic/nRF54/nRF54LM20B/lib/ioc`
- `ARM/Nordic/nRF54/nRF54LM20B/exemples/CoreClockTest/ioc`
- `ARM/Nordic/nRF54/nRF54LM20B/exemples/CorePowerTest/ioc`

The former combined `nRF54LM20x` target is now two targets, see
"nRF54LM20A and nRF54LM20B targets" at the end of this document. Earlier
sections that say nRF54LM20x describe the combined target as it was.

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

The measured 128 MHz nRF54L15 divisor is now 8,000,400 (us) and
3,000,150 (ns), in `ARM/Nordic/nRF54/src/system_nrf54l.c`.
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

The IOsonata `system_nrf54l.c` vendor portion was compared against
Nordic **nrfx v4.7.1 (MDK 9.2.2)** and already matches. The local
IOsonata changes are retained: PLL startup, instruction-cache enable,
`SystemCoreClockGet`, 128-MHz nRF54L15 delay calibration, and
`SystemOscInit`. No wholesale nrfx replacement or external dependency
update is required for this startup correction.


The shared ARM ResetEntry copies initialized data, clears BSS and calls
SystemInit before entering the application. The nRF54L system initializes
FICR trims and the CPU clock, but did not invoke the already-existing
SystemOscInit function linked by both nRF54L15 and nRF54LM20x libraries.
Consequently the board-described 32-MHz/32.768-kHz crystal load-capacitor
calculations were not being applied by IOsonata startup.

SystemInit now calls SystemOscInit **after vendor trim initialization** and
before any application/GRTC timer initialization. This is limited to secure
normal clock initialization. The routine uses Nordic's
`nrf_ficr_hfxo_cap_calculate` and `nrf_ficr_lfxo_cap_calculate` to derive
capacitor settings from the board's `LoadCap` in 0.1-pF units (converted
to femtofarads). For nrfx versions before 4.7.0, which do not
declare these functions, the oscillator source provides compatible signed
FICR-trim arithmetic using the same units and rounding. It keeps the
board's fractional-pF settings; the legacy integer-pF macros are not
used. The earlier hand-written slope and offset calculations
were incorrect and were removed. Settings are written only if different
and the corresponding oscillator is not already active; no clock is
started or stopped by this call.

Nordic documents that updating XOSC32KI.INTCAP after GRTC has requested
LFXO can increase supply current. The configuration must precede the first
GRTC LFXO request. An inherited bootloader/MPSL clock that is already
running before the application SystemInit retains its settings: the routine
does not overwrite capacitance while HFXO, system LFXO, or a directly
GRTC-owned LFXO is active.

**Board oscillator configuration:** The Nordic nRF54L15 DK and
IBK-NRF54L15 use the same crystal specifications: 32 MHz with an 8 pF
HFXO load capacitance and 32.768 kHz with a 9 pF LFXO load capacitance.
The existing weak `g_McuOsc` defaults in `osc_nrf54l.c` apply to both
boards; no IBK-specific override is required. Other boards with different
crystal specifications must override `g_McuOsc`.

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

## nRF54L15 CoreClockTest acceptance after SystemOscInit restoration

The maintainer supplied three complete, clean CoreClockTest serial captures
after rebuilding and restoring SystemOscInit, with **0 failures in every run**.
No HardFault or truncated UART lines were observed in these captures. This
supersedes the earlier crash suspicion; the earlier failure was not
reproduced after the clean rebuild.

At 128 MHz, all three runs measured msDelay(100) and nsDelay(100 ms)
at 100,000 us with the production divisors 8,000,400 (us) and
3,000,150 (ns). The sDelay(1) end-to-end times, in us, were:

| Mode | Run 1 | Run 2 | Run 3 |
| --- | ---: | ---: | ---: |
| Cold | 1,000,036 | 997,104 | 996,654 |
| Warm | 1,000,017 | 999,813 | 999,813 |
| Reuse | 1,000,017 | 999,813 | 999,812 |

The largest cold error was 3,346 us (0.335%); warm and reuse results
were within 188 us of one second. The 11 enumerated timer devices,
timer-trigger tests, independent clock ownership and disabled-group
contention all completed with no recorded failures.

The four GRTC devices reported approximately 248,724 / 248,809 /
248,851 us averaged over their 250-ms LFRC-phase measurements,
while the independent HF reference remained about 250,067 us.
After the system LF source was restored, the corresponding GRTC
averages were approximately 250,067 / 250,118 / 250,118 us.
This residual ~0.49-0.54% LFRC-phase timing discrepancy is an
unresolved clock-source observation, **not** a failing acceptance
check under the current 2% threshold. Do not alter UART, PLL, or
the production CPU delay factors based on it.

**Validation scope:** nRF54L15 bare-metal CoreClockTest at 128 MHz
has repeated passing hardware runs. CorePowerTest, nRF54LM20x
hardware, 64-MHz calibration and separate BLE/MPSL coexistence
remain independent acceptance tasks.

## nRF54LM20x next hardware validation target

The checked-in Eclipse/CDT `nRF54LM20x` managed library currently
uses `NRF54LM20B_XXAA` in Debug and Release. The `CoreClockTest`
and `CorePowerTest` managed examples still use
`NRF54LM20A_XXAA` in Debug and Release. This is an existing
build-variant mismatch that should be corrected to match the physical
DK's SoC before final hardware acceptance. Both examples reuse the
common nRF54L system, GRTC, power and oscillator implementations. The pin map agrees with Nordic's nRF54LM20 DK:
UARTE20 on P1.16 TX / P1.17 RX, and active-low Button 0 on P1.26.

**Check the DK revision and SoC first.** Nordic states that DK
revisions v0.3.4 and v0.5.4 have nRF54LM20A while v0.7.0 and
later have nRF54LM20B. An LM20B board should not be taken as
validated by a test binary compiled for `NRF54LM20A_XXAA`.
Update/validate the target's managed MCU configuration for LM20B
if that is the physical device, without replacing the A support.
Nordic development hardware compatibility:
https://docs.nordicsemi.com/r/bundle/comp_matrix_nrf54lm20a/page/comp/nrf54lm20a/nrf54lm20a_ic_rev_comp_with_dev_hw.html

The first nRF54LM20x 128-MHz `CoreClockTest` log reported
`CoreClockTest complete: 30 failures`. Its generic 128-MHz delay
divisor ran `msDelay(100)` for about 50,004 us, while `nsDelay`
was about 100,004 us and cold/warm/reuse `sDelay(1)` was accurate.
The resulting 30 failures were 15 `msDelay` timer-duration failures
and 15 continuous-trigger callback-count failures (four GRTC groups
in the LFRC phase and all eleven timer devices after LF restoration).
All measured `nsDelay` timer durations and GRTC/HF timer ownership
checks passed. No timer, UART or GRTC timing change is warranted by
these failures.

The HF Timer measured 15,998,720 us-loop iterations per second and
42,664,532 ns-loop iterations per second at 128 MHz. The
nRF54LM20A and nRF54LM20B share the same 128-MHz Cortex-M33 CPU,
and both now use `ArmDelayClockSetDivisors` values
**8,000,640 (us)** and **3,000,150 (ns)** at 128 MHz.
The nRF54LM20B adds the Axon NPU, but this does not alter the CPU
delay-loop implementation. This matches the measured loop rates to
within one iteration per second. The separately calibrated nRF54L15
remains unchanged. Neither LM20 variant has a calibrated 64-MHz
fallback yet. Do not use these factors for unrelated MCU families.

The corrected firmware still requires a managed Release clean build
and full hardware rerun with `CoreClockTest complete: 0 failures`.
Also verify that the board's `g_McuOsc` crystal load capacitances
match its hardware before signing off the power/clock implementation.

For a verified nRF54LM20 DK, align the managed MCU definitions and
linker scripts to the physical A/B variant, then clean-build the Release
library and both Core tests:

```sh
python3 tests/nordic/build_managed_power_clock.py \
    --target nrf54lm20a --config Release
```

First run `CoreClockTest` for startup clock, delay factors and all
GRTC/HF timer cases. Then run `CorePowerTest` and test System OFF
pin wake **without a debugger**, power-cycling after a debug session
if necessary. Require each complete test to report zero failures,
and verify post-System-OFF reset reason and retention readbacks.

## Managed clean-build commands

The shared Nordic headless Eclipse/CDT build script now includes both nRF54L
managed targets, including their library and generic Core tests:

```sh
python3 tests/nordic/build_managed_power_clock.py \
	--target nrf54l15 --target nrf54lm20a --target nrf54lm20b \
	--config Debug --config Release
```

`Debug_SDC` / `Release_SDC` are not offered for these two Core
tests, which contain only Debug and Release configurations. Build/validate
BLE/MPSL coexistence separately. The command has not been executed in this
session and does not replace the board tests.

## nRF54L15 System OFF button wake: debugger versus standalone

The maintainer reported that CorePowerTest completed all three resumable
standby timer-wake cycles and its pre-System-OFF checks with zero failures
on both the Nordic nRF54L15 DK and IBK-NRF54L15, but pressing the wake
button did not restart the test while a debugger was attached. Repeating
without the debugger confirmed that **button wake works in normal
standalone execution**. The attached-debugger behavior is therefore not
evidence of a broken SystemWakePin/GPIO implementation.

Nordic documents that System OFF is **emulated in Debug Interface mode**:
debug resources, including CPU and RAM, remain powered. The hardware
requires an infinite loop after writing SYSTEMOFF in this mode; the
existing SystemPowerOff implementation already has that loop using
__WFE(). True System OFF and GPIO wake acceptance must be performed
without an active debugging session, followed by a pin reset when
leaving Debug Interface mode if the debugger retained debug power.

Official references:
- https://docs.nordicsemi.com/r/bundle/ps_nrf54l15/page/pmu.html-unique_1272341417
- https://docs.nordicsemi.com/r/bundle/ps_nrf54l15/page/debug.html-debuginterfacemode

Do not change the GPIO pin mappings, the wake implementation or the
PRBS-tested UART driver on the basis of this debugger-only difference.

**Standalone post-wake verification (November 23, 2026):**
The newer CorePowerTest capture contains 35 complete boot/test cycles,
each reporting zero failures. In every cycle the three automatic timer
standby wakes, CPU/peripheral clock and UART-rate restoration, static/stack
RAM checks, and wake-pin configuration/rearm pass: 105/105 standby wakes.

The maintainer clarified the physical test sequence: the first run used
the debugger; the second run followed stopping the debugger and pressing
reset; thereafter the wake button was pressed only a few times (roughly
four), **not 33 times**. Most subsequent starts occurred without an
intentional button press. The repeated CorePowerTest starts must not be
counted as successful System OFF button wakes.

The decoded reset reasons must be distinguished from physical actions:

| Decoded reason | IOsonata meaning | Observed boots |
| --- | --- | ---: |
| `0x00000100` | `SYSTEM_RESET_WAKE_PIN` (System OFF pin wake) | 1 |
| `0x00000020` | `SYSTEM_RESET_DEBUG` (debug interface reset) | 32 |
| `0x00000002` | `SYSTEM_RESET_PIN` (reset pin) | 1 |
| `0x00000008` | `SYSTEM_RESET_SOFT` (software reset) | 1 |

On the `0x00000100` boot the test explicitly reported
`System OFF reset-on-wake: PASS`,
`RAM retention after System OFF: PASS`, and
`RAM non-retention after System OFF: PASS`
(`0x00000000 failures, 0x0000001a checks`). The other 34 boots
reported `0x00000017 checks`, as the reset-on-wake and retention-readback
assertions correctly ran only when the previous reset was a recognized
System OFF wake. Thus at least **one complete System OFF + button wake +
RAM-retention cycle is validated**, not 35 such cycles.

The earlier `SYSTEM_RESET_DEBUG` reasons on 32 boots were
unexpected and did not establish their physical cause. The maintainer
pressed the button only about four times in that original capture.
These unexplained restarts are retained here as historical evidence.

**Fresh power-cycle validation, November 23, 2026:** After power
cycling the nRF54L15, the maintainer made five physical wake-button
presses. The supplied log contains four complete post-wake boots
reporting decoded reset reason `0x00000100`, and each reported
`System OFF reset-on-wake: PASS`,
`RAM retention after System OFF: PASS`, and
`RAM non-retention after System OFF: PASS` with 26 checks and zero
failures. All three standby-timer wake cycles, restored CPU/peripheral
clocks and UART rate, and static/stack RAM tests also passed on each
captured post-wake boot. The log starts mid-cycle and ends at the final
System OFF prompt: five physical button presses must not be counted
as five complete post-wake records.

No `0x00000020` reset appears in the new clean power-cycle capture.
A pyOCD session that failed to exit Debug Interface mode cleanly
is the maintainer's leading hypothesis for the previous anomaly,
but this does **not** establish a pyOCD software defect or prove
its exact detach sequence failed. The power cycle removed the
behavior in this test. No GPIO, System OFF, PLL, delay, or UART
driver change is indicated.

**Current nRF54L15 status:** Standalone System OFF button wake
and post-wake RAM retention/non-retention are hardware validated
for the tested board. The earlier debug-state anomaly should be
retained as a note for debugger-assisted tests, not a blocker
for normal standalone wake. nRF54LM20x and BLE/MPSL coexistence
remain independent acceptance tasks.

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
   the board exposes them. nRF54L15 DK and IBK standalone button wake
   was reported working. One subsequent `0x00000100` boot verified
   `System OFF reset-on-wake` and both RAM retention/non-retention checks.
   The earlier debugger/reset capture contained unexpected
   `SYSTEM_RESET_DEBUG` boots, but a clean power-cycle capture
   subsequently showed four complete `SYSTEM_RESET_WAKE_PIN` boots
   with post-wake retention PASS and no unexpected restarts.
   Standalone button wake is accepted on the tested nRF54L15 board.
3. Reboot and repeat, paying attention to GRTC group ownership and BLE/MPSL
   coexistence. MPSL-enabled builds are a separate configuration from
   bare-metal acceptance.

The nRF54L15 bare-metal GRTC group ownership and interrupt operations
have now been exercised by three complete CoreClockTest hardware runs.
The initial nRF54L15 CorePowerTest capture verifies 105 timer-wake
cycles, and one recognized System OFF pin-wake/RAM-retention cycle,
but included unexplained debugger-classified restarts. The subsequent
clean power-cycle test verifies four additional full pin-wake/retention
cycles without unexpected restarts; the maintainer reported five
physical button presses. The remaining nRF54L15 standalone
power tests are accepted for the tested board. The suspected
residual pyOCD debug state remains unproven. This does **not**
validate nRF54LM20x, BLE/MPSL coexistence or all managed builds.
Do not merge to main until the remaining acceptance checks are complete.

## Source-level check

`python3 tests/nordic/check_nrf54l_core_port.py`

Source-level inspection of the nine main ownership, allocation, sleep and
managed-project invariants passed. The Python regression script has not been
executed in this session. Neither source inspection nor Python assertions are
a substitute for a managed build or hardware validation.

## nRF54LM20A microsecond-delay correction (November 23, 2026)

At 128 MHz, an LM20A hardware run measured approximately 125 ms for
`msDelay(250)`, while `nsDelay` and GRTC-based `sDelay` were accurate.
The 128-MHz microsecond divisor is provisionally corrected from 8,000,640
to 4,000,320 (31,997,440 us-loop iterations/s). The nanosecond divisor
remains 3,000,150. Neither the 64-MHz LM20A endpoint nor LM20B has
been hardware calibrated; LM20B retains its separate provisional constants.

This is a source-level correction **pending hardware verification**. Clean-build
LM20A CoreClockTest and check 100-ms delay against the HF reference, followed
by the complete timer matrix and CorePowerTest. Do not modify the PRBS-tested
UART driver to address the independent serial-output issue.

## LM20A two-run CoreClockTest follow-up (November 23, 2026)

Both hardware runs report `msDelay(100)=99,996 us`, `nsDelay(100ms)=99,999 us`,
and a calibrated us-loop rate of approximately 15,999,359/s. This confirms
the **original** LM20A 8,000,640 divisor, so the earlier unvalidated 4,000,320
change was reverted. The two cold `sDelay(1)` readings, 1,433,700 and
1,433,521 us, isolate 433,700/433,521 us of LFXO startup overhead. Warm
and reuse measurements are within 2 us of the requested second.

Set an LM20A-only **provisional** cold correction of 433,611 GRTC ticks
(1 MHz) and zero warm correction. The separate L15 correction remains
434,775 ticks, while LM20B remains unmeasured. Rebuild/reflash and confirm
cold `sDelay(1)` on multiple resets before accepting this compensation.
All 11 timer devices and trigger/ownership checks passed in both provided
runs. The previous serial-link limitation and 64-MHz endpoint calibration
are independent open issues.

## LM20x managed-project variant mismatch (November 23, 2026)

The follow-up CoreClockTest still showed 1.436/1.444 seconds for cold
`sDelay(1)` despite the LM20A compensation commit. Repository inspection
identified the cause: all four LM20x managed-library configurations defined
`NRF54LM20B_XXAA` for both C and C++ sources, while CoreClockTest and
CorePowerTest define `NRF54LM20A_XXAA`. As a result the library compiled
out the LM20A-only cold correction. The library configurations are now
aligned with the LM20A examples (eight macro entries), and a regression test
checks the project definitions. This does not establish LM20B validation;
any future LM20B build must use a matching B-specific library configuration.

**Required hardware check:** clean-rebuild the library as well as the
CoreClockTest application so an old B-compiled static library cannot remain
linked. Run fresh-reset cold/warm/reuse `sDelay(1)`, full timer matrix, and
CorePowerTest. Hardware confirmation of the fixed cold path is pending.

## nRF54LM20A and nRF54LM20B targets (November 23, 2026)

The combined `ARM/Nordic/nRF54/nRF54LM20x` target is replaced by two
targets named after their MCU define:

| Target | MCU define | Library project | Linker scripts |
| --- | --- | --- | --- |
| `ARM/Nordic/nRF54/nRF54LM20A` | `NRF54LM20A_XXAA` | `IOsonata_nRF54LM20A` | `gcc_nrf54lm20a_xxaa*.ld` |
| `ARM/Nordic/nRF54/nRF54LM20B` | `NRF54LM20B_XXAA` | `IOsonata_nRF54LM20B` | `gcc_nrf54lm20b_xxaa*.ld` |

Each target has its own library project and the same 40 example projects.
Every project of a target defines that target's MCU, links that target's
library and uses that target's linker scripts, so a library and an
application can no longer be built for different variants.

Per-target files:

- `lib/src/vector_nrf54lm20a.c` keeps IRQ 86 reserved, as in Nordic's
  nRF54LM20A startup file. `lib/src/vector_nrf54lm20b.c` has
  `AXONS_IRQHandler` there.
- `lib/include/nrfx_config_nrf54lm20a_application.h` and
  `nrfx_config_nrf54lm20b_application.h`, selected by
  `ARM/Nordic/include/nrfx_config.h` from the MCU define.

The shared sources under `ARM/Nordic/src` and `ARM/Nordic/nRF54/src` are
unchanged. Compiled with the managed Release settings of each target, 15
library sources produce the same object code as the combined target did
with the same MCU define; only the nRF54LM20A vector table differs (no
AXONS entry).

All nRF54LM20 hardware results recorded in this document were taken on an
nRF54LM20A board and belong to the nRF54LM20A target. The nRF54LM20B
target has no hardware result yet. Its cold `sDelay` compensation in
`timer_nrfx.cpp` is still zero and its delay divisors in
`system_nrf54l.c` are the nRF54LM20A values, not measured on a B chip.

The managed build script takes `--target nrf54lm20a` and
`--target nrf54lm20b`. Clean-build the library of a target before its
tests: an archive left from the combined project must not be linked.

## nRF54LM20A CoreClockTest hardware acceptance (November 23, 2026)

After splitting nRF54LM20A and nRF54LM20B into separate managed projects,
the nRF54LM20A CoreClockTest hardware capture records **four consecutive
complete runs with zero failures**, each at 128 MHz. Every run passes the
HF/LF lifecycle, core/peripheral clock switching and restoration, 11 timer
devices (four 1-MHz GRTC groups plus seven HF timers), one-shot/continuous
trigger operations, independent clock ownership and sDelay timer contention.

The four measured cold `sDelay(1)` values are 1,000,006, 1,011,782,
1,005,654 and 1,000,479 us, all within the 2% threshold (maximum error
1.1782%). Warm and reuse results remain within 3 us of 1 second. The
128-MHz busy-loop timing is consistent: `msDelay(100)=99,996 us`, and
`nsDelay(100 ms)=100,000 us`, with independently reported divisor
recommendations of 8,000,320 and 3,000,150. These runs used the
8,000,640 microsecond divisor.

**Follow-up:** the nRF54LM20A 128-MHz microsecond divisor in
`system_nrf54l.c` is now the measured 8,000,320 (15,999,360 loops/s),
which removes the 40 ppm shortfall of `msDelay`. The nanosecond divisor
stays 3,000,150. The nRF54LM20B branch keeps 8,000,640 until it is
measured on a B chip. Rerun CoreClockTest on nRF54LM20A to confirm
`msDelay(100)` at 100,000 us.

Cold LFXO/GRTC startup compensation of 433,611 ticks is **accepted at the
existing 2% tolerance on this LM20A board**, rather than claimed as an
exact startup time across conditions. CorePowerTest, the 64-MHz delay
endpoint, LM20B hardware testing and BLE/MPSL coexistence are still
separate acceptance tasks. The unsupported CPU low-power-mode and optional
RC-calibration reports in the capture are not test failures. No merge to
main is authorized by CoreClockTest results alone.

## nRF54LM20A CorePowerTest hardware acceptance (November 23, 2026)

The supplied CorePowerTest UART transcript contains **five complete boot/test
runs with zero failures**. The first two report 23 checks each with decoded
reset reasons 0x00000002 (pin reset) and 0x00000020 (debug reset). The next
three report 26 checks each with decoded reason 0x00000100 (System OFF pin
wake); all explicitly pass `System OFF reset-on-wake`, RAM-retention and
RAM non-retention checks. These are three recorded complete post-System-OFF
wake cycles, not five. The physical wake action was not independently
logged, so reset-cause classification is the evidence used.

Each of the five runs passes three automatic standby-timer wake cycles,
CPU/peripheral clock restoration, UART rate restoration, and static/stack
RAM checks: **15/15 standby wakes**. Retention-register read/write,
retention-region range checks, RAM retention enable/disable, and wake-pin
configuration/invalid-index/disable/rearm also pass. Reduced CPU power mode
is reported UNSUPPORTED, not FAIL. The reported capability mask is 0x3f.

The `LogDiag` counter `uartRejected` ranges from 0x2ab4 to 0x3f7d,
while `evicted` stays zero and the SysLog FIFO is empty at each summary.
The example reports UART `TxDropCnt` as *unaccepted submissions*, which
SysLog may retry: this counter is **not evidence of wire-byte loss**. It
remains a distinct serial transport/backpressure concern, not an observed
power/clock functional failure. The single 0x20 debug reset does not prove
a GPIO wake or reset fault; its cause remains unverified.

**Hardware status:** nRF54LM20A CoreClockTest 128-MHz and CorePowerTest
functional checks are accepted for the tested board. The transcript does
not establish broad operating-temperature cold-start margins, LM20A
64-MHz delay calibration, nRF54LM20B hardware acceptance, BLE/MPSL
coexistence, or clean managed build matrices. Keep these separate from the
successful bare-metal hardware results. Do not merge this branch into main
based on LM20A validation alone.

## nRF54LM20A measured delay-divisor follow-up (November 23, 2026)

After updating the nRF54LM20A 128-MHz microsecond divisor to the
CoreClockTest-measured 8,000,320 (15,999,360 loops/s), two complete
hardware CoreClockTest runs report **zero failures**. Both measure
`msDelay(100)` and `nsDelay(100 ms)` at exactly 100,000 us against the
HF timer. The nanosecond divisor remains 3,000,150. The two cold
`sDelay(1)` results are 998,125 and 1,004,740 us (within 0.474%),
with warm 999,998 us and reuse 999,997 us in both runs.

All 11 timer devices, GRTC/HF references, clock switching/restoration,
one-shot/continuous callbacks, trigger reuse, and timer contention
checks pass. Under LFRC, GRTC and HF-reference 250-ms results differ by
approximately 0.1–0.4%; after restoring the configured LF source they
agree closely. Optional CPU low-power and RC calibration operations
remain unsupported or not successful, not reported as test failures.

**Acceptance:** nRF54LM20A busy-loop delays and clock/timer operations
are confirmed at 128 MHz using the measured divisor. This is additional
hardware evidence following the earlier four-run clock test and the
five-boot CorePowerTest capture. It does not validate 64-MHz scaling,
nRF54LM20B, BLE/MPSL coexistence or the managed build matrix.

## nRF54LM20A CorePowerTest two-boot follow-up (November 23, 2026)

A further supplied capture contains two complete CorePowerTest boots with
zero failures: first reset reason `0x00000002` (pin reset, 23 checks), then
`0x00000100` (System OFF pin wake, 26 checks). Both pass three standby timer
wakes, CPU/peripheral clock and UART-rate restoration, static/stack RAM checks,
retention configuration, and wake-pin rearm: **6/6 standby cycles**. The second
boot explicitly passes System OFF reset-on-wake and both RAM retention and
non-retention checks. This capture corroborates earlier LM20A validation;
because overlap with the prior five-boot log is not established, do not add
these results to the previously counted independent wake totals.

Both summaries report `evicted=0` and an empty SysLog FIFO, with
`uartRejected=0x2aa5` and `0x3f70` respectively. These are unaccepted
UART submission attempts, not proven wire-byte losses. Unsupported reduced
CPU power mode is not a test failure. No driver change or additional power
calibration is indicated by these results.

## CPU frequency endpoint calibration: boot-only Nordic versus runtime-scalable MCUs

The generic CoreClockTest calibration design measures the fastest and
slowest *selectable* CPU frequencies using the same delay APIs and an
independent HF timer (DWT when available). MCU implementations that support
**safe runtime CPU frequency changes**, such as certain STM32 families,
should visit both endpoints in one run, retain their measured factors in
System Core, and interpolate the us/ns loop rates upon subsequent frequency
changes. The generic implementation currently has
`ArmDelayClockInterpolate()`, but that does not by itself prove a target
has installed measured endpoints or uses interpolation at runtime.

The Nordic nRF54LM20A/B clock is different: the product specification
lists a 64/128-MHz `PLL.FREQ` CPU frequency selection **on boot**.
nRF54L clock management on this branch deliberately rejects
`SYSTEM_POWER_MODE_LOW_POWER`: a live `PLL.FREQ` write is **not**
an acceptable substitute for an explicitly supported runtime transition.
`SystemCoreClockSelect()` on Nordic selects a clock source, not a CPU
frequency, and CoreClockTest therefore correctly observed only one
128-MHz operating point in its hardware runs.

To measure the 64-MHz endpoint, create an independently booted,
matching-library-and-application **64-MHz calibration build** of
CoreClockTest for each MCU variant, using its existing
`NRF_CONFIG_CPU_FREQ_MHZ=64` startup selection. Do not mix a
128-MHz-prebuilt library with a 64-MHz application; clean-build both and
check `PLL.CURRENTFREQ` / `SystemCoreClockGet()` at startup. Measure
both loops with the same HF reference, record 64-MHz and 128-MHz
factors separately for LM20A and LM20B, and validate delay accuracy.
These are two boot-selected configurations, not an application-switchable
low-power CPU speed. The existing 128-MHz results do not determine the
64-MHz factors without hardware measurement.

**Acceptance distinction:** two-point calibration is mandatory for an
MCU with supported runtime CPU scaling. For a boot-only selection, it
requires two separately booted calibration configurations, and
production selects the calibrated factor matching its startup speed;
runtime interpolation is not applicable until a safe switching mechanism
is specified by the silicon vendor and implemented/validated in System
Core. Any nRF54L low-power-mode support must be backed by documented
hardware behavior, not by an untested live PLL modification.

Nordic references:
- https://docs.nordicsemi.com/r/bundle/ps_nrf54lm20a/
- https://docs.nordicsemi.com/r/bundle/ps_nrf54lm20a/page/clock.html-concept_rp5_fgp_bs
