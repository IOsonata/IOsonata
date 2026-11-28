# MCU core power, clock and timer lifecycle

## Design constraints

1. **Low power.** A driver holds a clock for as long as its device needs it,
   including during a transfer or while a wake event is armed. After the last
   user releases the clock, the port stops it if the MCU permits. The
   application decides when it can reduce CPU speed or sleep.
2. **Small size and low resource use.** Keep flash, RAM and CPU overhead
   small enough to leave room for the application. Using startup or sleep
   alone must not pull unused peripheral drivers into the image. Drivers
   already own their device state; Power & Clock does not keep a second copy.

## Purpose and ownership

IOsonata Power & Clock manages MCU low-power operation and provides the
clocks the peripherals run from. It is an internal service used by startup
and drivers. Its declarations are shared across MCU ports so that an
application can use the same device code on different hardware.

This document is the design and the rules for writing an MCU port. It is not
specific to one MCU. Results and notes of each port are in
[MCU power/clock port evidence](core-power-clock-ports.md).

Terms used below: HF and LF are the high and low frequency oscillators the
board describes in `g_McuOsc`. A peripheral clock is one clock the MCU clock
tree derives from them for the devices. A user is a driver that needs a
clock to stay on.

| Owner | Responsibility |
| --- | --- |
| Board | Describe the fitted oscillators and their frequencies in `g_McuOsc`. |
| Application | Request a device operation such as a UART baud rate or timer interval. Decide when the application can sleep, prepare its devices through their drivers, and configure the intended wake event before calling `SystemSleep()`. |
| Device driver | Request the peripheral clock its device runs from and program its own divider for the requested operation. Own the device registers, pins, interrupts, DMA, the clock enable bit of the device and the device lifecycle. Hold the required shared clock while the hardware depends on it, including pending transfers and armed wake events. |
| MCU Power & Clock | Set up the board clocks and the peripheral clocks once at startup and share them. Enter the MCU sleep state and restore MCU clocks on wake where required. Respect clocks already owned by another core or radio stack. |

### Peripheral clocks

The peripheral clocks are assigned by the clock tree configuration of the
MCU port. `SystemInit()` sets them up, and the port numbers them: peripheral
clock 0, 1, 2 and so on. How many there are and what each number is depends
on the MCU. Examples:

- An STM32 can have peripheral clock 0 for the UARTs and 1 for the SPI.
- A Nordic nRF has one fixed 16 MHz peripheral clock, so only peripheral
  clock 0 is available.

A number the MCU does not have returns 0. The port documents its numbers in
its clock header.

A device driver is part of the MCU port, so it knows which peripheral clock
its device runs from. It requests that one with `SystemPeriphClockGet(n)`
and programs its own divider from the returned frequency to get the data
rate the application asked for. It reports the actual rate. The application
never names a peripheral clock: it asks the driver for a baud rate or a
timer interval.

`SystemPeriphClockSet(n, Freq)` returns the frequency in effect. A
peripheral clock is shared by the devices assigned to it, so the port does
not change it under them and does not look for them.

The HF and LF oscillators use the existing `Enable/Disable` calls with
internal reference counts. A driver calls them for an oscillator that can
stop on its own, such as a crystal oscillator the CPU does not run from.
Releasing one user preserves the oscillator while another user or the MCU
still needs it. That is the only purpose of the count. It is not a reason to
refuse another operation.

Power saving while the CPU runs comes from two things: a lower CPU
frequency, through `SystemPowerModeSet()`, and stopping an oscillator nobody
uses, through `Enable/Disable`. It does not come from changing oscillator.

### Delay functions

`usDelay`, `msDelay`, `nsDelay` and `sDelay` are declared in `idelay.h` of
the architecture, not in `system_core_clock.h`. `sDelay` sleeps for a number
of seconds on a timer. It is generic, in `src/sdelay.cpp`, built in the
library of a port whose `TimerInit` refuses a timer held by another
`TimerDev_t` (rule 7 of the timer section):

- It tries the low power timers from the last one down, as the application
  timers usually start at DevNo 0, then the high frequency timers from the
  first one, the lowest power one. The first `TimerInit` that succeeds gives
  the timer. A port with no low power timer uses a high frequency one.
- Before the Init, it requests the clock of those timers with
  `SystemLowFreqClockEnable` or `SystemHighFreqClockEnable` and reads the
  start time of that request. It releases its request after the Init, the
  timer keeps its own. The programmed time is the delay less the start
  time, so the call lasts the requested time when the clock was stopped.
- It waits in light sleep, SLEEPDEEP cleared on Cortex-M, with WFI and the
  interrupts masked so the timer interrupt cannot be lost between the check
  and the wait. A delay over 60 s is done in 60 s parts.
- It calls `TimerPowerOff` at the end, the timer is free again.
- The setup left, timer Init and power off, was under a millisecond in the
  earlier Nordic runs, against 20 ms for 2 percent of `sDelay(1)`. There is
  no per-timer correction.

### RC oscillator calibration

`SystemHighFreqClockCalibrate()` and `SystemLowFreqClockCalibrate()` are
for periodic calibration of an RC oscillator. The oscillator and its
calibration belong to the MCU power and clock code: one call runs one
calibration and returns its result. The period does not. The application
or subsystem that needs the accuracy decides when, and calls the function
each time, from its own timer or event loop in foreground context. The MCU
code owns no timer for it and schedules nothing. A port whose oscillator
cannot be calibrated returns false.

### Low-power operation

Before calling `SystemSleep()`, the application decides which operations
must finish and which devices must remain active. It uses the device APIs
to prepare them and arms its wake source. The drivers implement those
operations and own the peripheral registers. `SystemSleep()` handles MCU
sleep entry, clocks and return to the caller.

A wake source is a device interrupt, enabled through the driver of that
device. For a wake pin the application calls `IOPinEnableInterrupt` or
`IOPinAllocateInterrupt` of the pin driver; for a timed wake it arms a
timer trigger. The MCU power code has no function that selects a wake
device and does not call a device driver. Where a pin needs more than its
interrupt to wake the MCU from a sleep or power off state, the pin driver
of the port does it when the interrupt is enabled.

For example, if the last UART transmission must reach the wire, the
application waits for transmit completion before sleeping. If it needs
to receive during sleep, it configures supported UART wake through the
driver or defers sleep. For a periodic sensor sample, it arms the wake
timer before calling `SystemSleep()`. An enabled UART alone does not tell
the system which of these behaviors the application intends.

`Disable` permits a subsequent `Enable` with the existing configuration.
Device `PowerOff` requires `Init` before reuse. System sleep performs neither
operation on behalf of a device. System power-off is a separate application
decision: the MCU restarts from reset on wake.

### Scope of the generic API

Add a generic operation only for an identified low-power or peripheral-clock
requirement that an existing system or device operation cannot express.
Document its caller, the hardware it changes and its observable result.
Keep oscillator sequences, PLL calculations, register access and target
errata inside the MCU port. A register available on one MCU or a setting
exercised by `CoreClockTest` is insufficient reason to expose another API.

Porting measurements, delay calibration and diagnostic output belong in
the validation build. To check the cost of a change, compare flash and
static RAM in the same linked application before and after the change,
using the same MCU, build settings and device configuration.

### Rules for an MCU port

For the author of an MCU port and of its drivers.

1. Take the clock sources from `g_McuOsc` and set them up once in
   `SystemInit()`. Do not select a source at run time. A board with a
   crystal uses it, a board without one uses the RC.
2. Assign the peripheral clocks in the clock tree configuration and number
   them. Each driver of the port requests the one its device runs from.
3. The device asks for its clock. The clock code does not read device
   registers, does not search the clock tree for users, and does not ask a
   driver whether a clock is in use. This also applies to
   `SystemPowerModeSet()`. The port checks the clock hardware limits for
   the requested mode. The caller handles any device timing affected by
   the change, including a SysTick driven by the CPU clock.
4. `Enable` counts a user and starts the oscillator on the first one.
   `Disable` releases one user and stops the oscillator after the last one,
   where the hardware can stop it. Use the count for nothing else.
5. `SystemPeriphClockSet()` returns the frequency in effect. Change a
   divider only for a peripheral clock that feeds a single device.
6. Store the frequency of a clock when the clock setup sets it, and return
   the stored value from `SystemPeriphClockGet()`. `SystemCoreClockUpdate()`
   applies the CPU divider to the stored main frequency, sets
   `SystemCoreClock` and updates the delay factors. Do not rebuild a PLL
   frequency from the registers on each call.
7. `SystemSleep()` enters the MCU sleep state and returns after wake. It does
   not ask a driver whether sleep is allowed and does not pause or resume a
   driver. The application prepares its devices and arms the wake source
   before the call, through the driver of the wake device. The MCU power
   code defines no wake device function such as a wake pin selection.
8. A function that only `CoreClockTest` or `CorePowerTest` calls is not a
   reason to keep code in the library.
9. The calibrate functions run one RC calibration per call. The port does
   not start a timer or schedule a repeat; the caller handles the period.

`SystemCoreClockSelect()` and `SystemLowFreqClockSelect()` are not services
for applications or drivers. Some ports still call them from their own
startup. Those ports make them private startup functions when they are
brought to these rules; the declarations then leave `system_core_clock.h`.
A new port does not define them. The state of each port is in
[MCU power/clock port evidence](core-power-clock-ports.md#port-rules-status).

## Oscillator configuration and clock execution

### Two principal clock-source categories

IOsonata models **HF (high-frequency)** and **LF (low-frequency)** as the
two principal MCU clock-source categories. Their configured sources feed
the MCU's clock tree. Hardware-specific multiplexers, PLLs, dividers and
gates derive the clocks used by the CPU, peripheral buses and peripherals.
An MCU may also provide additional dedicated or external clock inputs;
the generic model does not imply that every output is routed through the
same two physical oscillators.

```text
g_McuOsc.CoreOsc                 g_McuOsc.LowPwrOsc
High-frequency source           Low-frequency source
RC / XTAL / TCXO                 RC / XTAL / TCXO
         |                                |
         +---------------+----------------+
                         |
             MCU clock selection/tree
             (mux / PLL / dividers / gates)
                         |
             +-----------+-----------+
             |           |           |
          CPU/core   Peripheral   Timer / RTC
                       domains     sources
```

The source is not the same thing as its derived clock domain. For
example, on Nordic nRF52, HFXO can be enabled for accurate HF timing while
the CPU clock continues to run; LFCLK must be started using the chosen
low-frequency source. On STM32, the CPU and APB peripheral clocks can be
derived through independently configured dividers. Implementations must
respect the actual routing and capabilities of each MCU.

The responsibilities are deliberately separated:

- `g_McuOsc` describes the board's oscillator configuration.
- `SystemInit()` configures the MCU clock tree from `g_McuOsc`.
- The shared HF/LF Enable/Disable functions manage oscillator or
  shared-clock usage and the corresponding reference counts; they do not
  disable an essential CPU clock or replace peripheral-module gates.
- `SystemPeriphClockGet/Set` give the frequency of the numbered peripheral
  clocks the port assigned in its clock tree configuration.
- Peripheral `Init/Enable/Disable` methods request the peripheral clock of
  their device, program their divider for the requested rate and handle the
  device setup. Applications do not need to know the clock tree.

Do not introduce duplicate enums, a private oscillator resource API, or a
synthesized-LFCLK mode to express this model.

`g_McuOsc` contains the board's configured oscillator descriptions:

- `CoreOsc`: high-frequency/core oscillator description.
- `LowPwrOsc`: low-frequency oscillator description.
- The types come from the existing `OSC_TYPE` enum: RC, XTAL and TCXO.

`SystemInit()` establishes the hardware clocks and power state from
`g_McuOsc`. When a clock is enabled, the MCU clock implementation uses the
oscillator configuration in `g_McuOsc`. There is no run time source
selection; see "Rules for an MCU port".

### Generic clock functions

Declared in `include/coredev/system_core_clock.h`:

```cpp
bool SystemHighFreqClockEnable(void);
void SystemHighFreqClockDisable(void);

bool SystemLowFreqClockEnable(void);
void SystemLowFreqClockDisable(void);

uint32_t SystemHighFreqClockStartTime(void);
uint32_t SystemLowFreqClockStartTime(void);
```

These functions operate the MCU's shared high- and low-frequency
clock sources, not individual peripheral clock gates. They are generic,
not Nordic-only. Each MCU port implements them according to its hardware:

- Enable increments the corresponding clock reference count. The first
  enable starts the configured source when it is not already active.
- Disable decrements the count only for an outstanding enable. The last
  disable stops a clock only when that hardware operation is supported and
  the clock is not required by the CPU, another hardware owner or a stack.
- An already running clock owned outside the HAL must not be stopped by a
  peripheral driver.
- A failure to start the clock must not increment the reference count.
- Any transition and counter update must be safe from relevant concurrent
  interrupt/peripheral activity; avoid waiting inside a critical section.
- Where a clock is essential or cannot be gated independently, Disable must
  not stop CPU execution or another necessary clock.
- StartTime gives the microseconds the last Enable waited for the
  oscillator, measured by the clock code while it polls the start. It is 0
  when Enable found the oscillator running, and always 0 on a port that
  starts its oscillators once in SystemInit. `sDelay` subtracts it, so the
  call lasts the requested time when it had to start the clock.

HF and LF hardware behavior is not necessarily symmetric. In particular,
on Nordic nRF52, HFCLK is available for the CPU from its internal source;
starting HFXO requests the accurate crystal source, not the CPU clock
itself. Stopping HFXO must not stop the core. LFCLK can be stopped and
started with its configured RC or external low-frequency oscillator.
Other MCU ports must implement their own supported behavior rather than
copying Nordic register assumptions.

Shared reference counting belongs in the implementation of the **shared
clock**, not in one peripheral or a duplicated resource-management API.
No `Acquire()`/`Release()` interface is required.

## Timer API and automatic source selection

The existing `TimerCfg_t`, `TIMER_CLKSRC`, `TimerInit()`,
`TimerEnable()`, `TimerDisable()` and `TimerSetFrequency()` remain
the user-facing timer interface.

Typical use:

```cpp
TimerCfg_t cfg = {
	.DevNo = 0,
	.ClkSrc = TIMER_CLKSRC_DEFAULT,
	.Freq = 1000000U,
	.IntPrio = 6,
	.EvtHandler = TimerHandler,
};

TimerInit(&timer, &cfg);
TimerEnable(&timer);
```

With `TIMER_CLKSRC_DEFAULT`, the selected timer implementation determines
its supported clock inputs and configures the best available source and
prescaler for `Freq`:

1. Respect the actual timer selected by `DevNo`. `DevNo` is a virtual
   timer index (typically low-frequency timers before high-frequency
   timers), not an automatic cross-peripheral timer search.
2. Choose a source that the selected timer can really use, accounting for
   frequency, resolution, counter width, accuracy and power requirements.
   Prefer a suitable low-power source; do not use a universal 32.768 kHz
   boundary as a rule for all MCUs.
3. Derive oscillator configuration from `g_McuOsc`. Do not invent a
   second clock-source configuration or duplicate the generic
   `TIMER_CLKSRC` enum.
4. Configure an achievable timer frequency, report the actual supported
   rate through the existing timer API and fail explicitly when a requested
   operation cannot be supported. Do not silently exchange one `DevNo`
   for a different timer.
5. `TimerEnable()` enables the required clock through the generic clock
   Enable function and starts the timer. `TimerDisable()` stops the
   timer and disables its clock requirement.
6. Reconfiguration or `TimerSetFrequency()` must respect other running
   users of shared clocks. Do not change their clock source unexpectedly.
7. `TimerInit()` refuses a `DevNo` held by another `TimerDev_t`, running
   or disabled, until that one calls `TimerPowerOff()`. The same
   `TimerDev_t` can call Init again after Disable. A port also refuses a
   timer that a running radio stack uses directly. `sDelay` relies on this
   to take only a free timer.

An explicit `ClkSrc` remains available for users who require a
particular supported timer clock. Unsupported explicit selections fail
rather than being silently substituted.

`Freq = 0` retains the existing meaning of requesting the highest
supported rate for the selected timer, where implemented. A requested
1 MHz rate for an inherently LF-only RTC does not turn it into an HF
timer; the driver must report its achievable rate or fail in accordance
with the generic timer interface.

### No synthesized LFCLK requirement

There is no requirement to preserve Nordic nRF52 synthesized LFCLK as a
generic system-clock mode. Do not add a synthesized oscillator type or a
new configuration field to `g_McuOsc`. MCU-specific high-frequency timer
kernel clocks are distinct from a synthesized 32 kHz LFCLK. The agreed
default low-frequency source is the one configured in
`g_McuOsc.LowPwrOsc`.

## Nordic implementation notes

The Nordic sections below describe the code as it is. It still has run time
source selection and refuses a selection from the request counts. Where they
differ from "Rules for an MCU port", the rules are the target and the Nordic
code is to be converted.

The Nordic working branch now uses `SystemHighFreqClockEnable/Disable` for
shared HF oscillator ownership and `SystemLowFreqClockEnable/Disable` for
LFCLK. The RTC, HF timer and USB controller use those shared-clock
functions. Dedicated HFXO/SDC tests exercise reference counting and BLE
ownership. CoreClockTest separately tests generic core selection, peripheral
clock frequency and timer behavior. The redundant Nordic LFCLK private
header has been deleted. Source review is not a substitute for a complete build,
SoftDevice/SDC integration tests, and actual hardware validation.

### Nordic BLE clock ownership is transparent to applications

The same generic HF/LF clock API must work before and after S132 or MPSL
initialization. Applications and generic peripheral drivers must not choose
an HFXO owner, split clock lifetime at BLE startup, or perform Nordic-specific
release/re-enable sequences. The Nordic clock implementation handles that transition.

An nRF52832 SDC/MPSL diagnostic reproduced the defect: a direct HFXO
Enable remained counted after MPSL initialization because Disable refused
a direct CLOCK STOP while MPSL owned the registers. This is an IOsonata
Nordic integration defect, not an invalid application request.

Implementation requirements:

- Preserve the logical shared HFXO reference count and active peripherals'
  clock requirements across BLE startup.
- Integrate owner transition within the Nordic clock and BLE initialization
  paths; do not require an application callback or additional public API.
- Use only Nordic-permitted clock operations for the current hardware owner;
  never STOP CLOCK through registers while S132/MPSL controls it.
- Where the underlying Nordic stack cannot accept an existing direct clock
  request without an interruption, explicitly address that limitation in the
  clock implementation. Do not silently stop an actively required clock or fake a balanced
  reference count.
- Verify both directions of clock-owner lifecycle, failed initialization,
  repeated enables, and final disables before claiming transparent operation.

### Installed MPSL capability confirmed (nRF52832)

The maintainer ran `tests/nordic/check_mpsl_external_clock.py` on the
installed sibling `external/sdk-nrfxlib`. All 19 installed MPSL headers
were searched. The external clock control registration and unregistration
APIs, both HFCLK/LFCLK callback types, and their exported symbols in the
nRF52 hard-float, soft-float, and softfp-float MPSL archives were found.
Thus the API is available in the actual installed toolchain libraries.
No production clock integration has yet been validated by that result.

### nRF52 SDC clock integration without a feature define

The nRF52 SDC library contains the external MPSL clock adapter by default.
There is **no IO_NRFX_MPSL_EXT_CLOCK compilation option** and no second
clock-specific library target. `MpslInit()` selects the clock implementation internally
from `g_McuOsc.LowPwrOsc` when initializing MPSL:

- `OSC_TYPE_XTAL`: register IOsonata's HFCLK/LFCLK callbacks
  before `mpsl_init()`; no LFRC calibration is required.
- `OSC_TYPE_RC`: register the same callbacks, start the initial LFRC
  calibration after MPSL initialization, then schedule calibration every
  four seconds through the CLOCK calibration timer. A separate HFXO
  reference is held until `EVENTS_DONE`, so a radio or application clock
  request cannot be inadvertently released by calibration. The clock code
  (`system_clock_nrfx.cpp`) runs one calibration per request. The four
  second period is handled by the MPSL file (`nrf_mpsl.cpp`), the
  subsystem that needs it.

The external HFCLK callbacks are nonblocking for MPSL high-priority radio
requests. The POWER_CLOCK handler processes HFCLKSTARTED, DONE and CTTO
events for LFRC calibration, then calls the MPSL handler.

The previous LFRC internal-MPSL implementation leaked a direct HFXO
request across `MpslInit()`. The shared external-clock implementation
has been committed but **has not yet passed managed CDT builds, hardware
clock tests, Bluetooth radio tests, or current measurements**.

The XTAL and RC external-clock configurations are still experimental on
nRF52 and are not signed off.
Validate actual MPSL callback signatures in the installed SDK, managed CDT
builds, RTC/HFCLK continuity, direct-to-stack handover, interrupt races,
BLE radio operation, and oscillator power consumption before declaring
ownership transfer complete. The S132 clock-owner integration is separate.
The generic application and peripheral API is unchanged.

### S132 direct-clock release across SoftDevice initialization

`system_clock_nrfx.cpp` defers direct HFXO shutdown while S132 is running.
If an outstanding IOsonata HFXO request was acquired directly before
SoftDevice initialization, the final `SystemHighFreqClockDisable()` while
SoftDevice is enabled clears the logical request but does **not** write a
direct CLOCK STOP. The physical oscillator remains on as an internal
SoftDevice-lifetime reservation. An `NRF_SDH_STATE_EVT_DISABLED` observer
completes the pending physical STOP after SoftDevice has relinquished
CLOCK. If an IOsonata clock request is still outstanding at shutdown,
the observer preserves the oscillator and attributes direct ownership
to the remaining user instead of stopping it.

This approach does not create an application mode, public API, or
additional library configuration. It preserves the already-tested normal
S132 clock paths while handling the previously failing pre-stack request
case. On nRF52832, the pre-stack request, S132 startup, clock release,
SoftDevice shutdown, and direct HFXO enable/disable after shutdown passed
hardware tests. Concurrent requests during active radio traffic, other
nRF52 targets, and power consumption still require measurement.

### Nordic owner integration investigation

Nordic MPSL has an external clock-control driver registration API,
`mpsl_clock_ctrl_source_register()`, which must be invoked before
`mpsl_init()`. This is a possible way to keep IOsonata as the physical clock
owner while MPSL requests the HF and LF sources. It is not a drop-in HFXO
handover API:

- Registration requires HFCLK and LFCLK callbacks, including LFCLK request,
  release, wait and calibration handling.
- MPSL can invoke HFCLK request/release from high-priority context; these
  callbacks cannot spin waiting for oscillator startup or depend on a
  preempted transition.
- The external clock-control integration is experimental on Nordic nRF52
  and nRF53, and availability in the actual linked nrfxlib version must be
  verified before use.
- MPSL's internal `mpsl_clock_hfclk_src_request/release` cannot be used
  simultaneously with its external clock-control driver.
- SoftDevice S132 instead owns the CLOCK peripheral and exposes
  `sd_clock_hfclk_request/release/is_running`; it does not share the same
  external registration interface. Its transition needs its own solution.

Do not implement a blind direct-to-MPSL count transfer, issue a direct
HFCLKSTOP after MPSL or SoftDevice initialization, or change the generic
application API. The Nordic clock implementation and stack initialization
implementation must coordinate ownership internally. A safe implementation
requires confirming the supported mechanism on the linked Nordic binaries,
then testing existing active IOsonata clock users across stack startup and
final Disable.

References:
- https://nrfconnectdocs.nordicsemi.com/ncs/latest/nrfxlib/mpsl/doc/clock.html
- https://nrfconnectdocs.nordicsemi.com/ncs/latest/nrfxlib/doxygen/html/mpsl__clock_8h_source.html

nRF52832 S132 and SDC/MPSL HFXO ownership tests passed with a direct
HFXO request outstanding during Bluetooth initialization. The S132 shutdown
and subsequent direct HFXO enable/disable tests also passed. These results
do not validate LFRC calibration, nRF52840, BLE radio traffic, or power
consumption. The portable `HfxoTestBle` example now calls `BtAppInit()`;
`tests/nordic/hfxo_test_ble_vendor_diagnostic.cpp` retains the Nordic-specific
clock and shutdown checks.

## Nordic oscillator declaration and selection

`g_McuOsc` declares board hardware support. Nordic keeps the temporary
HF/LF selected source in the System Core implementation without modifying that
declaration. RC is internal; external XTAL/TCXO selections are accepted only
when the board has declared that oscillator type and its frequency.

`Select` validates and configures but does not start or stop clocks.
`Enable` starts the selected source on first use, verifies readiness,
and increments the shared ref-count. `Disable` stops the source after
the last owner releases it. The board descriptor getters remain board
declarations, not temporary selection getters.

On nRF5340 the LFXO request can initially produce LFRC output while
the crystal starts. The clock manager waits for `LFCLKSTAT.SRC` to report
the active LFXO before claiming readiness. `LFCLKSRCCOPY` reports the
requested source and cannot by itself establish LFXO readiness.

## nRF5340 dual-core LFCLK priority and LFRC calibration

If the application core holds LFXO (as NetCoreStart does), a network-core
request for LFRC is served by the higher-priority shared LFXO. Nordic
nRF5340 clock-control priority is LFSYNT > LFXO > LFRC. The network-core
LFRC selection is the *requested* source; LFCLKSTAT.SRC is the *effective*
source. On nRF5340, only calibrate LFRC when LFCLKSTAT reports that LFRC
is actually running. A calibration DONE timeout must not invalidate an
otherwise running clock. Revision 1 anomaly 137 can suppress DONE when
the other core holds LFXO. Calibration failure is still reported by the
explicit operation; CoreClockTest treats it as unsupported/failed rather
than a clock or timer failure.

## nRF5340 NetCoreStart application-core LFXO preparation

NetCoreStart on the nRF5340 Audio DK runs as a secure application-core
image. Secure SystemInit configures the XL1/XL2 oscillator pin ownership
and board capacitor settings. NetCoreStart requests the declared LFXO
through SystemLowFreqClockSelect and SystemLowFreqClockEnable *before*
releasing NETWORK.FORCEOFF. It retains the shared LFCLK reference while
the application core waits in WFI. Failed LFXO startup prevents release
and keeps the success LED off.

The network core still uses its own LFCLK clock manager. Its existing
verified LFRC fallback remains appropriate when a standalone network
core does not have access to the shared crystal.

## Network-core LFXO availability fallback

The Nordic nRF5340 Audio DK declares an LFXO in `g_McuOsc`. The network
core requests the crystal when LFCLK is enabled, but application-core
configuration or other shared-hardware conditions can prevent LFXO from
becoming active. If the network core cannot activate LFXO, the clock owner
waits for the failed START's STOP and explicitly starts LFRC instead.

Only a successfully started LFRC is accepted as fallback. The requested
board source remains XTAL, while the effective RC source is tracked
internally for repeated enables and reference-counted disable. The
network core can retry XTAL on the next first enable. Other Nordic cores
retain their existing behavior.

`Select` remains configuration-only; no additional board capability
mask is needed. This fallback must be validated in network-only and
dual-core operation, including LFRC accuracy requirements.

## nRF5340 LFRC calibration API

The System Core provides `SystemHighFreqClockCalibrate()` and
`SystemLowFreqClockCalibrate()`. These are one-shot calibration requests.
They never initialize an RTC, select a Timer device, or schedule a
periodic task. A higher-level subsystem owns its own timer and decides
when to call the appropriate calibration function.

The functions return true only after calibration completes. A false return
means unavailable, invalid current source/state, or calibration failure.
The functions are foreground operations. Timer interrupt callbacks must
defer the call to foreground code rather than blocking inside the ISR.

On nRF5340, `SystemLowFreqClockCalibrate()` requires an active LFRC
shared-clock user. It acquires an additional LFCLK reference while
`TASKS_CAL` runs and releases it afterward; it waits for `EVENTS_DONE`
with a bounded timeout. The calibration retry interval is 1 ms for
nRF5340 revision 1 anomaly 137. HFXO is requested by the CAL hardware
and does not require a separate application request.

The existing startup calibration remains in place when LFRC first
starts. It passed CoreClockTest on both nRF5340 cores. Additional
calibrations are explicitly initiated by higher-level code, including
any periodic calls it chooses to schedule. A Timer callback may queue a
calibration event with AppEvtHandlerQue; AppRun executes the callback in
foreground context. System Core never owns that schedule or the queue.
See [Clock calibration from AppRun](clock-calibration-app-run.md).

On Nordic targets without an implemented RC calibration operation,
the corresponding API currently returns false. The nRF52 MPSL
calibration path continues through its existing clock-owner integration;
the new generic API must not bypass SoftDevice/MPSL. HFRC calibration
is not currently supported by the Nordic implementation.

For applications requiring calibrated LFRC accuracy continuously,
the higher-level subsystem must schedule and validate a suitable
recalibration interval, including across sleep/wake. The nRF5340
CLOCK peripheral does not have the nRF52 calibration interval timer.
Cross-core clock-source changes can abort calibration without DONE.

Validate scheduled calibration by measuring a long-running RTC user,
multiple calibration completions and time intervals, dual-core clock
requests, and HFXO release between requests before sign-off.

## nRF5340 application-core assembly fallback factors

The Audio DK's measured loop execution shows different microsecond-loop
costs at 128 MHz and 64 MHz: approximately 25 cycles per iteration at
128 MHz and 28 cycles at 64 MHz. The nRF5340 application port therefore
preconfigures the appropriate factor at each supported CPU speed instead
of applying a single 28-cycle factor at both speeds. The nanosecond-loop
cost remains approximately three cycles. DWT-backed delays use CPU cycles
directly; these calibrated loop factors protect translation units that
compile the assembly fallback.

## DWT and assembly-delay translation-unit consistency

Compile-time DWT selection may differ between translation units because
the compiler sees different target CMSIS definitions and include order.
Until all ARM translation units consistently select the DWT delay path, the
MCU System Core must **always** initialize and refresh the assembly-loop
fallback factors, even if its own translation unit compiles DWT delays.
The nRF5340 application core does so at startup and at every 128/64 MHz
transition. Never leave the global delay rates at their startup
1,000,000-loop/s placeholders. DWT itself remains CPU-local and does
not require loop-factor calibration.

## CPU clock changes and delay factors

The existing `ARM/include/idelay.h` chooses the delay method at compile
time: DWT CYCCNT where available, otherwise the calibrated assembly loops.
DWT delays count CPU cycles directly, handle 32-bit counter wraparound in
bounded chunks, and use the current `SystemCoreClock`. Neither path requires
Timer or RTC. Loop-cost calibration only affects assembly fallback.

Every successful CPU source or divider transition, including reduced-power
frequencies and wakeup restoration, must do the following in System Core:

1. After the clock change takes effect, determine the CPU frequency from
   the selected source frequency and the active CPU divider.
2. Update `SystemCoreClock` with that frequency.
3. Recalculate `SystemMicroSecLoopCnt` and `SystemnsDelayFactor`
   before returning control to callers that can use delay functions.

`ArmDelayClockSetDivisors` applies the existing MCU-specific loop
costs as **actual loop iterations per second**, not Q16 values. Both delay
functions calculate iterations directly from their calibrated loop rates
and the requested duration. This supports targets without DWT and has no
Timer, interrupt-wait, or event dependency.
The routine does not change clock hardware, and unsuccessful/unsupported
clock-source selection must not commit factors for a requested clock that
was never activated.

The MCU port supplies already-calibrated default-speed fallback loop costs.
A DWT-based delay requires **no assembly-loop recalibration** after CPU
frequency changes. Optional DWT characterization of the fallback loops is
still available for MCU porting, without Timer or LFCLK. The measured CPU-cycle
cost is cached and converted into loop iterations per second.
At the default frequency, IOsonata restores the port's original calibrated
loop costs. A target without DWT uses its port-provided cost and scales the
loop rate from the actual CPU frequency. Calibration may briefly mask
interrupts; it must not occur during ordinary delay calls.

On SAM4L, code that sets `SystemCoreClock` during an intermediate clock
change must also update both delay factors before a delay uses them.

The clock owner must serialize clock hardware transitions and factor
updates against other execution contexts; callers must not alter CPU
clock registers independently of System Core. Delays already in
progress while the CPU clock changes are not guaranteed to retain the
original duration. RTC and peripheral Timer drivers own their counters;
none are appropriated by System Core for delay-factor updates.

RISC-V ports use the counter-based delay implementation in
`RISCV/include/idelay.h`, which obtains the effective CPU frequency
without ARM loop factors. Do not introduce ARM delay-factor globals on
RISC-V targets.

Verify at least 250 ms delays before reduction, at reduced CPU speed,
and after restoration, using an independent hardware timing reference.
Also test different optimization levels and ports without DWT. Source
assertions and arithmetic checks alone are not hardware timing validation.

## Cross-MCU implementation and validation

This architectural change applies to Nordic, ST, Microchip, NXP,
Renesas, Espressif and other supported MCU ports. A generic header
declaration alone does not mean a port implements or validates the API.
Do not add success-only stubs to conceal missing behavior.

Before implementation, compare each port's existing oscillator setup,
clock gates, shared users and timer lifecycle. Verify at least:

- Repeated Enable/Disable and a Disable at a zero count.
- Two active users of a shared clock, with both Disable orders: the source
  stays on until the second Disable.
- External/stack clock ownership and transitions; SoftDevice/SDC where
  applicable.
- Timer initialization with DEFAULT, explicitly supported and
  unsupported clock sources, including actual-frequency readback.
- Clock behavior after sleep/wake and after failed oscillator start.
- Managed CDT Debug/Release builds and hardware tests separately.
- Existing UART, USB, RTC and BLE operation after clock-management
  changes.

See [power-clock integration review](power-clock-review-014.md),
[MCU power/clock port evidence](core-power-clock-ports.md) and
[clock/power demo matrix](power-clock-demo-matrix.md).
