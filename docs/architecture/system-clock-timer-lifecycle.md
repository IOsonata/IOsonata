# MCU core power, clock and timer lifecycle

Status: architecture decision and active implementation for IOsonata 0.14.
The generic clock Enable/Disable declarations and Nordic implementation
are present on `refactor_nrf5340_power_clock_014`. Other MCU ports are
not yet implemented or validated for these functions. The Nordic changes
still require focused builds and hardware retesting; this document is not
hardware PASS evidence.

## System core interface responsibility

`coredev/system_core_clock.h` defines MCU-level system services: core power
management, sleep and wake behavior, reset and retention functions, core
clock selection, and peripheral clock management where supported by the MCU.
These facilities are normally initialized by `SystemInit()` and used by
IOsonata's system and peripheral implementations. Applications normally
configure and use their peripherals without invoking system clock functions.
Direct application calls are reserved for special requirements, such as
explicit core clock selection, system sleep/power management or intentional
management of an MCU clock source.

`SystemPeriphClockGet()` and `SystemPeriphClockSet()` provide the existing
generic peripheral-clock frequency interface. The zero-based `Idx` selects
the MCU-specific peripheral clock domain required by a peripheral, not a
universal clock number shared across MCU families. For example, STM32L4
implements index 0 for PCLK1 (APB1) and index 1 for PCLK2 (APB2), and
supports independent bus prescalers. Nordic exposes index 0 as its fixed
16 MHz peripheral clock, without an independently configurable APB divider.
Each peripheral driver selects the applicable index and requested frequency
based on its hardware requirements. The system clock implementation reports
the actual achievable frequency; unsupported indices return 0. A fixed clock
may return its fixed frequency when the requested frequency cannot change it.
The application does not need to know the MCU's clock register layout or
prescaler settings. Source selection must not silently change unrelated
clock domains. This is distinct from a driver's
own peripheral enable bits and module clock gates: UART, SPI, I2C, timers
and other drivers still configure and enable their respective hardware.
Shared HF/LF clock Enable/Disable operations manage shared MCU clock sources;
they do not replace the peripheral drivers' lifecycle.

## Design principle

System clock and power functions are internal utilities of IOsonata even
though they are available in the generic API. In particular,
`SystemHighFreqClockEnable/Disable` and `SystemLowFreqClockEnable/Disable`
are normally called by peripheral drivers, not by applications. Drivers
must select, start, count and stop their required clocks automatically
through their existing Init/Enable/Disable lifecycle. A normal application
must not choose HF versus LF, manage reference counts, know which Nordic
BLE stack owns CLOCK, or change its startup order to keep a peripheral
working. An application may call the system APIs directly only when it
deliberately needs special MCU clock or power control.

An application specifies the peripheral operation it needs. The MCU
implementation selects and manages the required clocks. Applications should
not need MCU-specific oscillator startup sequences, clock reference counts,
or sleep-related peripheral restoration for ordinary operation.

The generic HAL is the source of interface definitions. Before adding a
private API, type, enum, or state mechanism, inspect the existing generic
interfaces, implementations, and other MCU ports. Do not duplicate generic
clock-source enums or introduce separate Acquire/Release functions.

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
- System core clock selection configures the MCU clock tree.
- The shared HF/LF Enable/Disable functions manage oscillator or
  shared-clock usage and the corresponding reference counts; they do not
  disable an essential CPU clock or replace peripheral-module gates.
- Existing `SystemPeriphClockGet/Set` functions manage derived peripheral
  clock frequencies and MCU-specific clock-domain indices where supported.
- Peripheral `Init/Enable/Disable` methods select suitable clock inputs
  from their requested operating frequency and handle module setup without
  requiring applications to understand the clock tree.

Do not introduce duplicate enums, a private oscillator resource API, or a
synthesized-LFCLK mode to express this model.

`g_McuOsc` contains the board's configured oscillator descriptions:

- `CoreOsc`: high-frequency/core oscillator description.
- `LowPwrOsc`: low-frequency oscillator description.
- The types come from the existing `OSC_TYPE` enum: RC, XTAL and TCXO.

`SystemInit()` establishes the initial hardware clocks and power state.
`SystemCoreClockSelect()` remains the explicit MCU core-clock selection
interface; `SystemLowFreqClockSelect()` remains the explicit low-frequency
source-selection interface. Selection must not double as an Enable/Disable
reference-count operation.

Normal peripheral operation must not require calling these selection
functions. When a clock is enabled, the MCU clock implementation uses the oscillator
configuration in `g_McuOsc`. Explicit source changes must not disrupt
active clock consumers.

### Generic clock functions

Declared in `include/coredev/system_core_clock.h` on the working branch:

```cpp
bool SystemHighFreqClockEnable(void);
void SystemHighFreqClockDisable(void);

bool SystemLowFreqClockEnable(void);
void SystemLowFreqClockDisable(void);
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

## Ownership boundaries

| Layer | Responsibility |
| --- | --- |
| Application | Configure and operate devices through generic APIs |
| System core power/clock | MCU power states, sleep/wake, reset/retention, core and supported peripheral clock frequency selection, and shared clock Enable/Disable |
| MCU system implementation | Apply `g_McuOsc`, manage shared oscillator reference counts and use the permitted hardware clock operations |
| Timer/peripheral driver | Select supported timer inputs and configure its own peripheral hardware; use system peripheral-clock and shared-clock APIs as needed during its normal lifecycle |
| BLE stack or other external owner | Keep its hardware ownership; HAL must not stop an externally owned clock |

The Nordic working branch now uses `SystemHighFreqClockEnable/Disable` for
shared HF oscillator ownership and `SystemLowFreqClockEnable/Disable` for
LFCLK. The RTC, HF timer and USB controller use those shared-clock
functions. Dedicated HFXO/SDC tests exercise reference counting and BLE
ownership. ClockDemo separately tests generic core selection, peripheral
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
  request cannot be inadvertently released by calibration.

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
HF/LF selected source in the System Core backend without modifying that
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
starts. It passed ClockDemo on both nRF5340 cores. Additional
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
Until all ARM translation units consistently select the DWT backend, the
MCU System Core must **always** initialize and refresh the assembly-loop
fallback factors, even if its own translation unit compiles DWT delays.
The nRF5340 application core does so at startup and at every 128/64 MHz
transition. Never leave the global delay rates at their startup
1,000,000-loop/s placeholders. DWT itself remains CPU-local and does
not require loop-factor calibration.

## CPU clock changes and delay factors

The existing `ARM/include/idelay.h` chooses the delay backend at compile
time: DWT CYCCNT where available, otherwise the calibrated assembly loops.
DWT delays count CPU cycles directly, handle 32-bit counter wraparound in
bounded chunks, and use the current `SystemCoreClock`. Neither path requires
Timer or RTC. Loop-cost calibration only affects assembly fallback.

Every successful CPU source or divider transition, including reduced-power
frequencies and wakeup restoration, must do the following in System Core:

1. Complete the clock change and read the effective CPU frequency from the
   relevant hardware clock register/divider.
2. Update `SystemCoreClock` with that frequency.
3. Recalculate BOTH `SystemMicroSecLoopCnt` and `SystemnsDelayFactor`
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

`SystemCoreClockUpdate` is the canonical place to read hardware clock
state and update factors. Ports such as SAM4L that assign the core
frequency directly during intermediate clock transitions must refresh
both factors at each assignment as well. This prevents stale factors
during low-power CPU divider changes and subsequent restoration.

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

- Repeated Enable/Disable, zero-count Disable and counter overflow.
- Two active consumers of a shared clock, including different Disable
  orders and an explicit source-selection attempt while busy.
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
