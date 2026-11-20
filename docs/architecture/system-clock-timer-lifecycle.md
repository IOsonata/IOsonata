# MCU core power, clock and timer lifecycle

Status: architecture decision and active implementation for IOsonata 0.14.
The generic clock Enable/Disable declarations and Nordic implementation
are present on `refactor_nordic_clock_ownership_014`. Other MCU ports are
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
functions. When a clock is enabled, its MCU backend uses the oscillator
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
| MCU core backend | Implement the MCU system facilities, apply `g_McuOsc`, maintain shared oscillator reference counts and protect hardware ownership |
| Timer/peripheral driver | Select supported timer inputs and configure its own peripheral hardware; use system peripheral-clock and shared-clock APIs as needed during its normal lifecycle |
| BLE stack or other external owner | Keep its hardware ownership; HAL must not stop an externally owned clock |

The Nordic working branch now uses `SystemHighFreqClockEnable/Disable` for
shared HF oscillator ownership and `SystemLowFreqClockEnable/Disable` for
LFCLK. Its RTC, HF timer, USB controller, HFXO tests and ClockDemo have been
updated to use those functions. The redundant Nordic LFCLK private header
has been deleted. Source review is not a substitute for a complete build,
SoftDevice/SDC integration tests, and actual hardware validation.

### Nordic BLE clock ownership is transparent to applications

The same generic HF/LF clock API must work before and after S132 or MPSL
initialization. Applications and generic peripheral drivers must not choose
an HFXO owner, split clock lifetime at BLE startup, or perform Nordic-specific
release/re-enable sequences. The MCU clock backend owns that integration.

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
  backend. Do not silently stop an actively required clock or fake a balanced
  reference count.
- Verify both directions of clock-owner lifecycle, failed initialization,
  repeated enables, and final disables before claiming transparent operation.

HfxoTestBle mode 0 (stack first) and mode 2 (application manually releases
before stack) have passed on S132 and SDC/MPSL. Mode 2 is a diagnostic for
a hardware-safe ordering, **not** an acceptable application requirement.
Mode 1 (direct HFXO request retained through BLE initialization) has failed
on SDC and remains an open bug. Source review and test success in other
modes do not close it.

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
