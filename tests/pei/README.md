# nRF52 PEI validation

Run from the IOsonata repository root:

```sh
python3 tests/pei/run.py
```

The host tests compile the production `src/coredev/pei.c` C matrix
and the `ARM/Nordic/nRF52/src/pei_nrf52.cpp` PPI backend against mock
registers. The matrix is shared by C (`PeiDev_t`) and C++ (`Pei`)
callers. They test bare-metal nRF52832 (20 configurable
channels), S132, S140, SDC, and nRF52805 (10 configurable channels). The mock
headers intentionally omit obsolete PPI_CH_NUM, PPI_GROUP_NUM, and
NRF_SOC_APP_PPI_CHANNELS_SD_ENABLED_MSK definitions. They also
compile the matrix-owning C++ `Pei` object with a C++ driver object and
exercise fan-out, fan-in, source replacement, independent disconnection,
and stale event handles.

The C tests exercise event allocation, fan-out, source rebinding,
enable/disable, release, and stale event rejection through `PeiDev_t`.
The C++ tests exercise the same matrix through the `Pei` wrapper.
`tests/pei/pei_hal_c_api.c` also syntax-checks the public Timer and
GPIO PEI entry points from C, without requiring an MCU build.

Software events (EGU) are tested with the EGU registers mapped at their
nRF52 addresses in host memory: allocation order, channels with INTEN set
by another user, the reserved instances of each build profile (two EGU on
the nRF52805 profile), firing, the task endpoint, unbind/rebind keeping the
task connections, a failed bind giving the channel back, release, deinit
and exhaustion.

Checks cover invalid event/task addresses, resource exhaustion, repeated
lifecycle calls, owned-channel checks, pre-existing channel and group use,
1-to-many and many-to-one connections, SoftDevice service errors, and
interrupt restoration. All SoftDevice operations are checked to run outside
a global interrupt-disabled section.

These are host model tests, **not** a hardware or full MCU-library build.


## Shared GPIOTE PORT event source

`IOPinEnablePortEvent()` exposes nRF52's combined
`GPIOTE.EVENTS_PORT` as a PEI producer without allocating a GPIOTE
channel, configuring pin SENSE, or enabling an interrupt. The caller
uses `IOPinSetSense()` for low/high level detection and explicitly
clears it when done. `IOPinDisablePortEvent()` unbinds only the
logical event source; other PORT interrupt and pin users are untouched.

The host suite includes C/C++ API compile checks and static
non-invasive-source checks. Run the
`ARM/Nordic/nRF52/nRF52832/exemples/PeiPortDemo/ioc` Eclipse
managed project on BLYST-NANO-DK to test the PORT and dedicated IN[n]
event paths concurrently. There is no hardware PASS yet.

## Non-PEI firmware link isolation

A standard Timer or GPIO application must not pull the PEI matrix or
nRF52 PPI backend from `libIOsonata_nRF52832.a`. The TimerDemo
linker map previously demonstrated the regression:

```text
iopincfg_nrfx.o -> pei.o (PeiGetEventEndpoint)
pei.o -> pei_nrf52.o (PeiConnect)
```

Normal GPIO functions now carry *weak* references to the optional PEI
binding functions; without a PEI application they cannot extract either
archive member. Timer initialization leaves its optional PEI callbacks
unset. Only a call to `TimerEnablePeiTrigger` or
`TimerEnablePeiEvent` explicitly requests the MCU's PEI callbacks.
HF Timer and RTC compare state reuse existing `TimerTrig_t` and
per-CC bookkeeping. A compare event never causes an automatic STOP or
CLEAR task, and no private PPI route is installed for rearming.
The Timer ISR advances the selected CC for continuous operation, while
PPI routes the event directly to its task. RTC TICK/OVRFLW bookkeeping
remains in optional PEI-only sections; `--gc-sections` should discard
it from non-PEI firmware.

Clean-build the nRF52832 HAL and TimerDemo, then check the new map:

```sh
python3 tests/pei/check_no_pei_link.py \
  ARM/Nordic/nRF52/nRF52832/exemples/TimerDemo/ioc/Debug/TimerDemo.map
```

The checker fails if the archive extracts `pei.o` or
`pei_nrf52.o` (including ordinary `.a(pei.o)` linker-map entries),
if PEI-specific text remains in the executable,
or if any HF Timer or RTC PEI state arrays remain. The previous
TimerDemo link retained 248 bytes in `s_nRfxRtc*Pei` and
`s_nRfxRtcRaw`; the updated checker rejects this regression. It ignores
discarded debug and input sections. PEI-enabled PeiBlinky must still
pass after this change; link isolation does not establish hardware
operation.

## Compile against installed Nordic CMSIS headers

The mock-register tests cannot catch changes in Nordic's device headers. Use
`python3 tests/pei/check_nrf52_cmsis.py` after installing/updating IOsonata's
external SDKs. This invokes `arm-none-eabi-g++` with the installed nrfx MDK
register declarations and IOsonata's S132, S140, SDC include paths for
nRF52805, nRF52810, nRF52811, nRF52820, nRF52832, nRF52833
and nRF52840. It does not use nrfx HAL drivers,
allocate hardware, or link any firmware. Real MCU-library and on-board builds
remain separate checks.

The script uses `../external/nrfx`, `../external/nRF5_SDK` and
`../external/sdk-nrfxlib` relative to IOsonata, or corresponding environment
variables. Custom paths can also be passed with `--nrfx`, `--nrf5-sdk` and
`--nrfxlib`.

## nRF52 endpoint completion tracker

See [nRF52 PEI coverage and acceptance tracker](../../docs/architecture/pei_nrf52_coverage.md)
for the remaining peripheral families, device-specific differences,
ownership rules and acceptance criteria. Do not move PEI development
to another MCU family until nRF52 coverage is complete.

The RTC Timer backend implements TICK/OVRFLW event sources through
`EVTEN` independently from `INTEN`. Each free compare channel may
bind its `EVENTS_COMPARE[n]` source to PEI without stopping or clearing
the counter. Continuous compare scheduling uses an RTC ISR to update
only `CC[n]` relative to the current counter; PPI still routes each
hardware event without CPU forwarding. Single-shot compare disarms
that event after the first match.

Mixed PEI/interrupt CC channels and TICK/OVRFLW sources are permitted.
`TimerDisableTrigger` unbinds only the selected PEI compare source.
The counter remains free-running for elapsed-time reads. A clean MCU
build and mixed-channel on-target test are required.

**TIMER3 configuration needs a clean MCU build and hardware validation.**
`PeiBlinky/src/board.h` selects `TimerCfg_t.DevNo = 6`, which is
**TIMER3** on nRF52832. With `Freq = 0`, IOsonata selects the 16 MHz
high-frequency timer clock. Clean-rebuild and flash BLYST-NANO-DK.
Check 500 ms LED alternation, BUT1 start, BUT2 stop, BUT1 resume,
teardown/reuse and TIMER3/GPIOTE ISR entries. Device numbering on
nRF52832: RTC0-2 are `DevNo = 0..2`, and TIMER0-4 are `DevNo = 3..7`.
This change does not validate S132/SDC coexistence; use only resources
available to the application when a Bluetooth stack is running.

## BLYST-NANO-DK hardware test

Import `ARM/Nordic/nRF52/nRF52832/exemples/PeiBlinky/ioc` into
IOcomposer. The shared `exemples/pei/pei_blinky.cpp` is MCU independent:
it binds the event and task endpoints into one `Pei` matrix and enables
hardware routing, without CPU event forwarding.

The example now uses `Timer::EnableTimerTrigger(pei, evt, ...)` to
prepare the configured Timer/RTC compare source and bind its event
to PEI. `IOPinEnableTask(pei, evt, ...)` asks the GPIO driver to prepare a
GPIOTE task and connect its endpoint. Peripheral setup stays in the
Timer and GPIO HAL drivers; PEI only programs PPI. The former
`pei_blinky_port_nrf52.c` adapter and `pei_blinky_port.h` are removed.

nRF52 uses a free-running counter. The TIMER3 ISR advances only
the matched CC for the next 500 ms event. PPI routes that event to
GPIOTE in hardware and does not use an ISR to toggle either LED.
Single-shot compares disarm only their own event; neither mode stops
or clears the counter. Mixed interrupt and PEI CC operation requires
hardware validation. `DisableTimerTrigger(trig)` unbinds only that CC.

The standalone project links the production `pei_nrf52.cpp` as an
application object compiled without `SOFTDEVICE_PRESENT`. The standard
`IOsonata_nRF52832` Debug and Release HAL libraries are built for S132;
their PEI implementation calls SoftDevice SVC functions. Without a
running SoftDevice, that path faults. This direct-source workaround is
specific to the standalone test, not to Bluetooth applications.

## Two-button hardware handoff

With `DevNo = 6`, `PeiBlinky` wires four hardware event-to-task
connections through PEI:

- TIMER3.COMPARE -> GPIOTE toggle task for LED 1 and LED 2
- BUT1 (P0.02, falling edge) -> TIMER3.START
- BUT2 (P0.13, falling edge) -> TIMER3.STOP

This TIMER3 selection has not yet received a hardware PASS. The
previously reported BLYST-NANO-DK behavior predates this configuration
change and must not be counted as a TIMER3 validation.

On reset, LED 1 starts off and LED 2 on (both are active-low).
The counter starts stopped while the low-frequency clock remains
enabled. Press BUT1 to begin alternating the LEDs at 500 ms per toggle;
press BUT2 to freeze both LEDs in their current states; press BUT1 again
to resume the counter. STOP does not reset the timer phase. Button
bounce can repeat START/STOP tasks but does not invoke any ISR-based
software forwarding or debounce logic.

The GPIO driver configures and reserves two GPIOTE input event channels
through `IOPinEnableEvent(PeiDev_t *, ...)`, implemented directly in
`iopincfg_nrfx.c`, without enabling GPIOTE interrupts. It shares the
GPIOTE allocation pool with the two output toggle tasks and conventional
GPIO interrupts. `IOPinEnableTask(PeiDev_t *, ...)` directly prepares
and connects GPIO tasks through the same C driver. The Timer driver
exposes its START/STOP task endpoints and
`TimerEnablePeiTrigger(TimerDev_t *, ...)` in C. C++ calls delegate to
these APIs. PEI configures only the four relationships. No application register
access and no example-specific port adapter are required.

Hardware observation (November 7, 2026): the device tester reports
that the updated two-button, two-LED `PeiBlinky` works on the
BLYST-NANO-DK (nRF52832): BUT1 starts alternating the LEDs, BUT2
stops/freezes them, and BUT1 resumes the timer. This is a **functional
hardware PASS** for three event sources and four event-to-task links,
including compare fan-out to both LEDs. That earlier hardware observation
does not validate the new TIMER3 selection. The 500 ms event period is
the configured value, not a separately measured timing result. A debugger
trace confirming expected TIMER3 compare ISR entries and no event-forwarding GPIOTE ISR is still pending.

Hardware observation (November 7, 2026): following the PEI C++
backend rename, Eclipse project-link corrections and Timer/GPIO HAL
integration, the device tester reports that the nRF52832 HAL and
`PeiBlinky` compiled and the BLYST-NANO-DK LED flashes at approximately
1 Hz. This is a **functional hardware PASS** for the earlier one-LED
shared C++ example: Timer compare -> PPI -> GPIOTE task. It does not
establish a measured frequency, independently confirm that no
Timer/GPIOTE ISR was entered, or constitute a test of the later button-controlled variant, which
is recorded separately above.

The older register-level `pei_timer_gpiote.c` example was also
observed blinking at approximately 1 Hz before the HAL integration.
The debugger IRQ trace, route disable/re-enable and release checks, and
Bluetooth S132/SDC coexistence tests remain pending.

## nRF5340 DPPI host model

Run `python3 tests/pei/run_nrf53.py` from the repository root. This
compiles the real `ARM/Nordic/nRF53/src/pei_nrf53.c` against simulated
DPPIC registers in application secure, application nonsecure, and
network-core builds. It checks event publisher sharing, independent
subscriber enable/disable, reserved channels/groups, duplicate task
rejection, foreign endpoint protection, exhaustion, stale copied handles,
and failed configurations.

This is not a firmware build. Nordic MDK header compilation and on-board
testing are separate acceptance checks.

The shared example uses no target-specific registers or platform
adapter. Timer and GPIO have the endpoint preparation APIs on nRF52.
The nRF53 DPPI backend and other MCU-specific peripheral drivers are
separate follow-up integrations.
