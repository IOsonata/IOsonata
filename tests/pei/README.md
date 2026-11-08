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

Checks cover invalid event/task addresses, resource exhaustion, repeated
lifecycle calls, owned-channel checks, pre-existing channel and group use,
1-to-many and many-to-one connections, SoftDevice service errors, and
interrupt restoration. All SoftDevice operations are checked to run outside
a global interrupt-disabled section.

These are host model tests, **not** a hardware or full MCU-library build.


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

The RTC Timer backend now implements TICK/OVRFLW event sources via
`EVTEN` independently of `INTEN`, and START/STOP/CLEAR/TRIGOVRFLW
task endpoints via the existing C Timer APIs. Hardware-only RTC COMPARE
supports single-shot and continuous operation: a private PPI link
routes COMPARE to STOP (single) or CLEAR (continuous). This is
not an ISR-rescheduled software trigger.

RTC exclusive ownership is required for this compare mode because
STOP/CLEAR affects the full 24-bit RTC counter. The driver rejects
other active compare triggers or bound RTC TICK/OVRFLW sources.
`TimerDisableTrigger` unbinds the public event and releases the
private PPI channel. Supported PPI channel reservations still apply.

**RTC compilation and hardware validation remain pending.** To test
the same two-button, two-LED example using RTC2 instead of TIMER2,
add `PEI_BLINKY_USE_RTC` in PeiBlinky's Eclipse C++ compiler
defined symbols, clean-rebuild, and flash BLYST-NANO-DK. The board
configuration selects RTC2 and 32768 Hz; no application source
changes are needed. Check 500 ms LED alternation, BUT1 start,
BUT2 stop, BUT1 resume, and RTC/GPIOTE ISR trace. Remove this
symbol to return to the previously tested TIMER2 configuration.

## BLYST-NANO-DK hardware test

Import `ARM/Nordic/nRF52/nRF52832/exemples/PeiBlinky/ioc` into
IOcomposer. The shared `exemples/pei/pei_blinky.cpp` is MCU independent:
it binds the event and task endpoints into one `Pei` matrix and enables
hardware routing, without CPU event forwarding.

The example now uses `Timer::EnableTimerTrigger(pei, evt, ...)` to
prepare TIMER2's hardware-only compare and bind its event to PEI.
`IOPinConnectTask(pei, evt, ...)` asks the GPIO driver to prepare a
GPIOTE task and connect its endpoint. Peripheral setup stays in the
Timer and GPIO HAL drivers; PEI only programs PPI. The former
`pei_blinky_port_nrf52.c` adapter and `pei_blinky_port.h` are removed.

nRF52 uses hardware COMPARE->CLEAR to generate the 500 ms periodic
event without an ISR. That shortcut affects the whole timer counter,
so the Timer driver refuses PEI periodic setup when it has another
active compare trigger or another hardware shortcut. The hardware-only
mode is currently continuous only; existing ISR modes are unchanged.
`DisableTimerTrigger(trig)` unbinds PEI before releasing the compare.

The standalone project links the production `pei_nrf52.cpp` as an
application object compiled without `SOFTDEVICE_PRESENT`. The standard
`IOsonata_nRF52832` Debug and Release HAL libraries are built for S132;
their PEI implementation calls SoftDevice SVC functions. Without a
running SoftDevice, that path faults. This direct-source workaround is
specific to the standalone test, not to Bluetooth applications.

## Two-button hardware handoff (functional hardware PASS)

`PeiBlinky` now wires four hardware event-to-task connections through PEI:

- TIMER2.COMPARE -> GPIOTE toggle task for LED 1 and LED 2
- BUT1 (P0.02, falling edge) -> TIMER2.START
- BUT2 (P0.13, falling edge) -> TIMER2.STOP

On reset, LED 1 starts off and LED 2 on (both are active-low).
The timer counter starts stopped while its high-frequency clock remains
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
including TIMER2 compare fan-out to both LEDs. The 500 ms event period is
the configured value, not a separately measured timing result. A debugger
trace confirming the absence of Timer/GPIOTE ISR entries is still pending.

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
