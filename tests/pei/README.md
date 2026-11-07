# nRF52 PEI validation

Run from the IOsonata repository root:

```sh
python3 tests/pei/run.py
```

The host tests compile the production `ARM/Nordic/nRF52/src/pei_nrf52.c`
against mock PPI registers. They test bare-metal nRF52832 (20 configurable
channels), S132, S140, SDC, and nRF52805 (10 configurable channels). The mock
headers intentionally omit obsolete PPI_CH_NUM, PPI_GROUP_NUM, and
NRF_SOC_APP_PPI_CHANNELS_SD_ENABLED_MSK definitions. They also
compile the matrix-owning C++ `Pei` object with a C driver object and
exercise fan-out, fan-in, source replacement, independent disconnection,
and stale event handles.

Checks cover invalid event/task addresses, resource exhaustion, repeated
lifecycle calls, owned-channel checks, pre-existing channel and group use,
1-to-many and many-to-one connections, SoftDevice service errors, and
interrupt restoration. All SoftDevice operations are checked to run outside
a global interrupt-disabled section.

These are host model tests, **not** a hardware or full MCU-library build.


## Compile against installed Nordic CMSIS headers

The mock-register tests cannot catch changes in Nordic's device headers. Use
`python3 tests/pei/check_nrf52_cmsis.py` after installing/updating IOsonata's
external SDKs. This invokes `arm-none-eabi-gcc` with the installed nrfx MDK
register declarations and IOsonata's S132, S140, SDC include paths for
nRF52805, nRF52810, nRF52832 and nRF52840. It does not use nrfx HAL drivers,
allocate hardware, or link any firmware. Real MCU-library and on-board builds
remain separate checks.

The script uses `../external/nrfx`, `../external/nRF5_SDK` and
`../external/sdk-nrfxlib` relative to IOsonata, or corresponding environment
variables. Custom paths can also be passed with `--nrfx`, `--nrf5-sdk` and
`--nrfxlib`.

## BLYST-NANO-DK hardware test

Import `ARM/Nordic/nRF52/nRF52832/exemples/PeiBlinky/ioc` into
IOcomposer. The shared `exemples/pei/pei_blinky.cpp` is MCU independent:
it binds the event and task endpoints into one `Pei` matrix and enables
hardware routing, without CPU event forwarding.

The nRF52832-specific `src/pei_blinky_port_nrf52.c` prepares TIMER2
compare 0 and GPIOTE task out 0, then starts TIMER2 after PEI is enabled.
It uses IOsonata's `IOPinConfig` and `IOPinSet` for GPIO configuration,
but still programs timer and GPIOTE task registers directly. This is
a temporary *platform adapter*, not part of the generic example or PEI.
Those operations belong in the Timer and GPIO drivers when their PEI
endpoint APIs are implemented.

The standalone project links the production `pei_nrf52.c` as an
application object compiled without `SOFTDEVICE_PRESENT`. The standard
`IOsonata_nRF52832` Debug and Release HAL libraries are built for S132;
their PEI implementation calls SoftDevice SVC functions. Without a
running SoftDevice, that path faults. This direct-source workaround is
specific to the standalone test, not to Bluetooth applications.

Expected: `g_PeiStatus == 1` and the first BLYST-NANO-DK LED toggles
every 500 ms (a complete 1 Hz blink cycle). Status 2 means preparation
failed, 3 event connection failed, 4 routing enable failed. Confirm
there are no TIMER2 or GPIOTE IRQs in the debugger.

Previously validated (November 7, 2026): the old, register-level
`pei_timer_gpiote.c` example on nRF52832 blinked the blue LED at
approximately 1 Hz after the SoftDevice SVC fix. This established a
functional TIMER2 -> PPI -> GPIOTE path on the board, **not** that the
new `PeiBlinky` C++ refactor or IRQ-free operation has been retested.
The debugger IRQ trace and Bluetooth S132/SDC coexistence tests remain
pending.

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

The shared example uses no target-specific registers. The nRF52832
platform adapter still directly prepares TIMER2 compare and GPIOTE
task-mode registers; those belong in the Timer/GPIO HAL drivers in a
fully abstracted version.
