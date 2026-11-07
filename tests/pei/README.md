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
compile the C++ `Pei` wrapper with a C driver object.

Checks cover invalid event/task addresses, resource exhaustion, repeated
lifecycle calls, owned-channel checks, pre-existing channel and group use,
1-to-many and many-to-one connections, SoftDevice service errors, and
interrupt restoration. All SoftDevice operations are checked to run outside
a global interrupt-disabled section.

These are host model tests, **not** a hardware or full MCU-library build.

## BLYST-NANO-DK hardware test

Import `ARM/Nordic/nRF52/nRF52832/exemples/PeiTimerGpiote/ioc` into
IOcomposer after rebuilding the nRF52832 library. The project links
`exemples/pei/pei_timer_gpiote.c` and uses the BLYST-NANO-DK's first LED.
It uses TIMER2 compare 0 -> PPI -> GPIOTE task out 0 to toggle the LED at
500 ms intervals. No timer/GPIOTE interrupt is enabled by the application.

Expected: `g_PeiStatus == 1` and the LED changes state twice per second.
`g_PeiStatus` is `2` for an unsupported LED port, `3` when no channel can be
allocated, and `4` when enabling the route fails. While debugging, verify
that there are no TIMER2 or GPIOTE IRQs associated with the toggles.

The physical device test is pending. Repeat with S132 and SDC advertising
active, using separate BLE-capable application projects, and confirm that the
controller continues advertising while the PEI-controlled LED changes state.
