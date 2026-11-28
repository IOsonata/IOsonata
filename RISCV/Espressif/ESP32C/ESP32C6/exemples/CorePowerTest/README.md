# CorePowerTest — generic MCU test

> **Release 0.14 migration:** This Eclipse managed project now compiles the shared
> `exemples/misc/core_power_test.cpp` test. The legacy demo instructions below
> are historical reference only; they do **not** describe the current test flow.
> Check `src/board.h` for this board's UART, oscillator, timer and wake-pin setup.
> Use the generic test output and capability reporting to determine supported
> functionality; build and hardware validation remain target-specific.

## Historical demo notes (pre-migration)

# ESP32-C6 CorePowerTest

Import `ioc` as an Eclipse managed project. Build the matching
ESP32C6 IOsonata library and CorePowerTest for the same
Debug or Release configuration. The UART0 console is TX16/RX17
at 1,000,000 baud on the DevKit N8 USB-UART bridge, not the
native USB Serial/JTAG port.

## Deep-sleep (default)

The committed board profile uses an external momentary switch
from GPIO3 to GND with `POWER_DEMO_RESUME_WAKE=0`. Release
it before reset. At the power-off prompt, press GPIO3 to
restart the MCU. The reset reason is expected to indicate
GPIO wake (previously reported 0x100); no normal stack data
survives a reset-on-wake transition. The demo records LP SRAM
and restart information separately. The UART is powered off
through its normal driver API before SystemPowerOff.

Prior hardware reports include seven consecutive GPIO3
Deep-sleep wake/restart cycles and three at an earlier revision.
These reports do not establish sleep current.

## Resumable Light-sleep

For BOOT-button wake instead, set `WAKE_BUT_PIN=9` and
`POWER_DEMO_RESUME_WAKE=1` in `src/board.h`. Release
BOOT before resetting, then press it after each
`Entering Standby` prompt. CorePowerTest verifies the
128-byte static and stack RAM samples and prints the UART
configured rate and CPU/APB clock restoration checks.
Normal UART output after resume exercises its TX path.

Previous Light-sleep GPIO9 wakes and RAM samples passed
with the UART output drained before sleep. The code
split and additional post-wake reports have **not** been
compiled or hardware revalidated as part of this review.
A return from SystemSleep does not by itself establish
PMU entry, and sleep-current measurements remain pending.

## System-owned sleep coordination (review 0.14)

For resumable sleep, CorePowerTest now leaves its UART enabled and calls
`SystemSleep()` directly after draining pending output. The target system
coordinates temporary UART suspension and restoration with the driver's
clock and interrupt state, without changing its enable reference count.
The UART-wake profile retains PICOUART's own receive preparation.
Previous runs used a different suspend sequence; this refactor requires
a new clean library/demo build and board test before validation.
