# CorePowerTest — generic MCU test

> **Release 0.14 migration:** This Eclipse managed project now compiles the shared
> `exemples/misc/core_power_test.cpp` test. The legacy demo instructions below
> are historical reference only; they do **not** describe the current test flow.
> Check `src/board.h` for this board's UART, oscillator, timer and wake-pin setup.
> Use the generic test output and capability reporting to determine supported
> functionality; build and hardware validation remain target-specific.

## Historical demo notes (pre-migration)

# LPC54628 CorePowerTest

Build the IOsonata library and this Eclipse managed project in the same
configuration. Both configurations print through Flexcomm 0 at 1000000 baud:
P0.29 RX and P0.30 TX, connected to LPC-Link2 VCOM.

The generic demo prints the reset reason and a counter in RTC GPREG0, waits,
stops the UART and enters Deep power-down. Release P0.4 before entry. That
button is only sampled before entry; it cannot wake Deep power-down. Use the
hardware RESET pin to restart. RTC alarm/countdown wake is also supported by
the power implementation, but this demo does not configure an RTC event.

Seven backup words are available. They alias RTC backup words 0 through 6;
coordinate their use if the application also uses the RTC class. GPREG7 is
reserved for the RTC time-valid marker. Deep power-down loses SRAM.

Reset reporting exposes the hardware POR, brownout, reset-pin, watchdog and
software-reset flags. It does not infer a wake cause from an RTC flag that
could already have been pending. A POR report restarts the generic demo count.
The watchdog is not enabled by startup or power entry.

Compilation and register checks passed; board wakeup, retention and current
measurements are pending. D11 remains the current hardware validation target.
