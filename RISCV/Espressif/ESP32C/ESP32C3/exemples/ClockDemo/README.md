# ESP32-C3 ClockDemo

Import `ioc` as an existing Eclipse managed project. Build the ESP32C3 library
and ClockDemo in the same Debug or Release configuration. Both configurations
log through UART0 TX21/RX20 at 1,000,000 baud, 8N1. No debugger is required.

The demo runs three cycles of XTAL 40 MHz, RC_FAST and XTAL restoration.
Each source exercises dividers 1, 2, 4 and 1. On C3 these divide CPU and APB
together. A stopped Timer Group counter is restarted after each change to
check a 250 ms delay and 10 ms periodic interrupts. Delay calculations use
frequency in Hz so fractional MHz from the C3 divider is preserved.

RC_FAST is measured against XTAL through the RTC divide-by-256 calibration
input before the CPU source changes. A timeout leaves the CPU on its previous
source. CPU/APB readback and delays use the measured frequency. The timer and
delay checks do not independently measure oscillator accuracy.
The LF step selects and measures RC_SLOW. RC32K is not offered by this C3
implementation, and an external 32 kHz crystal is not assumed.

At each `Entering sleep; press BOOT to resume` prompt, press and release BOOT
(GPIO9). The demo checks clock descriptors, frequency readback, timer counts
and interrupts after Light-sleep. There are nine sleep prompts across three
cycles. Comment out `CLOCK_DEMO_WAKE_PIN` in `src/board.h` for an automatic
clock-only run. The expected final line is `ClockDemo complete: 0 failures`.
No watchdog is enabled by the demo.

Debug and Release compile all 204 C3 library sources and link the demo with
RISC-V GNU 15.2.0, using the managed source lists and compiler settings.
This is a command-line compile/link check, not an Eclipse invocation.

The board run after 04d1522a8 completed three cycles and nine Light-sleep
wakes with zero failures. XTAL and calibrated RC_FAST, shared CPU/APB dividers,
RC_SLOW selection, clock restoration, delays and timer interrupts passed.
RC_FAST measured 20,805,079-20,810,365 Hz. The 250 ms samples measured
250,065-252,360 us with 25 interrupts. These checks use the on-chip clock
references; they are not external frequency measurements.

Startup gates unused peripherals and clears unused reset-default RTC wake
enables. RC_FAST power and digital output are enabled before calibration.
A failed calibration leaves the previous CPU source selected.
UART RX after clock changes and sleep-current measurements remain untested.
The dots before startup output remain unexplained.
