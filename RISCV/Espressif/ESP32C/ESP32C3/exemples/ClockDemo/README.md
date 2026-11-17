# ESP32-C3 ClockDemo

Import `ioc` as an existing Eclipse managed project. Build the ESP32C3 library
and ClockDemo in the same Debug or Release configuration. Both configurations
log through UART0 TX21/RX20 at 1,000,000 baud, 8N1. No debugger is required.

The demo runs three cycles of XTAL 40 MHz, RC_FAST and XTAL restoration.
Each source exercises dividers 1, 2, 4 and 1. On C3 these divide CPU and APB
together. A stopped Timer Group counter is restarted after each change to
check msDelay(250), nsDelay(250000000) and 10 ms periodic interrupts. Delay calculations use
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
cycles. Comment out `CLOCK_DEMO_WAKE_PIN` in `src/board.h` and set `CLOCK_DEMO_UART_RX_TEST` to 0 for an automatic
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

The default run now also checks polling UART receive. At each
`UART RX: send U within 30 s` prompt, send uppercase `U` (then Enter if the
terminal buffers a line). There are ten receive prompts: one at startup and
one after each of nine BOOT wakes. Receive timeout counts as a failure and
the demo continues. Do not send continuously while UART is disabled for sleep.
These checks use the existing generic ClockDemo receive function; interrupt
and DMA receive modes are not enabled.

Rebuild both the library and demo to include the shared RISC-V delay changes.
The header uses the port's cycle counter and current CPU frequency for both
delay units. The receive-enabled hardware run is recorded below.

Receive-enabled command-line build sizes:

| Build | text | data | bss |
| --- | ---: | ---: | ---: |
| Debug | 50720 | 608 | 2764 |
| Release | 24768 | 596 | 2760 |

## Receive-enabled hardware run, November 17, 2026

The maintainer completed three XTAL / RC_FAST / XTAL-restore cycles with
`ClockDemo complete: 0 failures`, following the C++11 initialization fix.
The submitted log does not identify Debug or Release.

- All nine BOOT Light-sleep wakes restored CPU/APB clocks and LF descriptors.
- All ten polling UART receive prompts passed: startup and after every wake.
- All 36 APB requests and nine post-wake APB restorations passed.
- msDelay: 45 samples, 250018-250478 us for a 250 ms request.
- nsDelay: 45 samples, 250018-250137 us for a 250 ms request.
- Every delay sample reported 25 timer interrupts.
- RC_FAST measured 20811686-20815652 Hz; RC_SLOW measured 135730-136293 Hz.

The largest observed delay error was 0.1912 percent. These checks validate
clock changes, timer operation and polling UART receive across repeated sleep;
they do not measure sleep current, absolute oscillator accuracy, interrupt/DMA
UART receive, or short nanosecond pulses. Dots preceding startup remain
unexplained; the three-cycle run itself completed without a reported failure.
