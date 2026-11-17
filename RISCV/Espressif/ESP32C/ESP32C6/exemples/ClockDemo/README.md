# ESP32-C6 ClockDemo

Import `ioc` as an existing Eclipse managed project. Build the ESP32C6 library
and ClockDemo in the same configuration, then flash ClockDemo.elf using the
normal IOcomposer flash command. Debug and Release both use UART0 at
1,000,000 baud, 8N1, without flow control. The DevKit N8 USB-to-UART bridge
connects TX16 and RX17; the native USB JTAG/serial port is not this UART.
Open the terminal before pressing RESET. No jumper or button is needed.

The shared source is `exemples/misc/clock_demo.cpp`. Board configuration selects
three repetitions of the following sequence:

1. Select the 40 MHz XTAL, calibrated RC_FAST, then XTAL again.
2. At each source, select APB divisors 1, 2, 4, then 1. Check the selected
   frequency and readback and confirm the CPU rate remains unchanged.
3. After every APB request, run Timer 0 at 1 MHz with a 10 ms interrupt period.
   Report the timer's elapsed time during a 250 ms CPU delay and the number of
   interrupts. Check the delay is within 2 percent of 250 ms. Stop the timer
   before the next clock change.
4. At each CPU source, select and calibrate RC_SLOW, RC32K, then RC_SLOW again.
   Print the measured LF frequency. No external 32 kHz crystal is assumed.

Each operation prints its result. The final line should read:

```
ClockDemo complete: 0 failures
```

A rejected core source is reported as FAIL and its dependent steps are skipped.
The timer result checks that the counter advances and interrupt counts agree
with its elapsed time, allowing one interrupt at the sampling boundary. It
is not an independent measurement of the timer's absolute frequency. The
250 ms delay uses the calibrated CPU frequency, including fractional MHz.
The timer provides a separate PLL-derived reference on C6. UART text
remaining readable checks TX operation throughout the source/rate changes;
this demo does not check UART RX or other peripheral drivers.

On normal completion, the selected sources are XTAL for the CPU and RC_SLOW
for the LF clock, with APB at 40 MHz. The original PLL CPU source is not restored;
the current selector does not expose PLL selection. RESET restores the normal
startup configuration. No watchdog or power-off operation is enabled.

Validation: all 206 C6 library sources and this example compiled and linked
in Debug and Release using RISC-V GNU 15.2.0 and the Eclipse-managed source
lists/settings. The calibrated timing change awaits hardware validation.

| Build | text | data | bss |
| --- | ---: | ---: | ---: |
| Debug | 47400 | 584 | 3008 |
| Release | 21918 | 564 | 2740 |

Hardware results before calibrated CPU timing (`e5044c35f`): all three cycles
completed with zero failures. Source selection, APB readback, LF calibration,
UART TX and timer interrupts passed. XTAL gave about 250021 us and 25 interrupts;
RC_FAST gave about 310200 us and 31 interrupts because delay calculation still
used the nominal 20 MHz rate. LF measurements were about 147.7 kHz RC_SLOW and
18.0 kHz RC32K. These observations do not specify oscillator accuracy.

C6 now measures RC_FAST against XTAL before selecting it. CPU and APB readback
use that measured rate. A failed or out-of-range measurement leaves the previous
clock selection and descriptor unchanged. Each RC selection measures again;
there is no background tracking of temperature or supply drift. ROM timing still
uses its integer-MHz interface, while IOsonata delays use full Hz precision.

The updated demo accepts the measured RC rate and derives APB requests from it.
On this board, expect CPU near 16.1 MHz and the 250 ms delay near 250000 us,
with about 25 interrupts for both sources. The new delay check is separate from
the counter/interrupt check; it can now fail even if callbacks agree with elapsed
time. Rebuild both the C6 library and ClockDemo before flashing.

Temporary production-source checks passed at O0 and Os for fractional-frequency
CPU/APB readback, repeated switching, failed calibration rollback, interrupt-mask
restoration, microsecond/nanosecond delay arithmetic and cycle-counter wrap.
Existing C3/C6 startup checks and five-chip timer checks also passed.
