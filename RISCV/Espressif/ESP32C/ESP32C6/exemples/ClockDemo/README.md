# ESP32-C6 ClockDemo

Import `ioc` as an existing Eclipse managed project. Build the ESP32C6 library
and ClockDemo in the same configuration, then flash ClockDemo.elf using the
normal IOcomposer flash command. Debug and Release both use UART0 at
1,000,000 baud, 8N1, without flow control. The DevKit N8 USB-to-UART bridge
connects TX16 and RX17; the native USB JTAG/serial port is not this UART.
Open the terminal before pressing RESET. No jumper or button is needed.

The shared source is `exemples/misc/clock_demo.cpp`. Board configuration selects
three repetitions of the following sequence:

1. Select the 40 MHz XTAL, nominal 20 MHz RC_FAST, then XTAL again.
2. At each source, select APB divisors 1, 2, 4, then 1. Check the selected
   frequency and readback and confirm the CPU rate remains unchanged.
3. After every APB request, run Timer 0 at 1 MHz with a 10 ms interrupt period.
   Report the timer's elapsed time during a 250 ms CPU delay and the number of
   interrupts. Stop the timer before the next clock change.
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
250 ms delay measurement reveals CPU-clock timing differences; RC_FAST uses
a nominal frequency and its delay need not be exactly 250000 us. UART text
remaining readable checks TX operation throughout the source/rate changes;
this demo does not check UART RX or other peripheral drivers.

On normal completion, the selected sources are XTAL for the CPU and RC_SLOW
for the LF clock, with APB at 40 MHz. The original PLL CPU source is not restored;
the current selector does not expose PLL selection. RESET restores the normal
startup configuration. No watchdog or power-off operation is enabled.

Validation: all 206 C6 library sources and this example compiled and linked
in Debug and Release using RISC-V GNU 15.2.0 and the Eclipse-managed source
lists/settings. Hardware execution is pending.

| Build | text | data | bss |
| --- | ---: | ---: | ---: |
| Debug | 44704 | 584 | 3008 |
| Release | 20810 | 564 | 2740 |

The first hardware run reported nine core-source failures and skipped every
APB/LF check. C6 PCR bus gates reset enabled, and startup had left those gates
on. The runtime clock checks consequently treated unused peripherals as active
users. Startup now clears the unused bus gates before driver initialization;
drivers enable their own gates. Flash, UART, interrupt routing, Timer Groups,
SYSTIMER and USB Serial/JTAG are preserved. USB Serial/JTAG is excluded from
the CPU/APB user list because its PLL clock is unchanged by these calls.
Rebuild the C6 library as well as ClockDemo for this correction.

A temporary register model reproduced the refusal with reset-enabled gates,
then passed repeated source/APB changes after startup initialization at O0 and
Os. It also checked preserved boot clocks and refusal with active peripheral,
timer and non-XTAL UART clocks. The existing C3/C6 startup checks passed.
The corrected firmware still requires the board retest.

The next hardware run passed all core-source selections, APB requests/readback
and LF selections over three cycles. Timer counters advanced, but every sample
reported zero interrupts (36 failures). TimerInit restored the initially masked
CPU state after the interrupt routing helper had enabled delivery. The timer
now enables global delivery after restoring its outer setup mask, following
the existing Espressif interrupt installation behavior. The demo also allows
2 ms for polling UART output to leave the hardware FIFO before clock changes.

The board reported about 250018 us for the XTAL delay and 309900 us for RC_FAST,
consistent with an RC_FAST rate of roughly 16.1 MHz against its nominal 20 MHz
value. LF calibration reported about 147.8 kHz for RC_SLOW and 18.0 kHz for
RC32K. These are observations from this board, not accuracy specifications.
The timer fix passed the existing five-chip Timer Group checks and a temporary
model starting with global interrupts masked, each at O0 and O2. C6 Debug and
Release library/demo builds passed; corrected interrupt delivery and complete
UART output still need the board retest.
