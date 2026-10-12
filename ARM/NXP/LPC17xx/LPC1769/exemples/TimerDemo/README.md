# LPC1769 TimerDemo

Import `ioc` and `../../lib/ioc` into IOcomposer. Build the LPC1769 library,
then TimerDemo with the same Debug or Release configuration. The application
links `libIOsonata_LPC1769.a` and the common `exemples/timer/timer_demo.cpp`;
it does not compile a separate timer driver.

The default is TIMER0 at a requested 1 MHz, interrupt priority 7. Edit
`TIMER_DEMO_DEVNO` in `src/board.h`, or define it in the compiler options,
to exercise all four devices:

| DevNo | Hardware | Counter width |
| --- | --- | --- |
| 0 | TIMER0 | 32 bits |
| 1 | TIMER1 | 32 bits |
| 2 | TIMER2 | 32 bits |
| 3 | TIMER3 | 32 bits |

Connect a 3.3 V UART adapter RX to P0.2 (TXD0), and connect ground.
P0.3 is RXD0. Open the terminal at 115200 baud, 8-N-1, no flow control.
The pin selection follows the LPC1769 DfuBoot example. UART retargeting is
enabled in both build configurations; a semihosting debugger is not required.

Expect startup periods of 100, 1000 and 250 ms, then measured periods near
100000, 1000000 and 250000 us. The first reported period for each trigger can
be zero until its first callback. P0.22 toggles every 100 ms; observe it with
a scope or attach an external LED with a series resistor. Change the map in
`board.h` for your board. Trigger counters and accepted periods are also
available as `g_TriggerCount` and `g_TriggerPeriod` in the debugger.

The default library oscillator is IRC; absolute UART and timer timing inherits
its tolerance. A board may supply `MCUOSC` in board.h using McuOsc_t fields.
The timer selects the closest available rate and reports accepted periods.
At exactly 1 MHz the hardware counter wraps every 4294.967296 seconds, and
the timer interrupt must be serviced within 2147.483648 seconds to maintain
the software count. Shorten the rollover test by increasing TIMER_DEMO_FREQ
(e.g. zero selects the current timer peripheral clock).

The standalone linker layout starts at flash address zero. This project is
not configured for a DFU application slot. RTC, RIT, tick interrupts and
external capture are outside the current shared timer adapter.

Validation: project XML/path checks and Cortex-M3 compiler-to-assembly checks
of the common application at Debug/Release optimization, for DevNo 0..3.
Full firmware linking and physical-board validation remain pending.
