# LPC11U35 TimerDemo

Import `ioc` and the LPC11U35 library project
(`../../lib/ioc`) into IOcomposer/Eclipse. Build the library and TimerDemo
with the same Debug or Release configuration, then flash TimerDemo.

This project links the existing `exemples/timer/timer_demo.cpp`.
The default timer is DevNo 0 (CT16B0), running at 10 kHz with interrupt
priority 3. Change TIMER_DEMO_DEVNO in `src/board.h`, or define it in the
compiler options, to test each device:

| DevNo | Hardware | Counter width |
| --- | --- | --- |
| 0 | CT16B0 | 16 bits |
| 1 | CT16B1 | 16 bits |
| 2 | CT32B0 | 32 bits |
| 3 | CT32B1 | 32 bits |

The LED mapping matches the existing LPC11U35 Blinky example. P0.3 toggles
every 100 ms and P0.4 every 1 s. Trigger 2 runs every 250 ms and is visible
in the UART output and debugger counters.

Connect a 3.3 V UART adapter RX to P0.19 and connect ground. P0.18 is UART RX.
Open the terminal at 115200 baud, 8-N-1, without flow control. Expect three
startup lines reporting 100, 1000 and 250 ms, followed by measured periods
near 100000, 1000000 and 250000 us. A trigger's measured period is initially
zero until its first callback. Small variations reflect interrupt latency.

At 10 kHz the 16-bit counter wraps every 6.5536 s. Run DevNo 0 and 1 beyond
several wraps and confirm that the printed count continues increasing and
trigger periods stay stable. In the debugger, g_TimerInitOk should be true,
g_TriggerCount[0..2] should advance, and g_TriggerPeriod[0..2] should contain
the accepted periods in nanoseconds. Repeat with DevNo 2 and 3.

The default rate gives the 16-bit timers a 3.2768 s half-cycle interrupt
service limit. Higher timer frequencies shorten that interval; see
`tests/lpc/timer/README.md` for driver limits.

Hardware validation and a full firmware link remain pending.
