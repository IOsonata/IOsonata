# ESP32C3 TimerIrqTest

Import this `ioc` project and `ESP32C3/lib/ioc` into IOcomposer. Build both
with matching Debug or Release configurations, then flash `TimerIrqTest.bin`
using Direct Boot at offset zero. Open the UART console at 1,000,000 baud,
8N1, no flow control, and reset the board to see the complete report.

UART0 TX is GPIO21, RX is GPIO20. Connect adapter RX to TX, adapter TX to RX,
and common ground. Use a 3.3 V adapter. Board wiring is in `src/board.h`.

The project links `exemples/timer/timer_dual_irq_test.cpp` unchanged.
It discovers the available high-frequency timers through the generic API:

- The first timer uses C++ initialization and a 100 ms periodic alarm.
- If a second timer exists, it uses C initialization and a 250 ms alarm.
- After three seconds, counts should be about 30 and 12 respectively.
- Disabling the timers must stop callbacks.
- Each timer is restarted through the C API for a 120 ms single-shot alarm.
  Each must fire exactly once and remain quiet afterward.

C2 runs the single-timer path; C3/C5/C6/C61 run both timers. The single-timer
path also exercises the C API through the existing Timer object's conversion
to `TimerDev_t *`. No second initializer or extra hardware timer is needed.

Expected final output: `PASS: 0 failed`. The test takes about four seconds.
Debug and Release both report through UART; no LED or semihosting is used.
Cross-build and host checks do not establish hardware validation.
