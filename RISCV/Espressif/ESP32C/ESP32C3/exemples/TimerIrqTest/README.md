# ESP32-C3 Timer Group IRQ hardware acceptance

Open the Eclipse Embedded CDT / IOcomposer managed project at
`RISCV/Espressif/ESP32C/ESP32C3/exemples/TimerIrqTest/ioc/`.
Use the ESP32-C3-DevKitM-1 with its CP2102N UART0 bridge on
GPIO21 (TX) and GPIO20 (RX); serial output runs at **1,000,000 baud**.

Use an ESP32-C3 IOsonata library built from the same branch, then
clean-build the TimerIrqTest target and flash the resulting application.

The test exercises both Timer Group counters simultaneously without the
board's WS2812 LED:

- **TG0/T0 (DevNo 0)**: C++ `Timer`, continuous 100 ms callback.
- **TG1/T0 (DevNo 1)**: C `TimerDev_t`, continuous 250 ms callback.
- Both have a 1 MHz tick clock and one available alarm.
- The main loop samples callback counters for 3 seconds without printing
  or allocating memory inside interrupts; both counts are checked with
  a conservative timing window.
- It disables the two alarms and verifies that callbacks stop.
- It restarts TG1 with a 120 ms single-shot alarm and verifies exactly one
  callback and no unexpected rearming.

Expected periodic counts after 3 seconds are approximately 30 on TG0
and 12 on TG1. Output includes individual `[PASS]` or `[FAIL]` checks
and a final `PASS: 0 failed` summary. `g_TimerIrqCount` and
`g_TimerUnexpectedEvents` remain ordinary C++ debugger-visible globals.

**Status:** **Hardware PASS, November 8, 2026 (ESP32-C3-DevKitM-1).**
The maintainer's UART log reported all checks passing with **0 failures**.
Both Timer Group counters ran concurrently at a 1 MHz tick clock.
The callbacks matched the programmed 100 ms and 250 ms intervals over
three successive one-second observation points:

| Elapsed | TG0/T0 (100 ms) | TG1/T0 (250 ms) |
| ---: | ---: | ---: |
| 1000 ms | 10 | 4 |
| 2000 ms | 20 | 8 |
| 3000 ms | 30 | 12 |

The hardware log further confirms:

- Both timer instances initialize correctly; each exposes one alarm.
- The C++ TG0/T0 and C TG1/T0 callbacks deliver periodic events independently.
- No unexpected timer events were recorded.
- Both timers stop generating interrupts after disable.
- TG1's 120 ms single-shot fires exactly once and does not rearm.

**Validation scope:** This establishes functional interrupt delivery and timer
lifecycle behavior in the tested configuration; it does not measure
microsecond-scale interrupt jitter, other timer frequencies, sleep/wakeup,
or frequency reconfiguration. Preserve those as separate validation targets.

**Maintainer's result:** `PASS: 0 failed`.
