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

**Status:** The example project is prepared for target validation. No
TG0/TG1 interrupt or single-shot hardware acceptance result is claimed
until the maintainer supplies the UART output. CryptoTest's existing
1 MHz benchmark timer was an initialization and measurement check, not
an interrupt-delivery check.
