# Raspberry Pi Pico and Pico 2

Import the library and example projects from the matching MCU directory in IOcomposer. The libraries use the register headers from pico-sdk 2.2.0 in the sibling `external/pico-sdk` directory. Both targets run on Arm core 0. Pico 2 boots a Secure Arm image.

| | Pico (RP2040) | Pico 2 (RP2350A) |
|---|---|---|
| CPU, performance / low power | 125 / 62.5 MHz | 150 / 75 MHz |
| Timer device numbers | 0–3 system timer alarms, 4 SysTick | 0–3 TIMER0 alarms, 4–7 TIMER1 alarms, 8 SysTick |
| UART | UART0 or UART1, polling or interrupt | UART0 or UART1, polling or interrupt |

Alarms on the same timer block share its 1 MHz counter. Each device owns one compare channel and tracks elapsed time independently across Disable and Enable. The alarm can wake core 0 from light sleep. SysTick uses the CPU clock. Finish or disable a running SysTick timer before changing the CPU frequency.

The examples use a 3.3 V USB-to-UART adapter at 1,000,000 baud, 8-N-1. Connect GP0 (header pin 1) to adapter RX, GP1 (pin 2) to adapter TX, and GND (pin 3) to adapter GND. For the button wake test connect a momentary switch from GP15 (pin 20) to GND.

* **CoreClockTest** checks shared clocks and measures usDelay, msDelay and nsDelay at both CPU speeds. Use the physical board results for the port's delay factors.
* **TimerTest** checks each timer, including periodic interrupt timing, Disable/Enable, PowerOff/Init and sDelay with the low-power timers occupied. Timer setup compensation is zero pending board measurements.
* **CorePowerTest** checks timer and GPIO wake from light sleep, clock and UART rate after wake, and watchdog scratch registers 0–3.
* **UartLoopback** echoes bytes received on the adapter.

Hardware calibration, measured sleep current, dormant mode and UART DMA remain open.
