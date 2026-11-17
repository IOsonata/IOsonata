# ESP32-C3 ClockDemo

Import `ioc` as an existing Eclipse managed project. Build the ESP32C3 library
and ClockDemo in the same Debug or Release configuration. Both configurations
log through UART0 TX21/RX20 at 1,000,000 baud, 8N1. No debugger is required.

The demo runs three cycles of XTAL 40 MHz, RC_FAST and XTAL restoration.
Each source exercises dividers 1, 2, 4 and 1. On C3 these divide CPU and APB
together. A stopped Timer Group counter is restarted after each change to
check a 250 ms delay and 10 ms periodic interrupts. Delay calculations use
frequency in Hz so fractional MHz from the C3 divider is preserved.

RC_FAST currently uses the port's nominal 17.5 MHz value; it is not calibrated.
The timer and delay checks do not independently measure oscillator accuracy.
The LF step selects and measures RC_SLOW. RC32K is not offered by this C3
implementation, and an external 32 kHz crystal is not assumed.

At each `Entering sleep; press BOOT to resume` prompt, press and release BOOT
(GPIO9). The demo checks clock descriptors, frequency readback, timer counts
and interrupts after Light-sleep. There are nine sleep prompts across three
cycles. Comment out `CLOCK_DEMO_WAKE_PIN` in `src/board.h` for an automatic
clock-only run. The expected final line is `ClockDemo complete: 0 failures`.
No watchdog is enabled by the demo. ClockDemo still needs C3 board validation;
PowerOffDemo wake results do not validate runtime clock changes.

Build validation uses RISC-V GNU 15.2.0 with the Eclipse managed source lists
and compiler settings. It is a command-line compile/link check, not an Eclipse
invocation. Debug and Release compile all 204 C3 library sources and link the
demo. The generic demo remains shared with ESP32-C6.

The first C3 board run rejected all nine source selections. Startup now gates
unused peripherals before drivers initialize them. The busy check permits the
system timer infrastructure and independent USB Serial/JTAG clock, while
continuing to reject active peripheral users. This follows the startup clock
gating in ESP-IDF v5.5.1 `components/esp_system/port/soc/esp32c3/clk.c`.
The correction passes host register checks and Debug/Release builds; the C3
board retest is pending.
