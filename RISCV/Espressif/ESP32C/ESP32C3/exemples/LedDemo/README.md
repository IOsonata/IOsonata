# ESP32C3 LED demo

Uses the shared `exemples/misc/led_demo.cpp`, `SPI` and `LedWs2812`.
The onboard addressable RGB LED on GPIO8 cycles red, green, blue and off,
with 500 ms per step. No external wiring is required. This pin map matches
the C3 devkit and Waveshare ESP32-C6-DEV-KIT-N8 used for this port; change
`board.h` for boards with a different LED data pin.

SPI device 0 drives only MOSI. SCK, MISO and CS are unused. The requested
3,333,333 Hz rate gives 300 ns SPI bits from a 40 MHz crystal: WS2812 zero
uses 1000 and one uses 1110. The driver leaves MOSI low for 300 us after
each frame. One pixel fits within the SPI hardware's 64-byte buffer.

Build the matching IOsonata library and LedDemo managed project in
IOcomposer, using the same Debug or Release configuration. Flash the raw
binary at offset 0. No UART console or semihosting is required.

The generic demo selects SPI with `LED_DEMO_SPI_ENABLE`. Other boards
retain their GPIO/PWM demonstration unless this option is enabled.
Hardware validation is pending.

UART progress output is enabled at 1,000,000 baud, 8N1, without flow control.
Open the board UART console before resetting. It prints initialization and each
color transition. `transfer returned` means the call returned; it does not prove
that the LED accepted the waveform. Set `LED_DEMO_TRACE` to 0 in `board.h` to
disable progress output after testing.
