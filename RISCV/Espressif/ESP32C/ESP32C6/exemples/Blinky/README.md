# ESP32-C6-DEV-KIT-N8 Blinky

This project targets the Waveshare ESP32-C6-DEV-KIT-N8. Its onboard
WS2812B-0807 RGB LED is connected to GPIO8; it cannot be driven by ordinary
GPIO toggles. The example cycles dim red, green and blue every 500 ms.

The example reserves RMT channel 0 and the RMT peripheral clock. It uses the
40 MHz crystal to generate the 24-bit GRB waveform and a 300 us low latch
interval. No SPI implementation, interrupts or DMA are needed. This is a
board-specific example, not a general RMT driver.

Clean and rebuild both IOsonata_ESP32C6 and Blinky in the same configuration.
The linker produces a Direct Boot image, following the C3 project pattern.
Flash the generated raw Blinky.bin at physical flash offset 0. Do not wrap
this binary in an ESP-IDF application image. Direct Boot must be enabled in
the chip's existing eFuse configuration; this example does not change eFuses.

Validation: the application and required library sources cross-compile and
link at -O0 and -Os with xPack RISC-V GCC 15.2.0. Host tests exercise clock
frequency decoding and startup register selection. Hardware boot and RGB
output remain to be checked on the board.

References:
- https://files.waveshare.com/wiki/ESP32-C6-DEV-KIT-N8/ESP32-C6-DEV-KIT-N8-Schematic.pdf
- https://documentation.espressif.com/esp32-c6_technical_reference_manual_en.pdf
- https://github.com/espressif/esp-idf/blob/v5.3/components/hal/esp32c6/include/hal/rmt_ll.h
- https://github.com/espressif/esp-idf/blob/v5.3/components/hal/esp32c6/include/hal/clk_tree_ll.h
