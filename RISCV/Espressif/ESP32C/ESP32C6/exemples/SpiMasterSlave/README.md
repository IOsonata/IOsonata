# ESP32C6 SpiMasterSlave

Uses `exemples/spi/spi_master_slave.cpp` with SPISoft as master and hardware
SPI2 as interrupt-driven slave. No external device is required.

| Signal | Master GPIO | Slave GPIO |
| --- | --- | --- |
| CS | 0 | 1 |
| SCK | 2 | 3 |
| MOSI | 4 | 6 |
| MISO | 5 | 7 |

Connect each row with a jumper. These four jumpers can stay fitted when
switching to `I2cMasterSlave`: GPIO4-GPIO6 carries SDA and GPIO5-GPIO7 carries
SCL; GPIO0-GPIO1 and GPIO2-GPIO3 are unused by that example. LP I2C fixes its
master pins to GPIO6/GPIO7, so SPI uses this arrangement to share the wiring.

Build the matching library and example, then
flash at offset 0. UART output uses 1,000,000 baud, 8N1. Debug and Release
configurations both report results over UART.

Tests modes 0 through 3 at a nominal software rate of 10 kHz, full-duplex
frames of 1, 16 and 64 bytes, repeated frames, and short CS frames. The
non-DMA slave buffer holds at most 64 bytes. The master must allow time
between frames for the completion interrupt to copy data and rearm.

This tests hardware SPI slave operation; use SpiLoopback separately to test
the hardware master.

C6 hardware validation passed on 2026-11-10 at `f9389531`, with a polling
UART console: all four SPI modes, 1/16/64-byte frames, repeated transfers
and short-frame rearming. The final result was 1 passed, 0 failed, 0 skipped.

The console now uses UART interrupts to check concurrent UART and SPI
interrupt delivery. This console configuration still needs a board run.
