# ESP32C3 SpiMasterSlave

Uses `exemples/spi/spi_master_slave.cpp` with SPISoft as master and hardware
SPI2 as interrupt-driven slave. No external device is required.

| Signal | Master GPIO | Slave GPIO |
| --- | --- | --- |
| CS | 0 | 1 |
| SCK | 2 | 3 |
| MOSI | 4 | 5 |
| MISO | 6 | 7 |

Connect each row with a jumper. Build the matching library and example, then
flash at offset 0. UART output uses 1,000,000 baud, 8N1. Debug and Release
configurations both report results over UART.

Tests modes 0 through 3 at a nominal software rate of 10 kHz, full-duplex
frames of 1, 16 and 64 bytes, repeated frames, and short CS frames. The
non-DMA slave buffer holds at most 64 bytes. The master must allow time
between frames for the completion interrupt to copy data and rearm.

This tests hardware SPI slave operation; use SpiLoopback separately to test
the hardware master. Hardware validation is pending.
