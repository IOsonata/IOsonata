# ESP32C6 SPI loopback

Uses the generic `exemples/spi/spi_loopback.cpp` through `SPI` and `DeviceIntrf`.
SPI device 0 selects GP-SPI2. Connect GPIO2 (MOSI) to GPIO7 (MISO).
GPIO6 is SCK and GPIO3 is automatic active-low CS; leave both unconnected
for this test. Check the board pin assignments before connecting hardware.

Build the matching IOsonata library and SpiLoopback managed project in
IOcomposer, using the same Debug or Release configuration. Flash the raw
binary at offset 0. The UART console runs at 1,000,000 baud; see `board.h`
for its pins. Debug and Release both use the UART console.

The test checks all four SPI modes at 1 MHz, six dummy-byte values,
129-byte transfers and command/read transactions. Expect the final line
`SPI master loopback PASS`.

The driver supports polling master, normal four-wire, 8-bit data, MSB/LSB
order, and GPIO or application-controlled chip select. Manual CS uses a
three-entry SCK/MISO/MOSI pin map. Automatic CS entries follow those pins;
the transfer selector is the zero-based CS index. MOSI is required. SCK and MISO may use port/pin -1 for MOSI-only
devices; receiving requires a connected MISO pin.

SPI2 also supports interrupt-driven slave transfers, with a 64-byte limit
per CS assertion and a required gap between frames for interrupt handling.
The C3/C6 SpiMasterSlave projects test that path with a software master.
DMA, master interrupt transfers, three-wire and multi-line modes are not
implemented. SpiLoopback selects the polling hardware master.
Rate selects the closest XTAL divider, with zero selecting the minimum.
Actual usable speed depends on GPIO matrix timing and board wiring.

Cross-build and modeled register tests do not establish hardware operation.
Hardware validation is pending for this target.
