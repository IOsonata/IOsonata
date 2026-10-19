# nRF52832 I2C scanner

Import `ioc` into IOcomposer. Build the matching Debug or Release library
under `../../lib/ioc` first. Both configurations link the shared
`exemples/i2c/i2c_scan.cpp`.

I2C 1 uses SDA P0.23 and SCL P0.24 at 100 kHz, matching the existing
I2CMasterSlave master pin map. Connect the device and common ground; provide
bus pull-ups when the device board has none.

The scanner probes addresses 0x08 through 0x77 by reading one byte. Nordic
StartTx only selects an address, so an address-only scan would report every
address. A read may advance a device's current register pointer or consume
queued data. Devices that do not accept reads may not be found.

Release output uses UART0 at 115200 baud, 8N1, no flow control. Debug output
uses the debugger's semihosting console, following the shared example.
The default UART pins use BLUEIO_UART mappings from blueio_board.h.
Define NORDIC_DK in board.h for RX P0.08 and TX P0.06.

Reset to scan again. Hardware validation remains pending.

Validation: the example translation unit compiles for Cortex-M4 in Debug
and Release using CMSIS compiler intrinsics. The Nordic SDK is unavailable
locally, so a full target library build and firmware link were not run.
`python tests/i2c/run_scan.py` passes at -O0 and -Os for both board
configurations, detecting three modeled responders among absent addresses.
