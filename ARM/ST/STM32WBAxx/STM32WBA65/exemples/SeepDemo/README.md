# STM32WBA65I-DK1 EEPROM demo

Import `ioc` into IOcomposer as SeepDemo. Build the matching Debug or Release
configuration of `../../lib/ioc` first, then build and flash this example.
Open the ST-LINK virtual COM port at 1000000 baud, 8N1, no flow control,
and reset the board.

This project links the existing `exemples/storage/eeprom_demo.cpp`. Its board
configuration selects the generic Nvm driver for the onboard M95P32 SPI EEPROM:
4 MB, 512-byte pages and three address bytes. The legacy Seep modes remain
available for the existing I2C configurations; no new memory driver is added.

SPI3 (DevNo 2) uses SCK PA0, MISO PA1 and MOSI PB8, all AF6, with memory CS
PH3. The demo uses SPI mode 0 at a requested 4 MHz. OLED CS PE1 is held high
because the display shares this bus. No external wiring is needed.

The test checks the device ID, overwrites 512 bytes at addresses 0x100 through
0x2ff, reads them back and compares every byte. The write spans two pages.
Nvm issues write enable, splits at page boundaries and polls the WIP status.
These addresses are in the external EEPROM. Existing content there is replaced.

Expected output ends with:

```text
Writing 512 bytes at 0x100...
Validate readback...
EEPROM test passed.
```

Debug and Release compile and link with Arm GCC 13.2.1. The shared demo and
production Nvm pass the command-model test at -O0 and -Os with ASan/UBSan:
`python tests/storage/run_wba_eeprom.py` from the repository root. The model
checks page splitting, WREN/WIP, overwriting zeros with ones, unchanged bytes
outside the test range and deselection of the OLED. The original I2C Nvm,
Seep C++ and Seep C selections also pass syntax checks.
Hardware operation of this EEPROM demo has not yet been tested.
