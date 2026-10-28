# LPCXpresso54628 SPIFI quad flash demo

Build the matching IOsonata_LPC54628 library and import `ioc`. The project
links the shared `exemples/storage/nvm_demo.cpp` with the generic `Nvm`
driver on the SPIFI (`ARM/NXP/LPC546xx/src/spifi_lpc546xx.cpp`, SPI DevNo 10)
and the 128 Mbit quad SPI flash of the board. Output on the LPC-Link2 VCOM,
Flexcomm 0, 1000000 baud 8N1, in Debug and Release.

The flash: recent boards have a Winbond W25Q128JV, earlier ones a Micron
MT25QL128. `src/board.h` selects the Winbond part; define `SPIFI_FLASH_MT25Q`
there for the Micron part. With the wrong part, Init fails on the JEDEC id.
Reads use 0x6B (quad output, 8 dummy cycles) and page programs 0x32 (quad
input). On the Winbond part Nvm sets the Quad Enable bit (status register 2
bit 1, non volatile) once.

The SPIFI clock is the main clock divided: 73.3 MHz for the 80 MHz request
of the board header (`NVM_DEMO_SPI_RATE`). The NXP SDK example runs the
flash at 96 MHz.

The demo prints the geometry and the region (three 4 KB sectors below the
top three sectors of the flash), then checks: a boot stamp written and read
back, erase of the scratch sectors to all ones, a write across a page
boundary, programming clears bits and never sets them, unaligned erase and a
read past the end refused. It ends with "ALL PASS". Power cycle the board:
the boot count of the stamp goes up by one.

Status: Debug and Release build and link. The SPIFI driver passes the host
test in `tests/lpc/lpc546xx/spifi_test.cpp` with a model of the SPIFI and of
a serial NOR flash, under the generic Nvm and Flash drivers. No hardware
result yet.
