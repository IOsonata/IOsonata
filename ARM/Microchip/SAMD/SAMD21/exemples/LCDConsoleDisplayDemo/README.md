# SAM D21 LCDConsoleDisplayDemo

Plug OLED1 into EXT1 with power disconnected. Import `ioc` and build the
matching Debug or Release IOsonata_SAMD21J18A library first.
The project links the existing generic display application.

The display shows the System 5x7 font sample, followed by a syslog counter
updated every two seconds. Output defaults to the display.

SPI uses SERCOM0: SCK PA07, MOSI PA06, CS PA05. Display D/C is PB06
and reset is PB05. BUTTON1 is PB04 with an internal pull-up.
SPI defaults to polling; `DISPLAY_SPI_INT_ENABLE` and
`DISPLAY_SPI_DMA_ENABLE` select the other transfer modes.
Maintainer hardware validation passed with all four interrupt/DMA combinations:
polling, interrupt only, DMA only, and interrupt with DMA. The display console
and syslog counter worked in each configuration. That run used the earlier
SPI master driver; the synchronous master transfers of Nov. 26, 2026 are not
run on hardware yet.
