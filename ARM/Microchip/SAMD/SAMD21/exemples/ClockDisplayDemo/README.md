# SAM D21 ClockDisplayDemo

Plug OLED1 into EXT1 with power disconnected. Import `ioc` and build the
matching Debug or Release IOsonata_SAMD21J18A library first.
The project links the existing generic display application.

The display shows a large HH:MM:SS clock starting at 12:00:00.
Short BUTTON1 presses advance minutes; holding it advances hours.
Time comes from the library SysTick timer; reset restores the initial time.
This demo does not enter Standby.

SPI uses SERCOM0: SCK PA07, MOSI PA06, CS PA05. Display D/C is PB06
and reset is PB05. BUTTON1 is PB04 with an internal pull-up.
SPI defaults to polling; `DISPLAY_SPI_INT_ENABLE` and
`DISPLAY_SPI_DMA_ENABLE` select the other transfer modes.
Hardware display validation is pending.
