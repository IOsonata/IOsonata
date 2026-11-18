# SAM4LC LCDConsoleDisplayDemo

Connect OLED1 to EXT2 on SAM4L8 Xplained Pro with power disconnected.
Import `ioc` and build the matching Debug or Release
`IOsonata_SAM4LCxC` library first. The project links the generic display demo.

SPI0 uses SCK PC30, MOSI PA22 and GPIO chip select PB11.
Display D/C is PC08 and reset is PC09. MISO PA21 is unconnected on OLED1.
SPI defaults to polling. Set `DISPLAY_SPI_INT_ENABLE` and
`DISPLAY_SPI_DMA_ENABLE` in `src/board.h` to select other transfer modes.

The display shows console output using the system 5x7 font.
`CONSOLE_LOG_OUTPUT` selects display (default) or UART output.
UART uses the EDBG virtual COM port at 1000000 baud, 8N1.

Debug and Release builds passed with all four SPI interrupt/DMA combinations.
Hardware validation is pending.
