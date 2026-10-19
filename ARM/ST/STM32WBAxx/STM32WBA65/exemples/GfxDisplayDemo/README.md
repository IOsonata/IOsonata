# STM32WBA65I-DK1 graphics demo

Import `ioc` as GfxDisplayDemo and build the matching Debug or Release library
in `../../lib/ioc` first. The project links the shared
`exemples/display/gfx_display_demo.cpp` through generic iGfx and LcdSSD1315.

The onboard 128x64 monochrome OLED cycles through a line fan, nested
rectangles, a checkerboard and a bouncing box. Coordinates follow the display
size. The box stays inside its border and its old position is erased each
frame. Colours are black and white on this panel.

No external wiring is needed. The pin map and SPI mode match the working
LCDConsoleDisplayDemo: SPI3 (DevNo 2), SCK PA0, MOSI PB8, CS PE1, D/C PE0
and reset PE3. SPI is mode 0, 8-bit MSB first, polling, at a requested 4 MHz.

Validation: Debug and Release compile and link. The SPI OLED RAM model runs
the shared demo through the real iGfx and SSD1315 driver at -O0 and -Os with
address and undefined-behaviour sanitizers. It checks initialization, drawing
bounds, distinct pages, rectangle and checkerboard pixels, and animation
cleanup. Run `python3 tests/display/run_ssd1315.py` from the repository root.
The existing Nordic colour-panel board configuration also compiles with the
shared demo. The graphics demo itself has not yet been tested on hardware.
