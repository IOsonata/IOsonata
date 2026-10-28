# LPCXpresso54628 LCD test pattern

Build the matching IOsonata_LPC54628 library and import `ioc`. The project
links the shared `exemples/display/fb_display_demo.cpp`, which builds
everything from the macros of `src/board.h`: the SDRAM first (`SDRAM_`, the
board values of SdramTest), then the 4.3 inch 480 x 272 TFT panel on
`LcdLpc546xx` (`ARM/NXP/LPC546xx/src/lcd_lpc546xx.cpp`, `DISPLAY_DRIVER`),
a display controller set up from `DisplayCfg_t`: the pins (`TFT_PINS`: no
DCX, the backlight on P3_31, no reset, then LCD_PWR on P2_11, the clock, sync
and data pins), the size and the panel timings (`DISPLAY_TIMING`, those of
the NXP SDK lcdc examples: HSYNC 2, front porch 8, back porch 43; VSYNC 10,
front porch 4, back porch 12; HSYNC and VSYNC active low), 9 MHz asked (9.17
MHz, 220 MHz / 24), RGB565 on VD3 to VD7, VD10 to VD15, VD19 to VD23. The
frame buffer is at the start of the SDRAM (0xA0000000), 261 KB. Output on the
LPC-Link2 VCOM, Flexcomm 0, 1000000 baud 8N1 in Release; the Debug build
uses semihosting.

The demo shows, about 4 seconds each, and prints each step:

1. Eight vertical color bars, from left to right: red, green, blue, white,
   yellow, cyan, magenta, black.
2. A white frame on the edge pixels of the screen and a green cross in its
   middle.
3. Text lines in the 5 x 7 system font until the screen scrolls.
4. Landscape, portrait, landscape inverted, portrait inverted: a red bar and
   the name of the orientation at the top left of each.
5. Touch, in landscape then in portrait, about 10 seconds each: a dot under
   each contact (one color per contact), the contacts printed with their
   coordinates, "released" when the finger is lifted.

Wrong colors in step 1 mean a wrong RGB order: add `DISPL_TIMING_BGR` to
the flags of `DISPLAY_TIMING`. A frame cut or moved in step 2 means a timing
to change. "SDRAM init failed" stops the demo before the panel is set up.

The touch controller is on Flexcomm 2 (P3_23 SDA, P3_24 SCL, 100 kHz),
reset P2_27, interrupt P4_0 (the `TOUCH_` macros). The board ships with a
RK043FN66HS-CTG (GT911) or a RK043FN02H-CT (FT5406) panel: the demo tries the
GT911 at 0x5D, then the FT5406 at 0x38, and prints the one found. A dot away
from the finger in step 5 means a wrong axis setting: `TOUCH_GT911_FLAGS` or
`TOUCH_FT5406_FLAGS` (`TOUCH_FLAG_XY_SWAP`, `TOUCH_FLAG_X_INV`,
`TOUCH_FLAG_Y_INV`). See [touch panels](../../../../../../docs/display/touch.md).

Status: Debug and Release build and link. The drivers pass the host tests
`tests/lpc/lpc546xx/lcd_test.cpp` and `tests/display/touch_test.cpp`. No
hardware result yet.
