# Capacitive touch panels: FT5406 and GT911

`Touch` in `display/touch.h` is a `Device` on an injected `DeviceIntrf`, I2C
in general. `Read` returns the contacts in the pixel coordinates of a display,
in the orientation set with `Orientation`, the same as `DisplayDotMatrix`: a
point read can be drawn at once. The drivers are independent of the MCU, use
no heap and no interrupt. Add `src/display/touch.cpp` and the controller
source to the target library project.

| Controller | Class | Source | Contacts | I2C address |
| --- | --- | --- | --- | --- |
| FocalTech FT5406 | `TouchFt5406` | `src/display/touch_ft5406.cpp` | 5 | 0x38 |
| Goodix GT911 | `TouchGt911` | `src/display/touch_gt911.cpp` | 10 (5 on most panels) | 0x5D or 0x14 |

## Configuration

`TouchCfg_t` gives:

- `DevAddr`: the 7 bit address, 0 for the default of the controller.
- `Width`, `Height`: the panel size in pixels with the display in landscape
  (`DISPL_ORIENT_LANDSCAPE`, the native orientation of the panel).
- `Flags`: how the axes of the controller lie on the panel.
  `TOUCH_FLAG_XY_SWAP` when the X of the controller runs along the panel
  height, then `TOUCH_FLAG_X_INV` and `TOUCH_FLAG_Y_INV` to invert the panel
  axes.
- `RstPin`: reset, active low. `IntPin`: interrupt. PortNo -1 when not
  connected.

The FT5406 reports pixels. The GT911 range is read from its configuration
(X and Y output maximum) and scaled to the panel size; the configuration is
not written. With the reset pin, Init resets the controller: FT5406 5 ms low,
then 300 ms before it answers; GT911 the address sequence of the NXP SDK
(reset low 20 ms, the interrupt pin at the address level 1 ms before the
reset goes high, low 50 ms after it, then released). Without the GT911 pins,
the address is the one the board gives.

## Reading

`Read(pPoints, MaxPoints)` returns the number of contacts stored, 0 when not
touched or disabled, -1 when the controller does not answer or is not set up.
Each point has `X`, `Y` and the track `Id` of the controller, kept while the
contact stays. The FT5406 contacts lifted are not returned. The GT911 gives
new coordinates when its buffer status is set; the last ones are returned
until then. Contacts out of the controller range are dropped.

`Disable` puts the controller in hibernate (FT5406) or sleep (GT911, with the
interrupt pin held low) mode; `Enable` wakes it up with the reset. Without
the reset pin (and the interrupt pin for the GT911) `Disable` does nothing.
`Reset` resets and sets up the controller again.

```cpp
static TouchGt911 touch;
// i2c has already been initialized; cfg remains valid.
if (touch.Init(cfg, &i2c)) {
    TouchPoint_t pt[5];
    touch.Orientation(display.Orientation());
    int n = touch.Read(pt, 5);
    for (int i = 0; i < n; i++) {
        display.Fill(pt[i].X, pt[i].Y, 3, 3, 0xFFFF);
    }
}
```

## LPCXpresso54628

The 4.3 inch panel of the board is a RK043FN66HS-CTG (GT911) or, on older
kits, a RK043FN02H-CT (FT5406); the board part number does not tell which.
Both are on Flexcomm 2 (P3_23 SDA, P3_24 SCL, the bus of the WM8904), reset
P2_27, interrupt P4_0. LcdDemo (`exemples/display/fb_display_demo.cpp`, the
`TOUCH_` macros of its `board.h`) tries the GT911, then the FT5406, the
FT5406 with `TOUCH_FLAG_XY_SWAP` (NXP SDK touch_cursor). See
[LcdDemo](../../ARM/NXP/LPC546xx/LPC54628/exemples/LcdDemo/README.md).

## Validation

`tests/display/run_touch.py` runs `tests/display/touch_test.cpp` at -O0 and
-Os with the address and undefined behavior sanitizers. The test has an I2C
bus model with an FT5406 and a GT911 and a model of the reset and interrupt
pins. The FT5406 is reset by its reset pin low 5 ms or more and answers 300 ms
after it goes high. The GT911 takes its address from the interrupt pin when
the reset pin goes high, needs the reset low 10 ms or more, the interrupt
level set 100 us before the rise, the interrupt pin low 50 ms from 5 ms after
it, then released, and answers 55 ms after the rise. The test checks the
configurations refused, the reset sequences and addresses, the failures of
each transfer, the contacts, their ids and the limits, the axes settings in
the four orientations against the frame buffer mapping of `LcdLpc546xx`,
Disable, Enable and Reset, and the detection of the LcdDemo board. All 131
mutants of the three sources are killed. No hardware result yet.
