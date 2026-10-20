# STM32WBA65I-DK1 joystick menu

Build `../../lib/ioc` and import this project's `ioc` directory as
JoystickMenuDemo. Select matching Debug or Release configurations. The
library must be rebuilt to include the new ADC4 implementation. No external
wiring or serial terminal is needed.

The shared `exemples/display/joystick_menu_demo.cpp` uses AdcDevice-compatible
ADC selection and the generic display API. `board.h` supplies mappings and
configuration only. It uses the working console demo's SPI3 mode 0, polling,
SCK PA0, MOSI PB8, CS PE1, D/C PE0 and reset PE3.

Release the joystick after startup. Up/down choose a menu item; select opens
it. The LED item toggles LD6 green (PD8, active low). In Level, arrows adjust
from 1 to 10 and select returns. About returns on select or left. Each press
needs a release; there is no automatic repeat. Samples are taken about every
10 ms and three equal samples debounce a transition. A failed ADC read shows
an error and requires release before accepting the next press.

The joystick is PA3, **ADC4 input 6**, not input 8. The current ST board
support header and DS14736 table 23 agree; UM3462 revision 1 table 6 says 8.
The ladder levels are approximately 0, 0.20, 0.40, 0.61, 0.80 and 1.00 of
VREF+ for select, left, down, up, right and idle. The board selects VREF+
3.3 V. Direction decoding uses ratios, not an absolute voltage threshold.

ADC4 is intentionally limited to software-triggered single rounds in polling
mode. Logical channels 0..7 select ADC inputs 1..10 through PinP.PinNo.
It supports 6/8/10/12-bit resolution, unity gain, single-ended inputs and a
caller-supplied VREF+ voltage. HCLK/16, 814.5-cycle acquisition. Interrupt,
continuous, differential, FIFO and oversampling requests are unsupported;
the driver rejects their configuration. It does not implement the internal
radio temperature measurement. An ADC object owns the peripheral until
PowerOff or destruction. No shared AdcDevice API was changed.

Validation: Debug and Release compile/link. `tests/stm32wba/run_adc.py` runs
the production ADC4 implementation at -O0 and -Os against a register model,
including calibration, conversion, ownership, multi-channel reads, voltage
scaling and timeout/overrun recovery. `tests/display/run_ssd1315.py` runs the
shared menu with scripted ADC input through the real SSD1315 driver and SPI
RAM model under ASan/UBSan at -O0 and -Os. It checks the ladder, debounce,
held keys, navigation, level limits, LED state and ADC read failures.
Maintainer hardware result, October 19, 2026: the joystick example works on
the STM32WBA65I-DK1.

References:

- https://github.com/STMicroelectronics/stm32wba65ri-dk1-bsp/blob/main/stm32wba65i_discovery.h
- https://www.st.com/resource/en/datasheet/stm32wba65ri.pdf
- https://www.st.com/resource/en/user_manual/um3462-discovery-kit-with-stm32wba65ri-mcu-stmicroelectronics.pdf


The same shared source also supports the optional stopwatch in the separate
[StopwatchDemo](../StopwatchDemo/README.md) project. This project's three-item
menu and controls are unchanged.
