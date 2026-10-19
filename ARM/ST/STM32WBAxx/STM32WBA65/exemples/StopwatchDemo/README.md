# STM32WBA65I-DK1 stopwatch

Build the matching Debug or Release library in `../../lib/ioc`, then import
this project's `ioc` directory as StopwatchDemo. No external wiring or serial
terminal is required.

This project reuses `exemples/display/joystick_menu_demo.cpp` with
JOYSTICK_STOPWATCH enabled in board.h. It starts on the stopwatch page and
adds Stopwatch to the existing LED/Level/About menu. The original
JoystickMenuDemo keeps its three-item menu without this option.

Release the joystick after startup, then use:

| Joystick | Action |
|---|---|
| Select | Start or pause; press again to resume |
| Right | Record the current elapsed time as a lap; total time continues |
| Left | Reset total and lap while paused; ignored while running |
| Up/down | Return to the menu; the stopwatch continues running |

Each action requires a release, using the hardware-tested joystick debounce.
Select Stopwatch in the menu to return to its page. Elapsed time displays as
HH:MM:SS.cc, refreshed about ten times per second while running. At
99:59:59.99 it pauses; reset before starting again.

TIM16 (Timer DevNo 2), requested 1 MHz, supplies elapsed time through the
generic Timer API. The application accumulates hardware time differences,
not loop iterations, so SPI, ADC and display delays do not accumulate as
timing error. Unsigned differences handle the 32-bit millisecond timestamp
wrap as long as the application polls at least once per wrap (about 49.7
days). Stopwatch accuracy follows the board's clock.

OLED SPI3 and joystick ADC4 input 6 on PA3 reuse the working configuration.
The board file contains configuration only. No new ADC, display or timer
implementation is introduced.

Validation: StopwatchDemo and the original JoystickMenuDemo compile/link in
Debug and Release. `python3 tests/display/run_ssd1315.py` exercises the shared
source with scripted hardware timestamps and joystick voltages, the real
Timer conversion and SSD1315 driver, and a SPI OLED RAM model under
ASan/UBSan at -O0/-Os. Tests cover displayed digits, a delayed iteration,
pause/resume, lap retention, reset guards, menu navigation, time continuing
through ADC read failure, counter wrap and the upper display limit.
Stopwatch hardware validation remains pending.
