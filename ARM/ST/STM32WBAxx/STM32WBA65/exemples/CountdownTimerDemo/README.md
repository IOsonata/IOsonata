# STM32WBA65I-DK1 countdown timer

Import `ioc` as an existing Eclipse/IOcomposer project. Rebuild the matching
Debug or Release IOsonata_STM32WBA65 library before building the application.
The shared source is `exemples/display/countdown_timer_demo.cpp`; `src/board.h`
contains the board configuration. No new driver or public API is required.

The OLED starts with a paused one-minute countdown. A bar shows the fraction
of the preset duration remaining. The clock displays minutes, seconds and
tenths, rounded up so it does not show zero before expiry.

| Joystick | Action |
| --- | --- |
| Select | Start or pause; after DONE, reload the preset and start again |
| Left | While paused, reset to the preset and clear DONE |
| Right | While paused, add 10 seconds to the preset |
| Up / Down | While paused, add / subtract one minute from the preset |

Changing the preset also reloads the remaining time. Duration is limited to
10 seconds through 99 minutes 59 seconds. Adjustment and reset are ignored
while running. At expiry the timer stops at zero, displays DONE, and turns
on the green LED. Resetting, changing the preset, or starting again clears
that indication. There is no sound output.

The ADC, joystick ladder, three-sample debounce and release requirement follow
JoystickMenuDemo. A key held at startup or through an ADC failure must be
released before it can activate anything. A read failure appears on the OLED;
timekeeping and the expiry LED continue.

The project uses the existing AdcSTM32WBA polling driver, generic Timer API
on TIM16 (Timer DevNo 2, requested 1 MHz), and the SSD1315 driver on SPI3.
OLED wiring is SCK PA0, MOSI PB8, CS PE1, DC PE0 and RESET PE3; joystick is
PA3 / ADC4 input 6. Rendering runs in the main loop about ten times per second.
Elapsed hardware timestamps include display/ADC work and handle the 32-bit
millisecond timestamp wrapping, provided the main loop runs at least once per
wrap. Time and the preset are not retained through reset.

Debug and Release compile/link checks pass. The display simulation checks
OLED clock digits, preset limits, debounce, held keys, pause/resume, reset,
expiry and LED state, ADC failure and timestamp wrap at -O0/-Os under ASan/UBSan.
On October 19, 2026, the maintainer confirmed that CountdownTimerDemo
works on the STM32WBA65I-DK1.
