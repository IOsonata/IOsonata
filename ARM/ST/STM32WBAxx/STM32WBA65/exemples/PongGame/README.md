# STM32WBA65I-DK1 Pong

Import `ioc` as an existing Eclipse/IOcomposer project and build the matching
Debug or Release IOsonata_STM32WBA65 library first. The application links the
shared `exemples/display/pong_game.cpp`; `src/board.h` contains only board
mapping and configuration.

The game uses the onboard 128x64 SSD1315 OLED, the analog joystick on PA3 /
ADC4 input 6, and TIM16 through the generic Timer API. No external wiring is
needed.

| Joystick | Action |
| --- | --- |
| Up / Down | Move the left paddle; the direction may be held |
| Select | Start, pause/resume, or start a new match after game over |
| Left / Right | No game action |

The right paddle is CPU controlled. The ball accelerates horizontally after
long rallies and changes vertical angle according to where it hits a paddle.
The first side to 9 points wins. After every point the next serve goes toward
the player that lost the point.

The game loop is timestamped from TIM16 at 25 frames/s. A delayed iteration
does not accumulate display-transfer time into the timer, and pause leaves the
ball stationary. Joystick transitions use three ADC samples; a SELECT held at
startup cannot start the game until it is released. If an ADC read fails, the
game continues while player movement stops and the display marks the input
failure.

The OLED uses the same validated SPI3 configuration as the other WBA display
examples: SCK PA0, MOSI PB8, CS PE1, D/C PE0 and reset PE3, mode 0, polling,
requested 4 MHz. The application uses only the generic SPI, ADC, Timer and
display APIs.

Host-model coverage is in `tests/display/pong_game_test.cpp` and is run by
`python3 tests/display/run_ssd1315.py` at -O0 and -Os with ASan/UBSan. It
checks startup gating, held paddle movement, wall and paddle collisions,
pause/resume, ADC failure, scoring, game-over and restart. Physical gameplay
on STM32WBA65I-DK1 remains to be validated.
