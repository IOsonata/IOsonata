# SAM D11 ClockDisplayDemo

Standalone Eclipse managed project for SAM D11 Xplained Pro with OLED1 on EXT1.
The generic application and instructions are in
[clock_display_demo.md](../../../../../../exemples/display/clock_display_demo.md).

The IOsonata library provides SysTick as timer device 4. The demo selects it
through TimerInit to fit in 16 KB; no timer driver source is in this project.
It does not use the hardware RTC. `src/board.h` sets the pin map and initial
time. Short SW0 presses advance minutes; holding SW0 advances hours.
