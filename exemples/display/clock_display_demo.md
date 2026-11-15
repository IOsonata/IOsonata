# ClockDisplayDemo

A 24-hour clock with large `HH:MM:SS` digits. The shared application is
`clock_display_demo.cpp`; `board.h` supplies the display class and geometry,
SPI pins and rate, timer device, button pin and initial time.

The application uses the existing SPI, Timer and Display interfaces. It renders
six seven-segment digits and two colons into a 512-byte, 128x32 monochrome
bitmap, then calls `BitBlt`. Digits are 16x31 pixels. No large font is added.
The display is redrawn only when the displayed second or a time setting changes.
Drawing stays in the foreground.

## Controls

- Short button press: advance one minute on release.
- Hold the button: advance one hour after one second, then once per second.
- A long press does not also advance a minute on release.

The button is active low, sampled at 20 Hz, with two samples required for a
state change. The displayed time is calculated from the timer count, so a
late redraw does not lose elapsed seconds. Hours wrap at 24.

Set `CLOCK_START_HOUR`, `CLOCK_START_MINUTE` and `CLOCK_START_SECOND` in the
board file. The default is 12:00:00. This demo does not obtain the current time
from a host or network and does not save settings across reset or power loss.

## SAM D11 with OLED1 Xplained Pro

Import `ARM/Microchip/SAMD/SAMD11/exemples/ClockDisplayDemo/ioc` as an existing
Eclipse project. Build `IOsonata_SAMD11D14AM` and `ClockDisplayDemo` with the
same Debug or Release configuration. Both use `-Os` and LTO.

Connect OLED1 to EXT1. The SPI and control pins match LCDConsoleDisplayDemo.
SW0 on PA14 adjusts the time. SPI uses polling at 1 MHz.
The output is the display in both configurations; no UART or logger is used.

The library contains the SAMD timer port and `ARM/src/timer_systick.cpp`.
`CLOCK_TIMER_DEVNO=4` selects SysTick through the normal `TimerInit` API.
The demo project contains only application sources. Devices 0 through 3 keep
their existing RTC, TC1, TC2 and TCC0 numbering. SysTick has one trigger and
cannot share its interrupt with an RTOS tick or another SysTick handler.

**The D11 build uses the core-clock SysTick, not the low-frequency RTC.** It
runs at the configured core clock, which must remain unchanged. It uses idle
waits between updates, not Standby. Clock accuracy follows the core oscillator;
this is not a battery-backed calendar clock.

A first build using the SAMD RTC timer needed 19,120 flash bytes, exceeding
16 KB by 2,736 bytes. The SysTick version fits with the existing
display virtual methods. The display still retains its ordinary
text formatter and system font through its virtual table.

Arm GCC 15.2.1, full D11 library archive:

| Configuration | text | data | bss | Flash | Static RAM |
| --- | ---: | ---: | ---: | ---: | ---: |
| Debug | 16032 | 176 | 2716 | 16208 | 2892 |
| Release | 16032 | 176 | 2716 | 16208 | 2892 |

Flash margin is 176 bytes. RAM beyond static storage is 1204 bytes; complete
stack usage has not been measured. Board operation is not yet validated.

## Checks

```sh
ASAN_OPTIONS=detect_leaks=0 python3 tests/display/run_clock.py
python3 tests/display/build_samd11_clock.py
```

The build check uses the managed library archives by default. To use existing
archives elsewhere, pass `--library-dir` with a directory containing `Debug`
and `Release` subdirectories. It links against the normal 16 KB linker script
and writes ELF, map and HEX files under the temporary `samd-clock-validation`
directory. It checks that UART, SysLog, hardware RTC/TC/TCC and unused SPI modes
are absent.

Host tests cover midnight rollover, delayed updates, button bounce, short and
long presses, and the bitmap bounds for every second in a day. They do not
validate oscillator accuracy, physical button wiring or SPI transfers.

For another board, provide its pin map, monochrome display class and timer
selection in `board.h`, and link the appropriate IOsonata library. The clock
face requires at least 128x32 pixels and is centered on larger panels. Select a timer device supported by that board's library; do not link a timer
implementation into the application project.
