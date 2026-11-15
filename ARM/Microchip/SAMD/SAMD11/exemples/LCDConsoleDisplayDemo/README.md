# SAM D11 OLED1 display console

Work in progress: the current image exceeds the SAMD11D14AM's 16 KB flash.
Do not flash an ELF produced with linker overflow checks bypassed. Hardware
validation is pending. This project is only for SAM D11 Xplained Pro with
OLED1 Xplained Pro connected to EXT1.

The existing `exemples/display/console_display_demo.cpp` selects the system
5x7 font, shows its character sample, then sends a counter every two seconds
through `SysLog` and `DisplayIntrf`. The display scrolls the text. UART and
semihosting are not used for this output. Watchdog startup policy is unchanged.

`DISPLAY_SYSLOG_ENABLE` selects this small-font log demonstration in `board.h`.
Other console projects retain their existing font demonstration.

| OLED1 signal | EXT1 pin | SAMD11 pin |
| --- | --- | --- |
| D/C | 5 | PA04 GPIO |
| Reset | 10 | PA15 GPIO |
| CS | 15 | PA08 GPIO |
| MOSI | 16 | PA06 function C, SERCOM0 pad 0 |
| SCK | 18 | PA07 function C, SERCOM0 pad 1 |

PA09 function D supplies the SPI driver's required MISO pad 3. OLED1 does not
connect that signal. SPI uses mode 0, 8-bit MSB-first transfers at 1 MHz,
without interrupts or DMA. Buttons and LEDs are not exercised by this demo.

`LcdSSD1306` reuses the SSD1315 implementation's compatible page addressing,
initialization commands and monochrome renderer. It retains that driver's
fixed 1024-byte framebuffer, even for this 128x32 panel.

## Display logging

Initialize a display as in the existing console example, then:

```cpp
// Keep these objects and the log memory alive while logging.
DisplayIntrf output;
output.Init(&display);
SysLogGetInstance()->Init(logCfg, &output);
SysLogGetInstance()->Printf("Count %lu\r\n", count);
```

The application supplies `logCfg` and its record memory as usual. The adapter
accepts any initialized `Display`; it does not own the panel or change its
power state. Disable the adapter before suspending the panel, enable it after
restoring the panel, then call `Flush()` to send queued records.

Log from the foreground/task owning the display. With a sink attached SysLog
sends immediately in the caller's context, so do not use this path in an ISR.
Display's void printing API cannot report physical transfer failures: a returned
byte count means text was passed to that API, not verified on the panel.

## Validation

- `python3 tests/display/run_display_intrf.py`: host tests of syslog output and
  this board's console initialization, with address/undefined-behavior checks.
  Environments running under ptrace need `ASAN_OPTIONS=detect_leaks=0`.
- `python3 tests/display/build_samd11_console.py`: Arm GCC source build using
  the target's real linker script. GCC 15.2.1, `-Os`, currently fails the flash
  limit by 2628 bytes in both Debug and Release after the syslog changes.
- No SAMD11 OLED hardware result yet. Flash size must be resolved first.

Hardware references:
- [OLED1 extension header](https://onlinedocs.microchip.com/oxy/GUID-26DC66D4-2EF7-4C54-8D2D-2A29BED57CAB-en-US-2/GUID-4AEF0FF0-9E97-4FCB-8565-668D17B30006.html)
- [SAM D11 EXT1](https://onlinedocs.microchip.com/oxy/GUID-B3EE4AA3-6D53-47F3-8031-6CDF20511C2D-en-US-1/GUID-EA8FB42A-40D4-431B-AA13-C62DC61C293B.html)
- [SSD1306 datasheet](https://www.olimex.com/Products/Modules/LCD/MOD-OLED-128x64/resources/SSD1306.pdf)

## Syslog resource changes

GCC 15.2.1, Cortex-M0+, `-Os`, real SAMD11 linker script, no LTO:

| Build | Before flash | After flash | Before static RAM | After static RAM |
| --- | ---: | ---: | ---: | ---: |
| Status-only probe, including startup | 7872 | 5394 | 564 | 468 |

Flash counts text plus initialized data. Static RAM counts data plus BSS;
it does not include the stack. These are probe image sizes, not logger-only
sizes. Run `python3 tests/display/build_samd11_console.py --syslog-only`.
The script also checks that the probe does not link printf or timestamp code.

The OLED console saves 764 flash bytes. Its queue changes from four 64-byte
records to two 32-byte records; together with four extra bytes for timestamp
selection, this saves 188 bytes of static RAM. It still exceeds flash.

The compiler-reported local stack frame of `SysLogVPrintf` falls from 48 to
32 bytes. This is not a measurement of the complete call chain's stack use.

`SysLogInit` remains available with the same arguments and timestamp behavior.
The new `SysLogInitNoTimer` leaves out timestamp support. C++ `Init` selects
that path when no timer is provided. The record format, severity filter, queue
policy and partial-write behavior are preserved. Rebuild the library and
application together: the internal `SysLog_t` layout has gained a function
pointer. Host tests cover full 64-bit status timestamps and truncation at every
record length from 2 through 64 bytes.
