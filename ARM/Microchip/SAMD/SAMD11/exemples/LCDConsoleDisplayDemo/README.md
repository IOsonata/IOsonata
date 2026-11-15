# SAM D11 OLED1 display console

Both selections fit the SAMD11D14AM's 16 KB flash with the LTO settings below.
Hardware validation is pending for both selections. This project is for
SAM D11 Xplained Pro with OLED1 Xplained Pro connected to EXT1.

The existing `exemples/display/console_display_demo.cpp` selects the system
5x7 font, shows its character sample, then sends a counter every two seconds
through `SysLog` and `DisplayIntrf` when display output is selected. The display
scrolls the text. UART output skips the display initialization and font sample
and sends the same counter to the EDBG virtual COM port at 1,000,000 baud, 8N1. Watchdog startup policy is unchanged.

`DISPLAY_SYSLOG_ENABLE` selects this small-font log demonstration in `board.h`.
Other console projects retain their existing font demonstration.

Select exactly one output in `src/board.h` or with a compiler definition:

- `CONSOLE_LOG_OUTPUT=CONSOLE_LOG_UART` (1): UART only.
- `CONSOLE_LOG_OUTPUT=CONSOLE_LOG_DISPLAY` (2): display only, the default.

The D11 board rejects 0, 3 (both), and other values. The unselected output's
objects are not compiled into the application. Link checks verify that its
drivers are absent from a successful image. This is a D11 application choice;
it is not a generic SysLog restriction. Larger MCUs can initialize separate
SysLog instances with their own record storage, one for UART and one for the
display, and write to both. The global logger itself still selects one sink.

UART uses interrupt-driven transmission without DMA. The foreground retries
queued records every 10 ms so a partial write to the small TX FIFO finishes
even when no new record follows.

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
  the target's real linker script, a separate library archive, GCC 15.2.1,
  `-Os` and LTO. Checks that unused SPI transfer modes are absent.
- `python3 tests/samd/run_spi.py`: host tests of all SPI master modes,
  completion, read-command synchronization, reinitialization, failed channel
  acquisition, reset timeout, power-off cleanup and slave arming.
- `python3 tests/samd/build_spi.py`: after the display build, links a C program
  with runtime mode selection against the archive, with and without LTO.
- Full managed D11 library: all 238 sources compile in Debug and Release;
  both console selections link against each complete archive at the sizes below.
- No SAMD11 OLED hardware result yet.

Hardware references:
- [OLED1 extension header](https://onlinedocs.microchip.com/oxy/GUID-26DC66D4-2EF7-4C54-8D2D-2A29BED57CAB-en-US-2/GUID-4AEF0FF0-9E97-4FCB-8565-668D17B30006.html)
- [SAM D11 EXT1](https://onlinedocs.microchip.com/oxy/GUID-B3EE4AA3-6D53-47F3-8031-6CDF20511C2D-en-US-1/GUID-EA8FB42A-40D4-431B-AA13-C62DC61C293B.html)
- [SSD1306 datasheet](https://www.olimex.com/Products/Modules/LCD/MOD-OLED-128x64/resources/SSD1306.pdf)

## Syslog resource changes

Earlier direct-object measurements, GCC 15.2.1, Cortex-M0+, `-Os`,
real SAMD11 linker script, no LTO:

| Build | Before flash | After flash | Before static RAM | After static RAM |
| --- | ---: | ---: | ---: | ---: |
| Status-only probe, including startup | 7872 | 5394 | 564 | 468 |

Flash counts text plus initialized data. Static RAM counts data plus BSS;
it does not include the stack. These are probe image sizes, not logger-only
sizes. The current script uses an archive instead of linking every object
directly. With `--syslog-only --no-lto`, the current probe uses 5062 bytes
flash and 296 bytes static RAM. The script also checks that the probe does not link printf or timestamp code.

The OLED console saves 764 flash bytes. Its queue changes from four 64-byte
records to two 32-byte records; together with four extra bytes for timestamp
selection, this saves 188 bytes of static RAM. The later SPI/LTO change below makes it fit flash.

The compiler-reported local stack frame of `SysLogVPrintf` falls from 48 to
32 bytes. This is not a measurement of the complete call chain's stack use.

`SysLogInit` remains available with the same arguments and timestamp behavior.
The new `SysLogInitNoTimer` leaves out timestamp support. C++ `Init` selects
that path when no timer is provided. The record format, severity filter, queue
policy and partial-write behavior are preserved. Rebuild the library and
application together: the internal `SysLog_t` layout has gained a function
pointer. Host tests cover full 64-bit status timestamps and truncation at every
record length from 2 through 64 bytes.

## Output-selection builds

Run `python3 tests/display/build_samd11_console.py --output uart` or
`--output display`. Both use the same Eclipse-managed project and board file.
The script checks the application object for unselected output references and
checks the final ELF when linking succeeds.

GCC 15.2.1, `-Os`, library and application rebuilt together. Debug and Release
have the same sizes:

| Output | Text | Data | BSS | Flash (text + data) | Static RAM (data + BSS) |
| --- | ---: | ---: | ---: | ---: | ---: |
| Display | 14720 | 204 | 1956 | 14924 | 2160 |
| UART | 14068 | 576 | 628 | 14644 | 1204 |

Display has 1460 bytes flash free and 1936 bytes RAM beyond static storage.
The remaining RAM must cover the stack; this is not a measured stack margin.

## SPI and LTO

The SAMD SPI driver selects a private constant operation table at initialization.
Common transfer and lifecycle functions call the selected operations, so a
constant polling configuration does not retain interrupt transfers, slave
handling or DMA transfers. Runtime configurations still support every mode;
no feature-disable macro or public API change is needed. The table pointer
adds four bytes per SERCOM instance. Removing unused DMA/interrupt storage
more than offsets that in the polling console.

Using the same archive build and LTO settings, the previous SPI driver required
16476 bytes flash (92 bytes too large); the updated driver requires 14924,
saving 1552 bytes. Earlier direct-object link measurements are not identical
to these archive builds.

The D11 library compiles with `-flto -ffat-lto-objects`; the console compiles
and links with `-flto`. The final link also passes `-ffunction-sections` and
`-fdata-sections`, alongside the existing `--gc-sections`, so code generated
by LTO can be discarded individually. Fat objects retain normal machine code
for applications explicitly linking with `-fno-lto`.

`src/iatomic.c` has a file-specific `-fno-lto` setting in the library project.
Cortex-M0+ atomic helper references appear during code generation after LTO;
keeping that archive member native lets the linker resolve those calls.
Clean and rebuild both the D11 library and the application after updating.

The archive still contributes the DMAC vector handler; the display ELF omits
DMA channel allocation and transfer functions, slave handling and the shared
SPI interrupt transmit buffer. No other board was built or hardware-tested
for this change.
