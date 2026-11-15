# SAM D11 TimerDemo

Build the IOsonata_SAMD11D14AM library and TimerDemo with the same Debug or
Release configuration. Clean both projects after changing the library.
Debug logs through semihosting; Release logs through UART at 1 Mbaud.

`TIMER_DEMO_DEVNO` in `src/board.h` selects RTC (0), TC1 (1), TC2 (2), or
TCC0 (3). The default is RTC. The D11 board selects the example's C API path
with `TIMER_DEMO_USE_C_API=1`. Compile and link LTO expose the constant device
selection so unused timer families can be discarded. The C++ virtual API is
unchanged; its runtime initializer can retain all timer families. Runtime
device selection also retains all supported choices. Per-device static state
is still allocated for the supported timers.

The project retains the existing `_sbrk` startup hook explicitly because
newlib can introduce its reference after LTO.

## Validation

With Arm GCC 15.2.1, `-Os`, LTO, and the default RTC configuration:

| Configuration | text | data | bss | Flash (text + data) |
| --- | ---: | ---: | ---: | ---: |
| Debug | 13664 | 172 | 2184 | 13836 |
| Release | 16124 | 172 | 3040 | 16296 |

Release leaves 88 bytes of flash and 884 bytes of RAM beyond static data.
These are build results, not measured stack usage or board validation.
Other timer selections with the full logging demo have not been size-qualified.

From the repository root:

```sh
python3 tests/samd/run_timer.py
python3 tests/samd/run_uart.py
python3 tests/samd/build_timer.py
```

The link check uses the managed library archives in `lib/ioc/Debug` and
`lib/ioc/Release`; `--library-dir` accepts another directory containing those
two configurations. It links the real demo and small C programs for all four
timer devices, checks that unused timer families are absent, and checks that
runtime selection remains available with and without LTO. All links use the
normal 16 KB flash region.

Host tests exercise rollover, trigger scheduling and cancellation, callback
reset, routing cleanup during reinitialization, and all UART DMA/interrupt
combinations. Register operations are modeled; peripheral synchronization and
timing still require board testing.
