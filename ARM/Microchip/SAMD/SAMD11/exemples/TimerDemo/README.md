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

The demo builds fixed decimal fields and sends them with `SysLogStatus`,
without linking the general printf formatter. Timing lines have the standard
runtime status prefix, for example:

```text
R:000:0003 Count = 100 ms, T0 = 100000 us
```

## Validation

With Arm GCC 15.2.1, `-Os`, and LTO, all four selections fit with the full demo:

| Timer | Debug flash | Release flash | Release flash free |
| --- | ---: | ---: | ---: |
| 0 (RTC) | 11612 | 14084 | 2300 |
| 1 (TC1) | 12564 | 15032 | 1352 |
| 2 (TC2) | 12608 | 15072 | 1312 |
| 3 (TCC0) | 12868 | 15340 | 1044 |

Flash includes text and initialized data. Release static RAM is 3112 bytes
for RTC and 3120 bytes for TC/TCC; stack usage has not been measured.

Before the fixed-field logging change, the maintainer validated RTC trigger 0
at 100 ms with Debug semihosting and Release UART, and all four TCC0 triggers
with Debug semihosting. TC1/TC2 hardware tests and the revised logging output
remain to be validated. Timer and UART driver code is unchanged by this
logging change.

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
