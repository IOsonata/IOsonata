# RE01 port regression checks

Run the Linux host checks from the repository root:

```sh
make -C tests/re01 test
```

The test compiles the real RE01 port and generated register header. A small
CMSIS shim replaces CPU instructions and NVIC operations; anonymous memory
backs the peripheral addresses. It checks clock reporting and divider bounds,
ICU ownership and reserved events, IRQ8/9 routing, GPIO validation and failed
allocation cleanup, TMR resume, AGT reset/divider limits, comparator allocation
and reuse, UART polling and interrupt paths, SCI9 module control, repeated
initialization, RX drops, error clearing, and partial IRQ allocation cleanup.
The baud-rate check uses an independent exhaustive BRR/MDDR calculation.

Build and link Blinky, TimerDemo and UartPrbsTxTest for Cortex-M0+ with both
normal linker scripts, then inspect the resulting ELF sections and option bytes:

```sh
python3 tests/re01/build_examples.py
```

Use `--tool-prefix /path/to/bin/arm-none-eabi-` if the compiler is not on PATH.
`--package DBN`, `--package CFB` or `--package CFP` selects the package define.
The check requires vectors below 0x400, 64 erased option bytes at 0x400,
code/data load addresses at or above 0x440, and heap/stack limits inside RAM.
These are direct compiler/linker checks of the example sources, not an
IOcomposer or Eclipse project build. The script uses newlib-nano and nosys.

Validated with GCC 14.3.Rel1 for all three package defines. The affected RE01
sources compile without warnings. Existing shared `ARM/src/iatomic.c` builtin
declaration warnings and example printf/unused-variable warnings are still
reported by the example builds.

This test does not emulate FIFO side effects, oscillator stabilization, voltage
transitions, asynchronous AGT stop acknowledgement, interrupt timing, or flash
programming. Board validation is still required for normal/boost startup, PLL
fallback, SCI loopback and burst traffic, GPIO wake, timer rate/reset/resume,
and simultaneous AGT compare events. DFU flash geometry and I2C/SPI support
are outside this repair pass.
