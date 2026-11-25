# CoreClockTest — generic MCU test

> **Release 0.14 migration:** This Eclipse managed project now compiles the shared
> `exemples/misc/core_clock_test.cpp` test. The legacy demo instructions below
> are historical reference only; they do **not** describe the current test flow.
> Check `src/board.h` for this board's UART, oscillator, timer and wake-pin setup.
> Use the generic test output and capability reporting to determine supported
> functionality; build and hardware validation remain target-specific.

## Historical demo notes (pre-migration)

# SAM4LCxC CoreClockTest

Import `ioc` as an Eclipse managed project. Clean rebuild the matching
SAM4LCxC library and CoreClockTest for the same Debug or Release configuration.
Console output uses USART1 on the EDBG virtual COM port at 1,000,000 baud.

CoreClockTest tests clock selection only. It cycles the RC4/RC12 references,
checks PBA divisors 1/2/4/1, verifies timer interrupts and measures
`msDelay` and `nsDelay` against timer time. The timer is stopped during
clock changes. UART is drained and reinitialized when the bus rate changes.
AST is acquired and stopped before the clock sweeps. The demo also checks
UART reception at the normal console rate. These checks do not independently
measure absolute oscillator accuracy.

PowerOffDemo now tests low-power operation, reset reasons, EIC or AST wake
and retention. PICOUART wake is an optional PowerOffDemo profile requiring
an external 3.3 V adapter instead of EDBG.

## Earlier hardware evidence

Before separating the demos, the maintainer's November 17, 2026 runs
confirmed six EIC RETENTION wakes, RC4/RC12 selection, PBA dividers,
timer IRQs, UART RX and clock restoration with zero failures. Reported
250 ms delays were approximately 251194..251600 us. An earlier run
separately confirmed three GPIO-only SLEEP0 wakes. An AST run reported
automatic 1000000 us wakes with zero failures. These were observations
from the former combined demo, not verification of the new PowerOffDemo
standby profiles.

Current consumption and external oscillator accuracy remain unmeasured.
