# LPC546xx host regressions

Run from a Linux checkout with G++ and Python 3:

```
python3 tests/lpc546xx/run.py
```

`timer_test.cpp` compiles the production CTIMER driver
(`ARM/NXP/LPC546xx/src/timer_lpc546xx.cpp`) against the repository's
LPC54628 device header. The CTIMER, SYSCON and ASYNC_SYSCON registers and the
NVIC are a host model: the counter jumps from match to match, the match
flags of the enabled match interrupts are set and the IRQ handler is called
like the NVIC would. It checks the 64 bit count over counter wraps, trigger
periods longer than a counter cycle, single shot and continuous triggers, the
pending interrupt for a deadline passed while arming, prescaler selection,
and the configuration checks of TimerInit.

The model does not cover the prescaler counter, bus timing through the async
APB bridge or interrupt preemption. Hardware validation is still required.
