# SAM4LCxC ClockDemo

Import `ioc` as an Eclipse managed project. Clean rebuild the SAM4LCxC library
and demo in the same configuration. Connect OLED1 to EXT2. Console output uses
USART1 on the EDBG virtual COM port at 1,000,000 baud in Debug and Release.

The demo runs three RC4/RC12 selection cycles. Each source supplies the DFLL;
the printed CPU rate is its measured output after division, not the RC input
rate. PBA divisors 1/2/4/1 are checked at each source. UART is drained and
powered off before changing clocks, then reinitialized at the new bus rate.
The TC measurement timer is stopped during changes and retimed afterward.
Both msDelay and nsDelay are checked over 250 ms with a 10 ms timer interrupt.
These checks do not independently measure absolute oscillator accuracy.

For each of the six sleep prompts:

1. Press and release OLED1 BUTTON1 (PC06 / EIC8).
2. The demo then sleeps again and wakes automatically after one second using
   AST alarm 0. No button press is needed for this second sleep.
3. Check `Timer wake interrupt: PASS`, clock descriptors, both delays and IRQs.
4. Send uppercase U at the UART RX prompt.

Both sleeps use RETENTION with PBA divided by four. AST is acquired at startup
because its registers survive non-power-on resets. It is stopped before clock
changes. The alarm handler clears its asynchronous wake request; subsequent
cycles exercise re-arming. No watchdog is enabled. The run should end with
`ClockDemo complete: 0 failures`.

The maintainer's November 17 result confirmed the six EIC RETENTION wakes,
RC4/RC12 selection, PBA divisors, timer IRQs, UART RX and clock restoration with
zero failures. The reported 250 ms delays were approximately 251194..251600 us.
An earlier run separately confirmed three GPIO-only SLEEP0 wakes. The subsequent AST run ended with zero failures and reported automatic
wake intervals of 1000000 us.

WAIT is selected by SystemSleep when PM.AWEN contains a configured asynchronous
peripheral wake source. This permits SleepWalking. This demo does not configure
such a peripheral; AST and EIC alone use RETENTION. Current consumption and
external oscillator configurations require separate hardware measurements.
