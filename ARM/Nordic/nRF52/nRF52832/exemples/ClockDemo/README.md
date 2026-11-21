# nRF52832 ClockDemo

Import `ioc` as an existing Eclipse managed project and build the matching
IOsonata library in the same Debug or Release configuration. Console pins and
oscillator selections are in `src/board.h`, copied from this target's
PowerOffDemo project. Default console speed is 1,000,000 baud unless board.h
specifies another rate. No debugger is required for UART logging.

ClockDemo exercises the generic core-clock selection, peripheral-clock
Get/Set, low-frequency source selection, and Timer APIs. Board definitions
supply only test inputs; the application does not select an MCU-specific
execution path. Shared HFXO reference counting is tested separately by
HfxoTest and HfxoTestBle.

UART transmission stays under the UART driver's control while clock
selections run. The application never reads the driver's FIFO or calls RX
to advance TX. The timer is initialized before the clock test cycles,
then tested after frequency changes using a 10 ms trigger over 250 ms.
The returned timer rate may differ from 1 MHz. Delay and interrupt checks use
the actual rate/period. They do not independently measure oscillator accuracy.
No watchdog is enabled. No sleep is requested by this configuration; use
PowerOffDemo for target wake validation. Reset restores startup clocks.

An omitted selector prints SKIP. A configured selector that fails prints FAIL;
unsupported selections are not silently counted as successful tests. Runtime
clock ownership, board wiring and hardware operation still require board
validation. See the cross-target power/clock demo matrix for build results.
