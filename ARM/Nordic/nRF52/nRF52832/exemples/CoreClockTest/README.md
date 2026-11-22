# nRF52832 ClockDemo

Import `ioc` as an existing Eclipse managed project and build the matching
IOsonata library in the same Debug or Release configuration. Console pins and
oscillator selections are in `src/board.h`, copied from this target's
PowerOffDemo project. Default console speed is 1,000,000 baud unless board.h
specifies another rate. No debugger is required for UART logging.

ClockDemo exercises the generic core-clock selection, peripheral-clock
Get/Set, low-frequency source selection, shared HF/LF reference counting,
and Timer APIs. Board definitions supply oscillator test inputs and physical
UART configuration; they do not select an MCU-specific execution path.
ClockDemo reports missing shared-clock API implementations as failures.
HfxoTest and HfxoTestBle additionally inspect physical oscillator behavior
and BLE ownership on supported hardware.

UART transmission stays under the UART driver's control while clock
selections run. The application never reads the driver's FIFO or calls RX
to advance TX. Every timer reported by TimerGetLowFreqDevCount and
TimerGetHighFreqDevCount is exercised with a 10 ms trigger over 250 ms.
When at least two timer devices exist, the demo checks that disabling
one does not stop the other.
The returned timer rate may differ from 1 MHz. Delay and interrupt checks use
the actual rate/period. They do not independently measure oscillator accuracy.
No watchdog is enabled. No sleep is requested by this configuration; use
PowerOffDemo for target wake validation. Reset restores startup clocks.

The demo checks peripheral clock indices 0 through 31 and verifies
invalid-index handling. The generic HAL does not currently expose a count
of peripheral clock domains or oscillator-source capabilities. Consequently,
this test cannot yet establish exhaustive coverage of every source or
domain on every MCU. An unavailable operation is reported explicitly,
not treated as PASS. Target builds, board wiring and oscillator operation
still require hardware validation.
