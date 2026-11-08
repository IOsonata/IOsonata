# PeiRngDemo - nRF52832 / BLYST-NANO-DK

Import `ioc` as an Eclipse Embedded CDT managed project.
Run a **bare-metal** build, not NRFXLIB_SDC or an active Nordic
SoftDevice configuration.

The IOsonata entropy driver owns the RNG peripheral.
`RngNrfPeiPrepare()` acquires its DeviceIntrf busy lock, enables
bias correction, clears old VALRDY and leaves RNG stopped.
On success `g_PeiRngStatus=1` means the PPI routes are armed,
not that a random value has been generated.

Press BUT1 (P0.02) once. GPIOTE/PPI sends RNG TASKS_START;
the next hardware VALRDY event sends RNG TASKS_STOP and
toggles separate LED1 (P0.30). No CPU event-forwarding or
RNG.VALUE read is involved.

Do not enable a SoftDevice or SoftDevice Controller while this
route owns RNG. In a normal application, first disconnect all
PEI task and event routes, then `RngNrfPeiRelease()` to stop
hardware and release the entropy driver's lock. RNG PPI is
not a mechanism for delivering cryptographically secure bytes.

This example has not been compiled or run on target hardware.
