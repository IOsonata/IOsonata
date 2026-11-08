# PeiQdecDemo - nRF52832 / BLYST-NANO-DK

Import `ioc` as an Eclipse Embedded CDT managed project.

The existing QDEC driver samples BUT1 (P0.02) and BUT2 (P0.13)
as its A/B inputs. A genuine quadrature encoder is **not** required to
test the hardware START/STOP tasks, but buttons do not validate
direction or position accuracy.

After boot `g_PeiQdecStatus=1` means configuration and routes are armed.
Press BUT1 to START QDEC over PPI; the SAMPLERDY event should toggle
the separate LED1 (P0.30) at approximately 131-ms intervals.
Press BUT2 to STOP QDEC; STOPPED should toggle RGB blue (P0.27).
No CPU ISR forwards an event to PPI. The QDEC IRQ is still
responsible for READCLRACC and accumulated position/error updates.

This source has not been built or tested on the board. Disconnect
all PEI routes before stopping/reinitializing QDEC. The sample period
and GPIO timing are expectations, not measured observations.
