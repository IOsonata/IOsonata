# PeiUartDemo — nRF52832 / BLYST-NANO-DK

Import `ioc` as an Eclipse Embedded CDT managed project.

No receiving UART device is required. UARTE0 transmits the fixed
eight-byte pattern at **1 Mbaud** on BLYST-NANO-DK TX (P0.07).
UARTE0 RX (P0.08) remains configured by the normal UART driver,
but the PEI test only controls the transmitter.

## Expected behavior

1. `g_PeiUartStatus=1` after boot means the DMA block and PPI routes
   are armed; it does **not** prove hardware transmission.
2. Press BUT1 (P0.02) once. Its GPIOTE event triggers UARTE0
   `STARTTX` through PPI, not software.
3. `ENDTX` toggles the **separate LED1** (P0.30) and triggers UARTE0
   `STOPTX` through two independent PPI connections.
4. `TXSTOPPED` toggles **RGB blue** (P0.27) through another PPI route.

To verify transmitted data and timing, connect a logic analyzer
or the board's existing USB-UART bridge to TX P0.07 at 1 Mbaud.
The LED indicators prove event-to-task routing when observed,
not received-byte integrity.

This is one prepared TX transaction. Do not rely on button presses
restarting TX without first explicitly releasing and preparing
another block. For cleanup: disable and disconnect TX task routes,
unbind UART event sources, then `UARTReleasePeiTx()` so the
driver's private TX buffer becomes available for normal UART Tx.
Do not disable, reinitialize or reconfigure UART before removing
the PEI connections. The driver still owns RX DMA and its FIFO;
PEI does not route STARTRX, STOPRX or FLUSHRX.

Hardware execution, per-device CMSIS compilation and cross-variant
UART/UARTE validation remain pending.
