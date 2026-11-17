# SAM4L8 Xplained Pro PowerOffDemo

Connect OLED1 Xplained Pro to **EXT2**. The demo uses OLED1 **BUTTON1**,
connected through EXT2 pin 9 to **PC06 / EIC8**, as an active-low Backup wake
input. The on-board SW0 is not the wake button for this configuration.

Build the SAM4LCxC library and PowerOffDemo in the same configuration.
Flash the application and run without an active debugger session. UART output
uses USART1 at 1000000 baud on the EDBG virtual COM port.

After `Power off, press the button to wake up`, allow one second for the UART
to shut down, then press and release OLED1 BUTTON1. A successful wake restarts
the application and reports `Reset reason 0x100: wake up by a pin`. Repeat
several times. The start count should increase if backup register retention
works; record the output to verify both wake and retention.

This demo does not initialize the OLED display. Its output remains on UART.
The first hardware wake reported pin wake and advanced the retained count
from 1 to 2. Re-arming then failed because the EIC line survived Backup while
its SRAM ownership flag was cleared. The driver now uses the retained Backup
pin routing to recognize that line. Repeated wake validation is pending.

Pin assignments: Microchip SAM4L8 Xplained Pro User Guide, table 4-2, and
OLED1 Xplained Pro User Guide, extension header table.
