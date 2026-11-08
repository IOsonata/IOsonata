# PeiTempDemo - nRF52832 / BLYST-NANO-DK

Import `ioc` as an Eclipse Embedded CDT managed project.

The on-chip TEMP sensor requires no external device. The normal
TempnRF5x driver initializes and enables DATARDY IRQ processing.
After boot `g_PeiTempStatus=1` means the PPI connections are armed.

Press BUT1 (P0.02). Its GPIOTE hardware event executes TEMP START
through PPI. The TEMP DATARDY event triggers hardware STOP and
toggles the separate LED1 (P0.30) through PPI. The existing TEMP
IRQ still updates the last temperature sample; it never forwards an
event to a PEI task. Repeated BUT1 presses should initiate new
one-shot conversions.

This example has not been compiled or run on target hardware.
Before disabling the temperature driver, disable and disconnect
all PEI task destinations and unbind DATARDY from PEI.
