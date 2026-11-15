# LPC1769 examples

## PowerOffDemo

Import `PowerOffDemo/ioc` and build the LPC1769 library first with the same
configuration. The project links the generic power demo. UART0 uses P0.2 TX
and P0.3 RX at 1000000 baud, with UART DMA disabled for this driver.

P2.10 is sampled before entry and must be released/high. It cannot wake Deep
Power-down; the demo reports this and asks for the hardware RESET pin. After
three seconds it stops UART and enters Deep Power-down. Press RESET to reboot.
The next report should identify reset-pin wake and increment the GPREG0 count.
Backup registers depend on VBAT, so removing the main supply alone need not
clear them. The project builds in Debug and Release; hardware validation is
pending. RTC wake is supported by the library but not armed by this demo.
