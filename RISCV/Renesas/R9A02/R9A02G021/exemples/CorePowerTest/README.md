# CorePowerTest on FPB-R9A02G021

Build the matching IOsonata_R9A02G021 library configuration. UART0 uses
P303 RX and P302 TX at 1 Mbaud through J-Link OB with the default board links.

The test exercises timer wake, then falling-edge wake on SW1 (P108/IRQ4).
The machine timer wakes ordinary Sleep. SW1 can wake Software Standby.
After each wake the test checks the CPU clock, peripheral clock, UART rate,
and static and stack RAM. The MCU has no reset-on-wake System OFF mode.
