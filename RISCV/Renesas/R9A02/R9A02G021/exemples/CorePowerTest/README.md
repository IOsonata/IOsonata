# R9A02G021 PowerOffDemo

Eclipse managed project using `exemples/misc/power_off_demo.cpp`. Build the
matching IOsonata_R9A02G021 library configuration first. On FPB-R9A02G021,
UART0 uses P303 RX and P302 TX at 1000000 baud through J-Link OB, with the
standard board links fitted. SW1 on P108/IRQ4 wakes on a falling edge.

The demo uses the generic resume path. It stops console traffic, sleeps,
resumes and checks 128 static plus 128 stack bytes. SystemSleep selects
Software Standby with an enabled wake interrupt, DTC stopped and Snooze off.
Otherwise it uses ordinary Sleep. Pending interrupt work returns immediately.
The printed "Standby" label does not measure the actual hardware state.

The generic path preserves pending ICU events and temporarily disables
non-wake CLIC interrupts according to Renesas TN-RIS-A0001B/E procedure B.
It does not start or feed either watchdog. R9A02G021 has no reset-on-wake
power-off mode, so SystemPowerOff and reset-on-wake retention are not supplied.

The target-specific R9a02LowPowerEnter API still serves the existing Snooze
demos. Its DTC/Snooze transition needs a separate review of the same erratum;
SystemSleep deliberately does not select that path.

Physical wake, oscillator restart and current measurements remain pending.
D11 remains the current hardware test target.
