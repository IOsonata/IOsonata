# RA4M1 PowerOffDemo

This Eclipse managed project uses the generic demo's resume path. Build the
matching IOsonata_RA4M1 library configuration first. Both configurations print
at 1000000 baud on SCI0, P100 RX and P101 TX. P000/IRQ6 is the falling-edge
wake input with an internal pull-up; connect a button to ground. These are MCU
pin numbers, not a claim about an onboard button or connector.

The demo stops UART traffic, sleeps, resumes, and checks 128 static plus 128
stack bytes. It uses Software Standby with the configured external IRQ wake
when DMA/DTC are disabled and no conflicting Snooze or oscillator-stop policy
is enabled. Other configurations use ordinary Sleep. The generic demo's
"Standby" log label does not measure which mode the hardware entered.

RA4M1 has no Deep Software Standby reset-on-wake state. This project calls
SystemSleep, not SystemPowerOff. SRAM remains valid across resume; no reset
counter or reset-on-wake retention test is used. The watchdog and independent
watchdog remain application-controlled.

Register checks and Debug/Release links passed. Repeated physical wake,
oscillator restart, full SRAM retention and current measurements are pending.
D11 remains the current hardware test board.
