# PeiGpioteStress (nRF52832 / BLYST-NANO-DK)

Import the ioc Eclipse managed project. No printf or semihosting is used.

## Automatic boot tests

- 64 rounds allocate all eight GPIOTE event channels on eight distinct pins. A ninth event must fail.
- After releasing a middle channel, an IRQ claiming an already-owned pin must fail, while the ninth event must succeed.
- All events and PEI identities are released and the cycle repeats.
- 128 rounds route an event through PPI to an LED task, enable/disable, disconnect, and free both resources.
- Each of those 128 rounds also allocates and releases an ordinary GPIO interrupt.

During exhaustion, LED and UART pins are configured only as digital inputs.
They do not drive anything. UART is not used in this firmware.
LED outputs and button pull-ups are restored for the live test.

Inspect g_GpioteStress after startup:

| Field | Expected |
| --- | --- |
| Status | 2: automatic checks passed; live test armed |
| Error | 0 |
| ExhaustionRounds | 64 |
| ReusedChannels | 64 |
| RouteRounds | 128 |
| IrqRounds | 128 |
| IrqCount | increases when BUT1 is pressed |

Negative Error values locate the failure in the firmware source.

## Physical coexistence test

BUT1 triggers a conventional GPIOTE IRQ, increments IrqCount and toggles LED2 in software. BUT2 has a GPIOTE IN event routed through PPI to toggle LED1 in hardware. Press them independently and repeatedly. Only the associated LED should respond.

Status 2 verifies automatic resource tests but is not proof of physical PPI routing. Observe LED1 and the IRQ counter separately. Hardware validation is pending.

This runs bare-metal on the BLYST-NANO-DK. It does not test coexistence with a SoftDevice, SDC or a second GPIOTE owner outside the IOsonata GPIO driver.
