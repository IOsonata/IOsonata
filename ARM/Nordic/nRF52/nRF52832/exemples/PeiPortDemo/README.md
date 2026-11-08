# PeiPortDemo (nRF52832)

Import `ioc` into IOcomposer/Eclipse as an Eclipse Embedded CDT managed project.

This example verifies two independent event-to-task routes:

- **BUT1 (P0.02)**: GPIO SENSE low -> shared GPIOTE.PORT -> PPI -> LED1 task.
- **BUT2 (P0.13)**: dedicated GPIOTE.IN[n] -> PPI -> LED2 task.

Press each button separately. Each should toggle only its corresponding LED.
`g_PeiPortDemoStatus == 1` means setup succeeded; negative values locate
failed setup steps. It is not a hardware PASS by itself.

The PORT event combines all SENSE-enabled pins, so it does **not** identify
the source pin or guarantee a per-pin edge. This example only configures
BUT1 SENSE, leaving BUT2 as a dedicated IN[n] source. A level held active
or a latch-based DETECT configuration may prevent another PORT transition.
Do not use PORT as a silent fallback for `IOPinEnableEvent()`.

During setup, the example unbinds and rebinds the PORT source to the
same logical event while keeping the LED task relationship. This tests
logical source reuse without altering GPIO configuration or claiming
another GPIOTE channel. PORT SENSE and any PORT interrupt are owned by
the caller, not by the PEI binding.

The example uses no semihosting or UART and links the standalone
`pei_nrf52.cpp` PPI backend directly, as the original PeiBlinky does.
It does not enable a SoftDevice.

**Hardware checks pending:** both buttons drive only their own LED,
multiple independent presses continue to work, and PORT rebind/reuse
has not changed the dedicated IN[n] event route.
