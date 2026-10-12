# LPC11Uxx clock decoding

Run `python3 tests/lpc_clock/run.py` from the repository root.

The host test compiles the actual system_LPC11Uxx.c with vendor CMSIS
peripheral types, maps SYSCON memory, and checks SystemCoreClockUpdate.
It verifies that decoding does not write peripheral registers.

Coverage includes the 12 MHz IRC, PLL input bypass, both supported PLL
inputs, all multiplier and post-divider encodings, the default 48 MHz PLL
configuration, every nonzero AHB divider, representative watchdog settings,
undefined watchdog selection, reserved PLL input selections and divider zero.
PLL field decoding tests include settings outside the permitted CPU range;
they do not authorize using those settings on hardware.

The old source fails the reset IRC regression by reporting 6 MHz. The fixed
source passes O0/O2 host runs with UBSan and Cortex-M0 compiler-to-assembly
checks. Hardware clock measurements and full firmware linking remain pending.

This change corrects frequency reporting only. Oscillator startup, PLL
programming, board oscillator selection and peripheral-clock API completion
remain separate review items. The system oscillator assumption is still 12 MHz.
Watchdog frequencies are nominal, not measurements.

Reference: NXP UM10462 revision 4.1, chapter 3, Tables 9, 14, 16, 20 and 22.
https://www.keil.com/dd/docs/datashts/nxp/lpc11uxx/um10462.pdf
