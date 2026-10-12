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

The setter linkage test builds the driver as C and links C and C++ callers.
Both SystemSetCoreClock and the retained SystemCoreClockSet symbol configure
the existing 48 MHz target and report the CPU rate after the AHB divider.
The test supplies PLL lock indications; it does not simulate analog startup.
The original source fails to link a caller of SystemSetCoreClock. The old
setter also returned the 96 MHz PLL rate for a 48 MHz CPU configuration.

Oscillator startup, PLL
programming, board oscillator selection and peripheral-clock API completion
remain separate review items. The system oscillator defaults to 12 MHz; a successful crystal setter records
the supplied input so later clock updates also work with other crystals.
Watchdog frequencies are nominal, not measurements.

Reference: NXP UM10462 revision 4.1, chapter 3, Tables 9, 14, 16, 20 and 22.
https://www.keil.com/dd/docs/datashts/nxp/lpc11uxx/um10462.pdf

PLL selection is validated before any clock register changes. Invalid physical
inputs (outside 10-25 MHz) and inputs with no exact 48 MHz CPU/USB solution
return zero and preserve registers and clock globals. This is an input-clock
constraint, not a nearest-rate request. The search includes M=1..32, all four
P encodings and the inclusive 156-320 MHz CCO range, with 64-bit arithmetic.
Tests cover negative/zero/large inputs, valid-range inputs with no solution,
and 12/16/19.2/24 MHz crystal configurations through both entry points.
PLL lock waits and safe switching of an already active PLL still require
separate startup work; these tests supply lock bits in mapped memory.
