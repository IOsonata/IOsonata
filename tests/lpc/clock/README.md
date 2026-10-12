# LPC11Uxx clock decoding

Run `python3 tests/lpc/clock/run.py` from the repository root.

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

The system oscillator defaults to 12 MHz; a successful external-clock configuration records
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
Clock reconfiguration now moves the CPU to IRC before powering down and
programming either PLL. Flash timing is set before switching up to 48 MHz.
Crystal range selection covers both oscillator ranges. Startup uses bounded
read-count delays and waits, independent of timers or delay calibration.

Callers must stop USB and other clock-dependent peripherals before changing
clocks. IRC mode leaves USB clock/pads disabled. Missing system-PLL or USB-PLL
lock returns zero, leaves the CPU on the 12 MHz IRC and disables both PLLs and
USB. Tests inject each lock failure after a successful PLL configuration and
verify fallback and successful retry through both C and C++ entry points.
They also check reserved flash bits and crystal range selection. Tests supply
lock bits in mapped memory; analog startup timing, clock glitches, IRQ latency
and the physical switching sequence still need board validation. A hardware
clock-source latch failure is distinct from PLL lock failure: if even the IRC
fallback latch fails, the globals report an unknown frequency (zero), and the
PLLs remain powered rather than stopping a possible CPU clock source.

SystemInit uses the generic weak g_McuOsc board descriptor. Its default remains
a 12 MHz crystal with USB enabled. An application can override it with its own
McuOsc_t definition (the MCUOSC pattern used by TimerDemo). OSC_TYPE_RC uses the
fixed 12 MHz IRC input, OSC_TYPE_XTAL uses the crystal, and OSC_TYPE_TCXO sets
SYSOSCCTRL.BYPASS for an external clock on XTALIN. All retain the existing
48 MHz CPU target. bUSBClk=false leaves USB PLL, clock and pads disabled;
IRC mode also leaves USB disabled even if bUSBClk is true.

Tests link the weak default and a strong board definition separately. The
strong-definition tests cover all three oscillator types and both USB choices
in C and C++, including 24 MHz startup with no USB lock indication required.
Legacy boolean setters retain their existing crystal/IRC and USB behavior;
they do not use the board descriptor. The low-power oscillator descriptor is
not used to program WDTOSCCTRL by this change.

The generic SystemCoreClockSelect entry point uses the same implementation as
startup, honors g_McuOsc.bUSBClk and updates CoreOsc.Type/Freq only on success.
OSC_TYPE_RC requires its actual 12 MHz input frequency. Invalid types or inputs
return false without register changes. A PLL lock failure returns false and
retains the descriptor; SystemCoreClock reports the actual IRC fallback rate.
This function selects a physical input and retains the fixed 48 MHz CPU target.

C and C++ callers test generic selection, descriptor updates, invalid-input
preservation, USB lock failure and successful retry. The previous source fails
to link these callers because SystemCoreClockSelect was missing.

SystemPeriphClockGet(0) reports the system bus clock (SystemCoreClock), matching
the LPC546xx index convention. Other indices return zero. UART/SSP function
clocks are separate and remain driver-managed. SystemPeriphClockSet explicitly
returns zero without changes: the bus has no divider independent of the CPU.
Clock-decoding tests check the getter across every tested source and AHB divider,
invalid indices and the setter's preservation of registers and CPU frequency.

## LPC17xx clock decoding

The same runner also compiles the production LPC17xx clock decoder at O0/O2
with UBSan and the vendor CMSIS peripheral declarations. It checks the
12 MHz / M=400 / N=24 / CPU-divider=4 configuration (100 MHz), the RTC
M=4272 / N=1 / CPU-divider=4 configuration (69,992,448 Hz), and reserved
PLL0STAT bit 15. The former overflowed the 32-bit product and the latter
lost multiplier bit 12 with the old 0xEFFF mask. The decoder now uses bits
14:0 and 64-bit intermediate arithmetic through the CPU division.

Tests also cover all CPU dividers, each source with the PLL bypassed,
every multiplier bit and every predivider encoding. Some field-decoding
cases are outside permitted operating limits; they are not clock settings
to program on a board. Every update must leave all peripheral registers
unchanged and update the existing delay-loop factor. Startup and clock
setter sections are discarded from this host test; PLL setup, switching,
flash timing and analog lock behavior are not validated by it.

Reference: NXP UM10360 revision 2, chapter 4, sections 4.5.5 and 4.5.10,
and table 21 for the RTC setting.
https://www.pololu.com/file/0J431/user.manual.lpc17xx.pdf
