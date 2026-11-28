# LPC11Uxx clock setup

Run `python3 tests/lpc/clock/run.py` from the repository root.

The host tests compile the actual system_LPC11Uxx.c with vendor CMSIS
peripheral types and map SYSCON and flash controller memory. The PLL lock
indications are written in the mapped memory; no analog startup is simulated.

clock_test.c checks SystemCoreClockUpdate: the CPU clock is the main clock
stored by the clock setup divided by SYSAHBCLKDIV, 0 for divider 0, for every
divider. The clock source and PLL registers are not read, no register is
written, and the delay factor follows the CPU clock. SystemPeriphClockGet(0)
is the bus clock, other indices give 0. SystemPeriphClockSet returns the bus
clock in effect and changes nothing, the bus has no divider of its own.

clock_link_test.c builds the driver as C and links C and C++ callers with a
strong g_McuOsc. SystemInit is the only setup: the library object defines none
of SystemCoreClockSelect, SystemLowFreqClockSelect, SystemCoreClockSet or
SystemSetCoreClock. The test covers the IRC, crystal and external clock
descriptors with USB requested and not, the 12, 16, 19.2 and 24 MHz crystal
PLL settings (CCO 156 to 320 MHz, USB PLL equal to the system PLL), flash
timing, inputs with no 48 MHz setting and an unknown oscillator type. These
give 48 MHz from the IRC without USB, and the descriptor then describes the
IRC (accuracy 10000 ppm, no load capacitance). An RC descriptor keeps the
application accuracy. A USB PLL that does not lock fails the crystal setup;
the IRC setup without USB follows. A system PLL that does not lock leaves the
CPU on the 12 MHz IRC with both PLLs and USB off.

The watchdog oscillator is set up when g_McuOsc.LowPwrOsc is an RC with a
nonzero frequency, to the closest nominal rate (9375 Hz to 2.3 MHz,
40 percent accuracy). The watchdog bus clock stays off. A zero frequency or a
crystal descriptor leaves the oscillator powered down.

Reference: NXP UM10462 revision 4.1, chapter 3, Tables 9, 14, 16, 20 and 22.
https://www.keil.com/dd/docs/datashts/nxp/lpc11uxx/um10462.pdf

## LPC17xx clock setup

The same runner builds the LPC17xx tests at O0/O2 with UBSan and the vendor
CMSIS peripheral declarations. The tests include system_LPC17xx.c to reach
its startup functions. A separately compiled object of the source is checked
for the symbols it must not define: SystemRamStart, owned by ResetEntry.c,
and the removed run time clock functions SystemCoreClockSelect,
SystemLowFreqClockSelect, SystemCoreClockSet, SystemSetCoreClock,
SystemConnectPLL1, GetSysClk and SystemClkFreq.

clock17_test.c checks SystemCoreClockUpdate: the CPU clock is the clock the
setup left at the CPU divider (PLL0 output, or the input with PLL0 off)
divided by CCLKCFG, for every divider. CLKSRCSEL and PLL0STAT are not read
and no register is written. SystemPeriphClockGet decodes every LPC17_PCLK
field and selection, the CAN divider of 6 and the fixed CCLK/8 RTC interface
clock. SystemPeriphClockSet selects the closest divider by rational error,
ties to the lower rate, sets CAN1, CAN2 and ACF together, keeps the other
fields and the interrupt mask, and returns 0 for an invalid index or an
unknown CPU clock.

flash17_test.c intercepts the system control block. SystemInit takes the CPU
oscillator from g_McuOsc.CoreOsc: the 4 MHz IRC, a crystal or an external
clock on XTAL1 at 1 to 25 MHz, at CORE_FREQ or the closest rate below the
device limit (120 MHz for LPC1759/LPC1769, 100 MHz otherwise). The PLL0
search, M 6 to 512 and N 1 to 32 with the CCO at 275 to 550 MHz, is checked
against an exhaustive oracle at several CPU rates. A missing oscillator, a
PLL that does not lock or connect, or an invalid descriptor leaves the CPU on
the undivided IRC, and the descriptor then describes the IRC (10000 ppm, no
load capacitance); an RC descriptor keeps the application accuracy.

The fixture also checks: six flash clocks during every change and the final
FLASHTIM for the CPU rate; the OSCSTAT wait with delayed ready status in both
range selections, and its bounded timeout without a clock change; bounded
PLL0 lock and connect waits; the settle time of a PLL reference above 20 MHz;
the disconnect and stop of a PLL0 left running by a bootloader before any
configuration change; the protected PLL0 feed with interrupts on and off;
USBCLKCFG written only for a MAIN PLL0 at 288, 384 or 480 MHz; and no range
change of the main oscillator while it drives the CPU.

After a PLL0 lock timeout the CPU runs from the input through its divider and
SystemCoreClock gives that rate. When PLL0 does not acknowledge a disconnect
or a stop, the CPU clock is not known and SystemCoreClock is 0.

Lock and ready status come from the fixture; feed latching, analog lock and
oscillator startup timing are not simulated and need board validation.
Reference: NXP UM10360 revision 2, chapters 3, 4 and 5.
https://www.pololu.com/file/0J431/user.manual.lpc17xx.pdf
