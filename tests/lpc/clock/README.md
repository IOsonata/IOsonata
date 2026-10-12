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

## LPC17xx flash timing

SystemCoreClockSet selects the safe six-clock flash access time before
changing a clock source or divider. After reading the resulting CPU rate,
it selects enough access clocks for that rate, preserving all other
FLASHCFG bits. Rates above 100 MHz retain the conservative six-clock
setting; this change does not select the optional five-clock timing of
LPC1759/LPC1769 at 120 MHz.

The host fixture uses LPC_SC_TypeDef from the vendor header and intercepts
register-block accesses. It checks that six-clock timing remains in effect
through clock programming and checks final timing at 20/22, 40/42, 60/62,
80/82, 100/102 and 120 MHz. The test supplies ready PLL status from the
configuration; it does not emulate feed latching, oscillator startup or
physical flash access. Existing PLL selection and switching limitations
remain separate work. The old code fails both transition ordering and
between-boundary timing checks. O0/O2 UBSan tests pass after the fix.
Reference: NXP UM10360 revision 2, chapter 5, table 49.

The external-oscillator path now waits for SCS.OSCSTAT (bit 6) to become
set before changing the CPU divider or selecting the main oscillator.
The previous comparison against 0x50 included enable/range bits and
skipped the wait with the oscillator enabled. The fixture tests 12 MHz
and 16 MHz inputs (both range selections), with immediately ready and
delayed status. It fails if clock changes occur before ready status.
Reference: UM10360 chapter 3, table 13.

The earlier multiplier search overflowed its signed FCCO * N product on
99/101/119 MHz requests from IRC. These remain regression cases. The new
selection search uses 64-bit arithmetic and finds exact rates with N=2,
M=99/101/119 and CPU divider=4, preferring smaller N over the previous N=8
settings when their frequency errors are equal. Flash ordering is checked.

PLL0CFG encoding now preserves all MSEL bits 14:0 and excludes reserved
bit 15, matching the corrected status decoder. The setter previously
cleared bit 12 with 0xEFFF, silently changing supported RTC multipliers.
Regressions request 72,007,680 Hz and 84,000,768 Hz from the 32,768 Hz RTC,
checking M=4395 and M=5127 respectively, N=1 and CPU divider=4. Both
multipliers are listed in UM10360 table 21. The first request fails before
the fix; both pass afterward at O0/O2 with UBSan. As in the other setter
tests, lock status is supplied by the fixture. This does not validate RTC
PLL lock detection, USB clock suitability, or arbitrary rate selection.

The legacy SystemCoreClockSet input-frequency argument is checked before
any peripheral or clock-global change. Zero, negative values and physical
inputs outside 1-25 MHz (except the exact 32768 Hz RTC input) return zero.
The 4 MHz value retains its existing IRC meaning. Other valid main-oscillator
inputs, including 1, 2 and 3 MHz, select MAIN and wait for readiness rather
than being mistaken for RTC. These limits describe the physical oscillator,
not the requested CPU frequency; this change does not reject or clamp CPU
rate requests. Reference: UM10360 chapter 3, table 13.

Tests check invalid-input preservation, including both signed integer
extremes, and 120 MHz CPU setup from 1/2/3 MHz main inputs with delayed ready
status. Existing tests retain 4 MHz IRC, 32768 Hz RTC and 12/16 MHz main
coverage. The original zero-input call fails UBSan with division by zero.

The main-oscillator wait is bounded to 1,000,000 status reads, following
the bounded-polling approach in LPC11Uxx startup. If OSCSTAT never arrives,
the setter returns zero before selecting the input or changing CPU-divider
and PLL settings. Clock globals retain their previous values. The main
oscillator remains enabled and flash timing remains at the safe six-clock
setting, allowing a later retry. This is a read-count bound, not a calibrated
startup time; its timing still needs board validation.

Tests withhold oscillator readiness for both range selections, verify an
IRC-started attempt returns without changing its clock selection or globals,
and then allow a successful retry. A host read-count guard catches the
previous endless wait.

PLL0 lock polling is also bounded to 1,000,000 status reads. On timeout,
the setter returns zero before requesting PLL connection or changing the
USB divider. It refreshes SystemCoreClock and the delay-loop factor from
the active clock registers. In a startup attempt the CPU therefore stays
on the selected input through the programmed CPU divider, with safe flash
timing; it does not report the requested PLL rate. PLL0 remains enabled
but is not requested as the CPU source. This is not an IRC reset sequence.

Tests withhold lock on IRC and main-oscillator startup paths, check that
no connect request occurs, and verify the reported bypass rate and retained
USB divider. The fixture now derives enable/connect status from PLL0CON;
feed latching and analog timing are still not simulated. The old loop hits
the read-count guard. Physical lock behavior, especially with RTC input,
still needs hardware validation.

PLL0 connection polling now has the same read-count bound. On timeout it
clears the connection request and feeds PLL0, then waits with a bound for
any late connection to clear. It returns zero and updates the reported
CPU rate from status. PLL0 stays enabled throughout recovery because it
may still be supplying the CPU if disconnection is not acknowledged.
Flash timing stays safe and the USB divider is not changed.

The host tests cover a connection that never arrives and a late connection
whose status remains set despite cancellation. Both must return failure,
cancel the request, retain PLL enable, and report the status-indicated
bypass or PLL rate. The old connect loop hits the test read-count guard.
This validates control flow; physical feed timing and clock transitions
still require hardware testing.

All PLL0 and PLL1 feed pairs in system_LPC17xx.c use a common helper that
saves PRIMASK, disables maskable interrupts, writes 0xAA then 0x55 through
the vendor register pointer, and restores PRIMASK. The protected section
contains only the two feed writes. Polling waits remain outside it.
This follows the saved-interrupt-state pattern already used by LPC17xx IAP
and the feed requirement in UM10360 section 4.5.8.

The host fixture detects an unprotected first feed write in the old code.
Tests exercise both PLL paths with interrupts initially enabled and disabled,
checking guard calls and exact mask restoration. Target assembly is checked
for CPSID before the stores and PRIMASK restoration afterward. This change
protects feed sequencing; it does not simulate hardware IRQ delivery.

PLL1 USB setup uses the documented 12/16/24 MHz main-oscillator inputs,
M=4/3/2 respectively, and P=2 (PSEL=1) for a 192 MHz CCO and 48 MHz USB
output. The old direct-divider path used P=1, putting the CCO at 96 MHz,
below its 156-320 MHz range. The old alternate search could also produce
incorrect USB rates. See UM10360 sections 4.6 and 4.6.9.

The legacy no-argument API requires MAIN selected and ready so its frequency
is known from SystemClkFreq. It leaves registers unchanged for unknown,
unready or unsupported inputs. This is a fixed USB clock requirement,
not an arbitrary frequency request. Hardware permits PLL1 while the CPU
uses another source, but this legacy API cannot independently describe the
main oscillator in that case. The caller must stop USB before setup.
Tests check all three valid encodings and preservation for invalid inputs,
including zero, plus IRC/RTC selection and an unready main oscillator.
Before writing PLL1CFG, setup disables and disconnects PLL1 with a protected
feed and waits for both status bits to clear. This stop wait is bounded to
1,000,000 status reads. On timeout, setup leaves the old configuration and
the disable request intact and returns without enabling or reconnecting.
Host tests start with an active PLL1 and check that configuration changes
only after shutdown, including a blocked-stop case and both IRQ mask states.
The regression fails against the previous code's configuration write while
PLL1 is still active.

PLL1 lock and connection waits are now each bounded to 1,000,000 status
reads. A timeout writes PLL1CON=0 and performs the protected feed sequence,
cancelling connection and disabling PLL1 before returning. PLL1 serves USB,
so this does not stop the CPU clock. USB must already be stopped as required
by this API. The existing void return type is unchanged and provides no
explicit error result; returning alone is not proof that the USB clock is
ready. The read-count bounds still require hardware timing validation.

Host tests withhold lock or connection for each 12/16/24 MHz input, verify
disable/feed, unchanged CPU clock and USB divider, preserved IRQ mask,
and successful retry when ready status is allowed. The old unbounded wait
hits the fixture's read-count guard. Hardware lock/connect transitions,
feed latching and active-PLL reconfiguration remain unvalidated.

PLL0 setup now follows UM10360 section 4.5.13 when the CPU PLL is already
active: disconnect and feed, confirm disconnection, disable and feed, then
confirm shutdown before changing CCLKCFG, CLKSRCSEL or PLL0CFG. Both waits
are bounded to 1,000,000 status reads. A disconnect timeout keeps PLL0
enabled; a shutdown timeout leaves the disable request in place. Both
return zero without changing the old source, divider, PLL configuration,
SystemClkFreq or USB divider, and refresh SystemCoreClock and its delay
factor from status. Flash timing stays at six clocks. The caller must stop
clock-dependent peripherals before changing CPU clocks.

Host cases start at 80 MHz from MAIN and request 120 MHz from IRC. They
check shutdown before register changes, blocked disconnect, blocked disable,
actual-rate reporting and both initial IRQ masks. The previous code fails
by changing the divider while PLL0 is still active. Startup tests also begin
with an active bootloader PLL and acknowledge feeds so initialization must
complete through the shutdown sequence. Analog timing and feed latching
remain hardware-validation work.

The LPC17xx public header declares both existing clock entry points with C
linkage. SystemCoreClockSet takes the physical input frequency in Hz.
SystemSetCoreClock retains its documented CLKSRCSEL selector argument and
translates IRC/MAIN/RTC into 4 MHz/16 MHz/32768 Hz before calling the same
implementation. MAIN uses the existing OSC_FREQ default; other main crystal
frequencies use SystemCoreClockSet. Unknown selectors return zero without
changing registers or clock globals. These are target-specific legacy APIs;
the generic system clock header is unchanged.

The clock fixture no longer supplies its own SystemCoreClockSet declaration.
Without the fix it fails compilation against the public header. The runner
compiles production and fixture sources as C, then links C and C++ callers
at O0/O2 with UBSan. Both public names are called from those callers; the
fixture checks all three selector mappings and rejects invalid selectors,
including a frequency accidentally passed in the selector position.

LPC17xx CPU rate selection now compares legal PLL0 ratios and input bypass
using exact rational error in 64-bit arithmetic. MAIN/IRC candidates use
M=6..512, RTC candidates use the 65 multipliers from UM10360 table 26, and
N is limited to 1..32. PLL CCO stays within 275..550 MHz, CPU division is
1..256 for bypass and constrained by the CPU ceiling for PLL operation.
The ceiling is the port's existing CORE_FREQ (120 MHz in this configuration);
this does not add device-specific 100 MHz part selection. Zero selects the
lowest rate rather than disabling the clock. Returned Hz truncates fractions.

For each PLL ratio, only the two divider values bracketing the request need
to be checked. Equal-error choices prefer an exact multiple of 96 MHz CCO
for the existing USB divider path, then retain the first candidate (bypass,
lower N, lower M). CPU-rate selection alone does not establish USB clock
suitability: input accuracy and fixed USB requirements still apply. Bypass
leaves PLL0 stopped and the USB divider unchanged. Flash timing is finalized
for bypass as well as PLL operation.

The independent host oracle enumerates every legal divider for each supported
M/N, checks the programmed ratio is legal, and verifies that no candidate
has smaller rational error. Cases include zero, UINT32_MAX, low rates,
inexact requests, 4/12/25 MHz input and RTC. The previous implementation
fails these cases by selecting an unsupported multiplier. Clock C/C++ tests,
startup tests and Cortex-M3 assembly compilation pass at O0/O2. Physical
PLL behavior, including RTC lock timing, remains unvalidated.
