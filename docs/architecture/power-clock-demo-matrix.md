# Power and clock demo projects

The projects use the shared `exemples/misc/clock_demo.cpp` and
`exemples/misc/power_off_demo.cpp`. Import each `ioc` directory in Eclipse and
build the matching library configuration first. Board pin maps come from the
existing example named in each README. These additions provide 29 ClockDemo
projects and 10 missing PowerOffDemo projects outside ESP32.

## Compile and link results

All 114 configurations below linked successfully. These are command-line GNU
builds using the managed project source lists and settings, not Eclipse builds.
Arm uses GCC 15.2.1; R9A02 uses RISC-V GCC 15.2.0. The checks compile the available
library sources, archive successful objects, then compile and link each demo.
Where the source count is incomplete, unrelated SDK-dependent library sources
failed or were unavailable. A linked demo does not establish that the complete
library project builds. Debug/Release results are listed separately where the
source counts differ. RE01 uses DebugCFB/ReleaseCFB.

| Target | ClockDemo | PowerOffDemo | Compiled library sources (Debug; Release) |
| --- | --- | --- | --- |
| SAM4E16E | PASS / PASS | PASS / PASS | 105/105; 105/105 |
| SAM4LCxC | PASS / PASS | PASS / PASS | 120/120; 120/120 |
| SAM4LSxC | PASS / PASS | PASS / PASS | 128/128; 128/128 |
| SAMD11 | PASS / PASS | PASS / PASS | 239/239; 239/239 |
| SAMD21 | PASS / PASS | PASS / PASS | 317/317; 317/317 |
| LPC11U35 | PASS / PASS | PASS / PASS | 114/114; 114/114 |
| LPC1769 | PASS / PASS | PASS / PASS | 124/124; 124/124 |
| LPC54628 | PASS / PASS | PASS / PASS | 179/179; 179/179 |
| nRF52832 | PASS / PASS | PASS / PASS | 334/474; 334/474 |
| nRF52840 | PASS / PASS | PASS / PASS | 344/458; 344/458 |
| nRF5340_App | PASS / PASS | PASS / PASS | 196/196; 196/196 |
| nRF5340_Net | PASS / PASS | Not provided | 116/118; 116/118 |
| nRF54L15 | PASS / PASS | PASS / PASS | 234/255; 234/255 |
| nRF54LM20x | PASS / PASS | PASS / PASS | 249/270; 249/270 |
| nRF9160 | PASS / PASS | PASS / PASS | 178/183; 178/183 |
| nRF91x1 | PASS / PASS | PASS / PASS | 179/184; 179/184 |
| RA4M1 | PASS / PASS | PASS / PASS | 305/305; 305/305 |
| RE01_1500KB | PASS / PASS | PASS / PASS | 113/113; 113/113 |
| STM32F030x8 | PASS / PASS | PASS / PASS | 109/109; 109/109 |
| STM32F401 | PASS / PASS | PASS / PASS | 68/68; 68/68 |
| STM32G474 | PASS / PASS | PASS / PASS | 173/173; 173/173 |
| STM32H753 | PASS / PASS | PASS / PASS | 322/322; 322/322 |
| STM32L152RE | PASS / PASS | PASS / PASS | 45/45; 45/45 |
| STM32L432 | PASS / PASS | PASS / PASS | 185/185; 185/185 |
| STM32L476 | PASS / PASS | PASS / PASS | 294/294; 294/294 |
| STM32L496 | PASS / PASS | PASS / PASS | 202/202; 202/202 |
| STM32L4S9 | PASS / PASS | PASS / PASS | 186/186; 186/186 |
| STM32WBA65 | PASS / PASS | PASS / PASS | 285/297; 285/297 |
| R9A02G021 | PASS / PASS | PASS / PASS | 196/196; 196/196 |

C3/C6 regression builds also pass: ClockDemo Debug/Release and PowerOffDemo
Light-sleep/Deep-sleep in both configurations (12 links). All 204 C3 and 206 C6
library sources compile. No new board execution was performed for this batch.
The shared demo now uses the UART C functions, as it already does for Timer.
This allows LTO to discard unused C++ UART methods without changing driver APIs.
SAMD11 selects SysTick as timer 4 and uses size optimization and LTO in both
configurations: 15,086 text, 208 data, 1,936 bss. Flash use is 15,294 bytes;
static RAM is 2,144 bytes, with stack space still required.

## What each configuration exercises

- SAM4E: internal RC at 4/8/12 MHz; SAM4L: internal RC at 4/12 MHz and LF RC.
  SAMD11/21: OSC8M; LF changes omitted while DFLL/DPLL may own the reference.
- LPC11/17/54628: internal source selection. LPC11 also selects its watchdog
  oscillator without enabling the watchdog. APB changes are omitted.
- Nordic: balanced HFXO request/release and LF RC selection. Clock ownership
  can refuse a change while another peripheral or radio owns the source.
- RA4M1, RE01 and R9A02: internal high-speed and low-speed RC selection.
- STM32F0/G4/L4/WBA: internal source selection; G4/L4/WBA include APB divisors.
  STM32F4/H7/L1 only read existing core clocks because their generic runtime
  selectors are incomplete. These are not runtime clock-switch validation.

The new ClockDemo configurations power off and reinitialize UART around clock
changes. They finish clock selection before acquiring a timer because some
ports retain timer module ownership after TimerDisable. The final timer check
requests 1 MHz and a 10 ms interrupt period over a 250 ms delay. It does not
validate timer operation at every intermediate clock. Unsupported selectors
print SKIP rather than being counted as a successful switch. Clock readback
and delay checks do not independently establish oscillator accuracy.

The new PowerOffDemo configurations have no assumed wake pin. They report
reset/retention information, power off UART and request MCU power off. RESET
or a power cycle restarts the application. Configure a supported board wake
pin before using them to test GPIO wake. Existing board-specific PowerOffDemo
configurations keep their wiring and modes. No demo enables a watchdog.

## Hardware and remaining coverage

C3 ClockDemo after 04d1522a8 passed three cycles and nine BOOT wakes with zero
failures. C3 PowerOffDemo passed six Light-sleep SRAM checks and four manual
GPIO3 Deep-sleep wake/restart cycles. C6 also passed repeated wake and combined
clock/sleep runs. SAMD11 passed Standby wake and SRAM checks. ClockDemo completed three
core/APB selections at 2de35f281 and measured 250094 us for msDelay and
250107 us for nsDelay, each with 25 IRQs for a 250 ms delay,
with zero failures. See
[core power and clock ports](core-power-clock-ports.md) for exact results.
Other new configurations remain build-only. Sleep current, UART RX after clock
changes and external oscillator accuracy have not been measured here.

LPC54605 has no existing board example to supply console wiring. nRF52805 and
nRF52810 lack a suitable UART board configuration; no pin maps were guessed.
nRF5340 network-core ClockDemo has no companion PowerOffDemo because the
application core controls system power. RP2040, nRF54H20 and ESP32C2/C5/C61
remain outside this batch because their relevant ports are incomplete.

## Delay calibration after clock changes

The ARM delay loops use SystemMicroSecLoopCnt and SystemnsDelayFactor as
iterations per microsecond, with 16 fractional bits.
Each port refreshes it in SystemCoreClockUpdate. LPC11 and SAM4L also refresh
it on their direct clock assignments during transitions and fallback. ResetEntry
no longer overwrites the port calibration after startup. Rebuild the library
and application together when changing from the former integer count.

The shared idelay.h calculation has no MCU-specific path. SAMD supplies its
13-cycle calibration from system_samd.c. Other ARM ports retain their existing
16-cycle calibration; their physical loop timing still needs board measurement.
Nanosecond delays retain existing port calibration where available; ports
without a prior calibration use a nominal three-cycle loop. These values
require hardware timing validation. Both counts update together. Zero delays
execute no loop, and nanosecond scaling uses 64-bit arithmetic.
RISC-V cycle-counter delays preserve fractional MHz for every target, using
the current SystemCoreClock. Loop overhead, interrupt handling and flash stalls
can extend delays; these functions do not replace a hardware timer.

The fixed-point ARM implementation passed the SAMD11 board run above. Other
MCUs and short nanosecond delays still require timing measurements.

ClockDemo now measures msDelay(250) and nsDelay(250000000) separately against
the same timer, checking elapsed time within two percent and interrupt counts.
This checks scaling over a measurable interval. It does not establish the
accuracy of an individual delay of a few nanoseconds; setup overhead dominates
such short calls. Both tests run after resume where ClockDemo supports sleep.

Software validation for the shared delay change: ClockDemo and PowerOffDemo
linked in 114 configurations across 29 targets; C3/C6 ClockDemo added four
links, and SAMD11 display-console and timer demos added four. Some Nordic
libraries still have unrelated missing SDK sources; these counts describe
successful demo links, not a complete library-source build. Existing LPC,
STM32F0/F4/G4/L4 and RE01 checks passed. Delay arithmetic passed 244 host
checks, including zero and UINT32_MAX nanosecond requests. ARM assembly
compiled for six Cortex-M cores at -O0/-Os; RISC-V counter and fallback
paths compiled at both optimization levels. None of these checks measures
physical instruction timing.

The RISC-V delay header contains no MCU checks or CSR addresses.
IOsonataRiscvCycle32 is supplied by the MCU system implementation: the
Espressif performance counter or standard cycle counter, and Renesas mcycle.
The shared code handles fractional frequencies and divides long waits into
intervals shorter than half the 32-bit counter range. msDelay calculates
cycles directly instead of overflowing a millisecond-to-microsecond product.
The non-CSR path remains an approximate two-instruction loop.

The RISC-V counter model passed 108 cases covering zero, fractional MHz,
counter wrap and long durations. The header compiled for RV32, RV32E and
RV64, with and without CSR use. C3/C6 ClockDemo and R9A02 ClockDemo/PowerOffDemo
relinked in Debug and Release. C2/C5/C61 startup and GPIO/UART host checks
passed, along with 24 C/C++ UART demo links. These do not establish hardware
support or measured timing on those additional chips.

SAMD11 ClockDemo now defaults to three SW0 Standby wake cycles after startup
clock checks and a baseline delay measurement. CPU/APB clocks, oscillator
descriptors and both delay/timer checks are repeated after every wake. Set
CLOCK_DEMO_RESUME_TEST to 0 in board.h to repeat the validated clock-selection
run. The complete source-selection plus sleep build exceeded D11 flash by
600 bytes; the two selectable runs fit separately. The Standby mode keeps the
startup clock and APB rate. The run at e017243bc passed all three SW0 wakes,
CPU/APB clocks and LF descriptors, with zero failures. Post-wake msDelay was
250097-250103 us and nsDelay 250108-250112 us, each with 25 interrupts.

Both SAMD11 modes linked in Debug and Release. The Standby mode uses 16,120
flash bytes and 2,132 static RAM bytes, excluding the stack. SAMD21 and
STM32F401 ClockDemo/PowerOffDemo and C3/C6 ClockDemo also relinked in both
configurations after the generic demo change. The D11 Standby mode passed
hardware validation as recorded above.

D11 Standby mode now enables CLOCK_DEMO_UART_RX_TEST. Send uppercase U at the
terminal prompt before the first sleep and after each wake. Each response has
a 30-second polling budget, and timeout increments the demo failure count.
This checks polling RX after UART disable/enable across Standby; it does not
validate interrupt or DMA receive modes. The receive check remains pending.

The UART receive version linked in both D11 modes, Debug and Release. Standby
mode uses 16,120 flash bytes and 2,132 static RAM bytes before the stack.
SAMD21 ClockDemo/PowerOffDemo and C3/C6 ClockDemo compatibility links passed.
The receive helper's response, timeout, unrelated-input, stale-input and timer
failure paths passed host checks; physical receive operation remains pending.
