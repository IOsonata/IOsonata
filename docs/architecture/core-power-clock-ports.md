# MCU core power and clock ports

The public functions are in `include/coredev/system_core_clock.h`.
Implementations belong to the MCU library. Applications use the same functions
and `exemples/misc/power_off_demo.cpp`, with board pins in `board.h`.

Power off means a mode exited by reset, not an ordinary interrupt-resumable
sleep. Unsupported retention ranges and wake pins return false. An implemented
operation must not return success without programming or checking the hardware.
Clock selection follows the MCU clock tree. The Nordic HFXO request count is
appropriate for its on-demand crystal; it must not stop a clock supplying another
MCU's CPU, PLL, USB, or peripheral.

## Source coverage

This table records source implementation, not hardware validation or completion
of every clock mode. Existing clock code still needs a family-by-family review.

| Family | Generic power functions | Clock work remaining |
| --- | --- | --- |
| nRF52 | Existing dedicated implementation | Preserve SoftDevice/MPSL ownership and tested HFXO requests |
| nRF53, nRF54L, nRF91 | Existing shared implementation, with per-core restrictions | Verify per-core and SDK configurations |
| STM32 F0/F4/L1/L4/G4/H7 | Existing shared implementation | Audit selectors, ready waits and active clock users |
| STM32 WBA | WBA65I Standby, GPIO wake, reset reason, backup registers and bank-wide SRAM retention on working branch | Validate radio ownership, SRAM bank retention and target build |
| SAM4E | Reset, Backup mode, wake pins, GPBR; no SRAM retention in Backup | Slow-clock and main-clock/PLLA selection implemented; hardware validation pending |
| SAM4L | Reset, Backup mode, EIC wake pins, four backup registers; no SRAM retention | Runtime recovery and main/low-frequency peripheral checks implemented; hardware validation pending |
| SAM D11/D21 | PM reset reasons and interrupt-resumable Standby; SAMD11 SW0/PA14 repeated resume passed | Validate SAMD21 wake, full SRAM continuity, sleep current and peripheral clock ownership |
| LPC11Uxx | Deep power-down, fixed wake pin, four retention words, reset causes and ordinary Sleep implemented | Core/LF selectors and enabled peripheral checks compiled; hardware pending |
| LPC17xx | Deep Power-down, reset reasons, five backup words and ordinary Sleep implemented | Runtime core/USB clock checks compiled; fixed RTC source has no selector; hardware pending |
| LPC546xx | Deep power-down, ordinary Sleep, reset causes and seven RTC backup words implemented | Runtime peripheral/PLL ownership and clock decoding checked; hardware pending |
| RA4M1 | Reset reporting, external-IRQ Software Standby and ordinary Sleep; no reset-on-wake power-off mode | Runtime module/CLKOUT checks added; protected clock models and firmware builds passed; hardware pending |
| R9A02G021 | Reset reporting, wake pins and resumable Sleep/Software Standby; no reset-on-wake power off | Runtime module and LF ownership checks added; explicit Snooze entry still needs erratum review; hardware pending |
| RE01 | Reset reporting, external-IRQ Software Standby, ordinary Sleep and reset-on-wake Deep Software Standby | Core/LF selectors, active-module checks and Boost exit implemented; hardware pending |
| ESP32 C-series | C3/C6 CPU sleep, GPIO wake and reset reason implemented; clock readout corrected | C3/C6 LF selection and calibration implemented; runtime core/peripheral roots, PMU Light-sleep/Deep-sleep and retention pending; hardware pending |
| nRF54H, Raspberry Pi ports | No generic power implementation found | Review port maturity and vendor startup integration first |

## SAM D11/D21 low-power work (0.14, unmerged)

The consolidated `review_system_power_014` branch contains
`SystemSleep()` (resume execution after Standby), EIC asynchronous wake
configuration, and the PM RCAUSE-based `SystemResetReason()` convenience
function. The shared `PowerOffDemo` selects the resumable path for SAMD
without changing existing reset-on-wake power-off behavior on other MCUs.
No separate `ResetReasonDemo` is retained in the consolidated branch.

Maintainer-reported reset-cause tests passed 128/128 combinations for
each MCU, and Arm GNU source compilation passed for D11/D21 including
the Standby source. On November 15, 2026, the maintainer reported five
consecutive SAMD11 SW0/PA14 Standby resumes, with counts 0x00000001 through
0x00000005 and another Standby entry. Subsequent D11 runs also passed the
128-byte static and 128-byte stack SRAM samples across five wakeups. This
is a sample of live RAM continuity, not a full SRAM test. SAMD21 hardware wake, full SRAM
testing and sleep-current measurements remain pending.

SAMD runtime clock selection returns the clock setup result. A failed
core-clock change attempts the same OSC8M fallback used at startup, updates
`SystemCoreClock` from the registers and returns false. Low-frequency
selection returns false when oscillator startup or generator synchronization
fails. Repeating the same request executes setup again, so a saved oscillator
descriptor cannot hide a previous failure. The descriptors retain the requested
configuration; a failed operation does not promise to restore the previous
clock tree. Frequency queries return zero for an oscillator that is not ready,
a disabled or unlocked DPLL, an unlocked closed-loop DFLL, or a failed GCLK
register read. USB recovery retains its nominal frequency before USB lock.
OSC8M does not require OSC8MRDY when already enabled, preserving the D11
startup behavior observed on hardware. Channel and generator reads share the
same bounded synchronization sequence.

Disabling the last EIC user releases its generic-clock channel after successful
synchronization. Interrupt, event, wake, NMI and sense users are checked before
stopping the shared controller. SW0 NMI wake polarity can be changed after
initial configuration. Failed wake/event reconfiguration retains ownership
until the line configuration succeeds, allowing a retry. Standby entry
continues to preserve the existing NMI/IRQ pending-state handling.

The caller must stop and release affected clock consumers before either
selector, including a repeated request. Under the interrupt mask, both selectors
now refuse enabled watchdog, RTC, EIC, USB or CRC operation, armed or active
DMA channels, pending DMA completions, allocated
peripheral generic-clock channels, and enabled clock outputs. A gated register
interface or a generic-clock synchronization timeout also causes refusal.
Watchdog state is only read; clock selection never enables or disables it.

Core selection rebuilds the clock tree, including the low-frequency generator,
and refuses enabled application generators beyond the three system generators.
Low-frequency selection refuses a running DFLL, a DPLL using XOSC32K or
generator 1, and other generators using OSC32K, XOSC32K or generator 1. A DPLL
using XOSC or generator 2 on OSC8M can continue. These checks are conservative:
USB-recovery DFLL operation and peripheral channels on unrelated sources also
block low-frequency changes. The GCLK inspection writes only register selector
IDs, without changing channel or generator configuration.

Stopped RTC, TC and TCC timers may retain their generic-clock connections
across selection. Every counter sharing the channel must have ENABLE and
SWRST clear and no synchronization pending. TC/TCC register bus gates are
opened temporarily for inspection and restored before returning; counter
configuration, pending flags and generic-clock connections are preserved.
A running shared-clock peer blocks selection even if its register bus was gated.

Timer Enable refuses a source frequency different from its cached BaseFreq.
After a successful clock change, call Frequency to select a rate using the new
source. This existing operation resets the counter, recalculates active trigger
periods and starts it. A missing source or failed reconfiguration returns zero.
A stop, setup, reset or start failure invalidates the cached source frequency,
so Enable refuses partial configuration even when the source is unchanged.
A successful Frequency retry restores the cached value. This does not claim
that hardware stopped when a stop synchronization failed; clock selection still
checks the counter registers before allowing a change. The final RTC read
synchronization after start or stop is also checked: failure suppresses timer
interrupt dispatch, invalidates the saved frequency and requires reconfiguration.
The period calculation uses an aligned local value before assigning the packed
64-bit trigger field. An unchanged source allows normal Disable/Enable resume.

DMA initialization leaves the controller enabled, but that
alone no longer blocks selection: every channel must be disabled, with no busy
or pending transfer, active arbitration level, completion awaiting its ISR, or
reset in progress. The inspection preserves CHID, descriptors and interrupt
flags. An idle DMA allocation can be reused after clock selection once its
peripheral is reconfigured. SysTick and scheduler
retiming remain the application's responsibility; reading SysTick CTRL here
would clear COUNTFLAG. The refusal checks do not retime peripheral drivers.

Validation with Arm GNU 15.2.1:

- Complete managed library source sets compile in Debug and Release:
  239/239 sources for D11 and 317/317 for D21.
- PowerOffDemo, UartPrbsTxTest and UartLoopback link for both targets and
  configurations. D21 TimerDemo and every D11 timer-family/runtime selection
  link, including the native and LTO checks.
- D11 LCDConsoleDisplayDemo links in all four interrupt/DMA combinations.
  With both enabled it uses 16,380 bytes of flash, leaving 4 bytes free.
- ClockDisplayDemo uses 16,244 bytes of flash and 2,892 bytes of static RAM,
  leaving 140 bytes of flash free. Its unused timer and transport checks pass.
- Core and low-frequency selector calls link against both target libraries.
- Existing UART, SPI, reset-reason, timer and Standby checks pass. The timer
  model includes failed final RTC synchronization after both start and stop.
- Temporary D11/D21 register checks at O0 and O2 cover clock readiness,
  failed GCLK reads, shared timer peers, all DMA channels, EIC last-user
  release, failed release/retry and SW0 polarity changes. Peripheral registers
  and synchronization are modeled; these are not hardware measurements.

Hardware clock switching, oscillator failure recovery, the latest wake-release
changes, D21 wake, full SRAM continuity and sleep-current measurements remain
pending. The prior D11 resume and SRAM-sample results above are retained as
separate hardware evidence.

## STM32WBA power port (0.14)

STM32WBA requires a family-specific power implementation. The WBA65 I
variant uses the `system_power_stm32wba.cpp` source linked into its Eclipse
managed MCU library. It implements reset and Standby wake reasons, GPIO
wakeup routing and polarity through PWR WUCR1/2/3, the TAMP backup registers,
and whole-bank SRAM1/SRAM2 Standby retention via PWR CR1 R1RSB1/R2RSB1.
The DK1 hardware reported 32 backup registers; a requested SRAM subrange
selects retention for its entire bank.

The dedicated PowerOffDemo reserves the SRAM test regions in its linker
script. Hardware validation on STM32WBA65I-DK1 (MB2143),
November 14, 2026: two consecutive PC13/WKUP2 Standby wake resets
(`SYSTEM_RESET_WAKE_PIN`, 0x100) passed. Backup-register startup counts
advanced from 1 through 3. Each wake preserved all 8192/8192 words in
the retained 32 KB SRAM2 test region and yielded 0/8192 unchanged words
in the deliberately non-retained SRAM1 test region.

The example disables USART1 and its DMA before Standby. Earlier tests
with an active UART did not wake; the exact peripheral/DMA/clock contribution
was not isolated. The simplified post-cleanup build still requires a
regression run. Bluetooth/radio and USB activity must be stopped by their
owners; active-stack power transitions are not validated. WBA G memory
variants and WBA2/WBA5 need separate port review.

Reference: ST RM0515 (STM32WBA6), chapter 11 Power control. STM32WBA5
uses RM0493 and has different wakeup and memory layouts.

## Nordic validation (0.14)

The Nordic HFXO reference-count and ownership regression was verified on
nRF52832 hardware: bare-metal HfxoTest 5/5, S132 SoftDevice HfxoTestBle 5/5,
and SDC/MPSL HfxoTestBle 5/5. The source-level clock test
`python3 tests/nordic/check_clock_hfxo_source.py` passed 5 tests. These
results do not cover BLE clock handover during stack initialization.

On the I-SYST IBK-NRF54L15, the Release PowerOffDemo was tested with
standalone System OFF / S1 GPIO wakeup (P1.08). RESETREAS mapped to
SYSTEM_RESET_WAKE_PIN (0x100), and GPREGRET boot counts increased across
successive wakeups. SRAM retention was verified in RAM1 for four successive
wakeups with 8192/8192 words preserved in the selected 32 KB section.
The subsequent non-retention comparison passed for three consecutive wakeups,
with 0/8192 words matching the original pattern in the other 32 KB section.
The two ranges start at 0x20020000 and 0x20028000.

System OFF validation must be done without an active debugger connection.
Repeated RESETREAS debug-interface (0x20) restarts were observed in a debug
setup; they are not counted as System OFF GPIO-wakeup validation, and their
precise trigger was not established. Run the Release firmware standalone and
monitor through UART. Do not add debug-reset suppression to the generic demo.

On the nRF54LM20 DK, Release PowerOffDemo completed three consecutive
System OFF / button wakeup cycles. Reset reason was
SYSTEM_RESET_WAKE_PIN (0x100), with GPREGRET counter increasing from 3 to 6.
The 32 KB retained RAM1 section at 0x20040000 preserved 8192/8192 words
each cycle. The separate non-retained RAM1 section at 0x20048000 had
0/8192 original words remaining each cycle. These results validate the
specified sections only, not all RAM retention modes.

On the nRF5340 Audio DK (PCA10121), the application-core Release PowerOffDemo
used UART RX P1.04, TX P1.05 and PLAY/PAUSE P0.04 for wakeup. Two
consecutive System OFF/wakeup cycles reported SYSTEM_RESET_WAKE_PIN (0x100).
The GPREGRET startup count advanced from 8 to 10. The 32 KB RAM1 region at
0x20040000 retained all 8192/8192 test words, and the separate 32 KB region
at 0x20048000 retained 0/8192 original words, after each wakeup. The example
uses P0.04 because the earlier P0.02 VOL- configuration read as asserted
when the button was not pressed. Temporary pin diagnostics were removed
after validation. The nRF5340 network-core power path was not exercised.

For nRF9151/nRF9161 the current PowerOffDemo is a bare-metal GPIO/UART
application: it does not initialize the Nordic modem library, and its Eclipse
project does not link a modem library. Consequently no modem shutdown call
is needed for this example. Other applications which initialize the modem
must first stop cellular operation and shut down the modem through their
own modem interface before SystemPowerOff. The nRF91x1 DK demo uses UART0
RX P0.26 / TX P0.27 and wake button P0.08. It should be tested standalone
in Release without an active debug session.

The nRF91 RAM linker region is shared with application data, stack and
optional modem shared-memory sections; unlike the tested nRF54 targets
it does not reserve a separate free RAM1 region. Do not reuse the nRF54
fixed RAM test addresses on nRF91 without a dedicated linker reservation.

The nRF91x1 PowerOffDemo was hardware validated on the reported DK setup:
three successive GPIO wakeups from System OFF, with reset reason
SYSTEM_RESET_WAKE_PIN (0x100) and GPREGRET start counter increasing from
3 to 6. This validates bare-metal System OFF, button wakeup and retention
registers for the tested configuration. It does not validate SRAM retention,
modem-active shutdown, or the independent nRF9160 variant.

Outstanding family-specific build checks and the nRF5340 network-core
power limitations still require review.

## SAM4E

`system_power_sam4e.cpp` implements the power functions. `SystemInit` captures
reset/wake status before a clock read can clear SUPC status. Reset reasons are
cached. Wake pins use the WKUP0..15 assignments in datasheet tables 10-2/10-3.
The application must configure a pin as a GPIO input; analog or capture uses of
the same pin must be stopped first. Both high and low wake levels are supported.

Backup mode switches off the core regulator and all SRAM. It keeps 20 GPBR
words while VDDIO remains supplied. `SystemRamRetain` returns false because
SRAM retention is not available in that mode. GPBR writes are checked by readback;
tamper protection may prevent a write.

`system_clock_sam4e.cpp` implements slow-clock selection and oscillator
initialization. RC is 32 kHz; crystal/bypass is 32768 Hz. Switching to the external
source is irreversible until the backup supply resets. A request to switch back
to RC or change between crystal and bypass after selection returns false.
Startup waits are bounded. On timeout, the requested configuration is not
reported as active; a late oscillator start can still occur in hardware.

RTC/RTT drivers own their alarm configuration. Power off preserves it. The reset
reason reads their alarm status only for Backup exits with the corresponding
wake source enabled. No GPBR word is reserved by this implementation.

Reference: [Microchip SAM4E datasheet](https://ww1.microchip.com/downloads/en/DeviceDoc/Atmel-11157-32-bit-Cortex-M4-Microcontroller-SAM4E16-SAM4E8_Datasheet.pdf),
sections 10, 16, 18 and 19.

Run `python3 tests/sam4e/run_power.py` for register-model tests, production-header
syntax checks and Eclipse source-link checks. These do not measure power, execute
a target firmware image, or validate physical wakeup. Those checks remain to be
performed on hardware.

Arm GCC 15.2.1 target checks passed on November 15, 2026:

- `python3 tests/microchip/compile_power.py`: all six SAM4E variants compile
  with the production CMSIS headers; every typed peripheral address is checked.
- `python3 tests/sam4e/build_examples.py`: SAM4E16E Blinky, UartRetargetDemo
  and UartPrbsTxTest compile and link in Debug and Release (six builds).
  The check uses selected library sources and the managed projects' source
  links, defines and linker scripts. It verifies that each image includes the
  SAM4E clock and reset implementations. It does not run IOcomposer or hardware.

Main-clock selection in `system_sam4e.c` switches MCK to SLCK before changing
MAINCK or PLLA. Prescaler and source writes are separate and each waits for
MCKRDY. All ready waits are bounded. A failed oscillator or PLL start leaves
MCK on SLCK; a failed MCK transition returns false and the getter reports the
register-selected frequency, which cannot confirm that the hardware stabilized.
Flash remains at five wait states so a late transition cannot outrun flash.

RC requests choose the nearest 4, 8 or 12 MHz setting (ties choose the lower
rate). External crystal inputs must be 3..20 MHz; bypass inputs supported by
this PLL configuration are 3..50 MHz. The supplied frequency describes the
physical input. PLLA chooses the highest legal output up to 240 MHz and MCK
uses half that rate. When USB clock support is requested, PLLA must provide an
exact multiple of 48 MHz and its USB divider is configured accordingly.
The USB driver still owns enabling the USB clock.

Clock changes are refused while USB or a programmable clock is enabled, or PMC
write protection is active. Applications must stop other peripheral transfers
before a runtime change and reconfigure their baud rates and timer periods
afterward. Runtime selection does not reinitialize the watchdog or slow clock.
Peripheral clock index 0 is MCK; other indices return 0.
The main-clock register model checks repeated selections, transition ordering,
PLL limits, bypass, RC rounding, USB division and ready timeouts at O0 and O2.
See datasheet sections 28, 29 and 46 for the clock tree and electrical limits.

## SAM4L

`system_power_sam4l.cpp` implements reset reasons, Backup entry, EIC wake pins
and four 32-bit backup registers. Backup powers down both SRAM banks, so
`SystemRamRetain` returns false. Ordinary Retention sleep is not used for
`SystemPowerOff`, since it resumes execution instead of resetting the core.
Stop DMA and peripheral transfers before calling power off. Existing AST and
other backup wake enables are preserved; their drivers own their configuration.

SAM4L startup leaves the watchdog disabled; applications enable it for their
own use case. Runtime core-clock selection preserves that application choice.
Low-frequency clock selection returns false on an oscillator-ready timeout,
restores the incoming interrupt mask, and updates the oscillator descriptor
only after success. Repeating the same request checks readiness again.
The descriptor records the last successful selection, not a measurement of
clock health. A failed external-mode change may leave that oscillator stopped.
Startup still halts when its required clock cannot be established. Runtime
main-clock selection now returns false on oscillator, PLL, DFLL, divider or
flash setup failure and attempts to leave the CPU on RCSYS. The clock getter
reports the register-selected source and divider after the attempt. A failed
hardware switch cannot be treated as a confirmed stable clock.

Runtime selection preserves the watchdog, low-frequency clock, cache settings
and voltage scale. It chooses dividers within the existing voltage scale
instead of requesting a new power transition. Startup retains its checked
power transition in SRAM and stops if its required clock setup fails.
The caller must still stop clock consumers before changing the main clock
and reconfigure their rates afterward. Runtime changes are refused while a
UART, SPI or I2C peripheral clock is enabled, USB is enabled (including suspended USB), or any PDCA channel is
enabled. USB or PDCA with its HSB clock enabled but register clock gated is
also refused. Application generic clocks must be disabled; GCLK0 remains the
system DFLL reference, and GCLK7 is treated as system-owned only when
`bUSBClk` is set and USB is disabled. Refusal leaves hardware registers and
oscillator descriptors untouched and restores the incoming interrupt mask.

Enabled analog, audio, LCD and other PBA peripheral clock gates also refuse
main-clock changes. The AST must be stopped with no synchronization pending.
For TC timers managed by IOsonata, the timer port reports running/unhealthy
state without reading TC status, so rollover and compare flags are preserved.
A TC block with no IOsonata timer owner is refused while its PBA clock is on.
Applications using TC registers directly must stop and gate that block before
changing the clock; mixing direct channel control with Timer objects in the
same block is not covered by the ownership query.

Disable every active timer before changing the clock. After a successful
change, TC `SetFrequency` reads the current PBA frequency and recalculates its
divider and trigger periods. It resets and starts the counter as before. The
optional timer query does not pull timer drivers into a clock-only image.
Core SysTick/scheduler timing remains the application's responsibility.
Low-frequency selection refuses a running or synchronizing AST, enabled
PICOUART, an LF-selected watchdog, enabled LCD/CATB clock gates, an active
frequency measurement, or any generic clock using OSC32K, RC32K or the 1 kHz
output. Both sampled brownout monitors are conservatively treated as busy;
continuous brownout detection and a watchdog using RCSYS may remain enabled.
The selector does not disable or reconfigure these users.

The BPM/BSCIF, AST, WDT, PICOUART, SCIF and FREQM register clocks must be
accessible for these checks. If one is gated, the request is refused rather
than temporarily enabling clocks or assuming its peripheral is stopped.
These gates are enabled by the reset configuration. Refused calls preserve
all registers, pending flags, oscillator descriptors and the interrupt mask.
After a permitted source change, AST SetFrequency refreshes its source rate
before recalculating its prescaler and periods.

Repeating a request rechecks the hardware setup; a matching descriptor alone
does not report success.
Run `python3 tests/sam4l/run_clock.py` for the Linux register-model checks.

Wake pins use asynchronous high/low level detection through the backup mux.
Configure the selected pin as a GPIO input first. Toggle detection is not
supported. An EIC line already enabled by another driver is refused. Wake
configuration does not install an ISR or enable NVIC delivery.

| Package | Backup wake pins |
| --- | --- |
| A | PA04, PA05, PA06, PA07 |
| B | A pins plus PB01 |
| C | B pins plus PC03, PC04, PC05, PC06 |

Both SAM4LC and SAM4LS use these mappings. Alternative EIC pin routes cannot
wake from Backup. Disabling a pin removes its backup mux and EIC enable while
preserving the shared EIC wake enable for other users.

The cause registers remain readable until the next reset/wakeup. SYSRESETREQ
is reported by hardware as OCDRST and maps to `SYSTEM_RESET_SOFT`; software
cannot distinguish a debugger-issued SYSRESETREQ. Backup EIC and AST wakeups
map to the generic pin and timer reasons. PICOUART wakeup has no generic reason
bit and is not misreported as USB or GPIO.

Run `python3 tests/sam4l/run_power.py`. It checks production headers for all
18 C/S, density and package combinations, runs host register tests for all six
package/family combinations at O0 and O2, and checks both Eclipse library links.
These are not Arm firmware builds or hardware power/wakeup validation.

Arm GCC 15.2.1 also compiled the power and clock sources for all 18 SAM4L
variants with `python3 tests/microchip/compile_power.py`. The SAM4LS8C
peripheral check linked 20 examples in each of Debug and Release, including
UsbCdcLoopbackTaktOS, using TaktOS commit
`86002a6c3fee280ac0e978283919630f99e3b53d` from
[IOsonata/TaktOS](https://github.com/IOsonata/TaktOS). Run it with
`python3 tests/sam4l/peripheral_build_test.py --taktos ../TaktOS`.
These 40 builds use selected library sources; hardware execution is unverified.
The 18-variant compilation and all 40 example links passed again after the
runtime failure-handling change. The existing clock register checks also passed
for SAM4LC8C and SAM4LS8C at O0 and O2, including failed oscillator/PLL/DFLL
startup, retry, flash readiness, interrupt-mask restoration and WDT preservation.
The low-frequency ownership checks cover every generic-clock index and verify
register preservation on refusal. The timer model checks AST and TC retiming.
All seven TimerDemo selections also compile and link in Debug and Release.

Reference: [Microchip SAM4L datasheet 42023H](https://ww1.microchip.com/downloads/aemDocuments/documents/OTH/ProductDocuments/DataSheets/Atmel-42023-ARM-Microcontroller-ATSAM4L-Low-Power-LCD_Datasheet.pdf),
sections 6.1.3, 10.7.15, 11.5, 11.6.9..11, 12.7.18 and 21.6.

## LPC11Uxx power and clock port

The LPC11U35 library implements the existing generic power functions using
UM10462 chapters 3 and 4. `SystemPowerOff` enters Deep power-down and discards
pending CPU interrupts. PIO0_16 wakes the chip on a falling edge and starts a
reset. The pin needs an external pull-up; the RESET pin is inactive in this
mode. `SystemWakePin` accepts only that pin and polarity. It returns false for
disable because the hardware wake input cannot be disabled. Applications must
not set the write-once PCON NODPD bit if they intend to use Deep power-down.

`SystemResetReason` caches the first read and clears the recorded flags.
DPDFLAG identifies wakeup rather than a cold power-on, even though wakeup
internally generates POR. Four full-width GPREG words survive Deep power-down
and software reset. GPREG4 is left to the application because it also contains
wake-pin hysteresis control. SRAM retention in Deep power-down is unsupported.
`SystemSleep` currently uses ordinary Sleep, preserves RAM and peripheral
clocks, and wakes from an enabled NVIC interrupt. Deep-sleep and Power-down
with clock restoration are not implemented. `SystemWakePin` selects the
Deep power-down input only; ordinary Sleep uses the normal interrupt drivers.

Runtime core-clock requests refuse enabled timer, SSP, UART, ADC, USB and WWDT
clock gates, an enabled I2C controller, or CLKOUT. Stopped timers whose gates
remain enabled are conservatively refused. These checks apply to the legacy
setters as well as `SystemCoreClockSelect`; startup configures the clocks before
drivers take ownership. Applications must stop and reconfigure SysTick and
scheduler timing around an accepted runtime change. Clock decoding reports
zero for powered-down sources or an unlocked PLL. The existing fixed 48 MHz
CPU target and bounded PLL recovery to 12 MHz IRC remain unchanged.

`SystemLowFreqClockSelect` selects the closest nominal watchdog-oscillator
rate for an RC request. Actual nominal frequency and 40-percent oscillator
accuracy are recorded in `g_McuOsc.LowPwrOsc`. A running WWDT, its enabled bus
clock, a main-clock WDO selection, or a WDO CLKOUT selection prevents changing
it. This does not enable or feed the watchdog. The default LF descriptor has
frequency zero, so startup leaves WDO unconfigured; an application-supplied
nonzero LF descriptor is applied during startup.

The managed `PowerOffDemo` project links the existing generic demo, with UART
DMA disabled for LPC11U35. Validation includes all 115 enabled library source
files in Debug and Release, TimerDemo, UART PRBS TX and loopback, SPI loopback,
I2CScan and PowerOffDemo links in both configurations, existing clock tests in
C and C++, and temporary power-register checks at O0/O2. These are compilation
and register-model results; sleep entry, wake reset, retention and current draw
still need hardware validation. D11 remains the current hardware test target.

## LPC17xx power and clock port

The LPC1769 managed library includes reset reporting, Deep Power-down,
ordinary interrupt-resumable Sleep and five full-width RTC backup registers.
RTC register access temporarily enables PCONP.PCRTC and restores the previous
gate state. Backup contents survive software reset and Deep Power-down while
the VBAT domain remains supplied. SRAM cannot be retained in Deep Power-down.
The watchdog is neither enabled nor fed by these functions.

Deep Power-down wakes through the RESET pin or an application-configured RTC
interrupt. `SystemPowerOff` preserves the RTC interrupt enable and pending
state, stops SysTick, and disables other CPU interrupts before entry. The
application must arm a future RTC event and handle any already-pending event
before entering. `SystemWakePin` returns false because there is no selectable
GPIO wake input for this mode. `SystemSleep` uses ordinary Sleep and enabled
NVIC interrupts; Deep Sleep and Power-down with PLL restoration are not yet
implemented.

`SystemResetReason` caches and clears RSID and DPDFLAG on the first call.
It reports power-on/brownout, external reset and watchdog reset. After Deep
Power-down it reports reset-pin wake when EXTR is set, otherwise RTC wake when
an RTC interrupt flag is present. It leaves RTC interrupt flags untouched.
RSID has no dedicated software-reset or lockup flag, so these are not inferred.
Read the reset reason before application RTC initialization clears wake flags.
With a separately supplied VBAT domain, retention and DPDFLAG can outlive a
loss of core supply; the registers do not identify every such power sequence.

Runtime core-clock changes refuse enabled peripheral gates except RTC and
GPIO, active PLL1, CLKOUT, a PCLK-clocked running watchdog, and the DAC counter.
The gate check is conservative: powered but idle peripherals must also be
released. Startup configures clocks before application drivers own them.
The divider/PLL search remains outside the interrupt lock; register changes
and ownership checks run under the lock. The closest-frequency calculation,
100/120 MHz device limits and bounded PLL failure handling remain in place.
Applications must stop and reconfigure SysTick and scheduler timing around a
runtime change. Peripheral-clock setters remain caller-controlled because
drivers use them while configuring their own peripherals.

The port refuses an oscillator-range change while MAIN feeds the CPU. Switch
to IRC first; changing the range of an unused oscillator stops and restarts it
with bounded ready checks. Other SCS fields are preserved. PLL1 configuration
refuses an enabled USB peripheral. The RTC has a fixed 32768 Hz external clock;
`SystemLowFreqClockSelect` returns false because there is no programmable LF
source selector. This does not stop or reset the RTC.

Validation covers all 124 enabled library sources in Debug and Release,
TimerDemo, PowerOffDemo, UART PRBS TX/loopback, SPI loopback, I2CScan and the six
USB demos in both configurations. Clock register tests cover C/C++ callers,
LPC1758/1759/1768/1769 limits, oscillator/PLL failures and busy-user rejection.
Temporary power-register checks cover reset caching, backup access, interrupt
selection and sleep register restoration at O0/O2. USB project source links
and four C++ event initializers were corrected; USB_IRQHandler now has the C
linkage required by the vector table. Hardware sleep/wake, retention and current
measurements remain pending. D11 remains the current hardware test board.

## LPC546xx power and clock port

Both LPC54605 and LPC54628 managed libraries include the target power unit.
It caches and clears SYSRSTSTAT on the first reset-reason query, implements
ordinary interrupt-resumable Sleep, and enters reset-on-wake Deep power-down.
The latter follows NXP libpower's POWER_EnterDeepPowerDown sequence: SCR deep
sleep, PDRUNCFG1 = 0x300002AF, PDRUNCFG0 = 0xFEFFFFF7, then WFI. These full
register values include reserved bits required by the vendor sequence. No
vendor binary is linked. Reference: UM10912 power management and power API
chapters, and [NXP MCUXpresso LPC54628 libpower_hardabi.a](https://github.com/nxp-mcuxpresso/legacy-mcux-sdk/blob/main/devices/LPC54628/gcc/libpower_hardabi.a),
Git blob d58823378ee47b14321d21e662696dd6a7bc578a.

Deep power-down leaves only the RTC domain and RESET wake input. RTC CTRL
alarm/countdown wake configuration is preserved. Applications must arm a
future RTC event and handle existing flags before entry. CPU interrupts and
SysTick are stopped; ordinary Sleep preserves their configuration and restores
the incoming SCR and interrupt mask. Deep-sleep clock and voltage restoration
is not implemented. No watchdog is enabled or fed by these functions.

There is no selectable GPIO wake input for Deep power-down, so SystemWakePin
returns false. SRAM retention is unsupported. Seven full-width backup words
alias the RTC driver's GPREG0..6; applications must coordinate their use.
GPREG7 remains the RTC time-valid marker. Backup access temporarily enables the
RTC bus clock and restores its prior state without resetting the RTC.
Reset reporting uses only the hardware POR, BOD, EXTRST, WDT and SYSRST flags.
It does not infer a Deep power-down wake cause from a possibly stale RTC flag.

Runtime core selection refuses peripheral clock gates other than memory,
GPIO/pin routing and RTC, active audio/USB PLLs, or CLKOUT. This is conservative:
a stopped peripheral with its gate enabled must still release it. The PLL
search precedes the interrupt lock; the ownership check and transition are
atomic against interrupts. Startup uses the same setup without application
ownership checks. Applications must stop and reconfigure SysTick and scheduler
rates around a runtime change. An external oscillator is stopped before its
mode/range is changed. Failed PLL setup leaves the core on the 12 MHz FRO.

Clock decoding rejects powered-down FRO, system oscillator and PLL sources,
unlocked PLLs, disabled FRO high-frequency output, and invalid PLL divisors.
The RTC's system-clock output must be enabled to be decoded. This is register
state, not an oscillator-health measurement. The LF descriptor now describes
the fixed 32768 Hz crystal; SystemLowFreqClockSelect returns false, and the RTC
driver retains ownership of oscillator startup and verification.

The new LPC54628 PowerOffDemo uses the existing generic example and VCOM UART.
It wakes through hardware RESET; P0.4 cannot wake Deep power-down. All 179 LPC54628 and 32 LPC54605 managed library sources compile in Debug
and Release. The LPC54628 timer, peripheral and USB checks pass 60 firmware
links across those configurations. Existing register tests cover clock ownership, failed PLL setup,
RTC, timers, transfers, storage, audio and other peripherals. Temporary power
register checks ran at O0/O2. These do not establish hardware wake behavior,
backup retention or current draw. D11 remains the current hardware test board.


## RA4M1 power and clock port

RA4M1 has Sleep, Software Standby and Snooze, but no Deep Software Standby.
The shared vendor header includes registers belonging to other RA variants;
the port does not use those as evidence of RA4M1 capabilities. There is no
SystemPowerOff implementation. The existing generic PowerOffDemo uses its
resume path through SystemSleep, with P000/IRQ6 and SCI0 P100/P101.

SystemWakePin uses the existing GPIO interrupt allocator. Falling and rising
edges are supported, with the IRQ filter disabled. A pin, IRQ line or event
route already owned by GPIO, DMA/DTC or another wake source is refused.
Disabling the owned pin releases its WUPEN bit and interrupt route. Ordinary
GPIO interrupt release also removes a wake bit owned by this port. Unrelated
WUPEN bits and pin routes are preserved.

SystemSleep uses Software Standby when external IRQ wake is configured and
an associated NVIC interrupt is enabled, DMA/DTC are stopped, Snooze is off,
and oscillator-stop detection is disabled. Otherwise it uses ordinary Sleep.
Other Software Standby wake sources are not yet handled by this implementation.
Active SysTick/PendSV pending work returns without sleep so it can be serviced.
Applications must stop incompatible peripheral transfers before entry.

The Standby entry follows procedure B of Renesas technical update
[TN-RA*-A0146A/E](https://www.renesas.com/en/document/tcu/note-about-interruption-during-transition-low-power-modes-0):
disable SysTick interrupt generation and non-wake NVIC delivery, then DSB/WFI.
Event routes and pending IRQ flags are preserved. After resume, restore SBYCR,
PRCR, SCR, NVIC enables and SysTick control before restoring PRIMASK. SBYCR
bit 14 is always written as one. The port does not touch unsupported DPSBYCR.
Software Standby retains SRAM and resumes execution; it does not generate a
reset reason. Reset-on-wake RAM retention and generic retention words report
unsupported. Battery backup memory integration remains outside this port.

Reset reporting caches the first read of RSTSR0/1, then clears their recorded
flags. POR and voltage-monitor resets map to POWERON, WDT/IWDT to WDT, and
SWRF to SOFT. RES clears all reset flags, so an all-zero first read maps to PIN.
Read this API before other code clears reset flags. Memory/MPU/stack errors
have no exact generic reason bit and are not mislabeled as CPU lockup. RSTSR2
cold/warm history is left untouched. Startup option words keep WDT and IWDT
auto-start disabled; sleep and clock selection do not feed or start them.

Runtime core-clock changes now refuse enabled peripheral module gates,
DMAC/DTC's enabled module gate, and CLKOUT before unlocking protected registers.
The SRAM gates and MSTPC0 clock-domain policy are excluded from that test.
This is conservative: a stopped but ungated peripheral still prevents a change.
Startup bypasses the application-ownership check. The existing oscillator
plans, bounded ready waits and protection/interrupt restoration are retained.
LF selection starts LOCO or SOSC without stopping the other oscillator or
resetting the RTC; the peripheral chooses its own LF source. Callers still
own SysTick/scheduler retiming after a successful core-clock change. Module-gate
checks do not track a watchdog started directly through registers; applications
must not change its PCLK timing while it is running.

All 305 enabled library sources compile in Debug and Release. All 19 managed
demo projects link in both configurations, including PowerOffDemo. GCC host
checks run the existing clock and GPIO models at O0/O2 with undefined-behavior
checks, including wake ownership and runtime busy refusal. The existing timer
model also passes at O0/O2. UART passes at O0/Os/O2 and with address/undefined-
behavior checks (1,167,463 interrupts per run). I2C and SPI models pass their optimization and
address/undefined-behavior checks, with leak detection disabled because the
workspace runs under ptrace. Temporary power
register checks cover reset caching, mode selection, pending work, interrupt
masking and register restoration at O0/O2. The original all-in-one validation
runner additionally requires Clang/LLD, which are not installed in this workspace;
its separate Clang ELF probes were not run. Hardware wake, current draw and
oscillator restoration remain pending; D11 is still the hardware test target.

Reference: RA4M1 hardware manual R01UH0887EJ0110, chapters 5, 8, 10 and 13,
and the RA4M1 feature definitions in Renesas FSP.


## R9A02G021 generic power and clock integration

SystemResetReason caches RSTSR0/1 before clearing recorded flags. Power-on and
voltage-monitor resets map to POWERON, watchdog resets to WDT and software
reset to SOFT. An all-zero status maps to PIN following the manual's reset
source flow. SRAM parity/ECC and bus faults have no matching generic reason
bit and are not reported as CPU lockup. RSTSR2 cold/warm history is preserved.

SystemWakePin uses the existing GPIO/ICU allocator, supports falling and rising
edges, and refuses owned pins, wake bits and Snooze requests. Releasing the
owned GPIO interrupt also releases its wake bit. It does not change another
source's WUPEN bits. Watchdogs remain application-controlled and stopped by
the default option words.

SystemSleep uses the existing target sleep implementation. Software Standby
is selected only with DTC stopped, Snooze unconfigured and an enabled wake
interrupt. Other configurations use Sleep. Enabled pending CLIC/ICU work
returns immediately. In Standby the generic wrapper masks non-wake CLIC
interrupts and restores their enables afterward without clearing pending flags
or routes, following procedure B of
[TN-RIS-A0001B/E](https://www.renesas.com/en/document/tcu/note-about-interruption-during-transition-low-power-modes-1).
Call it from normal application context after stopping incompatible transfers.
There is no reset-on-wake mode or generic retention storage on this part.

The existing explicit R9a02LowPowerEnter Snooze path is retained for the
UART, SPI and ADC demos. It is not covered by the generic wrapper's interrupt
masking. Its DTC/Snooze transition still needs a separate erratum review;
passing the existing emulator does not validate that hardware transition.

Core and peripheral clock changes refuse enabled module gates, CLKOUT,
Snooze and flash programming before protected writes. Startup bypasses the
runtime module check. A stopped peripheral with its module gate enabled still
prevents retiming. Direct-register watchdog use is not represented by module
gates; callers must not retime a running PCLK watchdog. Machine-timer and
scheduler handling remain application-owned.

LF selection no longer stops an RTC or silently substitutes the requested
source. A source change is refused while RTC, UARTA, TML32, REMC, CLKOUT,
Snooze or a core using SOSC owns the clock. Startup preserves a running RTC's
current source. Clock getters return zero for stopped sources and reserved
dividers; HOCO reporting also requires the stable flag. This does not measure
external clock presence or crystal stability.

PowerOffDemo uses the shared example's resume path on FPB-R9A02G021 with SW1
and the existing UART pins. RE01 remains a separate task: its startup still
needs boost/PLL failure-path cleanup and generic power integration.

Validation uses xPack RISC-V GCC 15.2.0. All 196 managed library sources
compile in both configurations, including Release LTO. Direct builds using
project source links, defines and linker flags pass 34 of 36 demo configurations.
All 18 Release configurations pass. CryptoTest Debug and DfuBoot Debug overflow
flash; both also fail with the previous commit's sources. I2CScan Debug's
forced semihost syscall archive caused duplicate stddev definitions and is now
linked without whole-archive.

Temporary checks execute the compiled RISC-V code at O0 and Os for reset
caching, clock refusal, divider changes, wake ownership/release, CLIC masking,
pending work and Sleep fallback. Existing UART/Snooze, I2C and SPI slave
emulators pass at O0 and Os. These are compiler/linker and register checks,
not an Eclipse headless build or hardware power validation. D11 remains the
current hardware target.

## RE01

The RE01_1500KB library now includes `system_power_re01.c` in all six
DBN/CFB/CFP Debug/Release configurations. Reset reporting caches and clears
RSTSR0/RSTSR1 on the first call. Deep Software Standby wake flags identify
pin and CCC wake. Startup disables the previous boot's wake configuration
and clears the Deep Standby mode selection without clearing those wake flags.
No retention registers or reset-on-wake RAM retention are exposed.

`SystemSleep()` uses Software Standby only in normal ALLPWON mode, with Snooze
disabled, the DMAC/DTC module stopped, and an enabled external IRQ wake route.
It selects ALLPWON -> EXFPWON -> ALLPWON and retains all eight SRAM banks.
Boost, other power-domain modes, Snooze, enabled DMAC/DTC, or other wake policies
use ordinary Sleep. Both paths preserve the caller's interrupt mask, register
protection, sleep settings and pending interrupts. Drivers still own stopping
peripherals before Standby. The port does not enable a watchdog.

`SystemWakePin()` uses the existing GPIO interrupt allocator for IRQ0 through
IRQ9. It accepts rising or falling edges on the fixed interrupt pin mappings.
An application GPIO handler or a pin already used for another function is not
replaced. Reconfiguring the same wake pin changes its edge; disabling it
releases only its own route. Disable wake before changing that pin's function.
On the four dedicated Deep Standby pins, it also reserves the corresponding
DPSIER0/DPSIEGR0 setting, preserving unrelated wake sources. Other IRQ pins
remain usable for `SystemSleep()` only.

| Deep Standby source | Pin |
| --- | --- |
| IRQ0-DS | P4.11 |
| IRQ1-DS | P2.7 |
| IRQ2-DS | P4.10 |
| IRQ3-DS | P4.9 |

P4.11 and P2.7 share debug functions and are refused while debug is enabled.

Startup no longer uses `goto`. Core-clock changes share the startup sequence
and preserve a live MOCO fallback when oscillator setup fails. Frequency
reporting returns zero for stopped or unready sources and reserved dividers.
Runtime core/divider changes refuse enabled peripheral modules, CLKOUT and
flash programming. Startup does not apply that runtime module check. Core RC
and divider requests select the nearest supported frequency; PCLKB remains at
or below ICLK and 32 MHz. External oscillator descriptors must describe a real
8-32 MHz input. Applications own retiming SysTick and software delays after a
runtime clock change. Selecting an internal core clock at 32 MHz or below
now exits Boost through MOCO, with PLL and HOCO stopped before the transition.
Changing only a divider does not exit Boost.
The LF selector starts LOCO or SOSC without stopping a source used by RTC/AGT;
TCXO selection is unsupported.

The new `PowerOffDemo` has six package/configuration targets and uses the
shared resume-on-interrupt demo, including static and stack RAM samples.
Its application clock configuration selects the internal 32 MHz oscillator
at startup, avoiding Boost. The UART uses the existing P2.2/P2.3 SCI4 mapping
at 1 Mbaud. The wake pin is P4.10 (IRQ2/IRQ2-DS), connected to SW3 on the
EK-RE01 1500KB. Other boards can use a normally-open button from P4.10 to
ground. Check the board/package pin availability before use.

The demo defaults to resumable Standby with static and stack RAM samples.
Set `POWER_DEMO_RESUME_WAKE=0` in the application compiler definitions to
exercise reset-on-wake Deep Standby instead. Both forms use the same project
and shared demo source. Since this part has no exposed retention registers,
the reset-on-wake demo's start count is not retained.

`SystemPowerOff()` disables SysTick/NVIC delivery, exits Boost if needed, and
enters Deep Software Standby with I/O state retained. Dedicated wake sources
reset the MCU; the function never resumes the application. If debug is enabled
or a required transition fails, it stays stopped until reset; Deep Standby
is not entered. Debug-mode Boost transitions are also refused
by the clock helpers. The application owns quiescing its devices before power
off, as shown by the demo's UART drain and disable.

For early WS1 silicon, compile the RE01 library with `CHIP_VERSION_WS1`.
The vendor restrictions require an application-configured CCC wake source for
Deep Standby; without it, `SystemPowerOff()` stays stopped until reset. The
port does not enable CCC or a watchdog on behalf of the application.

The transition sequence follows Renesas' RE01 driver package:
[`r_lpm_api.c`](https://github.com/renesas/re-driver-package/blob/master/SDK_RE01_1500KB/RE01_1500KB_DFP/Device/Driver/Src/r_lpm/r_lpm_api.c)
and
[`r_system_api.c`](https://github.com/renesas/re-driver-package/blob/master/SDK_RE01_1500KB/RE01_1500KB_DFP/Device/Driver/Src/r_system/r_system_api.c).
Pin assignments follow Renesas datasheet R01DS0363EJ0110, tables 1.5-1.7.
Hardware validation remains pending for wake/reset behavior, I/O retention,
Boost transitions and sleep current.

## ESP32-C3/C6 CPU sleep and clock readout

The C3 and C6 libraries now provide SystemSleep using CPU WFI. It keeps the
peripheral clocks, SRAM and stack intact. Interrupt delivery is masked around
WFI and restored to its previous state after wake. The GPIO driver implements
SystemWakePin with the existing interrupt registration and dispatch functions.
It accepts rising or falling transitions, refuses pins already owned by another
callback, and releases only its own callback. C6 pads assigned to LP IO are
refused. This mode does not configure the RTC or PMU sleep state machine.

SystemResetReason reads the last hardware reset cause without clearing it.
Power/brownout, software, watchdog and JTAG causes map to generic flags. C3's
power-on code also covers chip-level brownout and super-watchdog events; the
hardware code does not distinguish them. Unmapped reset causes return zero.
Deep-sleep wake causes are not decoded in this batch.

C3 clock decoding now applies PRE_DIV_CNT to XTAL and RC_FAST and returns zero
for reserved clock selections. Its peripheral clock 0 is the APB clock, limited
to 80 MHz. C6 peripheral clock 0 follows the root, AHB and APB dividers instead
of returning CPU frequency. C6 RC_FAST uses the nominal 20 MHz rate; a reserved
root returns zero without dividing by zero in the delay-period calculation.
Both ports reject invalid peripheral clock indices.

The C6 startup CPU-frequency option no longer requires ESP32_PLL_FREQ. When
ROM left PLL selected, it changes only the CPU divider, accounting for the
fixed root divide-by-three and the separate 120 MHz selection. AHB, APB, flash
and peripheral clock selections remain unchanged. A non-PLL boot configuration
is preserved. Default startup still preserves ROM clock selection. Watchdogs
remain disabled during startup and are not changed by SystemSleep.

PowerOffDemo managed projects are provided for C3 and C6, using UART0 at 1 Mbaud
and the GPIO9 BOOT button. They use the generic resume path and static/stack SRAM
sample check. Release BOOT before resetting the board. The generic demo prints
"Standby"; on these targets the implemented mode is CPU WFI only.

Runtime core/low-frequency oscillator selection and peripheral-clock setters
still return failure. SystemPowerOff is not implemented. No retention registers
are exposed because RTC scratch registers are used by ROM and clock setup;
SystemRamRetain returns false. PMU Light-sleep, Deep-sleep, RTC/LP wake sources,
retention and runtime oscillator ownership remain work to complete before
calling the ESP32 power and clock port complete.

Validation for this batch:

- All 200 C3 and 202 C6 managed-library translation units compile in both Debug
  and Release with RISC-V GCC 15.2.0.
- Both new PowerOffDemo projects link in Debug and Release using their managed
  source and linker settings. Release text/data/bss: C3 11137/480/1420 bytes;
  C6 11395/480/1600 bytes.
- 24 existing ESP32 Python checks pass, including extended C3/C6 clock MMIO
  checks at O0 and Os.
- GPIO interrupt/wake ownership and input-routing checks pass for all five
  C-series variants at O0 and O2. C2/C5/C61 startup clock checks also pass.
- The existing serial-example build script links 80 Debug/Release example
  configurations across C2/C3/C5/C6/C61.
- Temporary C3/C6 MMIO checks verify reset mapping, repeated reads, unsupported
  retention results and preservation of the interrupt mask around CPU sleep.
- No board execution or sleep-current measurement was performed. Hardware
  testing remains on SAMD11 until the maintainer selects another board.

Register references are Espressif ESP-IDF v5.5.1 per-chip clk_tree_ll.h,
reset_reasons.h, system_reg.h, pcr_reg.h, rtc_cntl_reg.h and lp_clkrst_reg.h.

## ESP32-C3/C6 startup clock preservation and ROM timing

C3 startup previously selected PLL regardless of the root left by ROM. It now
changes only the CPU tap when PLL is already selected. XTAL, RC_FAST and their
dividers are preserved. The default PLL tap remains 80 MHz, with
ESP32C3_CPU_160MHZ selecting 160 MHz. Startup does not power up, calibrate or
stop a PLL.

Both C3 and C6 startup now read back the resulting CPU frequency and update
IOsonata's cycle-delay values immediately. They also call the target ROM
ets_update_cpu_frequency entry so later ROM delays use the same CPU MHz. The
C3/C6 linker scripts supply the respective ROM addresses as Esp32RomClockUpdate.
Invalid clock selections do not overwrite the ROM timing value. Clock getters
remain read-only with respect to ROM timing state. Fractional MHz values use
the nearest integer supported by the ROM interface.

This follows the existing startup ownership rule: ROM initializes the crystal,
PLL and flash interface; IOsonata changes the supported CPU tap without
reconfiguring the bus or peripheral roots. Watchdogs remain disabled by startup.

Validation:

- All 200 C3 and 202 C6 library sources compile in Debug and Release.
- The default and 160 MHz PowerOffDemo variants link in both configurations
  for both chips, eight images total. Their ROM symbols resolve to 0x40000588
  on C3 and 0x40000048 on C6, matching the Espressif v5.5.1 ROM linker files.
- 24 ESP32 Python checks pass. The clock model covers C3 default/160 MHz
  startup, C6 CPU-divider choices, non-PLL entry, invalid source handling and
  the MHz value passed to the ROM function.
- C2/C5/C61 clock checks pass at O0 and Os. All 80 serial-example builds pass.
- These are compiler/linker and host-register checks, not board execution.

Runtime root-clock switching, low-frequency oscillator startup/calibration and
PMU sleep remain pending. The vendor review confirms that C3 Deep-sleep also
needs revision-dependent regulator settings and RTC-memory wake-stub handling;
C6 uses a different PMU sequence. CPU WFI must not be presented as either
Light-sleep or Deep-sleep.

References: Espressif ESP-IDF v5.5.1 components/esp_rom/esp32c3/ld/esp32c3.rom.ld,
components/esp_rom/esp32c6/ld/esp32c6.rom.ld, the per-chip rtc_clk.c and C3
rtc_sleep.c in components/esp_hw_support/port.

## ESP32-C3/C6 low-frequency clock selection

`SystemLowFreqClockSelect()` now uses the TIMG0 calibration unit to measure
low-frequency sources against the 40 MHz crystal before switching the RTC mux.
It follows the existing target-level oscillator selector and descriptor pattern;
the generic API is unchanged. `SystemOscInit()` applies the application oscillator
descriptor. Startup does not call it automatically. Call the boolean selector
when the application needs to handle a missing crystal or failed calibration.

Supported sources:

| Target | Internal RC | External crystal |
| --- | --- | --- |
| C3 | Nominal 136 kHz RC_SLOW | 32.768 kHz XTAL32K |
| C6 | Nominal 136 kHz RC_SLOW or 32.768 kHz RC32K | 32.768 kHz XTAL32K |

An RC request selects the nearest available nominal frequency; zero selects
RC_SLOW. The default C3/C6 descriptors now specify 136 kHz. A crystal request
must specify 32768 Hz or zero. TCXO input and C3 RC_FAST/256 are not implemented.
The RC descriptor reports the measured frequency. The crystal descriptor keeps
its nominal frequency, while the measured period is stored in the hardware's
Q13.19 calibration word for subsequent RTC timing.

This is an initialization operation: call before RTC timekeeping, LP-core
firmware, modem timing or low-power wake scheduling. GPIO0/1 must be reserved
for the crystal. Output, interrupt, GPIO input-matrix and C6 LP-pin assignments
are checked, but these checks do not constitute tracking of every possible
analog or direct-pin consumer. Active RTC/LP watchdogs, configured wake sources,
timer alarm interrupts and C6 modem clock use cause the call to fail. Existing
TIMG timer counters and watchdog configuration are not changed or reset.

One-shot calibration and application-configured continuous calibration are not
interrupted. The hardware-reset continuous configuration can be paused with a
bounded timeout and is restored afterward. Temporary bus gating, digital clock
routing and the C6 shared 32 kHz calibration selection are restored. A missing
or out-of-range oscillator leaves the previous RTC mux, calibration period and
oscillator descriptor unchanged, and restores oscillator configuration/power.
A successful selection leaves previously running oscillators enabled.

The call masks interrupts throughout. Its delay/polling budgets total less than
121 ms, including a 100 ms startup wait for a newly enabled crystal. It should
not be called from an interrupt or during latency-sensitive operation. It does
not enable a watchdog. PMU Light-sleep and Deep-sleep remain unimplemented.

Validation:

- All 201 C3 and 203 C6 library translation units compile in Debug and Release
  with RISC-V GCC 15.2.0, using the managed project source lists and settings.
- PowerOffDemo links for both targets in Debug and Release, both normally and
  with SystemOscInit explicitly retained so the new code is checked by the
  linker. Release flash text increases by 976 bytes on C3 and 1204 bytes on C6
  in the latter case. Unused LF code is discarded.
- Temporary production-source MMIO checks pass at O0 and Os for both chips:
  RC/crystal selection, repeat selection, reset calibration pause, busy
  calibration rejection, hardware timeout, stuck polling, invalid readings,
  rollback, register preservation, clock/pin ownership and interrupt state.
- The existing 24 ESP32 Python checks, 20 GPIO checks and all 80 serial-example
  links pass. No new demo or test framework was added.
- No board execution was performed. Hardware testing remains on SAMD11 until
  the maintainer selects another board.

Register references: Espressif ESP-IDF v5.5.1 per-chip clk_tree_ll.h,
rtc_clk.c, rtc_time.c, timer_group_reg.h, rtc_cntl_reg.h, lp_clkrst_reg.h,
lp_aon_reg.h, pmu_reg.h, pcr_reg.h and the C6 modem_lpcon_reg.h/reg_base.h.
