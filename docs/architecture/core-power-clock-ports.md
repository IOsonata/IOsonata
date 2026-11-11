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
| STM32 WBA | Missing from shared power implementation | Preserve radio clock ownership |
| SAM4E | Reset, Backup mode, wake pins, GPBR; no SRAM retention in Backup | Slow-clock and main-clock/PLLA selection implemented; hardware validation pending |
| SAM4L | Reset, Backup mode, EIC wake pins, four backup registers; no SRAM retention | Review runtime selection and oscillator initialization |
| SAMD | Missing | Audit existing selectors and low-power clocks |
| LPC11xx, LPC17xx, LPC546xx | Missing | Audit existing selectors and peripheral clocks |
| RA4M1, RE01, R9A02 | Missing | Audit existing selectors and protected register writes |
| ESP32 C-series | Missing | Audit SYSTEM/PCR variants and LP clock domains |
| nRF54H, Raspberry Pi ports | No generic power implementation found | Review port maturity and vendor startup integration first |

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

Reference: [Microchip SAM4L datasheet 42023H](https://ww1.microchip.com/downloads/aemDocuments/documents/OTH/ProductDocuments/DataSheets/Atmel-42023-ARM-Microcontroller-ATSAM4L-Low-Power-LCD_Datasheet.pdf),
sections 6.1.3, 10.7.15, 11.5, 11.6.9..11, 12.7.18 and 21.6.
