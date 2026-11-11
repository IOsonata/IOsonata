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
| SAM4E | Reset, Backup mode, wake pins, GPBR; no SRAM retention in Backup | Slow-clock selection implemented; main-clock/PLL switching still needs review |
| SAM4L, SAMD | Missing | Audit existing selectors and low-power clocks |
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
