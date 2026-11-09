# STM32 power functions

`ARM/ST/src/system_power_stm32.cpp` implements the power functions of
`include/coredev/system_core_clock.h` on the F0, F4, L1, L4, L4+, G4 and H7
families: `SystemResetReason`, `SystemPowerOff`, `SystemWakePin`,
`SystemRetRegCount`, `SystemRetRegRead`, `SystemRetRegWrite` and
`SystemRamRetain`. One source serves them all; the differences are keyed on
the CMSIS device header. The WBA is not done yet.

## Power off

`SystemPowerOff` enters Standby, or Shutdown on the L4, L4+ and G4 when no RAM
is kept (`SystemRamRetain`). Shutdown draws the least current; it has no brown
out reset and its exit is a power on reset. The function returns only when a
debugger keeps the MCU running.

Standby and Shutdown wake up on the WKUP pins, the RTC alarms and wakeup
timer, the IWDG and NRST. `SystemWakePin` accepts only the WKUP pins:

| Family | WKUP1 | WKUP2 | WKUP3 | WKUP4 | WKUP5 | WKUP6 | Levels |
|---|---|---|---|---|---|---|---|
| F030x8 | PA0 | PC13 | | | | | rising |
| F401 | PA0 | | | | | | rising |
| L152xE | PA0 | PC13 | PE6 | | | | rising |
| L4, L4+, G4 | PA0 | PC13 | PE6 | PA2 | PC5 | | both |
| H7 | PA0 | PA2 | PI8 | PC13 | PI11 | PC1 | both |

A pin the package does not have is accepted but cannot wake the MCU (PC13,
PE6 and PC5 on the L432KC, PE6 on the LQFP64 parts, PI8 and PI11 on the
LQFP144 H753). On the F0, F4 and L1 the hardware forces an enabled WKUP pin to
input pull down. On the L4, L4+ and G4 the pull resistor of the pin GPIO
configuration is kept in power off by PWR PUCRx/PDCRx and APC (left set after
the wake up), on the H7 by PWR WKUPEPR WKUPPUPDx. A pin already at its wake up
level, or an RTC event flag left set, wakes the MCU at once. The RTC events
are set up with `RtcStm32`.

The option bytes nRST_STDBY and nRST_SHDW at 0 reset the MCU instead of
entering the mode. On the H7, VOS0 (SYSCFG PWRCR ODEN) must not be active;
`SystemInit` does not set it.

## Reset reason

`SystemResetReason` reads and clears the reset flags of RCC CSR (RCC RSR on
the H7) and the Standby and wake up flags of PWR on its first call:

| Flags | Reason |
|---|---|
| PORRSTF, BORRSTF, V18PWRRSTF | `SYSTEM_RESET_POWERON` |
| IWDGRSTF, WWDGRSTF | `SYSTEM_RESET_WDT` |
| SFTRSTF, OBLRSTF, LPWRRSTF, FWRSTF | `SYSTEM_RESET_SOFT` |
| PINRSTF alone | `SYSTEM_RESET_PIN` |
| Standby flag and a WKUP pin flag | `SYSTEM_RESET_WAKE_PIN` |
| Standby flag and WUFI (L4, G4) or an RTC alarm or wakeup timer flag | `SYSTEM_RESET_WAKE_TIMER` |

An internal reset is output on NRST and sets PINRSTF with its own flag: the
reset pin is reported only when no other flag is set. After a Standby exit the
power reset flags are not reported. On the F0, F4 and L1 the wake up flag WUF
is set by the pins and by the RTC events, the RTC flags tell them apart. A
Shutdown exit is a brown out reset, reported as a power on.

## Retention registers and RAM

The retention registers are the backup registers of the RTC (TAMP on the G4),
the same registers as `RtcStm32::ReadBackup` and `WriteBackup`: none on the
F030x8, 20 on the F4, 32 on the others, 32 bits each. They are kept over a
system reset, Standby and Shutdown, and lost on a backup domain reset or a
tamper event. A write enables the backup domain write access (PWR DBP), left
enabled as `RtcStm32` does.

| Family | RAM kept in power off |
|---|---|
| L432 | SRAM2, 16 KB at 0x10000000, alias 0x2000C000 |
| L476 | SRAM2, 32 KB at 0x10000000 |
| L496 | SRAM2, 64 KB at 0x10000000, alias 0x20040000 |
| L4S9 | SRAM2, 64 KB at 0x10000000, alias 0x20030000 |
| G474 | SRAM2, 16 KB at 0x20014000 (not the CCM SRAM) |
| H753 | Backup SRAM, 4 KB at 0x38800000 (PWR CR2 BREN) |
| F030x8, F401, L152xE | none |

SRAM2 and the backup SRAM are kept whole: a `SystemRamRetain` range must be
inside one of them, other ranges are refused. Keeping SRAM2 selects Standby
instead of Shutdown.

## Libraries and tests

The F030x8, F401, L152RE, L432, L476, L496, L4S9, G474 and H753 library
projects build the functions. Example `exemples/misc/power_off_demo.cpp`:
`STM32L4xx/STM32L476/exemples/PowerOffDemo` (B-L475E-IOT01A, USART1 virtual
COM port, user button B1 PC13 to ground), `STM32G4xx/STM32G474/exemples/PowerOffDemo`
(NUCLEO-G474RE, LPUART1 virtual COM port, B1 PC13 to the supply) and
`STM32H7xx/STM32H753/exemples/PowerOffDemo` (NUCLEO-H753ZI, USART3 virtual
COM port, B1 PC13 to the supply). It logs the reset reason and a start count
kept in retention register 0, then powers off; B1 wakes the MCU. On the L476
and G474 the demo keeps no RAM, the MCU goes to Shutdown and its wake up is
reported as a power on.

`tests/stm32/run_power.py` builds them with `power_fw.cpp` for the F030x8,
F401xC, L152xE, L432, L476, L496, L4S9, G474 and H753 at -O0 and -Os and calls
each function in Unicorn against a model of the RCC reset flags, PWR, the RTC
backup registers and alarm flag, the GPIO pull resistors and SCB SCR. The wake
up pins and the RAM kept come from the reference manuals and datasheets in the
script. Not run on hardware yet.
