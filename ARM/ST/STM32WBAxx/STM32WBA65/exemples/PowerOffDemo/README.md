# STM32WBA65I-DK1 PowerOffDemo

This Eclipse managed project tests the bare-metal power API on STM32WBA65I
(TZEN=0). It uses the shared `exemples/misc/power_off_demo.cpp` source.

## Wiring and console

- USART1 VCOM (first ST-LINK virtual COM port), RX PA8 / TX PB12, 1,000,000 baud.
- Wire a **normally open momentary switch** between **PC13 (Arduino D4)**
  and **GND**. Do not keep it held during startup; the input uses pull-up.
- Standby wakeup uses WKUP2, source PC13, active low.
- The onboard joystick is analog, not a GPIO wake button.

## Memory layout

The example deliberately uses `gcc_stm32wba65_i_power_test.ld`, not the
normal `gcc_stm32wba65_i.ld` linker script. The separate memory regions are:

| Region | Address | Size | Use |
|---|---|---|---|
| RAM | 0x20000000 | 416 KB | Normal code data, stack |
| RAMDROP | 0x20068000 | 32 KB | SRAM1, deliberately not retained |
| RAMRET | 0x20070000 | 64 KB | SRAM2, retained |

The shared demo writes 32 KB patterns into RAMDROP and the first 32 KB of
RAMRET. Retention configuration is whole-bank; RAM1 retention is disabled
and RAM2 retention is enabled before Standby. Test memory must never overlap
`.data`, `.bss` or stack.

## Live power-register snapshot

The test builds with `POWER_DEMO_WAKE_DIAG=1`. After the first PC13
press/release, the example prepares the actual Standby register state without
yet issuing WFI, then prints:

- `WUCR1`, `WUCR2`, `WUCR3`, `WUSR` for WKUP2 enable, polarity, selection,
  and any pending wake flags
- `IORETENRC` for PC13 pull retention, `CR1` and `SR` for power mode/status
- GPIOC `MODER`, `PUPDR`, `IDR`, and SCB `SCR` for pin state and SLEEPDEEP

The second call, `SystemPowerOff()`, consumes this prepared state and
executes WFI without clearing or reprogramming the wake flags a second time.
If WFI returns without a reset, the diagnostic reports
`WFI returned without Standby reset` and stops. Before WFI the log also
reports EXTI rising/falling pending flags, SCB ICSR, SysTick CTRL and
NVIC pending registers ISPR0–2. Pending interrupts or peripheral flags
may prevent Standby mode entry, even when WUCR1/2/3 are correct.
This preparation entry point is temporary diagnostic support, not a generic
power API addition. Record these register values even if the second PC13 press
does not wake the board.

## Standby entry diagnostics

The WBA65 power implementation now clears stale WKUP status before enabling
the configured wake line, as recommended by the STM32WBA HAL sequence.
It verifies the WUCR1/2/3 readback before entering Standby. If readback
fails, SystemPowerOff returns, and the diagnostic build prints
`SystemPowerOff returned: Standby was not entered`. Absence of this
message is not by itself proof that Standby was reached; the next
confirmation is a GPIO wake reset with the expected retained data.

## Standby wake input pull retention

On STM32WBA6, configuring PC13 as GPIO input/pull-up before Standby is
not sufficient by itself: GPIO configuration is lost when the core domain
powers down. The power driver now enables the matching bit in
`PWR_IORETENRC` for an explicitly enabled wake pin that has a pull
configured, preserving that input pull while in Standby. It preserves other
pins' retention settings. The first press/release still checks PC13 in Run;
after the `Power off` message, a second press/release tests the WKUP2
Standby path. Record the resulting reset reason and SRAM comparisons.

## Procedure

1. Build `IOsonata_STM32WBA65` and this example with matching Release
   configurations.
2. Program the DK1, disconnect active debugging, and monitor the VCOM.
3. In this diagnostic build, the firmware first prints `Wake GPIO P2.13`
   (the port index for GPIOC is 2). While the 30-second input check runs,
   momentarily connect PC13/D4 to GND and release it. Confirm the log
   contains `Wake GPIO active level=0`, `Wake GPIO released level=1`,
   and `Wake GPIO check PASS`. A live input level is also printed every second. If it prints FAIL, Standby is skipped:
   verify wiring and PC13 GPIO operation before debugging power registers.
4. After the check passes and the MCU enters Standby, briefly connect
   PC13 to GND again and release it.
5. Confirm wake reason 0x100, increasing backup-register count, all 8192
   retained words matching and no prior-pattern words remaining in the
   nonretained region. Repeat multiple times.

A source review or a successful build does not establish SRAM retention or
wakeup behavior. These must be confirmed on hardware. The radio/USB shutdown
sequence is outside this bare-metal example. Do not use this test while BLE
or USB is active.
