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

## Hardware validation (November 14, 2026)

STM32WBA65I-DK1 (MB2143), bare-metal Release PowerOffDemo, without
active debugging: **two consecutive Standby and PC13/WKUP2 wakeup cycles
passed**. GPIO wake reason was 0x100 on both boots. The backup-register
startup counter advanced to 2 and then 3. After each wake, SRAM2 retained
8192/8192 words and the SRAM1 non-retained region had 0/8192 unchanged
words.

The successful test required disabling USART1 and its DMA before
`SystemPowerOff()`. Earlier tests with USART1 running did not wake.
Those results demonstrate the shutdown path works with the UART stopped,
but do not distinguish whether the active UART peripheral, its DMA or
a related clock/request prevented Standby.

The final example sets `POWER_DEMO_UART_OFF=1`, waits for the last
UART record, then calls `s_Uart.Disable()` before power-off. The
30-second GPIO continuity test, power-register snapshot and temporary
Standby preparation function were removed following validation.

## Procedure

1. Build `IOsonata_STM32WBA65` and `PowerOffDemo` in Release.
2. Program and run standalone, monitoring USART1 VCOM at 1 Mbaud.
3. Wait for `Power off, press the button to wake up` and for UART
   shutdown.
4. Momentarily connect PC13 (Arduino D4) to GND and release it.
5. Confirm reset reason 0x100, incremented startup counter, SRAM2
   retention PASS and SRAM1 non-retention PASS. Repeat.

A source review or a successful build does not establish SRAM retention or
wakeup behavior. These must be confirmed on hardware. The radio/USB shutdown
sequence is outside this bare-metal example. Do not use this test while BLE
or USB is active.

## Latest Standby experiment

The demo disables USART1 and its DMA before WFI, after allowing the UART message to finish. If WFI returns, it enables USART1 again to log the failure. Hardware outcome pending.
