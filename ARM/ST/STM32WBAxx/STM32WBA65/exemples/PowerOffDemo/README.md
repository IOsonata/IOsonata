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

## Standby register snapshot

`POWER_DEMO_STANDBY_DIAG=1` enables a short register readback immediately
before WFI. PC13 input switching was already validated on hardware, so
there is **no preliminary GPIO polling or press/release test**.

The example prints WUCR1/2/3, WUSR, IORETENRC, CR1, SR,
GPIOC MODER/PUPDR/IDR, SCB SCR, EXTI and SysTick state. It then prints
`Calling SystemPowerOff in 1 second` and enters Standby.
The temporary `StWbaPowerPrepareStandby()` routine prepares registers
once, so SystemPowerOff does not clear wake flags twice.
If the WFI instruction returns without reset, the diagnostic prints
`WFI returned without Standby reset`.

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
pins' retention settings. PC13 input transitions have been confirmed in Run mode. After the
`Power off` message, a single press/release tests WKUP2 Standby wake. Record the resulting reset reason and SRAM comparisons.

## Procedure

1. Build `IOsonata_STM32WBA65` and `PowerOffDemo` in Release.
2. Program and run standalone, monitoring USART1 VCOM.
3. Check the wake register snapshot and `Calling SystemPowerOff in 1 second`.
4. After that message and the one-second delay, momentarily connect
   PC13/D4 to GND and release it.
5. Look for wake reason 0x100, incremented backup count, 8192/8192 retained
   words, and the nonretained memory result. Repeat to validate.

A source review or a successful build does not establish SRAM retention or
wakeup behavior. These must be confirmed on hardware. The radio/USB shutdown
sequence is outside this bare-metal example. Do not use this test while BLE
or USB is active.

## Latest Standby experiment

The demo disables USART1 and its DMA before WFI, after allowing the UART message to finish. If WFI returns, it enables USART1 again to log the failure. Hardware outcome pending.
