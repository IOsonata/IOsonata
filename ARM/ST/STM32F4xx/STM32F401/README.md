# STM32F401 (xC / xE)

Build status: **PASS** - consolidated STM32F401 library and all six xE example projects compiled successfully (reported October 23, 2026).
Hardware testing and xC-specific application builds remain pending.

Both memory variants use the same STM32F401 peripheral registers, interrupts,
startup and drivers. They share one library project at `lib/ioc` named
`IOsonata_STM32F401`, compiled with `STM32F401xx`.

The common `ARM/ST/include/stm32.h` maps `STM32F401xx` to ST's
`STM32F401xE` register definitions. The library C compiler pre-includes this
IOsonata selector so ST's CMSIS system source remains unmodified. Runtime
Flash code must not assume the xE Flash boundary for an xC device.

## Memory variants

| Device | Flash | RAM | Linker script |
| --- | --- | --- | --- |
| STM32F401xC | 256 KB | 64 KB | `ARM/ST/STM32F4xx/ldscript/gcc_stm32f401xc.ld` |
| STM32F401xE | 512 KB | 96 KB | `ARM/ST/STM32F4xx/ldscript/gcc_stm32f401xe.ld` |

Select the appropriate linker script **in the application**, not in
the library. The included example Eclipse projects default to the xE script
for NUCLEO-F401RE. For an xC target, change the linker-script entry in both
Debug and Release to `gcc_stm32f401xc.ld`. The `lib/ioc/wizard.json` default
is the xE script.

## Examples

Import `lib/ioc` and then the example `ioc` projects under
`exemples/`: Blinky, TimerDemo, UartPrbsTxTest, UartLoopback,
WdtDemo and SPILoopback.

The included examples target NUCLEO-F401RE with PA5 LED, PC13 button
and USART2 PA2/PA3 ST-LINK virtual COM port. The SPI loopback
uses SPI1 PA5/PA6/PA7 and PB6 for chip select.

For an F401xC board, keep the same library but choose the xC linker
script and provide the correct board pin map and clock configuration.
The NUCLEO-F401RE pinout is not a generic F401xC board pinout.

## Peripheral status

The consolidated library and Blinky, TimerDemo, UartPrbsTxTest, UartLoopback,
WdtDemo and SPILoopback compiled successfully for the xE target. I2C v1
polling has initial source integration but has not been separately build- or
hardware-validated. ADC and F4 NVM programming are not supported by current
generic drivers. No F401 hardware runtime validation has been performed.
