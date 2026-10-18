# STM32H753 / NUCLEO-H753ZI

Initial IOsonata port for the STM32H753ZI Cortex-M7. It supplies startup,
GPIO/EXTI, polling and interrupt UART, and the 14 general-purpose/basic/advanced
TIM instances. Hardware validation is pending.

## Build and first board tests

Import `STM32H753/lib/ioc` and the required projects under
`STM32H753/exemples` into IOcomposer. Build the library first, then the example
with the matching Debug or Release configuration. GNU Arm GCC uses
`STM32H753xx`, Cortex-M7 and the normal IOsonata linker script. The library
also has FPU configurations using `fpv5-d16`; match the application's ABI
when using those configurations.

1. Run `Blinky`: the three user LEDs cycle.
2. Run `TimerDemo`: USART3 on the ST-LINK virtual COM port prints at 115200
   baud, 8N1. Default DevNo 8 selects TIM2, at 100 kHz with multiple triggers.
3. Run `UartPrbsTxTest` in Release, then on the host:
   `python3 Python/uartprbs_rx.py --baud 1000000 --port <serial-port>`.
4. Run `UartLoopback` in Release, then:
   `python3 Python/uartprbs_lb.py --baud 1000000 --port <serial-port>`.

The UART examples default to interrupt mode, DMA off, on the ST-LINK virtual
COM port: PD9 RX, PD8 TX, AF7, USART3 DevNo 3. Their `board.h` can select
`BOARD_UART_DEVNO` / `BOARD_UART_PINS` instead for USART1 on Arduino D0/D1,
PB7 RX and PB6 TX, AF7. Connect an external 3.3 V serial adapter there.
Board pin mappings are in `ARM/ST/include/stm32_boards.h`.

## Clocks and memory

Startup selects the internal LDO, voltage scale 1 and HSI64 / 32 feeding PLL1
with N=400 and P=2: 400 MHz CPU, 200 MHz AHB, 100 MHz APB1 through APB4.
Flash uses four wait states. No external oscillator or ST-LINK clock bridge
is required. HSI accuracy limits absolute baud and timer accuracy. HSE selection
is not implemented. Ready waits are bounded; a failed PLL start leaves the CPU
on HSI, with the clock variables reflecting the actual dividers.

`SystemPeriphClockGet/Set` indices 0, 1, 2 and 3 select APB1, APB2, APB3 and
APB4. APB setters choose the closest divider without exceeding 100 MHz.
The timer driver accounts for the AHB divider and TIMPRE independently of the
CPU clock.

The image uses 2 MiB flash and 128 KiB DTCM for data and stack. AXI SRAM is a
separate 512 KiB RAM2 region. D2 SRAM and SRAM4 are not allocated by this linker
script. CPU caches remain at their reset state, disabled. DMA is not supported;
DTCM must not be assumed accessible to DMA1/2.

## Device numbers

UART DevNo order is LPUART1, USART1, USART2, USART3, UART4, UART5, USART6,
UART7, UART8 (0 through 8). LPUART1 currently uses PCLK4; requested rates are
rounded to the supported divider range. With PCLK4 at 100 MHz, its minimum
rate is about 24.4 kbaud. The USART instances cover the example baud rates.

Timer device numbers are virtual indices, not hardware timer numbers:

| DevNo | Timer | Triggers |
|---|---|---|
| 0, 1 | TIM6, TIM7 | 1 each |
| 2, 3 | TIM13, TIM14 | 1 each |
| 4, 5 | TIM16, TIM17 | 1 each |
| 6, 7 | TIM12, TIM15 | 2 each |
| 8, 9, 10, 11 | TIM2, TIM3, TIM4, TIM5 | 4 each |
| 12, 13 | TIM1, TIM8 | 4 each |

The shared timer implementation uses a 16-bit counter extended in software.
LPTIM is not yet exposed. `TimerGetLowFreqDevCount()` returns 0.

## Limits and validation

I2C, SPI, USB, Ethernet, CAN, ADC/DAC, PWM, internal NVM, RNG/crypto and DMA
are not implemented by this port. Unsupported UART DMA configuration returns
false. The single library project includes the core device, UART, timer,
application event and logging sources needed by these examples.

`python3 tests/stm32h7/run.py` builds the library and all four examples in
Debug/Release, with and without hard-float, and runs Cortex-M instruction tests
using the actual CMSIS header. Install Python `unicorn` and `pyelftools` for
the modeled register tests. These check startup clock selection, all four APB
clock setters, UART3/GPIO setup, DMA rejection, all 14 timer device indices,
and timer clock calculation with APB /1 and TIMPRE.

All four configurations passed with GNU Arm GCC 13.2.1. This is compile/link
and modeled register evidence, not a hardware throughput or timing result.

## References

- ST RM0433 and the STM32H753 datasheet.
- [NUCLEO-H753ZI board manual UM2407](https://www.st.com/resource/en/user_manual/um2407-stm32h7-nucleo144-board-stmicroelectronics.pdf).
- ST `cmsis-device-h7`, commit `81db1ec63cdc191fae1565b772da3ea5aa29a683`:
  unmodified device, family and system headers in `include`, with ST's license.
- ST STM32CubeH7 `Projects/NUCLEO-H743ZI/Templates_LL/Src/main.c` confirms the
  400/200/100 MHz voltage-scale-1 clock and flash timing configuration. This
  port accesses the CMSIS registers directly; it does not depend on HAL or LL.
