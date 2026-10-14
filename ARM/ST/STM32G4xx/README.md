# STM32G4 port status

STM32G474 has a library project, `STM32G474/lib/ioc`, and UART and timer
example projects for the NUCLEO-G474RE (2026-10-13). Startup, GPIO, UART and
timers (LPTIM1 and the TIM) are implemented, the minimum for a supported
MCU. Not tested on hardware yet.
Other G4 parts need their own library project; the shared sources take the
G4 device headers in `include/`.

## Files

- `include/`: ST CMSIS device headers for the G4 family, release 1.2.6 of
  https://github.com/STMicroelectronics/cmsis_device_g4, Apache 2.0, see
  `include/LICENSE.md`. `iopinctrl.h` includes the shared STM32 GPIO header.
- `src/system_stm32g4xx.c`: SystemInit, SystemCoreClockUpdate,
  SystemCoreClockSelect, SystemPeriphClockGet and SystemPeriphClockSet.
- `STM32G474/lib/src/Vector_STM32G474xx.c`: vector table in the STM32G474xx
  interrupt order, 102 interrupt entries up to FMAC.
- `ldscript/gcc_stm32g474xx.ld`: STM32G474xE.
- Shared with the other STM32 families: `ARM/ST/src/iopincfg_stm32.cpp`
  (GPIO and EXTI, the L4 register names), `ARM/ST/src/uart_stm32.cpp` and the
  timer driver, `ARM/ST/src/timer_stm32.cpp`, `timer_lptim_stm32.cpp` and
  `timer_tim_stm32.cpp`.

## Clock

SystemInit runs the core from the PLL at 170 MHz, from HSI16 by default
(16 MHz / 4 x 85 / 2), or from HSE when `g_McuOsc.CoreOsc` is a crystal or an
external oscillator of 4 to 48 MHz (the NUCLEO-G474RE has a 24 MHz crystal:
24 / 6 x 85 / 2). If HSE is not ready after 200000 polls, at least 50 ms
at 16 MHz, SystemInit stays on HSI16 and records it in `g_McuOsc`. Above 150 MHz the
regulator runs in range 1 boost mode with 4 flash wait states; the switch to
the PLL goes through HCLK divided by 2, as RM0440 requires. AHB, APB1 and APB2
run undivided: PCLK1 = PCLK2 = HCLK. SystemPeriphClockSet selects the closest
APB divider. The FPU is enabled when the code is built for it. The vector table
base is set to `__Vectors`. SystemInit does not start the LSE.

## Virtual device numbers

UART indices are 0 for LPUART1, 1 for USART1, 2 for USART2, 3 for USART3,
4 for UART4 and 5 for UART5, as on L4. Polling, interrupt and DMA
(`bDMAMode`, with or without `bIntMode`) on every instance. With a
non-blocking CFIFO each TX DMA block, at most 32 bytes, is a copy taken out
of the CFIFO, see `ARM/ST/STM32F0xx/README.md`. USART1 to UART5
run from PCLK, the driver writes their CCIPR kernel clock selection to 0.
LPUART1 selects LSE, HSI16 or PCLK1 for the requested rate.

DMAMUX1 routes any request to any channel. The driver uses channels 1 to 6
of each controller, which every G4 part has. DMA2 channel 1 is DMAMUX1
channel 8 on the G474, the number of DMA1 channels.

| DevNo | Hardware | RX DMA | TX DMA | DMAMUX1 RX, TX |
|---|---|---|---|---|
| 0 | LPUART1 | DMA2 channel 5 | DMA2 channel 6 | 34, 35 |
| 1 | USART1 | DMA1 channel 1 | DMA1 channel 2 | 24, 25 |
| 2 | USART2 | DMA1 channel 3 | DMA1 channel 4 | 26, 27 |
| 3 | USART3 | DMA1 channel 5 | DMA1 channel 6 | 28, 29 |
| 4 | UART4 | DMA2 channel 1 | DMA2 channel 2 | 30, 31 |
| 5 | UART5 | DMA2 channel 3 | DMA2 channel 4 | 32, 33 |

Timer DevNo (`ARM/ST/include/timer_stm32.h`): 0 LPTIM1, 1 TIM6, 2 TIM7,
3 TIM16, 4 TIM17, 5 TIM15, 6 TIM2, 7 TIM3, 8 TIM4, 9 TIM5, 10 TIM1, 11 TIM8,
12 TIM20. `TimerGetLowFreqDevCount` returns 1.

LPTIM1 is a 16 bits counter extended in software, one compare trigger.
Clock: LSI (`TIMER_CLKSRC_LFRC`), the LSE (`TIMER_CLKSRC_LFXTAL`, crystal or
bypass with `OSC_TYPE_TCXO`, started by TimerInit, which fails if it does
not run within at least 2 s, SystemCoreClock / 2 polls) or HSI16;
`TIMER_CLKSRC_DEFAULT` picks LSI or LSE from `g_McuOsc.LowPwrOsc`. The
frequency is the closest of the kernel clock divided by 1 to 128. RM0440
allows writes to LPTIM_IER and LPTIM_CFGR only with the timer disabled: both
interrupts are enabled at initialization and a disabled trigger only stops
its callback, a frequency change restarts the counter. ARR and CMP are
written with the timer enabled, each confirmed by ARROK or CMPOK. The count
is extended from the previous read and a continuous trigger is scheduled one
period after its previous compare value.

The TIM count at the APB timer clock, 170 MHz, divided by 1 to 65536,
`TIMER_CLKSRC_DEFAULT` only. The triggers are the compare channels: 1 on
TIM16 and TIM17, 2 on TIM15, 4 on TIM1 to TIM5, TIM8 and TIM20; the basic
timers TIM6 and TIM7 have one trigger, the counter period. External trigger
and the tick interrupt are not supported. The timer vectors of
`Vector_STM32G474xx.c` are weak aliases of `Stm32TimerVector`, TimerInit
refuses a timer whose vector is replaced by an application handler (TIM6
and TIM7 share theirs with the DACs), see `ARM/ST/STM32F0xx/README.md`.

## Limits

No I2C, SPI, QSPI, ADC, USB or FDCAN driver for G4. The library project
leaves out `ARM/ST/src/dfu_stm32.cpp` (no G4 flash programming support) and
`src/sysstatus.c`. The CCM SRAM is linked as RAM2 only.

## Memory

`gcc_stm32g474xx.ld`: 496 KB of the 512 KB flash for the image, two 8 KB
storage partitions at the top of bank 2 (dual bank, the reset setting, 2 KB
pages), 96 KB of SRAM1 and SRAM2 as RAM, the 32 KB CCM SRAM at 0x10000000 as
RAM2.

## Example projects

`STM32G474/exemples/UartLoopback` (`exemples/uart/uart_loopback.cpp`),
`STM32G474/exemples/UartPrbsTxTest` (`exemples/uart/uart_prbs_tx.cpp`) and
`STM32G474/exemples/TimerDemo` (`exemples/timer/timer_demo.cpp`) link
`IOsonata_STM32G474` from `STM32G474/lib/ioc` in the same Debug or Release
configuration. Build the library first. `board.h` selects
`#define BOARD NUCLEO_G474RE` (`ARM/ST/include/stm32_boards.h`): USART1,
DevNo 1, on Arduino D0 PC5 RX and D1 PC4 TX, AF7. The ST-LINK virtual COM
port is LPUART1, DevNo 0, RX PA3 and TX PA2, AF12. Both run at 1 Mbaud with
the example defaults: interrupts without DMA for the loopback, DMA without
interrupts for PRBS TX. Like the L4 examples they link `rdimon` semihosting:
run them from a debugger with semihosting enabled.

TimerDemo prints on the ST-LINK virtual COM port, LPUART1 at 115200 8N1, and
toggles LD2 on trigger 0; it links `nosys`. `TIMER_DEMO_DEVNO` in `board.h`
selects the timer, LPTIM1 by default, from the LSE crystal, 100 ms per
trigger 0 event (3277 ticks, 100.006 ms). The TIM run at 100 kHz, a four
channel TIM prints triggers 0 to 3: 100 ms, 1 s, 250 ms and 500 ms.

## Validation

```sh
python3 tests/stm32g4/run.py
python3 tests/stm32/run_uart.py
python3 tests/stm32/run_timer.py
```

`tests/stm32g4/run.py` runs `system_stm32g4xx.c` with the ST STM32G474
header and plain memory at the peripheral addresses, with UBSan: PLL
settings from HSI16 and from 8, 12 and 24 MHz HSE, HSE bypass, HSE start
failure, wait states and boost mode, APB dividers and SystemCoreClockUpdate.
`tests/stm32/run_timer.py` runs the timer driver for the G474: LPTIM1 clock
selection and prescaler, tick count across the counter wrap, continuous and
single compare trigger, frequency change, disable and enable, every TIM with
a register model, and the timer vectors of the vector file.
`tests/stm32/run_uart.py` runs the UART
driver with the STM32G474 and STM32G431 headers (the G431 has 6 DMA channels
per controller): instance clocks, kernel clock selection, vectors, DMAMUX1
requests, polling, interrupt and DMA transfers, RX errors and LPUART1 clock
selection.

The library in its Debug, Release, Debug_FPU and Release_FPU configurations
and the three examples in Debug and Release were built from their `.project`
and `.cproject` settings with arm-none-eabi-gcc 13.2.1, outside IOcomposer; all
118 vector table words match the STM32G474xx interrupt list. The example
images ran in an instruction set emulator with USART1, DMA1, DMAMUX1, RCC and
flash models: SystemInit left PLLCFGR at HSI16 / 4 x 85 / 2, 4 wait states and
boost mode, USART1 BRR 170 at 1 Mbaud; the loopback echoes in order with no
lost character in polling, interrupt, DMA without interrupts and DMA with
interrupts, and the PRBS output matches `Prbs8()`. With a non-blocking CFIFO
the loopback echoed 98.4 kB/s in order in both DMA modes. TimerDemo, built
for LPTIM1, TIM6, TIM17, TIM5, TIM1 and TIM20, ran in the emulator with
LPTIM, TIM, RCC and LPUART1 models: 100 ms trigger 0 periods, the four
triggers of TIM5, TIM1 and TIM20, no LPTIM write order violation. This does
not replace a run on the board.

References: RM0440 (STM32G4 reference manual), UM2505 (NUCLEO-G474RE).
