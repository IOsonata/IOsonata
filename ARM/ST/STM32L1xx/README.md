# STM32L152RE / NUCLEO-L152RE

Initial port for the STM32L152RET6 Cortex-M3 on NUCLEO-L152RE. Startup,
GPIO/EXTI, polling and interrupt UART, and nine TIM instances are implemented.
Library and examples compile and pass modeled register checks. Hardware
validation is pending.

## Build and board tests

Import `STM32L152RE/lib/ioc` and the required projects under
`STM32L152RE/exemples` into IOcomposer. Build the library first, then the
example with the matching Debug or Release configuration. Both library
configurations produce `libIOsonata_STM32L152RE.a` in their own output folder.
The device define is `STM32L152xE`; compiler options are Cortex-M3 and Thumb.
This core has no FPU.

Use OpenOCD board configuration `board/st_nucleo_l1.cfg`. It selects the
ST-LINK interface and `target/stm32l1x_dual_bank.cfg` for this device:

```sh
openocd -f board/st_nucleo_l1.cfg
```

1. Run Blinky: the green LD2 on PA5 should flash at 1-second intervals.
2. Run UartPrbsTxTest in Release, then:
   `python3 Python/uartprbs_rx.py --baud 1000000 --port <serial-port>`.
3. Run UartLoopback in Release, then:
   `python3 Python/uartprbs_lb.py --baud 1000000 --port <serial-port>`.
4. Run TimerDemo in Release. Open the ST-LINK virtual COM port at 115200
   baud, 8N1. Default DevNo 5 selects TIM2 at 100 kHz, with trigger periods
   of 100, 1000, 250 and 500 ms. Override `TIMER_DEMO_DEVNO` to test other TIMs.

The UART examples use USART2 (DevNo 1), PA3 RX and PA2 TX, AF7, interrupt
mode and DMA disabled. The onboard ST-LINK virtual COM port uses these pins
with the default solder bridges; no external serial adapter is needed.
Arduino D0/D1 share the same UART pins. Board mappings are selected with
`BOARD NUCLEO_L152RE` in `stm32_boards.h`. B1 is PC13, active low.

Debug examples link rdimon semihosting. Enable it before resuming in the
OpenOCD debugger with `monitor arm semihosting enable`. Debug TimerDemo prints
to the OpenOCD console. Use Release UART output for timing and throughput
checks: semihosting halts the CPU and distorts interrupt timing.

## Clocks and memory

Startup selects voltage range 1, 64-bit flash access, prefetch and one wait
state. HSI16 feeds PLL x6 /3 for 32 MHz CPU, AHB, APB1 and APB2. No external
oscillator or ST-LINK MCO bridge is required. Oscillator and voltage waits
are bounded, and `SystemCoreClockUpdate` reports the selected clock even
if startup cannot complete the PLL switch.

`SystemPeriphClockGet/Set` indices 0 and 1 select APB1 and APB2. Setters choose
the closest available divider. TIM clocks double PCLK when the APB prescaler
is not 1. HSI accuracy limits absolute UART baud and timer accuracy. HSE and
low-frequency oscillator selection are not implemented in this initial port.

The linker uses all 512 KiB of flash at 0x08000000 and 80 KiB of SRAM at
0x20000000. EEPROM and flash storage partitions are not exposed.

## Device numbers

UART virtual indices are 0 USART1, 1 USART2, 2 USART3, 3 UART4 and 4 UART5.
The peripheral instance must also have usable pins on the selected package.

Timer DevNo values are virtual indices, not hardware timer numbers:

| DevNo | Timer | Triggers |
|---|---|---|
| 0, 1 | TIM6, TIM7 | 1 each |
| 2, 3 | TIM10, TIM11 | 1 each |
| 4 | TIM9 | 2 |
| 5, 6, 7, 8 | TIM2, TIM3, TIM4, TIM5 | 4 each |

All use the shared STM32 TIM driver with a 16-bit counter extended in software.
There is no LPTIM on this MCU. `TimerGetLowFreqDevCount()` returns 0.
The timer tick interrupt and external trigger modes are not implemented.

## Validation and limits

`python3 tests/stm32l1/run.py` builds the library and all four examples in
Debug and Release with GNU Arm GCC. Python `unicorn` and `pyelftools` are
required for the Cortex-M instruction tests. The tests use ST's CMSIS header
and model oscillator-ready flags; they do not model electrical signals.

GNU Arm GCC 13.2.1 Debug and Release builds pass. Register checks cover the
32 MHz startup, voltage/flash setup, APB divider selection, GPIO and L1 EXTI
port encoding, USART2 baud/pins/interrupt configuration, DMA rejection, all
nine TIM indices and trigger counts, and TIM clock calculation with APB
prescalers. H753 library/example/register tests also pass after the shared
GPIO and timer additions. No L152 hardware result is claimed yet.

DMA, I2C, SPI, USB, RTC, LCD, ADC/DAC, PWM, EEPROM/internal NVM and low-power
modes are not implemented by this port. UART initialization rejects DMA.
Only the minimum core device, UART, timer and application support sources
are included in the single library project.

## References

- ST STM32L151xE/STM32L152xE datasheet and RM0038 reference manual.
- NUCLEO-L152RE / MB1136 board documentation, UM1724.
- Unmodified ST CMSIS device headers from `cmsis-device-l1` commit
  `a23ade4ccf14012085fedf862e33a536ab7ed8be`, with the vendor license in `include`.
- STM32CubeL1 `Projects/NUCLEO-L152RE/Templates/Src/main.c`: HSI16 x6 /3,
  voltage range 1 and one flash wait state. The port uses CMSIS registers
  directly; it does not depend on HAL.
