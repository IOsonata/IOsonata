# STM32H753 / NUCLEO-H753ZI

Initial IOsonata port for the STM32H753ZI Cortex-M7. It supplies startup,
GPIO/EXTI, polling and interrupt UART, and the 14 general-purpose/basic/advanced
TIM instances, plus USB2 full-speed device support through the shared STM32
OTG driver. Startup, LED GPIO, USART3 TX/RX and TIM2 multi-trigger operation
are hardware tested on NUCLEO-H753ZI.

## Build and first board tests

Import `STM32H753/lib/ioc` and the required projects under
`STM32H753/exemples` into IOcomposer. Build the library first, then the example
with the matching Debug or Release configuration. GNU Arm GCC uses
`STM32H753xx`, Cortex-M7 and the normal IOsonata linker script. The library
also has FPU configurations using `fpv5-d16`; match the application's ABI
when using those configurations.

1. Run `Blinky`: the three user LEDs cycle.
2. Run `TimerDemo` in Release: USART3 on the ST-LINK virtual COM port prints
   at 115200 baud, 8N1. Default DevNo 8 selects TIM2 at 100 kHz, with trigger
   periods of 100, 1000, 250 and 500 ms. Debug uses semihosting; add
   `monitor arm semihosting enable` to the GDB startup commands before resume.
   Debug output appears in the OpenOCD console. Semihosting delays interrupt
   handling and distorts measured periods; use Release UART output for this test.
3. Run `UartPrbsTxTest` in Release, then on the host:
   `python3 Python/uartprbs_rx.py --baud 1000000 --port <serial-port>`.
4. Run `UartLoopback` in Release, then:
   `python3 Python/uartprbs_lb.py --baud 1000000 --port <serial-port>`.

The onboard ST-LINK UART bridge uses the same USB connection as debugging;
no external UART adapter or jumpers are needed with the default board configuration.
On macOS, find the port with `ls /dev/cu.usbmodem*`.

The UART examples default to interrupt mode, DMA off, on the ST-LINK virtual
COM port: PD9 RX, PD8 TX, AF7, USART3 DevNo 3. Their `board.h` can select
`BOARD_UART_DEVNO` / `BOARD_UART_PINS` instead for USART1 on Arduino D0/D1,
PB7 RX and PB6 TX, AF7. Connect an external 3.3 V serial adapter there.
Board pin mappings are in `ARM/ST/include/stm32_boards.h`.

## TaktOS UART PRBS example

`STM32H753/exemples/UartPrbsTxTestTaktOS/ioc` uses the existing shared
`exemples/uart/uart_prbs_tx_taktos.cpp`. It sends 16-byte PRBS buffers from a
TaktOS thread through USART3/ST-LINK VCP at 1 Mbaud, 8N1, interrupt mode,
DMA disabled. The scheduler uses processor-clock SysTick at 1 kHz.

Keep the TaktOS repository beside the IOsonata repository. Import
`TaktOS/ARM/cm7/ioc`, the H753 library, and this example into IOcomposer.
Build both libraries before the example, using these matching configurations:

| Example | IOsonata_STM32H753 | TaktOS_M7 |
|---|---|---|
| Debug | Debug_FPU | Debug_DP |
| Release | Release_FPU | Release_DP |

All three use Cortex-M7 with `-mfpu=fpv5-d16 -mfloat-abi=hard`. Debug links
semihosting, so enable it in the debugger before resume. Release uses nosys
and runs without a debugger. Use Release for throughput tests, then run:

```sh
python3 Python/uartprbs_rx.py --baud 1000000 --port <serial-port>
```

Debug and Release library/kernel/example builds pass with GNU Arm GCC 13.2.1.
The linked vectors select TaktOS PendSV and SysTick handlers. TaktOS operation
on the H753 board has not yet been hardware tested.

## Native USB CDC examples

Import `STM32H753/exemples/UsbCdcLoopback/ioc` or
`STM32H753/exemples/UsbCdcPrbsTx/ioc` alongside the same MCU library. These
projects link the existing common applications in `exemples/usb`. Build the
library and application in matching Debug or Release configurations. Debug
links semihosting; enable `monitor arm semihosting enable` before resume.
Release uses nosys and runs without the debugger.

Power the board through ST-LINK first, then connect a data cable from **CN13**
to the host. CN13 is the MCU's native USB port; its CDC port is separate from
the ST-LINK USART3 bridge. UM2407 requires board power before connecting CN13.
Keep the default SB13 USB supply and SB21/SB22/SB23 data/VBUS connections.

1. Flash `UsbCdcLoopback`, find the new serial port, and open it to check the
   banner and echoed characters. Then run:
   `python3 Python/usb_cdc_loopback.py --port <native-usb-port> --duration 60`.
   Expect zero errors and zero pending bytes after draining.
2. Flash `UsbCdcPrbsTx` and run:
   `python3 Python/uartprbs_rx.py --baud 1000000 --port <native-usb-port>`.
   The baud argument is CDC line-coding metadata; it does not set USB speed.
3. Check device reset, cable reconnect and host suspend/resume separately.

USB DevNo 0 selects USB2 OTG with its embedded FS PHY: PA11 D-, PA12 D+,
AF10, and PA9 VBUS sense. HSI48 supplies the USB clock and CRS trims it from
USB2 SOF without changing PLL1. VDD33USB is supplied externally by the board.
The controller has nine endpoint numbers and 4096 bytes of FIFO RAM, as
specified by RM0433. Transfers use interrupts and CPU FIFO copies, so caller
buffers may remain in DTCM. The generic USB core and classes are unchanged.

Control, bulk and interrupt transfers are supported. USB1, external ULPI,
high-speed operation, host mode, ISO and USB DMA are not exposed. These
examples do not advertise remote wakeup. See [shared STM32 USB notes](../USB.md).
USB enumeration, electrical behavior and sustained transfers still require
hardware validation on this board.

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
script. SystemInit enables the Cortex-M7 instruction cache for flash execution;
data cache remains disabled. DMA is not supported by the current peripheral
ports; DTCM must not be assumed accessible to DMA1/2.

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

I2C, SPI, Ethernet, CAN, ADC/DAC, PWM, internal NVM, RNG/crypto and DMA
are not implemented by this port. Unsupported UART DMA configuration returns
false. The single library project includes the core device, UART, timer,
application event and logging sources needed by these examples.

`python3 tests/stm32h7/run.py` builds the library and all six bare-metal examples in
Debug/Release, with and without hard-float, and runs Cortex-M instruction tests
using the actual CMSIS header. Install Python `unicorn` and `pyelftools` for
the modeled register tests. These check startup clock selection, all four APB
clock setters, UART3/GPIO setup, DMA rejection, all 14 timer device indices,
timer clock calculation with APB /1 and TIMPRE, and USB2 clock/power/pin setup
with a failed HSI48 start. USB firmware vectors select the shared OTG handler.

All four configurations passed with GNU Arm GCC 13.2.1. This is compile/link
and modeled register evidence, not a hardware throughput or timing result.

## Hardware results

User tests on NUCLEO-H753ZI, October 18, 2026:

| Test | Result |
|---|---|
| Library and Blinky builds | Successful on the user's toolchain |
| Blinky | All three user LEDs flash at 1-second intervals |
| UartPrbsTxTest, Release, 1 Mbaud | About 100.34 kB/s through ST-LINK VCP; no errors reported in the supplied log |
| UartLoopback, Release, 1 Mbaud | 58.6-65.0 kB/s received; zero errors in the supplied log |
| TimerDemo, Release, USART3 at 115200 baud | TIM2 (DevNo 8), all four triggers operating; steady periods of 100000, 1000000, 250000 and 500000 us |

UART tests used the default interrupt mode with DMA disabled. Timer first-sample
offsets settled on subsequent cycles; steady periods remained consistent through
the supplied 8.6-second log. These are measurements against the MCU timer clock,
not an independent measurement of absolute oscillator accuracy.

Other timer instances, other UART instances, polling UART and GPIO EXTI have
not yet been validated on hardware.

## References

- ST RM0433 and the STM32H753 datasheet.
- [NUCLEO-H753ZI board manual UM2407](https://www.st.com/resource/en/user_manual/um2407-stm32h7-nucleo144-board-stmicroelectronics.pdf).
- ST `cmsis-device-h7`, commit `81db1ec63cdc191fae1565b772da3ea5aa29a683`:
  unmodified device, family and system headers in `include`, with ST's license.
- ST STM32CubeH7 `Projects/NUCLEO-H743ZI/Templates_LL/Src/main.c` confirms the
  400/200/100 MHz voltage-scale-1 clock and flash timing configuration. This
  port accesses the CMSIS registers directly; it does not depend on HAL or LL.

