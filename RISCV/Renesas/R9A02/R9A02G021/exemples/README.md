# R9A02G021 examples

Import the library project `../lib/ioc` (`IOsonata_R9A02G021`), then the
example's `ioc` directory with **Existing Projects into Workspace**. Build the
library configuration (Debug or Release) before the example: the example
links that configuration's archive. All projects use RV32IMAC with Zba, Zbb,
Zbs, Zicsr and Zifencei, ilp32 ABI, GNU C17 and C++23, newlib-nano with nosys.

| Directory | Shared application source | Linker script | Setting |
| --- | --- | --- | --- |
| `Blinky` | `exemples/misc/blinky.c` | `gcc_R9A02G021.ld` | LED1, LED2, switch S1 on IRQ4 |
| `TimerDemo` | `exemples/timer/timer_demo.cpp` | `gcc_R9A02G021.ld` | Timer DevNo 0, default frequency |
| `UartLoopback` | `exemples/uart/uart_loopback.cpp` | `gcc_R9A02G021.ld` | UART0, 115200 8N1, interrupt mode |
| `UartPrbsTxTest` | `exemples/uart/uart_prbs_tx.cpp` | `gcc_R9A02G021.ld` | UART0, 115200 8N1, interrupt mode |
| `UartaLoopback` | `exemples/uart/uart_loopback.cpp` | `gcc_R9A02G021.ld` | UARTA1 (DevNo 4), 115200 8N1, interrupt mode, linker option `-Wl,-u,R9a02UartaInit` |
| `SpiLoopback` | `exemples/spi/spi_loopback.cpp` | `gcc_R9A02G021.ld` | SPI20 (DevNo 4) at 1 MHz, the four modes, interrupt and DTC reception, console on UART0 |
| `I2CScan` | `exemples/i2c/i2c_scan.cpp` | `gcc_R9A02G021.ld` | IICA1 (DevNo 1) at 100 kHz, HOCO 32 MHz, console on UART0 |
| `ADCDemo` | `src/adc_demo.cpp` (this port) | `gcc_R9A02G021.ld` | ANI2, ANI3, internal reference, temperature sensor, console on UART0 |
| `RtcDemo` | `src/rtc_demo.cpp` (this port) | `gcc_R9A02G021.ld` | RTC on the 32.768 kHz crystal, Software Standby, console on UART0 |
| `AnalogDemo` | `src/analog_demo.cpp` (this port) | `gcc_R9A02G021.ld` | TML32 interval, DAC8 channel 1 ramp, comparator 0 on the internal reference, console on UART0 |
| `AdcSnoozeDemo` | `src/adc_snooze_demo.cpp` (this port) | `gcc_R9A02G021.ld` | ANI2 converted in Snooze every 100 ms, TML32, DTC, ELC, wake-up above 1.65 V, console on UART0 |
| `DfuBoot` | `exemples/dfu/dfu_boot_main.cpp`, `dfu_boot_key.c` | `dfu_boot_r9a02g021.ld` | UART0, button P108 |

Application sources are linked resources, not copies. Each example's
`src/board.h` holds its pin and clock settings.

## FPB-R9A02G021 pins

From the FPB-R9A02G021 user's manual, default jumpers:

| Function | MCU pin | Note |
| --- | --- | --- |
| LED1 | P107 | Active high |
| LED2 | P100 | Active high |
| Switch S1 | P108, IRQ4 | Pull-up, falling edge |
| UART0 TX (TxD0_B) | P302 | J-Link OB virtual COM port, J5 and J6 at 2-3 |
| UART0 RX (RxD0_B) | P303 | J-Link OB virtual COM port |

UART pins use `IOPINOP_FUNC16` (PSEL 0x11, SAU). P300 and P301 are the
debug pins (TCKC, TMSC), keep them free.

The pins of the new examples are on the Arduino header and the Pmod
connectors, from the FPB-R9A02G021 quick start guide. The board user's manual
was not available here: check them against the board schematic.

| Example | Function | MCU pin | PinOp |
| --- | --- | --- | --- |
| UartaLoopback | RxDA1, TxDA1 | P008, P403 | `IOPINOP_FUNC5` (PSEL 0x06) |
| SpiLoopback | SCK20, SI20 (MISO), SO20 (MOSI) | P101, P102, P103 | `IOPINOP_FUNC16` |
| SpiLoopback | Chip select | P104 | GPIO |
| I2CScan | SDAA1, SCLA1 | P010, P011 | `IOPINOP_FUNC6` (PSEL 0x07), open drain |
| ADCDemo | ANI2, ANI3 (A2, A3) | P400, P401 | `IOPINOP_FUNC31` (analog) |
| AnalogDemo | DACOUT1, IVCMP0, wired together | P001, P106 | Analog, set by the drivers |
| AdcSnoozeDemo | ANI2 (A2) | P400 | `IOPINOP_FUNC31` (analog) |

The library clock default is kept in every example: HOCO 48 MHz core clock,
LOCO low frequency clock. `board.h` shows how to select the 32.768 kHz crystal
fitted on the board with `MCUOSC`.

## What to observe

**Blinky:** the LEDs are on after start, each one goes off for 250 ms in turn.
Pressing S1 stops the sequence (`g_bBut1Pressed`). The shared example prints
from the button handler; no console is configured, the output is dropped.

**TimerDemo:** see `g_TimerInitOk`, `g_TriggerPeriod[]`, `g_Period[]` and
`g_TriggerCount[]` in the debugger. LED1 toggles on trigger 0. DevNo 0 (machine
timer, 32768 Hz) and DevNo 1 to 8 (TAU channels 0 to 7) each have one trigger,
so only the 100 ms period is used. For a TAU channel, set `TIMER_DEVNO` to 1 to
8 and `TIMER_FREQ` to a frequency that holds 100 ms in 16 bits, for example
46875 (PCLKB 48 MHz / 1024); the default frequency (PCLKB) gives at most
1.37 ms and the trigger is rejected.

**UartLoopback:** terminal at 115200 8N1, no flow control, on the J-Link OB
virtual COM port. The firmware sends `UART Loopback Test` and echoes what it
receives.

**UartPrbsTxTest:** sends the `Prbs8` sequence at 115200 8N1, starting with
0xFF, no banner.

**UartaLoopback:** connect a 3.3 V serial adapter to P008 (RX) and P403
(TX), terminal at 115200 8N1. The firmware echoes what it receives. The
project links UARTA with `-Wl,-u,R9a02UartaInit` (Other linker flags).

**SpiLoopback:** connect P103 (MOSI) to P102 (MISO). The console on the
virtual COM port (Release configuration, `NDEBUG`) shows each mode and
pattern with PASS or FAIL, then `SPI master loopback PASS`. `board.h` sets
`SPI_MASTER_INT_ENABLE` and `SPI_MASTER_DMA_ENABLE`: the receptions run in
the background with the DTC. Set both false for polling, or only DMA for the
DTC with the call waiting.

**I2CScan:** connect I2C devices with pull-ups to P010 (SDA) and P011 (SCL).
The console lists the addresses that answer. The core clock is the HOCO at
32 MHz, the IICA needs PCLKB at 32 MHz or lower. `I2C_SCAN_INT_ENABLE true`
reads the register of `I2C_SCAN_REG_DEVADDR` in the interrupt. `board.h`
gives the pin map for the SAU simplified I2C IIC20 (DevNo 6, P101 SCL, P102
SDA).

**AnalogDemo:** wire P001 to P106. The TML32 (LOCO) steps DAC8 channel 1 every
20 ms, a ramp from 0 to VCC in about 2.6 s. Comparator 0 compares it with its
internal reference: the console shows `Above at code 112, 1443 mV` or near on
the way up, `Below at code 0` when the ramp starts again, LED2 follows the
comparator output and LED1 toggles at each event. The internal reference is
1.34 to 1.54 V.

**AdcSnoozeDemo:** apply a voltage to A2 (P400). The MCU stays in Software
Standby; every 100 ms the TML32 starts Snooze, the conversion of A2 runs
without the CPU. At 1.65 V or more the CPU wakes up, prints the voltage,
toggles LED1 and waits one second before the next Software Standby. Below
1.65 V nothing is printed and the current stays low. The debugger connection
may be lost in Software Standby.

**ADCDemo:** every second the console shows the voltage of A2 and A3, the
internal reference with the VCC it gives, and the die temperature from the
factory calibration point. LED1 toggles. `ADC_DEMO_INT_ENABLE` in `board.h`
selects the interrupt mode.

**RtcDemo:** the 32.768 kHz crystal is the low frequency clock. The RTC
wakes the MCU from Software Standby every second: the console shows the date
and time (the build time after a power-on reset) and LED1 toggles. An alarm
every 2 minutes toggles LED2. `RTC_DEMO_STANDBY false` in `board.h` uses
Sleep instead. The debugger connection may be lost in Software Standby.

**DfuBoot:** stage 0 boot for the layout of `ldscript/dfu_layout_r9a02g021.ld`
(boot 36 KB, record 4 KB, application slot from 0xA000). Build the Release
configuration: at -O0 (Debug) the boot does not fit in 36 KB. The Debug
configuration defines `DFU_BOOT_ALLOW_UNSIGNED`. An application for the slot
links with `gcc_R9A02G021_dfu.ld`.

## Status

All twelve examples build and link with xPack riscv-none-elf-gcc 15.2.0
(October 13, 2026). Sizes at -Os (text + data, KB of 1000 bytes): Blinky
11.8, TimerDemo 19.8, UartLoopback 15.0, UartPrbsTxTest 15.0, UartaLoopback
18.6, SpiLoopback 26.8, I2CScan 31.5, ADCDemo 34.0, RtcDemo 24.9, AnalogDemo
29.1, AdcSnoozeDemo 38.2, DfuBoot 36574 bytes of the 36864 of its region.
I2CInit links the SAU simplified I2C with the IICA. None has been run on the
board yet.
