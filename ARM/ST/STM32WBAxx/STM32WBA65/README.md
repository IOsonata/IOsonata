# STM32WBA65 MCU library

Target: STM32WBA65RIV7 (STM32WBA65I-DK1 Discovery Kit).

`lib/ioc` holds the Debug/Release Cortex-M33 library configurations
(`STM32WBA65xx`), including the Bluetooth stack glue (see Bluetooth in the
library). It uses the shared IOsonata directory structure. Flash
and SRAM size do not define a separate library variant; applications select
their linker script. A different peripheral inventory or interrupt vector
layout requires a separate library configuration.

## BLE validation projects

Import the projects below from `exemples/<project>/ioc`. They link the common
application sources, the same WBA65 library and the same two ST archives as
the hardware-tested BleAdvertiser. Build the library and application with
matching Debug or Release configurations. These new projects have not yet
passed full target builds or hardware tests.

All use USART1 (DevNo 1), PA8 RX / PB12 TX, on the first ST-LINK virtual COM
port, 8N1 without flow control. Board headers contain mappings only.
Application entry is the shared source's standard `main()`.

| Project | Shared source | UART baud | Purpose |
|---|---|---|---|
| UartBleDemo | uart_ble.cpp | 115200 | Open-link peripheral, BlueIO UART writes and notifications |
| BleIntrfPrbsTx | bleintrf_prbs_tx.cpp | 115200 | BtIntrf PRBS notification stream, advertised name BlePrbs |
| BleCentralScanDemo | ble_central_scan.cpp | 115200 | Advertising reports, device names and RSSI |
| UartBleCentral | uart_ble_central.cpp | 1000000 | Find UARTDemo, connect, discover its service and CCCD, subscribe and exchange UART data |
| BleSecurityDemo | ble_security_demo.cpp | 115200 | Default Just Works bonding and protected UART service, advertised name BLESec |

UartBleCentral sets `BLE_SC_METHOD=0` in both configurations for the first
open-link test against UartBleDemo. Security belongs to the stack; this does
not change the common example's security selection on other boards.
BleSecurityDemo uses its existing default security policy. Numeric Comparison
and passkey UI are not configured by this project.

The UART peripheral/security examples map their optional UART-toggle button
to the already-tested external input PC13 (Arduino D4) with EXTI13. Leave it
open during data tests; momentarily grounding it toggles the UART. The DK1
joystick is analog and is not used as a GPIO button.

Hardware sequence:

1. Run UartBleDemo and connect to UARTDemo with an external BLE central.
   Discover the BlueIO UART service, subscribe to its notify characteristic,
   send UART bytes and receive them over BLE; write the BLE characteristic
   and verify UART output. Disconnect and reconnect.
2. Run BleIntrfPrbsTx, subscribe, and check the received PRBS stream.
3. Run BleCentralScanDemo and verify names/RSSI from known external advertisers.
4. Run UartBleCentral with an external IOsonata UARTDemo peripheral.
   Verify connection, service/characteristic/CCCD discovery, subscription,
   both transfer directions and automatic scan/reconnect after disconnect.
   This example does not establish characteristic-read coverage.
5. Run BleSecurityDemo; test pairing, protected data transfer, disconnect,
   reconnect and bond reuse after reset. Record unsupported security modes.

Maintainer hardware results on October 17, 2026: GPIO output/input and EXTI
work; UART PRBS TX at 1 Mbaud reports about 100139 B/s and loopback about
67150 B/s with zero errors; LPTIM and TIM callbacks were tested at 100 ms
and 1000 ms without an external frequency reference. Advertising and counter
updates pass on WBA65. GATT peripheral, central and security tests remain
pending. Earlier validation sections below describe historical checks.

## Integration status

The IOC library includes IOsonata `ResetEntry.c`, the WBA65 interrupt
vector table, a shared system initializer with the 100 MHz clock setup,
shared GPIO configuration,
the shared STM32 UART driver (`ARM/ST/src/uart_stm32.cpp`) and the shared
STM32 timer driver (`ARM/ST/src/timer_stm32.cpp`, `timer_lptim_stm32.cpp`,
`timer_tim_stm32.cpp`). The vector table follows STM32WBA65xx interrupt
order; its timer vectors are weak aliases of `Stm32TimerVector`, which runs
the timer driver when the application uses it. The library is built for the FPU
with the hard float ABI; the system initializer enables CP10 and CP11,
installs the vector base and sets the core clock (see Core clock below). It
does not configure TrustZone.

**The MCU port is not complete.** The UART and timer example projects below
build and link. Outstanding work:

- Verify startup and vector placement on hardware with the chosen security
  configuration
- Check the core clock setup on hardware (PLL lock, 100 MHz, flash wait states)
- Verify EXTI interrupt routing/dispatch and the timers on hardware
- Validate the application linker script against STM32WBA65 flash/SRAM banks
  and the configured secure/nonsecure partitions
- Hardware-test the integrated STM32CubeWBA radio middleware with the
  peripheral/central test projects listed above

Use the WBA65 device definitions and interrupt mapping from a consistent
STM32CubeWBA release. Do not compile WBA5x startup or link with the WBA5x
1 MB/128 KB linker script. The STM32WBA Bluetooth adapter in `../src/`
additionally requires the CubeWBA HAL and BLE application headers.
`bt_adv_stm32wba.cpp`, `bt_app_stm32wba.cpp`, `bt_gap_stm32wba.cpp` and
`bt_gatt_stm32wba.cpp` include `stm32wbaxx_hal.h`.

## Local validation on October 17, 2026

Checked the prerelease sources based on `7d23fb537d35313b61b1a97e223867ca6b780fa7`.
GNU Arm 14.3.1 was used for these checks:

- Minimal firmware using the actual ResetEntry, WBA65 system and vector files,
  standard C runtime, and `gcc_stm32wba65_i.ld`: Debug (-O0) and Release (-Os)
  compile/link passed with warnings treated as errors. Both reached main in
  the instruction set emulator, vectors at 0x08000000, initial stack at
  0x20080000, SystemCoreClock 100000000. This was not the full MCU library or
  a BLE firmware build and did not model TrustZone provisioning.
- `tests/stm32wba/run_clock.py`: 36 checks passed. The new failure cases
  produced eight failed assertions before the clock guards were added.
- `tests/stm32wba/run.py`: 155 checks passed at each of -O0 and -O2 under the
  undefined-behavior sanitizer; timer sources also compiled for Cortex-M33
  at -O0 and -Os with warnings treated as errors.
- Shared UART source compiled for WBA65 at -O0 and -Os with warnings treated
  as errors.
- `tests/stm32wba/run_i2c.py`: 40 emulator checks passed.
- `tests/stm32wba/run_spi.py`: 46 emulator checks passed.

The checks above were local. The maintainer subsequently confirmed Blinky LED
operation on STM32WBA65I-DK1 on October 17, 2026. Full library/example Debug and Release rebuilds,
radio clock/RNG startup, GPIO input/EXTI, and BLE peripheral/central operation
remain unverified in this review. The earlier build results below describe
previous checks, not a new complete build. BleAdvertiser is hardware-tested. The additional BLE projects listed above
still require target builds and GATT/central/security hardware validation.

The concurrent BleAdvertiser project fix in `8f9d5e0` moves the ST archives\nfrom Other linker flags into Libraries so they follow the application\nobjects. That change is preserved; a fresh full BLE build is still required.\n\n## GPIO first: Blinky

Start hardware validation with `exemples/Blinky/ioc`. It links the common
`exemples/misc/blinky.c` and uses the existing DK1 board map. Build the MCU
library and Blinky in the same Debug or Release configuration, then flash
Blinky through the configured debugger. No serial terminal is needed.

Expected sequence: both LEDs off, LD6 green (PD8) on for approximately 250 ms,
all off for approximately 250 ms, LD5 red (PD9) on for approximately 250 ms,
all off for approximately 250 ms, repeated. Both LEDs are active low. These
are software delays; this is GPIO output validation, not timer calibration.

The project uses nosys and does not require semihosting. Its board.h contains
only mappings. It does not configure UART, BLE, or a hardware timer. The DK1
joystick is an ADC resistor ladder. The external button mapping uses PC13,
Arduino D4 on CN10 pin 5, as an input with an internal pull-up and falling-edge
EXTI13. Wiring follows UM3462 Table 8.

Input/EXTI test:

1. Leave D4 disconnected and reset. The LEDs should alternate normally.
2. Momentarily connect CN10 pin 5 (D4/PC13) to CN9 pin 6 (GND).
3. Blinking should stop with both LEDs off after the current blink finishes
   (allow 500 ms). The common button callback sets `g_bBut1Pressed` true.
4. Remove the jumper and reset; blinking should resume. Repeat the test.

The example uses the interrupt callback, not input polling, to stop blinking.
This checks the pull-up and falling-edge interrupt path; it does not establish
rising-edge/both-edge behavior or the GPIO read API. Input/EXTI hardware results
are still pending. The callback's existing printf uses nosys; no UART output
is required for the LED indication.

The common Blinky source, GPIO driver, ResetEntry, system initializer and
vector table were compiled and linked locally with the WBA65 I linker script
at -O0 and -Os using GNU Arm 14.3.1, with warnings treated as errors. Both
images ran in the emulator: GPIOD clock enabled, PD8/PD9 configured as outputs,
and the expected alternating BSRR set/clear sequence observed. The full MCU
library was not rebuilt for this check. The maintainer confirmed LED operation
on the DK1 on October 17, 2026. Input/EXTI, UART, timers and BLE remain pending.

## UART example projects

`exemples/UartLoopback` (`exemples/uart/uart_loopback.cpp`) and
`exemples/UartPrbsTxTest` (`exemples/uart/uart_prbs_tx.cpp`) link
`IOsonata_STM32WBA65` from `lib/ioc` in the same Debug or Release
configuration and use `ldscript/gcc_stm32wba65_i.ld`. Build the library first.

Each `src/board.h` selects the board pin map with
`#define BOARD STM32WBA65I_DK1` (`ARM/ST/include/stm32_boards.h`): USART2,
DevNo 2, RX PA11 on Arduino D0 and TX PA12 on D1, AF3. UM3462 also names these
pins as the second ST-LINK virtual COM port. For USART1 on the first ST-LINK
virtual COM port (RX PA8, TX PB12) use `BOARD_VCP_DEVNO` and `BOARD_VCP_PINS`.

Both run at 1 Mbaud, 8N1, from the 100 MHz core clock. A character takes 1000
CPU cycles, so UartLoopback sets `UART_DMA_MODE` in its board.h and runs with
interrupts and DMA: RX on GPDMA1 channel 2 with the linked list ring, TX on
channel 3. UartPrbsTxTest uses the example defaults, DMA without interrupts,
one character per transfer (`BYTE_MODE`), starting with 0xFF followed by
`Prbs8()`.

The projects link `rdimon` semihosting like the F030 and L476 examples: run
them from a debugger with semihosting enabled.

Validation: the library and both projects were built in Debug and Release
from their `.project` and `.cproject` settings with arm-none-eabi-gcc 13.2.1, outside
IOcomposer. The four images were run in an instruction set emulator with a
USART2 and GPDMA1 model at 1 Mbaud: the loopback echoes in order with no lost
character, and the PRBS output matches `Prbs8()`. This does not replace a
hardware run on the Discovery Kit.

## Core clock

`SystemInit` (`ARM/ST/STM32WBAxx/src/system_stm32wbaxx.c`, run by ResetEntry)
sets the core clock to 100 MHz from PLL1R, following `HAL_RCC_OscConfig` and
`HAL_RCC_ClockConfig` of the ST HAL:

1. HSI16 with PLL1 off and the bus dividers cleared, the reset state.
2. HSE32 is started when `g_McuOsc.CoreOsc` is `OSC_TYPE_XTAL` or
   `OSC_TYPE_TCXO` at 32000000 Hz. If it does not start, `g_McuOsc` is set
   back to the internal RC and HSI16 is used. The default `g_McuOsc` is the
   internal RC, so the default source is HSI16. The DK1 has a 32 MHz crystal;
   an application overrides `g_McuOsc` to use it. The HSE trimming register is
   left at its reset value.
3. The dividers are searched for the highest PLL1R up to 100 MHz with the PLL
   input at 4 to 16 MHz and the VCO at 128 to 544 MHz, the lowest VCO for a tie.
   Both sources give 100 MHz: M 4, N 25, R 2 from HSE32 (VCO 200 MHz), M 2, N 25,
   R 2 from HSI16.
4. Voltage scaling range 1 (above 16 MHz), then the flash wait states (0 up to
   32 MHz, 1 up to 64, 2 up to 96, 3 up to 100), then PLL1 on.
5. The switch goes through the PLL1R two step divider as the ST HAL does, then
   the divider is released. HCLK, PCLK1, PCLK2 and PCLK7 are not divided.

If PLL1 does not lock the clock stays on HSI16. Startup also stops clock
reconfiguration when HSI readiness, the switch back to HSI, PLL shutdown or
either voltage-ready acknowledgement times out. It leaves the current clock
running and updates SystemCoreClock instead of proceeding to an unsafe step. `SystemCoreClockUpdate` decodes
the RCC registers. `SystemCoreClockSelect(OSC_TYPE_RC, 0)` or
`SystemCoreClockSelect(OSC_TYPE_XTAL, 32000000)` runs the setup again.
`SystemPeriphClockSet` sets the APB prescalers, index 0 PCLK1, 1 PCLK2, 2 PCLK7.

Validation: `tests/stm32wba/run_clock.py` runs the setup in an instruction set
emulator with a model of the RCC, PWR and flash registers that checks the order
of the steps, the PLL limits, the flash wait states against the frequency, the
switch rules of the PLL1R divider and the fall back when HSE does not start (36
checks, including failed voltage acknowledgements, a refused switch to HSI
and a PLL that does not stop). It does not model lock time. Not tested on hardware.

## Timer example project

`exemples/TimerDemo` (`exemples/timer/timer_demo.cpp`) links
`IOsonata_STM32WBA65` from `lib/ioc` in the same Debug or Release
configuration and uses `ldscript/gcc_stm32wba65_i.ld`. Build the library
first. `src/board.h` selects `#define BOARD STM32WBA65I_DK1`: Timer DevNo 0,
LPTIM1, from the LSI at 32 kHz (`TIMER_DEMO_DEVNO` 1 for LPTIM2, 2 to 7 for
TIM16, TIM17, TIM2, TIM3, TIM4 and TIM1 at 100 kHz), trigger 0 every 100 ms
toggles LD6 (PD8), trigger 1 every 1 s toggles LD5 (PD9). The
demo prints on the first ST-LINK virtual COM port, USART1 DevNo 1, RX PA8 and
TX PB12, AF7, 115200 8N1, interrupt mode. It links `nosys`, not semihosting.
Expected output:

```
Timer 0 trigger 0: 100 ms
Timer 0 trigger 1: 1000 ms
Count = 100 ms, T0 = 100093 us, T1 = 0 us
Count = 200 ms, T0 = 100000 us, T1 = 0 us
...
Count = 1002 ms, T0 = 100000 us, T1 = 1001250 us
```

The first periods count from the start of the timer; the next ones are
100000 and 1000000 us at 32 kHz.

Validation: the library and the project were built in Debug and Release
from their `.project` and `.cproject` settings with arm-none-eabi-gcc 13.2.1,
outside IOcomposer. The images ran 3.3 s in an instruction set emulator
with an LPTIM1, LPTIM2, RCC and USART1 model that flags the LPTIM register
writes the reference manual does not allow (CFGR with the timer enabled;
DIER, ARR, CCR1, CCR2 with it disabled or before the previous write is
confirmed) and backup domain writes without DBP: no violation, the output
above, the counter extended across its wrap at 2.048 s, LPTIM2 and the LSE
crystal as well. With the first write after ENABLE lost, the DIER write made
again starts the timer. The same runs were made with the shared timer
driver, where the LPTIM vectors reach the driver through `Stm32TimerVector`.
`tests/stm32wba/run.py` runs the LPTIM driver source on the host with the
device header, `tests/stm32/run_timer.py` the TIM with a register model. The
TIM were not run in the emulator. This does not replace a hardware run.

The application linker layouts are now provided in `ldscript/`:

- `gcc_stm32wba65_i.ld`: 2 MB flash, 512 KB SRAM (STM32WBA65RIV7).
- `gcc_stm32wba65_g.ld`: 1 MB flash, 256 KB SRAM.

Each reserves two 16 KB flash NVM partitions at the upper end of its
variant's flash. The memory boundaries require verification against the
chosen ST secure/nonsecure image layout, option bytes, and radio firmware
reservation before flashing. These linker scripts are not evidence of a
successful boot or a working BLE stack.

The shared GPIO implementation now supports input, output, alternate functions,
pull resistors, output type, pin speed, and atomic BSRR set/clear. GPIO EXTI0-EXTI15 now route callbacks by pin number; interrupt allocation uses
the fixed EXTI line matching that pin, and a line cannot be assigned to two
ports simultaneously. Blinky LED output was hardware-validated on October 17,
2026; GPIO input levels, pull resistors and EXTI remain pending.

The initial bring-up sequence is startup, GPIO, UART, Timer, then BLE.
Discovery Kit GPIO/USART wiring belongs in each example's `board.h`,
not in the library.

References: STM32WBA6 reference manual RM0515 and STM32CubeWBA:
https://github.com/STMicroelectronics/STM32CubeWBA

## I2C and example project

`ARM/ST/src/i2c_stm32.cpp` is the I2C driver shared by the STM32 families
that have the I2C v2 peripheral (WBA, L4, G4). The WBA instance table selects
the RCC bits, the interrupts and the GPDMA requests. DevNo 0 to 3 select
I2C1, I2C2, I2C3 and I2C4. I2C3 is on APB7, the others on APB1. The kernel
clock is PCLK. The timing register is computed from the requested rate
(100 kHz, 400 kHz, 1 MHz) and the rate set is the closest the dividers give.
An address probe in StartTx returns false when the device does not
acknowledge, as `i2c_scan.cpp` expects. Transfers longer than 255 bytes use
RELOAD. Error handling follows the ST HAL (`I2C_IsErrorOccurred`,
`I2C_Flush_TXDR`).

Master: polling, interrupt (`bIntEn`) and GPDMA (`bDmaEn`). The GPDMA
channels come from `gpdma_stm32wba.cpp`, which the UART shares, so UART and
I2C do not take the same channel. Slave: interrupt driven, one or two 7 bit
addresses, with the read and write request events of the other IOsonata I2C
slaves.

Limits: 7 bit addresses only, no general call, no DMA in slave mode.

`exemples/I2CScan` (`exemples/i2c/i2c_scan.cpp`) scans the bus on the board
I2C of `STM32WBA65I_DK1`: SDA PB1, SCL PB2, AF4 (Arduino D14 and D15). These
pins match the ST board support package for the DK1 (I2C1, SCL PB2, SDA PB1,
AF4). The scan result is printed on USART2 through the UART retarget.

`exemples/I2CMasterSlave` (`exemples/i2c/i2c_master_slave.cpp`) runs the master
on I2C1 and the slave on I2C2 of the same chip, AF4. Master SDA PB1 (D14) and
SCL PB2 (D15), slave SDA PB11 (D9) and SCL PB10 (D13). Wire D14 to D9 and D15
to D13 (UM3462 table 8). D13 also drives LED3 through R42, so use 4.7k
external pull-ups to 3.3V on the slave bus or remove R42. PB13 and PB14 (D3 and
D5) cannot be used: SB24 and SB22 tie them to the audio codec signals (a
hardware run showed PB13 held low). The example starts with a polling master and an interrupt slave; the flags are in
`board.h`.

Validation: the driver is checked by `tests/stm32wba/run_i2c.py` in an
instruction set emulator with an I2C model and a GPDMA model (63 checks: polling,
interrupt, DMA, slave). Not tested on hardware.

## SPI and example project

`ARM/ST/src/spi_stm32.cpp` is the SPI driver for the STM32 families with the
SPI peripheral that has CFG1, CFG2 and TSIZE (WBA; the same IP is in the U5,
H5 and N6). DevNo 0 to 2 select SPI1, SPI2 and SPI3. SPI3 is on APB7, SPI1 on
APB2, SPI2 on APB1. The kernel clock is PCLK. The rate is the closest the MBR
divider gives (2 to 256), and the rate set is returned. The data size is 4 to
16 bits, one byte per item up to 8 bits and two bytes per item above.

Master: each TxData or RxData call is one hardware transfer, in chunks of up
to 65535 items, as `HAL_SPI_TransmitReceive` does. The chip select is a GPIO
of the pin map (index `SPI_CS_IOPIN_IDX` and up), low from StartTx or StartRx
to StopTx or StopRx, so it stays low between the command and the data of a
read. With `SPICSEL_MAN`, or no chip select pin in the map, the application
drives it. Receive clocks out `DummyByte`. Polling, interrupt (`bIntEn`) and
GPDMA (`bDmaEn`, shared with the UART and I2C through `gpdma_stm32wba.cpp`).

Slave: interrupt driven, with or without DMA. The fourth pin of the map is the
NSS pin, used by the hardware. Each frame is announced by
`DEVINTRF_EVT_STATECHG` (give the buffers with `SetSlaveRxBuffer` and
`SetSlaveTxData`, index 0) and ended by `DEVINTRF_EVT_COMPLETED` with the byte
count, or -1 after an overrun or a frame longer than the receive buffer. The
`DummyByte` follows the transmit data.

Limits: `SPIPHY_NORMAL` only. The families with the older SPI registers (F0,
F4, L4, G4) keep their own SPI files.

`exemples/SPILoopback` (`exemples/spi/spi_loopback.cpp`) runs the four SPI
modes with MOSI wired to MISO: SPI2, AF5, SCK PB10 (D13), MISO PA9 (D12), MOSI
PC3 (D11), chip select PB9 (D10) as a GPIO (UM3462 table 8). Wire D11 to D12. The AF numbers come from the ST pin
data for the STM32WBA65RI, and the ST board support package does not use
these pins (it uses SPI3 on PA0, PA1, PB8 for the LCD and EEPROM). The result
is printed on USART1, the first ST-LINK virtual COM port (PA8 RX, PB12 TX,
AF7, which matches the ST board support package).

`exemples/SPIMasterSlave` (`exemples/spi/spi_master_slave.cpp`) runs the slave
on SPI2, AF5, on the Arduino pins of UM3462 table 8: SCK PB10 (D13), MISO PA9
(D12), MOSI PC3 (D11), NSS PB9 (D10). The master is the software SPI: SCK PB11
(D9), MOSI PB0 (D6, also drives LED3), MISO PC13 (D4), chip select PA10 (D8).
Wire D9 to D13, D6 to D11, D12 to D4 and D8 to D10.

Validation: `tests/stm32wba/run_spi.py` runs the driver in an instruction set
emulator with a model of the SPI registers, the GPDMA channels and the chip
select pins (64 checks: polling, interrupt, DMA, slave, a receive FIFO overrun
check and the 70000 byte chunking).
Not tested on hardware.

## Build dependency

The WBA65 device header is `../include/stm32wba65xx.h` in this repository.
The library and example include paths find it before the STM32CubeWBA paths.
The library build needs the `external/Fusion` checkout made by the IOsonata
installer for `att_xiot_fusion.cpp`, and the `external/STM32CubeWBA` and
`external/stm32-mw-wpan` checkouts for the Bluetooth sources (see External
middleware checkout). The shared system module sets the core clock to 100 MHz,
see Core clock.

The existing `gcc_arm_flash.ld` calls IOsonata `ResetEntry` and places
`__Vectors` in `.ivector`. Do not link the STM32CubeWBA GCC startup module
in the same image or two startup/vector definitions will conflict.

## External middleware checkout

The ST Bluetooth link layer is a closed library. Its platform files (`bleplat.c`,
`linklayer_plat.c`, `ll_sys_if.c`, `power_table.c`) are ST sources under the ST license, so they stay in the ST
repositories and are not copied into IOsonata. The library builds them from the
STM32CubeWBA checkout.

The IOsonata clone scripts in `Installer` clone both repositories at matching
versions, next to the other `external` repositories:

- `external/stm32-mw-wpan` at tag `v2.10.0`;
- `external/STM32CubeWBA` at tag `v1.10.0`, with only `Projects/Common/WPAN`, the
  NUCLEO-WBA65RI BLE_p2pServer project, `Utilities` and `Drivers`, and the HAL
  and CMSIS device submodules.

To clone them by hand:

```sh
cd ~/swdev/external
git clone --depth=1 --branch v2.10.0 https://github.com/STMicroelectronics/stm32-mw-wpan.git
git clone --depth=1 --branch v1.10.0 --filter=blob:none --sparse https://github.com/STMicroelectronics/STM32CubeWBA.git
cd STM32CubeWBA
git sparse-checkout set Projects/Common/WPAN Projects/NUCLEO-WBA65RI/Applications/BLE/BLE_p2pServer Utilities Drivers
git submodule update --init --depth 1 Drivers/STM32WBAxx_HAL_Driver Drivers/CMSIS/Device/ST/STM32WBAxx
```

Use the tags above, not the default branch of either repository. The two
releases belong together: STM32CubeWBA 1.10.0 uses STM32_WPAN 2.10.0. The IOC project uses `external/stm32-mw-wpan/ble/`
and `external/stm32-mw-wpan/link_layer/` for the BLE middleware headers, and
STM32CubeWBA for the platform files, the HAL headers and `Utilities/tim_serv`.
The library defines `PHY_40nm_3_40_a` and `BLE_STACK_BASIC_PLUS=1` and takes the
link layer configuration from `ll_cmd_lib/config/ble_basic_plus`, so the headers
match the Basic Plus link layer library (extended advertising included).

## Bluetooth adapter status

The adapter sources `bt_{app,adv,scan,gap,gatt,smp_bond,pds}_stm32wba.cpp`
were compile-checked against STM32CubeWBA 1.10.0 and the WPAN middleware
commit 7e764835 (the revision that release uses). The BleAdvertiser section
below records the earlier firmware link results. Hardware testing remains
pending.

The stack headers have no `extern "C"` guard, so the adapter includes them
through `include/bt_wba_stack.h`.

How the adapter works with the 1.10.0 stack:

- The host stack hands events to `BLECB_Indication`. The adapter copies each
  event into a ring of 8 slots and handles it from the Bluetooth event queue
  (`BtEvtQue`), because no stack function may be called from inside
  `BleStack_Process`.
- `BleStackCB_Process` and `HostStack_Process` are defined in `bt_app_stm32wba.cpp`
  and run `BleStack_Process` from the event queue. Do not build the Cube
  `host_stack_if.c`.
- The 1 second timeout check and the millisecond clock use LPTIM2 (Timer DevNo 1)
  through the IOsonata timer driver. The Cube timer server and the System Clock
  Manager are not used.
- `BtAppCfg_t::TxPower` is in dBm and is mapped to the closest PA level of the
  table in the STM32WBA BLE Wireless Interface document.
- The public address, identity root and encryption root are written to the
  stack at start. The default (`BtAppWbaIdentityGet`, weak) derives them from the
  chip unique number, so each chip has its own address, but the two keys are not
  secret. An application that bonds should override it.
- LE Enhanced Connection Complete, advertising reports and extended advertising
  reports are handled. A central or peripheral with `MaxMtu` above 23 starts the
  ATT MTU exchange.

Platform layer under the stack:

`src/bt_wba_platform.cpp` supplies the functions that the stack and the link
layer library call, so a build does not need the Cube HAL, sequencer, timer
server, RTC, memory manager or flash manager:

- `HAL_NVIC_*` and the radio baseband clock: NVIC and RCC registers.
- `UTIL_SEQ_RegTask` and `UTIL_SEQ_SetTask`: the tasks run from `BtEvtQue`.
- `BLE_TIMER_Start` and `BLE_TIMER_Stop`: 8 software slots on one LPTIM1 trigger
  (Timer DevNo 0). The expiry callbacks run from `BtEvtQue`.
- `HW_RNG_Get` and the RNG clock requests: RNG peripheral, HSI16 kernel clock,
  polled.
- `BAES_*`: the IOsonata `CryptoSoftAes` engine. Only ECB encrypt, CMAC and CCM
  are provided, which is what the stack calls. The host test vectors (FIPS-197,
  RFC 4493 and RFC 3610 packet vector 1) pass.
- PKA controller hooks (`PKACTRL_CB_Process`, mutex, semaphore, `PKA_IRQHandler`).
- `SCM_HSE_WaitUntilReady`, `scm_*` and the link layer debug signals.

Limits: the temperature given to the link layer for radio calibration is fixed
at 25 degrees C (set `USE_TEMPERATURE_BASED_RADIO_CALIBRATION` to 0 in
`app_conf.h`, then it is not used). AES is software. The RNG, timer, sequencer
and PKA hook code has only been compiled and linked, not run.

Cube sources that remain in a Bluetooth build (all from the STM32CubeWBA 1.10.0
package and the WPAN middleware commit 7e764835):

- Project Target files `bleplat.c` (it includes `ble_wrap.c`, so do not compile
  that one), `linklayer_plat.c`, `ll_sys_if.c`, `power_table.c`. Take them from
  the BLE only project (BLE_p2pServer), not from the audio projects.
- `link_layer/ll_sys/src/ll_sys_startup.c`, `ll_sys_intf.c`, `ll_sys_cs.c`,
  `ll_sys_dp_slp.c`.
- Libraries `ble/stack/lib/stm32wba_ble_stack_full.a` and
  `link_layer/ll_cmd_lib/lib/LinkLayer_BLE_Basic_Plus_V3_0.a`.

Build notes found while linking these with the adapter:

- Define `BLE_LL` for the link layer sources, or `ll_sys_ble_cntrl_init` is not
  compiled.
- The `bleplat.c` copy of `BLEPLAT_NvmStore` clashes with the adapter. Compile
  `bleplat.c` with `-DBLEPLAT_NvmStore=BleplatNvmStoreCube`.
- `bleplat.c` and `ble_wrap.c` need `-include cmsis_compiler.h` for
  `__PACKED_STRUCT`.
- Set `CFG_SCM_SUPPORTED` to 0 in `app_conf.h`.
- Link with `--gc-sections` and function sections.

The platform file also provides the `RADIO_IRQHandler` and `HASH_IRQHandler`
vectors the link layer uses (HASH is the software low priority radio
interrupt), and `BtWbaPlatformInit()`. It starts HSE32 and selects the radio
sleep timer clock before `BleStack_Init`: LSE when `g_McuOsc.LowPwrOsc.Type`
is not `OSC_TYPE_RC`, LSI1 otherwise. Without it the link layer stops on a
missing sleep clock source.

## Bluetooth in the library

The Debug and Release configurations of `lib/ioc` include everything the ST
Bluetooth stack needs, so an application project links the library and the two
ST prebuilt archives and holds only its own sources:

- the adapter sources `bt_{app,adv,scan,gap,gatt,smp_bond,pds}_stm32wba.cpp` and
  `bt_wba_platform.cpp`. The configurations exclude the HCI versions
  `bt_{adv,gap,gatt,scan}_hci.cpp`, which define the same functions;
- `nvm_stm32.cpp` for the bond storage;
- `src/bleplat_wrap.c`, which renames the Cube `BLEPLAT_NvmStore` and includes
  `bleplat.c`, so no per file compiler option is needed;
- the Cube and WPAN platform files `linklayer_plat.c`, `ll_sys_if.c`,
  `power_table.c` and the WPAN
  `ll_sys_startup.c`, `ll_sys_intf.c`, `ll_sys_cs.c`, `ll_sys_dp_slp.c`, found
  through the `external` folder (see External middleware checkout);
- `src/bluetooth/bt_wba_pka.cpp`, which answers the P-256 key requests of the
  stack (public key and Diffie-Hellman) with the IOsonata `CryptoUecc` engine.
  The Cube PKA controller (`pka_ctrl.c`, `hw_pka.c`, `hw_pka_p256.c`) is not
  built and the PKA peripheral is not used. The multiplication is software and
  runs in the event queue context;
- the headers `include/app_conf.h`, `app_common.h`, `utilities_conf.h`,
  `stm32_rtos.h`, `common_wpan_conf.h` and `main.h`. The ST platform files
  include them by name. They are written for IOsonata (MIT) and define only
  what the built files use: the radio and PKA interrupt numbers and
  priorities, the link layer task identifiers, the power table selection and
  the sequencer prototypes that `bt_wba_platform.cpp` implements. They are not
  copies of the ST BLE_p2pServer files, which are under the ST license.

The configurations define `STM32WBA65xx`, `BLE_LL`, `IOSONATA_STM32_WBA`,
`PHY_40nm_3_40_a` and `BLE_STACK_BASIC_PLUS=1`, and pass
`-include cmsis_compiler.h` to the C compiler. Do not define `USE_HAL_DRIVER`,
`stm32wbaxx.h` would then pull in the HAL and its macros clash with the IOsonata
UART names.


## Bluetooth example project

`exemples/BleAdvertiser/ioc` is the IOcomposer project for
`exemples/bluetooth/ble_advertiser.cpp`. The project holds `ble_advertiser.cpp`
and `src/board.h` only. It links `IOsonata_STM32WBA65` from `lib/ioc/Debug`
or `lib/ioc/Release` (build the same configuration first) and the two ST stack
archives `stm32wba_ble_stack_full.a` and
`LinkLayer_BLE_Basic_Plus_V3_0.a` from `external/stm32-mw-wpan`.

Both Debug and Release configurations compile and link with arm-none-eabi
(Release about 234 KB code, 83 KB RAM). No test was run on hardware. The DK1 I2C
pins (PB1, PB2 AF4) and the VCP pins match the ST board support package. The SPI2
pins (PB10, PA9, PC3 AF5, CS PB9) and the second I2C and SPI pins of the
MasterSlave examples come from the ST pin data for the STM32WBA65RI and UM3462 table 8.

Still to do: ADC based temperature, use of the AES peripheral, and the
hardware test of the stack, RNG, timer and sequencer code.

## First hardware test of BleAdvertiser

1. Build the library project (Debug), then BleAdvertiser (Debug), flash the
   DK1 through the ST-LINK.
2. Open the first virtual COM port at 115200 baud. The example prints the
   `BleStack_Init` result and advertising state through SysLog.
3. A phone scanner (nRF Connect or similar) should show the advertiser with
   manufacturer data that increments on the advertising timeout.
4. The sleep clock is LSI1 by default (`g_McuOsc` is RC). To use the 32.768 kHz
   crystal, define `MCU_OSC` in `board.h` as
   `{{OSC_TYPE_XTAL, 32000000, 20}, {OSC_TYPE_XTAL, 32768, 20}, false}`. If the
   crystal does not start, `BtWbaPlatformInit` falls back to LSI1.
5. If nothing is advertised, check in this order: HSE32 ready (`RCC->CR`),
   `RCC->BDCR1` RADIOSTSEL not zero, the RNG answering, and that the RADIO and
   HASH vectors are reached.

