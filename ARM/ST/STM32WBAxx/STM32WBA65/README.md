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
matching Debug or Release configurations. BleAdvertiser and BleSecurityDemo
are hardware validated. The other BLE projects require hardware testing.

All use USART1 (DevNo 1), PA8 RX / PB12 TX, on the first ST-LINK virtual COM
port, 8N1 without flow control. Board headers contain mappings only.
Application entry is the shared source's standard `main()`.

| Project | Shared source | UART baud | Purpose |
|---|---|---|---|
| UartBleDemo | uart_ble.cpp | 1000000 | Open-link peripheral, BlueIO UART writes and notifications |
| BleIntrfPrbsTx | bleintrf_prbs_tx.cpp | 115200 | BtIntrf PRBS notification stream, advertised name BlePrbs |
| BleCentralScanDemo | ble_central_scan.cpp | 1000000 | Advertising reports, device names and RSSI |
| UartBleCentral | uart_ble_central.cpp | 1000000 | Find UARTDemo, connect, discover its service and CCCD, subscribe and exchange UART data |
| BleSecurityDemo | ble_security_demo.cpp | 115200 | LE Secure Connections, MITM, Numeric Comparison, bonding and protected UART service, advertised name BLESec |

UartBleCentral sets `BLE_SC_METHOD=0` in both configurations for the first
open-link test against UartBleDemo. Security belongs to the stack; this does
not change the common example's security selection on other boards.
BleSecurityDemo selects LE Secure Connections with MITM and DisplayYesNo.
Numeric Comparison uses the UART confirmation described below.

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
updates pass on WBA65. BleSecurityDemo is hardware validated as of October 18,
2026. Central scanning, connection, UART service discovery, notification data and
scan/reconnect are hardware validated. Explicit characteristic reads remain
unvalidated.
Earlier sections below retain the compiler and emulator test results.

## Crypto hardware validation

STM32WBA65I-DK1, October 18, 2026: **51 passed, 0 failed**.

The WBA crypto engines in `../src/crypto_stm32wba.cpp` use the RNG, AES and
PKA peripherals. The hardware test passed:

- Engine construction and AES/P-256 self-tests.
- RNG draws, odd-length buffer bounds, a 4 KB sample distribution check and
  the source's security-grade capability flag.
- AES ECB, CBC and CTR known-answer tests; CTR carry, wrap and partial blocks;
  invalid block lengths and key-usage rejection.
- CMAC, CCM and GCM known-answer tests; authenticated decryption rejects bad
  tags and clears the output.
- AES ECB/CBC/CTR/CMAC/CCM/GCM comparisons against software, 200 random rounds.
- P-256 public keys and ECDH shared secrets; consumed-key handling and rejection
  of invalid peer points and private scalars.
- ECDSA signing, verification in both hardware/software directions, and
  rejection of modified signatures and hashes.
- P-256 public-key, shared-secret and signature comparisons, 8 random rounds.

P-256 scalar multiplication, mean of 8: **31072 us** on the engine and
**188099 us** in software (6.05 times faster).

These results cover the standalone crypto engines. The ST BLE adapter uses the
AES and PKA engines. On the DK1, BleSecurityDemo paired with numeric comparison
and reconnected encrypted from the stored bond after reset. The prompt took
197 ms in the recorded run; reconnect encryption took 45 ms. The adapter also runs on the instruction
set emulator (`tests/stm32wba/run_bt_crypto.py`).

## USB CDC hardware validation

STM32WBA65I-DK1 native USB (CN5), October 18, 2026:

| Application | Build | Result |
|---|---|---|
| UsbCdcPrbsTx | Debug | Approximately 194.5 kB/s |
| UsbCdcPrbsTx | Release | Approximately 649 kB/s |
| UsbCdcLoopback | Release, 60 seconds | PASS: 72094720 bytes sent and received, 0 errors, 0 pending |

Loopback averaged 1201554.01 B/s TX and 1201451.62 B/s RX. First RX was
3 ms after the test started. The longest TX wait was 130 ms and the longest
RX gap was 129 ms. The host received the banner after toggling DTR.

Enumeration and CDC TX/RX are hardware validated. Cable reconnect,
suspend/resume, explicit FS fallback and other USB classes remain untested.
The host log did not record negotiated USB speed. The serial baud argument
does not set native USB throughput. See [STM32 USB](../../USB.md) for the
driver's supported modes and test commands.

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
- Validate the application linker script against STM32WBA65 flash/SRAM banks
  and the configured secure/nonsecure partitions
- Validate explicit BLE characteristic reads; scan/connect/discovery,
  notification data and reconnect have passed on hardware
- Validate USB cable reconnect, suspend/resume, FS fallback and other classes

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

The checks above used the compiler and emulator. Subsequent DK1 hardware
validation covers GPIO output/input and EXTI, UART TX/RX, LPTIM and TIM
callbacks, I2C and SPI master/slave, BLE advertising and BleSecurityDemo.
Crypto, USB CDC and BLE central results are recorded above. Explicit BLE
characteristic reads and the remaining USB modes still require hardware testing.

BleAdvertiser links the ST archives through the Libraries setting so they
follow the application objects.

## GPIO: Blinky

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
rising-edge/both-edge behavior or the GPIO read API. The falling-edge EXTI
callback is hardware validated. The callback's existing printf uses nosys; no UART output
is required for the LED indication.

The common Blinky source, GPIO driver, ResetEntry, system initializer and
vector table were compiled and linked locally with the WBA65 I linker script
at -O0 and -Os using GNU Arm 14.3.1, with warnings treated as errors. Both
images ran in the emulator: GPIOD clock enabled, PD8/PD9 configured as outputs,
and the expected alternating BSRR set/clear sequence observed. The full MCU
library was not rebuilt for this check. The maintainer confirmed LED operation
on the DK1 on October 17, 2026. GPIO input/EXTI, UART TX/RX, timer callbacks,
BLE advertising and BleSecurityDemo have also passed hardware testing.

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
reservation before flashing.

The shared GPIO implementation now supports input, output, alternate functions,
pull resistors, output type, pin speed, and atomic BSRR set/clear. GPIO EXTI0-EXTI15 now route callbacks by pin number; interrupt allocation uses
the fixed EXTI line matching that pin, and a line cannot be assigned to two
ports simultaneously. LED output and external input/EXTI are hardware validated.
Pull-resistor configurations and other interrupt edge modes were not tested separately.

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

Limits: 7 bit addresses only, no general call. In slave mode a GPDMA channel pair moves the data of a request into the buffer that the application installs from the request event; bytes past the buffer are dropped on a write and read as 0xFF on a read.

`exemples/I2CScan` (`exemples/i2c/i2c_scan.cpp`) scans the bus on the board
I2C of `STM32WBA65I_DK1`: SDA PB1, SCL PB2, AF4 (Arduino D14 and D15). These
pins match the ST board support package for the DK1 (I2C1, SCL PB2, SDA PB1,
AF4). The scan result is printed on USART2 through the UART retarget.

`exemples/I2CMasterSlave` (`exemples/i2c/i2c_master_slave.cpp`) runs the master
on I2C1 and the slave on I2C2 of the same chip, AF4. Master SDA PB1 (D14) and
SCL PB2 (D15), slave SDA PB11 (D9) and SCL PB10 (D13). Wire D14 to D9 and D15
to D13 (UM3462 table 8), the same wires as SPIMasterSlave uses. D13 also drives LED3 through R42, so use 4.7k
external pull-ups to 3.3V on the slave bus or remove R42. PB13 and PB14 (D3 and
D5) cannot be used: SB24 and SB22 tie them to the audio codec signals (a
hardware run showed PB13 held low). One run tests the master in polling,
interrupt, DMA and DMA with interrupt mode against the slave in interrupt and
DMA mode (8 combinations) and prints a summary line. `I2C_TEST_MASTER_MASK` and
`I2C_TEST_SLAVE_MASK` in `board.h` restrict the set.

Validation: the driver is checked by `tests/stm32wba/run_i2c.py` in an
instruction set emulator with an I2C model and a GPDMA model (70 checks: polling,
interrupt, DMA, slave in interrupt and DMA mode). On the DK1 the I2CScan example found the codec at 0x1A,
and the I2CMasterSlave example passed all 8 combinations in one run: the master
in polling, interrupt, DMA and DMA with interrupt mode against the slave in
interrupt and DMA mode (I2C1 to I2C2, 100 kHz).

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
`DummyByte` follows the transmit data. The frame ends when NSS goes inactive:
the peripheral raises no EOT for it while the transfer size is open, so the
driver watches the NSS pin (an EXTI line, DevNo of the pin number) and also
accepts the EOT flag. The EXTI line of the NSS pin must be free. While the
transmit FIFO is empty TXC stays set and EOTIE raises the interrupt for it, so
the handler puts one underrun item in the FIFO each time TXC is set with no
data left.

Limits: `SPIPHY_NORMAL` only. The families with the older SPI registers (F0,
F4, L4, G4) keep their own SPI files.

`exemples/SPILoopback` (`exemples/spi/spi_loopback.cpp`) runs the four SPI
modes with MOSI wired to MISO: SPI2, AF5, SCK PB10 (D13), MISO PA9 (D12), MOSI
PC3 (D11), chip select PB9 (D10) as a GPIO (UM3462 table 8). Wire D11 to D12. The AF numbers come from the ST pin
data for the STM32WBA65RI, and the ST board support package does not use
these pins (it uses SPI3 on PA0, PA1, PB8 for the LCD and EEPROM). The result
is printed on USART1, the first ST-LINK virtual COM port (PA8 RX, PB12 TX,
AF7, which matches the ST board support package).

`exemples/SPIMasterSlave` (`exemples/spi/spi_master_slave.cpp`) runs the master
on SPI3, AF6, and the slave on SPI2, AF5, on the Arduino pins of UM3462
table 8. Master: SCK PA0 (A5), MISO PA1 (A3), MOSI PC3 (D11), chip select PA10
(D8, a GPIO). Slave: SCK PB10 (D13), MISO PA9 (D12), MOSI PB0 (D6, also drives
LED3), NSS PB9 (D10). Wire A5 to D13, D11 to D6, A3 to D12 and D8 to D10.
One run tests the master in polling, interrupt, DMA and DMA with interrupt mode
against the slave in interrupt and DMA mode, in each SPI mode, with frames of 1,
16 and 129 bytes (`SPI_TEST_MASTER_MASK`, `SPI_TEST_SLAVE_MASK` and
`SPI_TEST_MODE_MASK` in `board.h` restrict the set).

Connectors (MB2143 schematic, checked on the DK1): CN8 pin 9 is D15 and pin 10
is D14, pins 1 to 6 are D8 to D13; CN10 pins 1 to 8 are D0 to D7; A3 and A5 are
CN11 pins 4 and 6. UM3462 table 8 lists D15 and D14 on pins 10 and 9, the
opposite of what the DK1 did. The I2CMasterSlave example adds D14 to D9
(CN8 10 to CN8 2) and needs D15 to D13 (CN8 9 to CN8 6) for SCL, while the SPI
example needs A5 to D13 (CN11 6 to CN8 6) for SCK. Two wires cannot share the
D13 socket, so one wire stays on CN8 6 and its other end moves between CN11 6
(SPI) and CN8 9 (I2C). The other five wires stay for both examples. The SPI wiring before this change used the SPISoft master on
other pins (SCK PB2, MOSI PB0, MISO PC13) and passed on the DK1 in all four SPI
modes with the slave in interrupt and in DMA mode.

Validation: `tests/stm32wba/run_spi.py` runs the driver in an instruction set
emulator with a model of the SPI registers, the GPDMA channels and the chip
select pins (64 checks: polling, interrupt, DMA, slave, a receive FIFO overrun
check and the 70000 byte chunking).
Hardware: SPIMasterSlave passed all 8 combinations on the DK1 (SPI3 master in
polling, interrupt, DMA and DMA with interrupt against the SPI2 slave in
interrupt and DMA mode), in all four SPI modes with frames of 1, 16 and 129
bytes, TX and RX. The 500000 Hz request ran at 390625 Hz (kernel clock divider).
Earlier runs with the SPISoft master also passed. SPILoopback is not tested.

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
below records the earlier firmware link results. The maintainer has since
validated advertising and BleSecurityDemo on hardware (October 18, 2026).

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
- `BAES_*`: the IOsonata `CryptoWbaAes` engine (AES peripheral, polled). Only
  ECB encrypt, CMAC and CCM are provided, which is what the stack calls. A call
  that arrives while the AES block is busy, a radio interrupt during a task
  level CCM for example, runs on `CryptoSoftAes` for that call. The host test
  vectors (FIPS-197, RFC 4493 and RFC 3610 packet vector 1) pass.
- PKA controller hooks (`PKACTRL_CB_Process`, mutex, semaphore, `PKA_IRQHandler`).
- `SCM_HSE_WaitUntilReady`, `scm_*` and the link layer debug signals.

Limits: the temperature given to the link layer for radio calibration is fixed
at 25 degrees C (set `USE_TEMPERATURE_BASED_RADIO_CALIBRATION` to 0 in
`app_conf.h`, then it is not used). The RNG, timer, sequencer
and PKA hooks are exercised by the BLE examples. Separate failure-path tests
for these hooks remain pending.

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
  stack (public key and Diffie-Hellman) with the IOsonata `CryptoWbaPka` engine
  (PKA peripheral, polled). The Cube PKA controller (`pka_ctrl.c`, `hw_pka.c`,
  `hw_pka_p256.c`) is not built. The operation runs in the event queue
  context;
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

Both Debug and Release configurations previously compiled and linked with
arm-none-eabi (Release about 234 KB code, 83 KB RAM). Advertising and counter
updates have since passed on hardware. The DK1 I2C
pins (PB1, PB2 AF4) and the VCP pins match the ST board support package. The SPI2
pins (PB10, PA9, PC3 AF5, CS PB9) and the second I2C and SPI pins of the
MasterSlave examples come from the ST pin data for the STM32WBA65RI and UM3462 table 8.

Remaining work: ADC-based radio temperature measurement and explicit BLE
characteristic-read tests.
The standalone crypto engines are hardware validated. The ST BLE adapter uses
them; pairing and a bonded reconnect ran on the DK1 with BleSecurityDemo.

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


## BleSecurityDemo pairing

Both Debug and Release select LE Secure Connections with MITM and bonding
(`BTGAP_SECTYPE_LESC_MITM`), with DisplayYesNo capability. Legacy pairing
fallback is disabled. Open the ST-LINK USART1 VCP at 115200 baud. Compare the
six-digit number printed there with the central's number; type `y` to accept
only when they match, or `n` to reject, and answer on the central as well.
UART confirmation bytes are consumed by the demo while a decision is pending.
The existing button confirmation used by other boards is unchanged.

Build the library and example in the same configuration. Security setup now
fails if ST rejects authentication requirements or IO capability. The maintainer
confirmed BleSecurityDemo hardware validated on October 18, 2026. The branch
includes the bonded reconnect fix. Rejection and other cases not explicitly
reported remain unvalidated. Use UartBleDemo for an open connection.

### Bonded reconnect after reset

Hardware validated on October 18, 2026:

- Fresh Numeric Comparison pairing succeeded and encryption was enabled.
- PDS saved 88 bytes with CRC `89681f20`.
- After reset, PDS restored 88 bytes with the same CRC.
- Peer lookup returned status `0x00`; encryption and pairing completed with
  status `0x00`, without another Numeric Comparison prompt.
- The initial prompt appeared 197 ms after the security request
  (7497 to 7694 ms). On reconnect, encryption was enabled 45 ms after the
  request (5622 to 5667 ms).

Hardware retest reported after the review fixes on October 18, 2026:
fresh Numeric Comparison pairing, a notification, reset and bonded reconnect
passed. nRF Connect reported mode 1, level 4 on fresh pairing and both
subsequent reconnects, without another Numeric Comparison prompt.

| Central log | Connected | Mode 1, level 4 | Elapsed |
|---|---|---|---|
| First reconnect | 18:01:06.968 | 18:01:07.017 | 49 ms |
| Second reconnect | 18:01:17.402 | 18:01:17.452 | 50 ms |

The first disconnect was reported as connection timeout; the second was a
local host disconnect. Both subsequent connections restored security. The
device log contains a second boot banner. Build configuration and exact
flashed commit were not recorded in the supplied log.

## Additional UART examples

The following projects link the existing shared sources under `exemples/uart`.
Import their `ioc` directories into IOcomposer and build the matching Debug or
Release configuration of `lib/ioc` first. All three select USART1 (DevNo 1),
PA8 RX / PB12 TX, through the ST-LINK VCP at 1000000 baud, 8N1, no flow control.
`board.h` contains only the board and pin mappings.

| Project | Shared source | Purpose |
|---|---|---|
| UartRetargetDemo | uart_retarget_demo.cpp | UART stdin/stdout with printf and scanf; type a word and press Enter to receive its echo |
| UartSlipPrbsTx | uart_slip_prbs_tx.cpp | Interrupt-driven UART transmitting 92-byte PRBS payloads with SLIP framing |
| UartSlipPrbsRx | uart_slip_prbs_rx.cpp | Interrupt-driven UART receiving SLIP frames and checking the decoded PRBS stream |

UartRetargetDemo deliberately routes both Debug and Release console I/O to
UART. The SLIP projects reserve UART for binary data and do not retarget stdio
to that stream. Debug links rdimon; Release links nosys. Debug console output
depends on the existing library stdio/semihosting configuration; if unavailable,
inspect `lcnt` in TX or `pkcnt` and `errcnt` in RX with the debugger.
Release provides no SLIP diagnostic console.

For a SLIP test, use a host SLIP/PRBS peer through the VCP, or a second board
running the complementary example with crossed TX/RX and common ground on
suitable exposed UART pins. Raw UART PRBS scripts do not decode SLIP framing.
The receive test should accumulate frames with zero PRBS errors. These new
projects compile and link in Debug and Release; hardware testing remains pending.

As of October 18, 2026, the maintainer reports BleSecurityDemo, TimerDemo,
SPI and I2C hardware validated. The I2C and SPI mode matrices above list the
tested configurations. Additional UART examples remain untested.

## Final candidate validation - October 18, 2026

Central scanning, connection, UART service discovery, notification data and
scan/reconnect are hardware validated. Explicit characteristic reads remain
unvalidated.
The central log discovered UARTDemo at RSSI -46, connected on handle 0 and
found RX `0x0010` and TX `0x0014`. After disconnect it resumed scanning,
connected again and rediscovered the service. Received UART text was shown;
the maintainer confirmed the data test passed.

A further hardware retest after the two-bank bond-storage change saved and
restored 88 bytes with matching CRC `d551074e`. Fresh Numeric Comparison
pairing succeeded; after reset, peer lookup, encryption and pairing returned
status `0x00` without another confirmation. Reconnect encryption took 45 ms
(4723 to 4768 ms). The supplied log does not identify the build configuration
or flashed commit. Physical power interruption during a save was not tested.

All 20 WBA examples compile and link in Debug and Release using Arm GCC
13.2.1 and the project settings at `e721268478c5475f018aaa36259c2c605af8ee54`.
The full library (276 translation units per configuration) was built earlier;
the subsequently changed bond object was rebuilt and both archives relinked.
Eclipse itself and the maintainer's GCC 15.2.1 were not run here.

## Onboard OLED

`exemples/LCDConsoleDisplayDemo/ioc` builds the existing console display demo in Debug
and Release. It uses SPI3 (DevNo 2), SCK PA0 and MOSI PB8 at AF6, with CS
PE1, D/C PE0 and reset PE3. No external wiring is needed. Rebuild the
matching library first; its Eclipse project includes the generic driver.
The shared `exemples/display/console_display_demo.cpp` cycles font samples,
scrolling and orientation. Fonts too large for the panel are skipped.

The driver and demo build and link with Arm GCC 13.2.1 in both configurations.
SPI display-model tests pass at `-O0` and `-Os` with ASan/UBSan. The maintainer
confirmed that the console display demo works on the board. See [SSD1315 usage](../../../../docs/display/ssd1315.md).


## Analog joystick and OLED menu

`exemples/JoystickMenuDemo/ioc` links the shared
`exemples/display/joystick_menu_demo.cpp`. It uses the onboard OLED, the
analog joystick on PA3 (ADC4 input 6) and the green LED on PD8. Up/down
navigate; select toggles the LED or opens Level/About. Level uses arrows to
adjust a bar and select to return. Each press requires a release.

The new `AdcSTM32WBA` implements the generic AdcDevice API for software-triggered
single rounds in polling mode. Its initial support is intentionally limited:
external inputs 1..10, single-ended, unity gain, 6/8/10/12-bit resolution,
VREF+ supplied by the caller. Continuous conversion, interrupt, DMA, FIFO,
oversampling and internal radio temperature measurement are not implemented.
Rebuild the matching Debug or Release MCU library first.

The example builds and links in Debug and Release. ADC register simulation
passes at -O0 and -Os, including failure recovery. The shared menu passes
scripted input and SPI OLED RAM tests under ASan/UBSan. On October 19, 2026,
the maintainer confirmed that the joystick example works on the board. See the [project instructions](exemples/JoystickMenuDemo/README.md).

## BLE scanner on the OLED

`exemples/BleScanDisplayDemo/ioc` enables the optional display in the existing
shared `exemples/bluetooth/ble_central_scan.cpp`. It lists four recently seen
advertiser addresses/names with RSSI values and bars. A name received in a
scan response updates the same entry; entries expire after ten seconds.
The timer queues a display refresh every 250 ms through AppEvt. No display
transfer is done in the timer interrupt or scan-report callback.

The board uses the working OLED SPI3 mapping and TIM16 (Timer DevNo 2),
leaving LPTIM1/2 to Bluetooth. UART reports remain on the ST-LINK USART1 VCP
at 1 Mbaud. Rebuild the matching Debug/Release MCU library first. Both the
OLED and original UART-only scan projects compile and link. Display-model
tests cover device identity, name preservation, RSSI bars, expiry, clock wrap
and event queue saturation under ASan/UBSan. On October 19, 2026, the
maintainer confirmed that the combined BLE scanner and OLED example works
on the STM32WBA65I-DK1.


## BLE connection and security status on the OLED

`exemples/BleStatusDisplayDemo/ioc` enables the optional OLED view in the
shared `exemples/bluetooth/ble_security_demo.cpp`. The device advertises as
BLEStatus and keeps the existing LE Secure Connections MITM policy and
UART numeric-comparison confirmation. It shows waiting/connected state,
two link handles and peer addresses, and each link's current security level
through BtGapConnSecGet. A failed query shows unknown. During comparison,
the six-digit number appears on both OLED and UART; y/n on the UART accepts
or rejects it. Reconnecting from a stored bond uses the existing stack path.

The project uses the working OLED pins, TIM16 (Timer DevNo 2), a 16-entry
AppEvt queue and AppRun. A 250 ms timer queues display work; OLED transfers
and security queries run in application context. UART remains at 1 Mbaud.
Both this project and the original BleSecurityDemo compile/link in Debug and
Release. SPI OLED tests exercise peer state, live security levels, numeric
comparison and UART decisions, disconnect cleanup and queue saturation
under ASan/UBSan. On October 19, 2026, the maintainer confirmed that BleStatusDisplayDemo
works on the STM32WBA65I-DK1.


## Joystick stopwatch

`exemples/StopwatchDemo/ioc` enables JOYSTICK_STOPWATCH in the existing shared
joystick menu example. It starts on the stopwatch page: select starts or
pauses, right records a lap, left resets while paused, and up/down opens
the menu. Elapsed time continues while the menu is open. The original
JoystickMenuDemo remains a three-item menu when the option is absent.

The generic Timer API on TIM16 (Timer DevNo 2), requested 1 MHz, supplies
elapsed time. Display and ADC delays do not accumulate as stopwatch error.
The screen refreshes about ten times per second and stops at 99:59:59.99.
Both projects compile/link in Debug and Release. SPI OLED model tests pass
at -O0/-Os under ASan/UBSan, covering pause/resume, lap, guarded reset,
menu navigation, elapsed digits, delayed polling, ADC failure, timestamp
wrap and the display limit. On October 19, 2026, the maintainer confirmed that StopwatchDemo works
on the STM32WBA65I-DK1.
See the [project instructions](exemples/StopwatchDemo/README.md).


## Joystick countdown timer

`exemples/CountdownTimerDemo/ioc` uses the shared
`exemples/display/countdown_timer_demo.cpp` with the working joystick ADC,
SSD1315 OLED and TIM16 timer. Select starts/pauses; while paused, right adds
10 seconds, up/down adds/subtracts one minute, and left resets to the preset.
Changing the preset reloads the remaining time. The range is 00:10 to 99:59.
At expiry the display shows DONE and the green LED turns on; select restarts
the preset. Hardware timestamps preserve elapsed time through display/ADC
work and timestamp wrap. ADC read failures do not stop expiry handling.

Debug/Release compile and link checks pass. OLED model tests cover digits,
preset bounds, debounce, pause/resume, reset, expiry/LED, input failures and
clock wrap at -O0/-Os under ASan/UBSan. On October 19, 2026, the maintainer confirmed that CountdownTimerDemo
works on the STM32WBA65I-DK1. See the [project instructions](exemples/CountdownTimerDemo/README.md).


## Native SAI PDM microphone capture

`pdm_stm32wba.cpp` implements the existing `coredev/pdm.h` C API using
SAI1 block A and one channel from the shared GPDMA manager. No generic PDM
configuration or function signature changes. The portable filter is in
`audio/pdm_filter.h` and `src/audio/pdm_filter.cpp`. Rebuild the matching
Debug or Release MCU library to include these new sources.

This captures a microphone wired directly to a SAI PDM input. The onboard
STM32WBA65I-DK1 microphone feeds the WM8904 codec, whose output reaches SAI
as audio samples. This native PDM driver does not initialize that codec or
implement its I2S capture path.

Supported configuration:

- One microphone pair on SAI1_D1, mono left/right or interleaved stereo.
- PDM pins use AF3. Clock routes are CK1 on PA1, PA9, PE2 or PG7, or CK2
  on PA6 or PE5. Data routes are D1 on PA2, PA10, PC3, PD5 or PE6.
  Select pins available on the MCU package and accessible on the board.
  The pin array is clock/output first, data/input second.
- HSI16 supplies the SAI clock. The driver selects the nearest PDM clock
  obtainable with dividers 1 through 63 and stores it in
  `PdmDev_t::CfgData.Freq`. Zero requests the 1 MHz default. For example,
  requesting 1.032 MHz selects 1 MHz; PCM rate is then 15,625 samples/s
  per channel, with the HSI oscillator's accuracy.
- Signed 16-bit PCM, native byte order, with 64:1 third-order CIC
  decimation. The CIC has no passband compensation or DC-removal stage;
  this is not a calibrated sound-pressure measurement.
  Only gain fields of zero are supported; other gain values fail Init.
- With `PDM_SMPLMODE_FALLING`, the first channel is the falling-edge
  microphone; RISING swaps the two edge streams. OpMode then selects the
  first/second channel or both.
- `bIntEn` selects interrupt notification or polling service. Both use DMA
  for continuous raw capture. There is no dependency on AppRun or AppEvt.

Use aligned caller-owned memory sized with
`CFIFO_TOTAL_MEMSIZE(blocks, pcm_bytes_per_block)`. A PCM block must contain
whole 16-bit samples, and whole left/right pairs in stereo. `NbSamples`
counts 16-bit values in a returned block, including both channels in stereo.
The driver owns four additional 512-byte raw DMA blocks and fixed filter
state; it allocates no heap memory.

Init claims SAI1 and a free GPDMA channel. It fails without disturbing a
different owner, an already clocked SAI1, invalid pins/buffers/modes, DMA
exhaustion or an HSI-ready timeout. Init enables the logical device;
Start begins capture. Enable after Disable allows another Start.
Stop discards queued/partial input and PCM. SetMode validates the new mode,
stops capture and requires a new Start; an invalid request leaves the
existing mode/capture intact. PowerOff releases the DMA channel and SAI1
clock and restores the prior SAI clock selection. HSI is kept available
for other MCU users. Reconfiguration uses PowerOff followed by Init.

The DMA interrupt publishes a finished raw block and starts another.
`DEVINTRF_EVT_RX_DATA` means raw input is available, so the callback should
wake or flag the application. Call `PdmGetSamples` repeatedly in application
context until it returns null. Each call converts at most one raw block
and returns as soon as a complete PCM block is available; even a one-sample
PCM buffer can be drained without losing the rest of a raw block. An
incomplete PCM block stays private. The returned pointer remains FIFO-owned
and must be consumed/copied before the next GetSamples call.

The first two SAI PDM frames are discarded on every Start, as required by
RM0515. On a DMA/SAI error, STATECHG signals stopped capture; exhausting the
raw queue reports RX_FIFO_FULL and stops rather than overwriting input
being filtered. GetSamples returns null after either failure. The application
can Start again to discard old data and reset the filters. In polling mode,
GetSamples services completion/error flags and must be called often enough
to keep the capture FIFO from overrunning. Keep the SAI/GPDMA clocks running
during capture; stop capture before a deep sleep that gates them. Lifecycle
calls and GetSamples have one application-context owner.

Validation:

- Driver and filter compile for Debug/Release; an application calling the
  PDM API links with the real shared GPDMA implementation.
- Cortex-M33 software/register simulation passes at -O0/-Os. It uses real
  CMSIS, PDM, CIC and CFifo code with the DMA transport modeled at the
  channel-manager API. Checks cover configuration, ownership, clock
  selection, mono/stereo/edge mapping, initial-frame discard, deferred
  PCM publication, tiny buffers, polling/interrupt service, raw queue
  exhaustion, DMA/SAI errors, HSI timeout, stuck-stop reset and restart.
- An independent finite FIR reference matches 2,560 PCM samples at each
  optimization under ASan/UBSan, including saturation, modulo wrap and reset.
- Physical PDM clock, microphone capture and sustained capture timing remain
  pending hardware validation.

Run the tests with:

```sh
python3 -m venv /tmp/iosonata-pdm-tests
/tmp/iosonata-pdm-tests/bin/pip install -r tests/stm32wba/requirements-pdm.txt
/tmp/iosonata-pdm-tests/bin/python tests/stm32wba/run_pdm.py
python3 tests/audio/run_pdm_filter.py
```

The emulation uses Unicorn 2.0.1.post1. Unicorn 2.1.4 misexecutes a Thumb IT
block in this optimized firmware; the same binary passes with the pinned
version. The runner rejects that version explicitly.

References: [RM0515](https://www.st.com/resource/en/reference_manual/rm0515-stm32wba6xxx-series-advanced-armbased-32bit-mcus-stmicroelectronics.pdf),
SAI PDM section and tables 422/423;
[DS14736](https://www.st.com/resource/en/datasheet/stm32wba65ri.pdf), AF3 table;
[board audio connections](https://github.com/STMicroelectronics/stm32wba65ri-dk1-bsp/blob/main/stm32wba65i_discovery_audio.c).


## I2S audio and onboard microphone

Import `exemples/I2sMicLevelDemo/ioc` and rebuild the matching Debug or Release
IOsonata library. The project links the portable example
`exemples/audio/i2s_mic_level_demo.cpp`. It records the onboard microphone
through WM8904 and SAI1, and shows peak amplitude, sample count and clipping
on the SSD1315. The bar spans -72 to 0 dBFS; the numeric peak is in dBFS.
This is a digital level meter, not a calibrated sound-pressure
measurement. The sample count should advance and the bar should respond to
speech or a finger tap near the microphone.

The board connections are:

| Signal | STM32WBA65I-DK1 connection |
| --- | --- |
| Codec control | I2C1 SDA PB1 / SCL PB2, AF4 |
| Codec address | 0x1A, seven-bit address |
| Microphone input | WM8904 DMICDAT2 |
| SAI1 SCK_A | PA7, AF13 |
| SAI1 FS_A | PC4, AF13 |
| SAI1 SD_B receive | PC5, AF13 |
| SAI1 MCLK_A | PE2, AF13 |

The example does not drive SD_A. SAI block A supplies the clocks and block B
receives the codec output. Board headers contain only pin/configuration
mapping. `Wm8904` is a generic `Device` accepting an injected `DeviceIntrf`
for control; PCM is read through the existing `I2S`/`DeviceIntrf` interface.
The codec driver supports DMICDAT1/2 capture, 16/24-bit I2S or justified
formats, gain from -71.625 to +17.625 dB in 0.375 dB steps, enable/disable,
reset and power off. Stereo headphone playback is also supported as described
below. Analogue capture and line-output playback are not implemented.

### WBA I2S operation

`ARM/ST/STM32WBAxx/src/i2s_stm32wba.cpp` implements DevNo 0 with SAI1 A
transmit/clock and synchronized B receive. It supports master/slave operation,
normal I2S, left/right justified data, 8/16/24-bit samples, and stereo or
left/right slot selection. All AF13 routes for these signals in DS14736
are accepted; choose routes present on the MCU package. A negative PortNo
omits SDI, SDO or MCLK. At least one data direction and both clock pins are
required. Slave mode expects externally supplied SCK_A/FS_A and reports the
configured frame rate; it does not measure that external clock.

Master mode uses a fixed MCLK/frame ratio of 256 and dividers 1 through 63.
When SYSCLK already runs from HSE32 or an HSE32-derived PLL, SAI reuses that
crystal-derived SYSCLK. The driver decodes the source before AHB division,
including HSEPRE and the PLL fractional divider. Otherwise it uses HSI16.
It does not reconfigure the CPU PLL. The selected source must remain stable
while audio runs. PowerOff restores the previous SAI clock selection.
Rate selects and returns the closest supported frame rate; zero selects the
16 kHz request. With HSI16 this returns **15,625 Hz** and 4 MHz MCLK. Use the
returned rate and `I2SDev_t::MClkFreq` to configure the codec.

Application samples are little-endian packed bytes: one, two or three bytes
per sample, and left then right in stereo. Transfer lengths must contain
whole selected-channel frames. FIFO memory must be four-byte aligned and
sized with `CFIFO_TOTAL_MEMSIZE(frames, frame_bytes)`. RX and TX buffers must
not overlap. Init divides the supplied memory into complete frames.

- Without a FIFO, receive/transmit use bounded synchronous polling. Receive
  without a FIFO is supported only with both interrupt and DMA disabled.
- FIFO receive is continuous while enabled. Rx copies completed frames and
  returns immediately, including zero when no frames are ready. It never
  publishes an incomplete frame or overwrites a frame being read.
- FIFO transmit returns the bytes queued. The peripheral transmits silence
  when the queue is empty. A positive return means accepted bytes, not that
  the last sample has left the pin. Transmit without a FIFO uses polling.
- `bDmaEn` uses dynamically allocated shared GPDMA channels for FIFO paths.
  DMA uses two fixed internal buffers per direction, in 32-frame transfers;
  receive memory must hold at least 32 frames. The interrupt rearms DMA and
  packs/unpacks at most 32 frames. No heap storage is used.
- `bIntEn` enables DMA/error interrupts, or hardware FIFO interrupts when
  DMA is disabled. With interrupts off, calls to Rx/Tx service the FIFOs and
  DMA flags. Poll often enough to avoid hardware overrun; polling does not
  create an autonomous background task.
- RX_DATA reports completed FIFO data. DMA TX_READY reports queue space
  after a transmit block, and TX_FIFO_EMPTY reports the queued block ending.
  Callbacks run in the service context, normally the interrupt. Defer slow
  processing and display work to the application. Do not change lifecycle,
  rate or ownership from a callback.

Init enables I2S. Disable stops both blocks/DMA and flushes both queues;
Enable restarts them. Reference-counted enables follow DeviceIntrf. Reset
clears an error, flushes queues and restarts if logically enabled. DMA,
clock/frame errors or RX overflow stop the stream and report STATECHG or
RX_FIFO_FULL. Rx/Tx then return zero until Reset or a fresh Init. PowerOff
releases SAI1 and DMA channels and restores the previous SAI clock selection.
Use one application-context owner for transfers and lifecycle. Reconfigure
with PowerOff followed by Init. Keep SAI/GPDMA clocks running during capture.

The shared `sai_stm32wba.cpp` owns SAI1_IRQHandler and arbitrates I2S versus
native PDM. These two modes cannot run simultaneously. The DMA manager
continues to own the GPDMA interrupt vectors and allocate channels alongside
UART, I2C and SPI.

The example disables capture during the codec's 100 ms bias settling delay,
then enables DMA capture. The codec driver stages and reads back its settings
with CLK_SYS_ENA cleared; it enables SYSCLK only after configuring the capture
path. This permits initialization before the audio interface starts MCLK.
WM8904 requires MCLK for ordinary control register access once CLK_SYS_ENA is
set; only the clock control register R22 remains accessible without MCLK.
Disable/PowerOff clear CLK_SYS_ENA before other writes, including after an
MCU-only reset. MCLK must be running for live gain changes after Enable.
The RX FIFO holds 1,024 mono samples. Main-loop
processing drains it and refreshes the OLED every 100 ms. A capture error or
500 ms without samples stops capture and displays an error.

Validation:

- Debug and Release builds compile the changed library sources and link
  I2sMicLevelDemo with the real GPDMA implementation and existing library
  objects. This is an incremental library build, not a clean rebuild of
  every library source.
- Cortex-M33 I2S simulation exercises production I2S, SAI ownership,
  DeviceIntrf and CFifo with modeled registers/DMA. It checks widths,
  packing, channels, RX/TX, polling/interrupt service, rate bounds,
  FIFO overflow, reset/restart, errors, allocation cleanup and receive-only
  capture. Both -O0 and -Os pass. Native PDM regression also passes.
- Generic codec tests run at -O0/-Os under ASan/UBSan, checking register byte
  order, routing, format, gain, lifecycle and each initialization write failing.
  The model enforces the missing-MCLK register restriction, including startup,
  restart and shutdown with MCLK stopped. Lost configuration writes are caught
  by readback before SYSCLK is enabled.
- The maintainer confirmed onboard microphone capture and the OLED level
  meter on October 19, 2026, after the codec clock-ordering fix. The maintainer also confirmed headphone
  tone playback and stereo routing on October 19, 2026. Sustained DMA
  timing under additional system load has not been measured.

```sh
# Same Arm toolchain and emulator environment as the PDM tests above.
/tmp/iosonata-pdm-tests/bin/python tests/stm32wba/run_i2s.py
python3 tests/audio/run_wm8904.py
```

References: [WM8904 Rev 4.1](https://statics.cirrus.com/pubs/proDatasheet/WM8904_Rev4.1.pdf),
[DS14736 AF13 routes](https://www.st.com/resource/en/datasheet/stm32wba65ri.pdf),
[ST board audio configuration](https://github.com/STMicroelectronics/stm32wba65ri-dk1-bsp/blob/main/stm32wba65i_discovery_audio.c).

## Stereo headphone tone demo

Import `exemples/I2sToneDemo/ioc`, rebuild the matching IOsonata library, and
build the example. Connect stereo headphones to the board audio jack.
`exemples/audio/i2s_tone_demo.cpp` repeats four one-second stages:

| OLED step | Left output | Right output |
| --- | --- | --- |
| LEFT 440 Hz | 440 Hz | Silence |
| RIGHT 880 Hz | Silence | 880 Hz |
| BOTH 660 Hz | 660 Hz | 660 Hz |
| SILENCE | Silence | Silence |

The example uses I2S DevNo 0, stereo 16-bit TX DMA and a 256-frame caller-owned
FIFO. PD5 AF13 carries SD_A to the WM8904 DAC input. SD_B is unused; the
microphone/ADC path is off. Control, MCLK/SCK/FS and OLED pins match the
microphone example. The OLED shows the step being queued; the FIFO adds a
small delay before those samples reach the jack.

The tones use a fixed sine table, a phase accumulator based on the actual
returned sample rate, and a 10 ms fade at each tone boundary. Sample peaks
are 4096 (about -18 dBFS), and headphone gain starts at -24 dB. The sequence
runs continuously, with one second of silence each cycle. A short Tx return
retains the remaining samples for the next call. Display drawing and sample
generation run in main; the DMA interrupt handles the output buffers.

`Wm8904Cfg_t` adds `bHeadphoneEn` and `HeadphoneGain`. Set `MicInput` to zero
for playback only, or select DMICDAT1/2 to enable capture as well. Keep MCLK
running throughout headphone Init/Enable: the charge pump, headphone PGAs and
DC servo need the codec clocks. The tone demo starts I2S TX with silence
before initializing the codec, and keeps those clocks running. Capture-only
initialization continues to support MCLK starting afterward.

Headphone startup holds the DAC muted and output shorts engaged, starts both
DC servo channels, and polls completion for at most 300 ms. It releases the
shorts and unmutes only after both channels complete. A control error or
servo timeout shuts down the outputs and fails Init. Disable/PowerOff mute,
re-engage shorts and turn off the drivers, servo, charge pump and PGAs.
HeadphoneGain accepts milli-dB requests, rounds to 1 dB steps and clamps to
-57 through +6 dB. Live gain changes require MCLK. The existing ADC Gain
continues to use 0.375 dB steps.

The WM8904 uses MCLK directly as SYSCLK and requires at least 3 MHz. DAC
oversampling is selected from that clock: normal OSR below 6 MHz, double OSR
at 6 MHz and above. The earlier headphone configuration unconditionally set
DAC_OSR128, violating the codec's 6 MHz minimum at this board's 4 MHz MCLK.
This is a confirmed configuration error; its correction needs a board retest
for the reported hiss during zero-sample playback. Init verifies the muted
DAC setting before enabling SYSCLK, and mute/unmute retain the selected OSR.
Tests cover the 3 MHz and 6 MHz boundaries, shutdown/restart, and a dropped
DAC control write.

Validation:

- Debug/Release incremental library builds and I2sToneDemo links pass; the
  existing I2sMicLevelDemo also builds and links with the updated codec.
- Cortex-M33 I2S tests cover transmit-only DMA and verify that its separately
  allocated channel sends packed samples correctly, alongside the existing
  receive/duplex, overflow and recovery cases.
- Codec ASan/UBSan tests cover playback-only and combined microphone/playback,
  gain limits, mute/short/servo ordering, servo timeout, missing MCLK, shutdown
  and each initialization write failing. Capture-only regression remains.
- The actual example's sample generator is tested at -O0/-Os under ASan/UBSan:
  stereo routing, frequency, signal bounds, fades, silence and chunk boundaries.
- The maintainer confirmed onboard headphone tone playback and stereo
  routing on October 19, 2026.

```sh
python3 tests/audio/run_wm8904.py
python3 tests/audio/run_i2s_tone.py
/tmp/iosonata-pdm-tests/bin/python tests/stm32wba/run_i2s.py
```

## Live microphone to headphones loopback

The WBA65 loopback board configuration now selects its 32 MHz crystal through
the existing g_McuOsc startup configuration. Startup produces a 100 MHz core
clock; SAI uses this HSE-derived SYSCLK divided by 25. The example requests
15,625 Hz explicitly, retaining the earlier 4 MHz MCLK and 15,625 Hz frame
rate while changing their reference from HSI16 to the crystal. The OLED
shows `XTAL 15625 Hz`. A crystal startup failure is reported instead of
silently running this comparison from HSI. No MCU register writes are added
to the application or board header.

The current charge-pump comparison defaults `AUDIO_LOOPBACK_MIC_ENABLE` to 1,
and `AUDIO_LOOPBACK_MIC_FORWARD` and `AUDIO_LOOPBACK_MIC_INVERT` to 0.
For microphone capture with headphones, WM8904 now selects and reads back
CP_DYN_PWR = 0: charge-pump operation follows the volume registers rather
than the instantaneous DAC signal level. Playback-only configurations retain
Class W (CP_DYN_PWR = 1). This is a supported alternate mode, tested here as
a possible influence on clock interference; it is not a confirmed fix.
The OLED shows `CP register -12 dB` and stays on.

The example repeats the clock isolation comparison. Codec capture stays enabled,
but MicrophoneClock(false) changes GPIO1 from DMIC clock output to logic zero.
After the startup tone, zero PCM and DAC mute phases, it sends zero PCM
continuously and labels the screen `MIC CLK OFF ZERO`. Five seconds later,
it restores only the microphone clock and labels the screen `MIC CLK ON ZERO`.
The restored clock stays on. Headphone PCM remains zero throughout both
phases. With no microphone clock, input peak is not a valid acoustic level.
Frame counts still advance.
The OLED stays on; headphone gain and audio clocks retain the loopback settings.
Set `AUDIO_LOOPBACK_MIC_ENABLE=0` to repeat the microphone-off comparison;
this stops capture and its clock without disconnecting its supply. Define
`AUDIO_LOOPBACK_MIC_FORWARD=1` with the microphone enabled to restore live
loopback and the OLED power test described below. Inverted playback remains
available by also setting `AUDIO_LOOPBACK_MIC_INVERT=1`; it displays
`MIC 180 DEG -> HP` and keeps the OLED on. That reverses both PCM channels,
saturating -32768 to 32767 and preserving transformed blocks across partial
TX retries. The startup tone retains its original polarity.

Import `exemples/I2sLoopbackDemo/ioc` and rebuild the matching library and
example. Connect headphones to the onboard audio jack and speak near the
onboard microphone. The WM8904 digitizes the microphone, I2S receives the
stereo frames, and main forwards them to I2S transmit for headphone playback.
The codec's mono microphone signal appears in both channels. The OLED shows
input peak in dBFS and the number of captured frames.

The onboard microphone is U21, MP23DB01HPTR, connected to the WM8904
DMICDAT2 input with its left channel selected in the MB2143 A02 schematic.

Startup sends a known 1 kHz tone for one second, two seconds of zero PCM,
and two seconds with the codec DAC soft-muted, then forwards the microphone.
The OLED labels these phases `Duplex tone 1 kHz`, `Zero PCM`, `DAC muted`,
and `MIC -> HEADPHONES`. RX, TX, microphone capture, codec clocks and headphone
stages remain enabled throughout. Wm8904::Mute changes and verifies only the
DAC control register, preserving its OSR setting; a failed control operation
shuts down the codec. MCLK must be running for this live operation.

After these startup phases, live microphone playback continues while the
OLED panel and its charge pump are disabled for five seconds. The screen
then returns automatically and stays on. No display transfers occur during
the black interval. Compare the buzz with the screen off and on; audio
clocks, microphone gain and headphone gain remain unchanged.

Compare noise during Zero PCM and DAC muted. Noise removed by codec mute
points toward the DAC or its digital input; noise remaining during mute
points farther along the output/power/headphone path. This comparison narrows
the fault but does not identify its exact source. The OLED also shows L/R
sample mismatches per display interval; the codec routes the same microphone
to both slots.
The maintainer heard the startup tone, reported hiss during zero-sample
playback, and confirmed zero L/R mismatches. The microphone peak varies but
does not track sound volume; intelligible loopback is not yet validated.
The maintainer reports that DAC mute does not affect the noise and that it
appears when the display starts. Turning off the OLED and its charge pump
did not remove the buzz on the board. The buzz disappeared with microphone
capture and its clock disabled. It returned with capture and microphone clock
enabled while headphone PCM stayed zero. The next comparison holds the
external microphone clock low while leaving codec capture enabled. This
separates external microphone activity from the codec capture circuitry.
With the microphone clock held low, the maintainer reports that the noise
stopped when the startup tone began and stayed off. The next board comparison
restores the clock after five seconds of quiet zero PCM. The maintainer
confirmed that the buzz returns with the microphone clock on. The next
comparison kept the same transition and zero PCM, using the crystal-derived
audio clock. The maintainer confirmed that the buzz still returns. The
crystal reference did not remove it. Inverting the captured microphone PCM
also made no audible difference. The charge-pump mode comparison now awaits
a board retest, using zero PCM and the same microphone clock OFF/ON sequence.

This exercises simultaneous RX and TX DMA through the same generic I2S
interface. Board mapping combines PC5 AF13 receive with PD5 AF13 transmit;
control and clock pins match the two working audio examples. The portable
example is `exemples/audio/i2s_loopback_demo.cpp`; it accesses audio through
DeviceIntrf Rx/Tx and contains no MCU register operations.

The receive FIFO holds 8,192 stereo frames (32 KiB of sample memory) so it
can keep receiving while codec Init waits through its 100 ms bias settling
and up to 300 ms servo calibration. The transmit FIFO holds 256 frames.
MCLK and both streams stay running throughout startup. The application
discards queued startup samples, then queues 128 silent frames (about 8 ms
at the default 15,625 Hz rate) ahead of live audio to cushion display updates.
This priming delay is not a measured end-to-end latency result.

Main retains a partially queued block until Tx accepts all of it, then reads
another block. It forwards whole stereo frames without altering sample values.
Display text is snapshotted every 100 ms and one row is sent only after audio
has been serviced and no application TX data remain pending. SSD1315 text
printing now transmits only changed spans within each page, instead of a full
screen for every printf. Unchanged text generates no SPI traffic. ADC gain requests
+18 dB and the codec selects its +17.625 dB maximum; headphone gain is -12 dB.
A DMA/SAI error, receive overflow or 500 ms without newly received samples
shuts down the codec and audio interface and displays an error.

Validation:

- Debug/Release incremental library builds and I2sLoopbackDemo links pass.
- Host ASan/UBSan tests include the actual example and verify exact sample
  forwarding across short Rx/Tx transfers, repeated transmit stalls, no buffer
  reuse while data remains pending, startup discard/priming, bounded startup
  draining and error detection at -O0/-Os.
- The existing Cortex-M33 I2S tests cover simultaneous receive/transmit DMA;
  codec tests cover microphone and headphone paths enabled together.
- The first board loopback run stopped with a SAI/DMA error after 64 frames.
  The driver previously built the next TX block before rearming its DMA and
  processed RX before servicing another pending TX completion. Both completed
  channels are now rearmed before FIFO processing, and TX uses a previously
  prepared alternate buffer. Native 8/16-bit samples use bulk FIFO copies.
  The additional prepared TX block is 32 frames (about 2 ms at 15,625 Hz).
- The DMA-ordering regression fails on the earlier driver and passes after
  the change. Tests also check FIFO wrap at a 40-frame capacity, sample order,
  packed widths, single-direction operation, error shutdown and reset.
- A later board run stopped with RX FIFO overflow after a few seconds. Full
  screen refreshes during text printing could exhaust the TX lead while RX
  kept running, leaving captured data queued behind silence sent by TX.
  Text updates now use changed page spans and the demo services audio between
  individual display rows.
- A one-minute host simulation runs the actual example and OLED driver with
  continuous 15,625 Hz capture/playback and injected 20 us delays per SPI byte.
  It checks for no TX underrun, bounded RX backlog, and audio priority over
  display work. This is a scheduling stress test, not measured board timing.
  The earlier OLED implementation fails the same underrun/backlog checks.
- OLED tests check bounded text traffic, unchanged text, page boundaries,
  rotation and clearing, alongside the existing console and graphics demos.
- Physical live microphone/headphone loopback still awaits a successful retest.

```sh
python3 tests/audio/run_i2s_loopback.py
```
