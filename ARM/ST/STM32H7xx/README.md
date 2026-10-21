# STM32H753 MCU port

IOsonata MCU port for the STM32H753 Cortex-M7, selected by `STM32H753xx`.
It supplies startup, GPIO/EXTI, polling, interrupt and DMA UART, the 14
general-purpose/basic/advanced TIM instances, USB2 full-speed device support
through the shared STM32 OTG driver, and an Ethernet MAC/DMA driver with
LAN8742A RMII PHY support. Peripheral support and limits are described below.

The MCU library is shared by boards using STM32H753. NUCLEO-H753ZI is the
board used for the supplied example configurations and hardware validation;
it does not define the scope of the MCU port.

## Board configuration and validation

Board pin mappings are in `ARM/ST/include/stm32_boards.h`; each example's
`board.h` selects its pins and peripheral instances. The three user LEDs,
ST-LINK USART3 bridge, CN13 USB connection and LAN8742A PHY on CN14 are
NUCLEO-H753ZI board details, not requirements of the MCU library.

For another STM32H753 board, select pins available on its MCU package and
match the peripheral routing, power supply, external devices and linker memory
layout to that board. The current startup uses the internal LDO and HSI64/PLL1;
HSE selection is not implemented. See [Clocks and memory](#clocks-and-memory).
Ethernet currently supports the LAN8742A RMII PHY; a different PHY requires
appropriate driver support.

Hardware results in this document identify tests run on NUCLEO-H753ZI.
Other boards need their own hardware validation. The connection and first-test
instructions below use the supplied NUCLEO-H753ZI example mappings.

## Build and first tests on NUCLEO-H753ZI

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

The UART PRBS TX and loopback examples now select interrupt mode with DMA on the ST-LINK virtual
COM port: PD9 RX, PD8 TX, AF7, USART3 DevNo 3. Their `board.h` can select
`BOARD_UART_DEVNO` / `BOARD_UART_PINS` instead for USART1 on Arduino D0/D1,
PB7 RX and PB6 TX, AF7. Connect an external 3.3 V serial adapter there.
Board pin mappings are in `ARM/ST/include/stm32_boards.h`.

## UART DMA

The shared `uart_stm32.cpp` supports TX and circular RX DMA on USART1, USART2,
USART3, UART4, UART5, USART6, UART7 and UART8. LPUART1 UART integration with BDMA remains unimplemented; its polling and
interrupt modes remain available. A DMA request fails initialization.

Each instance reserves two streams through the shared DMA manager. USART1,
USART2, USART3 and UART4 use DMA1 stream pairs 0/1, 2/3, 4/5 and 6/7; UART5,
USART6, UART7 and UART8 use the corresponding DMA2 pairs. RX is the first
stream of each pair. A conflicting owner makes initialization fail.

The `.uart_dma` section uses AXI SRAM with separate 32-byte cache lines for
RX rings and TX staging buffers. Caller FIFOs may remain in DTCM: TX copies
up to 32 bytes per DMA block; blocking FIFO bytes are released at completion.
Queued TX blocks now chain at DMA completion while the UART is still sending,
instead of waiting for USART TC between blocks. The final TX-ready event still
waits for USART TC, after the last character leaves the line.
RX uses a 64-byte circular ring, serviced at half/full completion and USART
IDLE, with polling service when interrupts are disabled. Poll frequently
in that mode so the ring cannot overrun. TX cleans and RX invalidates cache
lines when D-cache is enabled. Keep this section in AXI SRAM in custom linker
scripts. RX DMA errors increment RxOvrErrCnt and restart reception; TX errors
count the remaining bytes as dropped. A stream that cannot stop makes
initialization fail; reset the device before retrying a persistent failure.

Rebuild the library and `UartPrbsTxTest`, then `UartLoopback`, using Release.
Both now set `UART_DMA_MODE true` and retain the same 1 Mbaud host commands
above. Set that configuration value to false to compare interrupt-only mode.
The PRBS application's byte mode is unchanged.

Validation: GNU Arm GCC 13.2.1 builds and links both UART examples at -O0 and
-O3 with soft and hard float. The linked DMA section is aligned in AXI SRAM
and the USART3 and DMA vectors resolve to their driver handlers.
`python3 tests/stm32/run_uart.py` covers routing, TX/RX, FIFO wrap and overflow,
DMA errors, polling, lifecycle, ownership, DTCM caller buffers and cache
maintenance calls with the real shared driver. The register model does not
simulate a physical cache or DMA timing. Tests also cover chaining before USART
TC with blocking and non-blocking FIFOs, and final TX-ready notification.

Hardware on NUCLEO-H753ZI, October 21, 2026: USART3 PRBS TX with interrupts
and DMA enabled at 1 Mbaud averaged 100380.37 B/s after block chaining,
compared with about 99,578 B/s before the change.

UartLoopback with interrupts and DMA enabled passed a 60-second run through
ST-LINK VCP using `Python/usb_cdc_loopback.py`: 5892096 bytes sent and received,
98176.83 B/s TX and 98103.09 B/s RX, zero errors and zero pending bytes after
draining. First RX was 3 ms; longest TX wait was 25 ms and RX gap 15 ms.
The USB CDC banner was absent because the firmware was UartLoopback. This is
UART DMA TX/RX evidence, not a native USB transfer. Other UART instances and
DMA polling mode still need hardware validation.

## I2C and SPI

The shared `i2c_stm32.cpp` and `spi_stm32.cpp` now include H753 instance
mapping, clocks, reset and interrupt vectors. I2C DevNo 0-3 selects I2C1-4;
SPI DevNo 0-5 selects SPI1-6. Both support polling/interrupt master and interrupt slave operation, with or
without DMA. I2C1-3 and SPI1-5 allocate DMA1/2 streams through DMAMUX1;
I2C4 and SPI6 allocate BDMA channels through DMAMUX2. Allocation failure is an
initialization failure, never a fallback to CPU transfers.

The common DMA manager stages bus transfers through fixed 512-byte buffers
in SRAM4 (`.bus_dma`, 12 KiB total). DTCM and unaligned caller buffers work;
cache operations touch only the staging buffers' exclusive cache lines.
Completed RX chunks and partial stopped transfers are copied back before the
completion callback. Custom linker scripts must place `.bus_dma` in SRAM4.
Long transfers reload the DMA channel; SPI slave rates must allow its FIFO to
cover interrupt/refill latency. Overrun and DMA errors fail the frame.

H753 interrupt SPI writes use IRQ-driven staging for long command/data
buffers, retaining the temporary source until the last chunk is copied,
then reporting completion once. No length-based polling fallback is used.

I2C1-3 use PCLK1; I2C4 uses PCLK4. Rates use TIMINGR with analog filtering,
including Fast-mode Plus drive configuration above 400 kHz. The shared engine
handles repeated START, NACK/error recovery and reload for transfers over
255 bytes. Use external pull-ups and the alternate-function pin map for the
selected instance. Bus reset restores the alternate-function pin settings.

SPI1-3 select the shared PER clock, HSI64 with the default startup. This avoids
using PLL1Q, whose output is disabled by the default clock configuration.
SPI4/5 use PCLK2 and SPI6 uses PCLK4. Rate selection returns the closest
supported divider. The shared driver supports modes 0-3 and 4-16 bit items;
QSPI is DevNo 6, using the shared polling-only indirect command/data driver
with HCLK as its kernel clock. QSPI interrupt, DMA and memory-mapped modes
are not exposed. PER clock selection is
preserved; HSI and CSI sources are supported, external HSE is not configured
by this port.

`python3 tests/stm32h7/run_bus.py` builds the H753 drivers and generic
interfaces at -O0/-O3. The register model checks every I2C/SPI instance in
polling/interrupt and DMA master/slave paths, 1200-byte DTCM transfers,
unaligned buffers and guards, temporary command/data source lifetime, partial
31-byte and 600-byte slave frames, routing, reload/NACK and clock selection.
It also retains QSPI initialization checks. WBA I2C (70 checks) and SPI
(64 checks) regressions pass. H753, L476 and WBA sources compile with warnings
as errors; H753 also compiles with hard float. Models do not establish DMA
refill timing, physical cache coherency or hardware performance.

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
on NUCLEO-H753ZI has not yet been hardware tested.

## I2C and SPI master/slave examples

[I2CMasterSlave](STM32H753/exemples/I2CMasterSlave/README.md) connects I2C1
master to I2C2 slave with two jumpers and external pull-ups.
[SPIMasterSlave](STM32H753/exemples/SPIMasterSlave/README.md) connects SPI1
master to SPI3 slave with four jumpers. Both use the shared IOsonata example
applications and keep board mappings in `board.h`.

Build the library and application in Release and open ST-LINK VCP at
1000000 baud, 8N1. Each tests four master modes against interrupt and DMA
slaves; SPI covers modes 0-3. Expected summary is 8 passed, 0 failed, 0 skipped.
Only explicit hardware HAS_DMA flags may exclude DMA combinations. An init
failure is FAIL, not SKIPPED. Existing payload and completion checks remain.
I2C1 master / I2C2 slave passed all eight combinations on NUCLEO-H753ZI
(2026-10-21), including 254-byte writes and NACK recovery. SPI1 master /
SPI3 slave also passed all eight combinations on that date at 1 MHz:
416 transfers covering modes 0-3 and 1-, 16- and 129-byte frames. LOW SCK
slew resolved the earlier mode-1 corruption without changing the test.
H753 masters use LOW SCK slew up to 1 MHz and retain TURBO above it;
only 1 MHz has been hardware validated. See the example README.

## Native USB CDC examples

Import `STM32H753/exemples/UsbCdcLoopback/ioc` or
`STM32H753/exemples/UsbCdcPrbsTx/ioc` alongside the same MCU library. These
projects link the existing common applications in `exemples/usb`. Build the
library and application in matching Debug or Release configurations. Debug
links semihosting; enable `monitor arm semihosting enable` before resume.
Release uses nosys and runs without the debugger.

On NUCLEO-H753ZI, power the board through ST-LINK first, then connect a data cable from **CN13**
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
CDC enumeration and Release transfers are hardware validated. Run of
2026-10-20, macOS host: the 60-second USB CDC loopback transferred 33930752
bytes each way, averaging 565487.35 B/s TX and 565414.84 B/s RX, with zero
errors and zero pending bytes after draining; first RX 2 ms after start,
longest TX wait 6 ms, longest RX gap 4 ms. PRBS TX sustained approximately
817 kB/s with no loss. An earlier run measured 449927.81 B/s loopback and
770-780 kB/s PRBS TX. These are full-speed USB results; the WBA65 high-speed
result is a different bus mode.

### Additional USB examples

Import the corresponding project under `STM32H753/exemples/<name>/ioc`.
Each links the existing application in `exemples/usb` and uses the same
H753 library, Debug/Release settings and native CN13 connection as CDC.

| Project | Application |
|---|---|
| UsbDualCdcStress | Two CDC ports: simultaneous loopback and PRBS TX |
| UsbCustomBulkLoopback | Vendor bulk OUT/IN loopback |
| UsbHidLoopback | Vendor HID input/output reports |
| UsbIntLoopback | Interrupt loopback with alternate endpoint intervals |
| UsbMscRamDisk | Disposable 64 KiB FAT12 RAM disk; contents reset on reboot |

Use the host scripts and dependencies in the [USB User Guide](../../../docs/usb-user-guide.md).
The scripts are `Python/usb_dual_cdc_stress.py`,
`Python/usb_custom_bulk_loopback.py`, `Python/usb_hid_loopback.py` and
`Python/usb_int_loopback.py`; use `--help` for device selection and options.
The RAM disk appears as a USB storage device; eject it before reset or unplug.
The dual-CDC application's remote-wakeup setting is shared with other ports;
remote wakeup has not been validated on H753.

These five additional projects compile/link in all four H753 library
configurations; hardware validation remains pending. ISO and the
ISO-containing combo stress examples are not included.

## Ethernet examples

`EthernetDemo`, `EthernetUdpDemo` and `EthernetTcpDemo` are configured for the
NUCLEO-H753ZI's onboard
LAN8742A PHY and RJ45 CN14. They share the generic Ethernet frame interface,
lwIP IPv4 service and existing `SockIntrf` C/C++ interface. DHCP, ping, DNS,
connected UDP and connected TCP are implemented. Ethernet TLS/DTLS is not yet
implemented; nRF91 modem support is unchanged.

The console is USART3/ST-LINK VCP at 1 Mbaud, 8N1. Start with DHCP/ping using
`EthernetDemo`, then set the host address in either echo project's `board.h`
and run `Python/net_echo.py`. Assign a unique MAC address per board. The default
SDK layout must include `external/lwip` beside IOsonata; the tested version is
lwIP 2.2.0. See [networking and board bring-up](../../../docs/networking.md)
for build steps, pin mapping, ownership, memory, limits and validation.

All examples compile/link and the packet/register simulations pass. On
October 21, 2026, NUCLEO-H753ZI hardware tests confirmed 100 Mbit/s link,
DHCP, ping (20/20 replies), UDP/TCP echo, TCP server restart and cable
reconnection without a board reset. The UDP startup log had a reply gap;
DNS, sustained load and D-cache-enabled traffic still need hardware tests.
See the networking document for measurements and test limits.

## Crypto engines

`CryptoTest` uses the same generic `exemples/crypto/crypto_engine_test.cpp`
as the WBA port. H753 selects `CryptoRngStm32`, `CryptoStm32H7Aes`,
`CryptoStm32H7Hash` and `CryptoUecc`. CRYP handles AES-128/256 ECB/CBC/CTR;
AES-128 CMAC/CCM/GCM use the software constructions over hardware AES.
HASH handles one-shot SHA-256 and the digests in HMAC-SHA-256. Streaming
SHA-256 and P-256 remain software. Public crypto interfaces are unchanged.

The October 21, 2026 board run passed 92 checks with zero failures or skips.
This includes AES-128/256 encryption/decryption, in-place/bounds checks,
one-shot SHA-256, and HASH-accelerated HMAC-SHA-256 with long keys, truncated
output, key/message/output boundaries and overlapping output. Software P-256
and the hardware RNG checks also passed. See the example README for the exact
scope; fault-injection results remain separate host-model evidence.

The RNG selects HSI48 without changing USB clock selection or PLL settings.
A clock/seed error or data-ready timeout fails the request, clears its output
and disables the RNG; call Enable or Reset explicitly before trying again.
The HSI48 oscillator remains on because USB can share it.

Build `CryptoTest` in Release and open ST-LINK VCP at **115200 baud, 8N1**.
Debug uses semihosting. No Ethernet cable is needed. See the
[CryptoTest instructions](STM32H753/exemples/CryptoTest/README.md).
Run `python3 tests/stm32h7/run_crypto.py` for RNG register fault tests and
existing engine known answers. Simulation does not validate physical entropy;
the board results above validate the tested crypto paths. Run
`python3 tests/stm32h7/run_crypto_accel.py` for CRYP/HASH/HMAC register
model tests with timeout, wiping and shared-lock coverage. Ethernet TLS/DTLS
remains unimplemented.

## Internal flash NVM and DFU

The H753 library includes `nvm_stm32.cpp` and `dfu_stm32.cpp`. Both use
`flash_stm32h7xx.h` for the same register-level erase/program sequence.
The existing `NvmIntrf` / `Nvm` and `DfuTgt*` interfaces are unchanged.

- Geometry: 2 MiB, two 1 MiB banks, 128 KiB erase sectors, 32-byte writes.
- Addresses use the mapped flash range starting at 0x08000000. Active bank
  swap selects the physical control registers for both writes and erases.
- Writes require whole, aligned 32-byte units in erased flash. Source buffers
  may be unaligned. All-ones units stay erased; programmed units are read
  back. Do not rewrite an ECC unit before erasing its sector.
- Erase uses x32 parallelism and requires VDD between 2.7 and 3.6 V.
- Operations finish synchronously, report controller errors, and bound busy
  polling. A timeout leaves the active operation untouched; reset before
  retrying. Same-bank execution can stall for the duration of an operation.
- Changed flash data-cache lines and the instruction cache are invalidated.
  Preserve the default flash memory attributes; do not map flash write-back.
  DFU disables core caches before handing control to the application's vector
  table and accepts initial stacks only in the H753's actual SRAM ranges.

Reserve storage sectors in the application's linker layout. The driver does
not choose an NVM region or protect application/boot sectors from its caller.
DFU boot/record/slot linker layouts and an H753 DfuBoot example are not supplied
by this change. OTP, option-byte programming and recovery from uncorrectable
ECC read faults are not implemented.

Validation: both drivers compile with GNU Arm GCC 13.2.1 for Cortex-M7 at
-O0 and -O3, using soft-float and hard-float (`fpv5-d16`). Linux host register
checks cover both bank mappings, all sectors, programming/readback, rejected
alignment/range/rewrite requests, lock failure, status clearing and timeouts:
`python3 tests/storage/stm32h7_flash/run.py`. The model does not emulate ECC,
key unlocking or physical timing. Flash erase/write and DFU handoff still need
hardware validation; the earlier full-library build results below predate
these flash additions.

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
separate 512 KiB RAM2 region. SRAM4 is a 64 KiB RAM4 region for bus DMA
staging; D2 SRAM is not allocated by this linker script. SystemInit enables the Cortex-M7 instruction cache for flash execution;
data cache remains disabled. Ethernet descriptors and frame buffers use a
separate `.eth_dma` section in AXI SRAM, with explicit cache maintenance when
enabled. The shared `dma_stm32.cpp` manager supports all 16 DMA1/2 streams
and eight BDMA channels, with DMAMUX1/2 peripheral routing. It uses direct mode with byte, half-word or word transfers,
exclusive ownership, per-stream callbacks, completion/error flags and bounded
disable waits. Use `STM32_DMA_STREAM(controller, stream)` with controller 1/2
and stream 0-7, or `STM32_BDMA_CH(0..7)`. MDMA, circular/double-buffer
transfer setup and DMAMUX request
generators/synchronization are not provided by this manager.

H753 UART DMA uses this manager for exclusive stream ownership and vectors.
The earlier successful UART benchmarks used interrupt mode with DMA off.
I2C/SPI use the manager's buffered transfers in SRAM4. Raw transfers (used
by UART) require DMA-accessible memory and owner-managed cache maintenance;
raw Start rejects ITCM/DTCM and BDMA addresses outside SRAM4.

`python3 tests/stm32h7/run_dma.py` tests the compiled Arm manager against modeled
registers: all streams, routing, widths, direction, remaining counts, interrupt
vectors, flags, ownership, rejected buffers and disable timeout. The manager
compiles at -O0 and -O3 for H753 and L476. USART3 DMA TX/RX is hardware
validated as recorded in [UART DMA](#uart-dma); other routes remain untested.

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

CAN, ADC/DAC and PWM are not implemented by this port. LPUART1 DMA configuration returns false
because UART BDMA integration is not implemented. The single library project includes the core device, UART, timer,
application event and logging sources needed by these examples.

`python3 tests/stm32h7/run.py` builds the library and all bare-metal examples in
Debug/Release, with and without hard-float, and runs Cortex-M instruction tests
using the actual CMSIS header. Install Python `unicorn` and `pyelftools` for
the modeled register tests. These check startup clock selection, all four APB
clock setters, UART3/GPIO setup, DMA routing and buffer placement, all 14 timer device indices,
timer clock calculation with APB /1 and TIMPRE, and USB2 clock/power/pin setup
with a failed HSI48 start. USB firmware vectors select the shared OTG handler.

All four configurations passed with GNU Arm GCC 13.2.1. This is compile/link
and modeled register evidence, not a hardware throughput or timing result.

## Reorganization regression checks

The H753 library project includes the micro-ecc implementation and the 30
lwIP source files used by the Ethernet examples. It excludes the alternative
ARM SysTick TimerDev implementation in every configuration: the library must
have exactly one strong `TimerInit`, supplied by the shared STM32 driver.
SysTick primitives and the TaktOS tick remain available.

Run `python3 tests/stm32h7/run.py` for library/example builds, register tests
and post-link checks in all four configurations. The runner reads linked
source paths, preprocessor definitions and source exclusions from the
IOcomposer library project. It requires the SDK's external lwIP and Fusion
checkouts; `LWIP_ROOT` and `FUSION_ROOT` can select other locations.

To inspect already-built firmware, place its library archive and bare-metal
example ELF files in one directory and run:

```sh
python3 tests/stm32h7/check_link.py <build-directory>
```

This checks the unique timer initializer, stack/reset vectors, active USB,
UART/DMA and Ethernet vectors, STM32 timer selection, and AXI SRAM alignment
and placement of DMA buffers. It does not execute firmware on the board.

Additional regressions:

| Command | Coverage |
|---|---|
| `python3 tests/stm32h7/run_bus.py` | All I2C/SPI instances, DMA master/slave, staging, polling/IRQ, reload/NACK and clocks |
| `python3 tests/stm32h7/run_dma.py` | DMA1/2 and BDMA routing, staging/partial stop, ownership, flags and timeout |
| `python3 tests/stm32/run_uart.py` | Shared STM32 UART modes, FIFO, DMA and fault handling |
| `python3 tests/stm32h7/run_ethernet.py` | MAC/PHY registers, descriptors, cache handling and failures |
| `python3 tests/stm32h7/run_crypto.py` | RNG failures and software crypto known answers |
| `python3 tests/stm32h7/run_crypto_accel.py` | CRYP/HASH/HMAC known answers, bounds, timeout recovery, clearing and shared locks |
| `python3 tests/storage/stm32h7_flash/run.py` | Flash bank mappings, erase/program rules, errors and timeout |

Hardware priorities are I2C/SPI master/slave transfers,
Ethernet DNS and sustained traffic, then reserved-sector
flash NVM and DFU handoff. Previously recorded LED, timer, interrupt UART and
USB results remain the hardware evidence for those paths.

## Hardware results on NUCLEO-H753ZI

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
