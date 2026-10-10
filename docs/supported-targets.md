# Supported MCUs

IOsonata provides MCU drivers, portable device drivers and application examples
for the targets below. Nordic is the most developed and most complete MCU
family in IOsonata, with extensive Bluetooth, peripheral, sensor, storage and
RTOS application support. STM32F0, STM32L4 and SAM4LC also have established
hardware use. Each family section describes what its ports actually provide.

Reviewed against `prerelease_0.13` at
[`c505b73`](https://github.com/IOsonata/IOsonata/commit/c505b732e03277445cb01e33d2a38f344934d6bc):
all 30 MCU IOcomposer library projects, their source selections, target drivers
and representative application projects. Source-only and planned targets are
listed separately. Hardware results retain their original scope below.

## Reading the tables

The tables distinguish implemented features, supplied examples and validation.
A peripheral can be implemented without a recorded board test; a project can
also need integration work even when its driver exists. Named examples show
how the code is used, rather than implying that every example was rebuilt in
this documentation review.

- **Supported**: startup, GPIO, UART and IOsonata timers are implemented.
  This is the entry requirement for a port, not a description of its full
  capabilities. Read the peripheral and example columns for those.
- **Project integration incomplete**: an implementation exists but required
  files are missing from the supplied project.
- **Partial or incomplete port**: the row identifies the implemented pieces
  and the work still required.
- **Hardware validated**: maintainer-confirmed use on hardware. The recorded
  results identify the functions exercised.
- **Build/model tested**: compiler, linker or host register tests; hardware
  validation is listed separately.

Portable sensor, display, storage, protocol and software crypto drivers can be
used through the interfaces available on each MCU. For example, the
[environmental sensor application](../exemples/sensor/env_tph_demo.cpp) selects
I2C or SPI for the same sensor object. A generic driver in a library project
does not imply that every MCU peripheral needed by that driver is implemented.

Board pin assignments belong in the application's `board.h`. Timer device
numbers are virtual indices, not hardware timer numbers; shared ARM SysTick
support is separate from the MCU-specific timers listed here.

## Nordic

The nRF52832, nRF52840, nRF54L15 and nRF54LM20 ports are actively developed
and used on hardware. Their libraries and examples cover substantially more
than startup and timing, including complete applications using Bluetooth,
serial interfaces, portable device drivers and schedulers. Capabilities differ
by MCU and selected radio stack.

| MCU / core | Implemented peripherals and services | Application examples | Status and validation |
|---|---|---|---|
| [nRF52832](../ARM/Nordic/nRF52/nRF52832/lib/ioc/) | GPIO, UART/UARTE, I2C, SPI, RTC/TIMER, PWM, SAADC, analog comparator, PDM microphone capture, NFC target frames, internal NVM, RNG and Bluetooth | [Examples](../ARM/Nordic/nRF52/nRF52832/exemples/): BLE advertising/scanning and UART bridges, I2C/SPI master/slave, ADC, PWM, environmental/motion sensors, displays, EEPROM/flash/filesystems, FreeRTOS and TaktOS | Mature, extensive port; established BLE, UART, sensor and low-power hardware use. |
| [nRF52840](../ARM/Nordic/nRF52/nRF52840/lib/ioc/) | GPIO, UART/UARTE, I2C, SPI/QSPI, RTC/TIMER, PWM, SAADC, comparator, PDM, NFC target frames, NVM, RNG, CC3xx crypto, Bluetooth and native full-speed USB device | [Examples](../ARM/Nordic/nRF52/nRF52840/exemples/): BLE, sensors/displays, flash/filesystems, PDM capture, QSPI, UART/SLIP, CDC, HID, bulk, interrupt, ISO, MSC and composite USB; bare metal and RTOS | Mature, extensive port; recorded bare-metal and TaktOS composite USB endurance runs. |
| [nRF54L15](../ARM/Nordic/nRF54/nRF54L15/lib/ioc/) | GPIO, UART/UARTE, I2C, SPI, GRTC/TIMER, PWM, NVM, RNG, CRACEN crypto, NFC target frames and Bluetooth | [Examples](../ARM/Nordic/nRF54/nRF54L15/exemples/): BLE advertising/scanning, periodic advertising/sync, UART BLE peripheral/central, UART/SLIP PRBS, PWM, timers and TaktOS | Mature, actively developed port; established UART/Bluetooth and TaktOS hardware use. I2C/SPI modes and validation are noted below. |
| [nRF54LM20A/B](../ARM/Nordic/nRF54/nRF54LM20x/lib/ioc/) | Shared nRF54L GPIO, UART, I2C, SPI, timers, PWM, NVM, crypto, NFC and Bluetooth support, plus native high-speed USB device | [Examples](../ARM/Nordic/nRF54/nRF54LM20x/exemples/): nRF54L serial/BLE applications, USB/BLE bridges, CDC, HID, bulk, interrupt, ISO, MSC and composite USB, including TaktOS | Extensive port; recorded bare-metal and TaktOS high-speed USB composite runs. Select the matching LM20 variant. |
| [nRF5340 application core](../ARM/Nordic/nRF53/nRF5340_App/lib/ioc/) | GPIO, UART, I2C, SPI/QSPI, RTC/TIMER, PWM and PDM; application-core startup and shared-peripheral interrupt dispatch | [Examples](../ARM/Nordic/nRF53/nRF5340_App/exemples/): I2C/SPI master/slave, UART TX/RX PRBS, SLIP, retargeting and timers | Supported; current full IOC builds and hardware results are not recorded in this review. |
| [nRF5340 network core](../ARM/Nordic/nRF53/nRF5340_Net/lib/ioc/) | GPIO, UART, I2C, SPI, RTC/TIMER and RNG; separate network-core startup and dispatch | [Examples](../ARM/Nordic/nRF53/nRF5340_Net/exemples/): UART PRBS, timers and BLE projects | Supported MCU drivers. BLE example presence does not establish a validated inter-core radio application. |
| [nRF9160](../ARM/Nordic/nRF91/nRF9160/lib/ioc/) | GPIO, UART, I2C, SPI, RTC/TIMER, PWM, SAADC, watchdog, NVM, RNG/CC3xx, modem IPC, LTE, sockets and GNSS | [Examples](../ARM/Nordic/nRF91/nRF9160/exemples/): modem information, LTE UDP with bare metal/TaktOS, GNSS fixes, ADC, NVM, watchdog and crypto | Hardware tested: `LteModemInfo` modem initialization/AT replies and `LteUdpTaktOS` LTE-M registration with a successful UDP echo. GNSS needs modem firmware 1.3.4 or newer (revision 2 silicon); revision 1, limited to firmware 1.2.8, has no GNSS. |
| [nRF91x1](../ARM/Nordic/nRF91/nRF91x1/lib/ioc/) | Shared nRF91 peripheral, LTE/socket and GNSS implementations | [Examples](../ARM/Nordic/nRF91/nRF91x1/exemples/): LTE/GNSS, ADC, NVM, watchdog, crypto, I2C/SPI, EEPROM, environmental sensors, PWM, UART/SLIP and timers | Hardware tested on nRF9161: `LteUdpTaktOS` LTE-M registration and UDP echo confirmed by the maintainer. Debug/Release currently select `NRF9120_XXAA`. Select and verify the intended part and modem firmware. |
| [nRF52805](../ARM/Nordic/nRF52/nRF52805/lib/ioc/), [nRF52810](../ARM/Nordic/nRF52/nRF52810/lib/ioc/) | Shared nRF52 startup, GPIO, UART, I2C, SPI, RTC/TIMER, RNG and Bluetooth sources | Blinky and DFU projects in each target directory | RTC/TIMER drivers included in both library projects. Timer compile/link checks pass for both MCUs; hardware validation is pending. |
| nRF54H20 [application](../ARM/Nordic/nRF54/nRF54H20/nRF54H20_App/lib/ioc/), [network](../ARM/Nordic/nRF54/nRF54H20/nRF54H20_Net/lib/ioc/) and [RISC-V](../RISCV/Nordic/nRF54/nRF54H20/lib/ioc/) | Initial projects and partial target integration | See [H20 development notes](../ARM/Nordic/nRF54/nRF54H20/README.md) | Incomplete ports: startup, peripheral, NRFS USB, MPSL and inter-core HCI work remains. |

### Nordic implementation details

The shared [UART](../ARM/Nordic/src/uart_nrfx.cpp),
[I2C](../ARM/Nordic/src/i2c_nrfx.cpp) and
[SPI](../ARM/Nordic/src/spi_nrfx.cpp) drivers provide actual transfer paths,
including UART FIFO/DMA operation and bus master/slave support where the
selected MCU has those controllers. I2C master transfers use polling, with
EasyDMA when configured; asynchronous master interrupt handling is unfinished.
The [PDM driver](../ARM/Nordic/src/pdm_nrfx.cpp) captures microphone blocks into
a FIFO, and the [NFC driver](../ARM/Nordic/src/nfct_nrfx.cpp) handles target-mode
frames. These are distinct from the unfinished
[I2S transfer implementation](../ARM/Nordic/src/i2s_nrfx.cpp), which is not
included in the feature claims above.

The nRF52805/52810 library projects include the shared timer dispatcher,
RTC driver and TIMER driver. On both MCUs, timer devices 0 and 1 select RTC0
and RTC1; devices 2, 3 and 4 select TIMER0, TIMER1 and TIMER2. RTC0 provides
three triggers and RTC1 four. Each TIMER provides three triggers, with its
fourth compare register reserved for reading the counter. Avoid peripherals
reserved by the selected SoftDevice or SDK `app_timer` configuration.
The timer sources compile and link at `-O0` and `-Os` with Arm GNU 14.3.1
for both MCUs. These are timer-only checks, not full IOC application builds.
No hardware board is available for validation.

The nRF54L15/LM20 library projects include `i2c_nrfx.cpp` and `spi_nrfx.cpp`.
Both support polling EasyDMA masters (`bIntEn = false`) and interrupt-driven
slaves. Master interrupt operation is unfinished and these ports reject that
mode during initialization. I2C uses 7-bit addresses and selects the closest
100, 250, 400 or 1000 kbit/s rate. SPI uses 8-bit words and selects the closest
rate available from the instance's prescaler. Requests below or above the
available rates return the closest rate; the application decides whether to use
it. Slave clocks come from the external master.

Device numbers follow the same serial-instance order as UART:

| DevNo | Serial instance | I2C | SPI | MCU |
|---|---|---|---|---|
| 0 | SERIAL30 | TWIM/TWIS30 | SPIM/SPIS30 | L15, LM20A/B |
| 1 | SERIAL20 | TWIM/TWIS20 | SPIM/SPIS20 | L15, LM20A/B |
| 2 | SERIAL21 | TWIM/TWIS21 | SPIM/SPIS21 | L15, LM20A/B |
| 3 | SERIAL22 | TWIM/TWIS22 | SPIM/SPIS22 | L15, LM20A/B |
| 4 | SERIAL00 | Not available | SPIM/SPIS00 | L15, LM20A/B |
| 5 | SERIAL23 | TWIM/TWIS23 | SPIM/SPIS23 | LM20A/B |
| 6 | SERIAL24 | TWIM/TWIS24 | SPIM/SPIS24 | LM20A/B |

Do not use the same serial instance for UART, I2C and SPI at the same time.
Choose pins supported by the selected instance in the application's `board.h`.
The new bus code passes Arm compilation and register tests for L15, LM20A and
LM20B; physical I2C/SPI tests are still needed. See the
[bus test instructions](../tests/nrf54_buses/README.md).

The nRF52832 has no native USB controller, and LM20 USB support does not apply
to nRF54L15.

The nRF91 implementation includes
[LTE](../ARM/Nordic/nRF91/src/lte_nrf91.cpp),
[sockets](../ARM/Nordic/nRF91/src/sock_intrf_nrf91.cpp) and
[GNSS](../ARM/Nordic/nRF91/src/gnss_nrf91.cpp), with corresponding applications.
LTE requires a suitable modem firmware, SIM and network; GNSS features depend
on the selected part and modem firmware. On nRF9160 the modem library takes
GNSS requests from modem firmware 1.3.4 on, which needs revision 2 silicon.
Revision 1 runs at most firmware 1.2.8: LTE works there, GNSS does not, and
`GnssNrf91::Init` refuses the older firmware. Bluetooth on nRF91 uses an
external controller, not an on-chip BLE radio.

Radio-stack selections and their capabilities are documented in the
[Bluetooth guide](bluetooth-user-guide.md) and [dependencies](dependencies.md).
The library build configuration selects the relevant stack sources; a feature
available in one stack is not automatically available in another. Existing
SDK-specific examples remain useful, but their presence is not a claim that
all SDK integrations were rebuilt for 0.13.

## Renesas

| MCU | Implemented peripherals | Application examples | Status and validation |
|---|---|---|---|
| [RA4M1](../ARM/Renesas/RA4M1/README.md) | Startup, GPIO, ICU interrupt routing, SCI UART polling/interrupt operation, AGT0/1 and GPT0-7 timers | [Examples](../ARM/Renesas/RA4M1/exemples/): Blinky, TimerDemo, UART loopback and PRBS | Supported; register models and ARM layout checks are documented. Hardware validation is pending. USB and DMA/DTC are not implemented. |
| [RE01 1500 KB](../ARM/Renesas/RE01/RE01_1500KB/lib/ioc/) | Startup, GPIO, interrupt routing, SCI UART, AGT0/1, cascaded TMR0/1 and GPT0-5, polling RIIC0/1 I2C master and SPI0/1 master | [Examples](../ARM/Renesas/RE01/RE01_1500KB/exemples/README.md): UART/SLIP, TaktOS UART, timers, pulse trains, I2C and SPI | Supported; DBN/CFB/CFP compile/link/layout and register-model results are documented. Hardware validation is pending. |
| [R9A02G021](../RISCV/Renesas/R9A02/R9A02G021/lib/ioc/) | RISC-V startup, GPIO, UART and standalone AGT0 millisecond tick | [Examples](../RISCV/Renesas/R9A02/R9A02G021/exemples/): Blinky and DFU projects | Partial port. The tick helper supplies `R9A02_AgtTickInit/Isr`, not IOsonata `TimerInit`. Full IOC/hardware validation is not recorded here. |

RE01's [I2C](../ARM/Renesas/RE01/src/i2c_re01.cpp) and
[SPI](../ARM/Renesas/RE01/src/spi_re01.cpp) implementations perform polling
master transfers. They do not implement slave, interrupt or DMA transfers.
The library project includes both target drivers and the generic bus wrappers
for DBN, CFB and CFP, in Debug and Release configurations.
See [RE01 tests](../tests/re01/README.md) and
[RA4M1 timer details](../ARM/Renesas/RA4M1/TIMER.md).

## Microchip SAM

| MCU | Implemented peripherals | Application examples | Status and validation |
|---|---|---|---|
| [SAM4LCxC](../ARM/Microchip/SAM4L/SAM4LCxC/lib/ioc/) | GPIO, UART, I2C, SPI, AST and six TC timer channels, native USB device | [Examples](../ARM/Microchip/SAM4L/SAM4LCxC/exemples/): UART PRBS/retargeting, I2C master/slave, SPI master/slave/loopback, timers, CDC, HID, bulk, interrupt, ISO, MSC and dual CDC; CDC with TaktOS | Supported; SAM4LC8C hardware validated on SAM4L8 Xplained Pro for startup, GPIO, UART, timers, I2C, SPI and USB. |
| [SAM4LSxC](../ARM/Microchip/SAM4L/SAM4LSxC/README.md) | Same shared SAM4L GPIO, UART, I2C, SPI, AST/TC and USB device drivers | [Examples](../ARM/Microchip/SAM4L/SAM4LSxC/exemples/): matching serial, bus, timer and USB applications, including CDC with TaktOS | Supported; build-tested shared implementation and supplied IOcomposer projects. Not hardware validated; users can build and try the port. |
| [SAM4E16E](../ARM/Microchip/SAM4E/SAM4E16E/lib/ioc/) | Startup, GPIO, UART and polling I2C master implementation | [Examples](../ARM/Microchip/SAM4E/SAM4E16E/exemples/): Blinky, UART PRBS, UART retargeting and DFU project | Partial port. No target IOsonata Timer implementation. I2C slave/interrupt/DMA paths are unfinished; native SPI is not implemented. |

The SAM4L [UART](../ARM/Microchip/SAM4L/src/uart_sam4l.cpp),
[I2C](../ARM/Microchip/SAM4L/src/i2c_sam4l.cpp),
[SPI](../ARM/Microchip/SAM4L/src/spi_sam4l.cpp) and
[USB](../ARM/Microchip/SAM4L/src/usb_ctrlr_sam4l.cpp) drivers are shared by LC
and LS. Application pin maps and selected flash/RAM sizes distinguish projects.
SAM4E has its own [I2C implementation](../ARM/Microchip/SAM4E/src/i2c_sam4e.cpp);
its current transfer and interrupt code does not supply the same modes as SAM4L.

SAM4L is hardware validated, confirmed by the maintainer on 2026-10-08.
The [AST driver](../ARM/Microchip/SAM4L/src/timer_sam4l_ast.cpp) provides
virtual device 0 with one trigger. The
[TC driver](../ARM/Microchip/SAM4L/src/timer_sam4l_tc.cpp) provides devices
1 through 6 with three triggers each. Both SAM4LCxC and SAM4LSxC library
projects include the timer sources.

TimerDemo logs cover AST and TC devices 1, 3 and 6. TC triggers run at 100,
1000 and 250 ms. AST reports 99.609 ms at its default rate, 4000 ms at 1 Hz,
and 500 ms when a 10 Hz request selects 8 Hz. These longer periods follow
the four-tick minimum. The maintainer also confirmed LED0 on PC07.
See [timer tests and limits](../tests/sam4l/README.md).

SAM4L USB provides EP0 and seven physical data endpoints; each data direction
uses a physical endpoint. The complete composite stress configuration exceeds
that endpoint budget. USB host mode is unsupported. See the
[USB guide](usb-user-guide.md#sam4l) for supported individual examples.

## STM32 MCU support

The table describes the current source. L4 review evidence is recorded at
[b2eea8c](https://github.com/IOsonata/IOsonata/commit/b2eea8c4450b371eefffab77ce92aa70f4dc7ea4);
F030x8 now includes all seven peripheral TIM drivers.
Only the named MCUs are covered; support does not extend automatically to their
entire series.

| MCU | Implemented peripherals | Application examples | Status and validation |
|---|---|---|---|
| [STM32L476](../ARM/ST/STM32L4xx/STM32L476/lib/ioc/) | GPIO/EXTI, interrupt-driven UART, I2C, SPI, QSPI and LPTIM1/2 | [Examples](../ARM/ST/STM32L4xx/STM32L476/exemples/): EEPROM, flash memory, motion sensors, SPI master/slave, UART PRBS/SLIP and timers | Supported; hardware validated and used in existing projects. |
| [STM32L496](../ARM/ST/STM32L4xx/STM32L496/lib/ioc/) | GPIO/EXTI, interrupt-driven UART, I2C, SPI, QSPI and LPTIM1/2 | [Examples](../ARM/ST/STM32L4xx/STM32L496/exemples/): flash memory, motion sensors, SPI master/slave, UART PRBS/SLIP and timers | Supported; hardware validated and used in existing projects. |
| [STM32L4S9](../ARM/ST/STM32L4xx/STM32L4S9/lib/ioc/) | GPIO/EXTI, interrupt-driven UART, I2C, SPI, OSPI and LPTIM1/2 | [Examples](../ARM/ST/STM32L4xx/STM32L4S9/exemples/): EEPROM, flash memory, SPI master/slave, UART PRBS/SLIP and timers | Supported; hardware validated and used in existing projects. |
| [STM32F030x8](../ARM/ST/STM32F0xx/STM32F030x8/lib/ioc/) | GPIO/EXTI, UART with interrupt/FIFO and TX DMA, TIM6/14/16/17/15/3/1 | [Examples](../ARM/ST/STM32F0xx/STM32F030x8/exemples/README.md): UART loopback/PRBS/SLIP, TaktOS UART, timer triggers, pulse trains, software crypto and DFU image verification | Supported; hardware validated. Native I2C/SPI/ADC drivers are not supplied by this port. |
| STM32F401xC | Startup, vector and linker files | Library project | Incomplete: target GPIO, UART and Timer drivers are missing. |
| STM32F301x8, STM32F302x8 | Vector files | No complete MCU application project | Incomplete startup and peripheral ports. |
| STM32WBA | Partial Bluetooth sources and linker support | No complete MCU application project | Incomplete startup, GPIO, UART and Timer integration. |
| STM32L152 | Planned | Planned after 0.13 | Outside the 0.13 support list. |

The L4 [I2C](../ARM/ST/STM32L4xx/src/i2c_stm32l4xx.cpp) driver supports polling
master transfers. The [SPI driver](../ARM/ST/STM32L4xx/src/spi_stm32l4xx.cpp)
provides normal SPI transfers and slave receive interrupts; its DMA transfer
functions are unfinished. [QSPI](../ARM/ST/STM32L4xx/src/quadspi_stm32l4xx.cpp)
on L476/L496 and [OSPI](../ARM/ST/STM32L4xx/src/octospi_stm32l4xx.cpp) on L4S9
support polling master configurations. These interfaces allow applications to
use the portable memory and sensor drivers already shown in the examples.

L4 UART virtual devices 0-5 support seven/eight payload bits, none/even/odd
parity and one/two stop bits. LPUART1 cannot use seven payload bits without
parity; nine-bit payloads are unsupported. Timers use LPTIM1/2 at virtual
indices 0/1; general-purpose TIM indices remain unimplemented. See the
[L4 port notes](../ARM/ST/STM32L4xx/README.md) for mappings and mode restrictions.

### STM32 validation evidence

The maintainer confirms STM32F0 and STM32L4 hardware validation. The listed
STM32L476, STM32L496 and STM32L4S9 ports are also used in existing projects
(confirmation recorded 2026-10-08). They are not host-only or awaiting initial
board validation. The detailed tests below describe additional review coverage
and do not replace that established hardware use.

For the L4 fixes above, 12 focused host regression executions passed with
UBSan, including clock-source decoding, UART framing and receive overflow,
LPTIM disable/enable, and virtual-device dispatch. Twenty additional ARM
translation-unit builds passed using Arm GNU 14.3.1 at `-O0` and `-Os`:
startup, UART and LPTIM for all three MCUs, plus the alternative L4+ startup
source for L4S9. These are compilation and host-model results, not complete
IOC library/application links or hardware tests.

F030x8 has existing maintainer-reported use in TaktOS benchmarks. That evidence
is separate from the peripheral timer tests: the benchmark uses
SysTick for kernel timing and TIM17 as an IRQ probe. The new timers pass host
register tests and Cortex-M0 archive-link smoke checks, including strong
SysTick/TIM17 overrides. See the [F030x8 port notes](../ARM/ST/STM32F0xx/README.md)
for virtual ordering, capabilities and remaining board checks. The
[F030x8 example index](../ARM/ST/STM32F0xx/STM32F030x8/exemples/README.md)
lists the target projects and separates peripheral examples from software-only
crypto tests.

On 2026-10-08, the maintainer confirmed STM32F0308-DISCO startup, LED GPIO,
USART1 TX/stdio retargeting, TIM6 (virtual device 0) and TIM16 (virtual device 2).
TIM6's UART log contains 115 consecutive 100000 us intervals over 11.5 seconds.
The previous timing variation came from `rdimon` semihosting; a missing
level-shifter supply explained the temporary loss of UART output.

The implementation was merged in [PR 75](https://github.com/IOsonata/IOsonata/pull/75).
The exact flashed revision, successful build profile and complete tool versions
were not recorded with these runs. This is maintainer hardware evidence for
those functions, not a test of every mode or all seven timers. TIM14, TIM17,
TIM15, TIM3 and TIM1 still need hardware coverage.
The planned L152/Nucleo-64 work is outside the 0.13 support list.

## NXP, Espressif and Raspberry Pi

| MCU | Available implementation | Examples and remaining work |
|---|---|---|
| [LPC11U35](../ARM/NXP/LPC11xx/LPC11U35/lib/ioc/) | Legacy startup, GPIO, UART, I2C, SSP/SPI and IAP flash code | [Blinky and UART PRBS projects](../ARM/NXP/LPC11xx/LPC11U35/exemples/) are supplied. No target IOsonata Timer implementation; current interface/build compatibility needs verification. The USB source is only an include, not a working USB device driver. |
| [LPC1769](../ARM/NXP/LPC17xx/LPC1769/lib/ioc/) | Legacy startup, GPIO, UART, I2C and IAP flash code; shared SSP transfer code is linked | [DFU project](../ARM/NXP/LPC17xx/LPC1769/exemples/). No target Timer implementation or MCU-specific SPI initializer; current builds need verification. |
| [LPC54605](../ARM/NXP/LPC546xx/LPC54605/lib/ioc/) | Startup, vector, linker and IAP/DFU code | Incomplete GPIO, UART and Timer integration; shared LPC files do not provide a complete LPC54605 port. |
| [LPC54628](../ARM/NXP/LPC546xx/LPC54628/lib/ioc/) | Startup with the boot ROM vector checksum, core clock on the 12 MHz FRO, GPIO, pin interrupts and IAP | [Blinky project](../ARM/NXP/LPC546xx/LPC54628/exemples/) for the LPCXpresso54628. Port in progress: UART, Timer, I2C and SPI are not supplied yet. The core stays at 12 MHz because a higher clock needs the core voltage set by NXP's binary power library. No hardware result recorded yet. |
| [ESP32-C3](../RISCV/Espressif/ESP32C/ESP32C3/lib/ioc/) | RISC-V startup, GPIO, UART, clock control and interrupt routing | [Blinky and UART PRBS projects](../RISCV/Espressif/ESP32C/ESP32C3/exemples/). Partial port: no IOsonata Timer driver. The I2C source still contains Nordic register code and is not an ESP32 bus implementation. |
| [ESP32-C6](../RISCV/Espressif/ESP32C/ESP32C6/lib/ioc/) | Startup, peripheral clock/reset and interrupt-routing code | [Initial examples](../RISCV/Espressif/ESP32C/ESP32C6/exemples/). Incomplete port: no Timer driver; the library also omits shared Espressif GPIO/UART files. |
| [RP2040](../ARM/RPI/RP20/RP2040/lib/ioc/) | Initial library project and generic sources | Native startup, GPIO, UART and Timer integration is absent. Incomplete port. |

LPC11U35 combines its [SPI initializer](../ARM/NXP/LPC11xx/src/spi_lpc11uxx.c)
with [shared SSP transfers](../ARM/NXP/src/spi_lpcxx.c). The older LPC sources
are useful existing implementations, but require current build and API checks.
ESP32-C3 uses a native [UART driver](../RISCV/Espressif/src/uart_esp32.cpp)
with separate clock and interrupt modules. Wi-Fi/Bluetooth hardware on an
Espressif chip does not imply that an IOsonata radio port is supplied here.

## Recorded hardware evidence

This table records the scope of maintainer results, independently of the
implementation descriptions. It is not a claim that every row was tested at
the review commit or that every project passed both IOC build profiles.

| MCU | Hardware / configuration recorded | Exercised functions | Evidence scope |
|---|---|---|---|
| nRF52832 | IDK-BLYST-NANO, BLUEIO-TAG-EVIM, Nordic nRF52 DK | BLE, UART, sensors and low-power applications | Established application and peripheral development platform. |
| nRF54L15 | BLYSTL15, Nordic nRF54L15 DK | UART, Bluetooth and TaktOS benchmarks | Established Bluetooth and RTOS development platform. |
| nRF52840 | Composite USB, bare metal and TaktOS | CDC loopback/PRBS, HID, interrupt and ISO endurance | 2000-second results recorded in the [0.13 notes](releases/0.13.md#recorded-validation); exact firmware SHA/toolchain not supplied with those reports. |
| nRF54LM20 | Composite USB, bare metal and TaktOS | High-speed composite USB endurance | 2000-second results in the same release record; exact board/variant and firmware revision must accompany future runs. |
| nRF9160 | `LteModemInfo`, modem library `3.5.0-cellular-44ef973ed31b`, firmware `mfw_nrf9160_1.2.8` | Modem initialization and AT communication | Maintainer output on 2026-10-08: firmware, identity, hardware-version and functional-mode queries returned `OK`; `CFUN` was 0. This run did not exercise network registration, data transfer or GNSS. |
| nRF9160 | `LteUdpTaktOS`, firmware `mfw_nrf9160_1.2.8` | LTE-M registration and UDP transfer with TaktOS | Maintainer output on 2026-10-08: RRC connected, roaming registration, 32-byte send and matching 32-byte echo; reported RSRP -94 dBm. GNSS and network endurance were not tested in this log. |
| nRF9160 revision 1 | GNSS probe, modem library `3.5.0-cellular-44ef973ed31b`, firmware `mfw_nrf9160_1.2.8` | GNSS | Maintainer output on 2026-10-09: every GNSS setting refused with `EOPNOTSUPP`; the modem library reports that firmware 1.3.4 or newer is needed; the start returns but no position data arrives. GNSS is not available on revision 1. |
| nRF9161 (nRF91x1 port) | `LteUdpTaktOS` | LTE-M registration and UDP transfer with TaktOS | Maintainer confirmed the same successful registration and UDP echo on 2026-10-08. Modem firmware version was not supplied; GNSS and network endurance were not reported. |
| SAM4LC8C | SAM4L8 Xplained Pro | Startup, LED GPIO, UART, AST/TC timers, I2C, SPI, CDC loopback, ISO and manual suspend/wake | Hardware validated, confirmed 2026-10-08. See [timer results](../tests/sam4l/README.md) and [USB results](releases/0.13.md#sam4l-usb-port). |
| STM32F030x8 | STM32F0308-DISCO | Hardware validated: startup, GPIO, UART and timers; later UART TX DMA PRBS | Maintainer-confirmed validation. Detailed recorded timer runs cover TIM6/TIM16; coverage of additional modes is tracked separately. |
| STM32L476, STM32L496, STM32L4S9 | Maintainer's existing projects | Hardware validated and used in projects | Confirmed by the maintainer on 2026-10-08; board names and per-project configurations were not supplied in this confirmation. |

For RA4M1 and RE01, retain the model/compiler/linker results in their
port notes without relabeling them as physical tests. STM32L4 has separate
maintainer-confirmed hardware validation and project use. An unrecorded hardware
result is unknown, not proof that the port fails.

## Release build and example status

At `05e00ed`, clean USB and Bluetooth host builds/tests passed with
GCC 13.3, ASan/UBSan and leak detection disabled. The SAM4LC8C bare-metal CDC
example and its required sources compiled and linked with Arm GNU 14.3.1 in Debug and Release;
the existing RWX LOAD-segment warning remains. That script builds the sources required by the CDC example. It does not build
the complete IOC MCU library or the SAM4L timer sources.
The separate TimerDemo build check compiles and links all seven timer device
selections in Debug and Release with Arm GNU 14.3.1.

Full IOC library/application builds and final hardware checks are required
before release. Earlier measurements do not certify later changes.

The legacy `UartSdkLoopbackTest` and `UartLoopbackSdk5` target projects were
removed in [PR 79](https://github.com/IOsonata/IOsonata/pull/79).
The shared SDK UART source remains. The nRF52840
[BuzzerDemo](../ARM/Nordic/nRF52/nRF52840/exemples/BuzzerDemo/README.md)
uses the shared audio-layer buzzer player in
`exemples/audio/buzzer_demo.cpp` with application-owned `board.h` pin
selection. Host playback tests and selected-source Debug/Release ARM builds
have passed; Nordic PWM register tests cover stop/restart and inactive-pin
handling. These are not full IOC library builds. The latest player, named
effects and sweep quality have not yet been listening-tested on hardware.

## Desktop targets

macOS, Linux and Windows source is for host interfaces, tools and tests.
These are not MCU ports and are not included in the MCU tables.
Host tests cannot establish peripheral timing or electrical behavior.

## Updating this inventory

For implementation changes, describe the usable peripherals and applications,
link their drivers and projects, and record mode or integration limits beside
them. Update all affected MCU rows, including shared implementations. Keep
port maturity separate from the scope of an individual hardware test.

For build or hardware results, record the MCU and board, IOsonata commit,
IOcomposer/compiler versions, SDK or binary-component revisions, Debug/Release
build result, applications exercised and remaining limitations. Preserve
older maintainer results with their original scope when some metadata is
missing; do not silently promote them to a test of the current tree.

## Reporting a target issue

Include the MCU/core and board, project path, Debug/Release profile, IOsonata
revision, compiler/SDK versions, relevant `board.h` configuration, complete
build log and flash/runtime output.

## Related documentation

- [Documentation index](README.md)
- [Getting Started](getting-started.md)
- [IOcomposer workflow](architecture/iocomposer-workflow.md)
- [0.13 release notes](releases/0.13.md)
