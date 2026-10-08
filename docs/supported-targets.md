# Supported MCUs

Initial review: main `05e00ed094aaa0f5cb21493e5ed8dda871de7c26`, 2026-10-08.
Updated for the SAM4L timers and maintainer hardware validation in
[PR 81](https://github.com/IOsonata/IOsonata/pull/81).
This inventory covers all 30 MCU library projects under
`ARM/` and `RISCV/`, plus source-only and planned targets noted below.

## What support means

**Minimum MCU support** requires implemented startup, GPIO, UART and an
IOsonata timer implementation. Optional peripherals, project integration, build
verification and hardware results are separate facts. A generic wrapper such
as `src/coredev/timer.cpp` does not supply a MCU-specific timer implementation.

- **Supported**: startup, GPIO, UART and timers are implemented.
  Other implemented peripherals are described in the target notes.
  This does not certify every peripheral, operating mode or example.
- **Project integration incomplete**: reusable target code exists, but the
  supplied library project does not include the required implementation.
- **Incomplete port**: required target code is missing or contains unfinished
  operations. A device header or library project alone is insufficient.
- **Hardware evidence**: a recorded run of named functions on a target.
  Earlier results remain useful, but do not certify the current commit.
- **Build/model evidence**: compiler, linker or host register tests. These
  are not hardware tests or substitutes for a full IOC application build.

The library builder discovers projects; its menu is not a support-status list.
Timer device numbers are virtual indices, not hardware timer numbers. Shared
ARM SysTick support is separate from the MCU-specific Timer implementations listed here.

## Nordic

All rows below use the shared Nordic GPIO and UART implementations. The
startup and timer selections differ by MCU/core.

| MCU / core | Implementation and project status | Timer implementation | Validation and limits |
|---|---|---|---|
| [nRF52832](../ARM/Nordic/nRF52/nRF52832/lib/ioc/) | Supported | RTC + TIMER | Established BLE, UART, sensor and low-power hardware baseline. No native USB controller port. |
| [nRF52840](../ARM/Nordic/nRF52/nRF52840/lib/ioc/) | Supported | RTC + TIMER | Recorded bare-metal and TaktOS USB composite endurance runs; native full-speed USB. |
| [nRF54L15](../ARM/Nordic/nRF54/nRF54L15/lib/ioc/) | Supported | GRTC + TIMER | Established UART/Bluetooth and TaktOS hardware baseline. LM20 USB support does not extend to L15. |
| [nRF54LM20A/B](../ARM/Nordic/nRF54/nRF54LM20x/lib/ioc/) | Supported | GRTC + TIMER | Recorded bare-metal and TaktOS high-speed USB composite runs. Select the matching LM20 variant in the project. |
| [nRF5340 application core](../ARM/Nordic/nRF53/nRF5340_App/lib/ioc/) | Supported | RTC + TIMER | Separate startup and shared-interface dispatch. Current full IOC builds and hardware results are not established by this review. |
| [nRF5340 network core](../ARM/Nordic/nRF53/nRF5340_Net/lib/ioc/) | Supported | RTC + TIMER | Separate network-core startup and serial dispatch. Application-core results do not validate this core or an inter-core radio transport. |
| [nRF9160](../ARM/Nordic/nRF91/nRF9160/lib/ioc/) | Supported | RTC + TIMER | Shared nRF91 startup. LTE/GNSS depend on modem integration and firmware; MCU support does not certify those services. |
| [nRF91x1 project](../ARM/Nordic/nRF91/nRF91x1/lib/ioc/) | Supported; exact variant configuration matters | RTC + TIMER | Debug/Release currently define `NRF9120_XXAA`. The directory name is not evidence of a build or hardware test for every nRF91 variant. |
| [nRF52805](../ARM/Nordic/nRF52/nRF52805/lib/ioc/), [nRF52810](../ARM/Nordic/nRF52/nRF52810/lib/ioc/) | Project integration incomplete | Shared RTC/TIMER sources available, absent from these library source lists | Projects link the generic timer wrapper and SDK `app_timer.c`, but omit `timer_nrfx.cpp`, `timer_lf_nrfx.cpp` and `timer_hf_nrfx.cpp`. Restore the target Timer integration and verify builds before treating these projects as complete. |
| nRF54H20 [application](../ARM/Nordic/nRF54/nRF54H20/nRF54H20_App/lib/ioc/), [network](../ARM/Nordic/nRF54/nRF54H20/nRF54H20_Net/lib/ioc/) and [RISC-V](../RISCV/Nordic/nRF54/nRF54H20/lib/ioc/) | Incomplete ports | Target integration incomplete | App startup link names missing `system_nrf54h.c`; H20 peripheral, NRFS USB, MPSL and inter-core HCI integration remain incomplete. See [H20 notes](../ARM/Nordic/nRF54/nRF54H20/README.md). |

Source evidence: [GPIO](../ARM/Nordic/src/iopincfg_nrfx.c),
[UART](../ARM/Nordic/src/uart_nrfx.cpp),
[timer dispatch](../ARM/Nordic/src/timer_nrfx.cpp),
[RTC](../ARM/Nordic/src/timer_lf_nrfx.cpp),
[GRTC](../ARM/Nordic/nRF54/src/timer_lf_nrf54.cpp) and
[TIMER](../ARM/Nordic/src/timer_hf_nrfx.cpp).
Radio stack and SDK choices are described in the
[Bluetooth guide](bluetooth-user-guide.md) and [dependencies](dependencies.md).
A family-shared implementation does not establish hardware coverage for every
part or core.

## Renesas

| MCU | Implementation and project status | UART / timers | Validation and limits |
|---|---|---|---|
| [RA4M1](../ARM/Renesas/RA4M1/README.md) | Supported | SCI polling/interrupt; AGT0/1 and GPT0-7 | Startup/GPIO/ICU/UART/timer models and ARM layout checks are documented. Physical validation remains pending. No native USB controller or DMA/DTC support in this port. See [timer details](../ARM/Renesas/RA4M1/TIMER.md). |
| [RE01 1500 KB](../ARM/Renesas/RE01/RE01_1500KB/lib/ioc/) | Supported | SCI polling/interrupt; AGT0/1, cascaded TMR0/1 and GPT0-5 (nine virtual devices) | DBN/CFB/CFP example compile/link/layout checks and register models are documented. SPI and I2C currently provide polling master modes; do not claim interrupt/DMA or slave support. See [examples](../ARM/Renesas/RE01/RE01_1500KB/exemples/README.md) and [validation](../tests/re01/README.md). |
| [R9A02G021](../RISCV/Renesas/R9A02/R9A02G021/lib/ioc/) | Partial port | Startup, GPIO and UART sources; standalone AGT0 millisecond tick | `agt_tick_r9a02.c` supplies `R9A02_AgtTickInit/Isr`, not the generic `TimerInit` implementation. The tick helper does not establish complete IOsonata Timer support. Current full IOC/hardware validation is not recorded here. |

## Microchip SAM

| MCU project | Implementation and project status | Timer status | Validation and limits |
|---|---|---|---|
| [SAM4LCxC](../ARM/Microchip/SAM4L/SAM4LCxC/lib/ioc/) | Supported; hardware validated | AST + six TC channels | SAM4LC8C on SAM4L8 Xplained Pro: startup, GPIO, UART, timers, I2C, SPI and USB. See the recorded tests below. |
| [SAM4LSxC](../ARM/Microchip/SAM4L/SAM4LSxC/README.md) | Supported; shares SAM4L drivers | AST + six TC channels | Shared SAM4L drivers, IOcomposer library, Blinky and TimerDemo projects. Not hardware validated; users can build and try the port. |
| [SAM4E16E](../ARM/Microchip/SAM4E/SAM4E16E/lib/ioc/) | Incomplete minimum port | No target Timer implementation in the repository/project | Startup, GPIO and UART sources exist. The generic timer wrapper alone does not complete the port. |

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

| MCU | Startup | GPIO | UART | Timer | MCU support |
|---|---|---|---|---|---|
| STM32L476 | Implemented | Implemented | Interrupt-driven UART, virtual devices 0-5 | LPTIM1/2, virtual devices 0/1 | Supported; hardware validated |
| STM32L496 | Implemented | Implemented | Interrupt-driven UART, virtual devices 0-5 | LPTIM1/2, virtual devices 0/1 | Supported; hardware validated |
| STM32L4S9 | Implemented | Implemented | Interrupt-driven UART, virtual devices 0-5 | LPTIM1/2, virtual devices 0/1 | Supported; hardware validated |
| STM32F030x8 | Implemented | Implemented, including EXTI | Implemented | TIM6, TIM14, TIM16, TIM17, TIM15, TIM3, TIM1 | Supported; hardware validated |
| STM32F401xC | Implemented | No target GPIO driver | No target UART driver | No target timer driver | Incomplete MCU port |
| STM32F301x8, STM32F302x8 | Vector files only; startup port incomplete | No target GPIO driver | No target UART driver | No target timer driver | Incomplete MCU port |
| STM32WBA | Partial Bluetooth sources and linker support; startup missing | No target GPIO driver | No target UART driver | No target timer driver | Incomplete MCU port |
| STM32L152 | Not implemented | Not implemented | Not implemented | Not implemented | Planned after 0.13 |

The L4 UART supports seven/eight payload bits, none/even/odd parity and one/two
stop bits. LPUART1 cannot use seven payload bits without parity; nine-bit
payload requests are rejected. General-purpose TIM is not implemented:
initialization rejects its reserved indices and reports zero high-frequency
timers. LPTIM is sufficient for the minimum timer requirement.

L4 I2C and extended SPI (QSPI/OSPI) currently accept polling master
configurations only. These optional-peripheral limits do not change the
minimum MCU classification. See the [L4 port notes](../ARM/ST/STM32L4xx/README.md)
for virtual mappings, exact restrictions and validation commands.

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

| MCU project | Available implementation | Missing minimum support / integration |
|---|---|---|
| [LPC11U35](../ARM/NXP/LPC11xx/LPC11U35/lib/ioc/) | Legacy startup, GPIO, UART and other peripheral sources | No target Timer implementation; older interfaces/projects require current build verification. |
| [LPC1769](../ARM/NXP/LPC17xx/LPC1769/lib/ioc/) | Legacy startup, GPIO and UART sources | No target Timer implementation; older interfaces/projects require current build verification. |
| [LPC54605](../ARM/NXP/LPC546xx/LPC54605/lib/ioc/) | Startup/vector/linker project | Target GPIO, UART and Timer integration incomplete. Shared LPC code is not proof of a working LPC54605 port. |
| [ESP32-C3](../RISCV/Espressif/ESP32C/ESP32C3/lib/ioc/) | Startup, GPIO and UART sources, clock and interrupt routing | No IOsonata target Timer implementation. Optional bus sources do not establish complete peripheral support. |
| [ESP32-C6](../RISCV/Espressif/ESP32C/ESP32C6/lib/ioc/) | Startup, PCR and interrupt-routing sources | No target Timer implementation; the library source list also omits the shared Espressif GPIO/UART implementation. Fixing the Blinky source link alone does not complete this port. |
| [RP2040](../ARM/RPI/RP20/RP2040/lib/ioc/) | Project scaffold and generic sources | Native startup/GPIO/UART/Timer integration absent. Incomplete port. |

## Recorded hardware evidence

This table records the scope of maintainer results, independently of the
minimum-port classification. It is not a claim that every row was tested at
the review commit or that every project passed both IOC build profiles.

| MCU | Hardware / configuration recorded | Exercised functions | Evidence scope |
|---|---|---|---|
| nRF52832 | IDK-BLYST-NANO, BLUEIO-TAG-EVIM, Nordic nRF52 DK | BLE, UART, sensors and low-power applications | Established reference baseline; specific configurations still matter. |
| nRF54L15 | BLYSTL15, Nordic nRF54L15 DK | UART, Bluetooth and TaktOS benchmarks | Established reference baseline. |
| nRF52840 | Composite USB, bare metal and TaktOS | CDC loopback/PRBS, HID, interrupt and ISO endurance | 2000-second results recorded in the [0.13 notes](releases/0.13.md#recorded-validation); exact firmware SHA/toolchain not supplied with those reports. |
| nRF54LM20 | Composite USB, bare metal and TaktOS | High-speed composite USB endurance | 2000-second results in the same release record; exact board/variant and firmware revision must accompany future runs. |
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
The shared SDK UART source remains. The nRF52840 `PwmToneDemo` project still
references missing `exemples/pwm/pwm_tone_demo.cpp`; its presence is not a
working-example claim. An incomplete example does not by itself invalidate
the MCU's core drivers.

## Desktop targets

macOS, Linux and Windows source is for host interfaces, tools and tests.
These are not MCU ports and are not included in the minimum-driver matrix.
Host tests cannot establish peripheral timing or electrical behavior.

## Updating this inventory

For implementation changes, name the MCU/core, link the target driver and
library project, and record any intentionally unsupported modes. Keep a
partial driver explicit instead of marking an entire family as supported.

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
