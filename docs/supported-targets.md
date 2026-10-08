# Supported Targets

This document records MCU implementation support and hardware-validation evidence separately. The STM32 table reflects the startup, GPIO, UART and timer review for 0.13; it is not a claim that every peripheral on those MCUs is implemented.

The presence of a target port or build project means that implementation source exists. It does not by itself mean that the target is part of the current hardware-validation loop.

## Minimum MCU support

An MCU must provide startup, GPIO, UART and timer support to be considered supported.
A device header, vector table or library project alone is insufficient. Record
which timer driver and UART modes are implemented; other peripherals have
separate capability limits. Hardware-validation status is recorded separately
from this minimum implementation requirement.

## Status terms

- **Supported (minimum implemented)** - startup, GPIO, UART and at least one timer driver are implemented. Optional peripherals and modes have separate limits; full builds and hardware results are recorded independently.
- **Incomplete MCU port** - one or more minimum requirements are missing. Headers, linker scripts, vector tables or an available project do not make the MCU supported.
- **Hardware validated** — the current tree has been built and exercised on the named hardware using the documented IOcomposer workflow.
- **Build project available** — a target library project exists and can be selected by the installed builder, but it is not necessarily part of routine hardware validation.
- **Experimental** — implementation work is incomplete, locally dependent or awaiting broader hardware coverage.
- **Legacy** — source or projects remain for historical compatibility but are not maintained as a current baseline.

## Current hardware-validation baselines

| MCU target | Reference hardware | Role |
|---|---|---|
| Nordic nRF52832 | IDK-BLYST-NANO, BLUEIO-TAG-EVIM, Nordic nRF52 DK | Primary Cortex-M4F, Bluetooth, UART, sensor and low-power baseline |
| Nordic nRF54L15 | BLYSTL15, Nordic nRF54L15 DK | Cortex-M33 and `sdk-nrf-bm` bare-metal baseline |

These rows identify the active reference targets. Feature coverage still varies by target and subsystem. A hardware-validated MCU does not imply that every optional interface, radio stack, storage mode or crypto provider has the same validation depth.

The maintainer confirmed the STM32F030x8 minimum port on STM32F0308-DISCO:
startup, LED GPIO, USART1 UART output/retargeting, TIM6 and TIM16. See the
STM32 validation record below. The Nordic rows above identify routine
reference platforms; they are not the complete supported-MCU list.

## STM32 MCU support

The table describes the current source. L4 review evidence is recorded at
[b2eea8c](https://github.com/IOsonata/IOsonata/commit/b2eea8c4450b371eefffab77ce92aa70f4dc7ea4);
F030x8 now includes all seven peripheral TIM drivers.
Only the named MCUs are covered; support does not extend automatically to their
entire series.

| MCU | Startup | GPIO | UART | Timer | MCU support |
|---|---|---|---|---|---|
| STM32L476 | Implemented | Implemented | Interrupt-driven UART, virtual devices 0-5 | LPTIM1/2, virtual devices 0/1 | Supported (minimum implemented) |
| STM32L496 | Implemented | Implemented | Interrupt-driven UART, virtual devices 0-5 | LPTIM1/2, virtual devices 0/1 | Supported (minimum implemented) |
| STM32L4S9 | Implemented | Implemented | Interrupt-driven UART, virtual devices 0-5 | LPTIM1/2, virtual devices 0/1 | Supported (minimum implemented) |
| STM32F030x8 | Implemented | Implemented, including EXTI | Implemented | TIM6, TIM14, TIM16, TIM17, TIM15, TIM3, TIM1 | Supported (minimum implemented) |
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

## Other source ports

IOsonata contains additional Arm, RISC-V and host implementations under:

```text
ARM/
RISCV/
Linux/
Win/
OSX/
```

The installed MCU-library builder discovers the build projects available in the current checkout. Use that menu to determine which target libraries can be built by the installed environment.

Do not convert directory presence into a support claim. Read the complete target implementation, its example projects and recent test history before describing its status.

## Desktop targets

| Platform | Status | Intended use |
|---|---|---|
| macOS | Best effort | Host-supported interfaces, tools and test harnesses |
| Linux | Best effort | Host-supported interfaces, tools and test harnesses |
| Windows | Best effort | Host-supported interfaces, tools and test harnesses |

Desktop support does not mean that MCU peripheral drivers execute unchanged on a desktop operating system. Hardware-dependent behaviour must still be validated on the target MCU and board.

## Promoting a target to hardware validated

Record all of the following in the same change:

1. exact MCU and board;
2. IOsonata commit;
3. IOcomposer and compiler versions;
4. vendor SDK or binary-component revision where applicable;
5. Debug and Release library-build result;
6. application examples exercised on hardware;
7. flash, runtime and data-integrity results relevant to the subsystem;
8. any known unsupported modes or local requirements.

A target requiring uncommitted local patches is not hardware validated from the current tree.

## Reporting a target issue

Include:

- host operating system;
- IOcomposer version;
- compiler version;
- target project path;
- MCU and board name;
- Debug or Release profile;
- example or application name;
- relevant board configuration;
- complete build log;
- flash and runtime output when applicable.

## Related documentation

- [Documentation index](README.md)
- [Getting Started](getting-started.md)
- [IOcomposer workflow](architecture/iocomposer-workflow.md)
- [Architecture overview](architecture/README.md)
