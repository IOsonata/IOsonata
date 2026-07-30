# Supported Targets

This document records the current IOsonata hardware-validation baselines. It is not an inventory of every MCU directory, source port or historical project in the repository.

The presence of a target port or build project means that implementation source exists. It does not by itself mean that the target is part of the current hardware-validation loop.

## Status terms

- **Hardware validated** — the current tree has been built and exercised on the named hardware using the documented IOcomposer workflow.
- **Build project available** — a target library project exists and can be selected by the installed builder, but it is not necessarily part of routine hardware validation.
- **Experimental** — implementation work is incomplete, locally dependent or awaiting broader hardware coverage.
- **Legacy** — source or projects remain for historical compatibility but are not maintained as a current baseline.

## Current hardware-validation baselines

| MCU target | Reference hardware | Role |
|---|---|---|
| Nordic nRF52832 | IDK-BLYST-NANO, BLUEIO-TAG-EVIM, Nordic nRF52 DK | Primary Cortex-M4F, Bluetooth, UART, sensor and low-power baseline |
| Nordic nRF54L15 | BLYSTL15, Nordic nRF54L15 DK | Cortex-M33 and `sdk-nrf-bm` bare-metal baseline |
| STM32 baseline | STM32 development boards | Current STM32 startup, GPIO, UART and peripheral-port baseline |

These rows identify the active reference targets. Feature coverage still varies by target and subsystem. A hardware-validated MCU does not imply that every optional interface, radio stack, storage mode or crypto provider has the same validation depth.

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
