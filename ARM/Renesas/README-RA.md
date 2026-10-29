# Renesas MCU directory organization

Renesas MCU **series live directly under `ARM/Renesas`**, with individual
MCU groups nested below their series. Do not introduce an extra `RA/` parent.

```text
ARM/Renesas/
    include/          # common across compatible Renesas MCU families
    src/              # common across compatible Renesas MCU families
    RA0/
        <MCU-group>/
    RA2/
        <MCU-group>/
    RA4/
        RA4M1/
            include/
            src/
            ldscript/
            lib/ioc/
            lib/e2/
            exemples/
    RA6/
        <MCU-group>/
    RA8/
        <MCU-group>/
    RE01/
        RE01_1500KB/
```

The directories above describe the *intended taxonomy*, not a claim that
all listed RA series have implementations. Create actual directories as
supported targets are introduced. RISC-V Renesas targets stay under
`RISCV/Renesas`.

## Sharing code

Share proven peripheral-IP-compatible code at the narrowest useful scope,
for example `ARM/Renesas/RA4/include` and `ARM/Renesas/RA4/src`
for common RA4 IP, and target-specific code under `RA4/RA4M1`.
Do not presume register, IRQ, clock, flash, DMAC, security or peripheral
compatibility solely from the series name. Renesas-wide reusable code belongs in `ARM/Renesas/include` and
`ARM/Renesas/src`, but only after compatibility is verified across the
Renesas targets that compile it. This level is shared by RA and RE01 where
appropriate, not a dumping ground for hardware-specific implementations.
Do not force all Renesas devices to compile a common source if the underlying
peripheral IP differs.

Keep IOsonata's existing public interfaces, native MCU startup and
device-header selection, with no FSP dependency. MCU library projects select
MCU drivers; applications never select crypto or other MCU drivers.
Board headers define pin mapping, not peripheral register initialization.

## Current shared implementation

RA4M1 and RE01 use common IOsonata interfaces, but their peripheral register
implementations remain target-specific. `ARM/Renesas/src` is reserved for
implementations shared by compatible targets. Common declarations, including
`ARM/Renesas/include/usb_ctrlr.h`, do not imply a common register implementation.

The RA4M1 library is `RA4/RA4M1/lib/ioc`; its project name remains
`IOsonata_RA4M1`. See [the port guide](RA4/RA4M1/README.md) for setup and
peripheral limits, and [e2 studio projects](#e2-studio-projects) for that IDE.

## e2 studio projects

The `e2` directories contain Eclipse CDT / GNU Arm project configurations
for Renesas e2 studio. They preserve the existing IOsonata MCU-centric source
links and Debug/Release settings from the corresponding `ioc` configurations.

### Projects

- `RA4/RA4M1/lib/e2` plus Blinky, TimerDemo, UartLoopback and UartPrbsTxTest
  example projects in their respective `exemples/<name>/e2` directories.
- `RE01/RE01_1500KB/lib/e2` plus Blinky, TimerDemo, UartRetargetDemo,
  UartPrbsTxTest, I2CMasterDemo, SPIMasterDemo, PulseTrain and
  UartSlipPrbsRxTest example projects.
- Existing `RISCV/Renesas/R9A02/R9A02G021/lib/e2` is separate and uses
  its own native Renesas LLVM RISC-V toolchain. All fourteen R9A02G021
  examples also have `e2` projects; see the
  [R9A02G021 instructions](../../RISCV/Renesas/R9A02/R9A02G021/exemples/README.md).

Import the `lib/e2` project and desired example `e2` projects with
**File > Import > Existing Projects into Workspace**. Install an e2 studio
version with the Eclipse CDT managed-build / GNU Arm Embedded toolchain
integration and select an Arm GNU toolchain compatible with each target.
The generated projects are renamed with the `_e2` suffix to coexist
with the IOcomposer projects. Build the matching library configuration
before the example; the copied linker search paths select `lib/e2`.

RE01 configurations distinguish CFB, CFP and DBN package variants; select
the same variant and Debug/Release configuration in library and example.
Use application-local `board.h` for wiring. Library projects add no board
initialization.

These are **portable CDT import configurations**, not projects generated
by Renesas FSP/Smart Configurator. They have not been compiled inside licensed
or separately installed e2 studio, and executable/debugger/device-pack setup
must be validated with the installed IDE and the target hardware.
