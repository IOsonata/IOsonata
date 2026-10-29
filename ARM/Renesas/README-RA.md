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
peripheral limits, and [e2 studio projects](README-e2-studio.md) for that IDE.
