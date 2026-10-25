# Renesas MCU directory organization

Renesas MCU **series live directly under `ARM/Renesas`**, with individual
MCU groups nested below their series. Do not introduce an extra `RA/` parent.

```text
ARM/Renesas/
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
compatibility solely from the series name. Cross-series code should live in
an explicitly named common directory only after compatibility is verified.

Keep IOsonata's existing public interfaces, native MCU startup and
device-header selection, with no FSP dependency. MCU library projects select
hardware backends; applications never select crypto or other MCU drivers.
Board headers define pin mapping, not peripheral register initialization.

## RA4M1 migration checklist

The currently implemented RA4M1 port is still located at
`ARM/Renesas/RA4M1`; moving it to `ARM/Renesas/RA4/RA4M1` requires an
atomic update of linked Eclipse resources, relative project paths,
compiler include/search paths, linker scripts and documentation references.
Keep the existing `IOsonata_RA4M1` project and its library dependencies.
Preserve the working GPIO, UART, timer, startup and crypto configuration.
RA4M1 IIC master/slave remains unvalidated on physical hardware.

Verify IOC and e2 studio Debug/Release library projects and all RA4M1
examples before removing the old path. Do not copy common drivers or
leave two competing target source trees.
