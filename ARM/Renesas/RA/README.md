# Renesas RA Cortex-M family

The RA MCU family belongs under `ARM/Renesas/RA`. The currently implemented
RA4M1 port remains in `ARM/Renesas/RA4M1` until its Eclipse project links,
relative includes, linker references and regression tests can be relocated
atomically and validated. **Do not copy its sources into another RA target.**

## Organization

Target layout for an incremental, source-compatible migration:

```
ARM/Renesas/RA/
    README.md
    include/          # RA shared declarations, only when IP-compatible
    src/              # RA shared implementations, only when IP-compatible
    RA0/              # target groups as supported
    RA2/
    RA4/
        RA4M1/
            include/
            src/
            ldscript/
            lib/ioc/
            lib/e2/
            exemples/
    RA6/
    RA8/
```

Directories for unsupported families are intentionally not created. A
family/series directory alone is not evidence of a working MCU port.

## Source ownership

- `RA/include` and `RA/src`: *proven* common RA peripheral implementations,
  with explicit hardware capability predicates; do not assume identical SCI,
  ICU, GPT, IIC, DMAC, clock, flash, or security IP across RA series.
- `RA/<series>/<group>/include`: device CMSIS/register headers, interrupt
  event definitions, memory and feature descriptions.
- `RA/<series>/<group>/src`: startup, clock configuration, vector tables,
  flash layout, and drivers only where the peripheral hardware differs.
- `RA/<series>/<group>/ldscript`: target-specific flash and SRAM layouts.
- `lib/ioc` and `lib/e2`: IOsonata library projects; link shared and
  target-specific source, never copy common code.
- `exemples`: target example projects; `board.h` is for board pin mapping,
  not MCU register initialization.

## Porting rules

1. Keep public peripheral interfaces under the top-level IOsonata `include`.
   Hardware crypto, timers, UART, SPI and other backends are selected by the
   MCU library, never by an application.
2. Use the device header and peripheral IP revision, not the marketing series
   name, to decide whether a driver can be shared.
3. Do not include Renesas FSP or duplicate the generic interfaces.
4. Preserve RA4M1's existing working startup, GPIO, timer, UART and CryptoTest
   configurations. IIC master/slave remain unvalidated on hardware.
5. During the RA4M1 directory move, rewrite and verify every `.project`,
   `.cproject`, linker-script reference, documentation path and other
   repository reference. Test Debug and Release builds of the library and
   examples before removing `ARM/Renesas/RA4M1`.
6. Preserve the existing project name `IOsonata_RA4M1` and its downstream
   library dependencies until a coordinated project-name change is necessary.
7. The RISC-V R9A02 port and ARM RE01 port remain separate MCU families.

## Initial migration target

First move RA4M1 to `RA/RA4/RA4M1` with mechanically updated resource
links and unchanged code. After build validation, extract *verified* common
peripheral implementations into `RA/src` and introduce further device
groups only with real header, startup, linker and project support.
