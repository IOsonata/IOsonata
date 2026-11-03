# nRF54 linker-script variants

The family linker directory contains target-specific memory layouts. A script
without a current example-project reference remains an available variant, not
an obsolete script.

## Reviewed nRF54L bare-metal layouts

| Script | Application FLASH | NVM0 | RAM | Classification |
| --- | --- | --- | --- | --- |
| `gcc_nrf54l15_xxaa.ld` | 0x000000-0x17CFFF | 0x17D000-0x17FFFF (12 KB) | RAM0 and RAM1 retained | Bare-metal variant |
| `gcc_nrf54lm20a_xxaa.ld` | 0x000000-0x1FCFFF | 0x1FD000-0x1FFFFF (12 KB) | RAM0 and RAM1 retained | Bare-metal variant |

Additional preserved bare-metal and engineering variants now expose the
flash area already excluded by their existing FLASH lengths:

| Script | NVM0 start | Size |
| --- | --- | --- |
| `gcc_nrf54l05_xxaa.ld` | `0x7D000` | 12 KB |
| `gcc_nrf54l10_xxaa.ld` | `0xFD000` | 12 KB |
| `gcc_nrf54lm20a_enga_xxaa.ld` | `0x1FD000` | 12 KB |
| `gcc_nrf54ls05b_enga_xxaa.ld` | `0x7F000` | 4 KB |
| `gcc_nrf54lv10a_enga_xxaa.ld` | `0xFD000` | 12 KB |

These older variants retain their existing `INCLUDE "nrf_common.ld"`
dependency. Unlike the standardized GCC scripts, this shared file may be
supplied by an external SDK. Confirm that it is present at link time and
review compatibility with `gcc_arm_flash.ld` before changing the include.
The reference audit does not check these indirect SDK dependencies.

The two bare-metal scripts already ended FLASH before the final 12 KB of
RRAM. They now declare the omitted NVM0 region and export
`__start_nvm0`/`__stop_nvm0` in an `nvm0 (NOLOAD)` section. Existing FLASH
and RAM boundaries were not moved.

The S145 scripts are **different layouts**. For example,
`gcc_nrf54l15_xxaa_s145.ld` reserves two distinct 4 KB NVM regions,
one for the peer manager and one for the application, and starts RAM beyond
the SoftDevice reservation. Their DFU scripts additionally use slot layouts
and recovery flags. Preserve these variants; do not apply bare-metal NVM
boundaries to S145 or DFU configurations without reviewing those requirements.

## S145 DFU layouts reviewed

The nRF54L15, nRF54LM20A and nRF54LM20B DFU layouts use contiguous
stage-0 boot, recovery-record, application-slot and upload-slot boundaries.
The upload slot ends where NVM0 begins; NVM0 and NVM1 remain separate
4 KB partitions at their non-DFU S145 addresses.

| Target | Slot 0 | Slot 1 | NVM0 | Recovery flag |
| --- | --- | --- | --- | --- |
| nRF54L15 | `0x0D000–0xB1FFF` | `0xB2000–0x1587FF` | `0x158800` | `0x2003FFF0` |
| nRF54LM20A | `0x0D000–0xF1FFF` | `0xF2000–0x1D87FF` | `0x1D8800` | `0x2007FCB0` |
| nRF54LM20B | `0x0D000–0x71FFF` | `0x72000–0xD87FF` | `0x0D8800` | `0x2007FCB0` |

All three use a 48 KB stage-0 boot region and a 4 KB recovery-record
region. Application and boot scripts include the same target-specific DFU
layout. The nRF54L15 S145 linker scripts start application RAM at
`0x20004900`, not `0x20004780`; comments were corrected without
changing the existing memory map. The cross-variant regression checks
the boundaries and recovery words, but does not replace actual target
linking and DFU hardware validation.

## nRF54H20 Application and Radio core layouts

The H20 cores have separate primary MRAM and local RAM partitions, while the
linker scripts also expose shared MRAM-adjacent resources such as OTP, external
flash, and multiple global RAM banks. Those common entries are *physical shared
resources*, not separate allocations for each core. Multi-core firmware must
coordinate use and permissions; these scripts do not establish isolation.

| Core | Primary MRAM | Stage-0 boot | Recovery record | Slot 0 | Slot 1 | Local recovery word |
| --- | --- | --- | --- | --- | --- | --- |
| Application | `0x0E0A0000–0x0E0DFFFF` (256 KB) | 40 KB | 4 KB | 212 KB | none | `0x22007FF0` |
| Radio | `0x0E120000–0x0E19FFFF` (512 KB) | 48 KB | 4 KB | 228 KB | 232 KB | `0x2300FFF0` |

Application DFU is a single-slot wired-update layout; Radio DFU uses two
slots. In both cases the bootloader and application include the same
target-specific layout, and reserve the last 16 bytes of local RAM outside
their linker-allocated RAM region.

Unlike the nRF54L bare-metal scripts, the current nRF54H20 scripts allocate
their entire designated primary MRAM partitions to executable FLASH or DFU
slots. There is **no unused upper MRAM tail** to expose as NVM0 without
reducing a firmware slot or allocating a separately defined storage area.
Do not invent an NVM0 partition for these cores without a target-specific
storage decision. The non-DFU scripts retain their `nrf_common.ld` include;
DFU scripts use `gcc_arm_flash.ld`. Compatibility needs actual target
linking and review of the startup implementation.

The nRF54H20 library project configurations still describe port integration
as pending. DFU layout boundary tests are static checks only; compilation,
multi-core RAM permissions, DFU image handling, and boot behavior remain
unverified.



## Validation

Static regression tests cover the new bare-metal NVM boundaries. Compiler
linking, linker map sizes and target hardware behavior require separate
validation. Unselected scripts should not be deleted without proving that
their memory layout is obsolete and all consumers have been accounted for.
