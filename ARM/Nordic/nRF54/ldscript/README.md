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

The nRF54H20 Application and Radio core scripts describe separate cores and
MRAM/RAM allocations. They are not duplicates of nRF54L layouts.

## Validation

Static regression tests cover the new bare-metal NVM boundaries. Compiler
linking, linker map sizes and target hardware behavior require separate
validation. Unselected scripts should not be deleted without proving that
their memory layout is obsolete and all consumers have been accounted for.
