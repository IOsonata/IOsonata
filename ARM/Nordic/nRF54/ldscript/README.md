# nRF54 linker-script variants

The family linker directory contains target-specific memory layouts. A script
without a current example-project reference remains an available variant, not
an obsolete script.

## Reviewed nRF54L bare-metal layouts

The RRAM capacity, not the rounded address-space boundary, defines the
last valid byte. Nordic specifies 1524 KB for nRF54L15, 2036 KB for
nRF54LM20A, 500 KB for nRF54L05, 1012 KB for nRF54L10 and nRF54LV10A,
and 508 KB for nRF54LS05B. NVM0 is now reserved *inside* physical RRAM.

| Script | Application FLASH | NVM0 | Physical RRAM end |
| --- | --- | --- | --- |
| `gcc_nrf54l15_xxaa.ld` | 0x000000–0x179FFF | 0x17A000–0x17CFFF (12 KB) | 0x17D000 |
| `gcc_nrf54lm20a_xxaa.ld` | 0x000000–0x1F9FFF | 0x1FA000–0x1FCFFF (12 KB) | 0x1FD000 |
| `gcc_nrf54lm20b_xxaa.ld` | 0x000000–0x1F9FFF | 0x1FA000–0x1FCFFF (12 KB) | 0x1FD000 |
| `gcc_nrf54l05_xxaa.ld` | 0x000000–0x079FFF | 0x07A000–0x07CFFF (12 KB) | 0x07D000 |
| `gcc_nrf54l10_xxaa.ld` | 0x000000–0x0F9FFF | 0x0FA000–0x0FCFFF (12 KB) | 0x0FD000 |
| `gcc_nrf54lm20a_enga_xxaa.ld` | 0x000000–0x1F9FFF | 0x1FA000–0x1FCFFF (12 KB) | 0x1FD000 |
| `gcc_nrf54ls05b_enga_xxaa.ld` | 0x000000–0x07DFFF | 0x07E000–0x07EFFF (4 KB) | 0x07F000 |
| `gcc_nrf54lv10a_enga_xxaa.ld` | 0x000000–0x0F9FFF | 0x0FA000–0x0FCFFF (12 KB) | 0x0FD000 |

All nRF54 IOsonata GCC linker variants use `gcc_arm_flash.ld`, not
Nordic's `nrf_common.ld`. The bare-metal and engineering variants
export `__start_nvm0` and `__stop_nvm0` from an `nvm0 (NOLOAD)`
section. FLASH lengths were reduced to reserve real RRAM for NVM0;
RAM regions were not changed. Rebuild affected configurations and
verify the linker map. This change moves persistent storage; preserve
or migrate existing on-device contents before firmware updates.

Nordic documents 2036 KB of physical RRAM for both nRF54LM20A and
nRF54LM20B (FICR INFO.RRAM K2036). The LM20B bare-metal script now
uses this capacity and reserves 12 KB of NVM0 at the physical end.
Existing LM20B bare-metal images had no linker NVM0 reservation; this
changes the available linker FLASH range and requires rebuild validation.

The LM20B S145 and DFU scripts now use the same memory partitions as LM20A:
NVM0 at 0x1D8800, NVM1 at 0x1D9800, and S145 at 0x1DA800.
The layouts follow the common nRF54LM20 S145 image. Hardware validation
and storage migration remain necessary for previously provisioned LM20B devices.

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
| nRF54LM20B | `0x0D000–0xF1FFF` | `0xF2000–0x1D87FF` | `0x1D8800` | `0x2007FCB0` |

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
storage decision. Both non-DFU and DFU IOsonata scripts include `gcc_arm_flash.ld`.
Target linking and review of the startup implementation remain required.

The nRF54H20 library project configurations still describe port integration
as pending. DFU layout boundary tests are static checks only; compilation,
multi-core RAM permissions, DFU image handling, and boot behavior remain
unverified.



## Validation

Static regression tests cover the new bare-metal NVM boundaries. Compiler
linking, linker map sizes and target hardware behavior require separate
validation. Unselected scripts should not be deleted without proving that
their memory layout is obsolete and all consumers have been accounted for.
