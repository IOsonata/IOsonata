# nRF52 linker-script variants

Scripts in this directory are grouped by MCU family. A script is not obsolete
merely because no managed project currently selects it. Different memory maps
are supported variants and must remain selectable by future projects.

## IOsonata versus Nordic SDK linker scripts

All IOsonata GNU linker scripts in this family directory use
`gcc_arm_flash.ld`, not the Nordic SDK `nrf_common.ld`.

The standalone `nRF52832/exemples/UartPrbsTxnrfx` comparison uses Nordic's
`gcc_startup_nrf52.S` and `system_nrf52.c`, so its Nordic-specific
`nrf52_xxaa_nrfx_prbs.ld` stays local to the reference example's `src`
directory and retains `nrf_common.ld`. Debug and Release configurations
reference that local file; it is **not** an IOsonata linker script.
Keep this distinction for other upstream SDK comparison examples.

## nRF52832/nRF52840 application storage

The standard layout reserves **two separate linker NVM regions**:

- `NVM0`: 0x2000 bytes (8 KB), dedicated to Bluetooth PDS. This is
  two independently erasable nRF52 flash pages, required by PDS for
  transactional garbage collection.
- `NVM1`: 0x1000 bytes (4 KB), reserved for user application data.
  It is not part of PDS garbage collection.

With **no DFU**, there is no MBR reservation and NVM occupies the last
12 KB of the device flash:

| Device | Application FLASH end (exclusive) | NVM0 | NVM1 |
| --- | --- | --- | --- |
| nRF52832 | 0x7D000 | 0x7D000..0x7EFFF | 0x7F000..0x7FFFF |
| nRF52840 | 0xFD000 | 0xFD000..0xFEFFF | 0xFF000..0xFFFFF |

With **MBR + Nordic DFU**, the same NVM regions move directly below the
bootloader without overlapping the two bootloader metadata pages:

| Device | App origin | App FLASH end (exclusive) | NVM0 | NVM1 | DFU bootloader | MBR parameters | DFU settings |
| --- | --- | --- | --- | --- | --- | --- | --- |
| nRF52832 | 0x1000 | 0x74000 | 0x74000..0x75FFF | 0x76000..0x76FFF | 0x77000..0x7DFFF | 0x7E000..0x7EFFF | 0x7F000..0x7FFFF |
| nRF52840 | 0x1000 | 0xDD000 | 0xDD000..0xDEFFF | 0xDF000..0xDFFFF | 0xE0000..0xFDFFF | 0xFE000..0xFEFFF | 0xFF000..0xFFFFF |

MBR and DFU are coupled in these layouts. SDC does not need a distinct
linker map. IOsonata NVM replaces Nordic FDS; no separate FDS flash
allocation is made.

The linker exports `__start_nvm0`, `__stop_nvm0`,
`__start_nvm1`, and `__stop_nvm1`; `NvmRegionAddr()` and
`NvmRegionSize()` expose both regions. The `NVM0` region index remains
0, preserving the default Bluetooth bond store selection.

These boundaries match the documented Nordic DFU bootloader variants.
They require validation against the actual programmed bootloader and
generated ELF/HEX before flashing an MBR+DFU image. Existing NVM data
will not automatically migrate when a storage region moves.

## Reviewed variants

The following scripts are **distinct layouts** based on their current MEMORY
definitions. Reference status does not determine whether to retain them.

| Script | Purpose / observed memory difference | Classification |
| --- | --- | --- |
| `gcc_nrf52805_xxaa_s112.ld` | S112 application at 0x19000, 4 KB NVM0 at 0x2F000, RAM from 0x20003000 | Available variant; nRF52805 target |
| `gcc_nrf52810_xxaa_s112.ld` | Same currently defined S112 memory regions as nRF52805, but separate MCU target | Available variant; nRF52810 target |
| `gcc_nrf52832_xxaa.ld` | Bare-metal flash starts at 0x00000000; RAM at 0x20000000 | Available variant |
| `gcc_nrf52832_xxab.ld` | 256 KB flash, 32 KB RAM; different package memory capacity from XXAA | Available variant |
| `gcc_nrf52811_xxaa.ld` | 192 KB flash, 24 KB RAM; 4 KB NVM0 | Available variant |
| `gcc_nrf52820_xxaa.ld` | 256 KB flash, 32 KB RAM; 4 KB NVM0 | Available variant |
| `gcc_nrf52832_xxaa_s132.ld` | S132 application flash starts at 0x00026000; RAM at 0x20004000 | Available variant |
| `gcc_nrf52832_xxaa_s132_central.ld` | Same flash start as S132, but RAM starts at 0x20004300 | Available variant; distinct RAM reservation |
| `gcc_nrf52832_xxaa_s212.ld` | Flash starts at 0x00012000; RAM at 0x20000A00 | Available variant; different SoftDevice |
| `gcc_nrf52832_xxaa_s132_dfu.ld` | DFU slot defined through `gcc_nrf52832_xxaa_s132_dfu_layout.ld` | Available variant; preserve layout dependency |
| `gcc_nrf52832_xxaa_s132_sdk_debug_dfu_boot.ld` | Bootloader image starts at 0x00075A00; reserves UICR and settings regions | Available variant; distinct bootloader layout |
| `gcc_nrf52840_xxaa_bossa.ld` | Application flash starts at 0x00010000 after an Arduino/BOSSA bootloader | Available variant; distinct bootloader offset |
| `gcc_nrf52840_xxaa_s140.ld` | S140 flash starts at 0x00027000; RAM at 0x20003DC0; has NVM0 | Available variant |
| `gcc_nrf52840_xxaa_s140c.ld` | Flash 0x27000-0xFCFFF, NVM0 0xFD000-0xFEFFF, settings 0xFF000-0xFFFFF; RAM at 0x20005400 | Available variant; distinct RAM reservation |

The S140C script previously let application FLASH extend through the settings page.
Its flash boundary and NVM0 reservation were corrected to match the S140
flash/storage partition, without changing its distinct RAM allocation.
A static regression test checks these boundaries; linker and device validation
are still required.

These classifications establish that the scripts should **not** be removed
because of an absent `.cproject` or `.ewp` reference. They do not assert that
every layout has been hardware validated.

## Same layout, different MCU targets

The nRF52805 and nRF52810 S112 scripts currently have identical MEMORY
regions and section placement. They are **not obsolete duplicates**: the names
identify different supported MCU targets. Consolidation would require an
explicit shared layout and verified project references for both devices.

The BOSSA nRF52840 variant intentionally reserves 12 KB at the top of flash
as NVM0, unlike the S140 layouts which reserve 8 KB NVM0 plus a separate
4 KB bootloader-settings page. Keep the bootloader-specific layout distinct.

## GNU and IAR memory-map compatibility

The `gcc_nrf52832_xxaa.ld` script reserves 0x0007D000-0x0007EFFF
for `NVM0`, with application FLASH ending at 0x0007CFFF. Both
`iar_nrf52832_xxaa.icf` and `iar_nrf52832_xxaa_irom.icf` now
limit application ROM to 0x0007CFFF and export `__start_nvm0` =
0x0007D000 and `__stop_nvm0` = 0x0007F000. The final 4 KB
(0x0007F000-0x0007FFFF) remains outside the application ROM.

These are static linker-layout corrections. IAR build, symbol
compatibility with the IAR compiler, and hardware behavior still
require validation; different IAR section-placement models remain
intentionally separate.

The `gcc_nrf52832_xxab.ld`, `gcc_nrf52811_xxaa.ld`, and
`gcc_nrf52820_xxaa.ld` scripts represent different device capacities.
They must not be collapsed into XXAA or another part's map on the basis
of current project references.

## Criteria for removal

Only remove a script when all of these are established:

1. The script is truly superseded or invalid, not just unselected.
2. Its `MEMORY` regions, `SECTIONS`, symbols, `INCLUDE` dependencies, and
   bootloader/SoftDevice requirements have been compared with its replacement.
3. No supported target variant depends on the old layout, including external
   build recipes and future selectable configurations.
4. Every existing project or build recipe referencing it has been updated.
5. The relevant build and memory layout checks pass.

An identical filename suffix or similar flash origin does not establish a
duplicate. Do not delete DFU layouts, other bootloader offsets, or unsupported
project variants based on the reference audit alone.
