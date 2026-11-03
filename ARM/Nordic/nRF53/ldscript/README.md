# nRF53 linker-script variants

Linker scripts are grouped by MCU family, with the core identified in the
filename. An unselected configuration remains available if it describes a
valid target and memory layout; it is not obsolete solely because no current
Eclipse project selects it.

## nRF5340 core layouts

| Script | Layout | Classification |
| --- | --- | --- |
| `gcc_nrf5340_xxaa_application.ld` | Application flash 0x00000000-0x000FCFFF, NVM0 at 0x000FD000-0x000FFFFF; application RAM and shared RAM/IPC regions | Normal application variant |
| `gcc_nrf5340_xxaa_network.ld` | Network flash 0x01000000-0x0103FFFF and network RAM at 0x21000000 | Normal network variant |
| `gcc_nrf5340_xxaa_application_dfu_boot.ld` | Boot flash 0x00000000-0x0000BFFF | DFU boot variant |
| `gcc_nrf5340_xxaa_application_dfu.ld` | Slot 0 flash 0x0000D000-0x000FFFFF | DFU application variant |
| `gcc_nrf5340_xxaa_network_dfu_boot.ld` | Boot flash 0x01000000-0x01008FFF | DFU boot variant |
| `gcc_nrf5340_xxaa_network_dfu.ld` | Slot 0 flash 0x0100A000-0x010247FF | DFU application variant |

The `*_dfu_layout.ld` files are shared dependencies of their corresponding
boot and application scripts. The network DFU layout also declares slot 1
(0x01024800-0x0103FFFF). Preserve both files and both MCU-core configurations.

## DFU/NVM0 follow-up

The normal Application core linker script reserves the top 12 KB of internal
flash at 0x000FD000-0x000FFFFF for NVM0. In contrast,
`gcc_nrf5340_xxaa_application_dfu_layout.ld` sets
`__dfu_slot0_end = 0x00100000`, and the DFU application script gives that
entire range to `FLASH` with no NVM0 region.

These are **different storage layouts**, not interchangeable variants.
The DFU slot overlaps the location reserved as NVM0 by the normal layout.
Before changing this boundary, review the DFU image size, slot protocol,
bootloader/application storage compatibility, and upgrade behavior. A future
decision to reserve NVM0 in DFU mode must update the shared DFU layout,
bootloader and application expectations together. Do not shrink or move
slots based only on a file-name convention.

The normal Application core has a separate IPC0 region at
0x20070000-0x2007FFFF, whereas the DFU application script uses a continuous
RAM1 region up to the recovery flag at 0x2007EFF0. Review IPC/shared-RAM
ownership for DFU firmware before treating those as equivalent RAM layouts.

## Cleanup policy

Keep all valid MCU, core, DFU, bootloader, storage, and shared-memory
variants even if no managed example currently uses them. Obsolete status
requires verification of a superseding layout and all consumers. The
reference audit alone is insufficient to justify deletion.
