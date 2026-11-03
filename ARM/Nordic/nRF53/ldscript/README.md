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
| `gcc_nrf5340_xxaa_application_dfu.ld` | Slot 0 0x0000D000-0x000FCFFF; NVM0 0x000FD000-0x000FFFFF; IPC0 reserved below recovery flag | DFU application variant |
| `gcc_nrf5340_xxaa_network_dfu_boot.ld` | Boot flash 0x01000000-0x01008FFF | DFU boot variant |
| `gcc_nrf5340_xxaa_network_dfu.ld` | Slot 0 flash 0x0100A000-0x010247FF | DFU application variant |

The `*_dfu_layout.ld` files are shared dependencies of their corresponding
boot and application scripts. The network DFU layout also declares slot 1
(0x01024800-0x0103FFFF). Preserve both files and both MCU-core configurations.

## Application DFU NVM and IPC reservations

The application DFU layout now ends slot 0 at 0x000FD000 and reserves
0x000FD000-0x000FFFFF as NVM0. This matches the normal Application core
flash/NVM partition and prevents the DFU application from occupying the
persistent storage pages. Boot and application DFU scripts share the layout
symbols, so the slot-0 boundary is updated for both.

The Application core DFU script now keeps RAM1 to
0x20040000-0x2006FFFF and declares a separate IPC0 region starting at
0x20070000. IPC0 ends at 0x2007EFEF, immediately before the recovery
flag at 0x2007EFF0. The normal layout gives IPC0 the full upper 64 KB,
while the DFU variant reserves the recovery flag and the adjacent
0x2007F000 SystemInit/FICR copy area. The DFU region exports
`__start_ipc0` and `__stop_ipc0`, like the normal layout.

The IpcIntrfTest application and network projects each configure an IPC
block at 0x20070000 of size 0x2000 (8 KB). The DFU IPC0 region is
0xEFF0 bytes (61,424 bytes), so those example buffers fit within it.
The HCI controller's actual ring geometry and shared memory assumptions
have not yet been verified against the DFU region.

This is a source-level layout correction, **not** a completed DFU system
validation. Confirm the reduced IPC block is large enough for configured
HCI ring buffers and any direct/shared memory consumers. Run real
Application/Network core DFU builds, flash-layout checks, and a staged
update/recovery test before release.

## Cleanup policy

Keep all valid MCU, core, DFU, bootloader, storage, and shared-memory
variants even if no managed example currently uses them. Obsolete status
requires verification of a superseding layout and all consumers. The
reference audit alone is insufficient to justify deletion.
