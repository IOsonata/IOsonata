# SAM4E16E linker layouts

The SAM4E16E has a 1 MB application flash address range
`0x00400000–0x004FFFFF` and 128 KB SRAM at
`0x20000000–0x2001FFFF`. The normal IOsonata linker
`gcc_sam4e16.ld` uses these complete regions and includes
`gcc_arm_flash.ld`.

## Available layouts

| Script | Purpose |
| --- | --- |
| `gcc_sam4e16.ld` | Normal application, full flash and SRAM |
| `gcc_sam4e16_dfu.ld` | Application linked at `0x0040E000` after stage-0 boot and recovery record |
| `gcc_sam4e16_dfu_boot.ld` | 48 KB stage-0 boot image starting at `0x00400000` |
| `gcc_sam4e16_dfu_layout.ld` | Shared DFU boundaries and recovery word for boot/application |

The DFU layout partitions flash into stage-0 boot
(`0x00400000–0x0040BFFF`), recovery record
(`0x0040C000–0x0040DFFF`), and application slot
(`0x0040E000–0x004FFFFF`). The recovery request word
(`0x2001FFF0`) is outside the RAM range used by DFU boot/application.
The state unit is 512 bytes; flash region boundaries are 8 KB aligned.

There is no unallocated upper flash tail in these layouts. Introducing a
default NVM partition requires an explicit storage allocation decision;
it must not silently overlap the DFU application slot.

## Managed-project selections

Blinky, CryptoTest, UartPrbsTxTest and UartRetargetDemo select the
normal linker for Debug and Release. DfuBoot selects the DFU boot linker.
The DFU application linker remains a valid selectable variant, even if
no current Eclipse example directly selects it. Do not delete it based
on reference count.

## Validation

The regression tests check the flash/SRAM bounds, DFU slot continuity,
recovery flag, and example project selections.
