/* DFU RRAM layout, nRF54L15 with S145.
 *
 * Included by the stage 0 boot script (dfu_boot_nrf54l15_s145.ld) and the
 * application script (gcc_nrf54l15_xxaa_s145_dfu.ld), so both agree on where
 * everything is. Code reads these symbols through DfuLayoutGet; no C define
 * repeats an address. See docs/architecture/dfu.md.
 *
 *   0x000000 - 0x00BFFF  stage 0 boot            48 KB, runs first
 *   0x00C000 - 0x00CFFF  record of slot 0         4 KB
 *   0x00D000 - 0x0B1FFF  slot 0, application    660 KB
 *   0x0B2000 - 0x1587FF  slot 1, upload         666 KB, trailer at the end
 *   0x158800 - 0x1597FF  NVM0, peer manager       4 KB
 *   0x159800 - 0x15A7FF  NVM1, application        4 KB
 *   0x15A800 - 0x17CBFF  S145
 *
 * The boot sets VTOR to slot 0, which is 4 KB aligned, more than the vector
 * table needs. The NVM regions and the SoftDevice are where they are without
 * DFU, so a store written before DFU was added is still found.
 *
 * Slot 1 is larger than slot 0, for the image header, the TLV areas and the
 * trailer that come with a slot 0 sized payload.
 *
 * RRAM has no erase; the target layer writes ones in 2 KB units, so every
 * region here starts and ends on a 2 KB boundary.
 *
 * __dfu_flag is the recovery request word, the last 16 bytes of RAM: the
 * boot and the application both stop their RAM short of it, so neither
 * links anything there nor clears it at startup.
 */

__dfu_flag			= 0x2003FFF0;
__dfu_boot_start	= 0x00000000;
__dfu_boot_end		= 0x0000C000;
__dfu_rec_start		= 0x0000C000;
__dfu_rec_end		= 0x0000D000;
__dfu_slot0_start	= 0x0000D000;
__dfu_slot0_end		= 0x000B2000;
__dfu_slot1_start	= 0x000B2000;
__dfu_slot1_end		= 0x00158800;
__dfu_nvm0_start	= 0x00158800;
__dfu_nvm1_start	= 0x00159800;
__dfu_nvm_size		= 0x00001000;
