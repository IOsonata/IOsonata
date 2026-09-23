/* DFU flash layout, LPC1769 (512 KB flash: 16 sectors of 4 KB, then 14 of
 * 32 KB; 32 KB local SRAM, 2 x 16 KB AHB SRAM).
 *
 * Included by the stage 0 boot script (dfu_boot_lpc1769.ld) and the
 * application script (gcc_lpc1767_9_dfu.ld), so both agree on where
 * everything is. Code reads these symbols through DfuLayoutGet; no C define
 * repeats an address. See docs/architecture/dfu.md.
 *
 *   0x00000 - 0x0BFFF  stage 0 boot           48 KB, the part starts here
 *   0x0C000 - 0x0CFFF  record of slot 0        4 KB, a 4 KB sector
 *   0x0D000 - 0x7FFFF  slot 0, application   460 KB
 *
 * Wired update only: no slot 1. The boot writes an image received over the
 * UART straight into slot 0 and the record. Slot 0 starts in the 4 KB
 * sectors, on the 256 byte boundary its vector table needs for VTOR.
 *
 * RAM:
 *
 *   0x10000000 - 0x10007FDF  local SRAM, boot and application
 *   0x10007FE0 - 0x10007FFF  IAP ROM work area, kept free
 *   0x2007C000 - 0x20083FEF  AHB SRAM, application
 *   0x20083FF0 - 0x20083FFF  __dfu_flag, recovery request word
 *
 * The flag is at the end of the AHB SRAM, which the boot ROM leaves alone;
 * its own stack sits at the top of the local SRAM. Neither script links
 * anything at the flag nor clears it at startup.
 */

__dfu_flag			= 0x20083FF0;

/* State unit, DfuStateUnit(): the program unit, for the tools that build
 * the record and trailer off target (Python/dfu_prod_hex.py). */
__dfu_state_unit	= 256;
__dfu_ram_start		= 0x10000000;
__dfu_ram_end		= 0x10007FE0;
__dfu_boot_start	= 0x00000000;
__dfu_boot_end		= 0x0000C000;
__dfu_rec_start		= 0x0000C000;
__dfu_rec_end		= 0x0000D000;
__dfu_slot0_start	= 0x0000D000;
__dfu_slot0_end		= 0x00080000;
