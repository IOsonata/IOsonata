/* DFU flash layout, LPC54605J512 (512 KB flash in 32 KB sectors; SRAMX
 * 32 KB, SRAM0 64 KB, SRAM1 to SRAM3 96 KB).
 *
 * Included by the stage 0 boot script (dfu_boot_lpc54605.ld) and the
 * application script (gcc_lpc54605_dfu.ld), so both agree on where
 * everything is. Code reads these symbols through DfuLayoutGet; no C define
 * repeats an address. See docs/architecture/dfu.md.
 *
 *   0x00000 - 0x0FFFF  stage 0 boot           64 KB, the part starts here
 *   0x10000 - 0x17FFF  record of slot 0       32 KB, one sector
 *   0x18000 - 0x7FFFF  slot 0, application   416 KB
 *
 * The erase unit is the 32 KB sector, so the boot takes 48 KB rounded up to
 * two sectors and the record a whole sector.
 *
 * Wired update only: no slot 1. The boot writes an image received over the
 * UART straight into slot 0 and the record.
 *
 * RAM:
 *
 *   0x20000000 - 0x2000FFDF  SRAM0, boot and application
 *   0x2000FFE0 - 0x2000FFFF  kept free for the IAP ROM
 *   0x20010000 - 0x20027FCF  SRAM1 to SRAM3, application, placed explicitly
 *   0x20027FD0 - 0x20027FDF  __dfu_flag, recovery request word
 *   0x20027FE0 - 0x20027FFF  kept free for the IAP ROM
 *
 * SRAM1 to SRAM3 are clocked only once SystemInit has run, after ResetEntry
 * has set up .data and .bss, so both scripts keep those in SRAM0. The flag
 * is read and written from main on, when the clocks are on. Neither script
 * links anything at the flag nor clears it at startup.
 */

__dfu_flag			= 0x20027FD0;

/* State unit, DfuStateUnit(): the program unit, for the tools that build
 * the record and trailer off target (Python/dfu_prod_hex.py). */
__dfu_state_unit	= 256;
__dfu_ram_start		= 0x20000000;
__dfu_ram_end		= 0x2000FFE0;
__dfu_boot_start	= 0x00000000;
__dfu_boot_end		= 0x00010000;
__dfu_rec_start		= 0x00010000;
__dfu_rec_end		= 0x00018000;
__dfu_slot0_start	= 0x00018000;
__dfu_slot0_end		= 0x00080000;
