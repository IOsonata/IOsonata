/* DFU flash layout, SAM4E16E (1 MB flash, 128 KB RAM).
 *
 * Included by the stage 0 boot script (dfu_boot_sam4e16.ld) and the
 * application script (gcc_sam4e16_dfu.ld), so both agree on where
 * everything is. Code reads these symbols through DfuLayoutGet; no C define
 * repeats an address. See docs/architecture/dfu.md.
 *
 *   0x400000 - 0x40BFFF  stage 0 boot            48 KB, reset vector
 *   0x40C000 - 0x40DFFF  record of slot 0         8 KB
 *   0x40E000 - 0x4FFFFF  slot 0, application    968 KB
 *
 * Erase unit 8 KB: the EEFC erases 16 pages of 512 bytes at a time, one
 * lock region. Program unit 512 bytes, one page through the latch buffer.
 *
 * No radio on the part, so no slot 1: an update is written straight into
 * slot 0 by the boot, over its UART, in recovery.
 *
 * The chip starts from the flash at 0x400000 with GPNVM bit 1 set (boot from
 * flash, not the ROM); the boot is linked there.
 *
 * __dfu_flag is the recovery request word, the last 16 bytes of RAM: the
 * boot and the application both stop their RAM short of it, so neither
 * links anything there nor clears it at startup.
 */

__dfu_flag			= 0x2001FFF0;

/* State unit, DfuStateUnit(): the program unit, for the tools that build
 * the record and trailer off target (Python/dfu_prod_hex.py). */
__dfu_state_unit	= 512;
__dfu_boot_start	= 0x00400000;
__dfu_boot_end		= 0x0040C000;
__dfu_rec_start		= 0x0040C000;
__dfu_rec_end		= 0x0040E000;
__dfu_slot0_start	= 0x0040E000;
__dfu_slot0_end		= 0x00500000;
