/* DFU flash layout, SAM4LC8 and SAM4LS8 (512 KB flash, 64 KB RAM).
 *
 * Included by the stage 0 boot script (dfu_boot_sam4lx8.ld) and the
 * application script (gcc_sam4lx8_dfu.ld), so both agree on where
 * everything is. Code reads these symbols through DfuLayoutGet; no C define
 * repeats an address. See docs/architecture/dfu.md.
 *
 *   0x00000 - 0x0BFFF  stage 0 boot          48 KB, reset vector
 *   0x0C000 - 0x0CFFF  record of slot 0       4 KB
 *   0x0D000 - 0x7FFFF  slot 0, application  460 KB
 *
 * Erase and program unit 512 bytes, one page through the page buffer. The
 * record takes 8 pages: a state unit, then up to 3 KB of header and TLVs.
 *
 * No radio on the part, so no slot 1: an update is written straight into
 * slot 0 by the boot, over its UART, in recovery.
 *
 * __dfu_flag is the recovery request word, the last 16 bytes of HRAMC0: the
 * boot and the application both stop their RAM short of it, so neither
 * links anything there nor clears it at startup.
 */

__dfu_flag			= 0x2000FFF0;

/* State unit, DfuStateUnit(): the program unit, for the tools that build
 * the record and trailer off target (Python/dfu_prod_hex.py). */
__dfu_state_unit	= 512;
__dfu_boot_start	= 0x00000000;
__dfu_boot_end		= 0x0000C000;
__dfu_rec_start		= 0x0000C000;
__dfu_rec_end		= 0x0000D000;
__dfu_slot0_start	= 0x0000D000;
__dfu_slot0_end		= 0x00080000;
