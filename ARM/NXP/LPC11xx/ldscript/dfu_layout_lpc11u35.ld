/* DFU flash layout, LPC11U35 (64 KB flash, 4 KB sectors, 8 KB SRAM0).
 *
 * Included by the stage 0 boot script (dfu_boot_lpc11u35.ld) and the
 * application script (gcc_lpc11u35_dfu.ld), so both agree on where
 * everything is. Code reads these symbols through DfuLayoutGet; no C define
 * repeats an address. See docs/architecture/dfu.md.
 *
 *   0x00000 - 0x08FFF  stage 0 boot           36 KB, the part starts here
 *   0x09000 - 0x09FFF  record of slot 0        4 KB
 *   0x0A000 - 0x0FFFF  slot 0, application    24 KB
 *
 * The boot is the smallest sector multiple the DfuBoot Release build (-Os)
 * fits in: 34.1 KB of code and constants.
 *
 * Wired update only: no slot 1. The boot writes an image received over the
 * UART straight into slot 0 and the record.
 *
 * SRAM0:
 *
 *   0x10000000 - 0x100000BF  application vector table, copied there by the
 *                            boot, SYSMEMREMAP maps it at 0 (no VTOR)
 *   0x100000C0 - 0x100000CF  __dfu_flag, recovery request word
 *   0x100000D0 - 0x10001FDF  RAM of the boot and of the application
 *   0x10001FE0 - 0x10001FFF  IAP ROM work area, kept free
 *
 * The boot also puts buffers in the USB SRAM, 0x20004000, 2 KB, see
 * dfu_boot_lpc11u35.ld; the application has all of it.
 *
 * The flag sits low in SRAM0 rather than at its end: the boot ROM, which
 * runs at every reset, puts its stack just below the IAP work area
 * (UM10462, ISP and IAP memory use), so the top of the RAM is not sure to
 * keep a value across a reset. Neither script links anything at the flag
 * nor clears it at startup.
 */

__dfu_flag			= 0x100000C0;

/* State unit, DfuStateUnit(): the program unit, for the tools that build
 * the record and trailer off target (Python/dfu_prod_hex.py). */
__dfu_state_unit	= 256;
__dfu_ram_start		= 0x100000D0;
__dfu_ram_end		= 0x10001FE0;
__dfu_boot_start	= 0x00000000;
__dfu_boot_end		= 0x00009000;
__dfu_rec_start		= 0x00009000;
__dfu_rec_end		= 0x0000A000;
__dfu_slot0_start	= 0x0000A000;
__dfu_slot0_end		= 0x00010000;
