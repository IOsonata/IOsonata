/* DFU flash layout, Renesas R9A02G021.
 *
 * Included by the stage 0 boot script (dfu_boot_r9a02g021.ld) and the
 * application script (gcc_R9A02G021_dfu.ld), so both agree on where
 * everything is. Code reads these symbols through DfuLayoutGet; no C define
 * repeats an address. See docs/architecture/dfu.md.
 *
 *   0x000000 - 0x008FFF  stage 0 boot           36 KB, reset entry at 0
 *   0x009000 - 0x009FFF  record of slot 0        4 KB
 *   0x00A000 - 0x01FFFF  slot 0, application    88 KB, its reset entry
 *                                                     first
 *
 * Flash: laid out on 128 KB, the code flash Renesas lists for the part.
 * r9a02g021.h and gcc_R9A02G021.ld allow up to 256 KB; on such a part the
 * space above 128 KB is simply not used.
 *
 * RAM: the flag is at the end of 32 KB, the SRAM r9a02g021.h
 * (R9A02_SRAM_SIZE_MAX) and gcc_R9A02G021.ld give. A part with 16 KB of
 * SRAM needs it moved to 0x20003FF0, and gcc_riscv_flash.ld then has to
 * reserve less than two thirds of RAM for the heap for the boot to link.
 *
 * The part is small, so the boot takes the smallest number of 2 KB blocks
 * it fits in: 35.8 KB at -Os.
 *
 * No radio on this part: updates come over the wire into slot 0 directly,
 * so there is no slot 1.
 *
 * Regions start on the 2 KB erase block. The record is two blocks: one
 * state unit (16) for its magic, plus the record body, about 3.1 KB.
 *
 * __dfu_flag is the recovery request word, the last 16 bytes of the SRAM:
 * the boot and the application both stop their RAM short of it, so neither
 * links anything there nor clears it at startup.
 */

__dfu_flag			= 0x20007FF0;
__dfu_boot_start	= 0x00000000;
__dfu_boot_end		= 0x00009000;
__dfu_rec_start		= 0x00009000;
__dfu_rec_end		= 0x0000A000;
__dfu_slot0_start	= 0x0000A000;
__dfu_slot0_end		= 0x00020000;
