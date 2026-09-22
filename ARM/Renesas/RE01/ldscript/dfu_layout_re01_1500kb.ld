/* DFU flash layout, Renesas RE01 1500KB.
 *
 * Included by the stage 0 boot script (dfu_boot_re01_1500kb.ld) and the
 * application script (gcc_re01_1500kb_dfu.ld), so both agree on where
 * everything is. Code reads these symbols through DfuLayoutGet; no C define
 * repeats an address. See docs/architecture/dfu.md.
 *
 *   0x000000 - 0x00FFFF  stage 0 boot           64 KB, reset vectors, option
 *                                                     setting memory 0x400
 *   0x010000 - 0x017FFF  record of slot 0       32 KB
 *   0x018000 - 0x17FFFF  slot 0, application  1440 KB
 *
 * No radio on this part: updates come over the wire into slot 0 directly,
 * so there is no slot 1.
 *
 * Every region starts and ends on 32 KB, the erase unit dfu_re01 reports:
 * no code flash block of the FACI parts is larger, so erasing one region
 * never touches its neighbour whatever the block size of this part is.
 * Slot 0 also meets the VTOR alignment of the 100 entry vector table
 * (512 bytes).
 *
 * The boot is about 41 KB at -Os (Cortex-M0+ code, micro-ecc in C), more
 * than one 32 KB unit, so it takes two.
 *
 * The record needs one program unit (128) for its magic, plus the record
 * body, about 3.2 KB; the rest of its 32 KB stays erased.
 *
 * __dfu_flag is the recovery request word, the last 16 bytes of the 256 KB
 * SRAM: the boot and the application both stop their RAM short of it, so
 * neither links anything there nor clears it at startup.
 */

__dfu_flag			= 0x2003FFF0;
__dfu_boot_start	= 0x00000000;
__dfu_boot_end		= 0x00010000;
__dfu_rec_start		= 0x00010000;
__dfu_rec_end		= 0x00018000;
__dfu_slot0_start	= 0x00018000;
__dfu_slot0_end		= 0x00180000;
