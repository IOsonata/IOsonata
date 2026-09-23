/* DFU flash layout, STM32F401xC, 256 KB flash, 64 KB SRAM.
 *
 * Included by the stage 0 boot script (dfu_boot_stm32f401xc.ld) and the
 * application script (gcc_stm32f401xc_dfu.ld), so both agree on where
 * everything is. Code reads these symbols through DfuLayoutGet; no C define
 * repeats an address. See docs/architecture/dfu.md.
 *
 *   0x08000000 - 0x0800BFFF  stage 0 boot          48 KB, sectors 0 to 2
 *   0x0800C000 - 0x0800FFFF  record of slot 0      16 KB, sector 3
 *   0x08010000 - 0x0803FFFF  slot 0, application  192 KB, sectors 4 (64 KB)
 *                                                          and 5 (128 KB)
 *
 * Wired only: no slot 1. The boot writes slot 0 and the record directly
 * over the UART.
 *
 * F4 erases by sector, and the sectors differ in size: 16, 16, 16, 16, 64,
 * 128 KB here. Every region is whole sectors. The record needs 0xC14 bytes
 * (a 16 byte state unit, the record information, up to 0x800 of image
 * header and 0x400 of TLVs) and gets the last 16 KB sector.
 *
 * Slot 0 starts on 512 bytes for VTOR.
 *
 * __dfu_flag is the recovery request word, the last 16 bytes of SRAM: the
 * boot and the application both stop their RAM short of it, so neither
 * links anything there nor clears it at startup.
 *
 * There is no application script for this part in IOsonata other than the
 * DFU one below; it is gcc_stm32f401xc_dfu.ld.
 */

__dfu_flag				= 0x2000FFF0;
__dfu_boot_start		= 0x08000000;
__dfu_boot_end			= 0x0800C000;
__dfu_rec_start			= 0x0800C000;
__dfu_rec_end			= 0x08010000;
__dfu_slot0_start		= 0x08010000;
__dfu_slot0_end			= 0x08040000;
