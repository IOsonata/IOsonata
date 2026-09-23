/* DFU flash layout, STM32L476xG, 1 MB flash, 96 KB SRAM1.
 *
 * Included by the stage 0 boot script (dfu_boot_stm32l476.ld) and the
 * application script (gcc_stm32l476xx_dfu.ld), so both agree on where
 * everything is. Code reads these symbols through DfuLayoutGet; no C define
 * repeats an address. See docs/architecture/dfu.md.
 *
 *   0x08000000 - 0x0800BFFF  stage 0 boot          48 KB, where the part starts
 *   0x0800C000 - 0x0800CFFF  record of slot 0       4 KB
 *   0x0800D000 - 0x080FBFFF  slot 0, application  956 KB
 *   0x080FC000 - 0x080FDFFF  NVM0, bond store       8 KB
 *   0x080FE000 - 0x080FFFFF  NVM1, application      8 KB
 *
 * Wired only: no slot 1. The boot writes slot 0 and the record directly
 * over the UART.
 *
 * The flash programs 64 bit double words with ECC, each once between two
 * erases, so the state unit is 16 bytes. Record: one state unit, the record
 * information, up to 0x800 of image header and 0x400 of TLVs, 0xC14 bytes,
 * so two 2 KB pages.
 *
 * Slot 0 starts on a page, and on 512 bytes for VTOR. It runs across the
 * bank boundary at 0x08080000, which nothing here minds. NVM0 and NVM1 stay
 * where gcc_stm32l476xx.ld has them, at the top of bank 2.
 *
 * __dfu_flag is the recovery request word, the last 16 bytes of SRAM1: the
 * boot and the application both stop their RAM short of it, so neither
 * links anything there nor clears it at startup. SRAM1 keeps its content
 * across a system reset.
 */

__dfu_flag				= 0x20017FF0;
__dfu_boot_start		= 0x08000000;
__dfu_boot_end			= 0x0800C000;
__dfu_rec_start			= 0x0800C000;
__dfu_rec_end			= 0x0800D000;
__dfu_slot0_start		= 0x0800D000;
__dfu_slot0_end			= 0x080FC000;
__dfu_nvm0_start		= 0x080FC000;
__dfu_nvm1_start		= 0x080FE000;
__dfu_nvm_size			= 0x00002000;
