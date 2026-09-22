/* DFU flash layout, STM32WBA52/55, 1 MB flash, 128 KB SRAM.
 *
 * Included by the stage 0 boot script (dfu_boot_stm32wba5xxx.ld) and the
 * application script (gcc_stm32wba5xxx_dfu.ld), so both agree on where
 * everything is. Code reads these symbols through DfuLayoutGet; no C define
 * repeats an address. See docs/architecture/dfu.md.
 *
 *   0x08000000 - 0x0800BFFF  stage 0 boot          48 KB, where the part starts
 *   0x0800C000 - 0x0800DFFF  record of slot 0       8 KB
 *   0x0800E000 - 0x08081FFF  slot 0, application  464 KB
 *   0x08082000 - 0x080F7FFF  slot 1, upload        472 KB, trailer at the end
 *   0x080F8000 - 0x080FBFFF  NVM0, bond store      16 KB
 *   0x080FC000 - 0x080FFFFF  NVM1, application     16 KB
 *
 * A Bluetooth LE part: the application receives an image over the air into
 * slot 1 and the boot installs it. Slot 1 is one page larger than slot 0,
 * for the image header, the TLV areas and the trailer that come with a slot
 * 0 sized payload.
 *
 * The flash programs 128 bit quad words with ECC, each once between two
 * erases, in 8 KB pages, so the state unit is 16 bytes and the trailer two
 * of them. Record: one state unit, the record information, up to 0x800 of
 * image header and 0x400 of TLVs, 0xC14 bytes, in one page.
 *
 * Slot 0 starts on a page, and on 512 bytes for VTOR. NVM0 and NVM1 stay
 * where gcc_stm32wba5xxx.ld has them.
 *
 * __dfu_flag is the recovery request word, the last 16 bytes of SRAM (SRAM1
 * and SRAM2 contiguous, 128 KB on WBA52 and WBA55): the boot and the
 * application both stop their RAM short of it, so neither links anything
 * there nor clears it at startup.
 */

__dfu_flag				= 0x2001FFF0;
__dfu_boot_start		= 0x08000000;
__dfu_boot_end			= 0x0800C000;
__dfu_rec_start			= 0x0800C000;
__dfu_rec_end			= 0x0800E000;
__dfu_slot0_start		= 0x0800E000;
__dfu_slot0_end			= 0x08082000;
__dfu_slot1_start		= 0x08082000;
__dfu_slot1_end			= 0x080F8000;
__dfu_nvm0_start		= 0x080F8000;
__dfu_nvm1_start		= 0x080FC000;
__dfu_nvm_size			= 0x00004000;
