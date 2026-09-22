/* DFU flash layout, STM32L4S9xI, 2 MB flash, 640 KB SRAM.
 *
 * Included by the stage 0 boot script (dfu_boot_stm32l4s9.ld) and the
 * application script (gcc_stm32l4s9_dfu.ld), so both agree on where
 * everything is. Code reads these symbols through DfuLayoutGet; no C define
 * repeats an address. See docs/architecture/dfu.md.
 *
 *   0x08000000 - 0x0800BFFF  stage 0 boot           48 KB, the part starts here
 *   0x0800C000 - 0x0800DFFF  record of slot 0        8 KB
 *   0x0800E000 - 0x081F7FFF  slot 0, application  1960 KB
 *   0x081F8000 - 0x081FBFFF  NVM0, bond store       16 KB
 *   0x081FC000 - 0x081FFFFF  NVM1, application      16 KB
 *
 * Wired only: no slot 1. The boot writes slot 0 and the record directly
 * over the UART.
 *
 * L4+ pages are 4 KB with the DBANK option set (two 1 MB banks, the factory
 * setting) and 8 KB with it clear. Every region starts on 8 KB, so the
 * layout holds in both modes; the target layer reads DBANK.
 *
 * The flash programs 64 bit double words with ECC, each once between two
 * erases, so the state unit is 16 bytes. Record: one state unit, the record
 * information, up to 0x800 of image header and 0x400 of TLVs, 0xC14 bytes,
 * in one 8 KB unit.
 *
 * Slot 0 starts on 512 bytes for VTOR. It runs across the bank boundary at
 * 0x08100000. NVM0 and NVM1 stay where gcc_stm32l4s9.ld has them.
 *
 * __dfu_flag is the recovery request word, the last 16 bytes of SRAM (SRAM1,
 * SRAM2 and SRAM3 contiguous): the boot and the application both stop
 * their RAM short of it, so neither links anything there nor clears it at
 * startup.
 */

__dfu_flag				= 0x2009FFF0;
__dfu_boot_start		= 0x08000000;
__dfu_boot_end			= 0x0800C000;
__dfu_rec_start			= 0x0800C000;
__dfu_rec_end			= 0x0800E000;
__dfu_slot0_start		= 0x0800E000;
__dfu_slot0_end			= 0x081F8000;
__dfu_nvm0_start		= 0x081F8000;
__dfu_nvm1_start		= 0x081FC000;
__dfu_nvm_size			= 0x00004000;
