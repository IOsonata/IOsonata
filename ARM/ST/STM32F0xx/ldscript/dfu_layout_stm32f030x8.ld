/* DFU flash layout, STM32F030x8: 64 KB flash in 1 KB pages, 8 KB SRAM.
 *
 * Included by the stage 0 boot script (dfu_boot_stm32f030x8.ld) and the
 * application script (gcc_stm32f030x8_dfu.ld), so both agree on where
 * everything is. Code reads these symbols through DfuLayoutGet; no C define
 * repeats an address. See docs/architecture/dfu.md.
 *
 *   0x08000000 - 0x08008BFF  stage 0 boot          35 KB, where the part starts
 *   0x08008C00 - 0x08009BFF  record of slot 0       4 KB
 *   0x08009C00 - 0x0800FFFF  slot 0, application   25 KB
 *
 * The boot is 48 KB on larger parts; here it gets the fewest pages it fits
 * in. Built at -Os with arm-none-eabi-gcc 13.2 it is 35312 bytes of flash.
 *
 * Wired only: no slot 1. The boot writes slot 0 and the record directly
 * over the UART.
 *
 * Record: one 16 byte state unit, the record information, up to 0x800 of
 * image header and 0x400 of TLVs, 0xC14 bytes, so four 1 KB pages.
 *
 * Cortex-M0 has no VTOR. The boot copies the application vector table, 48
 * words, to the start of SRAM and maps SRAM at 0; the application script
 * starts its RAM after those 0xC0 bytes.
 *
 * __dfu_flag is the recovery request word, the last 16 bytes of SRAM: the
 * boot and the application both stop their RAM short of it, so neither
 * links anything there nor clears it at startup.
 *
 * SRAM is what limits the boot here, not flash. With DFU_BOOT_BUFSIZE 256
 * and DFU_BOOT_FIFODEPTH 64 (its board.h) the static data leaves 1772 bytes
 * of stack below __dfu_flag; the P-256 verify goes 1556 bytes deep, which
 * leaves 216 bytes, room for an interrupt frame and little else.
 */

__dfu_flag			= 0x20001FF0;
__dfu_boot_start	= 0x08000000;
__dfu_boot_end		= 0x08008C00;
__dfu_rec_start		= 0x08008C00;
__dfu_rec_end		= 0x08009C00;
__dfu_slot0_start	= 0x08009C00;
__dfu_slot0_end		= 0x08010000;
__dfu_vec_ram		= 0x20000000;
__dfu_vec_ram_size	= 0x000000C0;
