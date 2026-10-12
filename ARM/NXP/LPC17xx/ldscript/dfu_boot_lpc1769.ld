/* Stage 0 DFU boot, LPC1769.
 *
 * The boot ROM starts the image at 0 when word 7 of its vector table makes
 * the first eight words sum to zero. The linker calculates that word.
 *
 * RAM: the local SRAM below the IAP work area. The AHB SRAM, where the
 * recovery flag is, is left out.
 */

OUTPUT_FORMAT ("elf32-littlearm", "elf32-bigarm", "elf32-littlearm")
SEARCH_DIR(.)
SEARCH_DIR("../../../../ldscript")
SEARCH_DIR("../../../../../ldscript")
SEARCH_DIR("../../../../../../ldscript")
SEARCH_DIR("../../../../../../../ldscript")
SEARCH_DIR("../../../../../../../../ldscript")
SEARCH_DIR("../../../../IOsonata/ARM/ldscript")
SEARCH_DIR("../../../../../IOsonata/ARM/ldscript")
SEARCH_DIR("../../../../../../IOsonata/ARM/ldscript")
SEARCH_DIR("../../../../../../../IOsonata/ARM/ldscript")
GROUP(-lgcc -lc -lnosys)

INCLUDE "dfu_layout_lpc1769.ld"

MEMORY
{
	FLASH (rx)	: ORIGIN = __dfu_boot_start, LENGTH = __dfu_boot_end - __dfu_boot_start
	RAM (rwx)	: ORIGIN = __dfu_ram_start, LENGTH = __dfu_ram_end - __dfu_ram_start
}

INCLUDE "gcc_arm_flash.ld"

/* Cortex-M3 vector words 1 through 6 contain Thumb handler addresses. */
__valid_user_code_checksum = 0 - (__StackTop + (ResetEntry + 1) +
	(NMI_Handler + 1) + (HardFault_Handler + 1) + (MemManage_Handler + 1) +
	(BusFault_Handler + 1) + (UsageFault_Handler + 1));

ASSERT(ADDR(.ivector) == ORIGIN(FLASH), "LPC vectors must start the image")
