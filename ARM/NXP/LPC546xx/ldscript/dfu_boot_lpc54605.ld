/* Stage 0 DFU boot, LPC54605J512.
 *
 * The boot ROM starts the image at 0 when word 7 of its vector table makes
 * the first eight words sum to zero; the flash programmer writes that word,
 * as for every LPC image.
 *
 * RAM: SRAM0 below the words kept for the IAP ROM.
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

INCLUDE "dfu_layout_lpc54605.ld"

MEMORY
{
	FLASH (rx)	: ORIGIN = __dfu_boot_start, LENGTH = __dfu_boot_end - __dfu_boot_start
	RAM (rwx)	: ORIGIN = __dfu_ram_start, LENGTH = __dfu_ram_end - __dfu_ram_start
}

INCLUDE "gcc_arm_flash.ld"
