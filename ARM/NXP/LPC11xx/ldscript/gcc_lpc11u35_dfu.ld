/* Application with DFU, LPC11U35.
 *
 * Same as gcc_lpc11u35.ld with FLASH cut down to slot 0 and RAM to the range
 * the layout gives, from the layout the stage 0 boot shares. An image that
 * outgrows slot 0 fails to link.
 *
 * The first bytes of SRAM0 hold the vector table the boot copies there and
 * maps at 0, and the recovery flag follows it, so RAM starts after both.
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
GROUP(-lstdc++ -lgcc -lc -lnosys)

INCLUDE "dfu_layout_lpc11u35.ld"

MEMORY
{
	FLASH (rx)	: ORIGIN = __dfu_slot0_start, LENGTH = __dfu_slot0_end - __dfu_slot0_start
	RAM (rwx)	: ORIGIN = __dfu_ram_start, LENGTH = __dfu_ram_end - __dfu_ram_start
	RAM2 (rwx)	: ORIGIN = 0x20000000, LENGTH = 2K
	RAM3 (rwx)	: ORIGIN = 0x20004000, LENGTH = 2K
}

INCLUDE "gcc_arm_flash.ld"

/* Cortex-M0 vector words 4 to 6 are zero. Include the Thumb bit for each
 * handler, following the LPC546xx linker checksum convention. */
__valid_user_code_checksum = 0 - (__StackTop + (ResetEntry + 1) +
								  (NMI_Handler + 1) + (HardFault_Handler + 1));
