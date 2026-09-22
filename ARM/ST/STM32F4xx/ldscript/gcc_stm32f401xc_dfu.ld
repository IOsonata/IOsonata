/* Application with DFU, STM32F401xC, 256 KB flash, 64 KB SRAM.
 *
 * FLASH is slot 0 from the layout the stage 0 boot shares; an image that
 * outgrows it fails to link. RAM is all 64 KB of SRAM up to the recovery
 * request word.
 */

SEARCH_DIR(.)
SEARCH_DIR("../../../ldscript")
SEARCH_DIR("../../../../ldscript")
SEARCH_DIR("../../../../../ldscript")
SEARCH_DIR("../../../../../../ldscript")
SEARCH_DIR("../../../../../../../ldscript")
SEARCH_DIR("../../../../../../../../ldscript")
SEARCH_DIR("../../../../IOsonata/ARM/ldscript")
SEARCH_DIR("../../../../../IOsonata/ARM/ldscript")
SEARCH_DIR("../../../../../../IOsonata/ARM/ldscript")
SEARCH_DIR("../../../../../../../IOsonata/ARM/ldscript")
GROUP( -lstdc++ -lgcc -lc)

INCLUDE "dfu_layout_stm32f401xc.ld"

MEMORY
{
	FLASH (rx)	: ORIGIN = __dfu_slot0_start, LENGTH = __dfu_slot0_end - __dfu_slot0_start
	RAM (rwx)	: ORIGIN = 0x20000000, LENGTH = __dfu_flag - 0x20000000
}

INCLUDE "gcc_arm_flash.ld"
