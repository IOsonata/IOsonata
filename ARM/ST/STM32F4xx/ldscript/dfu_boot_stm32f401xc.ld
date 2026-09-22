/* Stage 0 DFU boot, STM32F401xC, 256 KB flash, 64 KB SRAM.
 *
 * The part starts from the start of main flash (BOOT0 low), aliased at 0,
 * so the boot sits there. Nothing else sits at a fixed flash address: the
 * option bytes are not in main flash.
 *
 * RAM: SRAM up to the recovery request word.
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
	FLASH (rx)	: ORIGIN = __dfu_boot_start, LENGTH = __dfu_boot_end - __dfu_boot_start
	RAM (rwx)	: ORIGIN = 0x20000000, LENGTH = __dfu_flag - 0x20000000
}

INCLUDE "gcc_arm_flash.ld"
