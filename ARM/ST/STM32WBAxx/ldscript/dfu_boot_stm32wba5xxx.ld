/* Stage 0 DFU boot, STM32WBA52/55, 1 MB flash, 128 KB SRAM.
 *
 * With TrustZone off and BOOT0 low the part starts from NSBOOTADD0, the
 * start of main flash by default, so the boot sits there. Nothing else sits
 * at a fixed flash address: the option bytes are not in main flash.
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

INCLUDE "dfu_layout_stm32wba5xxx.ld"

MEMORY
{
	FLASH (rx)	: ORIGIN = __dfu_boot_start, LENGTH = __dfu_boot_end - __dfu_boot_start
	RAM (rwx)	: ORIGIN = 0x20000000, LENGTH = __dfu_flag - 0x20000000
}

INCLUDE "gcc_arm_flash.ld"
