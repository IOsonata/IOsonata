/* Application with DFU, STM32F030x8.
 *
 * Same as gcc_stm32f030x8.ld with FLASH cut down to slot 0, from the layout
 * the stage 0 boot shares. An image that outgrows slot 0 fails to link.
 *
 * RAM starts after the vector table copy the boot makes at the start of
 * SRAM (Cortex-M0, no VTOR) and stops at the recovery request word.
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

INCLUDE "dfu_layout_stm32f030x8.ld"

MEMORY
{
	FLASH (rx)	: ORIGIN = __dfu_slot0_start, LENGTH = __dfu_slot0_end - __dfu_slot0_start
	RAM (rwx)	: ORIGIN = __dfu_vec_ram + __dfu_vec_ram_size, LENGTH = __dfu_flag - (__dfu_vec_ram + __dfu_vec_ram_size)
}

INCLUDE "gcc_arm_flash.ld"
