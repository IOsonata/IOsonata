/* Application with DFU, STM32L476xG, 1 MB flash, 96 KB SRAM1.
 *
 * Same as gcc_stm32l476xx.ld with FLASH cut down to slot 0 and the two NVM
 * regions from the layout the stage 0 boot shares. An image that outgrows
 * slot 0 fails to link. RAM stops at the recovery request word.
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

INCLUDE "dfu_layout_stm32l476.ld"

MEMORY
{
	FLASH (rx)	: ORIGIN = __dfu_slot0_start, LENGTH = __dfu_slot0_end - __dfu_slot0_start
	NVM0 (rw)	: ORIGIN = __dfu_nvm0_start, LENGTH = __dfu_nvm_size
	NVM1 (rw)	: ORIGIN = __dfu_nvm1_start, LENGTH = __dfu_nvm_size
	RAM (rwx)	: ORIGIN = 0x20000000, LENGTH = __dfu_flag - 0x20000000
	RAM2 (rwx)	: ORIGIN = 0x10000000, LENGTH = 32K
}

SECTIONS
{
	nvm0 (NOLOAD) :
	{
		__start_nvm0 = .;
		KEEP(*(nvm0))
		. = ORIGIN(NVM0) + LENGTH(NVM0);
		__stop_nvm0 = .;
	} > NVM0

	nvm1 (NOLOAD) :
	{
		__start_nvm1 = .;
		KEEP(*(nvm1))
		. = ORIGIN(NVM1) + LENGTH(NVM1);
		__stop_nvm1 = .;
	} > NVM1
}

INCLUDE "gcc_arm_flash.ld"
