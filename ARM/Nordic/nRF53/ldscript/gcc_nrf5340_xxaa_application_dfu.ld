/* Application with DFU, nRF5340 application core.
 *
 * Same as gcc_nrf5340_xxaa_application.ld with FLASH cut down to slot 0, from the
 * layout the stage 0 boot shares. NVM0 stays out of slot 0 and the
 * shared IPC block stays out of ordinary RAM. The recovery request word
 * is excluded from IPC0 and kept out of all allocated RAM regions. An image that outgrows slot 0 fails to link. The stage 0
 * boot sets VTOR to slot 0 before it starts the image.
 */

SEARCH_DIR(.)
SEARCH_DIR("../../../../ldscript")
SEARCH_DIR("../../../../../ldscript")
SEARCH_DIR("../../../../../../ldscript")
SEARCH_DIR("../../../../../../../ldscript")
SEARCH_DIR("../../../../../../../../ldscript")
SEARCH_DIR("../../IOsonata/ARM/ldscript")
SEARCH_DIR("../../../IOsonata/ARM/ldscript")
SEARCH_DIR("../../../../IOsonata/ARM/ldscript")
SEARCH_DIR("../../../../../IOsonata/ARM/ldscript")
SEARCH_DIR("../../../../../../IOsonata/ARM/ldscript")
SEARCH_DIR("../../../../../../../IOsonata/ARM/ldscript")
GROUP(-lgcc -lc)

INCLUDE "gcc_nrf5340_xxaa_application_dfu_layout.ld"

MEMORY
{
	FLASH (rx)	: ORIGIN = __dfu_slot0_start, LENGTH = __dfu_slot0_end - __dfu_slot0_start
	NVM0 (rw)	: ORIGIN = __dfu_nvm0_start, LENGTH = __dfu_nvm0_end - __dfu_nvm0_start
	EXTFLASH (rx)	: ORIGIN = 0x10000000, LENGTH = 0x8000000
	RAM (rwx)	: ORIGIN = 0x20000000, LENGTH = 0x40000
	RAM1 (rwx)	: ORIGIN = 0x20040000, LENGTH = 0x30000
	IPC0 (rw)	: ORIGIN = 0x20070000, LENGTH = __dfu_flag - 0x20070000
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

	ipc0 (NOLOAD) :
	{
		__start_ipc0 = .;
		KEEP(*(ipc0))
		. = ORIGIN(IPC0) + LENGTH(IPC0);
		__stop_ipc0 = .;
	} > IPC0
}

INCLUDE "gcc_arm_flash.ld"
