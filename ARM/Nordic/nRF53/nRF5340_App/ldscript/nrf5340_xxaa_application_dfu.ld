/* Application with DFU, nRF5340 application core.
 *
 * Same as nrf5340_xxaa_application.ld with FLASH cut down to slot 0, from the
 * layout the stage 0 boot shares, and RAM1 stopped short of the recovery
 * request word. An image that outgrows slot 0 fails to link. The stage 0
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

INCLUDE "dfu_layout_nrf5340_application.ld"

MEMORY
{
	FLASH (rx)	: ORIGIN = __dfu_slot0_start, LENGTH = __dfu_slot0_end - __dfu_slot0_start
	EXTFLASH (rx)	: ORIGIN = 0x10000000, LENGTH = 0x8000000
	RAM (rwx)	: ORIGIN = 0x20000000, LENGTH = 0x40000
	RAM1 (rwx)	: ORIGIN = 0x20040000, LENGTH = __dfu_flag - 0x20040000
}

INCLUDE "gcc_arm_flash.ld"
