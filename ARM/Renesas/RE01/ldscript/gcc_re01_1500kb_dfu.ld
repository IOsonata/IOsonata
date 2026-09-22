/* Application with DFU, Renesas RE01 1500KB.
 *
 * Same as gcc_re01_1500kb.ld with FLASH cut down to slot 0 and RAM stopped
 * short of the recovery request word, from the layout the stage 0 boot
 * shares. An image that outgrows slot 0 fails to link. The boot owns the
 * reset vectors and the option setting memory at 0x400; this image starts
 * at slot 0, where the boot points VTOR.
 */

SEARCH_DIR(.)
SEARCH_DIR("../../../ldscript")
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
GROUP( -lstdc++ -lgcc -lc)

INCLUDE "dfu_layout_re01_1500kb.ld"

MEMORY
{
	FLASH (rx)         : ORIGIN = __dfu_slot0_start, LENGTH = __dfu_slot0_end - __dfu_slot0_start
	E2S_TRACE_BUF (rw) : ORIGIN = 0x20000000, LENGTH = 0x000400  /* 1K   */

	/* The e2studio trace buffer requires 0x400 bytes of RAM starting at address 0x20000000. Those have been allocated above.
	* If trace is not used then these 0x400 bytes can be used by the application by changing the below RAM ORIGIN to 0x20000000,
	* increasing the LENGTH by 0x400 and removing the E2S_TRACE_BUF definition. */
	RAM (rwx)          : ORIGIN = 0x20000400, LENGTH = __dfu_flag - 0x20000400
	QSPI_FLASH (rx)    : ORIGIN = 0x60000000, LENGTH = 0x8000000  /* 128M */
}

INCLUDE "gcc_arm_flash.ld"
