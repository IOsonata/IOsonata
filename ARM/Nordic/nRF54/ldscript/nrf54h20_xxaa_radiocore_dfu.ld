/* Application with DFU, nRF54H20 radio core.
 *
 * nrf54h20_xxaa_radiocore.ld with FLASH cut down to slot 0, from the layout
 * the stage 0 boot shares, and RAM stopped short of the recovery request
 * word. The sections come from gcc_arm_flash.ld, the script ResetEntry.c is
 * written for. The stage 0 boot sets VTOR to slot 0 before it starts the
 * image.
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

INCLUDE "dfu_layout_nrf54h20_radiocore.ld"

MEMORY
{
	FLASH (rx)	: ORIGIN = __dfu_slot0_start, LENGTH = __dfu_slot0_end - __dfu_slot0_start
	FLASH1 (rx)	: ORIGIN = 0x2F840000, LENGTH = 0x4000 /* OTP0 */
	EXTFLASH (rx)	: ORIGIN = 0x70000000, LENGTH = 0x20000000
	RAM (rwx)	: ORIGIN = 0x23000000, LENGTH = __dfu_flag - 0x23000000
	RAM1 (rwx)	: ORIGIN = 0x23010000, LENGTH = 0x8000 /* Inside network RAM0 */
	RAM2 (rwx)	: ORIGIN = 0x2F000000, LENGTH = 0x80000 /* RAM00 */
	RAM3 (rwx)	: ORIGIN = 0x2F080000, LENGTH = 0x60000 /* RAM01 */
	RAM4 (rwx)	: ORIGIN = 0x2F880000, LENGTH = 0x10000 /* RAM20 */
	RAM5 (rwx)	: ORIGIN = 0x2F890000, LENGTH = 0x8000 /* RAM21 */
	RAM6 (rwx)	: ORIGIN = 0x2FC00000, LENGTH = 0x4000 /* RAM30 (low-speed) */
	RAM7 (rwx)	: ORIGIN = 0x2FC04000, LENGTH = 0x4000 /* RAM31 (low-speed) */
}

INCLUDE "gcc_arm_flash.ld"
