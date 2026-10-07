/* Secure stage, nRF9160.
 *
 * The secure stage is at 0 and runs out of reset, secure. It gives the
 * memory and the peripherals the layout assigns to the non secure side and
 * starts the non secure application, see nRF91SpeStart in spe_nrf91.h. Its
 * flash stops at the reserved non secure callable area, its RAM is the
 * secure RAM of the layout.
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
GROUP(-lgcc -lc -lnosys)

INCLUDE "tz_layout_nrf9160.ld"

MEMORY
{
	FLASH (rx)	: ORIGIN = __tz_s_flash_start, LENGTH = __tz_nsc_start - __tz_s_flash_start
	RAM (rwx)	: ORIGIN = __tz_s_ram_start, LENGTH = __tz_s_ram_end - __tz_s_ram_start
}

INCLUDE "gcc_arm_flash.ld"
