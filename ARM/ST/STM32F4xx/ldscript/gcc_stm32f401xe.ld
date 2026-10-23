/* STM32F401RE: 512 KB FLASH and 96 KB SRAM.
 * NUCLEO-F401RE, application at internal FLASH base.
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

MEMORY
{
	FLASH (rx) : ORIGIN = 0x08000000, LENGTH = 512K
	RAM (rwx) : ORIGIN = 0x20000000, LENGTH = 96K
}

INCLUDE "gcc_arm_flash.ld"
