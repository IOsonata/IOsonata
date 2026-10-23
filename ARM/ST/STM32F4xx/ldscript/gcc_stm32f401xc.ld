/* STM32F401xC: 256 KB flash, 64 KB SRAM. */
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
	FLASH (rx) : ORIGIN = 0x08000000, LENGTH = 256K
	RAM (rwx) : ORIGIN = 0x20000000, LENGTH = 64K
}
INCLUDE "gcc_arm_flash.ld"
