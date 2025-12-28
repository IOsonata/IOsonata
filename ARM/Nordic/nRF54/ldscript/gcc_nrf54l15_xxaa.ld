/* Linker script to configure memory regions. */

SEARCH_DIR(.)
SEARCH_DIR("../../../../ldscript")
SEARCH_DIR("../../../../../ldscript")
SEARCH_DIR("../../../../../../ldscript")
SEARCH_DIR("../../../../../../../ldscript")
SEARCH_DIR("../../IOsonata/ARM/ldscript")
SEARCH_DIR("../../../IOsonata/ARM/ldscript")
SEARCH_DIR("../../../../IOsonata/ARM/ldscript")
SEARCH_DIR("../../../../../IOsonata/ARM/ldscript")
SEARCH_DIR("../../../../../../IOsonata/ARM/ldscript")
GROUP(-lgcc -lc)

MEMORY
{
  FLASH (rx) : ORIGIN = 0x00000000, LENGTH = 0x17D000 /* Inside global RRAM0 */
  RAM (rwx) : ORIGIN = 0x20000000, LENGTH = 0x20000
  RAM1 (rwx) : ORIGIN = 0x20020000, LENGTH = 0x20000
}


INCLUDE "gcc_arm_flash.ld"
