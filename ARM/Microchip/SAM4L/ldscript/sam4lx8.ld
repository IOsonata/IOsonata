OUTPUT_FORMAT("elf32-littlearm", "elf32-littlearm", "elf32-littlearm")
OUTPUT_ARCH(arm)
SEARCH_DIR(.)
SEARCH_DIR("../../../../ldscript")
SEARCH_DIR("../../../../../ldscript")
SEARCH_DIR("../../../../../../ldscript")
SEARCH_DIR("../../../../../../../ldscript")
SEARCH_DIR("../../../../../../../../ldscript")
SEARCH_DIR("../../../../IOsonata/ARM/ldscript")
SEARCH_DIR("../../../../../IOsonata/ARM/ldscript")
SEARCH_DIR("../../../../../../IOsonata/ARM/ldscript")
SEARCH_DIR("../../../../../../../IOsonata/ARM/ldscript")
SEARCH_DIR("../../../../../../../../IOsonata/ARM/ldscript")
GROUP(-lgcc -lc -lnosys)

MEMORY
{
    FLASH (rx)  : ORIGIN = 0x00000000, LENGTH = 0x80000
    RAM (rwx) : ORIGIN = 0x20000000, LENGTH = 0x10000
}

INCLUDE "gcc_arm_flash.ld"

