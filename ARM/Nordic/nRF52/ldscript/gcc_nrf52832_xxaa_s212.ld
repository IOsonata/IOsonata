OUTPUT_FORMAT ("elf32-littlearm", "elf32-bigarm", "elf32-littlearm")
SEARCH_DIR(.)
SEARCH_DIR("../../../../ldscript")
SEARCH_DIR("../../../../../ldscript")
SEARCH_DIR("../../../../../../ldscript")
SEARCH_DIR("../../../../../../../ldscript")
SEARCH_DIR("../../IOsonata/ARM/ldscript")
SEARCH_DIR("../../../IOsonata/ARM/ldscript")
SEARCH_DIR("../../../../../IOsonata/ARM/ldscript")
SEARCH_DIR("../../../../../../IOsonata/ARM/ldscript")
SEARCH_DIR("../../IOcomposer/IOsonata/ARM/ldscript")
SEARCH_DIR("../../../IOcomposer/IOsonata/ARM/ldscript")
SEARCH_DIR("../../../../IOcomposer/IOsonata/ARM/ldscript")
SEARCH_DIR("../../../../../IOcomposer/IOsonata/ARM/ldscript")
SEARCH_DIR("../../../../../../IOcomposer/IOsonata/ARM/ldscript")
GROUP(-lgcc -lc)

MEMORY
{
	FLASH (rx) : ORIGIN = 0x12000, LENGTH = 0x6b000
	NVM0 (rw) : ORIGIN = 0x0007D000, LENGTH = 0x00002000
	RAM (rwx) :  ORIGIN = 0x20000a00, LENGTH = 0x7600
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
}

SECTIONS
{
  .fs_data :
  {
    PROVIDE(__start_fs_data = .);
    KEEP(*(.fs_data))
    PROVIDE(__stop_fs_data = .);
  } > RAM
} INSERT AFTER .data;

SECTIONS
{
  .pwr_mgmt_data :
  {
    PROVIDE(__start_pwr_mgmt_data = .);
    KEEP(*(SORT(.pwr_mgmt_data*)))
    PROVIDE(__stop_pwr_mgmt_data = .);
  } > FLASH
} INSERT AFTER .text

INCLUDE "gcc_arm_flash.ld"
