/* nRF9160 Linker script to configure memory regions. */

SEARCH_DIR(.)
SEARCH_DIR("../../../../ldscript")
SEARCH_DIR("../../../../../ldscript")
SEARCH_DIR("../../../../../../ldscript")
SEARCH_DIR("../../../../../../../ldscript")
SEARCH_DIR("../../../../IOsonata/ARM/ldscript")
SEARCH_DIR("../../../../../IOsonata/ARM/ldscript")
SEARCH_DIR("../../../../../../IOsonata/ARM/ldscript")
SEARCH_DIR("../../../../../../../IOsonata/ARM/ldscript")
GROUP(-lgcc -lc -lnosys)

MEMORY
{
  FLASH (rx) : ORIGIN = 0x00000000, LENGTH = 0x100000
  RAM (rwx) :  ORIGIN = 0x20000000, LENGTH = 0x40000
}

SECTIONS 
{
 	.bsdlib 0x20010000 : {KEEP(*(.bsdlibData))}
/* 	.nrf_rpc : 
 	{
		KEEP(*(SORT_BY_NAME(".nrf_rpc.*")))
		KEEP(*(SORT_BY_NAME(".bsd_init.*")))
	} > FLASH*/
}

INCLUDE "gcc_arm_flash.ld"
