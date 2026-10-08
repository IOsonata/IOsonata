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

MEMORY
{
	FLASH (rx)         : ORIGIN = 0x00000000, LENGTH = 0x0180000  /* 1.5M */
	OFS (rx)      : ORIGIN = 0x0100A150, LENGTH = 0x0000018  /* 24B */
	E2S_TRACE_BUF (rw) : ORIGIN = 0x20000000, LENGTH = 0x000400  /* 1K   */

	/* The e2studio trace buffer requires 0x400 bytes of RAM starting at address 0x20000000. Those have been allocated above.
	* If trace is not used then these 0x400 bytes can be used by the application by changing the below RAM ORIGIN to 0x20000000,
	* increasing the LENGTH by 0x400 and removing the E2S_TRACE_BUF definition. */
	RAM (rwx)          : ORIGIN = 0x20000400, LENGTH = 0x003FC00  /* 255K */
	QSPI_FLASH (rx)    : ORIGIN = 0x60000000, LENGTH = 0x8000000  /* 128M */
}



SECTIONS
{
	/* Reset options and security MPU words, as in the DFU boot image. */
	.osm 0x400 :
	{
		LONG(0xFFFFFFFF)
		LONG(0xFFFFFFFF)
		LONG(0xFFFFFFFF)
		LONG(0xFFFFFFFF)
		LONG(0xFFFFFFFF)
		LONG(0xFFFFFFFF)
		LONG(0xFFFFFFFF)
		LONG(0xFFFFFFFF)
		LONG(0xFFFFFFFF)
		LONG(0xFFFFFFFF)
		LONG(0xFFFFFFFF)
		LONG(0xFFFFFFFF)
		LONG(0xFFFFFFFF)
		LONG(0xFFFFFFFF)
		LONG(0xFFFFFFFF)
		LONG(0xFFFFFFFF)
	} > FLASH
} INSERT AFTER .ivector;

INCLUDE "gcc_arm_flash.ld"

ASSERT(ADDR(.ivector) + SIZEOF(.ivector) <= 0x400, "vector table overlaps option-setting memory")
ASSERT(ADDR(.osm) == 0x400 && SIZEOF(.osm) == 0x40, "option-setting memory misplaced")
ASSERT(ADDR(.Version) >= 0x440, "code overlaps option-setting memory")
