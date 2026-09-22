/* Stage 0 DFU boot, Renesas RE01 1500KB.
 *
 * The core starts from the vector table at 0, which is the start of the boot.
 *
 * The option setting memory (OFS0, OFS1, security MPU words) sits in the
 * code flash at 0x400 - 0x43F, read at every reset. It belongs to the boot
 * image: the boot keeps its code out of it and fills it with ones, the
 * erased value (watchdogs not started, voltage monitor off, security MPU
 * disabled). A product that needs other option values changes them here.
 * The application image never covers it.
 *
 * RAM: the first 1 KB is left to the e2studio trace buffer, as in
 * gcc_re01_1500kb.ld.
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
	FLASH (rx)         : ORIGIN = __dfu_boot_start, LENGTH = __dfu_boot_end - __dfu_boot_start
	OFS (rx)           : ORIGIN = 0x0100A150, LENGTH = 0x0000018  /* 24B */
	E2S_TRACE_BUF (rw) : ORIGIN = 0x20000000, LENGTH = 0x000400  /* 1K   */
	RAM (rwx)          : ORIGIN = 0x20000400, LENGTH = __dfu_flag - 0x20000400
}

SECTIONS
{
	/* Option setting memory, right after the vector table. */
	.osm 0x400 :
	{
		LONG(0xFFFFFFFF)	/* OFS0 */
		LONG(0xFFFFFFFF)	/* OFS1 */
		LONG(0xFFFFFFFF)	/* SECMPUPCS0 */
		LONG(0xFFFFFFFF)	/* SECMPUPCE0 */
		LONG(0xFFFFFFFF)	/* SECMPUPCS1 */
		LONG(0xFFFFFFFF)	/* SECMPUPCE1 */
		LONG(0xFFFFFFFF)	/* SECMPUS0 */
		LONG(0xFFFFFFFF)	/* SECMPUE0 */
		LONG(0xFFFFFFFF)	/* SECMPUS1 */
		LONG(0xFFFFFFFF)	/* SECMPUE1 */
		LONG(0xFFFFFFFF)	/* SECMPUS2 */
		LONG(0xFFFFFFFF)	/* SECMPUE2 */
		LONG(0xFFFFFFFF)	/* reserved */
		LONG(0xFFFFFFFF)	/* reserved */
		LONG(0xFFFFFFFF)	/* SECMPUAC, reserved */
		LONG(0xFFFFFFFF)	/* reserved */
	} > FLASH
} INSERT AFTER .ivector;

INCLUDE "gcc_arm_flash.ld"

ASSERT(ADDR(.osm) == 0x400 && SIZEOF(.osm) == 0x40, "option setting memory misplaced")
ASSERT(ADDR(.ivector) + SIZEOF(.ivector) <= 0x400, "vector table overlaps the option setting memory")
