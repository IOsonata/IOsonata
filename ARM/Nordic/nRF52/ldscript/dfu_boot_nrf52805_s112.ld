/* Stage 0 DFU boot, nRF52805 with MBR and S112.
 *
 * The MBR starts the boot because UICR.NRFFW[0] holds its address; the boot
 * image sets that word itself, so a merged hex needs nothing added.
 *
 * RAM: the MBR keeps the first 8 bytes. The SoftDevice is not running while
 * the boot is, so the rest is the boot's.
 */

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

INCLUDE "dfu_layout_nrf52805_s112.ld"

MEMORY
{
	FLASH (rx)			: ORIGIN = __dfu_boot_start, LENGTH = __dfu_boot_end - __dfu_boot_start
	RAM (rwx)			: ORIGIN = 0x20000008, LENGTH = __dfu_flag - 0x20000008
	UICR_BOOTADDR (r)	: ORIGIN = 0x10001014, LENGTH = 4
}

SECTIONS
{
	/* UICR.NRFFW[0]: where the MBR finds the boot. */
	.uicr_boot_addr :
	{
		LONG(ORIGIN(FLASH))
	} > UICR_BOOTADDR
}

INCLUDE "gcc_arm_flash.ld"
