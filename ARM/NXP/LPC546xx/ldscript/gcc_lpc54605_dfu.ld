/* Application with DFU, LPC54605J512.
 *
 * There is no LPC54605 application script without DFU in the tree; this
 * one follows the other IOsonata scripts: FLASH is slot 0, RAM is SRAM0 up
 * to the words kept for the IAP ROM, from the layout the stage 0 boot
 * shares. An image that outgrows slot 0 fails to link.
 *
 * SRAM1 to SRAM3 (RAM2 here) are for data placed there explicitly, used
 * once SystemInit has turned their clocks on; RAM2 stops short of the
 * recovery flag.
 */

OUTPUT_FORMAT ("elf32-littlearm", "elf32-bigarm", "elf32-littlearm")
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
GROUP(-lstdc++ -lgcc -lc -lnosys)

INCLUDE "dfu_layout_lpc54605.ld"

MEMORY
{
	FLASH (rx)	: ORIGIN = __dfu_slot0_start, LENGTH = __dfu_slot0_end - __dfu_slot0_start
	RAM (rwx)	: ORIGIN = __dfu_ram_start, LENGTH = __dfu_ram_end - __dfu_ram_start
	RAM2 (rwx)	: ORIGIN = 0x20010000, LENGTH = __dfu_flag - 0x20010000
	SRAMX (rwx)	: ORIGIN = 0x04000000, LENGTH = 32K
}

INCLUDE "gcc_arm_flash.ld"
