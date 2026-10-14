/* Application with DFU, LPC1767 to LPC1769.
 *
 * Same as gcc_lpc1767_9.ld with FLASH cut down to slot 0, the local SRAM
 * below the IAP work area and the AHB SRAM short of the recovery flag, from
 * the layout the stage 0 boot shares. An image that outgrows slot 0 fails to
 * link.
 *
 * .data_RAMAHB goes after .data so its load image cannot precede the
 * vector table at the start of slot 0.
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
GROUP( -lstdc++ -lgcc -lc)

INCLUDE "dfu_layout_lpc1769.ld"

MEMORY
{
	FLASH (rx)		: ORIGIN = __dfu_slot0_start, LENGTH = __dfu_slot0_end - __dfu_slot0_start
	RAM (rwx)		: ORIGIN = __dfu_ram_start, LENGTH = __dfu_ram_end - __dfu_ram_start
	RAMAHB (rwx)	: ORIGIN = 0x2007c000, LENGTH = __dfu_flag - 0x2007c000
	/* The AHB sram is divided into 2 regions of 16KB*/
	RAMAHB0 (rwx)	: ORIGIN = 0x2007c000, LENGTH = 16K	/* Bank 0 */
	RAMAHB1 (rwx)	: ORIGIN = 0x20080000, LENGTH = __dfu_flag - 0x20080000	/* Bank 1 */
}

SECTIONS
{
	.data_RAMAHB :
	{
		. = ALIGN(4);
		__data_ahb_start__ = .;
		*(.data.RAM2*)
		*(.data.$RAM2*)
		*(.data.$RamAHB32*)
		*(RAM2*)
		*(RAMAHB*)
		*(.RAMAHB*)
		*(.data.RAMAHB*)
		. = ALIGN(4);
		__data_ahb_end__ = .;
	} > RAMAHB AT > FLASH
	__data_ahb_loc__ = LOADADDR(.data_RAMAHB);
	__data_ahb_size__ = SIZEOF(.data_RAMAHB);
} INSERT AFTER .data;

INCLUDE "gcc_arm_flash.ld"

/* Cortex-M3 vector words 1 through 6 contain Thumb handler addresses. */
__valid_user_code_checksum = 0 - (__StackTop + (ResetEntry + 1) +
	(NMI_Handler + 1) + (HardFault_Handler + 1) + (MemManage_Handler + 1) +
	(BusFault_Handler + 1) + (UsageFault_Handler + 1));

ASSERT(ADDR(.ivector) == ORIGIN(FLASH), "LPC vectors must start the image")
