/*
 * LPC11xx
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


MEMORY
{
  
	FLASH (rx) : ORIGIN = 0x00000000, LENGTH = 64K
	/* UM10462 section 20.8.9: IAP uses the top 32 bytes of SRAM0. */
	RAM (rwx) : ORIGIN = 0x10000000, LENGTH = 8K - 32
  	RAM2 (rwx) : ORIGIN = 0x20000000, LENGTH = 2K
  	RAM3 (rwx) : ORIGIN = 0x20004000, LENGTH = 2K
}

/* USB DMA storage must not be cleared by ResetEntry before USBRAM is clocked.
 * The controller initializes this optional section after enabling its clock.
 * Region placement also makes overflow a link error. */
SECTIONS
{
	.usb_ram (NOLOAD) : ALIGN(256)
	{
		*(.usb_ram .usb_ram.*)
	} > RAM3
}

INCLUDE "gcc_arm_flash.ld"

/* Cortex-M0 vector words 4 to 6 are zero. Include the Thumb bit for each
 * handler, following the LPC546xx linker checksum convention. */
__valid_user_code_checksum = 0 - (__StackTop + (ResetEntry + 1) +
								  (NMI_Handler + 1) + (HardFault_Handler + 1));



