/* Stage 0 DFU boot, LPC11U35.
 *
 * The boot ROM starts the image at 0 when word 7 of its vector table makes
 * the first eight words sum to zero; the flash programmer writes that word,
 * as for every LPC image.
 *
 * DfuTgtStart maps RAM over the first 512 bytes before it jumps, so its code
 * has to be above them, which the end of this script checks.
 *
 * RAM: the SRAM0 range the layout gives, below the IAP work area and above
 * the application vector copy and the recovery flag. The boot copies the
 * application vectors to the start of SRAM0 just before it starts it, so it
 * keeps nothing there either.
 *
 * The boot and its stack do not fit the 8 KB of SRAM0, even with the
 * smaller SMP and FIFO sizes board.h may set, so the largest buffers of
 * dfu_boot_main that are only used once main runs go in the 2 KB USB SRAM:
 * the SMP receive and transmit buffers, the SMP server and the UART receive
 * FIFO. That section is not cleared by ResetEntry and its clock is off at
 * reset; the DFU target (dfu_lpc11uxx.cpp) turns the clock on and clears it
 * from the C library preinit array, before any constructor. A buffer that
 * no longer matches a name here stays in SRAM0, and the link then fails on
 * RAM rather than running short of stack.
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
GROUP(-lgcc -lc -lnosys)

INCLUDE "dfu_layout_lpc11u35.ld"

MEMORY
{
	FLASH (rx)	: ORIGIN = __dfu_boot_start, LENGTH = __dfu_boot_end - __dfu_boot_start
	RAM (rwx)	: ORIGIN = __dfu_ram_start, LENGTH = __dfu_ram_end - __dfu_ram_start
	USBRAM (rw)	: ORIGIN = 0x20004000, LENGTH = 2K
}

SECTIONS
{
	.usbram_bss (NOLOAD) : ALIGN(4)
	{
		__dfu_usbram_bss_start = .;
		*dfu_boot_main*(.bss._ZL7s_SerRx .bss._ZL7s_SerTx .bss._ZL5s_Mgr)
		*dfu_boot_main*(.bss._ZL12s_UartRxFifo)
		. = ALIGN(4);
		__dfu_usbram_bss_end = .;
	} > USBRAM
} INSERT BEFORE .bss;

INCLUDE "gcc_arm_flash.ld"

/* SYSMEMREMAP maps RAM over the first 512 bytes: the code that runs after
 * the remap, the end of DfuTgtStart, has to be above them. */
ASSERT(DfuTgtStart >= 0x200, "DfuTgtStart inside the 512 bytes SYSMEMREMAP maps")
