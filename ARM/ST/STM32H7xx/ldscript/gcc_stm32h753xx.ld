SEARCH_DIR(.)
SEARCH_DIR("../../../ldscript")
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

/* Data and stack in DTCM; DMA buffers in AXI SRAM. */
MEMORY
{
	FLASH (rx) : ORIGIN = 0x08000000, LENGTH = 2M
	RAM (rwx) : ORIGIN = 0x20000000, LENGTH = 128K
	RAM4 (rwx) : ORIGIN = 0x38000000, LENGTH = 64K
	RAM2 (rwx) : ORIGIN = 0x24000000, LENGTH = 512K
}
SECTIONS
{
	.bus_dma (NOLOAD) : ALIGN(32)
	{
		*(.bus_dma)
	} > RAM4
	.uart_dma (NOLOAD) : ALIGN(32)
	{
		*(.uart_dma)
	} > RAM2
	.eth_dma (NOLOAD) : ALIGN(32)
	{
		*(.eth_dma)
	} > RAM2
}
INCLUDE "gcc_arm_flash.ld"

