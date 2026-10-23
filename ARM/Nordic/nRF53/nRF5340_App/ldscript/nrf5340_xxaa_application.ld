/* Linker script to configure memory regions. */

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
GROUP(-lgcc -lc)

MEMORY
{
	FLASH (rx) : ORIGIN = 0x00000000, LENGTH = 0x100000
	EXTFLASH (rx) : ORIGIN = 0x10000000, LENGTH = 0x8000000
	RAM (rwx) : ORIGIN = 0x20000000, LENGTH = 0x40000
	RAM1 (rwx) : ORIGIN = 0x20040000, LENGTH = 0x30000
	/* Shared with the network core: the HCI packet rings of the IpcIntrf
	   link to the HciController running there. The network core owns and
	   formats the block, this core attaches to it (bt_hci_ctlr_ipc.cpp).
	   The address and size are what the HciController network core build
	   expects. */
	IPC0 (rw) : ORIGIN = 0x20070000, LENGTH = 0x10000
}

SECTIONS
{
	/* The region as a section, so the reservation shows up in the map file
	   and the bounds arrive as __start_ and __stop_ symbols, the way the NVM
	   regions do. Nothing is placed in it by this image. */
	ipc0 (NOLOAD) :
	{
		__start_ipc0 = .;
		KEEP(*(ipc0))
		. = ORIGIN(IPC0) + LENGTH(IPC0);
		__stop_ipc0 = .;
	} > IPC0
}

INCLUDE "gcc_arm_flash.ld"
