/* Linker script to configure memory regions. */
SEARCH_DIR(.)
SEARCH_DIR("../../../../ldscript")
SEARCH_DIR("../../../../../ldscript")
SEARCH_DIR("../../../../../../ldscript")
SEARCH_DIR("../../../../../../../ldscript")
SEARCH_DIR("../../../../IOsonata/ARM/ldscript")
SEARCH_DIR("../../../../../IOsonata/ARM/ldscript")
SEARCH_DIR("../../../../../../IOsonata/ARM/ldscript")
SEARCH_DIR("../../../../../../../IOsonata/ARM/ldscript")
GROUP(-lgcc -lc)

MEMORY
{
	/* nRF52840 has 1MB Flash (0x100000), 256KB RAM (0x40000) */

	/* App flash ends at 0xFD000, leaving two pages (0xFD000..0xFEFFF) for the
	   BLE bond store (bt_pds NVMC backend) and one page (0xFF000) for the
	   bootloader settings. Keep the nRF52840 BT_PDS_NVMC_REGION_ADDR=0xFD000
	   default in bt_pds_sdc_nrf52.cpp in sync with this. */
	FLASH (rx) : ORIGIN = 0x27000, LENGTH = 0xd6000
	RAM (rwx) :  ORIGIN = 0x20000400, LENGTH = 0x3FC00

	/** Location of non initialized RAM. Non initialized RAM is used for exchanging bond information
	*  from application to bootloader when using buttonless DFU OTA.
	*/
	NOINIT (rwx) :  ORIGIN = 0x20007F80, LENGTH = 0x80

	/** BLE bond store: two flash pages, written by the bt_pds NVMC backend
	    through MPSL timeslots. Reserved here so the application does not place
	    code in it. */
	BT_PDS (rw) : ORIGIN = 0x000FD000, LENGTH = 0x2000

	/** Location of bootloader setting in flash. */
	BOOTLOADER_SETTINGS (rw) : ORIGIN = 0x000FF000, LENGTH = 0x1000
}


SECTIONS
{
	/* Place the bootloader settings page in flash. */
	.bootloaderSettings(NOLOAD) :
	{

	} > BOOTLOADER_SETTINGS

	/* No init RAM section in bootloader. Used for bond information exchange. */
	.noinit(NOLOAD) :
	{

	} > NOINIT
}

SECTIONS
{
	.pwr_mgmt_data :
	{
		PROVIDE(__start_pwr_mgmt_data = .);
		KEEP(*(SORT(.pwr_mgmt_data*)))
		PROVIDE(__stop_pwr_mgmt_data = .);
	} > FLASH
	
	.log_const_data :
	{
	  	PROVIDE(__start_log_const_data = .);
	  	KEEP(*(.log_const_data))
	  	PROVIDE(__stop_log_const_data = .);
	} > FLASH
	
    .crypto_data :
    {
        PROVIDE(__start_crypto_data = .);
        KEEP(*(SORT(.crypto_data*)))
        PROVIDE(__stop_crypto_data = .);
    } > FLASH
	
	.cli_command :
	{
		PROVIDE(__start_cli_command = .);
		KEEP(*(.cli_command))
		PROVIDE(__stop_cli_command = .);
	} > FLASH
	
	.sdh_stack_observers :
	{
		PROVIDE(__start_sdh_stack_observers = .);
		KEEP(*(SORT(.sdh_stack_observers*)))
		PROVIDE(__stop_sdh_stack_observers = .);
	} > FLASH
	
	.sdh_req_observers :
	{
		PROVIDE(__start_sdh_req_observers = .);
		KEEP(*(SORT(.sdh_req_observers*)))
		PROVIDE(__stop_sdh_req_observers = .);
	} > FLASH
	
	.sdh_state_observers :
	{
		PROVIDE(__start_sdh_state_observers = .);
		KEEP(*(SORT(.sdh_state_observers*)))
		PROVIDE(__stop_sdh_state_observers = .);
	} > FLASH
	
	.sdh_ant_observers :
	{
		PROVIDE(__start_sdh_ant_observers = .);
		KEEP(*(SORT(.sdh_ant_observers*)))
		PROVIDE(__stop_sdh_ant_observers = .);
	} > FLASH
	
	.sdh_ble_observers :
	{
		PROVIDE(__start_sdh_ble_observers = .);
		KEEP(*(SORT(.sdh_ble_observers*)))
		PROVIDE(__stop_sdh_ble_observers = .);
	} > FLASH
	
	.sdh_soc_observers :
	{
		PROVIDE(__start_sdh_soc_observers = .);
		KEEP(*(SORT(.sdh_soc_observers*)))
		PROVIDE(__stop_sdh_soc_observers = .);
	} > FLASH
    .nrf_queue :
  {
    PROVIDE(__start_nrf_queue = .);
    KEEP(*(.nrf_queue))
    PROVIDE(__stop_nrf_queue = .);
  } > FLASH
} INSERT AFTER .text

SECTIONS
{
	. = ALIGN(4);
	.svc_data :
	{
		PROVIDE(__start_svc_data = .);
		KEEP(*(.svc_data))
		PROVIDE(__stop_svc_data = .);
	} > RAM AT > FLASH
	
	.fs_data :
	{
		PROVIDE(__start_fs_data = .);
		KEEP(*(.fs_data))
		PROVIDE(__stop_fs_data = .);
	} > RAM AT > FLASH
	
	.log_dynamic_data :
	{
		PROVIDE(__start_log_dynamic_data = .);
		KEEP(*(.log_dynamic_data))
		PROVIDE(__stop_log_dynamic_data = .);
	} > RAM AT > FLASH
	.cli_sorted_cmd_ptrs :
	{
		PROVIDE(__start_cli_sorted_cmd_ptrs = .);
		KEEP(*(.cli_sorted_cmd_ptrs))
		PROVIDE(__stop_cli_sorted_cmd_ptrs = .);
	} > RAM AT > FLASH
} INSERT AFTER .data;

INCLUDE "gcc_arm_flash.ld"
