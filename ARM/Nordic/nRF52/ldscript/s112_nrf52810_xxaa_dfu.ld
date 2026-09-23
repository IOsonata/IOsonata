/* Application with DFU, nRF52810 with MBR and S112.
 *
 * nrf52810_xxaa.ld for an application on the SoftDevice: FLASH is slot 0,
 * from the layout the stage 0 boot shares, and the SoftDevice handler
 * sections are added. An image that outgrows slot 0 fails to link.
 *
 * RAM starts after what S112 takes with the configuration bt_app_nrf52
 * asks for (one link, 3000 byte attribute table); sd_ble_enable reports
 * the exact start it needs if that is ever more.
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
GROUP(-lgcc -lc)

INCLUDE "dfu_layout_nrf52810_s112.ld"

MEMORY
{
	FLASH (rx)	: ORIGIN = __dfu_slot0_start, LENGTH = __dfu_slot0_end - __dfu_slot0_start
	RAM (rwx)	: ORIGIN = 0x20003000, LENGTH = __dfu_flag - 0x20003000
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
