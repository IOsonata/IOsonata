OUTPUT_FORMAT ("elf32-littlearm", "elf32-bigarm", "elf32-littlearm")
SEARCH_DIR(.)
SEARCH_DIR("../../../../ldscript")
SEARCH_DIR("../../../../../ldscript")
SEARCH_DIR("../../../../../../ldscript")
SEARCH_DIR("../../../../../../../ldscript")
SEARCH_DIR("../../IOsonata/ARM/ldscript")
SEARCH_DIR("../../../IOsonata/ARM/ldscript")
SEARCH_DIR("../../../../../IOsonata/ARM/ldscript")
SEARCH_DIR("../../../../../../IOsonata/ARM/ldscript")
SEARCH_DIR("../../IOcomposer/IOsonata/ARM/ldscript")
SEARCH_DIR("../../../IOcomposer/IOsonata/ARM/ldscript")
SEARCH_DIR("../../../../IOcomposer/IOsonata/ARM/ldscript")
SEARCH_DIR("../../../../../IOcomposer/IOsonata/ARM/ldscript")
SEARCH_DIR("../../../../../../IOcomposer/IOsonata/ARM/ldscript")
GROUP(-lgcc -lc)

MEMORY
{
	FLASH (rx) : ORIGIN = 0x00000000, LENGTH = 0x80000
	RAM (rwx) :  ORIGIN = 0x20000000, LENGTH = 0x10000
}

SECTIONS
{
	. = ALIGN(4);
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

SECTIONS
{
	.pwr_mgmt_data :
	{
		PROVIDE(__start_pwr_mgmt_data = .);
		KEEP(*(SORT(.pwr_mgmt_data*)))
		PROVIDE(__stop_pwr_mgmt_data = .);
	} > FLASH
  
	.crypto_data :
	{
	    PROVIDE(__start_crypto_data = .);
	    KEEP(*(SORT(.crypto_data*)))
	    PROVIDE(__stop_crypto_data = .);
	} > FLASH

	.log_const_data :
	{
		PROVIDE(__start_log_const_data = .);
		KEEP(*(.log_const_data))
		PROVIDE(__stop_log_const_data = .);
	} > FLASH
  
	.cli_command :
	{
		PROVIDE(__start_cli_command = .);
		KEEP(*(.cli_command))
		PROVIDE(__stop_cli_command = .);
	} > FLASH
} INSERT AFTER .text

INCLUDE "gcc_arm_flash.ld"
