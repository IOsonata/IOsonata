/* NVM1: 768 KB user storage, a USB MSC flash disk. No NVM0, no Bluetooth PDS.
 * MBR + Nordic USB DFU: the region ends where the bootloader starts.
 * Flash reservations are page-aligned (nRF52 4 KB erase pages).
 */
/* Linker script to configure memory regions. */
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
  /* A bootloader doing dual bank updates writes the new image right after the
     current one, so two images must fit here for the update to stay off the
     disk. An image that grows into the disk fails to link. */
  FLASH (rx) : ORIGIN = 0x1000, LENGTH = 0x1F000
  NVM1 (rw) : ORIGIN = 0x00020000, LENGTH = 0xC0000
  RAM (rwx) :  ORIGIN = 0x20000008, LENGTH = 0x3fff8
}

SECTIONS
{
  nvm1 (NOLOAD) :
  {
    __start_nvm1 = .;
    KEEP(*(nvm1))
    . = ORIGIN(NVM1) + LENGTH(NVM1);
    __stop_nvm1 = .;
  } > NVM1
}

SECTIONS
{
  . = ALIGN(4);
  .mem_section_dummy_ram :
  {
  }
  .log_dynamic_data :
  {
    PROVIDE(__start_log_dynamic_data = .);
    KEEP(*(SORT(.log_dynamic_data*)))
    PROVIDE(__stop_log_dynamic_data = .);
  } > RAM AT > FLASH
  .log_filter_data :
  {
    PROVIDE(__start_log_filter_data = .);
    KEEP(*(SORT(.log_filter_data*)))
    PROVIDE(__stop_log_filter_data = .);
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
  .mem_section_dummy_rom :
  {
  }
  .log_const_data :
  {
    PROVIDE(__start_log_const_data = .);
    KEEP(*(SORT(.log_const_data*)))
    PROVIDE(__stop_log_const_data = .);
  } > FLASH
  .log_backends :
  {
    PROVIDE(__start_log_backends = .);
    KEEP(*(SORT(.log_backends*)))
    PROVIDE(__stop_log_backends = .);
  } > FLASH
    .cli_command :
  {
    PROVIDE(__start_cli_command = .);
    KEEP(*(.cli_command))
    PROVIDE(__stop_cli_command = .);
  } > FLASH
  .pwr_mgmt_data :
  {
    PROVIDE(__start_pwr_mgmt_data = .);
    KEEP(*(SORT(.pwr_mgmt_data*)))
    PROVIDE(__stop_pwr_mgmt_data = .);
  } > FLASH
    .nrf_queue :
  {
    PROVIDE(__start_nrf_queue = .);
    KEEP(*(.nrf_queue))
    PROVIDE(__stop_nrf_queue = .);
  } > FLASH
    .nrf_balloc :
  {
    PROVIDE(__start_nrf_balloc = .);
    KEEP(*(.nrf_balloc))
    PROVIDE(__stop_nrf_balloc = .);
  } > FLASH

} INSERT AFTER .text


INCLUDE "gcc_arm_flash.ld"
