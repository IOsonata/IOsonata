/* Non secure application, nRF91x1 (nRF9120 die: nRF9131, nRF9151, nRF9161).
 *
 * Runs after the secure stage, from the non secure flash and RAM of the
 * layout. Build it with a non secure configuration of the library
 * (NRF_TRUSTZONE_NONSECURE). The secure stage points the non secure VTOR at
 * the start of FLASH and takes the stack pointer and reset handler from there.
 *
 * The memory shared with the modem goes first in RAM: the modem reaches the
 * lower 128 KB of RAM only. The Modem library glue puts its shared memory in
 * the .modem_shm sections (see modem_nrf91.h). Shared memory nothing refers
 * to is dropped. The section is not cleared at startup; the glue initializes
 * what the library needs.
 */

SEARCH_DIR(.)
SEARCH_DIR("../../../../ldscript")
SEARCH_DIR("../../../../../ldscript")
SEARCH_DIR("../../../../../../ldscript")
SEARCH_DIR("../../../../../../../ldscript")
SEARCH_DIR("../../../../../../../../ldscript")
SEARCH_DIR("../../IOsonata/ARM/ldscript")
SEARCH_DIR("../../../IOsonata/ARM/ldscript")
SEARCH_DIR("../../../../IOsonata/ARM/ldscript")
SEARCH_DIR("../../../../../IOsonata/ARM/ldscript")
SEARCH_DIR("../../../../../../IOsonata/ARM/ldscript")
SEARCH_DIR("../../../../../../../IOsonata/ARM/ldscript")
GROUP(-lgcc -lc)

INCLUDE "tz_layout_nrf9120.ld"

MEMORY
{
	FLASH (rx)	: ORIGIN = __tz_ns_flash_start, LENGTH = __tz_ns_flash_end - __tz_ns_flash_start
	RAM (rwx)	: ORIGIN = __tz_ns_ram_start, LENGTH = __tz_ns_ram_end - __tz_ns_ram_start
}

SECTIONS
{
	.modem_shm (NOLOAD) : ALIGN(4)
	{
		__start_modem_shm = .;
		*(SORT_BY_NAME(.modem_shm*))
		. = ALIGN(4);
		__stop_modem_shm = .;
	} > RAM
}

/* End of the RAM the modem reaches */
__modem_shm_limit = 0x20020000;

ASSERT(__stop_modem_shm <= __modem_shm_limit, "modem shared memory must be in the lower 128 KB of RAM")

INCLUDE "gcc_arm_flash.ld"
