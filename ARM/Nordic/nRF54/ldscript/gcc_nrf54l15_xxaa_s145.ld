/* Linker script to configure memory regions.
 *
 * nRF54L15 with SoftDevice S145
 *
 * RRAM layout:
 *   0x000000 - 0x156BFF  slot0_partition (app) — 1371K
 *   0x156C00 - 0x158BFF  storage_partition     — 8K
 *     0x156C00 - 0x157BFF  peer_manager_partition — 4K
 *     0x157C00 - 0x158BFF  storage0_partition     — 4K
 *   0x158C00 - 0x17CBFF  softdevice_partition  — 144K
 *
 * SRAM layout (128K at 0x20000000):
 *   0x20000000 - 0x200017FF  softdevice_static_ram  — 6K
 *   0x20001780 - 0x200047FF  softdevice_dynamic_ram — 12K
 *   0x20004780 - 0x2001FFFF  app_ram                — ~237K (spans into RAM1)
 */

SEARCH_DIR(.)
SEARCH_DIR("../../../../ldscript")
SEARCH_DIR("../../../../../ldscript")
SEARCH_DIR("../../../../../../ldscript")
SEARCH_DIR("../../../../../../../ldscript")
SEARCH_DIR("../../IOsonata/ARM/ldscript")
SEARCH_DIR("../../../IOsonata/ARM/ldscript")
SEARCH_DIR("../../../../IOsonata/ARM/ldscript")
SEARCH_DIR("../../../../../IOsonata/ARM/ldscript")
SEARCH_DIR("../../../../../../IOsonata/ARM/ldscript")
GROUP(-lgcc -lc)

MEMORY
{
	FLASH (rx)  : ORIGIN = 0x00000000, LENGTH = 0x156C00  /* slot0 only, before storage partition */
	RAM   (rwx) : ORIGIN = 0x20004780, LENGTH = 0x3B880   /* after S145 reserved (0x4780), spans RAM0+RAM1 */
}


SECTIONS
{
	/* === ROM observer sections — placed in FLASH === */

	.sdh_observers : ALIGN(4)
	{
		__start_nrf_sdh_stack_evt_observers = .;
		KEEP(*(SORT(nrf_sdh_stack_evt_observers.*)))
		__stop_nrf_sdh_stack_evt_observers = .;

		. = ALIGN(4);
		__start_nrf_sdh_soc_evt_observers = .;
		KEEP(*(SORT(nrf_sdh_soc_evt_observers.*)))
		__stop_nrf_sdh_soc_evt_observers = .;

		. = ALIGN(4);
		__start_nrf_sdh_ble_evt_observers = .;
		KEEP(*(SORT(nrf_sdh_ble_evt_observers.*)))
		__stop_nrf_sdh_ble_evt_observers = .;
	} > FLASH

	/* === RAM observer section — initialized data, loaded from FLASH ===
	 *
	 * nrf_sdh_state_evt_observers contains is_busy field written at runtime
	 * plus handler/context pointers that need initialization from flash.
	 *
	 * Startup code must copy this section from flash to RAM:
	 *   memcpy(&__sdh_state_obs_start, &__sdh_state_obs_load,
	 *          &__sdh_state_obs_end - &__sdh_state_obs_start);
	 *
	 * If gcc_arm_flash.ld's data copy loop uses the full RAM range
	 * (e.g. __data_start__ to __data_end__), this section will be
	 * covered automatically since it precedes .data in the RAM region.
	 */
	.sdh_state_observers : ALIGN(4)
	{
		__sdh_state_obs_start = .;
		__start_nrf_sdh_state_evt_observers = .;
		KEEP(*(SORT(nrf_sdh_state_evt_observers.*)))
		__stop_nrf_sdh_state_evt_observers = .;
		__sdh_state_obs_end = .;
	} > RAM AT> FLASH
	__sdh_state_obs_load = LOADADDR(.sdh_state_observers);
}

INCLUDE "gcc_arm_flash.ld"
