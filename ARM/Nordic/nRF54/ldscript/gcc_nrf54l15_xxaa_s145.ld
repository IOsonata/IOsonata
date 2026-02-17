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
	FLASH (rx)  : ORIGIN = 0x000000, LENGTH = 0x156C00  /* slot0 only, before storage partition */
	RAM   (rwx) : ORIGIN = 0x20004780, LENGTH = 0x3B880   /* after S145 reserved (0x4780), spans RAM0+RAM1 */
}

/* Default symbols for observer sections that may be empty.
 * PROVIDE only takes effect if nothing else defines the symbol.
 * When a section has entries, the in-section definitions override these.
 * When empty, start == stop so the iteration loop is a no-op. */
PROVIDE(__start_nrf_sdh_stack_evt_observers = 0);
PROVIDE(__stop_nrf_sdh_stack_evt_observers = 0);
PROVIDE(__start_nrf_sdh_soc_evt_observers = 0);
PROVIDE(__stop_nrf_sdh_soc_evt_observers = 0);
PROVIDE(__start_nrf_sdh_ble_evt_observers = 0);
PROVIDE(__stop_nrf_sdh_ble_evt_observers = 0);
PROVIDE(__start_nrf_sdh_state_evt_observers = 0);
PROVIDE(__stop_nrf_sdh_state_evt_observers = 0);

/* === FLASH observer sections ===
 *
 * Output section names match the C identifiers used by TYPE_SECTION_FOREACH
 * so the GNU linker auto-generates __start_<name> / __stop_<name> symbols.
 * Input sections are SORTed by priority suffix (.0 HIGHEST … .4 LOWEST).
 */
SECTIONS
{
	nrf_sdh_stack_evt_observers : ALIGN(4)
	{
		KEEP(*(SORT(nrf_sdh_stack_evt_observers.*)))
	} > FLASH

	nrf_sdh_soc_evt_observers : ALIGN(4)
	{
		KEEP(*(SORT(nrf_sdh_soc_evt_observers.*)))
	} > FLASH

	nrf_sdh_ble_evt_observers : ALIGN(4)
	{
		KEEP(*(SORT(nrf_sdh_ble_evt_observers.*)))
	} > FLASH
} INSERT AFTER .rodata;

/* === RAM observer section ===
 *
 * nrf_sdh_state_evt_observers has a mutable is_busy field so it lives
 * in RAM with initial values loaded from FLASH.
 *
 * Placed between .data and .bss so gcc_arm_flash.ld's startup copy
 * covers it:  __data_size__ = __bss_start__ - __data_start__
 */
SECTIONS
{
	nrf_sdh_state_evt_observers : ALIGN(4)
	{
		KEEP(*(SORT(nrf_sdh_state_evt_observers.*)))
	} > RAM AT> FLASH
} INSERT BEFORE .bss;

INCLUDE "gcc_arm_flash.ld"
