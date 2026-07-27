/* Linker script to configure memory regions.
 *
 * nRF54L15 with SoftDevice S145
 *
 * RRAM layout:
 *   0x000000 - 0x1587FF  slot0_partition (app) — 1371K
 *   0x158800 - 0x15a7FF  storage_partition     — 8K
 *     0x158800 - 0x1597FF  peer_manager_partition — 4K
 *     0x159800 - 0x15a7FF  storage0_partition     — 4K
 *   0x15a800 - 0x17CBFF  softdevice_partition  — 137K
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
	FLASH (rx)  : ORIGIN = 0x000000, LENGTH = 0x158800  /* slot0 only, stops at the store */
	/* The two storage partitions above, named so the linker owns the
	   layout: FLASH stops short of them, so an image that grows into a
	   store fails to link. NvmRegionAddr reads the sections below, so
	   nothing states the addresses twice. 1 KB RRAM sectors. */
	NVM0  (rw)  : ORIGIN = 0x158800, LENGTH = 0x1000    /* peer manager */
	NVM1  (rw)  : ORIGIN = 0x159800, LENGTH = 0x1000    /* application */
	RAM   (rwx) : ORIGIN = 0x20004900, LENGTH = 0x3B700   /* after S145 reserved (0x4780), spans RAM0+RAM1 */
}

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
		__start_nrf_sdh_stack_evt_observers = .;
		KEEP(*(SORT(nrf_sdh_stack_evt_observers.*)))
		__stop_nrf_sdh_stack_evt_observers = .;
	} > FLASH

	nrf_sdh_soc_evt_observers : ALIGN(4)
	{
		__start_nrf_sdh_soc_evt_observers = .;
		KEEP(*(SORT(nrf_sdh_soc_evt_observers.*)))
		__stop_nrf_sdh_soc_evt_observers = .;
	} > FLASH

	nrf_sdh_ble_evt_observers : ALIGN(4)
	{
		__start_nrf_sdh_ble_evt_observers = .;
		KEEP(*(SORT(nrf_sdh_ble_evt_observers.*)))
		__stop_nrf_sdh_ble_evt_observers = .;
	} > FLASH

	nrf_sdh_state_evt_observers : ALIGN(4)
	{
		__start_nrf_sdh_state_evt_observers = .;
		KEEP(*(SORT(nrf_sdh_state_evt_observers.*)))
		__stop_nrf_sdh_state_evt_observers = .;
	} > FLASH
} INSERT AFTER .rodata;

/* === The storage regions ===
 *
 * Same shape as the observer sections above: the bounds arrive as
 * __start_ and __stop_ symbols, so the size is the distance between them
 * and the length is stated once, in the MEMORY line. NOLOAD because
 * nothing is programmed here at flash time; a store writes it at run
 * time. KEEP lets a variable be placed in one if anything ever wants to.
 */
SECTIONS
{
	nvm0 (NOLOAD) :
	{
		__start_nvm0 = .;
		KEEP(*(nvm0))
		. = ORIGIN(NVM0) + LENGTH(NVM0);
		__stop_nvm0 = .;
	} > NVM0

	nvm1 (NOLOAD) :
	{
		__start_nvm1 = .;
		KEEP(*(nvm1))
		. = ORIGIN(NVM1) + LENGTH(NVM1);
		__stop_nvm1 = .;
	} > NVM1
}

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
