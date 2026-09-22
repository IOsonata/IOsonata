/* Application with DFU, nRF54LM20 (A and B) with S145.
 *
 * Same as gcc_nrf54lm20a_xxaa_s145.ld with FLASH cut down to slot 0, from the
 * layout the stage 0 boot shares, and RAM stopped short of the recovery
 * request word. An image that outgrows slot 0 fails to link. The stage 0
 * boot sets VTOR to slot 0 before it starts the image.
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

INCLUDE "dfu_layout_nrf54lm20a_s145.ld"

MEMORY
{
	FLASH (rx)  : ORIGIN = __dfu_slot0_start, LENGTH = __dfu_slot0_end - __dfu_slot0_start
	NVM0  (rw)  : ORIGIN = __dfu_nvm0_start, LENGTH = __dfu_nvm_size	/* peer manager */
	NVM1  (rw)  : ORIGIN = __dfu_nvm1_start, LENGTH = __dfu_nvm_size	/* application */
	RAM   (rwx) : ORIGIN = 0x20004780, LENGTH = __dfu_flag - 0x20004780   /* after S145 reserved (0x4780), spans RAM0+RAM1 */
}

/* === FLASH observer sections ===
 *
 * Output section names match the C identifiers used by TYPE_SECTION_FOREACH
 * so the GNU linker auto-generates __start_<name> / __stop_<name> symbols.
 * Input sections are SORTed by priority suffix (.0 HIGHEST ... .4 LOWEST).
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
