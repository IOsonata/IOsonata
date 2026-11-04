/* nRF9160 Linker script to configure memory regions. */

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
  FLASH (rx) : ORIGIN = 0x00000000, LENGTH = 0x100000
  RAM (rwx) :  ORIGIN = 0x20000000, LENGTH = 0x40000
}

/* Memory shared with the modem, first in RAM: the modem reaches the lower
 * 128 KB of RAM only. The Modem library glue puts it in the .modem_shm
 * sections (modem_nrf91.h); without the modem it is empty. It fills whole
 * 8 KB SPU RAM regions, which nRF91ModemInit makes non secure for the modem,
 * so nothing else of the application is in them. Not cleared at startup.
 */
SECTIONS
{
	.modem_shm (NOLOAD) : ALIGN(0x2000)
	{
		__start_modem_shm = .;
		*(SORT_BY_NAME(.modem_shm*))
		. = ALIGN(0x2000);
		__stop_modem_shm = .;
	} > RAM
}

ASSERT(__stop_modem_shm <= 0x20020000, "modem shared memory must be in the lower 128 KB of RAM")

INCLUDE "gcc_arm_flash.ld"
