OUTPUT_ARCH( "riscv" )

SEARCH_DIR(.)
SEARCH_DIR("../../../ldscript")
SEARCH_DIR("../../../../ldscript")
SEARCH_DIR("../../../../../ldscript")
SEARCH_DIR("../../../../../../ldscript")
SEARCH_DIR("../../../../../../../ldscript")
SEARCH_DIR("../../../../../../../../ldscript")
SEARCH_DIR("../../IOsonata/RISCV/ldscript")
SEARCH_DIR("../../../IOsonata/RISCV/ldscript")
SEARCH_DIR("../../../../IOsonata/RISCV/ldscript")
SEARCH_DIR("../../../../../IOsonata/RISCV/ldscript")
SEARCH_DIR("../../../../../../IOsonata/RISCV/ldscript")
SEARCH_DIR("../../../../../../../IOsonata/RISCV/ldscript")
GROUP( -lstdc++ -lgcc -lc)

MEMORY
{
  /* Reserve 0x40800000..0x40807FFF for cache before executable SRAM. */
  IRAM (RWX) : ORIGIN = 0x40808000, LENGTH = 0x8000

  /* Use the ESP-IDF v5.5.1 application SRAM limit. The upper SRAM
   * contains ROM state, including the cache function table at 0x4087FFC8.
   * The application stack must not start at the physical SRAM end. */
  RAM  (RW)  : ORIGIN = 0x40810000, LENGTH = 0x5E610

  /* External Flash -- XIP-mapped via cache */
  FLASH (RX) : ORIGIN = 0x42000000, LENGTH = 0x400000
}

/* Direct Boot: flash offset 0 contains the ROM magic; entry follows it. */
RESET_VECTOR = 0x42000008;
__global_pointer$ = ORIGIN(RAM) + 0x800;

SECTIONS
{
	.dboot_hdr ORIGIN(FLASH) :
	{
		LONG(0xaedb041d);
		LONG(0xaedb041d);
	} > FLASH
}

/* Common sections keep .iram.text.vectors first and aligned to 256 bytes,
 * copy initialized RAM from the unified C6 flash mapping, and place .reset. */
INCLUDE "gcc_riscv_flash.ld"

/* ESP-IDF v5.5.1 ROM ets_update_cpu_frequency, argument in MHz. */
PROVIDE(Esp32RomClockUpdate = 0x40000048);
/* Cache_Suspend_ICache and Cache_Resume_ICache, ESP-IDF v5.5.1 ROM table. */
PROVIDE(Esp32RomCacheSuspend = 0x40000698);
PROVIDE(Esp32RomCacheResume = 0x4000069c);

ASSERT(__StackTop <= 0x4086E610, "ESP32-C6 stack overlaps reserved upper SRAM")
