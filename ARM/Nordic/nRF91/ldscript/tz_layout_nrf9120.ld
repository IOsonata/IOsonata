/* TrustZone layout, nRF91x1 (nRF9120 die: nRF9131, nRF9151, nRF9161).
 *
 * Included by the secure stage script (nrf9120_xxaa_spe.ld) and the non
 * secure application script (nrf9120_xxaa_ns.ld), so both agree on where the
 * secure and non secure memory is. The secure stage reads these symbols to
 * program the SPU; no C define repeats an address.
 *
 *   0x00000000 - 0x00007BFF  secure stage                 31 KB
 *   0x00007C00 - 0x00007FFF  non secure callable, reserved 1 KB
 *   0x00008000 - 0x000FFFFF  non secure application       992 KB
 *
 *   0x20000000 - 0x20003FFF  secure RAM                    16 KB
 *   0x20004000 - 0x2003DFFF  non secure application RAM   232 KB
 *   0x2003E000 - 0x2003FFFF  non secure, not linked         8 KB
 *
 * The SPU sets the security of flash in 32 KB regions and of RAM in 8 KB
 * regions, so the non secure start addresses are on those boundaries. All
 * memory from the non secure start to the end of flash or RAM is non secure.
 *
 * The non secure callable area is at the end of the last secure flash
 * region, where the SPU places it. Nothing uses it yet: it is reserved so
 * that secure services called from the application do not move anything.
 *
 * The application RAM stops at the 0x2003E000 page where SystemInit copies
 * FICR for the non secure side at every reset (NRF_FICR_NS in nrf.h). The
 * last 16 bytes of RAM are the DFU recovery request word (__dfu_flag in
 * dfu_layout_nrf9120.ld).
 *
 * The modem shares memory with the application in the lower 128 KB of RAM
 * only. The application script places that memory at the start of its RAM.
 */
__tz_s_flash_start	= 0x00000000;
__tz_nsc_start		= 0x00007C00;
__tz_s_flash_end	= 0x00008000;
__tz_ns_flash_start	= 0x00008000;
__tz_ns_flash_end	= 0x00100000;

__tz_s_ram_start	= 0x20000000;
__tz_s_ram_end		= 0x20004000;
__tz_ns_ram_start	= 0x20004000;
__tz_ns_ram_end		= 0x2003E000;

ASSERT(__tz_ns_flash_start % 0x8000 == 0, "non secure flash must start on a 32 KB SPU region")
ASSERT(__tz_ns_ram_start % 0x2000 == 0, "non secure RAM must start on an 8 KB SPU region")
