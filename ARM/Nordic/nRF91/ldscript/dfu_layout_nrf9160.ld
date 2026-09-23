/* DFU flash layout, nRF9160.
 *
 * Included by the stage 0 boot script (dfu_boot_nrf9160.ld) and the
 * application script (nrf9160_xxaa_dfu.ld), so both agree on where
 * everything is. Code reads these symbols through DfuLayoutGet; no C define
 * repeats an address. See docs/architecture/dfu.md.
 *
 *   0x000000 - 0x00BFFF  stage 0 boot            48 KB
 *   0x00C000 - 0x00CFFF  record of slot 0         4 KB
 *   0x00D000 - 0x085FFF  slot 0, application    484 KB
 *   0x086000 - 0x0FFFFF  slot 1, upload         488 KB
 *
 * Slot 1 is larger than slot 0, for the image header, the TLV areas and the
 * trailer that come with a slot 0 sized payload.
 *
 * The image is fetched over LTE by the application into slot 1 through the
 * target layer (no radio stack shares the flash controller on this core),
 * marked pending, and installed by the boot after a reset.
 *
 * The boot is at 0 and runs out of reset. It sets VTOR to slot 0, which is
 * 4 KB aligned.
 *
 * __dfu_flag is the recovery request word, the last 16 bytes of RAM, above
 * the 0x2003E000 page where SystemInit copies FICR for the non secure side
 * at every reset (NRF_FICR_NS in nrf.h): the boot and the application both
 * stop their RAM short of it, so neither links anything there nor clears it
 * at startup.
 */

__dfu_flag			= 0x2003FFF0;
__dfu_boot_start	= 0x00000000;
__dfu_boot_end		= 0x0000C000;
__dfu_rec_start		= 0x0000C000;
__dfu_rec_end		= 0x0000D000;
__dfu_slot0_start	= 0x0000D000;
__dfu_slot0_end		= 0x00086000;
__dfu_slot1_start	= 0x00086000;
__dfu_slot1_end		= 0x00100000;
