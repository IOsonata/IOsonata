/* DFU flash layout, nRF52832 xxAA with MBR and S132 7.x.
 *
 * Included by the stage 0 boot script (dfu_boot_nrf52832_s132.ld) and the
 * application script (s132_nrf52832_xxaa_dfu.ld), so both agree on where
 * everything is. Code reads these symbols through DfuLayoutGet; no C define
 * repeats an address. See docs/architecture/dfu.md.
 *
 *   0x000000 - 0x000FFF  MBR
 *   0x001000 - 0x025FFF  S132
 *   0x026000 - 0x04AFFF  slot 0, application   148 KB
 *   0x04B000 - 0x070FFF  slot 1, upload        152 KB, trailer at the end
 *   0x071000 - 0x071FFF  NVM0, peer manager      4 KB
 *   0x072000 - 0x072FFF  NVM1, application       4 KB
 *   0x073000 - 0x073FFF  record of slot 0        4 KB
 *   0x074000 - 0x07FFFF  stage 0 boot           48 KB, UICR.NRFFW[0]
 *
 * Slot 0 starts at the end of the SoftDevice: the SoftDevice starts the
 * application there and forwards its interrupts there.
 *
 * Slot 1 is one page larger than slot 0, for the image header, the TLV areas
 * and the trailer that come with a slot 0 sized payload.
 *
 * The xxAB part (256 KB) has no room for two slots.
 *
 * __dfu_flag is the recovery request word, the last 16 bytes of RAM: the
 * boot and the application both stop their RAM short of it, so neither
 * links anything there nor clears it at startup.
 */

__dfu_flag			= 0x2000FFF0;
__dfu_slot0_start	= 0x00026000;
__dfu_slot0_end		= 0x0004B000;
__dfu_slot1_start	= 0x0004B000;
__dfu_slot1_end		= 0x00071000;
__dfu_nvm0_start	= 0x00071000;
__dfu_nvm1_start	= 0x00072000;
__dfu_nvm_size		= 0x00001000;
__dfu_rec_start		= 0x00073000;
__dfu_rec_end		= 0x00074000;
__dfu_boot_start	= 0x00074000;
__dfu_boot_end		= 0x00080000;
