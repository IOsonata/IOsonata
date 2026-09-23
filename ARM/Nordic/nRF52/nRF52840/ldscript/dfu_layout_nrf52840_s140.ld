/* DFU flash layout, nRF52840 with MBR and S140 7.x.
 *
 * Included by the stage 0 boot script (dfu_boot_nrf52840_s140.ld) and the
 * application script (s140_nrf52840_xxaa_dfu.ld), so both agree on where
 * everything is. Code reads these symbols through DfuLayoutGet; no C define
 * repeats an address. See docs/architecture/dfu.md.
 *
 *   0x000000 - 0x000FFF  MBR
 *   0x001000 - 0x026FFF  S140
 *   0x027000 - 0x08AFFF  slot 0, application   400 KB
 *   0x08B000 - 0x0F0FFF  slot 1, upload        408 KB, trailer at the end
 *   0x0F1000 - 0x0F1FFF  NVM0, peer manager      4 KB
 *   0x0F2000 - 0x0F2FFF  NVM1, application       4 KB
 *   0x0F3000 - 0x0F3FFF  record of slot 0        4 KB
 *   0x0F4000 - 0x0FFFFF  stage 0 boot           48 KB, UICR.NRFFW[0]
 *
 * Slot 0 starts at the end of the SoftDevice: the SoftDevice starts the
 * application there and forwards its interrupts there.
 *
 * Slot 1 is one unit larger than slot 0, for the image header, the TLV areas
 * and the trailer that come with a slot 0 sized payload.
 *
 * __dfu_flag is the recovery request word, the last 16 bytes of RAM: the
 * boot and the application both stop their RAM short of it, so neither
 * links anything there nor clears it at startup.
 */

__dfu_flag			= 0x2003FFF0;
__dfu_slot0_start	= 0x00027000;
__dfu_slot0_end		= 0x0008B000;
__dfu_slot1_start	= 0x0008B000;
__dfu_slot1_end		= 0x000F1000;
__dfu_nvm0_start	= 0x000F1000;
__dfu_nvm1_start	= 0x000F2000;
__dfu_nvm_size		= 0x00001000;
__dfu_rec_start		= 0x000F3000;
__dfu_rec_end		= 0x000F4000;
__dfu_boot_start	= 0x000F4000;
__dfu_boot_end		= 0x00100000;
