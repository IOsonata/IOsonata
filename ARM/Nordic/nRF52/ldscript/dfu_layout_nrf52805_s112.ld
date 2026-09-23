/* DFU flash layout, nRF52805 with MBR and S112 7.x.
 *
 * Included by the stage 0 boot script (dfu_boot_nrf52805_s112.ld) and the
 * application script (s112_nrf52805_xxaa_dfu.ld), so both agree on where
 * everything is. Code reads these symbols through DfuLayoutGet; no C define
 * repeats an address. See docs/architecture/dfu.md.
 *
 *   0x000000 - 0x000FFF  MBR
 *   0x001000 - 0x018FFF  S112                   96 KB
 *   0x019000 - 0x025FFF  slot 0, application    52 KB
 *   0x026000 - 0x026FFF  record of slot 0        4 KB
 *   0x027000 - 0x02FFFF  stage 0 boot           36 KB, UICR.NRFFW[0]
 *
 * Wired update only: 192 KB holds no second slot next to the SoftDevice and
 * an application, so there is no slot 1. The boot takes a signed image over
 * the UART straight into slot 0.
 *
 * Slot 0 starts at the end of the SoftDevice: the SoftDevice starts the
 * application there and forwards its interrupts there.
 *
 * The boot slot is the smallest number of pages the boot fits in, measured
 * at -Os.
 *
 * __dfu_flag is the recovery request word, the last 16 bytes of RAM: the
 * boot and the application both stop their RAM short of it, so neither
 * links anything there nor clears it at startup.
 */

__dfu_flag			= 0x20005FF0;
__dfu_slot0_start	= 0x00019000;
__dfu_slot0_end		= 0x00026000;
__dfu_rec_start		= 0x00026000;
__dfu_rec_end		= 0x00027000;
__dfu_boot_start	= 0x00027000;
__dfu_boot_end		= 0x00030000;
