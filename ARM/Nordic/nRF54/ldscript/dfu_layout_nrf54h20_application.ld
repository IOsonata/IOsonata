/* DFU MRAM layout, nRF54H20 application core.
 *
 * Included by the stage 0 boot script (dfu_boot_nrf54h20_application.ld) and
 * the application script (nrf54h20_xxaa_application_dfu.ld), so both agree
 * on where everything is. Code reads these symbols through DfuLayoutGet; no
 * C define repeats an address. See docs/architecture/dfu.md.
 *
 *   0x0E0A0000 - 0x0E0A9FFF  stage 0 boot            40 KB
 *   0x0E0AA000 - 0x0E0AAFFF  record of slot 0         4 KB
 *   0x0E0AB000 - 0x0E0DFFFF  slot 0, application    212 KB
 *
 * Wired update only, no slot 1: the boot takes a signed image over the UART
 * straight into slot 0. The application core has no radio; a
 * Bluetooth update goes to the radio core.
 *
 * The regions are those of the application core partition in
 * nrf54h20_xxaa_application.ld. The secure domain firmware decides where
 * this core starts and what it may write: the layout assumes it starts the
 * core at the start of its partition, where the boot is, and lets it write
 * the whole partition.
 *
 * MRAM has no erase and a 16 byte write unit; the target layer writes ones
 * in 2 KB units. Regions are kept on 4 KB boundaries, the MRAM page, and
 * slot 0 on more than the 2 KB its 500 entry vector table needs.
 *
 * The boot slot is the smallest number of 4 KB pages the boot fits in, from
 * its -Os size on the nRF5340 application core, the same Cortex-M33 code:
 * this library has no UART port to link it with yet.
 *
 * __dfu_flag is the recovery request word, the last 16 bytes of the core's
 * local RAM: the boot and the application both stop their RAM short of it,
 * so neither links anything there nor clears it at startup.
 */

__dfu_flag			= 0x22007FF0;
__dfu_boot_start	= 0x0E0A0000;
__dfu_boot_end		= 0x0E0AA000;
__dfu_rec_start		= 0x0E0AA000;
__dfu_rec_end		= 0x0E0AB000;
__dfu_slot0_start	= 0x0E0AB000;
__dfu_slot0_end		= 0x0E0E0000;
