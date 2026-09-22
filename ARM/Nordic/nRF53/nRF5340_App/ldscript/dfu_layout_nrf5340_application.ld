/* DFU flash layout, nRF5340 application core.
 *
 * Included by the stage 0 boot script (dfu_boot_nrf5340_application.ld) and
 * the application script (nrf5340_xxaa_application_dfu.ld), so both agree on
 * where everything is. Code reads these symbols through DfuLayoutGet; no C
 * define repeats an address. See docs/architecture/dfu.md.
 *
 *   0x000000 - 0x00BFFF  stage 0 boot            48 KB
 *   0x00C000 - 0x00CFFF  record of slot 0         4 KB
 *   0x00D000 - 0x0FFFFF  slot 0, application    972 KB
 *
 * Wired update only, no slot 1: the boot takes a signed image over the UART
 * straight into slot 0. The application core has no radio; a
 * Bluetooth update goes to the network core.
 *
 * The boot is at 0 and runs out of reset. It sets VTOR to slot 0, which is
 * 4 KB aligned.
 *
 * __dfu_flag is the recovery request word, the last 16 bytes of RAM below
 * 0x2007F000, where SystemInit copies FICR for the non secure side at every
 * reset (NRF_FICR_NS in nrf.h): the boot and the application both stop their
 * RAM short of it, so neither links anything there nor clears it at startup.
 */

__dfu_flag			= 0x2007EFF0;
__dfu_boot_start	= 0x00000000;
__dfu_boot_end		= 0x0000C000;
__dfu_rec_start		= 0x0000C000;
__dfu_rec_end		= 0x0000D000;
__dfu_slot0_start	= 0x0000D000;
__dfu_slot0_end		= 0x00100000;
