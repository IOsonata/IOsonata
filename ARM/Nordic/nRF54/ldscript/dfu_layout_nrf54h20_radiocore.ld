/* DFU MRAM layout, nRF54H20 radio core.
 *
 * Included by the stage 0 boot script (dfu_boot_nrf54h20_radiocore.ld) and
 * the application script (nrf54h20_xxaa_radiocore_dfu.ld), so both agree on
 * where everything is. Code reads these symbols through DfuLayoutGet; no C
 * define repeats an address. See docs/architecture/dfu.md.
 *
 *   0x0E120000 - 0x0E12BFFF  stage 0 boot            48 KB
 *   0x0E12C000 - 0x0E12CFFF  record of slot 0         4 KB
 *   0x0E12D000 - 0x0E165FFF  slot 0, application    228 KB
 *   0x0E166000 - 0x0E19FFFF  slot 1, upload         232 KB
 *
 * Slot 1 is larger than slot 0, for the image header, the TLV areas and the
 * trailer that come with a slot 0 sized payload.
 *
 * The regions are those of the radio core partition in
 * nrf54h20_xxaa_radiocore.ld. The secure domain firmware decides where this
 * core starts and what it may write: the layout assumes it starts the core
 * at the start of its partition, where the boot is, and lets it write the
 * whole partition.
 *
 * MRAM has no erase and a 16 byte write unit; the target layer writes ones
 * in 2 KB units. Regions are kept on 4 KB boundaries, the MRAM page.
 *
 * There is no NvmIntrf port for this core: the application writes slot 1
 * through the target layer, with DFU_SLOT1_TGT defined (see
 * exemples/bluetooth/ble_ota.cpp).
 *
 * __dfu_flag is the recovery request word, the last 16 bytes of the core's
 * RAM region: the boot and the application both stop their RAM short of it,
 * so neither links anything there nor clears it at startup.
 */

__dfu_flag			= 0x2300FFF0;
__dfu_boot_start	= 0x0E120000;
__dfu_boot_end		= 0x0E12C000;
__dfu_rec_start		= 0x0E12C000;
__dfu_rec_end		= 0x0E12D000;
__dfu_slot0_start	= 0x0E12D000;
__dfu_slot0_end		= 0x0E166000;
__dfu_slot1_start	= 0x0E166000;
__dfu_slot1_end		= 0x0E1A0000;
