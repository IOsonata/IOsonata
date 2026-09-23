/* DFU flash layout, nRF5340 network core.
 *
 * Included by the stage 0 boot script (dfu_boot_nrf5340_network.ld) and the
 * application script (nrf5340_xxaa_network_dfu.ld), so both agree on where
 * everything is. Code reads these symbols through DfuLayoutGet; no C define
 * repeats an address. See docs/architecture/dfu.md.
 *
 *   0x01000000 - 0x01008FFF  stage 0 boot            36 KB
 *   0x01009000 - 0x01009FFF  record of slot 0         4 KB
 *   0x0100A000 - 0x010247FF  slot 0, application    106 KB
 *   0x01024800 - 0x0103FFFF  slot 1, upload         110 KB
 *
 * Slot 1 is larger than slot 0, for the image header, the TLV areas and the
 * trailer that come with a slot 0 sized payload.
 *
 * The network core starts at 0x01000000, where the boot is. The boot, at
 * -Os, takes 33.6 KB; its slot is 36 KB, 2 KB more than the smallest
 * number of pages it fits in, so a slightly larger boot still fits a layout
 * already in the field. The slots get the rest of the 256 KB.
 *
 * There is no NvmIntrf port for this core: the application writes slot 1
 * through the target layer, with DFU_SLOT1_TGT defined (see
 * exemples/bluetooth/ble_ota.cpp).
 *
 * __dfu_flag is the recovery request word, the last 16 bytes of the network
 * core RAM: the boot and the application both stop their RAM short of it, so
 * neither links anything there nor clears it at startup.
 */

__dfu_flag			= 0x2100FFF0;
__dfu_boot_start	= 0x01000000;
__dfu_boot_end		= 0x01009000;
__dfu_rec_start		= 0x01009000;
__dfu_rec_end		= 0x0100A000;
__dfu_slot0_start	= 0x0100A000;
__dfu_slot0_end		= 0x01024800;
__dfu_slot1_start	= 0x01024800;
__dfu_slot1_end		= 0x01040000;
