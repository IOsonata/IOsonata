# EthernetDemo: LPCXpresso54628

Import `ioc` together with `../../lib/ioc` in IOcomposer. Build the
IOsonata_LPC54628 library and the application in matching configurations. The
SDK needs lwIP in `external/lwip` beside IOsonata; built with lwIP 2.2.0.

The project links the shared `exemples/net/ethernet_demo.cpp`, the same
application as the NUCLEO-H753ZI EthernetDemo: DHCP, then it answers ping and
reports the link and the address once a second. `src/board.h` sets the RMII
pins of the NXP SDK enet examples, the LAN8720A at MDIO address 0 and a
locally administered MAC address (02:00:00:54:62:80). Set a distinct address
for each board on the same network. The driver is
`ARM/NXP/LPC546xx/src/ethernet_lpc546xx.cpp`.

The console is Flexcomm 0 on the LPC-Link2 VCOM at 1000000 baud, 8N1. The
startup line names the LAN8742A of the H753 board; the LPCXpresso54628 has a
LAN8720A, both are supported by the driver.

Connect the RJ45 jack of the board to a LAN with a DHCP server. Expected
console output: "Waiting for Ethernet link and DHCP", then the link speed and
the address given by DHCP. Then `ping` the address from a computer on the
same LAN. "Ethernet initialization failed" means the ENET DMA reset did not
end (no 50 MHz reference clock from the PHY on P4_14) or no LAN8720A answered
on MDIO.

See [Ethernet bring-up and limits](../../../../../../docs/networking.md) for
the DHCP and ping procedure.

Status: Debug and Release build and link. The driver passes the host test
`tests/lpc/lpc546xx/ethernet_test.cpp`. No hardware result yet.
