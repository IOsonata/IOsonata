# EthernetDemo: NUCLEO-H753ZI

Import `ioc` together with `../../lib/ioc` in IOcomposer. Build the H753
library and application in matching configurations. The SDK needs lwIP in
`external/lwip` beside IOsonata; validated with lwIP 2.2.0.

The console uses USART3 through ST-LINK VCP at 1000000 baud, 8N1.
Connect the onboard RJ45 CN14 to a DHCP LAN. Set a unique demonstration MAC
address in `src/board.h`. Debug requires semihosting enabled; Release can run
without the debugger.

See [Ethernet bring-up and limits](../../../../../../docs/networking.md)
for DHCP/ping and UDP/TCP echo instructions. Socket examples need
`NET_DEMO_HOST` set to the computer's LAN address and `Python/net_echo.py`
running on port 2469. EthernetDemo only runs DHCP and responds to ping.

Builds and simulations pass. Physical Ethernet testing remains pending.
