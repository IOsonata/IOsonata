# EthernetTcpDemo: NUCLEO-H753ZI

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

NUCLEO-H753ZI hardware validation on October 21, 2026 confirmed TCP echo,
reconnection after restarting the host server, and DHCP/TCP recovery after
unplugging and reconnecting the cable without resetting the board.

A binary integrity run passed 6000 messages, 4445400 bytes each way in
120068 ms, including 4096-byte messages, with no reported corruption or timeout.
This is a paced integrity test, not a maximum-throughput measurement.

Set `NET_DEMO_HTTP_TEST` to `1` in `src/board.h` for a public DNS/HTTP check.
It selects example.com:80 without a local echo server. Hardware returned
`HTTP/1.1 200 OK` and `DNS/HTTP PASS` on October 21, 2026. This validates
plaintext HTTP connectivity, not TLS. Set the flag back to `0` for echo tests.
