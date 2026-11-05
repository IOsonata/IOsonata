# EthernetHttpServer: NUCLEO-H753ZI

A small web server. Import `ioc` together with `../../lib/ioc` in IOcomposer. Build
the H753 library and the application in matching configurations. The SDK needs lwIP
in `external/lwip` beside IOsonata; built with lwIP 2.2.0.

The project links the shared `exemples/net/http_server.cpp`. `src/board.h` sets the
Ethernet pins, the LAN8742A at MDIO address 0, the first LED and a locally
administered MAC address. Set a distinct address for each board on the same network.

Connect the RJ45 jack CN14 to a LAN with a DHCP server. The console is USART3 through
the ST-LINK VCP at 1000000 baud, 8N1. Debug needs semihosting enabled; Release runs
without the debugger. Expected console output: "Waiting for the network", then
"Open http://<address>/".

Open that address in a browser. The page reads `/status` once a second and its button
toggles the LED. From a computer:

```sh
curl http://<address>/status
curl "http://<address>/hello?name=me"
curl -X POST http://<address>/led
```

See the [networking guide](../../../../../../docs/networking.md#small-web-server) for
the routes, the limits and how it was tested. It is plain HTTP without authentication,
for a network that is trusted.

Status: Debug and Release build and link. The example runs on a computer with the real
server and lwIP against `curl`, `tests/net/run_http_tap.py`. No hardware result yet.
