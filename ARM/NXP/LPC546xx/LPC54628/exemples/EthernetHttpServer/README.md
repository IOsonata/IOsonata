# EthernetHttpServer: LPCXpresso54628

A small web server. Import `ioc` together with `../../lib/ioc` in IOcomposer. Build the
IOsonata_LPC54628 library and the application in matching configurations. The SDK needs
lwIP in `external/lwip` beside IOsonata; built with lwIP 2.2.0.

The project links the shared `exemples/net/http_server.cpp`. `src/board.h` sets the RMII
pins of the NXP SDK enet examples, the LAN8720A at MDIO address 0, LED1 and a locally
administered MAC address. Set a distinct address for each board on the same network.

Connect the RJ45 jack of the board to a LAN with a DHCP server. The console is Flexcomm 0
on the LPC-Link2 VCOM at 1000000 baud, 8N1. Expected console output: "Waiting for the
network", then "Open http://<address>/".

Open that address in a browser. The page reads `/status` once a second and its button
toggles LED1. From a computer:

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
