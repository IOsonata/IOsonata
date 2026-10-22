# CoapDemo: NUCLEO-H753ZI

This project runs the new shared CoAP application in
`exemples/net/coap_generic.cpp`. It injects a connected `DeviceIntrf *` into the existing
IOsonata protocol client. `exemples/net/net_demo.cpp` initializes the selected
medium, console, timer and RNG; this project's `src/board.h` contains
configuration only. The same protocol application is used by all three target
families.

See the [generic protocol example guide](../../../../../../docs/network-protocol-examples.md) for transport selection,
public peers, custom interfaces and the shared checks. The earlier LTE and
Ethernet examples remain separate projects.

## Build and connect

Import these IOcomposer projects:

1. `ARM/ST/STM32H7xx/STM32H753/lib/ioc`, named `IOsonata_STM32H753`.
2. `ARM/ST/STM32H7xx/STM32H753/exemples/CoapDemo/ioc`, named `CoapDemo`.

Build the library first, then the application using matching **Debug** or
**Release** configurations. Refresh/reopen the library project if an older
workspace import has stale source links. Each target has a project named
`CoapDemo`; use separate workspaces when keeping several target variants
imported.

Connect CN14 to a DHCP-enabled Ethernet LAN. The console is USART3 through
ST-LINK VCP at **1000000 baud, 8N1, no flow control**. Debug requires
semihosting enabled; Release can run without the debugger. Both configurations
use the existing `gcc_stm32h753xx.ld` linker script. Its `.eth_dma` section is
in AXI SRAM at `0x24000000`; Ethernet DMA cannot access DTCM.

Keep the SDK's `external/lwip` checkout beside IOsonata. The existing H753
library supplies the MAC/DMA driver and shared lwIP socket implementation.
The target configuration retains `STM32H753xx`, GNU C++23, no exceptions and
no RTTI.

`NET_DEMO_MEDIUM` selects `NET_DEMO_MEDIUM_ETHERNET`. The board uses LAN8742A
RMII with PHY address 0 and TXD1 on **PB13**, not PG14. Assign a distinct
unicast `ETHERNET_MAC_ADDR` to every board. `NET_DEMO_DHCP` defaults to `true`.
For a static address, set it to `false` and configure `NET_DEMO_IP_ADDR`,
`NET_DEMO_NETMASK`, `NET_DEMO_GATEWAY` and `NET_DEMO_DNS` for the attached LAN.
Internet endpoints also require working routing and DNS.

The board selects the existing `CryptoRngStm32Instance()` provider. Timer
DevNo 8 and the shared IP service retain the established H753 example setup;
the foreground protocol pump runs every 20 milliseconds.

## Peer and protocol configuration

Edit the settings in `src/board.h`, then rebuild the application.

The defaults are a read-only confirmable GET to `coap.me:5683`, resource
`/test`, with `COAP_DEMO_INTERVAL = 10000U`. A success is a 2.05 Content
response with at most 1024 payload bytes; public content can change, so its
body is not compared against a fixed string. The test uses plaintext UDP,
with no DTLS or OSCORE.

For full method, binary-echo and separate-response checks, set
`COAP_DEMO_PUBLIC_TEST` to `0` and `COAP_DEMO_HOST` to the reachable address
of your `Python/coap_server.py` instance. The shared guide describes all eight
steps and the helper setup. `COAP_DEMO_PROBING_RATE = 0U` selects the library's
default one-byte/second rate for unanswered peers; recovery after large
unanswered requests can therefore take longer than the minimum retry delay.

Changing `NET_DEMO_MEDIUM` also requires that the selected implementation and
its board configuration are available. It is not a runtime network selector
for the current per-MCU socket libraries. A custom Wi-Fi or other interface
can use the hooks described in the shared guide; this repository does not yet
provide a functioning Wi-Fi transport for these projects.

## Validation

Run the new host checks from the repository root:

```sh
python3 tests/net/run_generic_demo.py --protocol coap --target stm32h753
```

The host checks use substitutes for hardware and do not replace an ARM
compile/link or a board run. New-project validation results are recorded in
the shared guide. Previous nRF9160 and H753 hardware results apply to their
original examples; they do not establish a hardware result for this new
project.
