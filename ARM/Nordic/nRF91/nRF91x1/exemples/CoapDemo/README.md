# CoapDemo: nRF9151 DK or nRF9161 DK

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

1. `ARM/Nordic/nRF91/nRF91x1/lib/ioc`, named `IOsonata_nRF91x1`.
2. `ARM/Nordic/nRF91/nRF91x1/exemples/CoapDemo/ioc`, named `CoapDemo`.

Build the library first, then the application using matching **Debug** or
**Release** configurations. Refresh/reopen the library project if an older
workspace import has stale source links. Each target has a project named
`CoapDemo`; use separate workspaces when keeping several target variants
imported.

The nRF9151 DK and nRF9161 DK use UART0 through the interface MCU's
**VCOM0**, RX P0.26 and TX P0.27, at **115200 baud, 8N1, no flow control**.
Program the selected nRF91x1 application target using the normal DK setup,
then reset it with the console open. RTC0 trigger 0 supplies modem waits and
trigger 1 drives the foreground application pump.

The project retains the existing nRF91x1 `LteUdp` target settings:

| Setting | Value |
| --- | --- |
| Target definition | `NRF9120_XXAA` |
| CPU and ABI | Cortex-M33, Thumb, hard-float |
| Linker script | `ARM/Nordic/nRF91/ldscript/nrf9120_xxaa.ld` |
| Modem library | `external/sdk-nrfxlib/nrf_modem/lib/cellular/nrf9120/hard-float` |
| C++ | GNU C++23, no exceptions, no RTTI |

The `nrf9120` names are the existing SDK/compiler selections for this port.
Keep the `external/nrfx` and `external/sdk-nrfxlib` layout used by the other
nRF91x1 examples; startup and modem shared-memory placement come from the
existing library and linker script.

Use the bare-metal library configurations, not the TaktOS variants. The board
selects `CryptoRngNrfInstance()`, whose CryptoCell provider requires secure
access. Keep the existing secure-image configuration and do not define
`NRF_TRUSTZONE_NONSECURE` in either the library or application.

`NET_DEMO_MEDIUM` selects `NET_DEMO_MEDIUM_LTE`. `NET_DEMO_APN` defaults to
`nullptr`; set the SIM provider's APN string if required. `NET_DEMO_RAT` and
`NET_DEMO_RAT_PREF` both select LTE-M. The SIM needs data service, DNS and
outbound access to the chosen peer. The setup code disables requested PSM,
eDRX and RAI for these continuous protocol checks.

The existing nRF91 socket port allows 30 seconds per connection attempt and
then applies the example's 100-millisecond send wait. DNS has its own modem
wait. Failed IPv6-only address attempts can be followed by an explicit IPv4
lookup. Preserve any `nRF91 socket ...` diagnostics with the registration and
application log.

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
python3 tests/net/run_generic_demo.py --protocol coap --target nrf91x1
```

The host checks use substitutes for hardware and do not replace an ARM
compile/link or a board run. New-project validation results are recorded in
the shared guide. Previous nRF9160 and H753 hardware results apply to their
original examples; they do not establish a hardware result for this new
project.
