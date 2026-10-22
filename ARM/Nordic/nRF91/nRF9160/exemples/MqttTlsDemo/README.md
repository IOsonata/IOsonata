# MqttTlsDemo: Nordic Thingy:91 or nRF9160 DK

This project runs the shared `exemples/net/mqtt_generic.cpp` application over
an nRF91 modem TLS 1.2 socket. The setup in `exemples/net/net_demo.cpp` requires
CA-chain and hostname verification before returning its connected
`DeviceIntrf *` to the existing MQTT client. The MQTT implementation and
application use the same injected stream as the plaintext `MqttDemo`.

This is a separate project with its own `src/board.h`. The board header selects
the modem TLS profile, peer and credential tag; it contains no protocol code.
The project also links the public demonstration CA declaration and data in
`exemples/net/net_demo_tls_ca.h` and `net_demo_tls_ca.cpp`.

See the [generic protocol example guide](../../../../../../docs/network-protocol-examples.md#mqtt-over-modem-tls)
for the shared credential setup, verification limits and test procedure.

## Build and connect

Import these IOcomposer projects:

1. `ARM/Nordic/nRF91/nRF9160/lib/ioc`, named `IOsonata_nRF9160`.
2. `ARM/Nordic/nRF91/nRF9160/exemples/MqttTlsDemo/ioc`, named `MqttTlsDemo`.

Build the library first, then the application using matching **Debug** or
**Release** configurations. Refresh/reopen the library project if an older
workspace import has stale source links. Each target has a project named
`MqttTlsDemo`; use separate workspaces when keeping several target variants
imported.

The default console is Thingy:91 UART0, RX P0.19 and TX P0.18, at
**115200 baud, 8N1, no flow control**. Keep the working nRF52840 Connectivity
Bridge or HciController firmware that forwards this UART to USB. This
application does not start a BLE or HCI host. Program the nRF9160 application
using the normal Thingy:91 setup and open the console before resetting it.

For the nRF9160 DK, select `NRF9160_DK` instead of `NORDIC_THINGY91` in
`src/board.h`. The console then uses UART0 RX P0.28 and TX P0.29 through VCOM0.
The timer is RTC0: trigger 0 supplies modem waits and trigger 1 drives the
foreground application pump.

The projects retain `NRF9160_XXAA`, Cortex-M33 Thumb hard-float, GNU C++23,
no exceptions and no RTTI. They link the existing Nordic modem library from
`external/sdk-nrfxlib/nrf_modem/lib/cellular/nrf9160/hard-float` and use
`ARM/Nordic/nRF91/ldscript/nrf9160_xxaa.ld`. Preserve the existing SDK layout
with `external/nrfx` and `external/sdk-nrfxlib` beside IOsonata.

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

## TLS and peer configuration

The supplied `src/board.h` includes `net_demo_tls_ca.h` and selects:

```cpp
#define NET_DEMO_MODEM_TLS 1
#define NET_DEMO_TLS_SEC_TAG 42420
#define NET_DEMO_TLS_CA_CERT g_NetDemoMosquittoCa
#define NET_DEMO_TLS_REPLACE_CA 0
#define MQTT_DEMO_HOST "test.mosquitto.org"
#define MQTT_DEMO_PORT 8883
#define MQTT_DEMO_INTERVAL 10000U
```

Reserve modem credential tag **42420** for this demonstration. At modem
initialization the setup checks that LTE has disconnected, prepares the CA
while the radio is offline, then reconnects LTE. A missing CA is written from
`NET_DEMO_TLS_CA_CERT`. An existing CA at the tag is reused with the default
`NET_DEMO_TLS_REPLACE_CA = 0` when reported present. That query does not
compare CA contents, and a false result can also indicate a listing failure;
setup then attempts the configured CA write. Use only the dedicated tag owned
by this demo. The shared guide records the supplied CA's origin.

Use `NET_DEMO_TLS_REPLACE_CA = 1` only when deliberately replacing the CA in
that dedicated tag. This overwrites its CA entry; it does not delete other
credential types. Return the setting to zero after provisioning to avoid
rewriting it on subsequent modem initializations. With
`NET_DEMO_TLS_CA_CERT = nullptr`, the selected tag must already contain a CA.
Credential preparation runs at startup and modem reinitialization, not every
MQTT reconnect. Setup failures prevent network readiness for this application.

The hostname is used for DNS and server identity verification. Keep
`test.mosquitto.org` for the default public service; substituting its numeric
IP address changes the identity being checked. A successful TLS open precedes
MQTT CONNECT. A rejected TLS connection never falls back to plaintext. The
profile accepts streams over LTE only; it does not enable encrypted UDP,
Ethernet or custom-medium sockets.

Nordic documents that `mfw_nrf9160_1.2.8` does not verify server certificate
expiry. This example therefore claims CA-chain and hostname verification,
without certificate validity-date enforcement. An expired-certificate server
is not a valid expected-rejection test for this profile. The shared guide
links the Nordic references and describes wrong-CA and hostname checks.

The public TLS listener does not require a client certificate. This is server
authentication, not mutual TLS or MQTT account authentication. Public topics
remain accessible to other broker clients; send only the synthetic test data.
The native IOsonata TLS 1.3 external-PSK client is a separate implementation
and is unchanged by this modem profile.

## MQTT check and validation

The application generates a boot-stable `iosnet<16-hex-suffix>` client ID and
subscribes to `iosonata/demo/<16-hex-suffix>/echo` at QoS 1. It checks the
returned topic and payload together with the broker's PUBACK. The first
publication follows subscription; subsequent publications are at least
10 seconds apart. A subscription alone is not a completed echo check.

Record the modem firmware, registration messages, security label, MQTT
subscription and checked counters. The first checked exchange and every
10th are logged. After a connection failure the application closes the
endpoint and retries a fresh TLS/MQTT connection, then subscribes again.
Preserve any native `nRF91 socket ...` diagnostics when reporting a failure.

Run the dedicated host checks from the repository root:

```sh
python3 tests/net/run_mqtt_tls_demo.py --target nrf9160
```

Host validation for this profile is recorded in the shared guide. Host
substitutes do not perform the modem's cryptography, validate its certificate
handling or establish a live broker session. No hardware result or fresh
automated ARM compile/link result is claimed for `MqttTlsDemo`. Earlier
plaintext LTE and native-PSK H753 results apply to their recorded profiles.
