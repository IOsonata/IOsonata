# MqttTlsDemo: nRF9151 DK or nRF9161 DK

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

1. `ARM/Nordic/nRF91/nRF91x1/lib/ioc`, named `IOsonata_nRF91x1`.
2. `ARM/Nordic/nRF91/nRF91x1/exemples/MqttTlsDemo/ioc`, named `MqttTlsDemo`.

Build the library first, then the application using matching **Debug** or
**Release** configurations. Refresh/reopen the library project if an older
workspace import has stale source links. Each target has a project named
`MqttTlsDemo`; use separate workspaces when keeping several target variants
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

Nordic documents that `mfw_nrf91x1_2.0.1` does not verify server certificate
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

After a QoS 1 exchange has fully completed, a non-retained QoS 0 publication
on the exact test topic is ignored only if its entire payload is byte-for-byte
identical to the current payload. It increments a separate extra-copy counter,
logged first and every 10, and never increments checked or satisfies an
outstanding echo. A new publication or session reset revokes this eligibility.
Other non-retained QoS 0 publications on the exact test topic still fail.
This behavior belongs to the shared example application; it does not change
the MQTT protocol client.

Record the modem firmware, registration messages, security label, MQTT
subscription and checked counters. The first checked exchange and every
10th are logged. After a connection failure the application closes the
endpoint and retries a fresh TLS/MQTT connection, then subscribes again.
Preserve any native `nRF91 socket ...` diagnostics when reporting a failure.

Run the dedicated host checks from the repository root:

```sh
python3 tests/net/run_mqtt_tls_demo.py --target nrf91x1
```

Host validation for this profile is recorded in the shared guide. Host
substitutes do not perform the modem's cryptography, validate its certificate
handling or establish a live broker session. The separate maintainer-supplied
hardware result is recorded below. No fresh automated ARM compile/link result
is claimed. Earlier plaintext LTE and native-PSK H753 results apply to their
recorded profiles.

## Initial hardware result and unresolved echo failures

The maintainer ran this project with `mfw_nrf91x1_2.0.1`. The console reported
CA storage at tag 42420, roaming LTE-M registration and
`test.mosquitto.org:8883` with `TLS 1.2, CA/hostname verified`. Client
`iosnetd92521fd559d5c03` subscribed to
`iosonata/demo/d92521fd559d5c03/echo` at QoS 1 and checked returned payloads.

The initial session printed checked counter 1, then reported
`echo payload or QoS mismatch` at checked 4 and failures 1. A fresh verified
TLS connection and subscription checked counter 5, then the same error raised
failures to 2. A second fresh TLS/MQTT session checked counter 6 before the
same error raised failures to 3. The application used its 30-second reconnect
policy after the first two failures. Every failure reported
`mqtt_error=0 mqtt_result=0`; these are shared MQTT state fields, not an
explanation of the rejected echo. The underlying cause remains unresolved.

This confirms initial credential provisioning, TLS/MQTT interoperability,
six checked self-echoes and the two observed automatic reconnects. A clean
extended run, deliberate broker or radio-loss recovery and negative
CA/hostname checks are not established. See the
[detailed hardware record](../../../../../../docs/network-protocol-examples.md#hardware-evidence-for-the-modem-tls-mqtt-example)
for the observed counters and diagnostic scope.

For a repeat run, capture the short `MQTT echo match=<0|1> done=<0|1>` line,
`MQTT echo rejected`, the bounded payload hex dump and any
`MQTT echo sequence received` line before the failure summary, together with
the client ID and topic. The
[echo diagnostic instructions](../../../../../../docs/network-protocol-examples.md#echo-failure-diagnostics)
explain these fields. They preserve the strict echo checks and do not establish
a fix or show that another public-broker client caused the original failures.

A follow-up run with client `iosnet172bbd285c24efa7` reused the stored CA,
subscribed at QoS 1 and checked echo 1. It then reported
`MQTT echo rejected: QoS qos=0 dup=0 len=22 expected_seq=00000004`.
The received QoS explains that rejection, but the console capture lost part
of the hex payload and failure summary. The received sequence, publication
origin and console truncation cause remain unresolved. `expected_seq` is not
the received sequence or verified-echo count. See the
[follow-up capture procedure](../../../../../../docs/network-protocol-examples.md#follow-up-received-qos-0-publication)
for an independent PC subscriber; keep the board on TLS port 8883.

A subsequent run used client `iosnet00c6a7784c9623aa` and topic
`iosonata/demo/00c6a7784c9623aa/echo`. Its initial session checked counter 1,
then rejected QoS 0 with length 22 and expected sequence 1, raising failures
to 1. The first reconnect checked counter 2 before another QoS 0, length-22
rejection with expected sequence 2 and failures 2. The second reconnect
progressed through checked counter 6 before the same QoS/length rejection
with expected sequence 6 raised failures to 3.

The second supplied hex dump contains 40 characters, while 22 bytes require
44, and other dumps and summaries are cut off. Full byte equality and the
received sequence therefore remain unestablished. The packet origin and
console truncation cause are unresolved, with no PC observer result supplied.
These logs predate the additional-copy policy and do not establish that their
packets qualify for it or that the hardware failures have been resolved.

## Follow-up hardware result: ten verified echoes

A later run with client `iosnetbd19c00cd4898c0a` and
`mfw_nrf91x1_2.0.1` reused the stored CA at tag 42420, registered on roaming
LTE-M and connected to `test.mosquitto.org:8883` with
`TLS 1.2, CA/hostname verified`. It subscribed to
`iosonata/demo/bd19c00cd4898c0a/echo` at QoS 1 and printed checked counters
1 and 10, both with `failures=0`.

This confirms ten verified QoS 1 self-echoes in the supplied capture. No
extra-copy, rejection or reconnect line appears, so the capture does not
demonstrate the additional-copy path or resolve the earlier publication
origin and console truncation. Extended operation, deliberate recovery and
negative CA/hostname checks remain unestablished by this run. See the
[follow-up hardware record](../../../../../../docs/network-protocol-examples.md#follow-up-ten-verified-echoes-without-failures).
