# Generic MQTT and CoAP examples

`MqttDemo` and `MqttTlsDemo` use the same shared MQTT application; `CoapDemo`
uses the shared CoAP application. Both protocols receive a selected
`DeviceIntrf *`. The target project supplies board configuration;
`exemples/net/net_demo.cpp` initializes the selected network and opens its
interface. The MQTT and CoAP applications use the returned pointer without
depending on the modem or Ethernet driver.

These are new examples. The existing `LteMqttDemo`, `LteCoapDemo`,
`EthernetCoapDemo` and `EthernetTcpDemo` applications remain available with
their original settings and validation records.

## Projects and files

| Target | Library | MQTT project | Modem TLS MQTT project | CoAP project | Default medium |
| --- | --- | --- | --- | --- | --- |
| nRF9160, Thingy:91 or DK | `IOsonata_nRF9160` | [MqttDemo](../ARM/Nordic/nRF91/nRF9160/exemples/MqttDemo/README.md) | [MqttTlsDemo](../ARM/Nordic/nRF91/nRF9160/exemples/MqttTlsDemo/README.md) | [CoapDemo](../ARM/Nordic/nRF91/nRF9160/exemples/CoapDemo/README.md) | LTE-M |
| nRF9151 DK or nRF9161 DK | `IOsonata_nRF91x1` | [MqttDemo](../ARM/Nordic/nRF91/nRF91x1/exemples/MqttDemo/README.md) | [MqttTlsDemo](../ARM/Nordic/nRF91/nRF91x1/exemples/MqttTlsDemo/README.md) | [CoapDemo](../ARM/Nordic/nRF91/nRF91x1/exemples/CoapDemo/README.md) | LTE-M |
| NUCLEO-H753ZI | `IOsonata_STM32H753` | [MqttDemo](../ARM/ST/STM32H7xx/STM32H753/exemples/MqttDemo/README.md) | - | [CoapDemo](../ARM/ST/STM32H7xx/STM32H753/exemples/CoapDemo/README.md) | Ethernet with DHCP |

Each project links these shared example files:

| File | Responsibility |
| --- | --- |
| `exemples/net/mqtt_generic.cpp` | MQTT identity, subscription, publication, payload checks and session recovery |
| `exemples/net/coap_generic.cpp` | CoAP public GET or controlled-helper requests, response checks and exchange recovery |
| `exemples/net/net_demo.cpp` | Console, event queue, timer, RNG, selected medium and connection setup |
| `exemples/net/net_demo.h` | Example setup functions and custom-medium hooks |
| Target `src/board.h` | Pins, medium selection, timing, RNG provider and peer configuration |

`MqttTlsDemo` also links `exemples/net/net_demo_tls_ca.h` and
`net_demo_tls_ca.cpp`, which declare and hold the public demonstration CA.

`net_demo.h` is an example helper, not a new public library interface. The
existing protocol implementations remain `src/net/mqtt.cpp` and
`src/net/coap.cpp`. Their C functions operate on caller-owned protocol state;
the C++ `Mqtt : Device` and `Coap : Device` wrappers hold that same state and
accept `DeviceIntrf *`.

## Build and start

1. Import the selected MCU's `lib/ioc` project into IOcomposer.
2. Import its `exemples/MqttDemo/ioc`, `exemples/MqttTlsDemo/ioc` or
   `exemples/CoapDemo/ioc` project as listed above.
3. Configure the application's `src/board.h` for the board and peer.
4. Build the library first, then the application with matching Debug or Release
   configurations. For Nordic, use the bare-metal library configuration.
5. Program the intended MCU and reset it with the diagnostic console open.

The target variants use the same IOcomposer project names. Use separate
workspaces when keeping several variants of the same example imported.
The library must contain the committed MQTT/CoAP source links; refresh or reopen
an older workspace import and rebuild the library if those links are stale.

| Board | Console | Additional setup |
| --- | --- | --- |
| Thingy:91 | UART0 RX P0.19 / TX P0.18, 115200 baud | Keep the working nRF52840 UART-to-USB bridge; program the nRF9160 application |
| nRF9160 DK | UART0 RX P0.28 / TX P0.29, VCOM0, 115200 baud | Select `NRF9160_DK` instead of `NORDIC_THINGY91` in the board header |
| nRF9151 DK / nRF9161 DK | UART0 RX P0.26 / TX P0.27, VCOM0, 115200 baud | Use the nRF91x1 library and existing `NRF9120_XXAA` target configuration |
| NUCLEO-H753ZI | USART3 through ST-LINK VCP, 1000000 baud | Connect CN14 to Ethernet and assign a unique MAC address |

All consoles use 8N1 without flow control. The setup helper uses a static
1024-byte UART TX FIFO and defers application processing through the existing
event queue. Its timer schedules the foreground pump every 20 milliseconds;
substantial protocol work is not performed in the timer interrupt.

Nordic projects retain the existing `external/nrfx` and
`external/sdk-nrfxlib` layout, Cortex-M33 hard-float settings and modem library.
The nRF9160 project selects `nrf9160_xxaa.ld` and the `nrf9160/hard-float`
modem library; nRF91x1 selects `nrf9120_xxaa.ld` and `nrf9120/hard-float`.
The selected `CryptoRngNrfInstance()` requires the existing secure-image
configuration. Do not define `NRF_TRUSTZONE_NONSECURE` in the library or example.

H753 projects use the existing `external/lwip` dependency and
`gcc_stm32h753xx.ld`. Preserve `.eth_dma` in AXI SRAM at `0x24000000`, because
Ethernet DMA cannot access DTCM. The LAN8742A uses RMII, PHY address 0 and
TXD1 on PB13. Debug requires semihosting enabled; Release can run without the
debugger. The selected RNG is `CryptoRngStm32Instance()`.

## Selecting the interface

The selection follows the sensor example in
`exemples/sensor/env_tph_demo.cpp`: initialize the concrete transport, select
its `DeviceIntrf *`, and pass it to the unchanged device implementation.
Here the example helper returns the selected connected network interface:

```cpp
DeviceIntrf *pIntrf = NetDemoOpen(false, MQTT_DEMO_HOST, MQTT_DEMO_PORT);

if (pIntrf)
{
	// cfg is the application's MqttCfg_t; open may have taken time.
	mqtt.Init(cfg, pIntrf, NetDemoNow());
}
```

CoAP uses `NetDemoOpen(true, COAP_DEMO_HOST, COAP_DEMO_PORT)` and injects its
result into `Coap::Init`. `false` requests a connected byte stream; `true`
requests a connected datagram interface. `NetDemoNow()` is sampled after a
possibly blocking open so protocol deadlines start at the appropriate time.

The application files contain no LTE registration, DHCP, MAC or PHY operations.
Those belong in the selected setup code. LAN and Internet peers use the same
protocol application; the network must provide the required route and DNS.

### Built-in medium selections

`net_demo.h` is included before `board.h`, which selects one of these values:

```cpp
#define NET_DEMO_MEDIUM NET_DEMO_MEDIUM_LTE
```

or:

```cpp
#define NET_DEMO_MEDIUM NET_DEMO_MEDIUM_ETHERNET
```

The supplied Nordic projects select LTE. Configure `NET_DEMO_APN`,
`NET_DEMO_RAT` and `NET_DEMO_RAT_PREF` for the SIM and network. Defaults are
the network's APN (`nullptr`) and LTE-M. The helper disables requested PSM,
eDRX and release assistance for these continuous checks, waits for registration
and uses the nRF91 modem's connected `SockIntrf`.

The H753 projects select Ethernet. Configure `ETHERNET_PINS`,
`ETHERNET_MAC_ADDR` and `ETHERNET_PHY_ADDR` for the board. DHCP is enabled by
default through `NET_DEMO_DHCP`. For static addressing, disable DHCP and set
`NET_DEMO_IP_ADDR`, `NET_DEMO_NETMASK`, `NET_DEMO_GATEWAY` and `NET_DEMO_DNS`.
The helper waits for `IpReady()` and uses the shared lwIP `SockIntrf`.
Its socket configuration allows a 10000-millisecond DNS/connect wait and
stream-send wait. The 20-millisecond pump period is a scheduling interval,
not a maximum duration for a blocking network operation.

Changing the selection requires an implementation and board configuration for
that medium. Each current MCU library selects its own global `SockIntrf`
implementation. The modem and lwIP implementations cannot both be linked
unchanged into one firmware image, because they define the same public socket
functions. Simultaneous media require suitable distinct transport
implementations and explicit application selection between them.

### Custom Wi-Fi or another compatible interface

Select `NET_DEMO_MEDIUM_CUSTOM` and provide these functions in an application
source linked into the project:

| Function | Required behavior |
| --- | --- |
| `NetDemoCustomInit(TimerDev_t *)` | Initialize the selected medium; return whether initialization succeeded |
| `NetDemoCustomProcess(uint32_t)` | Pump any medium-specific foreground work |
| `NetDemoCustomReady()` | Report whether the network can open a connection |
| `NetDemoCustomOpen(bool, const char *, uint16_t)` | Open the requested stream or datagram endpoint and return its caller-owned `DeviceIntrf *` |
| `NetDemoCustomConnected()` | Report whether that endpoint remains usable |
| `NetDemoCustomClose()` | Close that endpoint and release its connection resources |

The startup banner uses `NetDemoMediumName()` to identify the selected medium.
Built-in selections report `LTE` or `Ethernet`. A custom project can define
`NET_DEMO_MEDIUM_NAME` in `board.h`, for example `"Wi-Fi"`; otherwise the label
is `custom network`.

These functions perform setup and status work. Protocol data flows through
the returned `DeviceIntrf`, using the existing MQTT or CoAP implementation.
Keep the interface object and its buffers alive for the whole protocol
session, including failure processing and protocol reset. Closing a connection
must not destroy the caller-owned `DeviceIntrf` or its `DevIntrf_t` storage.
Use static or caller-owned storage and one foreground owner.

`NetDemoCustomProcess()` must not silently replace or reopen an active
endpoint. Latch a lost connection as disconnected until the application has
called Close and then Open, even if the network itself has recovered. This
ensures that a short interruption or the reuse of the same interface object
still produces fresh protocol initialization.

For MQTT, the interface must provide an ordered byte stream and support partial
reads and writes. For CoAP, it must preserve UDP-style message boundaries:
one write submits one whole datagram and one read returns at most one datagram.
A partial datagram write is a failure, not a request to send the remaining
bytes as another message. Receive truncation must be detectable. A raw UART
or SPI interface does not acquire these network semantics merely by being a
`DeviceIntrf`.

The examples use synchronous transfer results and set `MaxRetry` to zero for
bounded polling. A zero result means no progress yet; a negative transfer
result indicates failure. An interrupt-driven implementation whose negative
return means pending completion needs a suitable synchronous adapter before
it can be injected into these protocol clients.

There is no working Wi-Fi transport supplied in this repository. The CMSIS
Wi-Fi file under `DriverTemplates` returns unsupported operations, and the
`DEVINTRF_TYPE_WIFI` identifier does not implement network access. The custom
hooks are the integration point for a working Wi-Fi or other interface; they
do not claim Wi-Fi hardware support.

### Connection replacement

An interface pointer is selected for a connection, not exchanged beneath an
active protocol session. When the selected medium or endpoint is lost, report
it as unavailable, let the application reset its protocol state, close the old
connection, and open a fresh one. MQTT then sends a new CONNECT and subscribes
again. CoAP uses a fresh endpoint when its context is reset and preserves any
remaining unanswered-peer pacing delay in the application.

Changing the pointer cannot migrate an established TCP or MQTT session.
The current MQTT client uses clean sessions; the example has no durable
publication queue. Application delivery policy across reconnects remains
explicit. If a custom interface provides authenticated TLS, finish its
handshake before presenting it as connected to MQTT. `MqttDemo` and `CoapDemo`
default to plaintext TCP and UDP. The separate Nordic `MqttTlsDemo` profile
uses modem TLS with CA-chain and hostname verification. CoAP remains
plaintext; these examples do not enable DTLS or OSCORE.

## MQTT check

All `MqttDemo` targets default to the same settings:

```cpp
#define MQTT_DEMO_HOST "test.mosquitto.org"
#define MQTT_DEMO_PORT 1883
#define MQTT_DEMO_INTERVAL 10000U
```

The [public broker's service page](https://test.mosquitto.org/) identifies
1883 as an unencrypted, unauthenticated listener. A public test needs no
broker on the PC, but the selected network must provide DNS and outbound TCP
access. For repeatable failure and recovery testing, use a broker you control.

The application uses eight random bytes from the configured RNG to make a
16-hex-digit suffix. The client ID is `iosnet<suffix>` and the topic is
`iosonata/demo/<suffix>/echo`. They remain stable during that boot, including
reconnections. Each board therefore has its own test topic without placing a
SIM identifier, device serial number or credential in a public publication.

After connecting, the application subscribes to that exact topic at QoS 1
and publishes synthetic sequence-numbered data. A checked exchange requires
both the broker's PUBACK and a matching returned topic and payload. The first
publication follows subscription; later publications are at least 10 seconds
apart. Retained or unrelated-topic messages do not count as live echoes, and
old QoS 1 duplicates do not increment the counter. A subscription grant below
QoS 1 fails this particular test even though the shared client supports QoS 0.

The example ignores one additional case without ending the session: a
non-retained QoS 0 publication on the exact test topic whose complete payload
is byte-for-byte identical to the current payload, after that QoS 1 exchange
has fully completed. The application must already have received the matching
QoS 1 self-echo and the broker's PUBACK, completed its pending acknowledgements
and counted the exchange (`!s_Waiting && s_EchoReceived`). The extra copy never
satisfies an outstanding echo and never increments the checked counter.
Starting a new publication or resetting the session revokes this eligibility.
Other non-retained QoS 0 publications on the exact test topic still fail.

Ignored copies have a separate counter, logged on the first and every tenth:

```text
MQTT extra QoS 0 copy ignored <count>, checked=<count>
```

This is an example policy applied through the same shared MQTT client on
each medium. It does not change MQTT parsing or establish where an additional
publication originated.

The public topic is not private or authenticated. Another broker client can
read or publish to it. To observe the exact topic from a PC, substitute the
suffix printed by the board:

```sh
mosquitto_sub -h test.mosquitto.org -p 1883 \
  -t 'iosonata/demo/SUFFIX_FROM_CONSOLE/echo' -q 1 -v
```

The observer is optional. Keep it read-only while the board verifies payloads.
Use the application's checked-message counter, not successful socket writes,
as evidence of the round trip.

The keepalive is 60 seconds. Protocol operations and PUBACK/echo completion
use 30-second deadlines. Session failure closes the endpoint and permits a
fresh connection after the retry delay and network readiness. The existing
nRF91 port separately allows 30 seconds for each TCP connection attempt,
then applies the example's 100-millisecond send wait; DNS has its own modem
wait. Thus opening an endpoint can take longer than a foreground pump period.

Changing port 1883 to 8883 does not configure security. The public broker's
encrypted listeners use X.509 certificates; the native IOsonata TLS client
currently implements external PSK rather than certificate authentication.
See [shared networking documentation](networking.md) for the separate native
TLS and nRF91 modem credential paths.

### Echo failure diagnostics

When an incoming publication on the live test topic fails the echo check,
the application prints the reason before its existing failure summary:

```text
MQTT echo match=<0|1> done=<0|1>
MQTT echo rejected: <reason> qos=<n> dup=<0|1> len=<bytes> expected_seq=<8hex>
MQTT echo payload hex (<shown>/<total> bytes): <hex>
```

The short first line reports whether the complete received payload matches
the current payload (`match`) and whether the current QoS 1 exchange has
fully completed (`done`). It does not decode a missing sequence suffix from
the console capture.

Reasons distinguish the QoS, length, prefix, sequence encoding and a sequence
ahead of the expected value. The hex dump shows at most 32 payload bytes.
A sequence ahead also prints:

```text
MQTT echo sequence received=<8hex>
```

Capture those lines with the client ID, exact topic and subsequent failure
summary. They identify the rejected publication; the strict QoS 1 and payload
integrity checks remain in place. Retained messages, other topics and normal
old QoS 1 duplicates remain excluded from a live echo check. A different
client can publish to the same topic on this public broker, including when
the board uses TLS, but this possibility alone does not establish the cause
of an observed mismatch. The initial modem-TLS log below predates these
detailed diagnostics and does not identify the rejected packet.

### MQTT over modem TLS

The separate `MqttTlsDemo` projects for nRF9160 and nRF91x1 select the modem's
TLS 1.2 socket. They link the same `mqtt_generic.cpp` and shared MQTT client
as `MqttDemo`. The setup opens and authenticates the stream before returning
its `DeviceIntrf *`; the MQTT application does not provision credentials or
implement TLS. Existing plaintext example settings remain available.

The new board headers include `net_demo_tls_ca.h` and select:

```cpp
#define NET_DEMO_MODEM_TLS 1
#define NET_DEMO_TLS_SEC_TAG 42420
#define NET_DEMO_TLS_CA_CERT g_NetDemoMosquittoCa
#define NET_DEMO_TLS_REPLACE_CA 0
#define MQTT_DEMO_HOST "test.mosquitto.org"
#define MQTT_DEMO_PORT 8883
#define MQTT_DEMO_INTERVAL 10000U
```

The [broker's service page](https://test.mosquitto.org/) lists port 8883 as
TLS without a required client certificate. The setup requires peer
verification and supplies the configured hostname to the modem. Use the DNS
hostname whose identity the server certificate establishes; substituting a
numeric address changes the identity being checked. A TLS failure returns no
usable stream and never falls back to plaintext. This selection currently
requires LTE: other built-in/custom media are rejected at compile time, and
datagram opens are refused.

#### Credential preparation

Reserve tag **42420** for this demo's CA. It is a persistent modem credential
slot, so do not share it with another application's trust configuration.
During startup or modem reinitialization, setup initializes the modem,
checks that `LteDisconnect()` has put the radio offline, prepares the CA,
then checks `LteConnect()`. Network readiness remains false if any step fails.
Ordinary MQTT socket reconnections reuse the prepared credentials.

- With `NET_DEMO_TLS_REPLACE_CA = 0`, an existing CA at the selected tag is
  reused when reported present. Presence alone does not show that it matches
  the compiled CA. A false presence result can also mean credential listing
  failed; setup then attempts the configured CA write. Use only the dedicated
  tag owned by this demo.
- If no CA exists, the configured PEM string is written. Set
  `NET_DEMO_TLS_CA_CERT` to `nullptr` to require an already-provisioned CA.
- Setting `NET_DEMO_TLS_REPLACE_CA = 1` explicitly replaces the CA entry in
  that tag. Other credential types are not deleted. Return the setting to
  zero after provisioning to avoid rewriting it on each modem initialization.

The supplied `g_NetDemoMosquittoCa` is the public CA from
[the broker's HTTPS certificate link](https://test.mosquitto.org/ssl/mosquitto.org.crt),
retrieved on October 22, 2026. Its DER SHA-256 fingerprint is:

```text
FC:9E:45:E2:8F:6F:49:87:FD:14:81:E1:46:BC:FC:31:12:4E:F2:FA:BF:20:AB:04:7E:3F:85:92:C2:2B:65:39
```

The certificate records validity from June 9, 2020, 11:06:39 UTC through
June 7, 2030, 11:06:39 UTC. It is public test data, not a private key or a
production trust policy. Changing the broker requires its corresponding CA
and hostname configuration; an existing tag is not updated merely by changing
the compiled PEM string.

#### Verification scope and checks

The modem verifies the certificate chain and configured hostname, but does
not verify server certificate expiry. Nordic documents this limitation for
[nRF9160 firmware 1.2.8](https://nsscprodmedia.blob.core.windows.net/prod/software-and-other-downloads/dev-kits/nrf9160-dk/release_notes_modemfirmware/mfw_nrf9160_1.2.8_release_notes.txt)
and [nRF91x1 firmware 2.0.1](https://nsscprodmedia.blob.core.windows.net/prod/software-and-other-downloads/sip/nrf91x1-sip/nrf91x1-lte-modem-firmware/release-notes/mfw_nrf91x1_2.0.1_release_notes.txt).
This example does not add certificate validity-date enforcement. Expiration
alone is therefore not an expected-rejection test for this profile.

After a successful open the security label is
`TLS 1.2, CA/hostname verified`. Then require a QoS 1 subscription and a
checked self-echo, which requires both PUBACK and the matching returned
topic/payload. The client ID, topic, keepalive, timing and reconnect behavior
are the same as the plaintext example. Reconnection performs a fresh TLS
open before MQTT CONNECT and re-subscription.

For negative checks, use an unused dedicated tag with
`NET_DEMO_TLS_CA_CERT = nullptr` to exercise missing credentials, or a separate
owned tag containing a valid CA that does not trust the broker. The default
configured CA would otherwise automatically populate a missing tag. A
hostname check needs a reachable TLS server with a trusted CA and a mismatched
configured hostname; a nonexistent DNS name only tests name resolution.
There must be no MQTT subscription or new checked echo after failure. Restore
the correct configuration and verify a fresh session. Capture native socket
diagnostics and the modem firmware version with the result; host substitutes
cannot establish the modem's cryptographic behavior.

The public listener authenticates the server to this connection; it does not
authenticate the MQTT user or make public topics private. It is not mutual
TLS. The shared native TLS 1.3 external-PSK implementation remains separate
and unchanged, with no native certificate authentication or shared DTLS
added by this profile. The nRF91x1 hardware result and unresolved echo-check
failures are recorded below. nRF9160 modem-TLS hardware validation and fresh
automated ARM compile/link validation remain unestablished.

## CoAP check

All supplied targets default to a public read-only profile:

```cpp
#define COAP_DEMO_PUBLIC_TEST 1
#define COAP_DEMO_HOST "coap.me"
#define COAP_DEMO_RESOURCE "test"
#define COAP_DEMO_PORT 5683
#define COAP_DEMO_INTERVAL 10000U
#define COAP_DEMO_PROBING_RATE 0U
```

The [coap.me service](https://coap.me/about) publishes a public client-test
server. This profile sends a confirmable GET with Uri-Host and one Uri-Path
segment. `COAP_DEMO_RESOURCE` contains that single raw segment, without a
leading slash, URL scheme, query or fragment. It must contain 1..255 bytes and
must not be `.` or `..`.

A checked response has code 2.05 Content and at most 1024 payload bytes. Its
body can change, so the public check reports its length rather than comparing
it with fixed content. This is a connectivity and CoAP GET check; it does not
establish the full helper exercise below. Public service availability and the
selected network's UDP policy can affect the result.

### Controlled helper exercise

Set `COAP_DEMO_PUBLIC_TEST` to `0` and point `COAP_DEMO_HOST` at a reachable
server you control. On that server, use Python 3.10 or newer:

```sh
python3 -m venv .venv-coap
. .venv-coap/bin/activate
python3 -m pip install aiocoap==0.4.17
python3 Python/coap_server.py --host SERVER_INTERFACE_IP --port 5683 --verbose
```

Replace `SERVER_INTERFACE_IP` with an address assigned to that server. The
helper binds one concrete address, not `0.0.0.0` or `::`; its loopback default
is for host tests. Allow UDP 5683 through the server's firewall.

The eight-step cycle checks:

| Step | Request | Expected response |
| --- | --- | --- |
| 1 | CON GET `/hello` | 2.05 and `IOsonata CoAP` followed by newline |
| 2 | CON POST `/echo` | 2.05 and the exact binary request bytes |
| 3 | NON POST `/echo` | 2.05 and the same binary bytes |
| 4 | CON PUT `/value` | 2.04, empty body |
| 5 | CON GET `/value` | 2.05 and the stored binary bytes |
| 6 | CON DELETE `/value` | 2.02, empty body |
| 7 | CON GET `/value` | 4.04, empty body |
| 8 | CON GET `/slow` | Empty ACK, then a separate CON 2.05 with `separate response` followed by newline |

Binary lengths rotate through 1, 16, 257 and 900 bytes across cycles. The
helper's `/value` resource is shared and volatile; use one board per helper
instance and avoid modifying that resource during the check. All requests
remain at least `COAP_DEMO_INTERVAL` apart. The helper has no authentication
or persistent storage.

For Ethernet on the same LAN, use the server's LAN address. A private address
such as `10.5.5.51` is not directly reachable through ordinary cellular
Internet. LTE needs a public/routed server, an explicit VPN/private-APN route,
or a suitable router forwarding UDP 5683 to the PC. With forwarding, the board
uses the router's reachable address and the helper binds the PC's own address.
Carrier or upstream NAT can prevent that forwarding arrangement.

### CoAP recovery and limits

The client keeps one exchange outstanding. Its configured operation deadline
is 95 seconds; a silent confirmable request can consume roughly 93 seconds
of retransmission time before failure. An explicit transport or Reset error
can be reported sooner. Protocol failures can release the exchange while
retaining the live UDP endpoint; transport loss closes it and requires a new
endpoint and protocol initialization.

`COAP_DEMO_PROBING_RATE = 0` selects the library default of one byte per second
for an unanswered peer. Successfully transmitted request bytes, including
retransmissions, can impose an additional wait before the next request. The
application retains that wait across connection replacement and also observes
its minimum retry delay. The default is appropriate for these public-network
checks; a controlled LAN test may explicitly choose another rate. It does not
limit normal requests when the peer responds.

The shared client supports unicast GET/POST/PUT/DELETE, CON/NON, piggyback and
separate responses, retransmission, Reset and duplicate CON acknowledgement.
It has a maximum complete datagram of 1152 bytes and at most 16 options. It
does not implement DTLS, OSCORE, Observe, blockwise transfer, multicast,
proxying, CoAP over TCP or an embedded server engine. See
[networking documentation](networking.md) for ownership, option and timing
details.

## Validation status

Run the new example checks from the repository root:

```sh
python3 tests/net/run_generic_demo.py
python3 tests/net/run_generic_demo.py --protocol mqtt --target nrf9160
python3 tests/net/run_generic_demo.py --protocol coap --target stm32h753
```

The runner accepts `--protocol mqtt|coap|all` and
`--target nrf9160|nrf91x1|stm32h753|all`. The original plaintext-example
default matrix passed:

| Check | Result |
| --- | --- |
| All six project source links and actual application/setup board headers | Host syntax passed for nRF9160, nRF91x1 and STM32H753 |
| MQTT application with the actual shared protocol implementation | O0/O2 ASan/UBSan passed for all three target configurations |
| CoAP public and helper profiles with the actual shared protocol implementation | O0/O2 ASan/UBSan passed for all three target configurations |
| Empty-host startup guard for both protocols | O0/O2 ASan/UBSan passed on the first target configuration |
| LTE, Ethernet and custom setup implementations | O0/O2 ASan/UBSan passed with hardware substitutes |

This run completed 28 sanitized runtime builds. Protocol fixtures inject
different `DeviceIntrf` implementations and check fresh protocol sessions
after endpoint replacement. CoAP checks also cover all eight helper steps,
public response bounds, datagram boundaries, duplicate acknowledgements and
retained probing delay through repeated loss and clock wrap. Setup fixtures
exercise startup, initialization retries, network status and deferred pumping.

The hardware interfaces in this automated run are host substitutes. These
checks do not perform an ARM compile/link, a live public-server session or a
hardware test. Subsequent maintainer-supplied hardware evidence is recorded
below with its observed scope.

The established protocol tests remain available separately:

```sh
python3 tests/net/run_mqtt.py
python3 tests/net/run_coap.py --require-interop
python3 tests/net/run_nrf91_socket.py
```

Their existing results and dependencies are documented in
[networking.md](networking.md).

### Modem TLS profile checks

Run the dedicated profile checks from the repository root:

```sh
python3 tests/net/run_mqtt_tls_demo.py
python3 tests/net/run_mqtt_tls_demo.py --target nrf9160
python3 tests/net/run_mqtt_tls_demo.py --target nrf91x1
```

The full runner passed for both target configurations:

| Check | Result |
| --- | --- |
| Actual board headers with the application, setup and CA sources | Six host syntax checks passed |
| Modem TLS selected with Ethernet or a custom medium | Both unsupported selections rejected at compile time |
| Secure MQTT application with the actual shared client | Four O0/O2 ASan/UBSan binaries passed |
| Credential preparation and network setup | Six O0/O2 ASan/UBSan binaries passed across 24 scenario executions |
| Actual nRF91 socket port with modem substitutes | Two O0/O2 ASan/UBSan binaries passed |

This completed 12 sanitized runtime binaries and 30 runtime process
executions. Setup checks cover missing/stored CA credentials, explicit
replacement, the pre-provisioned-only setting, initialization/radio-off/write/
post-write-check/reattach failures, retry timing with clock wrap, TLS-only
stream opens, datagram rejection, partial-open cleanup and recovery after
socket, registration or modem faults. Socket reconnections are checked to
avoid repeated CA writes. The existing plaintext example matrix was also
rerun successfully across all 28 sanitized runtime builds after this change.

These checks use host substitutes for the modem. They do not perform an ARM
compile/link, a modem TLS handshake, certificate authentication or a live
broker session, and do not validate modem firmware behavior. The plaintext
MQTT and CoAP hardware records below do not establish TLS operation.

The echo diagnostic update passed `run_generic_demo.py --protocol mqtt`
(14 sanitized runtime builds) and the full `run_mqtt_tls_demo.py` runner
at O0/O2 with ASan/UBSan. The application fixture checks all five rejection
reasons, metadata and payload bytes captured before release, empty and binary
payloads, and the 32-byte sample limit. A valid MQTT QoS 0 publication confirms
that an application rejection can have `mqtt_error=0 mqtt_result=0`. Accepted
duplicates, retained messages and other topics produce no rejection diagnostic.
These results validate the diagnostics, not the cause of the hardware failure.

The additional QoS 0 copy policy also passed
`run_generic_demo.py --protocol mqtt` (14 sanitized runtime builds) and
`run_mqtt_tls_demo.py` (12 binaries and 30 process executions), at O0/O2 with
ASan/UBSan for all configured targets. The fixtures check ignoring an exact
copy after a completed exchange, rejection while acknowledgements remain
pending, changed payload bytes or length, and revoked eligibility after the
next publication or a session reset. These are host checks; they add no ARM
compile/link, hardware or live-broker result and do not resolve the incomplete
hardware payload captures or console truncation.

### Hardware evidence for the modem TLS MQTT example

The maintainer ran `MqttTlsDemo` on nRF91x1 with modem firmware
`mfw_nrf91x1_2.0.1`. The console reported `TLS CA stored under tag 42420`,
roaming LTE-M registration and a connection to `test.mosquitto.org:8883`
labelled `TLS 1.2, CA/hostname verified`. The client ID was
`iosnetd92521fd559d5c03` and the QoS 1 subscription used
`iosonata/demo/d92521fd559d5c03/echo`.

The log contains successful self-echo checks and repeated application failures:

| Session | Observed result |
| --- | --- |
| Initial TLS/MQTT session | Printed checked counter 1; the later failure reported checked 4 and failures 1. |
| First automatic reconnect | Fresh verified TLS connection and QoS 1 subscription; checked 5 with failures 1, then a failure at checked 5 and failures 2. |
| Second automatic reconnect | Fresh verified TLS connection and QoS 1 subscription; checked 6 with failures 2, then a failure at checked 6 and failures 3. |

All three failures reported `echo payload or QoS mismatch` with
`mqtt_error=0 mqtt_result=0`. The application followed its configured
30-second reconnect policy after the first two failures. Those zero fields
describe the shared MQTT client's state; they do not explain the application's
rejected echo. The example already ignores retained messages, other topics
and normal old QoS 1 duplicates. That initial log does not identify the
rejected QoS, payload or sequence, and the underlying cause remains unresolved.
Use the [echo failure diagnostics](#echo-failure-diagnostics) to capture a
recurrence; adding diagnostics does not establish a fix.

This establishes initial CA provisioning, TLS/MQTT interoperability, six
checked self-echoes and the two observed automatic session reconnections on
the tested nRF91x1 hardware. It does not establish a clean extended run,
deliberate broker or radio-loss recovery, wrong-CA/hostname rejection or
nRF9160 modem-TLS operation. No fresh automated ARM compile/link validation
is claimed.

#### Follow-up: received QoS 0 publication

A later run with the same modem firmware reported reuse of the stored CA at
tag 42420, a verified TLS connection, a QoS 1 subscription and checked echo 1
with no failures. Client `iosnet172bbd285c24efa7` used topic
`iosonata/demo/172bbd285c24efa7/echo`. The new diagnostic then reported:

```text
MQTT echo rejected: QoS qos=0 dup=0 len=22 expected_seq=00000004
```

This identifies the rejection as a received publication reporting QoS 0.
The payload hex line and failure summary were truncated in the supplied
console capture. The visible bytes resemble the demo payload prefix, but do
not include its sequence suffix. `expected_seq` is the application's expected
value, not a decoded received sequence or a verified-echo count. The capture
does not establish the full payload, the publication's origin or a cause for
the missing console bytes. It also does not establish that the earlier three
failures rejected the same kind of message.

The maintainer subsequently supplied a run using client
`iosnet00c6a7784c9623aa` and topic `iosonata/demo/00c6a7784c9623aa/echo`.
The continued log spans three sessions in that same run. Each rejected
publication reported QoS 0 and length 22:

| Session | Checked counter before rejection | Expected sequence at rejection | Failure counter |
| --- | --- | --- | --- |
| Initial session | 1 | `00000001` | 1 |
| First reconnect | 2 | `00000002` | 2 |
| Second reconnect | Progressed through 6 | `00000006` | 3 |

The second supplied payload dump contains 40 hex characters, while 22 bytes
require 44. Other payload dumps and failure summaries are also cut off. These
captures do not establish byte equality with the demo's current payload or
the full received sequence. The publication's origin and the console
truncation cause remain unresolved, and no PC observer result has been
provided.

The application still publishes at QoS 1 and requires a QoS 1 SUBACK result.
Its additional-copy policy above applies only after a completed QoS 1
exchange and an exact full-payload comparison. The historical captures do not
prove that their rejected packets meet those conditions, so they do not
establish a hardware fix. An independent PC subscriber can capture the QoS,
length and complete payload without relying on the board console:

```sh
mosquitto_sub -h test.mosquitto.org -p 1883 -V mqttv311 -q 1 \
	-t 'iosonata/demo/00c6a7784c9623aa/echo' \
	-F 'qos=%q len=%l hex=%x'
```

Use the topic printed by the current board run; a new boot selects a new
random topic. This observer uses the public plaintext listener, while the
board stays on TLS port 8883. The command only subscribes. A matching QoS 0
capture would independently establish broker-delivered traffic to that
subscriber; it would not identify the original publisher. The
[Mosquitto client manual](https://mosquitto.org/man/mosquitto_sub-1.html)
documents these output fields. No clean extended run or fix is established.

#### Follow-up: ten verified echoes without failures

A later maintainer capture used client `iosnetbd19c00cd4898c0a` with
`mfw_nrf91x1_2.0.1`. It reported reuse of the stored CA at tag 42420,
roaming LTE-M registration and `test.mosquitto.org:8883` with
`TLS 1.2, CA/hostname verified`. The QoS 1 subscription used
`iosonata/demo/bd19c00cd4898c0a/echo`, followed by:

```text
MQTT QoS 1 echo checked 1, failures=0
MQTT QoS 1 echo checked 10, failures=0
```

This establishes ten verified QoS 1 self-echoes with zero reported failures
in the supplied run. The capture contains no extra-copy, rejection or
reconnect line, so it does not demonstrate the additional-copy path or
resolve the earlier publication origin and console truncation. An extended
run, deliberate recovery and negative CA/hostname checks remain unestablished
by this capture.

### Hardware evidence for the generic MQTT example

The maintainer's first hardware log for the new `MqttDemo` reports nRF91x1
with modem firmware `mfw_nrf91x1_2.0.1`. It registered on roaming LTE-M,
reported network ready and connected to `test.mosquitto.org:1883` over
plaintext TCP. The client ID was `iosnet0201acf5a919bd59` and the topic was
`iosonata/demo/0201acf5a919bd59/echo`. A QoS 1 echo was checked at counter 1
with `failures=0`.

This confirms one successful MQTT round trip for the generic application's
LTE path on the tested nRF91x1 hardware. Recovery, an extended run and TLS
are not established by this log. No fresh automated ARM compile/link result
is claimed.

### Hardware evidence for the generic CoAP example

The maintainer's first hardware log for the new `CoapDemo` reports nRF91x1
with modem firmware `mfw_nrf91x1_2.0.1`. The startup banner identified
`Generic CoAP over LTE UDP` and the public read-only GET profile. The modem
registered on roaming LTE-M, and the application reported network ready.
The console then showed:

```text
CoAP UDP peer coap.me:5683, plaintext, interval 10000 ms
CoAP public GET checked 1 code=2.05 bytes=66 failures=0
CoAP public GET smoke test verified
```

The maintainer subsequently supplied the continued log through checked
counter 100. The printed checkpoints at 1 and every 10 responses through
100 all reported 2.05 Content, 66 payload bytes and `failures=0`. The last
checkpoint was:

```text
CoAP public GET checked 100 code=2.05 bytes=66 failures=0
```

This confirms 100 application-checked public CoAP GET responses over LTE UDP
on the tested nRF91x1 hardware, with zero reported application failures and
a configured 10000-millisecond request interval. The log contains no elapsed
timestamps or packet/retransmission statistics. It does not establish the
full controlled-helper exercise, deliberate loss/recovery behavior or
long-run stability. The test uses plaintext UDP; DTLS and OSCORE are not
implemented. No fresh automated ARM compile/link result is claimed.

### Hardware evidence from the original examples

The maintainer supplied these MQTT-over-LTE results for the existing
`LteMqttDemo` application:

| Target | Modem firmware | Observed result |
| --- | --- | --- |
| nRF9160 | `mfw_nrf9160_1.2.8` | Roaming LTE-M, `test.mosquitto.org:1883`, QoS 1 subscription and verified echoes at 1 and 10, both with `failures=0` |
| nRF91x1 | `mfw_nrf91x1_2.0.1` | Roaming LTE-M, `test.mosquitto.org:1883`, QoS 1 subscription and verified echo at 1 with `failures=0` |

The nRF91x1 log identified client `iosltee282c3c1af8e35d7` and topic
`iosonata/lte/e282c3c1af8e35d7/echo`. That confirms the original LTE example
and its shared MQTT/socket path on the tested nRF91x1 hardware. The log does
not establish an extended run, recovery test, TLS test or a run of the new
generic application.

H753 MQTT/TLS results likewise belong to the original Ethernet example.
These records do not substitute for testing the new `MqttDemo` and `CoapDemo`
applications. The earlier intermittent H753 TLS issue remains unresolved;
the confirmed port-mismatch incident did not establish its cause or fix.
