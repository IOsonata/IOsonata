# Generic MQTT and CoAP examples

`MqttDemo` and `CoapDemo` demonstrate the same protocol application using a
selected `DeviceIntrf *`. The target project supplies board configuration;
`exemples/net/net_demo.cpp` initializes the selected network and opens its
interface. The MQTT and CoAP applications use the returned pointer without
depending on the modem or Ethernet driver.

These are new examples. The existing `LteMqttDemo`, `LteCoapDemo`,
`EthernetCoapDemo` and `EthernetTcpDemo` applications remain available with
their original settings and validation records.

## Projects and files

| Target | Library | MQTT project | CoAP project | Default medium |
| --- | --- | --- | --- | --- |
| nRF9160, Thingy:91 or DK | `IOsonata_nRF9160` | [MqttDemo](../ARM/Nordic/nRF91/nRF9160/exemples/MqttDemo/README.md) | [CoapDemo](../ARM/Nordic/nRF91/nRF9160/exemples/CoapDemo/README.md) | LTE-M |
| nRF9151 DK or nRF9161 DK | `IOsonata_nRF91x1` | [MqttDemo](../ARM/Nordic/nRF91/nRF91x1/exemples/MqttDemo/README.md) | [CoapDemo](../ARM/Nordic/nRF91/nRF91x1/exemples/CoapDemo/README.md) | LTE-M |
| NUCLEO-H753ZI | `IOsonata_STM32H753` | [MqttDemo](../ARM/ST/STM32H7xx/STM32H753/exemples/MqttDemo/README.md) | [CoapDemo](../ARM/ST/STM32H7xx/STM32H753/exemples/CoapDemo/README.md) | Ethernet with DHCP |

Each project links these shared example files:

| File | Responsibility |
| --- | --- |
| `exemples/net/mqtt_generic.cpp` | MQTT identity, subscription, publication, payload checks and session recovery |
| `exemples/net/coap_generic.cpp` | CoAP public GET or controlled-helper requests, response checks and exchange recovery |
| `exemples/net/net_demo.cpp` | Console, event queue, timer, RNG, selected medium and connection setup |
| `exemples/net/net_demo.h` | Example setup functions and custom-medium hooks |
| Target `src/board.h` | Pins, medium selection, timing, RNG provider and peer configuration |

`net_demo.h` is an example helper, not a new public library interface. The
existing protocol implementations remain `src/net/mqtt.cpp` and
`src/net/coap.cpp`. Their C functions operate on caller-owned protocol state;
the C++ `Mqtt : Device` and `Coap : Device` wrappers hold that same state and
accept `DeviceIntrf *`.

## Build and start

1. Import the selected MCU's `lib/ioc` project into IOcomposer.
2. Import its `exemples/MqttDemo/ioc` or `exemples/CoapDemo/ioc` project.
3. Configure the application's `src/board.h` for the board and peer.
4. Build the library first, then the application with matching Debug or Release
   configurations. For Nordic, use the bare-metal library configuration.
5. Program the intended MCU and reset it with the diagnostic console open.

The three target variants use the same IOcomposer project names. Use separate
workspaces when keeping several variants of `MqttDemo` or `CoapDemo` imported.
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
handshake before presenting it as connected to MQTT. The supplied projects
use plaintext TCP and UDP; they do not enable TLS, DTLS or OSCORE.

## MQTT check

All supplied targets default to the same settings:

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
`--target nrf9160|nrf91x1|stm32h753|all`. The complete default matrix passed:

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

### Hardware evidence for the generic MQTT example

The maintainer's first hardware log for the new `MqttDemo` reports nRF91x1
with modem firmware `mfw_nrf91x1_2.0.1`. It registered on roaming LTE-M,
reported network ready and connected to `test.mosquitto.org:1883` over
plaintext TCP. The client ID was `iosnet0201acf5a919bd59` and the topic was
`iosonata/demo/0201acf5a919bd59/echo`. A QoS 1 echo was checked at counter 1
with `failures=0`.

This confirms one successful MQTT round trip for the generic application's
LTE path on the tested nRF91x1 hardware. CoAP, recovery, an extended run and
TLS are not established by this log. No fresh automated ARM compile/link
result is claimed.

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
