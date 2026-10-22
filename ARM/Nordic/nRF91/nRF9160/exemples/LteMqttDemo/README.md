# LteMqttDemo on Nordic Thingy:91

This bare-metal example tests the shared IOsonata MQTT 3.1.1 client through the
nRF9160 modem's TCP socket. The default board is Nordic Thingy:91; an nRF9160 DK
pin selection is also provided. The application is `exemples/lte/lte_mqtt.cpp`.
The protocol implementation remains `include/net/mqtt.h` and `src/net/mqtt.cpp`.

The default peer is **test.mosquitto.org, TCP port 1883**. No local PC server or
LTE inbound port forwarding is needed. The SIM must provide an Internet data
connection with DNS and outbound TCP access to that broker.

## What the example checks

The application generates a 16-hex-digit suffix using the existing Nordic RNG.
It uses that suffix in the 22-character client ID `ioslte<suffix>` and the topic
`iosonata/lte/<suffix>/echo`. It subscribes to that exact topic at QoS 1, then
publishes a 26-byte synthetic payload: `IOsonata LTE MQTT ` followed by an
eight-digit hexadecimal sequence number. The first publication follows the
successful subscription immediately; later publications are at least 10 seconds
apart. It checks the returned topic and payload and counts a success only after
both the outgoing PUBACK and self-echo have completed. The topic and client ID
remain stable across reconnects within that boot and are printed on the console
so a second client can observe the test if needed.

The random suffix reduces accidental client/topic collisions between boards.
The broker is public: another client can read or publish on the topic. The test
uses no IMEI, IMSI, SIM identifier, password or sensor data in its payload.
It does not retain publications. Use its returned-message counter to confirm
end-to-end delivery; successful TCP writes or MQTT enqueueing alone are not a
pass.

The [broker's official page](https://test.mosquitto.org/) identifies port 1883
as unencrypted and unauthenticated. It is a community test service and may
restart, be slow or be unavailable. The example reconnects and subscribes again.
Use a broker you control for deliberate server-restart tests and reproducible
recovery measurements.

This demo has **no TLS**. The public service's encrypted listeners use X.509
certificates. IOsonata's native TLS client currently implements external PSK,
not certificate authentication, so changing `MQTT_DEMO_PORT` to 8883 does not
enable a compatible secure connection. The nRF91 modem has separate TLS and
credential operations, but configuring and validating those is a separate test.

## Same protocol on Ethernet and LTE

`Mqtt : Device` receives an already-connected `DeviceIntrf *`; its C functions
operate on the same protocol state through `DevIntrf_t *`. The application
injects the existing `SockIntrf` configured for TCP. On nRF91, that interface
uses the modem's IP stack. On H753, the corresponding interface uses shared
lwIP over the Ethernet driver. The MQTT protocol source is the same.

A LAN broker and an Internet broker therefore require different addressing,
network setup and sometimes timeout settings, rather than separate MQTT
implementations. All application, socket and protocol processing belongs to one
foreground owner. The board header holds configuration only.

This portability does not transfer a live session between networks. After a
connection loss or interface change, close the old transport, reset MQTT, open
the new connection and initialize MQTT again. The current MQTT profile uses
clean sessions, so subscribe again before publishing. If an authenticated TLS
interface is injected, complete its handshake before initializing MQTT. Pending
application data needs an explicit retry/delivery policy across reconnects.

## Configuration and console

Edit `src/board.h` beside this README:

| Setting | Default | Meaning |
| --- | --- | --- |
| `MQTT_DEMO_HOST` | `"test.mosquitto.org"` | Broker hostname or address, without a URL scheme |
| `MQTT_DEMO_PORT` | `1883` | Plaintext TCP listener |
| `MQTT_DEMO_INTERVAL` | `10000U` | Minimum interval between synthetic publications, in milliseconds |
| `LTE_MQTT_APN` | `nullptr` | Network default APN; use a string if required by the SIM |
| `LTE_MQTT_RAT` | `LTE_RAT_LTEM` | LTE-M radio access |
| `LTE_MQTT_RAT_PREF` | `LTE_RAT_LTEM` | LTE-M preference |

The Thingy:91 console is nRF9160 UART0, RX P0.19 and TX P0.18, at **115200 baud,
8N1, no flow control**. Use the USB serial port that its nRF52840 Connectivity
Bridge or HciController firmware forwards from UART0. The demo does not start
BLE or an HCI host; retain the working companion console firmware.

For the nRF9160 DK, select `NRF9160_DK` instead of `NORDIC_THINGY91` in the
board header. UART0 then uses RX P0.28 and TX P0.29 through VCOM0. RTC0
(`LTE_TIMER_DEVNO = 0`) supplies modem waits on trigger 0 and the application
pump on trigger 1.

The example disables requested PSM, eDRX and release assistance for this first
continuous MQTT check. Low-power scheduling and MQTT keepalive need a separate
validation. The KeepAlive value is 60 seconds.

The foreground application pump runs every 20 milliseconds. Socket sends wait
at most 100 milliseconds; modem DNS resolution and TCP connection setup have
their own blocking waits. MQTT protocol operations and the matching PUBACK/echo
have 30-second deadlines. A refused subscription or a grant below QoS 1 fails
this QoS 1 test, although the shared client also supports QoS 0 applications.

## Build and run

Keep the existing IOsonata SDK layout and Nordic modem dependencies. Import:

1. `ARM/Nordic/nRF91/nRF9160/lib/ioc`, named `IOsonata_nRF9160`.
2. `ARM/Nordic/nRF91/nRF9160/exemples/LteMqttDemo/ioc`, named `LteMqttDemo`.

Build the library first, then the demo using matching `Debug` or `Release`
configurations. Use the bare-metal configurations, not the TaktOS variants.
The project retains `NRF9160_XXAA`, Cortex-M33 Thumb hard-float, GNU C++23,
no exceptions and no RTTI, and links `IOsonata_nRF9160` plus Nordic's `modem`
library. The existing include/library paths use `external/nrfx` and
`external/sdk-nrfxlib/nrf_modem`, including its
`lib/cellular/nrf9160/hard-float` library directory. There is no lwIP dependency
for this modem socket transport.

Use the existing secure-image configuration. The selected
`CryptoRngNrfInstance()` uses the CryptoCell RNG through `Cc3xxIntrf`, which
requires secure access. Do not define `NRF_TRUSTZONE_NONSECURE` in either build.
Both the nRF9160 and nRF91x1 libraries include the shared MQTT source; this
particular board/application project targets nRF9160.

Program the nRF9160 using the normal Thingy:91 setup and open the console before
reset. Confirm the modem firmware, LTE registration, broker connection,
subscription and verified returned messages. A public broker check needs no
Mosquitto process on the PC.

Expected console lines have this form; the suffix is generated at boot:

```text
MQTT TCP peer test.mosquitto.org:1883, plaintext, client ioslte<16-hex-suffix>
MQTT subscribed: iosonata/lte/<16-hex-suffix>/echo QoS=1
MQTT QoS 1 echo checked 1, failures=0
```

The first verified exchange on each connection is logged, then every 10 checked
messages globally. Old QoS 1 duplicates are acknowledged without incrementing
the count; retained or unrelated-topic messages are not accepted as live echoes.

To observe the topic from a PC with Mosquitto clients installed, replace
`SUFFIX_FROM_CONSOLE` below with the exact suffix printed by the board:

```sh
mosquitto_sub -h test.mosquitto.org -p 1883 \
  -t 'iosonata/lte/SUFFIX_FROM_CONSOLE/echo' -q 1 -v
```

Keep this observer read-only during payload verification. It is not needed for
the board to receive its own publication. Avoid broad wildcard subscriptions
on the public service.

## Recovery and delivery limits

After registration loss or a closed socket, the application reconnects and
subscribes again before continuing synthetic traffic. It waits 30 seconds after
failure handling completes and requires registration before reopening TCP.
Failed modem initialization and modem faults also use a 30-second retry. The
example uses the current clean-session client, not persistent MQTT sessions or a durable publication
queue. QoS 1 can deliver duplicates and does not by itself establish exactly-once
application delivery across reconnects.

For a controlled recovery test, point `MQTT_DEMO_HOST` at a reachable broker you
own and stop/restart that broker. A PC's private LAN address such as
`10.5.5.51` is not directly routable from ordinary cellular Internet; supply an
explicit route, VPN/private APN arrangement or a suitable public server address.
Record the console error, subscription recovery and first verified returned
message without resetting the board. Also record registration loss/recovery
where the test setup permits it.

Keep the modem firmware, SIM/APN, build profile, broker endpoint and elapsed
recovery time with the results. Public service interruptions are not evidence
of a protocol defect without the surrounding transport/server observations.

## Validation status

The shared MQTT client has existing host wire, local Mosquitto and native
TLS/MQTT tests described in `docs/networking.md`. H753 MQTT hardware results and
Thingy:91 LTE UDP hardware results are separate evidence; neither establishes
MQTT-over-LTE hardware operation.

`python3 tests/net/run_lte_mqtt.py` passed host syntax with the actual Thingy:91
board header and Nordic RNG declaration, using an empty `nrf.h` host stub.
The actual application and shared MQTT implementation passed O0/O2 ASan/UBSan
checks for 1/7/1024-byte stream fragments, both PUBACK/self-echo arrival orders,
duplicate/stale/retained/unrelated messages, malformed or QoS 0 echoes,
subscription downgrade/refusal, sequence/clock wrap and blocked-I/O deadlines.
Recovery checks cover socket/registration/modem loss with fresh CONNECT and
SUBSCRIBE, retry timing after blocking calls finish, random identity failure
and stability, and application queue refusal. Hardware interfaces are replaced
by host test implementations.

The shared `tests/net/run_mqtt.py` wire suite also passed at O0/O2, including
the C header check. Its optional Mosquitto interoperability checks were skipped
in this session because the executable was unavailable; no fresh live broker
result is claimed.

The public broker setting is taken from its published service documentation.
A live public probe could not resolve the host in the implementation environment,
so current public broker availability was not verified there.

No fresh ARM compile/link or nRF9160 MQTT hardware result is claimed by this
example. Record the maintainer's build and board output after running it.
