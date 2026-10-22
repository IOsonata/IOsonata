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

The foreground application pump runs every 20 milliseconds. The nRF91 socket
port allows 30 seconds for each address's TCP connection attempt, then applies
this demo's 100-millisecond send timeout after connection. DNS resolution has
its own modem blocking wait; the complete socket initialization can take longer
than 30 seconds when name lookup or another address attempt is needed. If IPv6
attempts fail and no IPv4 address has already been tried, the port explicitly
resolves and tries IPv4. It does not assume the first resolver result contains
both address families.

MQTT protocol operations and the matching PUBACK/echo have 30-second deadlines.
A refused subscription or a grant below QoS 1 fails this QoS 1 test, although
the shared client also supports QoS 0 applications.

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
MQTT TCP connecting: test.mosquitto.org:1883
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

### Diagnosing socket initialization failures

The application prints `MQTT TCP connecting` before socket initialization. An
initialization failure is now labeled `socket initialization`; the subsequent
`mqtt_error` and `mqtt_result` fields describe MQTT state, so values of zero do
not explain why the native socket failed. The earlier log labeled this failure
`DNS or TCP connect` and used the shorter fields `error` and `result`.

When the existing system logger is enabled, native failures have this form:

```text
nRF91 socket <operation> failed: family=<family> result=<return-value> errno=<error>
```

The port saves the error before cleanup. Operations include `DNS`, `create`,
`bind`, `connect` and `poll callback`. The operations `connect timeout` and
`TX timeout` identify a failure to set the corresponding socket option; an
actual connection failure is reported as `connect`. Keep these lines with the
application failure and LTE registration log. A failed address attempt can be
followed by a successful fallback; inspect the final connection/subscription
result as well.

The earlier port applied the demo's 100-millisecond send timeout before
`nrf_connect()`. Nordic's modem socket also uses that option for TCP connection
setup, so it could stop connection establishment much earlier than intended.
The port now uses a 30-second connection timeout per address and applies the
configured send timeout afterward. It also explicitly retries IPv4 after
unsuccessful IPv6-only resolver results. The hardware results below include
the initial failures and a successful retest after these corrections.

## Validation status

The shared MQTT client has existing host wire, local Mosquitto and native
TLS/MQTT tests described in `docs/networking.md`. H753 MQTT hardware results and
Thingy:91 LTE UDP hardware results are separate from the nRF9160 MQTT-over-LTE
result recorded below.

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

`python3 tests/net/run_nrf91_socket.py` passed the actual nRF91 socket port and
generic `DeviceIntrf` implementation at O0/O2 with ASan/UBSan. Host Nordic API
substitutes exercise a 750-millisecond connection with a 100-millisecond send
wait, address-family fallback, failure diagnostics, cleanup and retries. The
earlier port fails the same connection-timeout check. These tests do not
establish an ARM build, modem firmware compatibility or cellular connectivity.

The shared `tests/net/run_mqtt.py` wire suite also passed at O0/O2, including
the C header check. Its optional Mosquitto interoperability checks were skipped
in that host test session because the executable was unavailable; that suite
did not produce a fresh live broker result.

The public broker setting is taken from its published service documentation.
A live public probe could not resolve the host in the implementation environment,
so public broker availability was not verified there. The maintainer's later
board test did reach the broker, as recorded below.

### nRF9160 hardware results

The maintainer ran `LteMqttDemo`. The console reported modem
firmware `mfw_nrf9160_1.2.8` and successful roaming LTE-M registration, followed
by three initial socket-initialization failures. That run did not demonstrate
MQTT subscription or verified self-echo. The reported
`error=0 result=0` values were MQTT state, not the modem's DNS or TCP error.
The log establishes a running application and LTE registration; it does not
identify the failing native socket operation or establish a firmware-version
cause.

After the socket corrections in commit `68aae178`, the maintainer retested with
the same modem firmware, `mfw_nrf9160_1.2.8`, and a 115200-baud console. The board
registered on roaming LTE-M, connected to `test.mosquitto.org:1883`, printed its
generated client ID and topic, and subscribed at QoS 1. The console then showed:

```text
MQTT QoS 1 echo checked 1, failures=0
MQTT QoS 1 echo checked 10, failures=0
```

This confirms successful plaintext MQTT over LTE for that run, including the
QoS 1 subscription, outgoing PUBACKs and matching returned payloads. The log
does not identify the cause of the initial failures or separate the effects of
the connection timeout and IPv4 fallback changes. Recovery after broker or
registration loss, an extended soak and TLS are not established by this log.
No fresh automated ARM compile/link validation is claimed.
