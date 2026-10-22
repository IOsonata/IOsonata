# Shared networking protocols and connected sockets

`SockIntrf` presents connected UDP and TCP through `DeviceIntrf`, with C helpers
and a C++ wrapper. nRF91 uses its modem's IP stack. STM32H753 now uses the shared
`src/net/ip_lwip.cpp` service over an injected `EthernetDev_t`. Protocol clients
can use the same socket interface on either target. Public socket signatures
are unchanged.

NUCLEO-H753ZI hardware tests confirm DHCP, ping, UDP/TCP integrity, public
DNS/HTTP and TCP recovery after server restart and cable reconnection. See the
hardware results below
for the tested scope and remaining checks.

## Protocols across Ethernet, LTE, LAN and WAN

The shared protocol owns protocol state; the injected interface moves its
bytes. `Mqtt : Device` accepts `DeviceIntrf *`. The C functions operate on
caller-owned `MqttDev_t` state through `DevIntrf_t *`; the C++ wrapper contains
that same C state structure. `Coap : Device` follows the same pattern with a
datagram transport. Neither client derives from an Ethernet
controller or LTE modem, and neither contains board or MCU register access.

| Component | Implementation used by the application |
| --- | --- |
| MQTT 3.1.1 | Shared `src/net/mqtt.cpp` over an already-connected stream |
| CoAP | Shared `src/net/coap.cpp` over an already-connected UDP interface |
| Native TLS 1.3 PSK | Shared `Tls : DeviceIntrf` over a TCP interface, authenticated before MQTT starts |
| Ethernet socket | `SockIntrf` implemented by shared `src/net/ip_lwip.cpp` over the MCU Ethernet driver |
| nRF91 socket | `SockIntrf` implemented by `ARM/Nordic/nRF91/src/sock_intrf_nrf91.cpp` through the modem |

An application waits for DHCP/IP readiness or LTE registration, opens the
appropriate socket, and injects it into the shared protocol. A local server and
a public server use the same protocol code. The application supplies a reachable
address, port, routing/APN settings and suitable timeouts. The same transport
rules still apply across a LAN or WAN: TCP is a byte stream; UDP preserves
datagram boundaries. These LTE examples do not modify the shared MQTT or CoAP
implementation or introduce modem-specific protocol code.

A connection change requires a new session. Close the old socket, reset the
protocol context, bring up the selected network and open a new socket. If TLS
is selected, authenticate that connection before initializing MQTT, then
subscribe again because the current MQTT profile uses clean sessions. Changing
an interface pointer cannot move an existing TCP/TLS/MQTT session to a different
network. Application delivery state must handle any interrupted publication.
For CoAP, create a fresh UDP endpoint after reset and carry the remaining peer
probing wait across the change. The current target library builds select their
respective `SockIntrf` implementation. Using Ethernet and LTE simultaneously in
one firmware would require distinct compatible transport implementations and
application selection between them. `SockIntrf` does not currently provide a
network selector or automatic failover.

### nRF91 socket initialization

The nRF91 port gives each address's TCP connection attempt a 30-second timeout,
then applies `SockIntrfCfg_t::Timeout` to sends on the connected socket. The LTE
MQTT example uses a 100-millisecond send wait. Applying that short wait before
`nrf_connect()` also limited connection establishment, because the modem uses
`NRF_SO_SNDTIMEO` for TCP connect. DNS resolution remains a separate modem wait;
30 seconds is a per-address connection limit, not an overall initialization
deadline.

The modem's `NRF_AF_UNSPEC` lookup can return IPv6 results without IPv4 results,
even when the bearer cannot use IPv6. If IPv6 attempts fail and no IPv4 address
has already been tried, the port explicitly looks up and tries IPv4. With the
existing system logger enabled, native failures use the format
`nRF91 socket <operation> failed: family=<family> result=<return-value> errno=<error>`.
The port saves the error before cleanup. These messages identify DNS, socket
creation, options or connect failures separately from the shared MQTT state.
An address-level failure can still be followed by a successful fallback.

Nordic documents the [TCP connection timeout behavior](https://devzone.nordicsemi.com/f/nordic-q-a/92085/how-to-set-a-udp-socket-peer-connect-timeout)
and the [IPv6-first resolver behavior in its v2.7.0 socket API](https://github.com/nrfconnect/sdk-nrfxlib/blob/v2.7.0/nrf_modem/include/nrf_socket.h).

`python3 tests/net/run_nrf91_socket.py` passed at O0/O2 with ASan/UBSan, using
the actual nRF91 socket port and generic `DeviceIntrf` implementation with
host Nordic API substitutes. A simulated 750-millisecond connection succeeds
with a 100-millisecond send wait; the earlier port fails that same check.
The tests also cover IPv4 fallback, working IPv6, per-stage diagnostics with
the original errno, resource cleanup, socket-table exhaustion and retries.
They do not establish an ARM build, modem firmware compatibility or a working
cellular connection. The maintainer's subsequent nRF9160 MQTT hardware retest
succeeded; its observed scope is recorded below.

### LTE example targets

Both target families use the shared `exemples/lte/lte_mqtt.cpp` and
`exemples/lte/lte_coap.cpp` applications, the shared protocols and the nRF91
modem socket port. Their projects select the target library and board pins.

| Target | Library | MQTT example | CoAP example | Default UART0 console |
| --- | --- | --- | --- | --- |
| nRF9160, Thingy:91 | `IOsonata_nRF9160` | [LteMqttDemo](../ARM/Nordic/nRF91/nRF9160/exemples/LteMqttDemo/README.md) | [LteCoapDemo](../ARM/Nordic/nRF91/nRF9160/exemples/LteCoapDemo/README.md) | RX P0.19 / TX P0.18 through the nRF52840 USB bridge |
| nRF9151 DK or nRF9161 DK | `IOsonata_nRF91x1` | [LteMqttDemo](../ARM/Nordic/nRF91/nRF91x1/exemples/LteMqttDemo/README.md) | [LteCoapDemo](../ARM/Nordic/nRF91/nRF91x1/exemples/LteCoapDemo/README.md) | RX P0.26 / TX P0.27 through VCOM0 |

The consoles use 115200 baud, 8N1 and no flow control. The nRF9160 projects also
provide an nRF9160 DK pin selection. The nRF91x1 projects follow that target's
existing LteUdp settings, including `NRF9120_XXAA`, its `nrf9120` linker script
and hard-float modem library. No nRF91x1 ARM build or hardware result is claimed;
the successful MQTT result below applies to nRF9160.

## Layers and memory

| Layer | Responsibility |
|---|---|
| `include/net/sock_intrf.h` | Connected socket configuration and transfers |
| `src/net/ip_lwip.cpp` | Shared IPv4, ARP, ICMP echo, DHCP, DNS, UDP and TCP using lwIP raw callbacks |
| `include/net/ethernet.h` | Portable Ethernet frame interface, C structure/functions and C++ wrapper |
| `ARM/ST/STM32H7xx/src/ethernet_stm32h7.cpp` | H753 MAC, DMA, cache maintenance, MDIO and LAN8742A RMII negotiation |
| Example `board.h` | Pins, PHY address, MAC address and console configuration |

The lwIP profile uses `NO_SYS=1`, fixed packet/PCB pools and no libc heap.
One foreground owner must serialize all IP and socket calls. The default
bare-metal `AppRun()` is suitable. An RTOS application must use one worker to
own this service and its sockets; concurrent calls from different tasks are
not supported. This does not require an RTOS or a BSD socket layer.

Initialize the application event queue, timer and Ethernet interface before
`IpInit()`. Supply an already-running timer and reserve one trigger for the
10 ms IP service. The service temporarily owns the Ethernet event callback.
Timer and Ethernet interrupts only request foreground work through the event
queue. `IpCheckStatus()` retries a refused event; the default `AppCheckStatus()`
calls it weakly. Applications overriding that function must include the check.
No lwIP processing runs in an interrupt handler.

Call `SockIntrfInit()` after `IpReady()` becomes true. Initialization resolves the
host and connects TCP within the configured timeout; UDP selects a remote peer.
These waits service IP traffic but defer socket callbacks until a subsequent
foreground `IpProcess()` call, after initialization or the transfer has returned.
They do not dispatch unrelated application events. This lets receive callbacks
read queued data after the socket's transfer lock has been released.
Socket event callbacks run in the foreground IP service and must return promptly.
Do not initialize another socket or perform blocking work inside a callback.

`Tx()` returns bytes accepted, with TCP allowed to accept a prefix; retry the
remaining bytes. It does not mean the peer received them. UDP accepts a complete
datagram or returns zero. `Rx()` is nonblocking. TCP preserves unread stream
bytes; UDP returns one datagram and discards a truncated remainder. A zero read
can mean no queued data; inspect `Connected()` for connection loss. Buffered data
can still be read after loss of the connection. Link loss, DHCP lease expiry and
local address changes disconnect sockets; applications close and initialize
them again after the network recovers.

`IpClose()` releases the timer trigger, closes sockets, removes the network
interface and restores its callback. Close IP before closing its Ethernet
interface or timer. `IpInit()` can subsequently start the service again.
Other applications do not link the IP stack or DMA storage unless they use it.

## Current limits

- One Ethernet interface and four connected TCP/UDP socket objects.
- IPv4 only; Ethernet MTU 1500, maximum UDP payload 1472 bytes. No IP
  fragmentation/reassembly, IPv6, multicast membership or listening sockets.
- Four received packet chains per socket. TCP applies receive backpressure;
  UDP drops new datagrams when its receive queue is full.
- Four asynchronous DNS query contexts. A timed-out query remains detached
  until lwIP finishes it, so a late answer cannot change a replacement socket.
- Ethernet SockIntrf TLS/DTLS selections, credential storage and LTE
  release-assistance operations return false. The separate native `Tls` wrapper
  supports the PSK profile described below. nRF91 modem TLS/DTLS support is
  unchanged. Shared MQTT uses the selected stream; shared CoAP uses a connected
  UDP interface with the limited profile described below.
- H753 driver: MAC DevNo 0, LAN8742A PHY over RMII, auto-negotiated 10/100 Mbit/s,
  four RX and four TX descriptors. A fatal DMA bus error requires reinitialization.
  Other PHYs, MII and Ethernet power-management modes are not implemented.

Descriptors and frame buffers occupy a 12,544-byte `.eth_dma` section in AXI
SRAM at `0x24000000`, outside DTCM. Each descriptor occupies one 32-byte cache
line. The driver maintains descriptor/buffer cache lines if D-cache is enabled.
An application linker script must retain this section in DMA-accessible SRAM.
The supplied H753 linker script already does this. The MCU's normal data/stack
remain in DTCM; callers may pass DTCM buffers because transfers copy frames.

## Secure socket implementation status

`SockIntrfCfg_t` already selects TLS/DTLS, a security tag, peer verification
and a hostname. `SockIntrfCredWrite/Delete/Exists` already define credential
operations. The nRF91 implementation delegates these to the modem, including
hostname verification settings and modem credential storage. In contrast,
`src/net/ip_lwip.cpp` accepts only TCP/UDP and returns false for credentials.
Changing the protocol enum alone cannot enable encrypted Ethernet traffic.

The generic crypto interfaces already provide RNG, AES authenticated
encryption, SHA-256/HMAC, streaming hashes, P-256 agreement and signatures.
H753 supplies its existing RNG/CRYP/HASH providers and software P-256; WBA
has its own providers. Secure networking must reuse those interfaces and
keep controller access in the MCU ports.

### Native TLS 1.3 PSK client

`net/tls.h` and `src/net/tls.cpp` implement an initial external-PSK client.
The `Tls` DeviceIntrf wraps a connected TCP `SockIntrf`, following the layered
interface pattern. The existing socket TLS/DTLS protocol selections and credential
storage remain unsupported by lwIP; selecting that enum does not select this
client. The nRF91 modem's TLS/DTLS path is separate from this native client.

The client offers TLS 1.3 and `TLS_AES_128_GCM_SHA256`. With a non-null
`TlsCfg_t::pKeyAgree`, it requires `psk_dhe_ke` with a fresh P-256 key pair and
rejects a PSK-only server response. Without this engine it offers only `psk_ke`,
which has no forward secrecy and requires server support for PSK without DHE.
Neither mode implements X.509, certificate hostname verification, DTLS,
early data, session resumption, or HelloRetryRequest support. Unsupported
negotiation fails closed. It is an initial implementation, not a claim of
complete TLS feature coverage or an independently audited security library.

The caller supplies existing synchronous `HashEngine`, `MacEngine`, `AeadEngine`
and security-grade `RngEngine` instances. PSK-DHE additionally uses an enabled,
synchronous `KeyAgreeEngine` supporting P-256, configured with its secure RNG.
Its context must fit `CRYPTO_KEYCTX_MAX` and the alignment of `uint64_t` storage.
The engine validates the peer point; TLS clears its ephemeral context after
agreement and on failure/reset. No duplicate crypto implementation,
controller access, system allocation, RTTI or exceptions are introduced. Provider
errors, including BUSY, abort the connection; async providers are rejected.
PSKs contain 16..64 bytes, identities 1..128 bytes, and optional SNI at most
253 bytes. The application provisions and owns the credentials. SNI is a routing
hint; authentication is possession of the PSK. Use independently provisioned
high-entropy keys, not the public demo credential.

`TlsInit` consumes credentials and queues ClientHello. Call `TlsProcess` from
one foreground owner with a wrapping millisecond timestamp and current TCP-open
state. It performs at most one transport write and one read per call. Transport
calls must return synchronous byte counts; their own blocking duration still
applies. `Connected()` becomes true only after server Finished authentication
and submission of client Finished. Application data uses the normal `DeviceIntrf`
Rx/Tx helpers. Tx accepts at most 1024 plaintext bytes into its fixed output
buffer; retry the remainder. Accepted bytes are queued, not necessarily delivered.
This initial interface is polled; it does not publish TLS event callbacks.

All connection storage is in `TlsDev_t`: a 16645-byte input record buffer,
1046-byte output buffer, two 2048-byte handshake/transcript buffers, keys and
state, plus a fixed key-agreement context. The measured x86-64 context size is
22304 bytes; this is not an ARM RAM measurement and excludes provider objects
and stack. The receiver supports the
TLS 1.3 maximum ciphertext record size. Individual handshake messages and the
initial transcript are bounded to 2048 bytes. Oversized tickets/messages fail
rather than allocating. Received tickets are checked and discarded.

Handshake and pending-output deadlines use `TlsCfg_t::Timeout`. KeyUpdate is
handled, including requested responses. A conservative cap of 2^20 records per
traffic key aborts the connection before sequence reuse; the application can
reconnect. Bad authentication never exposes plaintext. Failed sessions wipe
record buffers, transcript and secrets, and require reinitialization. On failure
the caller must close the raw transport; this initial client does not send a
fatal TLS alert. `TlsClose` queues close_notify once output has drained;
continue processing until TLS_CLOSED, then close TCP. An unannounced TCP EOF
is an error, not an authenticated close. `TlsReset` wipes the context's sensitive
storage without closing the caller-owned transport.

Mbed TLS is not a dependency of this client. The earlier upstream Mbed TLS
installer addition has been removed; existing Nordic SDK and standalone PSA
crypto dependencies are unchanged. Existing external checkouts are not deleted.

Host validation: `python3 tests/net/run_tls.py` compiles the actual TLS and
software crypto sources at O0/O2 under ASan/UBSan. It checks fragmented records
and handshakes, malformed records, bad tags and Finished values, insecure/failed
RNG, deadlines including timer wrap, backpressure, EOF, key updates, size limits,
secret wiping, and the C header. OpenSSL 3 provides an independent live server
for 19/1024/4096/16000-byte reverse echoes, one-byte I/O, wrong-PSK rejection and
context reuse across connections in both PSK-only and P-256 PSK-DHE modes.
The O2 run also checks 20 consecutive PSK-DHE connections against one OpenSSL
process, verifying all four echo lengths on each connection.
DHE checks include malformed/duplicate shares, invalid peer points, missing
shares (downgrade rejection), key-context bounds, provider failure and wiping.
The host runner uses the existing micro-ecc checkout; `UECC_ROOT` can override
its location. The maintainer's H753 TLS hardware results are recorded below.

Certificate authentication, DTLS, broader protocol/error handling,
credential-tag integration and independent security review remain future work.
The shared protocol/crypto separation allows this component to move with the
communications code into Voci when the ecosystem is ready.

## Shared MQTT client

`include/net/mqtt.h` and `src/net/mqtt.cpp` implement MQTT 3.1.1 over an
injected, already-connected synchronous `DevIntrf_t`. The C API keeps all state
in caller-owned `MqttDev_t`; the C++ `Mqtt : Device` wrapper calls that same
implementation and accepts `DeviceIntrf *`. It is a protocol client, not another socket driver.
Use a TCP SockIntrf or an authenticated Tls interface. No controller, lwIP,
broker or crypto implementation is copied into MQTT.

The initial profile supports clean-session CONNECT/CONNACK, optional username
and binary password, QoS 0/1 publish and receive, retained publications,
one-filter SUBSCRIBE and UNSUBSCRIBE, acknowledgements, PINGREQ/PINGRESP and
DISCONNECT. Topic names and filters are checked for UTF-8 and wildcard placement.
QoS 2, persistent sessions, automatic retransmission across reconnects, Will
messages, MQTT 5 and MQTT-SN are not implemented. Subscriptions must be renewed
after reconnect. Do not interpret a successful enqueue as broker delivery.

The client has one output packet, one outstanding acknowledged operation and
one received-message view. Each packet's Remaining Length is limited to 1024
bytes, including topic and packet identifier overhead. Both buffers and all
state are embedded in the context; no heap, exceptions, RTTI or RTOS is needed.
Outgoing calls copy their arguments and return false when busy or invalid.
Malformed, unsupported or oversized received packets fail the session.

Call `Process(now, transportOpen)` frequently from the same foreground owner
as IP/TLS. It makes at most one write and one read per call; underlying
synchronous transport blocking and MaxRetry still apply. Set MaxRetry to zero
for bounded polling. When TLS is selected, pump TLS before MQTT and pass the
TLS readiness state. Never interleave raw application writes on that stream.

`Message()` returns a borrowed topic/payload view; the topic is length-delimited,
not NUL terminated. Consume it promptly, then call `Release()`. For QoS 1,
Release accepts ownership and permits PUBACK to be sent. Holding a message
applies backpressure and prevents processing later packets, including PINGRESP.
QoS 1 may deliver duplicates; the application must tolerate them. `Busy()`
remains true until output and any required acknowledgement/control reply have
completed. After SUBACK, `Result()` is the granted QoS or 0x80 for a refused
subscription. A refused subscription does not close the MQTT connection.

Timeout covers CONNECT, pending output/acknowledgement and a partially received
packet. Timestamps wrap as uint32_t milliseconds; Timeout must be 1..INT32_MAX.
KeepAlive is in seconds; zero disables it. Failure wipes buffered credentials
and messages. The caller closes the transport and initializes a fresh session.
Reset does not close an injected transport. Disable pauses only the C++ pump;
deadlines continue to elapse. DISCONNECT completion means accepted by the
transport: with TLS, finish flushing its output and perform TLS close_notify
before closing TCP.

`python3 tests/net/run_mqtt.py` checks wire fixtures, partial transfers, binary
payloads, QoS acknowledgements, malformed packets, UTF-8, buffer bounds,
keepalive, timeout wrap, EOF and wiping at O0/O2 with ASan/UBSan, plus the C header.
With Mosquitto installed (or MOSQUITTO pointing to it), it also runs a localhost
broker test. The TLS variant requires OpenSSL 3 and the existing micro-ecc
checkout (UECC_ROOT overrides its path). It uses an OpenSSL TLS 1.3 PSK-DHE
endpoint forwarding to Mosquitto, because the tested Mosquitto 2.0.18 PSK
listener does not accept TLS 1.3. This is a host test, not a production proxy.

The H753 EthernetTcpDemo includes an optional MQTT subscribe/self-publish test.
See its README for broker setup and firmware configuration. Plaintext and
TLS-protected MQTT echo, broker/cable recovery and wrong-PSK rejection passed
the hardware checks recorded below. The earlier intermittent TLS failure remains
unresolved. Both nRF9160 and nRF91x1
library projects now include the shared MQTT header and source.

### LTE MQTT public-broker example

[LteMqttDemo](../ARM/Nordic/nRF91/nRF9160/exemples/LteMqttDemo/README.md)
links `exemples/lte/lte_mqtt.cpp` and runs the same shared MQTT client over the
nRF9160 TCP socket. Its default board is Thingy:91 with the existing
115200-baud UART0 console bridge. The default peer is `test.mosquitto.org:1883`;
the [broker's published settings](https://test.mosquitto.org/) identify this as
an unauthenticated plaintext MQTT listener.

The application generates a random client suffix with the existing Nordic RNG,
subscribes at QoS 1 to its own `iosonata/lte/<suffix>/echo` topic, and publishes
synthetic test payloads every 10 seconds. It checks the returned topic/payload
and subscribes again after reconnecting. The suffix avoids ordinary client/topic
collisions; it does not make a public topic private. Use this for connectivity
checks, not confidential telemetry or broker performance measurements.

This demo does not enable TLS. The public broker's encrypted listeners use
X.509 certificates, which the native external-PSK TLS implementation does not
support. Changing the port to 8883 alone does not make this demo encrypted.
The nRF91 modem's separate credential/TLS operations remain available to an
application that explicitly configures and validates that path.

`python3 tests/net/run_lte_mqtt.py` passed host syntax with the actual Thingy:91
board header and Nordic RNG declaration using an empty `nrf.h` host stub.
O0/O2 ASan/UBSan checks use the actual application and shared MQTT client with
1/7/1024-byte stream fragments, both PUBACK/self-echo arrival orders, duplicate,
stale, retained, unrelated and malformed messages, QoS 0 rejection and
subscription downgrade/refusal. They also cover sequence/clock wrap, blocked
I/O and deadlines, random identity failure/stability, timer queue refusal and
socket/registration/modem recovery with fresh CONNECT and SUBSCRIBE. Target
interfaces are replaced by host test implementations.
The default target is `nrf9160`; `--target nrf91x1` selects the nRF9151/nRF9161
DK board configuration. That host variant also passed at O0/O2 with ASan/UBSan.

The shared MQTT wire suite and C header check also passed in this session.
Optional Mosquitto interoperability checks were skipped because the executable
was unavailable. Public-host DNS resolution was unavailable in the environment,
so these host checks add no live public-broker or fresh automated ARM
compile/link result. The maintainer's initial failed hardware run and successful
nRF9160 retest after the socket corrections are recorded below.

Protocol reference: [OASIS MQTT 3.1.1 plus Errata 01](https://docs.oasis-open.org/mqtt/mqtt/v3.1.1/errata01/os/mqtt-v3.1.1-errata01-os-complete.html).

## Shared CoAP client

`include/net/coap.h` and `src/net/coap.cpp` implement an RFC 7252 unicast client
over an injected, already-connected synchronous UDP `DevIntrf_t`. The C-callable
functions keep protocol state in caller-owned `CoapDev_t`; the C++
`Coap : Device` wrapper accepts `DeviceIntrf *` and calls those same functions.
As with MQTT, the shared
implementation is compiled from a `.cpp` file and has no separate C++ protocol
implementation. The client does not include controller access, lwIP or modem
calls.

### Supported operations and limits

The initial profile supports GET, POST, PUT and DELETE; confirmable (CON) and
non-confirmable (NON) requests; piggyback and separate responses; empty ACK;
Reset; and confirmable retransmissions. There is one outstanding request per
context (NSTART=1). Applications must coordinate multiple contexts addressing
the same server. Response codes in the 2.xx, 4.xx and 5.xx classes are delivered
as ordinary responses; an application decides which code it expects.

The client matches responses by the connected peer and its eight-byte token.
A piggyback ACK must also carry the current request message ID. An empty ACK
stops request retransmission while the client waits for a separate response.
Separate CON responses are acknowledged, including a CON response to a NON
request. A request using CON may also receive a separate NON response.

The complete datagram is limited to **1152 bytes**, including the four-byte
header, token, options and payload marker. There are at most **16 options**.
The receiver reads into a 1153-byte buffer so a truncated oversized datagram
cannot be accepted as a complete supported packet. State, the request/response
buffer, receive buffer and four duplicate records reside in the context. There
is no heap, exceptions, RTTI or RTOS dependency.

The client remembers the last four accepted CON response message IDs for up to
247 seconds. It acknowledges duplicates still in that cache without delivering
the response again. Older, evicted or expired unexpected CON responses may
receive Reset; the cache does not promise unlimited duplicate retention.

There is no DTLS, OSCORE, Observe, blockwise transfer, proxy, multicast, CoAP over
TCP or embedded server engine. The bounded `CoapEncode` and `CoapDecode` helpers
are also available for message encoding/decoding; they do not implement a
server. The native TLS stream wrapper cannot encrypt CoAP UDP traffic.

### Datagram transfers and ownership

Call `CoapInit` after the UDP socket is connected and set its `MaxRetry` to zero.
Each `RxData` must consume one datagram, discarding a truncated tail. Each
`TxData` must accept the complete datagram or return zero; negative asynchronous
results and short positive writes are unsupported and report a transport error.
The client pairs the normal StartRx/RxData/StopRx and StartTx/TxData/StopTx hooks
once per transfer. It does not accumulate reads or retry a partial write as if
the interface were a byte stream. The connected transport filters remote
addresses and ports; do not inject an unfiltered or shared receive stream.

`CoapProcess(now, transportOpen)` performs at most one datagram read and one
write per call. Call it frequently from the same foreground owner as IP, passing
the current socket `Connected()` result. Synchronous transport calls can still
block up to their configured timeout. Do not call the client from an ISR or
concurrent callback, and do not interleave another protocol's reads/writes on
its socket. There are no CoAP event callbacks in this initial implementation.

`Request()` copies options and payload before returning. Invalid requests return
false without starting an exchange; a busy rejection leaves the existing
exchange unchanged. A ready client can also be waiting for the peer's probing
rate limit; `RetryAfter(now)` reports that wait and `Request()` then returns
false with `COAP_ERROR_RATE`. `Response()` returns a borrowed view into
retained bytes.
Tokens, options and payloads are length-delimited, not NUL-terminated strings.
Consume the response before `Release`, `Cancel`, `Reset` or reinitialization.
Continue pumping while holding a response so duplicate CON packets can be
acknowledged. A pending ACK can keep `Busy()` true until the transport accepts
it, including after `Release()`.

`Release()` releases a response or failed exchange and preserves the peer's
message-ID, duplicate and probing-rate state for another request. `Cancel()`
abandons local request state but cannot undo an operation already sent. `Reset()` clears the
context without closing the caller-owned socket. A fresh initialization after
reset requires a fresh UDP endpoint, normally a closed/reopened socket with a
new ephemeral source port; do not reset message IDs on a live endpoint.
Carry any remaining `RetryAfter()` wait across endpoint replacement so a reset
does not bypass the rate limit for an unresponsive peer.
The C++ `Disable()` pauses the pump, while deadlines continue to elapse.

### Options, randomness and timing

Requests take an ordered array of raw option-value spans. Use one Uri-Path
option per path segment and one Uri-Query per query component. Values are
decoded bytes, not percent-encoded URI text, and an empty final Uri-Path
represents a trailing slash. The library does not hide a URI parser. Integer
request options use the shortest big-endian encoding; zero has an empty value.
Callers must supply valid normalized UTF-8 in string-valued request options.
The raw option helpers do not validate or normalize incoming text; applications
interpreting those spans as text must handle that themselves.
The supported request options are If-Match, Uri-Host, ETag, If-None-Match,
Uri-Port, Uri-Path, Content-Format, Uri-Query, Accept and Size1, with their
RFC-defined lengths and repetition rules. Uri-Path segments `.` and `..` are
rejected; normalize the path before constructing its options.

Responses expose ETag, Location-Path, Content-Format, Max-Age, Location-Query,
Size1 and unknown elective options. Unknown or invalid critical response
options fail the exchange; a CON response is rejected with Reset. Invalid or
misplaced elective options and surplus repetitions of non-repeatable elective
options are ignored. Response integer options may include leading zero bytes.
Unsupported reserved Location extensions (options 128, 132, 136 or 140 together
with Location-Path/Location-Query) reject the response under
[RFC erratum 4949](https://www.rfc-editor.org/errata/eid4949).
Malformed or unrelated packets do not replace a valid response or complete the
pending request. Oversized matching responses report a limit error.

The caller supplies an already-enabled synchronous instance of the existing
`RngEngine`. It generates the initial message ID, eight-byte request tokens and
the randomized initial retransmission interval. Provider errors report
`COAP_ERROR_RANDOM`; asynchronous providers are rejected. The H753 demo reuses
the existing STM32 hardware RNG. Plaintext CoAP has no peer authentication.
Message IDs advance through the 16-bit space. After exhausting that space, the
client requires a full 247-second quiet period after the last ID allocation or
request transmission before reusing IDs on that endpoint.

A CON request starts with a randomized 2..3-second acknowledgement timeout.
Up to four retries double that interval, preserving the same datagram, message
ID and token. Retransmissions end after the 45-second maximum transmit span;
the full acknowledgement wait can reach 93 seconds. An empty ACK ends those
retries but leaves the response deadline active. `CoapCfg_t::Timeout` bounds the
entire request from enqueue, including local send backpressure and separate
response waiting, and may shorten the exchange. It must be 1..INT32_MAX
milliseconds. Timestamps use wrapping `uint32_t` milliseconds. The H753 demo
sets 95 seconds so the normal confirmable retry schedule can finish.

`CoapCfg_t::ProbingRate` limits new requests to an unresponsive peer in bytes per
second; zero selects the RFC default of **1 byte/second**. Accepted request
datagrams and retransmissions accrue a wait measured from the first send.
Matching responses or Reset establish that the peer is responsive and clear
that wait. Timeout, `Release()` and `Cancel()` retain it. After a failed
exchange, wait for `RetryAfter(now)` to reach zero before another request;
the protocol continues to acknowledge duplicate responses while being pumped.
Applications using multiple contexts for one peer must coordinate their
combined traffic and preserve pending waits across resets.

The H753 demo explicitly sets `COAP_DEMO_PROBING_RATE` to 1024 bytes/second for
its controlled Ethernet LAN, an application-specific parameter permitted by
RFC 7252 section 4.8.1. This is not the library default. Set it to zero to use
the default 1-byte/second probing rate. Normal replies clear the wait, so this
parameter does not limit traffic to a responsive server.

### CoAP examples and checks

[EthernetCoapDemo](../ARM/ST/STM32H7xx/STM32H753/exemples/EthernetCoapDemo/README.md)
links the shared
`exemples/net/coap_demo.cpp`. It checks GET/POST/PUT/DELETE, binary CON/NON echoes,
an expected 4.04 and a delayed separate response, then continues without
resetting the board after reported network/exchange failures. Its README gives
the build settings, PC command and expected console lines.

[LteCoapDemo](../ARM/Nordic/nRF91/nRF9160/exemples/LteCoapDemo/README.md)
links `exemples/lte/lte_coap.cpp` through the existing nRF9160 modem UDP socket.
Its default board is Thingy:91, with the existing 115200-baud UART0 USB console
bridge. `COAP_DEMO_PUBLIC_TEST = 1` sends read-only CON GET requests to
`coap.me:5683/test`; this public client-test service is described at
[coap.me/about](https://coap.me/about). It checks 2.05 Content and reports the
payload length without requiring a fixed public response body. This checks
DNS/UDP/CoAP reachability and does not replace the full helper exercise.

Set `COAP_DEMO_PUBLIC_TEST` to `0` and select a controlled, reachable helper
address for the same eight-step method/binary/separate-response exercise as
EthernetCoapDemo. The Ethernet PC's private `10.5.5.51` address needs an explicit
route or public UDP forwarding from ordinary cellular Internet. The default
request interval is 10 seconds, and unanswered-peer pacing retains the
library's 1-byte/second default. This is a standalone LTE test; it does not
start BLE.
The nRF9160 and nRF91x1 libraries include shared CoAP, and both target families
have dedicated demo projects listed above. The nRF9160 project uses its
existing secure-image CryptoCell RNG configuration. The nRF91x1 project reuses
the same application for the nRF9151 DK and nRF9161 DK; its ARM build and
hardware validation remain pending.

`Python/coap_server.py` uses the independent aiocoap implementation for the PC
peer. It serves `/hello`, `/echo`, `/value` and `/slow`, with plaintext UDP on
5683. Install `aiocoap==0.4.17` in the Python environment, then pass the PC's
explicit interface address with `--host`. Its default loopback address serves host
tests. The helper is a test resource server, not an embedded IOsonata server or
an encrypted deployment service.

Host validation passed at O0/O2 with ASan/UBSan using the actual CoAP,
`DeviceIntrf` and `Device` sources. The checks include the C header and C++
lifecycle, fixed wire fixtures, 8000 bounded parser inputs, malformed options,
MID/token matching, exact 1152-byte acceptance and 1153-byte rejection, copied
request data and stable held responses. They also exercise ACK backpressure,
retransmission timing and stalled pumps, wrapping clocks, cache expiry,
message-ID reuse guard, probing-rate waits across failure/release/cancel,
Location-option erratum handling, transport failures and transfer-lock release.

Independent aiocoap 0.4.17 tests passed over real localhost UDP, both directly
and through a packet-loss proxy. They cover GET/POST/PUT/DELETE, CON/NON binary
echoes up to 1000 payload bytes, 4.04 and an empty ACK followed by a separate
CON response. Dropping the first piggyback response causes a request retry.
Dropping the first ACK for a separate response causes a server retransmission,
which the client acknowledges after the application has released its response.

Run `python3 tests/net/run_coap.py --require-interop` with aiocoap installed to
repeat all checks. Without that flag, a missing aiocoap dependency skips the
live interoperability checks explicitly. The runner does not download
dependencies. A host C++23 syntax check of the application with the actual H753
board configuration also passed.

`python3 tests/net/run_lte_coap.py` passed the LTE application host syntax check
with the actual Thingy:91 board header and Nordic RNG declaration, using an
empty `nrf.h` host stub. Its O0/O2 ASan/UBSan checks cover registration-loss
close/reopen, pacing retained across repeated modem faults and clock wrap,
30-second failed-initialization retry, timeout recovery retaining a live UDP
endpoint, timer queue refusal/coalescing and empty-host startup without LTE.
Both the public GET and full helper profiles pass; public-profile fixtures also
check the configured path, variable response content, response-code/size errors,
recovery and 10-second request spacing. The default target is `nrf9160`;
`--target nrf91x1` selects the nRF9151/nRF9161 DK board configuration, which also
passed at O0/O2 with ASan/UBSan. These use substitutes for target hardware.
The public endpoints are published service settings; live probes
could not resolve them in the implementation environment, so no new public
reachability result is claimed. CoAP hardware results have not yet
been recorded, and no fresh ARM compile/link validation is claimed.

Protocol reference: [RFC 7252](https://www.rfc-editor.org/rfc/rfc7252.html).
Independent peer: [aiocoap 0.4.17](https://aiocoap.readthedocs.io/en/0.4.17/).

## NUCLEO-H753ZI bring-up

The MB1364 board contains a LAN8742A PHY and RJ45 connector CN14. Its RMII pins
use AF11:

| Signal | Pin |
|---|---|
| REF_CLK | PA1 |
| MDIO | PA2 |
| MDC | PC1 |
| CRS_DV | PA7 |
| RXD0 / RXD1 | PC4 / PC5 |
| TX_EN | PG11 |
| TXD0 / TXD1 | PG13 / PB13 |

Use the SDK's `external/lwip` checkout beside IOsonata. This work was validated
with lwIP 2.2.0, tag `STABLE-2_2_0_RELEASE`, commit
`0a0452b2c39bdd91e252aef045c115f88f6ca773`. IOcomposer source links and include
paths expect that SDK layout. lwIP remains an external dependency with its
upstream license; it is not copied into IOsonata.

1. Import the H753 library and `STM32H753/exemples/EthernetDemo/ioc` into
   IOcomposer. Build the library first, then the application with the matching
   Debug or Release configuration.
2. Assign a unique unicast MAC in `src/board.h`. The default
   `02:00:00:75:30:01` is a locally administered demonstration address;
   do not run multiple boards with that same address.
3. Connect ST-LINK for power/debug and open its USART3 virtual COM port at
   **1000000 baud, 8N1**. Debug uses semihosting; enable it in the debugger.
   Release runs without a debugger. The application console uses UART in both.
4. Connect CN14 to a LAN with a DHCP server. Expect the banner, then an IPv4
   address and negotiated link rate. Ping that address from a computer on the
   same LAN. `EthernetDemo` responds to ping without opening a client socket.
5. For echo transfers, import `EthernetUdpDemo` or `EthernetTcpDemo`. Set
   `NET_DEMO_HOST` in that project's `src/board.h` to the computer's LAN IPv4
   address. Both use port 2469. Run on the computer:

   ```sh
   python3 Python/net_echo.py --host 0.0.0.0 --port 2469
   ```

   Allow UDP and TCP port 2469 through the host firewall. Expect counter messages
   once a second and matching RX messages in the board console. TCP may split or
   combine messages; it is a byte stream.
6. Unplug/reconnect the cable, restart the echo server and reset the board.
   Verify DHCP recovery and socket reconnect. Check sustained traffic separately
   before drawing throughput conclusions.

Static IPv4 configuration is available by setting `s_IpCfg.bDhcp = false` in
`exemples/net/ethernet_demo.cpp` and supplying address, mask, gateway and DNS for
that LAN. There is no automatic static-address fallback after DHCP failure.

### Binary echo integrity run

Use the existing EthernetUdpDemo and EthernetTcpDemo projects. In their
`src/board.h`, set `NET_DEMO_TEST_COUNT` to `6000` and keep `NET_DEMO_HOST`
set to the computer running the echo server. Build and flash one project at
a time. Start the server before resetting the board:

```sh
python3 Python/net_echo.py --quiet
```

The test sends deterministic binary payloads of 1, 15, 16, 17, 255, 256, 257,
1024 and 1472 bytes. TCP also tests 4096 bytes, handling partial sends and
arbitrary receive splits. Every received byte and UDP datagram length is
checked. One message is outstanding at a time, with a 10 ms foreground
service interval; this tests sustained integrity, not maximum throughput.

Expect `Echo checked 100/6000` progress lines followed by `Echo PASS` with
message/byte counts and elapsed milliseconds. A mismatch, unexpected length,
5-second message timeout, network loss or socket disconnect ends the run with
`Echo FAIL`. A name lookup/connect failure also ends the run. Reset to retry;
the test does not silently reconnect and discard an unfinished exchange.
Keep the cable and server connected for this test. Use `NET_DEMO_TEST_COUNT 0`
to return to the text demo and its reconnect behavior.

### DNS hostname check

For a public Internet check, set `NET_DEMO_HTTP_TEST` to `1` in
EthernetTcpDemo's `src/board.h`, rebuild and reset the board. This selects
`example.com` over TCP port 80 regardless of `NET_DEMO_HOST` and
`NET_DEMO_TEST_COUNT`. No Python echo server or local DNS record is needed.
The board uses the DNS server supplied by DHCP, sends one HTTP GET request,
and prints the status line followed by `DNS/HTTP PASS`. Set the flag back to
`0` to restore the selected echo mode.

This is a bounded connectivity check, not an HTTP client implementation: it
validates an HTTP/1.0 or HTTP/1.1 status line with code 200-599, then closes.
A redirect or error response still demonstrates a response over the network;
the code is printed for inspection. It does not validate the body, follow
redirects or authenticate the server. DNS/connect failure, malformed status,
early disconnect or a 15-second deadline ends the check with `DNS/HTTP FAIL`.
Public service availability and LAN filtering can affect the result.

Create or use a LAN DNS A record for the computer running `net_echo.py`.
Verify with `nslookup <hostname>` that the answer is that computer's LAN IPv4
address, using the same DNS server provided to the board by DHCP. Then replace
the numeric `NET_DEMO_HOST` with that hostname, rebuild and reset the board to
clear the stack's DNS cache. A successful echo run now exercises DNS lookup
before opening the socket. A computer name that works only through mDNS
(`.local`) or a local hosts file is insufficient: this lwIP profile uses
unicast DNS and does not enable mDNS name resolution.

## Validation

```sh
python3 tests/net/run.py
python3 tests/net/run_echo.py
python3 tests/stm32h7/run_ethernet.py
python3 tests/stm32h7/run.py
```

`LWIP_ROOT` can select a different local lwIP checkout. Host tests require GCC;
Arm compilation needs GNU Arm GCC. Register models require Python `unicorn`
and `pyelftools`.

`run_echo.py` requires no MCU or lwIP checkout. It checks the example's binary
test state with simulated partial transfers, corruption, extra/short data,
refused sends and timeout across timer wrap, at O0/O2 with ASan/UBSan. It also
checks the Python server using local UDP and TCP sockets in normal/quiet mode.
The HTTP checks cover partial writes, fragmented status lines, final buffered
data after TCP close, malformed/oversized replies, connect failure and timeout.
These checks validate the test code; they do not establish board results.

The packet tests execute the real lwIP sources with an independent Ethernet
packet peer under AddressSanitizer and UndefinedBehaviorSanitizer at O0/O2.
They cover ARP, ICMP echo, DHCP with address-conflict probes, DNS, UDP boundaries
and truncation, TCP multi-segment traffic, connect timeout, link loss/recovery,
close/reopen, event-queue refusal and a late DNS result after socket reuse.
They also cover deferred callbacks during DNS/connect/send waits, receive
callbacks after TCP send-buffer exhaustion, and DHCP lease expiry/recovery
while the physical link remains up.
They do not emulate the physical Ethernet controller.

The separate H753 register model executes the production driver at O0/Os with
cache maintenance enabled and disabled. It checks descriptors, ring exhaustion
and wraparound, buffer-address restoration, invalid frames, ownership/lifecycle,
interrupt notifications, PHY negotiation and bounded initialization failures.
It does not model real cache coherence or the physical MAC/DMA bus.

The full H753 runner builds the library and nine bare-metal applications in
Debug/Release with and without hard-float. It also checks DMA section placement,
Ethernet/USB vector selection, unused networking removal and the existing
startup/UART/timer/USB register tests. These automated checks are separate from
the hardware results below.

### NUCLEO-H753ZI hardware results, October 21, 2026

The maintainer ran EthernetDemo, EthernetUdpDemo and EthernetTcpDemo with the
LAN8742A RMII PHY on CN14 and USART3/ST-LINK VCP at 1000000 baud, 8N1.
The LAN negotiated 100 Mbit/s and DHCP assigned `10.5.5.54`. Initial link-up
appeared around 2 seconds and the address around 11 seconds after startup.
The exact flashed commit, compiler and Debug/Release profile were not recorded.

| Check | Observed result |
|---|---|
| ICMP echo | 20 sent, 20 received, zero loss; RTT min/avg/max 1.000/1.330/1.631 ms |
| UDP echo | Both directions confirmed with `Python/net_echo.py`; replies 8-64 were consecutive after a startup gap |
| TCP echo | Matching board TX/RX messages and host receipts |
| TCP server restart | New connection on source port 64862; message numbering continued at 52 without resetting the board |
| Cable recovery | Link down at 160 s, link restored at 189 s, DHCP and TCP restored at 196 s; new source port 64863 and matching replies 102-107 without resetting the board |

Temporary diagnostics showed no application event-queue refusals in these logs.
Those diagnostics have been removed. The example retains its explicit static
1024-byte UART TX FIFO so startup and message bursts print complete lines.

The initial UDP text log received replies 0-3 together but did not show replies
4-7; the cause was not established. Later binary integrity runs against
`10.5.5.51:2469` passed on the same board on October 21, 2026:

| Protocol | Messages verified | Bytes each direction | Elapsed time |
|---|---:|---:|---:|
| UDP | 6000 | 2207018 | 120030 ms |
| TCP | 6000 | 4445400 | 120068 ms |

These runs checked every payload byte with no reported corruption or timeout.
UDP lengths ranged from 1 to 1472 bytes; TCP also exercised 4096-byte messages.
They used one outstanding message and a 10 ms service interval, so they are
sustained integrity results, not maximum-throughput measurements or UDP burst
queue tests. The numeric-IP tests did not exercise DNS.

A subsequent EthernetTcpDemo public DNS/HTTP check resolved `example.com`,
connected to TCP port 80, received `HTTP/1.1 200 OK`, and printed
`DNS/HTTP PASS`. This confirms public hostname resolution and routed TCP
traffic on the board; it is plaintext HTTP, not TLS or certificate validation.
The exact flashed commit and build profile were not supplied with this log.
DHCP lease expiry/address changes, maximum throughput, burst load and
D-cache-enabled operation still need hardware tests.

### Native TLS hardware results, October 21, 2026

The maintainer built and ran EthernetTcpDemo's TLS mode on NUCLEO-H753ZI
against OpenSSL at `10.5.5.51:2469`, with the board at `10.5.5.54`.
The demo selects the existing H7 AES/HASH and STM32 RNG providers.
The exact flashed commit, compiler version and build profile were not supplied.

| Check | Observed result |
|---|---|
| TLS 1.3 external PSK | `TLS authenticated: AES-128-GCM/SHA-256, PSK-only` |
| Encrypted reverse echo | Verified counter reached 1000 before server interruption and 1300 after recovery |
| Server restart | Transport error 7, retries while unavailable, new authenticated session and continued echo checks without board reset |
| Wrong PSK | Repeated peer error 8; no authenticated session or successful echo shown |
| Correct PSK restored | Automatic authentication and verified echo checks resumed through 700 without board reset |
| Cable interruption | Network-down notification, DHCP recovery, a new authenticated TLS session and verified echoes resumed through 600 without resetting the board |

The maintainer subsequently clean-built and ran P-256 PSK-DHE mode. The console
reported `TLS authenticated: AES-128-GCM/SHA-256, PSK-DHE P-256`, verified echoes
through 1200 before an interruption, and resumed through 1600 after server
recovery. Cable removal produced a network-down notification; DHCP recovery
was followed by a new authenticated session and verified echoes through 1800
without resetting the board.

A later run had unexpected timeout (6), transport (7), and peer-alert (8)
failures with the correct PSK and no intentional cable changes. Restarting
OpenSSL restored service. After board timing diagnostics and OpenSSL
`-state -msg` logging were added, the maintainer could not reproduce the issue.
The cause remains unresolved; no uninterrupted soak duration was supplied.
These observations do not establish long-run stability. Capture both logs before
restarting the server if the failure recurs. The demo diagnostics distinguish
engine enable, TCP connect and TLS init failures and record pre-failure counters.

These are initial interoperability and recovery results, not certificate
validation, maximum-throughput measurements or a security audit. The running
firmware establishes a successful maintainer build; a fresh automated ARM build
was not completed in the implementation session.

### MQTT hardware results, October 21, 2026

EthernetTcpDemo connected to the broker at 10.5.5.51 over plaintext TCP,
subscribed to iosonata/h753/echo at QoS 1 and verified 200 returned messages.
Stopping the broker produced transport error 2 and TCP retries every three
seconds. Restarting it restored the connection and subscription; verified
counts continued through 400. Cable removal then produced Network down;
DHCP recovery restored MQTT and its subscription, with echoes through 500.
Neither recovery required a board reset. The exact build profile was not supplied.

The subsequent MQTT-over-TLS run used TLS 1.3 external PSK with P-256 PSK-DHE:
board `10.5.5.54` to the PC at `10.5.5.51:8883`, then
`Python/mqtt_tls_bridge.py` to Mosquitto on `127.0.0.1:1883`. The board
authenticated TLS, subscribed at QoS 1 and verified returned MQTT payloads.

| Check | Observed result |
|---|---|
| Broker stop/restart | Transport error 7. TLS could authenticate to the helper while the broker remained unavailable, then transport closed. After the broker returned, MQTT subscribed again and verified echoes reached 200. |
| Cable removal/reconnection | DHCP returned. The first TLS handshake lost transport; the next authenticated successfully, MQTT subscribed again and verified echoes reached 300 without a board reset. The failed handshake's cause was not established. |
| Wrong PSK | Repeated peer-alert error 8; no authenticated MQTT subscription or new successful echoes. |
| Correct PSK restored | TLS authenticated, MQTT subscribed again and verified echoes reached 400 without a board reset. |

A separate apparent recurrence was a confirmed port mismatch: packet capture
showed the board connecting to TCP 1883 and the PC returning RST while TLS was
listening on 8883. Correcting the firmware port restored service. This does not
explain or close the older intermittent TLS issue described above, or the single
failed handshake after cable recovery.

The exact flashed commit and build profile were not supplied. These maintainer
results establish hardware interoperability and the observed recovery cases;
they are not an uninterrupted soak test or a fresh automated ARM compile/link
result. The EthernetTcpDemo README documents the helper and wrong-key setup.

### nRF9160 MQTT hardware results

The maintainer ran `LteMqttDemo` with modem firmware
`mfw_nrf9160_1.2.8`. The console showed successful roaming LTE-M registration,
then three socket-initialization failures with no successful broker connection.
That initial run did not demonstrate MQTT subscription or a verified self-echo.
It provided evidence of a running application and LTE
registration, not successful DNS, TCP or MQTT operation.

The original `error=0 result=0` fields described the shared MQTT state before
initialization. They did not expose the native socket failure or establish a
firmware-version cause. The application now labels those fields `mqtt_error`
and `mqtt_result`, prints `MQTT TCP connecting` before the attempt and labels
failed socket setup `socket initialization`. The nRF91 port separates its
30-second per-address connection timeout from the example's 100-millisecond
send wait, explicitly retries IPv4 after unsuccessful IPv6-only results and
records native failure details before cleanup.

The maintainer's retest after commit `68aae178` succeeded with the same
`mfw_nrf9160_1.2.8` modem firmware and a 115200-baud console. The board registered
on roaming LTE-M and connected to `test.mosquitto.org:1883`. It printed its
generated client ID and topic, subscribed at QoS 1 and reported verified echo
counts of 1 and then 10, both with `failures=0`. Each checked exchange requires
the outgoing PUBACK and the matching returned payload.

This establishes plaintext MQTT-over-LTE hardware operation for that nRF9160
run. The log does not identify the cause of the initial failures or separate
the effects of the connection timeout and IPv4 fallback changes. Recovery after
broker or registration loss, an extended soak and TLS are not established by
this log. It does not establish nRF91x1 hardware operation or fresh automated
ARM compile/link validation.

## References

- [ST MB1364-H753ZI-E01 schematic](https://www.st.com/resource/en/schematic_pack/mb1364-h753zi-e01-schematic.pdf)
- [STM32H7 reference manual RM0433](https://www.st.com/resource/en/reference_manual/rm0433-stm32h742-stm32h743753-and-stm32h750-value-line-advanced-armbased-32bit-mcus-stmicroelectronics.pdf)
- [ST AN4839: Cortex-M7 cache and DMA](https://www.st.com/resource/en/application_note/an4839-level-1-cache-on-stm32f7-series-and-stm32h7-series-stmicroelectronics.pdf)
- [lwIP raw API](https://www.nongnu.org/lwip/2_2_x/raw_api.html)


