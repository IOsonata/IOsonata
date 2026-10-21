# Ethernet and connected sockets

`SockIntrf` presents connected UDP and TCP through `DeviceIntrf`, with C helpers
and a C++ wrapper. nRF91 uses its modem's IP stack. STM32H753 now uses the shared
`src/net/ip_lwip.cpp` service over an injected `EthernetDev_t`. Protocol clients
can use the same socket interface on either target. The nRF91 implementation
and public socket signatures are unchanged.

NUCLEO-H753ZI hardware tests confirm DHCP, ping, UDP/TCP integrity, public
DNS/HTTP and TCP recovery after server restart and cable reconnection. See the
hardware results below
for the tested scope and remaining checks.

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
- Ethernet TLS, DTLS, credential storage and LTE release-assistance operations
  return false. nRF91 modem TLS/DTLS support is unchanged. MQTT and CoAP work
  remains separate.
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
client. nRF91 modem TLS/DTLS is unchanged.

The profile offers only TLS 1.3, `TLS_AES_128_GCM_SHA256`, and `psk_ke`.
The server must explicitly support external PSK without DHE. This profile has
no forward secrecy. It has no X.509, certificate hostname verification, DTLS,
early data, session resumption, or HelloRetryRequest support. Unsupported
negotiation fails closed. It is an initial implementation, not a claim of
complete TLS feature coverage or an independently audited security library.

The caller supplies existing synchronous `HashEngine`, `MacEngine`, `AeadEngine`
and security-grade `RngEngine` instances. No duplicate crypto implementation,
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
state. The measured x86-64 context size is 22232 bytes; this is not an ARM RAM
measurement and excludes provider objects and stack. The receiver supports the
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
context reuse across connections. No hardware TLS result is recorded yet.

Certificate authentication, PSK-DHE, DTLS, broader protocol/error handling,
credential-tag integration and independent security review remain future work.
The shared protocol/crypto separation allows this component to move with the
communications code into Voci when the ecosystem is ready.

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

## References

- [ST MB1364-H753ZI-E01 schematic](https://www.st.com/resource/en/schematic_pack/mb1364-h753zi-e01-schematic.pdf)
- [STM32H7 reference manual RM0433](https://www.st.com/resource/en/reference_manual/rm0433-stm32h742-stm32h743753-and-stm32h750-value-line-advanced-armbased-32bit-mcus-stmicroelectronics.pdf)
- [ST AN4839: Cortex-M7 cache and DMA](https://www.st.com/resource/en/application_note/an4839-level-1-cache-on-stm32f7-series-and-stm32h7-series-stmicroelectronics.pdf)
- [lwIP raw API](https://www.nongnu.org/lwip/2_2_x/raw_api.html)

