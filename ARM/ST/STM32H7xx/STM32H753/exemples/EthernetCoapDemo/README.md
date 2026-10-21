# EthernetCoapDemo: NUCLEO-H753ZI

This application exercises the shared `Coap : Device` client over a connected
UDP `SockIntrf`. Its application source is `exemples/net/coap_demo.cpp` and the
protocol implementation is `src/net/coap.cpp`. Ethernet, the IP service and the
socket remain the existing H753/lwIP implementations. There is no CoAP controller
code in the MCU port or board header.

## Build and connect

Import this `ioc` project and `../../lib/ioc` into IOcomposer. Build the H753
library first, then the application in the matching Debug or Release
configuration. The library project includes the shared CoAP source. The SDK
needs its existing lwIP checkout at `external/lwip` beside IOsonata.

Connect the RJ45 at CN14 to a DHCP LAN. The console uses USART3 through ST-LINK
VCP at **1000000 baud, 8N1**, with a static 1024-byte TX FIFO. Debug requires
semihosting enabled; Release can run without the debugger. Set a unique unicast
MAC and the PC's actual LAN address in this project's `src/board.h`:

```c
#define COAP_DEMO_HOST "10.5.5.51"
#define COAP_DEMO_PORT 5683
#define COAP_DEMO_PROBING_RATE 1024U
```

`10.5.5.51` is the PC in the recorded Ethernet setup; `10.5.5.54` was the board's
DHCP address. The demo uses the existing STM32 RNG for tokens, initial message
IDs and retransmission timing. Pin mappings remain in `board.h`, including
RMII TXD1 on PB13. No TLS/DTLS options are needed: this test uses plaintext UDP.
The 1024-byte/second probing rate is an explicit controlled-LAN choice under
RFC 7252 section 4.8.1. It governs new requests after unanswered traffic, not
normal requests to a responsive server. Set it to zero to use the library's
default 1 byte/second; that can require substantially longer recovery waits.

## Independent PC peer

Use Python 3.10 or later. From the repository root, create a Python environment
and install the independent aiocoap implementation:

```sh
python3 -m venv .venv-coap
. .venv-coap/bin/activate
python3 -m pip install aiocoap==0.4.17
python3 Python/coap_server.py --host 10.5.5.51
```

Supply an address assigned to the PC and allow UDP port 5683 through its
firewall. The helper binds that one address; `0.0.0.0` and `::` are not accepted.
Its default `127.0.0.1` is for host tests, so pass the LAN address for the board.
`--port` changes the listening port and must match `COAP_DEMO_PORT`.

The helper delegates CoAP encoding, acknowledgements and retransmissions to
aiocoap. It defines these test resources:

| Resource | Operation | Response |
|---|---|---|
| `/hello` | GET | 2.05, `IOsonata CoAP` followed by newline |
| `/echo` | POST | 2.05, exact binary request payload |
| `/value` | PUT | 2.04, stores the binary request payload |
| `/value` | GET | 2.05 with stored bytes, or 4.04 when absent |
| `/value` | DELETE | 2.02 and removes the stored bytes, or 4.04 when absent |
| `/slow` | GET | Empty ACK followed by a separate 2.05 CON response for a CON request |

`/slow` waits 0.3 seconds by default; `--delay` accepts 0.2..30 seconds. Its body
is `separate response` followed by newline. `/echo` and `/value` accept at most
1024 payload bytes. `/value` is shared and volatile, so run one board against
this helper and do not modify the resource during the check. This is an
interoperability helper, without authentication or persistent storage.

## Board checks

After DHCP the application performs one request at a time, with new requests
spaced at least one second apart. Each cycle checks:

1. Confirmable GET `/hello`.
2. Confirmable POST `/echo` with a deterministic binary payload.
3. Non-confirmable POST `/echo` with the same bytes.
4. Confirmable PUT `/value`.
5. Confirmable GET `/value`, verifying the stored bytes.
6. Confirmable DELETE `/value`.
7. Confirmable GET `/value`, expecting 4.04 and an empty body.
8. Confirmable GET `/slow`, verifying the separate response and its payload.

Binary lengths rotate through 1, 16, 257 and 900 bytes across cycles. Every
response code and payload byte is checked. A 4.xx response is delivered by CoAP
as a response; the expected 4.04 is an application check, not a transport error.

Expected console output includes:

```text
CoAP over UDP: LAN8742A RMII, DHCP, console 1 Mbaud
Waiting for Ethernet link and DHCP
IPv4 10.5.5.54, link 100 Mbit/s
CoAP UDP peer 10.5.5.51:5683, plaintext, one request/second
CoAP checked 1 failures=0
CoAP methods, CON/NON and separate response verified
CoAP checked 100 failures=0
```

Counters print on the first verified response and every 100. Stop/restart the
helper and remove/reconnect the cable to test recovery. A failed exchange logs
`CoAP FAIL` and increments the failure count. Protocol failures release the
exchange while retaining the live UDP endpoint and its message-ID, duplicate
and probing-rate state. Transport/network loss closes UDP and starts a fresh
endpoint. Both paths wait at least three seconds and honor any remaining
probing-rate wait; that wait is carried across socket replacement. Verification
resumes from
`/hello`; the checked count continues without resetting the board. Restarting
the helper forgets `/value`, so it can first produce an application mismatch if
the board was between PUT and GET when the server restarted.

A silent peer may require the full confirmable retransmission wait, up to
about 93 seconds. The application request deadline is 95 seconds. A network or
ICMP error can be reported earlier. With the default library probing rate,
large unanswered requests can require additional delay before a new request;
the demo's explicit LAN rate shortens that delay. The expected failures during
deliberate outages remain in the counter; a subsequent recovery does not erase them.

## Scope and validation

The shared client supports unicast GET/POST/PUT/DELETE, CON/NON requests,
piggyback and separate responses, retransmission, Reset, bounded option parsing
and duplicate CON acknowledgements. It uses static storage and the existing
injected `DeviceIntrf`. Maximum complete CoAP datagram size is 1152 bytes with
at most 16 options; the payload limit is smaller by the header, token and
option lengths. There is no DTLS, OSCORE, Observe, blockwise transfer, proxy,
multicast, CoAP over TCP or embedded server engine in this implementation.

Run the host tests from the repository root with the same Python environment:

```sh
python3 tests/net/run_coap.py --require-interop
```

Host wire/error checks passed at O0/O2 with ASan/UBSan. Independent aiocoap
checks passed directly and through a UDP packet-loss proxy, including a retried
request and acknowledgement of a duplicate separate response. The C header,
C++ lifecycle and this application's host syntax with the H753 board
configuration also passed. The runner does not install dependencies; omitting
`--require-interop` permits an explicit skip if aiocoap is unavailable.

See [networking documentation](../../../../../../docs/networking.md) for shared
behavior and recorded validation. CoAP hardware results have not yet been
recorded. Existing UDP/TCP and MQTT/TLS hardware results do not establish CoAP
hardware validation, and no fresh ARM compile/link result is claimed here.

Protocol and independent peer references:

- [RFC 7252](https://www.rfc-editor.org/rfc/rfc7252.html)
- [aiocoap documentation](https://aiocoap.readthedocs.io/en/0.4.17/)
