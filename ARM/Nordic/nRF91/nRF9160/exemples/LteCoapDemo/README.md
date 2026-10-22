# LteCoapDemo on Nordic Thingy:91

This bare-metal example runs the shared IOsonata CoAP client over the nRF9160
modem's UDP `SockIntrf`. The default board is Nordic Thingy:91; the nRF9160 DK
pin selection is also provided. The application is `exemples/lte/lte_coap.cpp`.
Protocol handling remains in `include/net/coap.h` and `src/net/coap.cpp`.

The default profile sends a read-only CON GET to **coap.me:5683, resource
`/test`**, every 10 seconds. No local PC server is needed. A second profile
retains the full eight-step exercise against `Python/coap_server.py` on a server
you control. Both use plaintext UDP and have no DTLS, OSCORE or certificate
authentication.

## Public server check

The [coap.me service](https://coap.me/about) is a public CoAP client test server.
Leave these defaults in `src/board.h` for the initial LTE check:

```cpp
#define COAP_DEMO_PUBLIC_TEST 1
#define COAP_DEMO_HOST "coap.me"
#define COAP_DEMO_RESOURCE "test"
#define COAP_DEMO_PORT 5683
```

The SIM must provide DNS and outbound UDP reachability to the server. The
application sends GET only, with Uri-Host and one Uri-Path option, and expects
a 2.05 Content response with 0..1024 payload bytes. Public resource content may
change, so this profile checks the response code and bounded payload length
without comparing against the helper's fixed strings. It is a
DNS/UDP/CoAP connectivity check; it does not establish POST/PUT/DELETE, binary
echo, NON-request or separate-response behavior.

`COAP_DEMO_RESOURCE` is one 1..255-byte Uri-Path segment. Empty values, `/`, `?`,
`#`, and the segments `.` or `..` are rejected before LTE starts. It is not
a full URL or a multi-segment path parser. The service is shared and its
availability can change. Keep the default 10-second request interval for this
small check. Use the controlled helper profile for reproducible full validation.

The first checked response prints `CoAP public GET smoke test verified`.
The result line `CoAP public GET checked N code=2.05 bytes=L failures=F` is
printed for the first success and every 10 successes thereafter. An unexpected
response prints its actual code and payload length before reporting failure.

## Controlled server reachable from LTE

For the eight-step helper exercise, set `COAP_DEMO_PUBLIC_TEST` to `0` and
replace `COAP_DEMO_HOST` with your helper's reachable address. This profile
uses the fixed resource paths in the application and ignores
`COAP_DEMO_RESOURCE`. It writes and deletes the helper's volatile `/value`
resource, so run one board against that helper at a time. Use this profile only
with the supplied helper or an endpoint configured for the same test behavior.

The prior Ethernet PC address `10.5.5.51` is a private LAN address. An ordinary
cellular Internet connection cannot route to that PC directly. Set
`COAP_DEMO_HOST` to an address reachable through the SIM's data connection:

- Run `Python/coap_server.py` on a server with a reachable public address and
  permit inbound UDP 5683 in its host and network firewall.
- Or forward UDP 5683 on a router with a reachable public address to the PC's
  UDP 5683, and use that public address or its DNS name in the board setting.
  The helper still binds the PC's actual local interface address, such as
  `10.5.5.51`. A router behind carrier NAT needs an upstream route or forwarding
  arrangement; its local port-forward rule alone does not make it reachable.
- A private APN or VPN arrangement can also work when it explicitly routes the
  modem's connection to the server's private address.

The board initiates the UDP exchange. The board does not need an inbound port
forward through the cellular operator. A generic public CoAP test service does
not provide this demo's resource behavior; run the supplied helper for the full
exercise.

On the chosen server, from the IOsonata repository root:

```sh
python3 -m venv .venv-coap
. .venv-coap/bin/activate
python3 -m pip install aiocoap==0.4.17
python3 Python/coap_server.py --host SERVER_INTERFACE_IP --port 5683 --verbose
```

Replace `SERVER_INTERFACE_IP` with an address assigned to that server. The
helper requires an explicit interface address and rejects wildcard binds. Its
default `127.0.0.1` serves local host tests only. If the server is behind NAT,
bind its private interface address rather than the router's public address.
For the forwarded PC setup, the last command is:

```sh
python3 Python/coap_server.py --host 10.5.5.51 --port 5683 --verbose
```

Python 3.10 or newer is required. The helper uses aiocoap's UDP implementation
and serves `/hello`, `/echo`, `/value` and `/slow`. The default `/slow` delay of
0.3 seconds produces an empty ACK followed by a separate CON response.

## Board configuration and console

Edit `src/board.h` beside this README. Supply the hostname or address only,
without `coap://` or a resource path. If `COAP_DEMO_HOST` is explicitly set to an
empty string, the application prints a configuration message and stays idle
without starting LTE.

| Setting | Default | Meaning |
| --- | --- | --- |
| `COAP_DEMO_PUBLIC_TEST` | `1` | Read-only public GET; `0` selects the full helper exercise |
| `COAP_DEMO_HOST` | `"coap.me"` | Reachable address of the selected test server |
| `COAP_DEMO_RESOURCE` | `"test"` | Single Uri-Path segment for the public GET profile |
| `COAP_DEMO_PORT` | `5683` | Server's UDP port |
| `COAP_DEMO_INTERVAL` | `10000U` | Minimum interval between new requests, in milliseconds |
| `COAP_DEMO_PROBING_RATE` | `0U` | Use the library default of 1 byte/second while a peer is not responding |
| `LTE_COAP_APN` | `nullptr` | Use the network's default APN; set a string when the SIM requires one |
| `LTE_COAP_RAT` | `LTE_RAT_LTEM` | LTE-M radio access |
| `LTE_COAP_RAT_PREF` | `LTE_RAT_LTEM` | LTE-M preference |

The example disables requested PSM, eDRX and release assistance while testing
request/response exchanges. This keeps the initial validation separate from
low-power scheduling, including the delayed separate response. It does not
change these settings in other examples.

The Thingy:91 console uses nRF9160 UART0, RX P0.19 and TX P0.18, at **115200
baud, 8N1, no flow control**. Its companion nRF52840 must forward this UART to
USB, using the existing Connectivity Bridge or HciController CDC bridge.
Select the USB serial port carrying that console. The application does not
initialize BLE or an HCI host and needs no change to the companion firmware
when its console bridge already works.

To use the nRF9160 DK, select `NRF9160_DK` instead of `NORDIC_THINGY91` in the
board header. UART0 then uses RX P0.28 and TX P0.29 through the interface MCU's
VCOM0. RTC0 (`LTE_TIMER_DEVNO = 0`) is shared by the modem wait trigger 0 and
the application's pump trigger 1.

## Build and run

Keep the existing IOsonata SDK layout and dependencies used by `LteUdp`.
Import these IOcomposer projects:

1. `ARM/Nordic/nRF91/nRF9160/lib/ioc`, named `IOsonata_nRF9160`.
2. `ARM/Nordic/nRF91/nRF9160/exemples/LteCoapDemo/ioc`, named `LteCoapDemo`.

Build the library first, then the application using matching `Debug` or
`Release` configurations. These are the bare-metal configurations, not the
library's TaktOS variants. The project preserves the existing `NRF9160_XXAA`,
Cortex-M33 Thumb and hard-float settings, GNU C++23, no exceptions and no RTTI.
It links `IOsonata_nRF9160` and the Nordic `modem` library from
`external/sdk-nrfxlib/nrf_modem/lib/cellular/nrf9160/hard-float`; its header
paths also use `external/nrfx` and `external/sdk-nrfxlib/nrf_modem/include`.
There is no lwIP dependency for the modem socket transport.

Use the existing secure-image configuration. The Nordic CryptoCell RNG provider
requires secure access; do not add `NRF_TRUSTZONE_NONSECURE` to either build.

The nRF9160 library includes the shared `src/net/coap.cpp`. The board selects
`CryptoRngNrfInstance()` from `crypto_rng_nrf.h`, reusing the existing
CryptoCell RNG through `Cc3xxIntrf`. No new random generator is introduced.
The nRF91x1 library also includes shared CoAP, but this dedicated board and
application project targets the nRF9160.

Build and program the nRF9160 application using your normal Thingy:91 setup.
Open the console before resetting the board. Confirm the server address, LTE
registration and the request result lines. Registration must succeed before the
UDP socket is opened. The public profile repeatedly checks GET `/test`.

### Full helper exercise

With `COAP_DEMO_PUBLIC_TEST` set to `0` and the controlled helper running, the
demo repeats this sequence:

| Step | Request | Checked result |
| --- | --- | --- |
| 1 | CON GET `/hello` | 2.05 and `IOsonata CoAP` plus newline |
| 2 | CON POST `/echo` | 2.05 and identical binary bytes |
| 3 | NON POST `/echo` | 2.05 and identical binary bytes |
| 4 | CON PUT `/value` | 2.04 with an empty payload |
| 5 | CON GET `/value` | 2.05 and the stored binary bytes |
| 6 | CON DELETE `/value` | 2.02 with an empty payload |
| 7 | CON GET `/value` | Expected 4.04 with an empty payload |
| 8 | CON GET `/slow` | Separate CON 2.05 and `separate response` plus newline |

Binary payload lengths rotate through 1, 16, 257 and 900 bytes across cycles.
The first cycle logs every checked response; later output reports every 10
successful requests. With the 10-second interval, allow about 80 seconds for
the first complete cycle, plus network registration and any retransmissions.
A count of successful sends alone is not a pass: the application verifies the
response code and payload before advancing the sequence.

The first complete cycle prints:

```text
CoAP methods, CON/NON and separate response verified
```

The preceding `CoAP checked` lines identify the step, response code, byte count
and cumulative failures. `CoAP FAIL` identifies the failed exchange; the next
`CoAP retry wait` line reports its cooldown in milliseconds.

The 20-millisecond timer defers socket and CoAP work to the application event
queue. Interrupt handlers do not run CoAP, make AT calls or log responses.
The socket TX wait is at most 100 milliseconds; modem DNS/socket initialization
has separate blocking waits. The CoAP exchange timeout is 95 seconds.
Confirmable requests use the shared client's randomized
retransmission timing. No CoAP message is split across UDP datagrams.

## Recovery checks

In the helper profile, after verifying a complete cycle, stop and restart your
server. Record the
exchange error, the retry wait and the return to verified responses without a
board reset. The application keeps the existing context after an exchange
failure when the socket is still open. Transport or registration loss closes
the UDP endpoint and starts a fresh context after registration returns. A
modem fault schedules reinitialization with a 30-second retry interval.

Retry waits are at least three seconds and also honor the remaining CoAP
probing-rate delay. The default 1-byte/second rate limits repeated probes to an
unresponsive peer; large unanswered messages and retransmissions can therefore
produce a long wait. The delay is retained across endpoint replacement. Normal
matching responses clear it. Avoid resetting the board merely to bypass this
wait while checking server recovery.

Also record registration-loss/re-registration behavior when the test setup can
produce it, including whether pending exchanges fail cleanly and the sequence
restarts. Keep console and server logs together, and include the modem firmware,
SIM/APN, server address, elapsed recovery time and first recovered response.
Do not infer end-to-end delivery from LTE registration or successful UDP sends.

## Validation status

The shared CoAP implementation has passed the host wire and aiocoap
interoperability tests described in `docs/networking.md`.

`python3 tests/net/run_lte_coap.py` passed the application host syntax check
using the actual Thingy:91 board header and Nordic RNG declaration, with an
empty `nrf.h` host stub. Focused application checks passed at O0/O2 with
ASan/UBSan: registration-loss close/reopen, retained pacing across repeated
modem faults and clock wrap, failed-initialization retry after 30 seconds,
timeout recovery retaining a live endpoint, timer enqueue refusal/coalescing
and empty-host startup without LTE. Both the public GET and full helper profiles
pass. Public-profile fixtures check the configured path, alternate response
content, response-code/size errors, recovery and 10-second request spacing.
These checks use host substitutes for the modem, timer, UART and RNG hardware.

The public endpoint is a published service setting. A live public probe could
not resolve the host in the implementation environment, so current public
reachability was not verified there.

The existing BleGateway example has separately recorded Thingy:91 LTE-M UDP
hardware results with modem firmware 1.3.6 and the HciController console bridge.
Those results do not validate this CoAP application.

No nRF9160 CoAP hardware result or fresh ARM compile/link result is claimed by
this example. Record the maintainer's build and hardware output after running
these steps.
