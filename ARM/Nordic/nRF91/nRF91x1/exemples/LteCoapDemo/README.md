# LteCoapDemo on the nRF9161 DK and nRF9151 DK

This bare-metal project runs the shared `exemples/lte/lte_coap.cpp` application
through the nRF91 modem's UDP `SockIntrf`. The shared CoAP implementation is
`src/net/coap.cpp`. The target project and board header select the existing
nRF91x1 library and DK resources.

The default profile sends a read-only confirmable GET to
**coap.me:5683, resource `/test`**, at least 10 seconds apart. It expects a
2.05 Content response with a payload of at most 1024 bytes. The SIM needs
Internet data, DNS and outbound UDP access. This test uses plaintext UDP and
does not enable DTLS or OSCORE.

## Build and program

Import these IOcomposer projects:

1. `ARM/Nordic/nRF91/nRF91x1/lib/ioc`, named `IOsonata_nRF91x1`.
2. `ARM/Nordic/nRF91/nRF91x1/exemples/LteCoapDemo/ioc`, named `LteCoapDemo`.

Clean and build the library first, then the application, using matching
**Debug** configurations for bring-up. Matching Release configurations are
also supplied. Use the bare-metal library configuration, not its TaktOS
variant. Use separate workspaces if keeping both the nRF9160 and nRF91x1
`LteCoapDemo` projects imported, since their project names are the same.

The target settings follow the existing nRF91x1 `LteUdp` project:

| Setting | Value |
| --- | --- |
| C and C++ target definition | `NRF9120_XXAA` |
| CPU and ABI | Cortex-M33, Thumb, hard-float |
| IOsonata library | `IOsonata_nRF91x1` |
| Linker script | `ARM/Nordic/nRF91/ldscript/nrf9120_xxaa.ld` |
| Nordic modem library | `external/sdk-nrfxlib/nrf_modem/lib/cellular/nrf9120/hard-float` |
| C++ | GNU C++23, no exceptions, no RTTI |

Keep the existing `external/nrfx` and `external/sdk-nrfxlib` layout. The
`nrf9120` names are the SDK/compiler selections used by this IOsonata port.
The library supplies startup, interrupt vectors, modem integration and the
existing `CryptoRngNrfInstance()` provider. Keep the secure-image configuration
for its CryptoCell RNG; do not define `NRF_TRUSTZONE_NONSECURE` in either
project. Program the application using the normal nRF9161 or nRF9151 DK setup.

## Board and peer configuration

Both DKs use UART0 through the interface MCU's VCOM0: **RX P0.26, TX P0.27,
115200 baud, 8N1, no flow control**. The pins use operation 1. RTC0 supplies
modem wait trigger 0 and application pump trigger 1.

Defaults in `src/board.h` are:

```cpp
#define COAP_DEMO_PUBLIC_TEST 1
#define COAP_DEMO_HOST "coap.me"
#define COAP_DEMO_RESOURCE "test"
#define COAP_DEMO_PORT 5683
#define COAP_DEMO_INTERVAL 10000U
#define LTE_COAP_APN nullptr
```

Supply the SIM provider's APN string when required. LTE-M is selected, and the
shared application disables requested PSM, eDRX and RAI for this initial test.
The resource is a single Uri-Path segment, without a leading slash. Public
service content can change; this profile checks the response code and size.

Open the console before reset. Following registration and UDP initialization,
the first checked response has this form, with the actual payload length:

```text
CoAP public GET checked 1 code=2.05 bytes=<length> failures=0
CoAP public GET smoke test verified
```

To exercise the full method, binary echo, CON/NON and separate-response
sequence, set `COAP_DEMO_PUBLIC_TEST` to `0` and point `COAP_DEMO_HOST` at the
supplied `Python/coap_server.py` helper on a server reachable from the SIM.
The [shared demo instructions](../../../nRF9160/exemples/LteCoapDemo/README.md)
give the helper setup, exact eight-step sequence and recovery checks. Use this
page's nRF91x1 project paths and DK pins when building.

## Validation status

The application host syntax and O0/O2 ASan/UBSan checks passed with this board
configuration for both public and helper profiles. Run them and the shared
socket checks from the repository root:

```sh
python3 tests/net/run_lte_coap.py --target nrf91x1
python3 tests/net/run_nrf91_socket.py
```

Host checks exercise the real application and CoAP implementation with Nordic
hardware substitutes, including both public and helper profiles. The socket
suite checks the shared nRF91 port independently. These do not replace an ARM
compile/link or cellular hardware run. No hardware result for this nRF91x1
`LteCoapDemo` application has yet been recorded.
