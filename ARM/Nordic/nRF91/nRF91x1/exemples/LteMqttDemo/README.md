# LteMqttDemo on the nRF9161 DK and nRF9151 DK

This bare-metal project runs `exemples/lte/lte_mqtt.cpp` on the nRF91x1 DKs.
It uses the same shared `Mqtt : Device` implementation and nRF91 modem TCP
`SockIntrf` as the nRF9160 example. The target project and board configuration
select the MCU library and console pins; they contain no MQTT implementation.

The default test connects to **test.mosquitto.org, TCP port 1883**, subscribes
to a random per-boot topic at QoS 1, and verifies its own publications. No
broker account or PC server is needed. The SIM must provide Internet data,
DNS and outbound TCP access. This example uses plaintext TCP, with no TLS.

## Build and program

Import these IOcomposer projects from this target directory:

1. `ARM/Nordic/nRF91/nRF91x1/lib/ioc`, named `IOsonata_nRF91x1`.
2. `ARM/Nordic/nRF91/nRF91x1/exemples/LteMqttDemo/ioc`, named `LteMqttDemo`.

Clean and build the library first, then the application, using matching
**Debug** configurations for bring-up. Matching Release configurations are
also provided. Use the bare-metal library configuration, not its TaktOS
variant. The nRF9160 and nRF91x1 applications have the same project name, so
use a separate workspace when keeping both target projects imported.

The project follows the existing nRF91x1 `LteUdp` settings:

| Setting | Value |
| --- | --- |
| C and C++ target definition | `NRF9120_XXAA` |
| CPU and ABI | Cortex-M33, Thumb, hard-float |
| IOsonata library | `IOsonata_nRF91x1` |
| Linker script | `ARM/Nordic/nRF91/ldscript/nrf9120_xxaa.ld` |
| Nordic modem library | `external/sdk-nrfxlib/nrf_modem/lib/cellular/nrf9120/hard-float` |
| C++ | GNU C++23, no exceptions, no RTTI |

The `nrf9120` target names are the existing SDK/compiler selections for this
IOsonata port. Keep the `external/nrfx` and `external/sdk-nrfxlib` layout used
by the other nRF91x1 examples. Startup, interrupt vectors, modem shared-memory
placement and the secure-image setup come from the existing library and linker
script. The board selects `CryptoRngNrfInstance()` for the random client/topic
identity. Keep secure access for its CryptoCell RNG; do not define
`NRF_TRUSTZONE_NONSECURE` in either project.

Program the application on the nRF9161 or nRF9151 target using the normal DK
setup, then reset with the console open.

## Board configuration

Both DKs use the existing nRF91x1 console mapping:

| Resource | Configuration |
| --- | --- |
| Console | UART0 through the interface MCU's VCOM0 |
| RX | P0.26, pin operation 1 |
| TX | P0.27, pin operation 1 |
| Serial settings | **1000000 baud, 8N1, no flow control** |
| Timer | RTC0, modem wait trigger 0 and application pump trigger 1 |

Edit `src/board.h` for `MQTT_DEMO_HOST`, `MQTT_DEMO_PORT`, and
`LTE_MQTT_APN`. The default APN is `nullptr`; supply the SIM provider's APN
string if required. LTE-M is selected, with a minimum 10-second publication
interval. The shared application disables requested PSM, eDRX and RAI for
this initial continuous-traffic test.

The nRF91 socket port allows 30 seconds per TCP connection attempt, then
applies the application's 100-millisecond send timeout. It tries IPv4 after
unsuccessful IPv6-only results and logs native setup failures before cleanup.
Rebuild `IOsonata_nRF91x1` to include those socket corrections.

## Expected result

After registration, the console should show the following sequence. The
16-hex-digit suffix changes at boot:

```text
MQTT TCP connecting: test.mosquitto.org:1883
MQTT TCP peer test.mosquitto.org:1883, plaintext, client ioslte<16-hex-suffix>
MQTT subscribed: iosonata/lte/<16-hex-suffix>/echo QoS=1
MQTT QoS 1 echo checked 1, failures=0
MQTT QoS 1 echo checked 10, failures=0
```

Each checked exchange requires the outgoing PUBACK, matching topic/payload
self-echo and outgoing acknowledgement of that echo to complete. The example
reconnects and subscribes again after a failed session. Capture any
`nRF91 socket ...` error lines together with the modem registration log.

The [shared demo instructions](../../../nRF9160/exemples/LteMqttDemo/README.md)
describe payload checks, PC observation, recovery tests and current MQTT/TLS
limits. Use the build paths and DK pin map on this page for nRF91x1.

## Validation status

The application host syntax and O0/O2 ASan/UBSan checks passed with this board
configuration. Run them and the shared socket checks from the repository root:

```sh
python3 tests/net/run_lte_mqtt.py --target nrf91x1
python3 tests/net/run_nrf91_socket.py
```

The application checks use host substitutes for the modem, UART, timer and
RNG. The socket checks use the actual shared nRF91 socket implementation with
host Nordic API substitutes. They do not replace an ARM compile/link or a
cellular hardware run. The recorded nRF9160 result reached 10 verified QoS 1
echoes with zero failures; an nRF91x1 build and hardware result are still
pending.
