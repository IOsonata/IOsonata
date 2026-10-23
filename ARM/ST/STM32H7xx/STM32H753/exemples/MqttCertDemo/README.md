# MqttCertDemo: native TLS certificates on NUCLEO-H753ZI

This new project runs the shared MQTT QoS 1 application over the shared native
TLS 1.3 client and an Ethernet TCP socket. The same MQTT application is used by
`MqttDemo` and the nRF91 modem `MqttTlsDemo`. The selected interface is injected
after its TLS handshake completes. No MQTT packet or verified-peer message is
sent before server authentication succeeds.

The first certificate profile uses P-256 ECDHE, ECDSA/SHA-256 certificates and
`TLS_AES_128_GCM_SHA256`. It checks the configured CA chain, certificate validity
dates and DNS subjectAltName. The generated local broker credentials match this
profile. RSA chains, client certificates, TLS 1.2 and DTLS are not implemented
by this native profile. It does not replace the nRF91 modem's TLS path.

## Prepare the local broker and firmware trust anchor

Use a PC with OpenSSL and Mosquitto. Run the helper from the repository root:

```sh
python3 Python/mqtt_cert_demo.py prepare \
  --output ../mqtt-cert-demo \
  --hostname iosonata.test
```

The helper generates a new P-256 CA, a server key/certificate and
`mosquitto.conf` in the selected directory. The server certificate is valid
for 30 days by default; `--days` changes that interval. The commissioning PC's
clock determines the certificate dates, so check that clock before generating
them. Private keys stay in that directory with owner-only permissions. Do not
commit or distribute those keys.

Only the public CA DER bytes are written to
`src/net_demo_tls_trust_generated.h`, which this project ignores in Git. The
shared `exemples/net/net_demo_tls_trust.cpp` includes that header when present.
It compiles without the header, but startup reports missing TLS configuration
and does not start a connection. There is no default CA, date or plaintext
fallback. Generating a different CA requires rebuilding the firmware. To
replace an existing generated header, select a new output directory and use
`--replace-header` explicitly.
Clean/rebuild the application after generating the header, especially if it
was previously built without one; that earlier dependency file cannot name a
header that did not yet exist.

Set these values in `src/board.h`:

```cpp
#define NET_DEMO_NATIVE_TLS      1
#define NET_DEMO_TLS_SERVER_NAME "iosonata.test"
#define MQTT_DEMO_HOST           "10.5.5.51"
#define MQTT_DEMO_PORT           8883
#define MQTT_DEMO_INTERVAL       10000U
```

Use the PC's reachable LAN address for `MQTT_DEMO_HOST`. That address selects
the TCP destination. `NET_DEMO_TLS_SERVER_NAME` selects SNI and the DNS name
that must match the certificate SAN; it must match `--hostname`. This separation
allows a controlled LAN test without changing local DNS. The native verifier
does not support IP-literal certificate identities or common-name fallback.

Start the broker as the same ordinary user that generated the credentials:

```sh
mosquitto -c ../mqtt-cert-demo/mosquitto.conf -v
```

The generated listener accepts anonymous MQTT clients on TCP port 8883 with
TLS 1.3 and the configured server certificate. It does not request client
certificates. Use a test LAN and synthetic data; configure broker access
control separately for a deployed application. Allow the board to reach the
listener through the PC firewall. An OpenSSL PSK forwarding helper is not used.
The options are described in the
[Mosquitto configuration manual](https://mosquitto.org/man/mosquitto-conf-5.html).

## Build and connect the board

Import these IOcomposer projects:

1. `ARM/ST/STM32H7xx/STM32H753/lib/ioc`, named `IOsonata_STM32H753`.
2. `ARM/ST/STM32H7xx/STM32H753/exemples/MqttCertDemo/ioc`, named `MqttCertDemo`.

Build the library first, then this application with matching Debug or Release
configurations. Refresh an older library import so that `src/net/tls.cpp` and
`src/crypto/x509.cpp` are included. The existing `external/lwip` and `micro-ecc`
dependencies are used. No TLS package or new crypto library is required.

Connect CN14 to a DHCP-enabled Ethernet LAN and use a unique unicast
`ETHERNET_MAC_ADDR` per board. LAN8742A RMII uses PHY address 0 and TXD1 on PB13.
The console is USART3 through ST-LINK VCP, 1000000 baud, 8N1, no flow control.
Debug requires semihosting; Release can run without the debugger. The existing
linker script keeps `.eth_dma` in AXI SRAM at `0x24000000`, outside DTCM.

The setup uses the existing STM32 RNG provider, shared software AES-GCM,
SHA-256/HMAC and P-256 engines, static TLS state, an 8192-byte handshake buffer
and a 12288-byte transcript buffer. These two buffers are additional to the
internal buffers in `TlsDev_t`. The target linker map determines total RAM use.

## Provision UTC at each boot

Certificate verification requires current UTC. This example deliberately
starts without a trusted time. The physical console and commissioning PC
provide it; the selected network is not used as an unauthenticated time source.

Install pyserial if needed, identify the ST-LINK port, and run:

```sh
python3 -m serial.tools.list_ports
python3 Python/mqtt_cert_demo.py console --port /dev/cu.usbmodemYOUR_STLINK
```

On Linux use the corresponding `/dev/ttyACM...` device; on Windows use its COM
name. Close any other program holding the serial port. The helper sends the
PC's current UTC in seconds since 1970-01-01 and displays the board log. It sends
fresh time again when it observes a new startup prompt after a board reset.
The helper does not change the board's network or broker settings.
When attached to a board that already has an active connection, it stops
time-command retries after the board's rejection and continues displaying logs.
A later startup prompt enables time provisioning again.

The console command is `time <Unix seconds>` followed by CR or LF. A valid
command is acknowledged with `TLS UTC set from console`. Invalid, overflowing
or overlong commands are rejected. An active endpoint prevents time changes;
close the session or reset before changing its validation time. The foreground
pump reads at most 32 console bytes per call and advances UTC from the monotonic
millisecond timer, including timer wrap. UTC is lost on reset and must be
provided again. A deployed application should supply a trusted RTC or another
authenticated provisioning method with suitable accuracy and persistence.

## Expected run and recovery checks

The exact ordering of network and clock messages depends on when the console
connects. A successful session should include:

```text
Generic MQTT QoS 1 echo through Ethernet
Native TLS waiting for UTC: time <Unix seconds>
Ethernet started, waiting for IPv4
TLS UTC set from console
Ethernet IPv4 ...
MQTT connecting: 10.5.5.51:8883
MQTT peer 10.5.5.51:8883, TLS 1.3, CA/hostname/time verified, client iosnet...
MQTT subscribed: iosonata/demo/.../echo QoS=1
MQTT QoS 1 echo checked 1, failures=0
```

These are expected messages, not a recorded board result. The peer message
appears after TLS authentication; a completed QoS 1 check additionally requires
the broker's PUBACK and the matching subscribed echo. The application retains
its boot identity and check counter across connection recovery.

For board verification, record repeated echoes, stop/restart the broker, and
remove/reconnect the Ethernet cable. Reconnection must create a new authenticated
TLS session and MQTT subscription. During a controlled negative test, use a
different server CA, a wrong configured DNS name, or a time outside the
certificate validity interval. The connection must fail before MQTT subscribes.
Restore the correct configuration/time afterward. `Native TLS ended` reports
both `error` and `certificate_error` for diagnosis.

## Host validation and supported scope

From the repository root:

```sh
python3 tests/net/run_native_tls_demo.py
python3 tests/net/run_tls_cert.py
python3 tests/net/run_generic_demo.py
python3 tests/net/run_mqtt_tls_demo.py
```

The new example runner checks project links, real board-header syntax,
credential generation and OpenSSL hostname verification. At O0/O2 with
ASan/UBSan it checks native setup over substituted LTE, Ethernet and custom
transports, missing-trust refusal, UTC parsing and wrap, pending/failed TLS
startup, connection loss, and MQTT initialization after transport readiness.
TLS state is scripted in that setup test; `run_tls_cert.py` exercises the real
TLS/X.509 implementation and crypto against independent local peers.

No fresh ARM compile/link or MqttCertDemo hardware result is recorded for this
change. The earlier H753 PSK tests and nRF91 modem TLS results apply to those
profiles. Full protocol restrictions are in
[networking.md](../../../../../../docs/networking.md); shared application and
medium selection are in the
[generic example guide](../../../../../../docs/network-protocol-examples.md).
