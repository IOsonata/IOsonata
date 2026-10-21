# EthernetTcpDemo: NUCLEO-H753ZI

Import `ioc` together with `../../lib/ioc` in IOcomposer. Build the H753
library and application in matching configurations. The SDK needs lwIP in
`external/lwip` beside IOsonata; validated with lwIP 2.2.0.

The console uses USART3 through ST-LINK VCP at 1000000 baud, 8N1.
Connect the onboard RJ45 CN14 to a DHCP LAN. Set a unique demonstration MAC
address in `src/board.h`. Debug requires semihosting enabled; Release can run
without the debugger.

See [Ethernet bring-up and limits](../../../../../../docs/networking.md)
for DHCP/ping and UDP/TCP echo instructions. Socket examples need
`NET_DEMO_HOST` set to the computer's LAN address and `Python/net_echo.py`
running on port 2469. EthernetDemo only runs DHCP and responds to ping.

NUCLEO-H753ZI hardware validation on October 21, 2026 confirmed TCP echo,
reconnection after restarting the host server, and DHCP/TCP recovery after
unplugging and reconnecting the cable without resetting the board.

A binary integrity run passed 6000 messages, 4445400 bytes each way in
120068 ms, including 4096-byte messages, with no reported corruption or timeout.
This is a paced integrity test, not a maximum-throughput measurement.

Set `NET_DEMO_HTTP_TEST` to `1` in `src/board.h` for a public DNS/HTTP check.
It selects example.com:80 without a local echo server. Hardware returned
`HTTP/1.1 200 OK` and `DNS/HTTP PASS` on October 21, 2026. This validates
plaintext HTTP connectivity, not TLS. Set the flag back to `0` for echo tests.


## Native TLS 1.3 PSK test

Set `NET_DEMO_TLS_TEST` to 1, `NET_DEMO_HTTP_TEST` and `NET_DEMO_TEST_COUNT`
to 0, and `NET_DEMO_HOST` to the PC's LAN address (for the recorded setup,
`10.5.5.51`, not the board's `10.5.5.54`). Rebuild the H753 library and example.
The library project includes the shared TLS source; it is linked only when used.
`NET_DEMO_TLS_DHE` defaults to 1: PSK authentication with ephemeral P-256
key agreement. Set it to 0 only to repeat the earlier PSK-only test.

On the PC run OpenSSL 3 (the macOS system LibreSSL command may not support this):

```sh
openssl s_server -accept 2469 -nocert -tls1_3 -groups P-256 \
  -ciphersuites TLS_AES_128_GCM_SHA256 -psk_identity IOsonata \
  -psk 000102030405060708090a0b0c0d0e0f101112131415161718191a1b1c1d1e1f -rev
```

This uses a PUBLIC TEST KEY. P-256 PSK-DHE adds forward secrecy to PSK
authentication. To test PSK-only with `NET_DEMO_TLS_DHE=0`, add
`-allow_no_dhe_kex` to the server command; that mode has no forward secrecy.
`Python/net_echo.py` is a plaintext server and cannot serve this TLS test.
The demo uses the existing H7 AES/HASH and RNG providers, the existing
CryptoUecc P-256 implementation, and verifies OpenSSL's
reversed line response. Expected console output includes:

```text
TLS 1.3 PSK handshake: 10.5.5.51:2469
TLS authenticated: AES-128-GCM/SHA-256, PSK-DHE P-256
TLS echo checked 1
TLS echo checked 100
```

Stop/restart the server or unplug/replug the cable. The demo closes failed
sessions, wipes TLS state, and retries after three seconds once the network is
ready. Change one PSK byte on the server to check that authentication fails.
Hardware testing on October 21, 2026 confirmed authenticated encrypted echoes,
server restart recovery, wrong-PSK rejection, cable/DHCP/TLS recovery and
automatic recovery after the correct key was restored, without resetting the
board. These initial hardware results used PSK-only mode. Subsequent clean-build
P-256 PSK-DHE runs confirmed authenticated echoes and recovery after server
restart and cable reconnection, with the counter reaching 1800.

An intermittent PSK-DHE failure remains unresolved: timeout, transport and peer
errors occurred, and restarting OpenSSL restored service. It did not recur in
the subsequent diagnostic run; no soak duration was recorded. This is not a
confirmed fix.
The exact compiler/build profile was not supplied. See `docs/networking.md`
for hardware evidence and host tests. HTTP and ordinary TCP modes remain available.

For the wrong-key test, stop OpenSSL with Ctrl+C and change the first PSK byte
from `00` to `ff` in the command above. Leave the board unchanged. Expect
handshake failures and no new authenticated echoes. Stop OpenSSL again and
restore the original key beginning `00010203`; the board retries automatically.

For intermittent failures, append `-state -msg` to the OpenSSL command and
capture stdout and stderr, for example `2>&1 | tee tls-server.log`. Save that
file and the board console before restarting the server. Board diagnostics
report TCP-connect and TLS-init duration, state transitions, slow process calls,
and counters sampled before a failed call. Times are milliseconds. State values
are 0 idle, 1 ServerHello, 2 EncryptedExtensions, 3 server Finished, 4 client
Finished, 5 ready, 6 closing, 7 closed and 8 failed. Only counters and state are
logged by the board; keys and decrypted record buffers are not logged.

## MQTT broker test

Set these values in `src/board.h`, then rebuild the H753 library and example:

```c
#define NET_DEMO_MQTT_TEST 1
#define NET_DEMO_TLS_TEST 0
#define NET_DEMO_HTTP_TEST 0
#define NET_DEMO_TEST_COUNT 0
#define NET_DEMO_HOST "10.5.5.51"
#define NET_DEMO_MQTT_PORT 1883
```

Use the broker computer's actual LAN address. Assign a unique
NET_DEMO_MQTT_CLIENT_ID and NET_DEMO_MQTT_TOPIC for each board.
The example subscribes at QoS 1 and publishes a numbered message to that same
topic once a second. It validates the returned topic and payload and prints
`MQTT echo checked 1`, then every 100 messages. Use a topic without an existing
retained message, and do not publish unrelated messages to it during this test.

On a Mac with Homebrew, install Mosquitto if needed:

```sh
brew install mosquitto
```

Run this separate foreground broker configuration for the trusted test LAN.
This listener is plaintext and allows anonymous clients; it is not a deployment
configuration. Stop any other broker already using port 1883.

```sh
cat > mosquitto-demo.conf <<'EOF'
listener 1883 0.0.0.0
allow_anonymous true
persistence false
EOF
mosquitto -c mosquitto-demo.conf -v
```

Allow TCP port 1883 through the host firewall. Reset the board and expect DHCP,
`MQTT connecting`, `MQTT subscribed`, and increasing verified echo counts.
Stop/restart the broker and disconnect/reconnect the cable to check fresh
CONNECT, resubscription and resumed echoes without resetting the board.
Host tests pass; these MQTT hardware checks remain pending.

MQTT can also use the existing TLS layer by setting NET_DEMO_TLS_TEST to 1
and selecting a compatible broker endpoint and port. This uses the demo's
TLS 1.3 external PSK and P-256 profile, not certificate authentication.
The tested Mosquitto 2.0.18 PSK listener does not accept TLS 1.3, so do not
point this mode at that listener. The host test uses an OpenSSL TLS 1.3 endpoint
in front of the localhost broker to test the combined MQTT/TLS implementation.
Start hardware validation with plaintext MQTT above.

Broker references: [installation](https://mosquitto.org/download/) and
[configuration](https://mosquitto.org/man/mosquitto-conf-5.html).
