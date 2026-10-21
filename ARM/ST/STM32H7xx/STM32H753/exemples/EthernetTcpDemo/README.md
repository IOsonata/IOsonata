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

On the PC run OpenSSL 3 (the macOS system LibreSSL command may not support this):

```sh
openssl s_server -accept 2469 -nocert -tls1_3 -allow_no_dhe_kex \
  -ciphersuites TLS_AES_128_GCM_SHA256 -psk_identity IOsonata \
  -psk 000102030405060708090a0b0c0d0e0f101112131415161718191a1b1c1d1e1f -rev
```

This uses a PUBLIC TEST KEY, PSK-only authentication and no forward secrecy.
`Python/net_echo.py` is a plaintext server and cannot serve this TLS test.
The demo uses the existing H7 AES/HASH and RNG providers and verifies OpenSSL's
reversed line response. Expected console output includes:

```text
TLS 1.3 PSK handshake: 10.5.5.51:2469
TLS authenticated: AES-128-GCM/SHA-256, PSK-only
TLS echo checked 1
TLS echo checked 100
```

Stop/restart the server or unplug/replug the cable. The demo closes failed
sessions, wipes TLS state, and retries after three seconds once the network is
ready. Change one PSK byte on the server to check that authentication fails.
Hardware testing on October 21, 2026 confirmed authenticated encrypted echoes,
server restart recovery, wrong-PSK rejection and automatic recovery after the
correct key was restored, without resetting the board. The cable test showed
DHCP recovery and a new handshake; its completed TLS recovery still needs a log.
The exact compiler/build profile was not supplied. See `docs/networking.md`
for hardware evidence and host tests. HTTP and ordinary TCP modes remain available.

For the wrong-key test, stop OpenSSL with Ctrl+C and change the first PSK byte
from `00` to `ff` in the command above. Leave the board unchanged. Expect
handshake failures and no new authenticated echoes. Stop OpenSSL again and
restore the original key beginning `00010203`; the board retries automatically.
