# Ethernet and connected sockets

`SockIntrf` presents connected UDP and TCP through `DeviceIntrf`, with C helpers
and a C++ wrapper. nRF91 uses its modem's IP stack. STM32H753 now uses the shared
`src/net/ip_lwip.cpp` service over an injected `EthernetDev_t`. Protocol clients
can use the same socket interface on either target. The nRF91 implementation
and public socket signatures are unchanged.

NUCLEO-H753ZI hardware tests confirm DHCP, ping, UDP/TCP echo and TCP recovery
after server restart and cable reconnection. See the hardware results below
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

## Validation

```sh
python3 tests/net/run.py
python3 tests/stm32h7/run_ethernet.py
python3 tests/stm32h7/run.py
```

`LWIP_ROOT` can select a different local lwIP checkout. Host tests require GCC;
Arm compilation needs GNU Arm GCC. Register models require Python `unicorn`
and `pyelftools`.

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

The UDP startup log received replies 0-3 together but did not show replies 4-7;
the cause was not established. This is basic echo validation, not a loss-free
UDP endurance result. DNS hostnames, DHCP lease expiry/address changes,
large payloads, sustained load, throughput and D-cache-enabled operation still
need hardware tests. The numeric-IP echo tests did not exercise DNS.

## References

- [ST MB1364-H753ZI-E01 schematic](https://www.st.com/resource/en/schematic_pack/mb1364-h753zi-e01-schematic.pdf)
- [STM32H7 reference manual RM0433](https://www.st.com/resource/en/reference_manual/rm0433-stm32h742-stm32h743753-and-stm32h750-value-line-advanced-armbased-32bit-mcus-stmicroelectronics.pdf)
- [ST AN4839: Cortex-M7 cache and DMA](https://www.st.com/resource/en/application_note/an4839-level-1-cache-on-stm32f7-series-and-stm32h7-series-stmicroelectronics.pdf)
- [lwIP raw API](https://www.nongnu.org/lwip/2_2_x/raw_api.html)
