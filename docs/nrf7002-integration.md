# nRF7002 integration into IOsonata

## Existing components

- DeviceIntrf/SPI: transport primitives
- EthernetDev_t: raw Ethernet frame ingress and egress
- IpInit/lwIP: IPv4, ARP, DHCP, DNS, ICMP, UDP, TCP
- SockIntrf: generic application sockets
- CoAP/MQTT/TLS: shared application-layer protocols

Do not duplicate the networking stack or introduce a second socket interface.

## Host configurations

Thingy:91 X has a shared SPI connection to nRF7002. It can be driven by the
nRF9151 for scan/positioning, or by nRF5340 for Wi-Fi networking. Exactly one
host owns the bus and Wi-Fi device at a time.

### nRF9151 (scan-only)

- Power sequencing, SPI arbitration and HOST_IRQ
- nRF70 host firmware/HAL bootstrap
- Scan request/results interface, exposing SSID, BSSID, channel, RSSI
- No Ethernet interface is required for scan-only use

### nRF5340 (networking)

- Host SPI, HOST_IRQ and BUCKEN integration
- Download/verify Nordic RPU firmware and initialize MAC control channels
- Authentication, association, regulatory-domain and connection-state handling
- Data-path queues into existing EthernetDev_t / shared IpInit
- BLE/Wi-Fi antenna coexistence and RF-switch control

## Nordic bare-metal driver identified

Nordic supplies a dedicated bare-metal integration library at
https://github.com/nrfconnect/nrf70-bm (`nrf70_bm_lib`). The upstream
repository depends on an `nrf_wifi` Git submodule and external nRF70
firmware patches. It contains scan-oriented bare-metal samples and a
Zephyr reference shim for hardware porting.

The upstream repository has no top-level SPDX license metadata recognized by
GitHub; inspect the license headers of each dependency before importing code.
Do not copy Zephyr MCU drivers into IOsonata. A minimal scan-only port should
implement Nordic's portable library platform hooks against IOsonata SPI,
GPIO, interrupt, allocation and timing services.

The bare-metal library is not by itself evidence of a complete station-mode
WPA2/WPA3 networking implementation; treat that as separate work.

## Dependencies not present in IOsonata

An nRF7002 host Wi-Fi stack is not in the current source tree. It is not
equivalent to the MCU SPI driver, and its firmware management, control protocol,
and data queues are not available through the generic Ethernet interface.

Nordic's host implementation, firmware packaging and supported interface need
to be evaluated before integrating any source; do not fabricate SPI commands
or treat register reads as a usable Wi-Fi driver. Observe existing project
restriction against copying Zephyr peripheral drivers into IOsonata.

## Work sequence

1. Audit `nrfconnect/nrf70-bm` API, dependency revision, license headers and
   firmware blobs, then port its scan-only hardware hooks.
2. Verify shared SPI muxing, host ownership, nPM6001 sequencing, HOST_IRQ,
   BUCKEN and nRF5340 coexistence wiring on PCA20065.
3. Bring up scan-only Wi-Fi positioning on nRF9151.
4. Add nRF5340 network operation using the same device HAL and the existing
   Ethernet/IP integration.
5. Build and validate both hosts on actual Thingy:91 X hardware.

## Reference

- https://docs.nordicsemi.com/r/bundle/ps_nrf7002/page/chapters/functional/doc/host_connection.html
- https://docs.nordicsemi.com/r/bundle/ug_thingy91x/page/ug/thingy91x/hw_description/nrf7002.html
