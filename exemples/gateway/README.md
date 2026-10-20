# IOsonata LTE/BLE UDP examples

Development branch: `ble_gateway_asset_tracker_udp` (based on IOsonata main).
These examples reuse the IOsonata LTE, UDP socket and BLE host implementations;
HciController runs independently on the companion nRF52840 of Thingy:91.
No Zephyr or NCS driver code is included.

## BleGateway — first integration milestone

Project: `ARM/Nordic/nRF91/nRF9160/exemples/BleGateway/ioc`
Application: `exemples/gateway/ble_gateway.cpp`.

- nRF9160 board configuration currently selects `NORDIC_THINGY91`.
- UART0 (115200) is the diagnostic console through HciController's CDC bridge.
- UART2 (1000000, RTS/CTS) connects to the nRF52840 HCI controller.
- BLE runs in observer mode, with active scanning and the latest advertising
  address, RSSI and cumulative report count sampled at the UDP send interval.
- LTE-M initialization, UDP socket, registration handling, timer, and
  application event queue are derived from the existing `LteUdp` example.
- UDP target currently uses the original echo-server development default,
  `echo.u-blox.com:7`, which is not a production telemetry backend.
  Set `LTE_UDP_HOST` and `LTE_UDP_PORT` in the board configuration.

**Status: built and hardware validated on Nordic Thingy:91 (October 16, 2026).**
The nRF9160 (modem firmware 1.3.6) with HciController on the nRF52840
successfully scanned BLE advertisers while sending and receiving LTE-M UDP
echo telemetry. Four consecutive messages (sequences 0–3) were echoed;
BLE report counts increased 552, 1044, 1217, 1676. Observed LTE-M network
registration was roaming, with intermittent unregistered/registered events.
This confirms the advertisement-observer UDP milestone only.
BLE central connections, GATT discovery/subscriptions, data batching, retry
persistence and secure cloud delivery are not implemented by this milestone.
An observer reports advertisements; it is not equivalent to a connected
sensor gateway yet.

## AssetTracker — next application

The tracker should reuse `exemples/lte/lte_udp.cpp` for LTE/UDP and
`exemples/gnss/gnss_demo.cpp` for GNSS. It will need timestamped location
records, fix validity, location acquisition scheduling and an offline queue.
Do not assume that the Thingy:91 nRF9160 supports GNSS: revision 1 with
modem firmware 1.2.8 does not; modem firmware 1.3.4 or newer on supported
hardware is required for the existing GNSS example.

## Test gates

1. Build the new IOcomposer project against the current IOsonata and Nordic
   modem libraries; review any linker diagnostics before flashing.
2. On Thingy:91, confirm HciController UART H:4 startup and BLE scan reports.
3. Confirm LTE-M registration and receive UDP datagrams at a controlled server.
4. Run simultaneous scanning and UDP telemetry while watching modem/HCI errors.
5. Add GATT notification forwarding and exercise a known sensor before claiming
   a complete BLE sensor gateway.
6. Build and validate AssetTracker independently after BLE gateway verification.

No changes to IOsonata `main` or to HciController firmware are required for
this branch's initial application integration.

## BlueIO SensorTag and original Nordic Thingy:52

The nRF52832 `ARM/Nordic/exemples/TPHSensorTag.cpp` broadcasts non-connectable
I-SYST manufacturer data. `ARM/Nordic/exemples/BlueIOThingy/BlueIOThingy.cpp`
also advertises compatible TPH/GAS payloads, but additionally exposes
connectable GATT sensor services. BleGateway now decodes the company ID and
TPH (pressure Pa, temperature in centi-degrees Celsius, relative humidity in
centi-percent) and GAS (gas resistance and air-quality index) records into UDP.
Only the latest decoded measurement is retained; this does not provide a
multi-device history.

The **original Nordic Thingy:52** is a different firmware/protocol target:
its native environmental sensor data is exposed through its GATT services.
The BlueIO advertising decoder does not automatically cover that device.
Thingy:52 service discovery and notifications remain to implement and test.
These additional source changes have not yet been rebuilt or hardware tested.
