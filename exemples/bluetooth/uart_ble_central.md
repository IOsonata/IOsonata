# UartBleCentral throughput checks

Build the application and matching IOsonata library in Release (SoftDevice)
or Release_SDC. The nRF52840 Release_SDC application now uses size optimization,
matching the other Release configurations.

The BLE notification callback writes directly to UART. The UART TX FIFO holds
ten 244-byte notification payloads. There is no intermediate BLE-to-UART FIFO.
AppEvt is initialized with 12 entries before UART or BLE event sources start.
SDC requests 251-byte link-layer packets and 2120 us TX time, as BlePrbsRx does.

Application preprocessor settings:

| Setting | Default | Purpose |
| --- | --- | --- |
| CENTRAL_PHY | BT_GAP_PHY_1MBITS | Set BT_GAP_PHY_2MBITS for the 2M trial |
| CENTRAL_UART_BAUD | 1000000 | Set 2000000 for full 2M BLE throughput |
| CENTRAL_PRBS_CHECK | 0 | Set 1 to check PRBS at BLE entry for diagnosis |

Ordinary forwarding does not check PRBS byte by byte. The optional checker
retains state locally during each notification and publishes it once afterward.
Receive, accepted-byte, and refused-byte counters remain enabled in both builds.
The explicit stats command reports prbs_check so a disabled check cannot be
mistaken for measured zero discontinuities.

SoftDevice event diagnostics stop after the first payload notification to keep
later discovery/write/PHY text out of the binary stream. Explicit console
commands still produce text; stop the host PRBS measurement before requesting
stats. Connection and disconnection messages remain outside each stream.

Example 1M PHY test at the default UART rate:

```sh
../.venv/bin/python3 Python/uartprbs_rx.py --port /dev/cu.usbmodem40303180003282 --baud 1000000 --central-stats
```

For the 2M PHY test, compile with CENTRAL_PHY=BT_GAP_PHY_2MBITS and
CENTRAL_UART_BAUD=2000000, and use --baud 2000000 in the receiver command.

The firmware-only reference runs measured roughly 75 kB/s for SDC at 1M,
61 kB/s for SoftDevice at 1M, and 133 kB/s for both at 2M, without PRBS or
notification FIFO errors. The bridge changes still need hardware validation;
they do not establish a fix for the remaining SoftDevice 1M difference or the
previous host-observed serial discontinuities.
