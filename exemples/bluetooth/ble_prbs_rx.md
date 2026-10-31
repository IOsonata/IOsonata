# BlePrbsRx

Standalone firmware counterpart to `Python/hciprbs_rx.py --phy both --duration 60`.
It checks notification payloads in firmware. UART carries text results only.
UartBleCentral remains a separate application.

Import an Eclipse managed project:

- `ARM/Nordic/nRF52/nRF52832/exemples/BlePrbsRx/ioc`
- `ARM/Nordic/nRF52/nRF52840/exemples/BlePrbsRx/ioc`
- `ARM/ST/STM32WBAxx/STM32WBA65/exemples/BlePrbsRx/ioc`

Use `Release` for S132/S140 or `Release_SDC` for SDC, with the matching Release
IOsonata library. For STM32WBA65I-DK1 use `Release` and rebuild the
`IOsonata_STM32WBA65` Release library first. Its UART is the first ST-LINK
VCP (USART1, TX PB12 / RX PA8). The WBA project uses the same ST BLE stack
and link-layer libraries as UartBleCentral. Board pins are in the new project's `src/board.h`, copied
from the corresponding UartBleCentralDemo board configuration.

Start the `BlePrbs_SDC` transmitter. Open a serial terminal at 1000000 baud,
8N1, without flow control. Do not use `uartprbs_rx.py`: this firmware sends
text, not the PRBS payload.

Example terminal command:

```sh
../.venv/bin/python3 -m serial.tools.miniterm /dev/cu.usbmodem40303180003282 1000000
```

The receiver requests a 15 ms connection interval, zero latency, and ATT MTU
247. SDC and STM32WBA explicitly request 251-byte link-layer packets and 2120 us TX time;
SoftDevice uses the existing IOsonata GATT data-length negotiation.
Notifications are enabled only after the requested PHY and MTU are established.

The sequence is automatic:

1. Connect at 1M PHY and measure 60 seconds from the first notification.
2. Print final results and disconnect.
3. Reconnect at 2M PHY and measure another 60 seconds.
4. Print final results and disconnect. Reset the board to repeat.

Status prints every second: interval bytes/second, cumulative average,
elapsed milliseconds, bytes, notifications, raw PRBS discontinuities (`drop`),
out-of-frame bytes, payload length range, and average notifications per 15 ms.
The 15 ms figure uses the requested connection interval; SoftDevice rejects a
trial if its connection events report a different interval or nonzero latency.
The SDC and WBA receiver paths do not expose the negotiated interval here;
the notifications/15ms figure is based on the requested interval.

`drop` uses the same adjacent-byte PRBS successor check as the Python receiver.
There are no missing-byte estimates: PRBS repeats every 127 bytes and cannot
uniquely determine long losses. The BLE callback copies each notification, its length, and arrival timestamp
into a 12-packet CFIFO, then queues `PrbsReceive` through AppEvt. The AppEvt
queue is explicitly initialized with 12 entries. PRBS checking and UART
reporting run from the main loop; no payload is forwarded to UART. A refused
AppEvt is retried without discarding the queued payload. Notification FIFO
overflow fails the trial and is reported in the final queue-drop counter.

The minimum average notification targets match the Python test: four per
15 ms at 1M, eight at 2M. These are minimum thresholds, not the measured HCI
baseline. Compare actual rates against the supplied HCI results: about
75847 B/s at 1M and 134910 B/s at 2M, both with zero discontinuities.

Validation: host tests simulate the two complete trials, check one-second
reports, reject the wrong PHY before subscription, detect damaged PRBS, and
check timer wraparound. These tests are not radio throughput measurements.

## STM32WBA central receive review

The WBA port uses ST's native host, not the IOsonata HCI host. Received events
are copied from `BLECB_Indication` into the port event ring, then routed from
`BtEvtQue` to `BtGattClientNotified`. The receiver copies the notification to
its CFIFO and checks it in `PrbsReceive`.

The central now exchanges MTU before starting primary-service discovery. This
avoids the previous ordering where the application started discovery in its
connected callback and the subsequent MTU command could be refused. Discovery
waits for ST's procedure-complete event, keeps its cursor on command refusal,
and retries from the status check. WBA also implements the existing data-length
request and the receiver reads the actual PHY for both trials.

The event copy includes a separately supplied event tail and restores the
previous interrupt mask. A full event ring returns refusal so the ST host can
retry after the ring drains. The copy and application callbacks run outside
`BleStack_Process`; stack calls are not made from its indication callback.

The existing host event FIFO is 256 16-bit words and the port ring holds eight
events. These are possible burst limits, not measured causes of reduced
throughput. `CFG_BLE_HOST_EVT_FIFO_SIZE` and `BT_APP_WBA_EVT_SLOTS` are library
build settings; defining them only in the application does not resize the
compiled library. Connection-event length hints remain automatic (zero).
Measure the two trials before changing these budgets.

Validation includes host tests for MTU/discovery ordering, command refusal,
independent peers, data-length bounds, split event copying, full-ring refusal,
and saved interrupt state at -O0 and -O2. The shared PRBS test runs with both
SDC and WBA selected. The complete receiver source also passes a host syntax check using the WBA
board definitions. A full port build requires the Arm toolchain and CubeWBA
headers; neither a firmware link nor hardware throughput was tested here.
