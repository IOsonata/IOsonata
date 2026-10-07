# Bluetooth example event loops

The IOsonata bare-metal BLE examples use `AppRun()`. Bluetooth, USB, and
application callbacks share AppEvt. Do not initialize or execute `app_sched`
in these applications.

`AppRun()` drains the event queue, calls `AppCheckStatus()` when it is empty,
and sleeps only when that check reports idle and no event is queued. The
default status check services linked Bluetooth and USB subsystems. Examples
without additional pending work use that default.

An application overriding `AppCheckStatus()` must also call the status checks
for its subsystems (`BtAppCheckStatus()` and, when used, `UsbCheckStatus()`).
The sensor examples `TPHSensorTag` and `BlueIOThingy` retain a timer request
until its AppEvt callback runs. Their status checks retry a refused callback
and report busy while the request remains. Sensor reads and advertising updates
therefore run from the main loop, including after queue saturation.

The UART, PRBS, DUT and USB/BLE bridge examples also retain application requests
until their callbacks run. Their `AppCheckStatus()` overrides retry those
requests after the queue drains, alongside the checks for their subsystems.

AppEvt copies the event ID and context pointer, not the pointed-to data. Pass
small values through the event ID, or use static/caller-owned context that
remains valid until the callback runs. Use `nullptr` when no context is needed.

The TaktOS and FreeRTOS BLE examples override `BtEvtQue()` with their worker
queue. Each worker drains its queue and calls `BtAppCheckStatus()` before
blocking. The USB/BLE TaktOS bridge checks each subsystem from its own worker.
The bridge also retries refused application work on the worker that owns it.
Its FIFO producer fills reserved spans before publishing them; consumers copy
spans before releasing them, so interrupts and thread switches cannot expose
unfilled data or overwrite data still being copied.

The legacy Nordic SDK applications `Thingy52`, `UartBleCentralSdk5`, and
`dfu_ble` use the SDK's own event handling, rather than the IOsonata Bluetooth
application layer. Their SDK scheduler dependencies are separate from AppEvt;
do not combine the two loops. Use `BlueIOThingy`, `uart_ble_central.cpp`, and
`ble_ota.cpp` for the corresponding IOsonata application examples.
