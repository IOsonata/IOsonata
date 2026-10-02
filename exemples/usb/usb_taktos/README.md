# USB CDC with TaktOS

[usb_cdc_loopback_taktos.cpp](../usb_cdc_loopback_taktos.cpp) runs a CDC
loopback port and a periodic heartbeat thread. It demonstrates RTOS integration
with the existing USB stack and statically allocated thread and FIFO storage.

## Build

Place IOsonata and [TaktOS](https://github.com/IOsonata/TaktOS) beside each
other in the same directory. Use matching headers and libraries. This example
was syntax-checked against TaktOS revision `a97346327d6915fd95b7fe50ff50740f255b3fe5`.

1. Build `IOsonata/ARM/Nordic/nRF52/nRF52840/lib/ioc`: Debug or Release.
2. Build `TaktOS/ARM/cm4/ioc`: DebugFPU or ReleaseFPU respectively.
3. Open
   [UsbCdcLoopbackTaktOS/ioc](../../../ARM/Nordic/nRF52/nRF52840/exemples/UsbCdcLoopbackTaktOS/ioc)
   in IOcomposer and clean-build its matching Debug or Release configuration.

The project uses Cortex-M4 hard-float libraries, the ordinary nRF52840 linker
script and native USB controller. It does not require a SoftDevice or a board
pin map. TaktOS supplies its SysTick and PendSV handlers; do not add a second
scheduler tick or override those handlers. Keep the IOsonata C++ startup.

## Execution and ownership

- The USB thread initializes the core and CDC class, then enables USB.
- That thread alone calls `UsbProcess()`, `Rx()` and `Tx()`. On nRF52840,
  `UsbProcess()` also dispatches the shared AppEvt queue. Do not dispatch that
  queue concurrently from another thread.
- CDC is nonblocking. The thread retains a partly accepted packet and retries
  the unsent suffix after servicing USB again. A full TX FIFO must not stall
  the thread responsible for dispatching completions.
- Each service burst has at most 32 passes, followed by a one-tick sleep.
  At the configured 1000 Hz tick this permits lower-priority work to run.
  This is a periodic service example, not an interrupt-to-task wakeup bridge
  or a maximum-throughput benchmark.
- The lower-priority heartbeat thread increments `g_UsbTaktOSHeartbeat`
  about once per second. Inspect it in the debugger without adding text to
  the loopback stream.
- Other application threads should hand work to the USB owner through an
  application queue. This example does not make a CDC object safe for
  arbitrary concurrent callers.

There are no application USB callbacks and no RTOS calls from USB interrupts.
Controller IRQ work and deferred completion retain the existing stack path.
The USB thread keeps running while the cable is absent, allowing reattachment.

## Validate on hardware

Flash the image and use its native USB data connection. From the IOsonata root:

```bash
python3 Python/usb_cdc_loopback.py --port /dev/cu.usbmodemXXXX --duration 2000
```

Replace the port with the current host-assigned path. The example sends no
banner, so the runner's banner warning is expected. Check byte integrity and
the final result, and confirm that the heartbeat advances during traffic.
Also test starting without a cable, reconnecting, and reopening the port.
Restart the host runner after reconnecting; USB removal is not a lossless
continuation of an old serial session.

Host syntax and project-structure checks do not validate scheduling or IRQ
integration on the board. A clean ARM build and hardware run are still required.
