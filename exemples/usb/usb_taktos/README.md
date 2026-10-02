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
- CDC FIFOs use `bBlocking = true` to preserve queued data and apply
  backpressure when full. This flag does not put the calling thread to sleep.
  The thread retains a partly accepted packet and retries
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

## Multithreaded composite stress

[usb_combo_stress_taktos.cpp](../usb_combo_stress_taktos.cpp) uses the same
device composition as the bare-metal combo example, from
[usb_combo_stress_device.h](../usb_combo_stress_device.h). Build the
[UsbComboStressTaktOS/ioc](../../../ARM/Nordic/nRF52/nRF52840/exemples/UsbComboStressTaktOS/ioc)
project with the same library configurations listed above.

| Thread | Work | Stack budget, excluding kernel overhead |
|---|---|---:|
| Service | `UsbProcess()`, sole AppEvt dispatcher | 2048 bytes |
| Loopback | CDC0 RX checking and echo, including partial TX | 1024 bytes |
| PRBS | CDC1 PRBS TX and target-error markers | 1024 bytes |
| Heartbeat | Increment `g_UsbComboTaktOSHeartbeat` every second | 512 bytes |

All four use normal priority. The service thread yields after each USB pass.
Each CDC thread runs four of its original work iterations before yielding;
the heartbeat sleeps between increments. PRBS still generates and transmits
one byte per call, matching the bare-metal workload. This amortizes scheduling
without introducing packet-sized PRBS writes or changing FIFO capacities. Yield only schedules
equal- or higher-priority ready threads, so this continuously active stress
example does not give CPU time to lower-priority threads or the idle thread.
Do not copy that policy into a low-power product unchanged.

Each CDC data path has one application owner. Both CDC instances keep the
bare-metal `bBlocking = true` FIFO policy: full RX applies backpressure and
full TX returns the accepted byte count without overwriting queued data.
`false` allows drops and is unsuitable for this integrity test.
The loopback thread publishes a cumulative error count through a lock-free
32-bit atomic; the PRBS thread sends the same zero-byte target-error markers
as the bare-metal version. Thread memory is static and sized with
`TAKTOS_THREAD_MEM_SIZE`.

HID, raw INT and ISO keep their existing callback execution paths. They are
not separate threads, and ISO frames are not delayed until a scheduler tick.
USB class initialization completes before starting the scheduler. No traffic
thread reinitializes a class or dispatches AppEvt.

The VID/PID, banner, descriptor layout, alternate settings, packet formats,
FIFO capacities and ISO diagnostic request match the bare-metal composite.
Additional thread stacks and scheduler storage must be included in memory
comparisons. Thread scheduling changes the traffic mix, so compare measured
results rather than assuming identical throughput.

Run the existing composite host test:

```bash
python3 Python/usb_combo_stress.py \
  --loop-port /dev/cu.usbmodemXXXX01 \
  --prbs-port /dev/cu.usbmodemXXXX03 --duration 2000
```

Confirm the heartbeat advances while all five functions carry traffic.
Record the complete result, firmware revision, TaktOS revision and clean-build
size. Repeat cable-absent startup and reconnect tests. This new integration
still requires an ARM build and hardware validation; the bare-metal PASS does
not validate TaktOS context switching under composite traffic.
