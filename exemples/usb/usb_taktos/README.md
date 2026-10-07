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
- That thread alone runs the USB work and calls `Rx()` and `Tx()`. The
  example overrides `UsbEvtQue()`: the USB stack puts its process event (bus
  reset, suspend, resume, cable) in a queue owned by this thread, which runs
  it. On nRF52 and nRF54 endpoint events run in the USB interrupt.
  The application event queue is not used or linked.
- CDC FIFOs use `bBlocking = true` to preserve queued data and apply
  backpressure when full. This flag does not put the calling thread to sleep.
  The thread retains a partly accepted packet and retries
  the unsent suffix after servicing USB again. A full TX FIFO must not stall
  the thread responsible for dispatching completions.
- The thread runs the queued USB work, then one loopback step. When the step
  moved nothing and the queue is empty it blocks on a binary semaphore. The
  CDC event callback gives it on received data and on an emptied TX FIFO,
  and `UsbEvtQue()` gives it for every queued event, so a received packet, TX
  space or a cable change wakes it at once on either port. There is no
  periodic sleep: a one tick sleep per pass left the endpoints NAKing for up
  to 1 ms.
- The lower-priority heartbeat thread increments `g_UsbTaktOSHeartbeat`
  about once per second. It runs while the USB thread waits for an event.
  Inspect it in the debugger without adding text to the loopback stream.
- Other application threads should hand work to the USB owner through an
  application queue. This example does not make a CDC object safe for
  arbitrary concurrent callers.

The CDC event callback runs in the USB interrupt on nRF52 and nRF54.
It only gives the semaphore, an interrupt safe call, as the
`UsbEvtQue()` override does.
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
device composition as the bare-metal combo example. Each benchmark contains
its own device state, callbacks and initialization.
[usb_combo_stress.h](../usb_combo_stress.h) contains only constants and data
types. CDC calls use concrete objects directly to preserve the benchmark
comparison with TinyUSB. Build the
[UsbComboStressTaktOS/ioc](../../../ARM/Nordic/nRF52/nRF52840/exemples/UsbComboStressTaktOS/ioc)
project with the same library configurations listed above.

| Thread | Work | Stack budget, excluding kernel overhead |
|---|---|---:|
| Service | USB work queue (`UsbEvtQue()` override) | 2048 bytes |
| Loopback | CDC0 RX checking and echo, including partial TX | 1024 bytes |
| PRBS | CDC1 PRBS TX and target-error markers | 1024 bytes |
| Heartbeat | Increment `g_UsbComboTaktOSHeartbeat` every second | 512 bytes |

The service thread runs at high priority, the three others at normal. On
nRF52 and nRF54, endpoint readiness and completion callbacks run in the USB
interrupt. They publish received data and advance queued transfers there.
The service thread handles the USB process event and any other work sent
through `UsbEvtQue()`. It drains its queue and blocks on a binary semaphore
that `UsbEvtQue()` gives when work is posted.
Loopback receives and echoes in the same pass, up to four packets per turn.
It yields early on empty RX, closed port, or partial/full TX backpressure,
retaining any unsent suffix. PRBS still generates and transmits one byte per
call, up to four accepted bytes per turn, and yields early when the port is
closed or TX cannot accept a byte. The heartbeat sleeps between increments.
FIFO capacities and one-byte PRBS calls match the bare-metal workload.
The traffic threads yield to each other and never block, so this continuously
active stress example does not give CPU time to lower-priority threads or the
idle thread. Do not copy that policy into a low-power product unchanged.

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
thread reinitializes a class or runs the USB work queue.

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
