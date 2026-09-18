""" -------------------------------------------------------------------------
@file   usb_dual_cdc_stress.py

@brief  Simultaneous USB dual-CDC loopback and PRBS integrity test

The target must run exemples/usb/usb_dual_cdc_stress.cpp. CDC instance zero
is driven in sustained full-duplex loopback while CDC instance one is checked
as a continuous device-to-host PRBS stream. Both checks run concurrently to
exercise controller DMA arbitration across multiple endpoints.

@param  --loop-port   CDC port for the loopback function
        --prbs-port   CDC port for the PRBS transmit function
        --baud        CDC line coding value (default 1000000)
        --duration    Active test duration in seconds (default 60)
        --trace       Host-side startup and I/O activity diagnostics

@author Hoang Nguyen Hoan
@date   Sep. 5, 2026

@license

MIT License

Copyright (c) 2026 I-SYST inc. All rights reserved.

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.

----------------------------------------------------------------------------
"""

import argparse
import sys
import threading
import time

import serial


LOOPBACK_BANNER = b"IOsonata USB Dual CDC Loopback"


def prbs8(curval):
    newbit = ((curval >> 6) ^ (curval >> 5)) & 1
    return ((curval << 1) | newbit) & 0x7f


def make_prbs_block(state, length):
    data = bytearray(length)

    for index in range(length):
        state = prbs8(state)
        data[index] = state

    return bytes(data), state


def check_stream(data, expected):
    errors = 0

    for value in data:
        if expected is not None and value != expected:
            errors += 1

        # Resynchronize after a missing or corrupt byte so one fault does not
        # make every following byte appear wrong.
        expected = prbs8(value)

    return expected, errors


def check_prbs_stream(data, expected):
    errors = 0
    target_rx_errors = 0

    for value in data:
        # PRBS8() produces only 1..127. The target inserts zero when its
        # loopback RX checker sees a discontinuity, without advancing PRBS.
        if value == 0:
            target_rx_errors += 1
            continue

        if expected is not None and value != expected:
            errors += 1

        expected = prbs8(value)

    return expected, errors, target_rx_errors


class TestStats:
    def __init__(self):
        self.lock = threading.Lock()
        self.loop_tx = 0
        self.loop_rx = 0
        self.loop_errors = 0
        self.prbs_rx = 0
        self.prbs_errors = 0
        self.target_rx_errors = 0
        self.io_error = None

    def add_loop_tx(self, count):
        with self.lock:
            self.loop_tx += count

    def add_loop_rx(self, count, errors):
        with self.lock:
            self.loop_rx += count
            self.loop_errors += errors

    def add_prbs_rx(self, count, errors, target_rx_errors):
        with self.lock:
            self.prbs_rx += count
            self.prbs_errors += errors
            self.target_rx_errors += target_rx_errors

    def set_io_error(self, source, error):
        with self.lock:
            if self.io_error is None:
                self.io_error = "%s: %s" % (source, error)

    def snapshot(self):
        with self.lock:
            return (self.loop_tx, self.loop_rx, self.loop_errors,
                    self.prbs_rx, self.prbs_errors, self.target_rx_errors,
                    self.io_error)


class HostTrace:
    """Record host calls; never print from the traffic threads."""

    def __init__(self, enabled=False):
        self.enabled = enabled
        self.started = time.monotonic()
        self.lock = threading.Lock()
        self.streams = {
            name: dict(calls=0, returns=0, empty=0, short=0, bytes=0,
                       first=None, last=None, last_return=None, pending=None,
                       requested=0, sample=None, error=None)
            for name in ("loop TX", "loop RX", "PRBS RX")
        }

    def event(self, message):
        if self.enabled:
            print("TRACE +%.3fs %s" %
                  (time.monotonic() - self.started, message), flush=True)

    @staticmethod
    def property(comm, name):
        try:
            return getattr(comm, name)
        except (OSError, serial.SerialException, AttributeError,
                NotImplementedError) as error:
            return "unavailable (%s)" % error

    def port(self, label, comm):
        if self.enabled:
            self.event("%s port=%s host_dtr=%s in_waiting=%s out_waiting=%s "
                       "read_timeout=%s write_timeout=%s" %
                       (label, comm.port, self.property(comm, "dtr"),
                        self.property(comm, "in_waiting"),
                        self.property(comm, "out_waiting"),
                        comm.timeout, comm.write_timeout))

    def before_flush(self, label, comm, direction):
        if self.enabled:
            name = "in_waiting" if direction == "input" else "out_waiting"
            self.event("%s resetting %s buffer; host queued bytes=%s" %
                       (label, direction, self.property(comm, name)))

    def begin(self, source, requested):
        if self.enabled:
            with self.lock:
                stream = self.streams[source]
                stream["calls"] += 1
                stream["pending"] = time.monotonic()
                stream["requested"] = requested

    def finish(self, source, count=0, data=None, error=None):
        if self.enabled:
            with self.lock:
                stream = self.streams[source]
                now = time.monotonic()
                stream["pending"] = None
                stream["last_return"] = now
                if error is not None:
                    stream["error"] = repr(error)
                    return
                stream["returns"] += 1
                stream["bytes"] += count
                if count == 0:
                    stream["empty"] += 1
                elif count < stream["requested"]:
                    stream["short"] += 1
                if count > 0:
                    if stream["first"] is None:
                        stream["first"] = now
                        if data is not None:
                            stream["sample"] = data[:16].hex(" ")
                    stream["last"] = now

    def snapshot(self, reason, stats, ports, threads):
        if not self.enabled:
            return
        with self.lock:
            streams = {name: dict(value)
                       for name, value in self.streams.items()}
        now = time.monotonic()
        loop_tx, loop_rx, _, prbs_rx, _, _, io_error = stats.snapshot()
        self.event("%s: loop_tx=%d loop_rx=%d pending=%d prbs_rx=%d" %
                   (reason, loop_tx, loop_rx, loop_tx - loop_rx, prbs_rx))

        def age(when):
            return "never" if when is None else "%.3fs ago" % (now - when)

        for thread in threads:
            stream = streams[thread.name]
            pending = stream["pending"]
            call = ("idle" if pending is None else
                    "pending %dB for %.3fs" %
                    (stream["requested"], now - pending))
            first = ("never" if stream["first"] is None else
                     "+%.3fs" % (stream["first"] - self.started))
            self.event("%s alive=%s calls=%d returns=%d empty=%d short=%d "
                       "bytes=%d first_data=%s last_data=%s last_return=%s "
                       "call=%s" %
                       (thread.name, thread.is_alive(), stream["calls"],
                        stream["returns"], stream["empty"], stream["short"],
                        stream["bytes"], first, age(stream["last"]),
                        age(stream["last_return"]), call))
            if stream["sample"] is not None:
                self.event("%s first read prefix: %s" %
                           (thread.name, stream["sample"]))
            if stream["error"] is not None:
                self.event("%s exception: %s" %
                           (thread.name, stream["error"]))
        for label, comm in ports:
            self.port(label, comm)
        if io_error is not None:
            self.event("I/O error: %s" % io_error)
        if "stalled" in reason:
            for name in ("loop RX", "PRBS RX"):
                stream = streams[name]
                if stream["first"] is None:
                    self.event("%s: no bytes reached Python during the active "
                               "test; %d empty reads returned" %
                               (name, stream["empty"]))
                else:
                    self.event("%s: traffic started; last bytes %s" %
                               (name, age(stream["last"])))
            self.event("Host trace cannot distinguish firmware submission, "
                       "DMA, or AppEvt stalls.")


def parse_args():
    parser = argparse.ArgumentParser(
        description="IOsonata simultaneous dual-CDC stress test")
    parser.add_argument(
        "--loop-port", required=True,
        help="loopback CDC port, e.g. /dev/cu.usbmodemXXXX, COM5")
    parser.add_argument(
        "--prbs-port", required=True,
        help="PRBS CDC port, e.g. /dev/cu.usbmodemYYYY, COM6")
    parser.add_argument(
        "--baud", type=int, default=1_000_000,
        help="CDC line coding value (default: 1000000)")
    parser.add_argument(
        "--duration", type=float, default=60.0,
        help="active test duration in seconds (default: 60)")
    parser.add_argument(
        "--block", type=int, default=4096,
        help="loopback write block size in bytes (default: 4096)")
    parser.add_argument(
        "--read-size", type=int, default=4096,
        help="serial read block size in bytes (default: 4096)")
    parser.add_argument(
        "--report", type=float, default=1.0,
        help="throughput report interval in seconds (default: 1)")
    parser.add_argument(
        "--drain-timeout", type=float, default=3.0,
        help="time to wait for the final loopback echo (default: 3)")
    parser.add_argument(
        "--stall-timeout", type=float, default=3.0,
        help="maximum time either active stream may stop (default: 3)")
    parser.add_argument(
        "--trace", action="store_true",
        help="log host startup, DTR errors, I/O activity and live stall state")
    return parser.parse_args()


def hold_port_closed(comm, trace, label):
    trace.event("%s requesting DTR=False" % label)
    try:
        comm.dtr = False
        trace.event("%s DTR=False setter returned successfully" % label)
        time.sleep(0.05)
    except (OSError, serial.SerialException) as error:
        trace.event("%s DTR=False FAILED: %r" % (label, error))

    trace.before_flush(label, comm, "input")
    comm.reset_input_buffer()
    trace.before_flush(label, comm, "output")
    comm.reset_output_buffer()


def open_loopback_port(comm, trace):
    trace.event("loop requesting DTR=True")
    try:
        comm.dtr = True
        trace.event("loop DTR=True setter returned successfully")
    except (OSError, serial.SerialException) as error:
        trace.event("loop DTR=True FAILED: %r" % error)

    deadline = time.monotonic() + 1.0
    received = bytearray()

    while time.monotonic() < deadline:
        waiting = comm.in_waiting
        data = comm.read(waiting if waiting > 0 else 1)

        if data:
            received.extend(data)
            if LOOPBACK_BANNER in received:
                break

    # The banner is not part of the loopback PRBS stream.
    trace.event("loop banner wait: received=%d banner_found=%s prefix=%r" %
                (len(received), LOOPBACK_BANNER in received,
                 bytes(received[:64])))
    time.sleep(0.05)
    trace.before_flush("loop", comm, "input")
    comm.reset_input_buffer()
    return LOOPBACK_BANNER in received


def open_prbs_port(comm, trace):
    trace.event("PRBS requesting DTR=True")
    try:
        comm.dtr = True
        trace.event("PRBS DTR=True setter returned successfully")
    except (OSError, serial.SerialException) as error:
        trace.event("PRBS DTR=True FAILED: %r" % error)


def loopback_writer(comm, stop_event, abort_event, stats, block_size, trace):
    state = 0xff
    pending = b""

    while not stop_event.is_set() and not abort_event.is_set():
        if not pending:
            pending, state = make_prbs_block(state, block_size)

        try:
            trace.begin("loop TX", len(pending))
            count = comm.write(pending)
            trace.finish("loop TX", count)
        except (OSError, serial.SerialException,
                serial.SerialTimeoutException) as error:
            trace.finish("loop TX", error=error)
            stats.set_io_error("loopback write", error)
            abort_event.set()
            return

        if count > 0:
            stats.add_loop_tx(count)
            pending = pending[count:]


def loopback_reader(comm, stop_event, abort_event, stats, read_size, trace):
    expected = prbs8(0xff)

    while not stop_event.is_set() and not abort_event.is_set():
        try:
            trace.begin("loop RX", read_size)
            data = comm.read(read_size)
            trace.finish("loop RX", len(data), data)
        except (OSError, serial.SerialException) as error:
            trace.finish("loop RX", error=error)
            stats.set_io_error("loopback read", error)
            abort_event.set()
            return

        if data:
            expected, errors = check_stream(data, expected)
            stats.add_loop_rx(len(data), errors)


def prbs_reader(comm, stop_event, abort_event, stats, read_size, trace):
    # Synchronize to the first byte observed. The device may have transmitted
    # before the host reader thread was scheduled.
    expected = None

    while not stop_event.is_set() and not abort_event.is_set():
        try:
            trace.begin("PRBS RX", read_size)
            data = comm.read(read_size)
            trace.finish("PRBS RX", len(data), data)
        except (OSError, serial.SerialException) as error:
            trace.finish("PRBS RX", error=error)
            stats.set_io_error("PRBS read", error)
            abort_event.set()
            return

        if data:
            expected, errors, target_rx_errors = check_prbs_stream(
                data, expected)
            stats.add_prbs_rx(len(data), errors, target_rx_errors)


def open_serial(port, baud):
    return serial.Serial(
        port=port,
        baudrate=baud,
        timeout=0.1,
        write_timeout=1.0,
        rtscts=False,
        dsrdtr=False)


def main():
    args = parse_args()
    trace = HostTrace(args.trace)

    if (args.loop_port == args.prbs_port or args.duration <= 0 or
            args.block <= 0 or args.read_size <= 0 or args.report <= 0 or
            args.drain_timeout < 0 or args.stall_timeout <= 0):
        print("ERROR: ports must differ and numeric arguments must be valid",
              file=sys.stderr)
        return 2

    loop_comm = None
    prbs_comm = None

    trace.event("Host diagnostics enabled; host_dtr is the requested host "
                "state, not firmware acknowledgement. TX bytes are accepted "
                "by the host driver, not proof of device reception.")
    try:
        trace.event("opening loop port %s" % args.loop_port)
        loop_comm = open_serial(args.loop_port, args.baud)
        trace.port("loop", loop_comm)
        trace.event("opening PRBS port %s" % args.prbs_port)
        prbs_comm = open_serial(args.prbs_port, args.baud)
        trace.port("PRBS", prbs_comm)
    except (OSError, serial.SerialException) as error:
        print("ERROR: cannot open both CDC ports: %s" % error,
              file=sys.stderr)
        if loop_comm is not None:
            loop_comm.close()
        if prbs_comm is not None:
            prbs_comm.close()
        return 2

    try:
        hold_port_closed(loop_comm, trace, "loop")
        hold_port_closed(prbs_comm, trace, "PRBS")

        if open_loopback_port(loop_comm, trace):
            print("Connected to IOsonata USB Dual CDC Loopback")
        else:
            print("WARNING: loopback banner not detected")

        open_prbs_port(prbs_comm, trace)

        traffic_stop = threading.Event()
        reader_stop = threading.Event()
        abort_event = threading.Event()
        stats = TestStats()
        threads = [
            threading.Thread(
                name="loop TX",
                target=loopback_writer,
                args=(loop_comm, traffic_stop, abort_event, stats, args.block,
                      trace),
                daemon=True),
            threading.Thread(
                name="loop RX",
                target=loopback_reader,
                args=(loop_comm, reader_stop, abort_event, stats,
                      args.read_size, trace),
                daemon=True),
            threading.Thread(
                name="PRBS RX",
                target=prbs_reader,
                args=(prbs_comm, reader_stop, abort_event, stats,
                      args.read_size, trace),
                daemon=True),
        ]

        start_time = time.monotonic()
        end_time = start_time + args.duration
        report_time = start_time
        report_loop_tx = 0
        report_loop_rx = 0
        report_prbs_rx = 0
        last_loop_progress = start_time
        last_prbs_progress = start_time
        observed_loop_rx = 0
        observed_prbs_rx = 0
        stall_error = None
        ports = (("loop", loop_comm), ("PRBS", prbs_comm))

        trace.event("starting traffic threads; duration=%.3fs "
                    "stall_timeout=%.3fs" %
                    (args.duration, args.stall_timeout))
        for thread in threads:
            thread.start()

        try:
            while time.monotonic() < end_time and not abort_event.is_set():
                time.sleep(0.02)
                now = time.monotonic()
                loop_tx, loop_rx, loop_errors, prbs_rx, prbs_errors, \
                    target_rx_errors, _ = \
                    stats.snapshot()

                if loop_rx != observed_loop_rx:
                    observed_loop_rx = loop_rx
                    last_loop_progress = now
                if prbs_rx != observed_prbs_rx:
                    observed_prbs_rx = prbs_rx
                    last_prbs_progress = now

                if now - start_time >= args.stall_timeout:
                    if now - last_loop_progress >= args.stall_timeout:
                        stall_error = "loopback receive stalled"
                        break
                    if now - last_prbs_progress >= args.stall_timeout:
                        stall_error = "PRBS receive stalled"
                        break

                if now - report_time >= args.report:
                    elapsed = now - report_time
                    loop_tx_rate = (loop_tx - report_loop_tx) / elapsed
                    loop_rx_rate = (loop_rx - report_loop_rx) / elapsed
                    prbs_rx_rate = (prbs_rx - report_prbs_rx) / elapsed
                    pending = loop_tx - loop_rx

                    print(
                        "Loop Tx/Rx B/s : %.2f / %.2f, errors %d, "
                        "pending %d | PRBS Rx B/s : %.2f, errors %d, "
                        "target RX errors %d" %
                        (loop_tx_rate, loop_rx_rate, loop_errors, pending,
                         prbs_rx_rate, prbs_errors, target_rx_errors),
                        flush=True)
                    trace.snapshot("active", stats, ports, threads)

                    report_time = now
                    report_loop_tx = loop_tx
                    report_loop_rx = loop_rx
                    report_prbs_rx = prbs_rx
        except KeyboardInterrupt:
            print("KeyboardInterrupt. Stopping test.")

        # Capture the live failure before stopping threads or draining data.
        trace.snapshot(stall_error or "stopping traffic", stats, ports, threads)
        traffic_stop.set()
        threads[0].join(timeout=1.5)
        if threads[0].is_alive():
            stats.set_io_error("loopback write", "writer did not stop")
            abort_event.set()

        active_end = time.monotonic()
        active_snapshot = stats.snapshot()

        # Keep both readers running while every byte accepted by the host
        # loopback writer is drained from the device.
        if not abort_event.is_set():
            drain_deadline = time.monotonic() + args.drain_timeout
            while time.monotonic() < drain_deadline:
                loop_tx, loop_rx, _, _, _, _, _ = stats.snapshot()
                if loop_rx >= loop_tx:
                    break
                time.sleep(0.01)

        reader_stop.set()
        for thread in threads[1:]:
            thread.join(timeout=0.5)
            if thread.is_alive():
                stats.set_io_error("serial read", "reader did not stop")

        trace.snapshot("after drain", stats, ports, threads)
        loop_tx, loop_rx, loop_errors, prbs_rx, prbs_errors, \
            target_rx_errors, io_error = \
            stats.snapshot()
        active_loop_tx, active_loop_rx, _, active_prbs_rx, _, _, _ = \
            active_snapshot
        elapsed = active_end - start_time
        loop_tx_rate = active_loop_tx / elapsed if elapsed > 0 else 0.0
        loop_rx_rate = active_loop_rx / elapsed if elapsed > 0 else 0.0
        prbs_rx_rate = active_prbs_rx / elapsed if elapsed > 0 else 0.0

        print()
        print("Loop TX bytes  : %d" % loop_tx)
        print("Loop RX bytes  : %d" % loop_rx)
        print("Loop TX B/sec  : %.2f" % loop_tx_rate)
        print("Loop RX B/sec  : %.2f" % loop_rx_rate)
        print("Loop errors    : %d" % loop_errors)
        print("Loop pending   : %d" % (loop_tx - loop_rx))
        print("PRBS RX bytes  : %d" % prbs_rx)
        print("PRBS RX B/sec  : %.2f" % prbs_rx_rate)
        print("PRBS errors    : %d" % prbs_errors)
        print("Target RX errors: %d" % target_rx_errors)

        if stall_error is not None:
            print("Stall error    : %s" % stall_error)
        if io_error is not None:
            print("I/O error      : %s" % io_error)

        passed = (io_error is None and stall_error is None and
                  loop_tx > 0 and loop_rx == loop_tx and loop_errors == 0 and
                  prbs_rx > 0 and prbs_errors == 0 and target_rx_errors == 0)
        print("Result         : %s" % ("PASS" if passed else "FAIL"))
        return 0 if passed else 1
    finally:
        trace.event("closing both ports")
        loop_comm.close()
        prbs_comm.close()


if __name__ == "__main__":
    sys.exit(main())
