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
    return parser.parse_args()


def hold_port_closed(comm):
    try:
        comm.dtr = False
        time.sleep(0.05)
    except (OSError, serial.SerialException):
        pass

    comm.reset_input_buffer()
    comm.reset_output_buffer()


def open_loopback_port(comm):
    try:
        comm.dtr = True
    except (OSError, serial.SerialException):
        pass

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
    time.sleep(0.05)
    comm.reset_input_buffer()
    return LOOPBACK_BANNER in received


def open_prbs_port(comm):
    try:
        comm.dtr = True
    except (OSError, serial.SerialException):
        pass


def loopback_writer(comm, stop_event, abort_event, stats, block_size):
    state = 0xff
    pending = b""

    while not stop_event.is_set() and not abort_event.is_set():
        if not pending:
            pending, state = make_prbs_block(state, block_size)

        try:
            count = comm.write(pending)
        except (OSError, serial.SerialException,
                serial.SerialTimeoutException) as error:
            stats.set_io_error("loopback write", error)
            abort_event.set()
            return

        if count > 0:
            stats.add_loop_tx(count)
            pending = pending[count:]


def loopback_reader(comm, stop_event, abort_event, stats, read_size):
    expected = prbs8(0xff)

    while not stop_event.is_set() and not abort_event.is_set():
        try:
            data = comm.read(read_size)
        except (OSError, serial.SerialException) as error:
            stats.set_io_error("loopback read", error)
            abort_event.set()
            return

        if data:
            expected, errors = check_stream(data, expected)
            stats.add_loop_rx(len(data), errors)


def prbs_reader(comm, stop_event, abort_event, stats, read_size):
    # Synchronize to the first byte observed. The device may have transmitted
    # before the host reader thread was scheduled.
    expected = None

    while not stop_event.is_set() and not abort_event.is_set():
        try:
            data = comm.read(read_size)
        except (OSError, serial.SerialException) as error:
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

    if (args.loop_port == args.prbs_port or args.duration <= 0 or
            args.block <= 0 or args.read_size <= 0 or args.report <= 0 or
            args.drain_timeout < 0 or args.stall_timeout <= 0):
        print("ERROR: ports must differ and numeric arguments must be valid",
              file=sys.stderr)
        return 2

    loop_comm = None
    prbs_comm = None

    try:
        loop_comm = open_serial(args.loop_port, args.baud)
        prbs_comm = open_serial(args.prbs_port, args.baud)
    except (OSError, serial.SerialException) as error:
        print("ERROR: cannot open both CDC ports: %s" % error,
              file=sys.stderr)
        if loop_comm is not None:
            loop_comm.close()
        if prbs_comm is not None:
            prbs_comm.close()
        return 2

    try:
        hold_port_closed(loop_comm)
        hold_port_closed(prbs_comm)

        if open_loopback_port(loop_comm):
            print("Connected to IOsonata USB Dual CDC Loopback")
        else:
            print("WARNING: loopback banner not detected")

        open_prbs_port(prbs_comm)

        traffic_stop = threading.Event()
        reader_stop = threading.Event()
        abort_event = threading.Event()
        stats = TestStats()
        threads = [
            threading.Thread(
                target=loopback_writer,
                args=(loop_comm, traffic_stop, abort_event, stats, args.block),
                daemon=True),
            threading.Thread(
                target=loopback_reader,
                args=(loop_comm, reader_stop, abort_event, stats,
                      args.read_size),
                daemon=True),
            threading.Thread(
                target=prbs_reader,
                args=(prbs_comm, reader_stop, abort_event, stats,
                      args.read_size),
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

                    report_time = now
                    report_loop_tx = loop_tx
                    report_loop_rx = loop_rx
                    report_prbs_rx = prbs_rx
        except KeyboardInterrupt:
            print("KeyboardInterrupt. Stopping test.")

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
        loop_comm.close()
        prbs_comm.close()


if __name__ == "__main__":
    sys.exit(main())
