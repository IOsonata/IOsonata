""" -------------------------------------------------------------------------
@file   usb_cdc_loopback.py

@brief  USB CDC sustained loopback integrity and throughput test

The target must run exemples/usb/usb_cdc_loopback.cpp. The test continuously
writes a PRBS byte stream while reading the echoed stream on a second thread
of execution. Concurrent traffic keeps both CDC OUT and IN active and allows
USB backpressure to propagate naturally without losing host-side accounting.

@param  --port       CDC serial port
        --baud       CDC line coding value (default 1000000)
        --duration   Test duration in seconds (default 30)
        --block      Host write block size in bytes (default 4096)
        --read-size  Host read block size in bytes (default 4096)

@author Hoang Nguyen Hoan
@date   Sep. 1, 2026

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


CDC_BANNER = b"IOsonata USB CDC Loopback"


def prbs8(curval):
    newbit = ((curval >> 6) ^ (curval >> 5)) & 1
    return ((curval << 1) | newbit) & 0x7f


def make_prbs_block(state, length):
    data = bytearray(length)

    for i in range(length):
        state = prbs8(state)
        data[i] = state

    return bytes(data), state


class TestStats:
    def __init__(self):
        self.lock = threading.Lock()
        self.tx_bytes = 0
        self.rx_bytes = 0
        self.errors = 0
        self.write_error = None

    def add_tx(self, count):
        with self.lock:
            self.tx_bytes += count

    def add_rx(self, count, errors):
        with self.lock:
            self.rx_bytes += count
            self.errors += errors

    def set_write_error(self, error):
        with self.lock:
            self.write_error = error

    def snapshot(self):
        with self.lock:
            return (self.tx_bytes, self.rx_bytes,
                    self.errors, self.write_error)


def parse_args():
    parser = argparse.ArgumentParser(
        description="IOsonata USB CDC sustained loopback test")
    parser.add_argument(
        "--port", required=True,
        help="CDC port, e.g. /dev/cu.usbmodemXXXX, /dev/ttyACM0, COM5")
    parser.add_argument(
        "--baud", type=int, default=1_000_000,
        help="CDC line coding value (default: 1000000)")
    parser.add_argument(
        "--duration", type=float, default=30.0,
        help="test duration in seconds (default: 30)")
    parser.add_argument(
        "--block", type=int, default=4096,
        help="write block size in bytes (default: 4096)")
    parser.add_argument(
        "--read-size", type=int, default=4096,
        help="read block size in bytes (default: 4096)")
    parser.add_argument(
        "--report", type=float, default=1.0,
        help="throughput report interval in seconds (default: 1)")
    parser.add_argument(
        "--drain-timeout", type=float, default=3.0,
        help="time to drain the final echo in seconds (default: 3)")
    return parser.parse_args()


def prepare_port(comm):
    try:
        comm.dtr = False
        time.sleep(0.05)
    except (OSError, serial.SerialException):
        pass

    comm.reset_input_buffer()
    comm.reset_output_buffer()

    try:
        comm.dtr = True
    except (OSError, serial.SerialException):
        pass

    deadline = time.monotonic() + 1.0
    banner = bytearray()

    while time.monotonic() < deadline:
        waiting = comm.in_waiting
        data = comm.read(waiting if waiting > 0 else 1)

        if data:
            banner.extend(data)
            if CDC_BANNER in banner:
                break

    # Do not let the firmware open message become part of the PRBS stream.
    time.sleep(0.05)
    comm.reset_input_buffer()

    return CDC_BANNER in banner


def writer(comm, stop_event, stats, block_size):
    state = 0xff
    pending = b""

    try:
        while not stop_event.is_set():
            if not pending:
                pending, state = make_prbs_block(state, block_size)

            # write_timeout=0 makes this a non-blocking host write. pySerial
            # returns the exact number of bytes accepted by the OS, including
            # a short write or zero when USB OUT is backpressured.
            count = comm.write(pending)

            if count > 0:
                stats.add_tx(count)
                pending = pending[count:]
            else:
                # Avoid spinning at 100% CPU while the device is applying
                # legitimate USB OUT backpressure.
                time.sleep(0)
    except (OSError, serial.SerialException) as error:
        stats.set_write_error(str(error))


def check_data(data, expected):
    errors = 0

    for value in data:
        if value != expected:
            errors += 1

        # Resynchronize after a missing/corrupt byte so one fault does not
        # make every following byte look bad.
        expected = prbs8(value)

    return expected, errors


def read_and_check(comm, stats, expected, read_size):
    data = comm.read(read_size)

    if data:
        expected, errors = check_data(data, expected)
        stats.add_rx(len(data), errors)

    return expected


def print_report(stats, now, previous_time, previous_rx):
    tx_bytes, rx_bytes, errors, _ = stats.snapshot()
    elapsed = now - previous_time
    rate = (rx_bytes - previous_rx) / elapsed if elapsed > 0 else 0.0
    pending = tx_bytes - rx_bytes

    print("Bytes/sec : %.2f, errors %d, pending %d" %
          (rate, errors, pending))

    return now, rx_bytes


def main():
    args = parse_args()

    if (args.duration <= 0 or args.block <= 0 or args.read_size <= 0 or
            args.report <= 0 or args.drain_timeout <= 0):
        print("ERROR: duration, block, read-size, report and drain-timeout "
              "must be greater than zero", file=sys.stderr)
        return 2

    try:
        comm = serial.Serial(
            port=args.port,
            baudrate=args.baud,
            timeout=0.1,
            # Non-blocking writes are required here. They let the writer
            # observe short/zero writes under USB backpressure and keep exact
            # byte accounting without the partial-write ambiguity of
            # SerialTimeoutException.
            write_timeout=0,
            rtscts=False,
            dsrdtr=False)
    except (OSError, serial.SerialException) as error:
        print("ERROR: cannot open %s: %s" % (args.port, error),
              file=sys.stderr)
        return 2

    try:
        banner_seen = prepare_port(comm)

        if banner_seen:
            print("Connected to IOsonata USB CDC Loopback")
        else:
            print("WARNING: loopback banner not detected")

        stop_event = threading.Event()
        stats = TestStats()
        tx_thread = threading.Thread(
            target=writer,
            args=(comm, stop_event, stats, args.block),
            daemon=True)

        expected = prbs8(0xff)
        start_time = time.monotonic()
        end_time = start_time + args.duration
        report_time = start_time
        report_rx = 0

        tx_thread.start()

        try:
            while time.monotonic() < end_time:
                expected = read_and_check(
                    comm, stats, expected, args.read_size)

                now = time.monotonic()
                if now - report_time >= args.report:
                    report_time, report_rx = print_report(
                        stats, now, report_time, report_rx)
        except KeyboardInterrupt:
            print("KeyboardInterrupt. Stopping test.")

        # Non-blocking writes let the writer stop immediately without
        # cancelling an in-progress OS write.
        stop_event.set()
        active_end = time.monotonic()
        _, active_rx, _, _ = stats.snapshot()
        tx_thread.join(timeout=1.0)

        if tx_thread.is_alive():
            stats.set_write_error("writer did not stop")

        # tx_bytes is now final. Drain every byte accepted by the host serial
        # driver and verify the exact byte count.
        drain_deadline = time.monotonic() + args.drain_timeout

        while time.monotonic() < drain_deadline:
            tx_bytes, rx_bytes, _, _ = stats.snapshot()

            if rx_bytes >= tx_bytes:
                break

            expected = read_and_check(
                comm, stats, expected, args.read_size)

        tx_bytes, rx_bytes, errors, write_error = stats.snapshot()
        elapsed = active_end - start_time
        rate = active_rx / elapsed if elapsed > 0 else 0.0
        pending = tx_bytes - rx_bytes

        print()
        print("TX bytes       : %d" % tx_bytes)
        print("RX bytes       : %d" % rx_bytes)
        print("Loopback B/sec : %.2f" % rate)
        print("Errors         : %d" % errors)
        print("Pending        : %d" % pending)

        if write_error is not None:
            print("Write error    : %s" % write_error)

        if write_error is None and errors == 0 and pending == 0:
            print("Result         : PASS")
            return 0

        print("Result         : FAIL")
        return 1
    finally:
        comm.close()


if __name__ == "__main__":
    sys.exit(main())
