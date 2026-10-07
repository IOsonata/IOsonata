""" -------------------------------------------------------------------------
@file   usb_cdc_loopback.py

@brief  USB CDC sustained loopback integrity and throughput test

The target must run exemples/usb/usb_cdc_loopback.cpp. The test continuously
writes a PRBS byte stream while reading the echoed stream on a second thread
of execution. Concurrent traffic keeps both CDC OUT and IN active and avoids
host-side deadlock when either USB direction applies backpressure.

A write that cannot go out right away is not an error: the device applies
backpressure by NAKing OUT while its own buffers are full. Every write returns
the exact byte count the port accepted, so the TX count and the PRBS stream
stay in step. The test fails as stalled when neither direction moves for the
stall timeout.

Everything received from the moment the port opens is kept. The banner the
firmware sends when the port opens is matched there, and the input has to be
quiet before the PRBS stream starts, so a late banner is not counted as data
errors.

@param  --port           CDC serial port
        --baud           CDC line coding value (default 1000000)
        --duration       Test duration in seconds (default 30)
        --block          Host write block size in bytes (default 4096)
        --read-size      Host read block size in bytes (default 4096)
        --report         Throughput report interval in seconds (default 1)
        --drain-timeout  Time to wait for the final echo in seconds (default 3)
        --stall-timeout  Time without TX or RX progress that fails the test
                         in seconds (default 3)

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
import errno
import os
import select
import sys
import threading
import time

import serial


CDC_BANNER = b"IOsonata USB CDC Loopback"
CDC_BANNER_LINE = CDC_BANNER + b"\r\n"

# Time to look for the banner sent when the port opens, then after a DTR
# toggle when it did not show up.
BANNER_OPEN_WAIT = 0.5
BANNER_DTR_WAIT = 1.0

# The input must stay quiet this long before the PRBS stream starts.
QUIET_TIME = 0.2
QUIET_LIMIT = 1.0

# Longest a single write waits for the port to take data.
WRITE_WAIT = 0.1


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
        self.start_time = 0.0
        self.first_rx_time = None
        self.last_tx_time = 0.0
        self.last_rx_time = 0.0
        self.max_tx_wait = 0.0
        self.max_rx_gap = 0.0

    def start(self, now):
        with self.lock:
            self.start_time = now
            self.last_tx_time = now
            self.last_rx_time = now

    def add_tx(self, count):
        now = time.monotonic()

        with self.lock:
            self.tx_bytes += count
            self.max_tx_wait = max(self.max_tx_wait, now - self.last_tx_time)
            self.last_tx_time = now

    def add_rx(self, count, errors):
        now = time.monotonic()

        with self.lock:
            self.rx_bytes += count
            self.errors += errors
            if self.first_rx_time is None:
                self.first_rx_time = now
            self.max_rx_gap = max(self.max_rx_gap, now - self.last_rx_time)
            self.last_rx_time = now

    def set_write_error(self, error):
        with self.lock:
            self.write_error = error

    def snapshot(self):
        with self.lock:
            return (self.tx_bytes, self.rx_bytes,
                    self.errors, self.write_error)

    def last_progress(self):
        with self.lock:
            return max(self.last_tx_time, self.last_rx_time)


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
        help="time to wait for the final echo in seconds (default: 3)")
    parser.add_argument(
        "--stall-timeout", type=float, default=3.0,
        help="time without TX or RX progress that fails the test in "
             "seconds (default: 3)")
    return parser.parse_args()


def read_available(comm, limit):
    # Take what is already there, or wait up to the port timeout for a byte.
    waiting = comm.in_waiting
    return comm.read(min(limit, waiting) if waiting > 0 else 1)


def read_until(comm, buf, duration, marker):
    deadline = time.monotonic() + duration

    while time.monotonic() < deadline:
        data = read_available(comm, 4096)
        if data:
            buf.extend(data)
            if marker in buf:
                return True

    return False


def wait_quiet(comm, buf, quiet, limit):
    deadline = time.monotonic() + limit
    last = time.monotonic()

    while time.monotonic() < deadline:
        data = read_available(comm, 4096)
        now = time.monotonic()

        if data:
            buf.extend(data)
            last = now
        elif now - last >= quiet:
            return True

    return False


def prepare_port(comm):
    """
    Wait for the banner and for the input to go quiet.

    Returns (how the banner was seen or None, bytes received before the PRBS
    stream, True when the input went quiet).
    """
    comm.reset_output_buffer()
    early = bytearray()

    # pyserial raises DTR when it opens the port, which is when the firmware
    # sends its banner. Keep what arrives from then on.
    banner = None
    if read_until(comm, early, BANNER_OPEN_WAIT, CDC_BANNER_LINE):
        banner = "at open"
    else:
        # Drop and raise DTR so the firmware sees the port open again.
        # Bytes already received are kept.
        try:
            comm.dtr = False
            time.sleep(0.05)
            comm.dtr = True
        except (OSError, serial.SerialException):
            pass

        if read_until(comm, early, BANNER_DTR_WAIT, CDC_BANNER_LINE):
            banner = "after DTR toggle"

    quiet = wait_quiet(comm, early, QUIET_TIME, QUIET_LIMIT)

    return banner, bytes(early), quiet


def port_write(comm, data):
    """
    Write what the port takes within WRITE_WAIT and return the exact count.

    pyserial raises a write timeout on a partial write and drops the count of
    what already went out, which would put the TX count and the PRBS stream
    out of step. On POSIX the port descriptor is non-blocking, so write to it
    directly. On Windows a blocking pyserial write returns the exact count,
    including after cancel_write.
    """
    if os.name == "nt":
        return comm.write(data)

    try:
        _, ready, _ = select.select([], [comm.fd], [], WRITE_WAIT)
        if not ready:
            return 0
        return os.write(comm.fd, data)
    except InterruptedError:
        return 0
    except OSError as error:
        if error.errno in (errno.EAGAIN, errno.EWOULDBLOCK):
            return 0
        raise serial.SerialException("write failed: %s" % error)


def writer(comm, stop_event, stats, block_size):
    state = 0xff
    pending = b""

    while not stop_event.is_set():
        if not pending:
            pending, state = make_prbs_block(state, block_size)

        try:
            count = port_write(comm, pending)
        except (OSError, serial.SerialException) as error:
            stats.set_write_error(str(error))
            stop_event.set()
            return

        if count > 0:
            stats.add_tx(count)
            pending = pending[count:]


def check_data(data, expected):
    errors = 0

    for value in data:
        if value != expected:
            errors += 1

        # Resynchronize after a missing/corrupt byte so one fault does not
        # make every following byte look bad.
        expected = prbs8(value)

    return expected, errors


def print_report(stats, now, previous_time, previous_tx, previous_rx):
    tx_bytes, rx_bytes, errors, _ = stats.snapshot()
    elapsed = now - previous_time
    tx_rate = (tx_bytes - previous_tx) / elapsed if elapsed > 0 else 0.0
    rx_rate = (rx_bytes - previous_rx) / elapsed if elapsed > 0 else 0.0
    pending = tx_bytes - rx_bytes

    print("Tx B/s : %.2f, Rx B/s : %.2f, errors %d, pending %d "
          "(tx %d, rx %d)" %
          (tx_rate, rx_rate, errors, pending, tx_bytes, rx_bytes))

    return now, tx_bytes, rx_bytes


def describe_early(early):
    """Bytes received before the PRBS stream, other than the banner line."""
    rest = early.replace(b"\r\n" + CDC_BANNER_LINE, b"")
    rest = rest.replace(CDC_BANNER_LINE, b"")

    return rest


def main():
    args = parse_args()

    if args.duration <= 0 or args.block <= 0 or args.read_size <= 0 or \
            args.stall_timeout <= 0:
        print("ERROR: duration, block, read-size and stall-timeout must be "
              "greater than zero", file=sys.stderr)
        return 2

    try:
        # Writes go through port_write; write_timeout None keeps the Windows
        # path blocking so it returns exact counts.
        comm = serial.Serial(
            port=args.port,
            baudrate=args.baud,
            timeout=0.1,
            write_timeout=None,
            rtscts=False,
            dsrdtr=False)
    except (OSError, serial.SerialException) as error:
        print("ERROR: cannot open %s: %s" % (args.port, error),
              file=sys.stderr)
        return 2

    try:
        banner, early, quiet = prepare_port(comm)
        extra = describe_early(early)

        if banner is not None:
            print("Connected to IOsonata USB CDC Loopback (banner %s)" % banner)
        else:
            print("WARNING: loopback banner not detected")
        if extra:
            print("WARNING: %d other bytes before the test: %s" %
                  (len(extra), extra[:32].hex(" ")))
        if not quiet:
            print("WARNING: input did not go quiet before the test")

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
        report_tx = 0
        report_rx = 0
        stall = None

        stats.start(start_time)
        tx_thread.start()

        try:
            while time.monotonic() < end_time and not stop_event.is_set():
                data = read_available(comm, args.read_size)

                if data:
                    expected, errors = check_data(data, expected)
                    stats.add_rx(len(data), errors)

                now = time.monotonic()
                if now - report_time >= args.report:
                    report_time, report_tx, report_rx = print_report(
                        stats, now, report_time, report_tx, report_rx)

                idle = now - stats.last_progress()
                if idle >= args.stall_timeout:
                    stall = "no TX or RX progress for %.1f s" % idle
                    break
        except KeyboardInterrupt:
            print("KeyboardInterrupt. Stopping test.")

        stop_event.set()
        if os.name == "nt":
            comm.cancel_write()
        tx_thread.join(timeout=1.5)

        if tx_thread.is_alive():
            stats.set_write_error("writer did not stop")

        active_end = time.monotonic()
        active_tx, active_rx, _, _ = stats.snapshot()

        # The writer has stopped. Drain every byte that was accepted by the
        # host serial driver so the final result also verifies byte counts.
        drain_deadline = time.monotonic() + args.drain_timeout

        while time.monotonic() < drain_deadline:
            tx_bytes, rx_bytes, _, _ = stats.snapshot()

            if rx_bytes >= tx_bytes:
                break

            data = read_available(comm, args.read_size)
            if data:
                expected, errors = check_data(data, expected)
                stats.add_rx(len(data), errors)

        tx_bytes, rx_bytes, errors, write_error = stats.snapshot()
        elapsed = active_end - start_time
        tx_rate = active_tx / elapsed if elapsed > 0 else 0.0
        rx_rate = active_rx / elapsed if elapsed > 0 else 0.0
        pending = tx_bytes - rx_bytes

        print()
        print("Banner         : %s" % (banner if banner else "not seen"))
        print("Bytes before TX: %d" % len(early))
        print("TX bytes       : %d" % tx_bytes)
        print("RX bytes       : %d" % rx_bytes)
        print("TX B/sec       : %.2f" % tx_rate)
        print("RX B/sec       : %.2f" % rx_rate)
        if stats.first_rx_time is None:
            print("First RX       : never")
        else:
            print("First RX       : %.3f s after start" %
                  (stats.first_rx_time - start_time))
        # Include a wait still running when the test stopped.
        print("Longest TX wait: %.3f s" %
              max(stats.max_tx_wait, active_end - stats.last_tx_time))
        print("Longest RX gap : %.3f s" %
              max(stats.max_rx_gap, active_end - stats.last_rx_time))
        print("Errors         : %d" % errors)
        print("Pending        : %d" % pending)

        if write_error is not None:
            print("Write error    : %s" % write_error)
        if stall is not None:
            print("Stalled        : %s" % stall)

        if write_error is None and stall is None and errors == 0 and \
                pending == 0:
            print("Result         : PASS")
            return 0

        if rx_bytes == 0 and tx_bytes > 0:
            print("Note           : the device took %d bytes and sent none "
                  "back; IN data is not reaching the host" % tx_bytes)
        elif stall is not None:
            print("Note           : traffic stopped after %d bytes out, "
                  "%d bytes back" % (tx_bytes, rx_bytes))

        print("Result         : FAIL")
        return 1
    finally:
        comm.close()


if __name__ == "__main__":
    sys.exit(main())
