""" -------------------------------------------------------------------------
@file   uartprbs_lb.py

@brief  PRBS UART loopback test with one or two serial ports

@author Hoang Nguyen Hoan
@date   July 27, 2019

@license

MIT License

Copyright (c) 2019 I-SYST inc. All rights reserved.

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


def prbs8(curval):
    newbit = ((curval >> 6) ^ (curval >> 5)) & 1
    return ((curval << 1) | newbit) & 0x7f


def parse_args():
    parser = argparse.ArgumentParser(description="Threaded PRBS UART loopback")
    parser.add_argument("--port", required=True, help="Transmit serial port")
    parser.add_argument("--rx-port", help="Separate receive port (default: --port)")
    parser.add_argument("--baud", type=int, default=1000000)
    parser.add_argument("--timeout", type=float, default=1.0,
                        help="No-receive timeout in seconds (default: 1)")
    parser.add_argument("--report-interval", type=float, default=1.0)
    parser.add_argument("--block-size", type=int, default=16,
                        help="Transmitted bytes per write (default: 16)")
    parser.add_argument("--window", type=int, default=128,
                        help="Maximum unacknowledged bytes (default: 128)")
    return parser.parse_args()


def main():
    args = parse_args()
    if (args.baud <= 0 or args.timeout <= 0 or args.report_interval <= 0 or
            args.block_size <= 0 or args.window < args.block_size):
        print("ERROR: invalid baud, timeout, interval, block size or window",
              file=sys.stderr)
        return 2

    tx = None
    rx = None
    worker = None
    stop = threading.Event()
    condition = threading.Condition()
    state = {"sent": 0, "received": 0, "errors": 0, "failure": None}

    # PRBS7 repeats every 127 bytes; generate the stream once for efficient
    # block transmission, then independently check every received byte.
    sequence = bytearray()
    value = prbs8(0xff)
    for _ in range(127):
        sequence.append(value)
        value = prbs8(value)
    pattern = bytes(sequence)

    def transmit():
        try:
            while not stop.is_set():
                with condition:
                    condition.wait_for(
                        lambda: stop.is_set() or
                        state["sent"] - state["received"] < args.window,
                        timeout=0.1)
                    if stop.is_set():
                        return
                    available = args.window - (state["sent"] - state["received"])
                    if available <= 0:
                        continue
                    offset = state["sent"]
                    length = min(args.block_size, available)
                block = bytes(pattern[(offset + i) % len(pattern)]
                              for i in range(length))
                written = tx.write(block)
                if written != length:
                    raise serial.SerialTimeoutException(
                        "Short UART write: %d of %d" % (written, length))
                with condition:
                    state["sent"] += written
                    condition.notify_all()
        except (serial.SerialException, OSError) as exc:
            with condition:
                state["failure"] = exc
                condition.notify_all()
            stop.set()

    try:
        tx = serial.Serial(port=args.port, baudrate=args.baud,
                           timeout=0.1, write_timeout=args.timeout,
                           rtscts=False)
        rx = (serial.Serial(port=args.rx_port, baudrate=args.baud,
                            timeout=0.1, rtscts=False)
              if args.rx_port and args.rx_port != args.port else tx)
        rx.reset_input_buffer()

        worker = threading.Thread(target=transmit, name="uart-prbs-tx",
                                  daemon=True)
        worker.start()
        start = time.perf_counter()
        last_report = start
        last_received_at = start
        prior_received = 0

        while not stop.is_set():
            # Read data already available without waiting for a full window.
            # The transmit window is also args.window, so requesting that
            # many bytes causes a timeout-driven send/receive feedback loop
            # whenever fewer bytes are outstanding.
            ready = rx.in_waiting
            data = rx.read(min(ready, args.window) if ready else 1)
            now = time.perf_counter()
            if data:
                with condition:
                    position = state["received"]
                    for byte in data:
                        if byte != pattern[position % len(pattern)]:
                            state["errors"] += 1
                        position += 1
                    state["received"] = position
                    condition.notify_all()
                last_received_at = now

            with condition:
                sent = state["sent"]
                received = state["received"]
                errors = state["errors"]
                failure = state["failure"]

            if failure is not None:
                raise failure
            if now - last_report >= args.report_interval:
                elapsed = now - last_report
                rate = (received - prior_received) / elapsed
                print("Rx B/s : %.2f, errors %d, pending %d" %
                      (rate, errors, max(0, sent - received)), flush=True)
                prior_received = received
                last_report = now
            if now - last_received_at >= args.timeout:
                raise serial.SerialTimeoutException(
                    "No UART echo for %.2f seconds (%d sent, %d received)" %
                    (args.timeout, sent, received))

    except KeyboardInterrupt:
        print("KeyboardInterrupt. Exiting.")
    except (serial.SerialException, OSError) as exc:
        print("ERROR: %s" % exc, file=sys.stderr)
        return 2
    finally:
        stop.set()
        with condition:
            condition.notify_all()
        if worker is not None:
            worker.join(timeout=args.timeout + 0.2)
        if rx is not None and rx is not tx:
            rx.close()
        if tx is not None:
            tx.close()
    return 0


if __name__ == "__main__":
    sys.exit(main())
