""" -------------------------------------------------------------------------
@file   uartprbs_rx.py

@brief  PRBS receive test

@param  --port : Serial port to use
        --baud : Baud rate (default 1MBaud)
        --read-size : Serial read block size (default 4096)
        --report-interval : Status report interval in seconds (default 1.0)

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
import time

import serial


def prbs8(curval):
    newbit = (((curval >> 6) ^ (curval >> 5)) & 1)
    return ((curval << 1) | newbit) & 0x7f


def parse_args() -> argparse.Namespace:
    p = argparse.ArgumentParser(description="PRBS serial receiver")
    p.add_argument("--port", required=True,
                   help="Serial port (e.g., /dev/ttyUSB0, /dev/cu.usbmodemXXX, COM5)")
    p.add_argument("--baud", type=int, default=1_000_000,
                   help="Baud rate (default: 1000000)")
    p.add_argument("--read-size", type=int, default=4096,
                   help="Bytes per serial read (default: 4096)")
    p.add_argument("--report-interval", type=float, default=1.0,
                   help="Status report interval in seconds (default: 1.0)")
    return p.parse_args()


def main():
    args = parse_args()

    if args.read_size <= 0:
        print("ERROR: --read-size must be greater than zero", file=sys.stderr)
        return 2

    if args.report_interval <= 0:
        print("ERROR: --report-interval must be greater than zero", file=sys.stderr)
        return 2

    try:
        comm = serial.Serial(port=args.port, baudrate=args.baud, timeout=0.1)
    except Exception as e:
        print(f"ERROR: cannot open {args.port} at {args.baud} baud: {e}", file=sys.stderr)
        return 2

    comm.reset_input_buffer()

    try:
        first = b""
        while not first:
            first = comm.read(1)

        expected = prbs8(first[0])
        byte_count = 0
        drop_count = 0
        start_time = time.perf_counter()
        last_report = start_time

        while True:
            data = comm.read(args.read_size)
            if not data:
                continue

            for curval in data:
                if curval != expected:
                    drop_count += 1
                expected = prbs8(curval)

            byte_count += len(data)
            now = time.perf_counter()

            if now - last_report >= args.report_interval:
                elapsed = now - start_time
                bytes_per_sec = byte_count / elapsed if elapsed > 0 else 0.0
                print("Bytes/sec : %.2f, drop %d " % (bytes_per_sec, drop_count))
                last_report = now

    except KeyboardInterrupt:
        print("KeyboardInterrupt. Exiting.")
    finally:
        comm.close()

    return 0


if __name__ == '__main__':
    sys.exit(main())
