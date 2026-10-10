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
import time

import serial


def prbs8(curval):
    newbit = ((curval >> 6) ^ (curval >> 5)) & 1
    return ((curval << 1) | newbit) & 0x7f


def parse_args():
    parser = argparse.ArgumentParser(description="PRBS UART loopback test")
    parser.add_argument("--port", required=True, help="Serial transmit port")
    parser.add_argument("--rx-port", help="Separate receive port (default: --port)")
    parser.add_argument("--baud", type=int, default=1000000,
                        help="Baud rate (default: 1000000)")
    parser.add_argument("--timeout", type=float, default=1.0,
                        help="Receive timeout in seconds (default: 1)")
    parser.add_argument("--report-interval", type=float, default=1.0,
                        help="Statistics interval in seconds (default: 1)")
    return parser.parse_args()


def main():
    args = parse_args()
    if args.baud <= 0 or args.timeout <= 0 or args.report_interval <= 0:
        print("ERROR: baud, timeout, and report interval must be positive",
              file=sys.stderr)
        return 2

    tx = None
    rx = None
    try:
        tx = serial.Serial(port=args.port, baudrate=args.baud,
                           timeout=args.timeout, write_timeout=args.timeout,
                           rtscts=False)
        if args.rx_port and args.rx_port != args.port:
            rx = serial.Serial(port=args.rx_port, baudrate=args.baud,
                               timeout=args.timeout, rtscts=False)
        else:
            rx = tx

        rx.reset_input_buffer()
        value = prbs8(0xff)
        count = 0
        errors = 0
        start = time.perf_counter()
        last_report = start

        while True:
            tx.write(bytes((value,)))
            data = rx.read(1)
            if not data or data[0] != value:
                errors += 1
            value = prbs8(value)
            count += 1

            now = time.perf_counter()
            if now - last_report >= args.report_interval:
                elapsed = now - start
                print("Bytes/sec : %.2f, errors %d" %
                      (count / elapsed if elapsed > 0 else 0.0, errors))
                last_report = now

    except KeyboardInterrupt:
        print("KeyboardInterrupt. Exiting.")
    except (serial.SerialException, OSError) as exc:
        print("ERROR: %s" % exc, file=sys.stderr)
        return 2
    finally:
        if rx is not None and rx is not tx:
            rx.close()
        if tx is not None:
            tx.close()
    return 0


if __name__ == "__main__":
    sys.exit(main())
