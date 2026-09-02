""" -------------------------------------------------------------------------
@file   usb_cdc_packet_boundary.py

@brief  USB CDC full-speed packet-boundary loopback regression test

The target must run exemples/usb/usb_cdc_loopback.cpp. The test writes a
sequence of payload sizes around the 64-byte full-speed bulk maximum packet
size and verifies that each payload is echoed exactly.

@param  --port       CDC serial port
        --baud       CDC line coding value (default 1000000)
        --repeat     Number of complete boundary-size cycles (default 100)
        --timeout    Per-case echo timeout in seconds (default 2)

@author Hoang Nguyen Hoan
@date   Sep. 2, 2026

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
import time

import serial


CDC_BANNER = b"IOsonata USB CDC Loopback"
BOUNDARY_SIZES = (1, 7, 63, 64, 65, 127, 128, 129)


def prbs8(curval):
    newbit = ((curval >> 6) ^ (curval >> 5)) & 1
    return ((curval << 1) | newbit) & 0x7f


def make_prbs_block(state, length):
    data = bytearray(length)

    for i in range(length):
        state = prbs8(state)
        data[i] = state

    return bytes(data), state


def parse_args():
    parser = argparse.ArgumentParser(
        description="IOsonata USB CDC packet-boundary loopback test")
    parser.add_argument(
        "--port", required=True,
        help="CDC port, e.g. /dev/cu.usbmodemXXXX, /dev/ttyACM0, COM5")
    parser.add_argument(
        "--baud", type=int, default=1_000_000,
        help="CDC line coding value (default: 1000000)")
    parser.add_argument(
        "--repeat", type=int, default=100,
        help="complete boundary-size cycles (default: 100)")
    parser.add_argument(
        "--timeout", type=float, default=2.0,
        help="per-case echo timeout in seconds (default: 2)")
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

    time.sleep(0.05)
    comm.reset_input_buffer()
    return CDC_BANNER in banner


def write_all(comm, data):
    offset = 0

    while offset < len(data):
        count = comm.write(data[offset:])
        if count <= 0:
            raise serial.SerialTimeoutException("zero-length serial write")
        offset += count

    comm.flush()


def read_exact(comm, length, timeout):
    data = bytearray()
    deadline = time.monotonic() + timeout

    while len(data) < length and time.monotonic() < deadline:
        chunk = comm.read(length - len(data))
        if chunk:
            data.extend(chunk)

    return bytes(data)


def first_mismatch(expected, actual):
    count = min(len(expected), len(actual))

    for i in range(count):
        if expected[i] != actual[i]:
            return i

    return count if len(expected) != len(actual) else -1


def run_boundary_test(comm, repeat, timeout):
    state = 0xff
    total_bytes = 0
    total_cases = 0

    for cycle in range(repeat):
        for size in BOUNDARY_SIZES:
            payload, state = make_prbs_block(state, size)
            write_all(comm, payload)
            echoed = read_exact(comm, size, timeout)
            total_cases += 1

            if echoed != payload:
                mismatch = first_mismatch(payload, echoed)
                print("FAIL cycle %d, size %d, received %d bytes" %
                      (cycle + 1, size, len(echoed)))
                if mismatch >= 0 and mismatch < len(echoed):
                    print("First mismatch : offset %d, expected 0x%02x, got 0x%02x" %
                          (mismatch, payload[mismatch], echoed[mismatch]))
                elif len(echoed) < len(payload):
                    print("First mismatch : missing byte at offset %d" % mismatch)
                return False, total_cases, total_bytes

            total_bytes += size

    # Catch a duplicated or delayed final packet that would otherwise have no
    # following payload to expose the stream displacement.
    time.sleep(0.05)
    if comm.in_waiting != 0:
        extra = comm.read(comm.in_waiting)
        print("FAIL extra echo data: %d bytes" % len(extra))
        return False, total_cases, total_bytes

    return True, total_cases, total_bytes


def main():
    args = parse_args()

    if args.repeat <= 0 or args.timeout <= 0:
        print("ERROR: repeat and timeout must be greater than zero",
              file=sys.stderr)
        return 2

    try:
        comm = serial.Serial(
            port=args.port,
            baudrate=args.baud,
            timeout=0.05,
            write_timeout=args.timeout,
            rtscts=False,
            dsrdtr=False)
    except (OSError, serial.SerialException) as error:
        print("ERROR: cannot open %s: %s" % (args.port, error),
              file=sys.stderr)
        return 2

    try:
        if prepare_port(comm):
            print("Connected to IOsonata USB CDC Loopback")
        else:
            print("WARNING: loopback banner not detected")

        print("Packet sizes    : %s" %
              ", ".join(str(size) for size in BOUNDARY_SIZES))
        print("Cycles          : %d" % args.repeat)

        try:
            passed, cases, byte_count = run_boundary_test(
                comm, args.repeat, args.timeout)
        except (OSError, serial.SerialException,
                serial.SerialTimeoutException) as error:
            print("Serial error    : %s" % error)
            print("Result          : FAIL")
            return 1

        print("Cases           : %d" % cases)
        print("TX/RX bytes     : %d" % byte_count)
        print("Result          : %s" % ("PASS" if passed else "FAIL"))
        return 0 if passed else 1
    except KeyboardInterrupt:
        print("KeyboardInterrupt. Exiting.")
        return 130
    finally:
        comm.close()


if __name__ == "__main__":
    sys.exit(main())
