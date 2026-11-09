""" -------------------------------------------------------------------------
@file   uartprbs_tx.py

@brief  PRBS transmit test

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


def prbs8(curval):
    return ((curval << 1) | (((curval >> 6) ^ (curval >> 5)) & 1)) & 0x7f


def payload(state, size):
    data = bytearray()
    for _ in range(size):
        state = prbs8(state)
        data.append(state)
    return state, bytes(data)


def slip_encode(data):
    return b"\xc0" + data.replace(b"\xdb", b"\xdb\xdd").replace(b"\xc0", b"\xdb\xdc") + b"\xc0"


def main():
    import serial

    parser = argparse.ArgumentParser(description="Send IOsonata PRBS bytes and display receiver diagnostics.")
    parser.add_argument("--port", required=True)
    parser.add_argument("--baud", type=int, default=1000000)
    parser.add_argument("--chunk", type=int, default=92)
    parser.add_argument("--slip", action="store_true", help="Send SLIP frames")
    parser.add_argument("--duration", type=float, default=0, help="Seconds; zero runs until interrupted")
    args = parser.parse_args()
    if args.chunk <= 0 or args.duration < 0 or args.baud <= 0:
        parser.error("chunk and baud must be positive; duration must be nonnegative")

    comm = serial.Serial(port=None, baudrate=args.baud, timeout=0, write_timeout=2,
                         rtscts=False, dsrdtr=False)
    comm.dtr = False
    comm.rts = False
    comm.port = args.port
    state = 0xff
    start = time.monotonic()
    try:
        comm.open()
        while not args.duration or time.monotonic() - start < args.duration:
            state, data = payload(state, args.chunk)
            wire = slip_encode(data) if args.slip else data
            sent = 0
            while sent < len(wire):
                count = comm.write(wire[sent:])
                if not count:
                    raise RuntimeError("UART write made no progress")
                sent += count
            reply = comm.read(comm.in_waiting)
            if reply:
                sys.stdout.write(reply.decode("ascii", errors="replace"))
                sys.stdout.flush()
    except KeyboardInterrupt:
        pass
    finally:
        comm.close()


if __name__ == "__main__":
    main()
