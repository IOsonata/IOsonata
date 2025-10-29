""" -------------------------------------------------------------------------
@file   uartprbs_rx.py

@brief  PRBS receive test


@param  --port : Serial to use
        --baud : Bbadrate (default 1MBaud) 

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
import logging
import time
import serial


SLIP_END = 0xC0
SLIP_ESC = 0xDB
SLIP_ESC_END = 0xDC
SLIP_ESC_ESC = 0xDD


def prbs8(curval):
    newbit = (((curval >> 6) ^ (curval >> 5)) & 1)
    return ((curval << 1) | newbit) & 0x7f

def parse_args() -> argparse.Namespace:
    p = argparse.ArgumentParser(description="PRBS UART receiver (raw or SLIP-framed) with CSV/plot and STAT decoding")
    p.add_argument("--port", required=True, help="Serial port (e.g., /dev/ttyUSB0, /dev/cu.usbserial-XXX, COM5)")
    p.add_argument("--baud", type=int, default=1_000_000, help="Baud rate (default: 1000000)")
    return p.parse_args()

def main():
    args = parse_args()

    try:
        comm = serial.Serial(port=args.port, baudrate=args.baud)
    except Exception as e:
        print(f"ERROR: cannot open {args.port} at {args.baud} baud: {e}", file=sys.stderr)
        return 2

#    comm = serial.Serial(port="/dev/cu.usbmodem142302", baudrate=1000000, rtscts=True)
    comm.flushInput()

    curval = 0xff
    byteCount = 0
    dropcnt = 0
    deltatime = 0
    drop = False
    startTime = time.time()
    d = comm.read(1)
    curval = int.from_bytes(d, byteorder = 'little')
    val = prbs8(curval)
    while True:
        try:
            startTime = time.time()
            d = comm.read(1)
            endTime = time.time()
            deltatime += endTime - startTime
            curval = int.from_bytes(d, byteorder = 'little')
            if curval != val:
                dropcnt += 1
            val = prbs8(curval)
            byteCount += 1

            bytesPerSec = byteCount / deltatime #(endTime - startTime)

            #print("Bytes : {0}".format(bytes))
            #if drop:
            #    print("Dropped.... Bytes/sec : {0}".format(bytesPerSec))
            #else:
            if (byteCount & 0xff) == 0:
                print("Bytes/sec : %.2f, drop %d " %(bytesPerSec, dropcnt))
        except KeyboardInterrupt:
            print("KeyboardInterrupt. Exiting.")
            break

    comm.close()


if __name__ == '__main__':
    main()
