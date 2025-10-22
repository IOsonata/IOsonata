""" -------------------------------------------------------------------------
@file   uartprbs_rx.py

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
import logging
import serial
import time

def prbs8(curval):
    newbit = (((curval >> 6) ^ (curval >> 5)) & 1)
    return ((curval << 1) | newbit) & 0x7f

def main():

    comm = serial.Serial(port="/dev/cu.usbmodem40303180003282", baudrate=1000000, rtscts=True)
    comm.flushInput()

    curval = 0
    val = prbs8(0xff)
    byteCount = 0
    dropcnt = 0
    deltatime = 0
    drop = False
    while True:
        try:
            comm.write(val)
            startTime = time.time()
            endTime = time.time()
            deltatime += endTime - startTime
            val = prbs8(val)
            byteCount += 1

            if deltatime > 0:
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
