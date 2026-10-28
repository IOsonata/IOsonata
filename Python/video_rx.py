""" -------------------------------------------------------------------------
@file   video_rx.py

@brief  Receive the frames of exemples/video/video_capture_demo.cpp

Reads the console UART of the board. A frame is the 4 characters VFRM, the
frame length in bytes 32 bits little endian, then the frame data. Each frame
is written to a JPEG file, cut after the JPEG end of image marker FF D9. The
text between the frames is printed as received.

Printed for each frame:
    frame number, frame length, JPEG length, frames per second since start
    and a warning when the JPEG start (FF D8) or end (FF D9) marker is
    missing. Such a frame is still written, with the .bin extension.

@param  --port  : Serial port to use
        --baud  : Baud rate (default 1MBaud)
        --out   : Directory for the frame files (default frames)
        --count : Number of frames to receive, 0 for no limit (default 0)

@author Hoang Nguyen Hoan
@date   Oct. 28, 2026

@license

MIT License

Copyright (c) 2026, I-SYST inc., all rights reserved

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

----------------------------------------------------------------------------"""
import argparse
import os
import struct
import sys
import time

import serial

FRAME_TAG = b'VFRM'
FRAME_LEN_MAX = 8 * 1024 * 1024     # Larger is a corrupted header
JPEG_SOI = b'\xff\xd8'
JPEG_EOI = b'\xff\xd9'


def read_exact(port, n):
    """Read n bytes, None if the port stops sending"""
    data = bytearray()
    while len(data) < n:
        d = port.read(n - len(data))
        if not d:
            return None
        data += d
    return bytes(data)


def print_text(data):
    if data:
        sys.stdout.write(data.decode('ascii', errors='replace'))
        sys.stdout.flush()


def save_frame(outdir, num, frame):
    """Write the frame, return the JPEG length and a warning text"""
    warn = ''
    start = frame.find(JPEG_SOI)
    end = frame.rfind(JPEG_EOI)

    if start != 0:
        warn += ' no JPEG start'
    if end < 0:
        warn += ' no JPEG end'

    if warn:
        name = os.path.join(outdir, 'frame%05d.bin' % num)
        jpeg = frame
    else:
        name = os.path.join(outdir, 'frame%05d.jpg' % num)
        jpeg = frame[:end + 2]

    with open(name, 'wb') as f:
        f.write(jpeg)

    return len(jpeg), warn


def main():
    parser = argparse.ArgumentParser(description='Receive the video_capture_demo frames')
    parser.add_argument('--port', required=True, help='Serial port')
    parser.add_argument('--baud', type=int, default=1000000, help='Baud rate')
    parser.add_argument('--out', default='frames', help='Directory for the frame files')
    parser.add_argument('--count', type=int, default=0, help='Number of frames, 0 no limit')
    args = parser.parse_args()

    os.makedirs(args.out, exist_ok=True)

    port = serial.Serial(args.port, args.baud, timeout=2)
    print('Receiving on %s at %d baud, frames in %s' % (args.port, args.baud, args.out))

    buf = bytearray()
    num = 0
    t0 = None

    try:
        while args.count == 0 or num < args.count:
            d = port.read(port.in_waiting or 1)
            if not d:
                continue
            buf += d

            idx = buf.find(FRAME_TAG)
            if idx < 0:
                # Keep a possible partial tag at the end
                keep = len(FRAME_TAG) - 1
                print_text(bytes(buf[:-keep]))
                del buf[:-keep]
                continue

            print_text(bytes(buf[:idx]))
            del buf[:idx + len(FRAME_TAG)]

            hdr = bytes(buf[:4])
            del buf[:4]
            if len(hdr) < 4:
                rest = read_exact(port, 4 - len(hdr))
                if rest is None:
                    print('\nTimeout in frame header')
                    continue
                hdr += rest

            length = struct.unpack('<I', hdr)[0]
            if length == 0 or length > FRAME_LEN_MAX:
                print('\nBad frame length %d, resync' % length)
                continue

            frame = bytes(buf[:length])
            del buf[:length]
            if len(frame) < length:
                rest = read_exact(port, length - len(frame))
                if rest is None:
                    print('\nTimeout in frame data, %d of %d bytes' % (len(frame), length))
                    continue
                frame += rest

            if t0 is None:
                t0 = time.time()
            num += 1
            jlen, warn = save_frame(args.out, num, frame)
            dt = time.time() - t0
            fps = (num - 1) / dt if dt > 0 else 0.0
            print('\nframe %d: %d bytes, JPEG %d bytes, %.2f fps%s' % (num, length, jlen, fps, warn))

    except KeyboardInterrupt:
        pass
    finally:
        port.close()

    print('\n%d frames received' % num)


if __name__ == '__main__':
    main()
