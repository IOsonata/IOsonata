""" -------------------------------------------------------------------------
@file   video_view.py

@brief  Show the frames of the IOsonata video examples on the PC

Receives the stream of exemples/video/video_usb_demo.cpp (USB CDC),
video_ble_demo.cpp (BLE) or video_capture_demo.cpp (UART) and shows the JPEG
frames in a window. A frame is the 4 characters VFRM, the frame length in
bytes 32 bits little endian, then the frame data. The text between the frames
is printed as received.

The window shows the frame rate, the data rate and the frame size. q or Esc
closes it.

Serial port, USB CDC or UART:
    python3 video_view.py --port /dev/tty.usbmodem1101
BLE, the board advertises as BleVideo:
    python3 video_view.py --ble
    python3 video_view.py --ble --address XX:XX:XX:XX:XX:XX

Requires numpy and opencv-python, pyserial for --port, bleak for --ble.

@param  --port       : Serial port, USB CDC or UART
        --baud       : UART baud rate (default 1MBaud), not used by USB CDC
        --ble        : Connect over BLE to the device of this name
                       (default BleVideo)
        --address    : BLE address or macOS identifier, skips the scan
        --scale      : Window scale factor (default 1, 2 for small frames)
        --save       : Directory to write the frames to as JPEG files
        --no-display : Print the frame rate only, no window

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
import asyncio
import collections
import os
import struct
import sys
import threading
import time

import numpy as np
import cv2

FRAME_TAG = b'VFRM'
FRAME_LEN_MAX = 8 * 1024 * 1024     # Larger is a corrupted header
JPEG_EOI = b'\xff\xd9'

# BlueIO UART service, the stream comes in the notifications
BLE_DEVICE_NAME = 'BleVideo'
BLE_SERVICE_UUID = '00000101-287c-11e4-ab74-0002a5d5c51b'
BLE_DATA_UUID = '00000102-287c-11e4-ab74-0002a5d5c51b'

WINDOW_NAME = 'IOsonata video'


class FrameParser:
    """Split the received stream in text and frames, in any part size"""

    def __init__(self, on_text, on_frame):
        self.on_text = on_text
        self.on_frame = on_frame
        self.buf = bytearray()
        self.need = None            # Frame length expected, None while looking for a tag

    def feed(self, data):
        self.buf += data
        while True:
            if self.need is None:
                idx = self.buf.find(FRAME_TAG)
                if idx < 0:
                    # Keep a possible partial tag at the end
                    keep = len(FRAME_TAG) - 1
                    if len(self.buf) > keep:
                        self.on_text(bytes(self.buf[:-keep]))
                        del self.buf[:-keep]
                    return
                if idx > 0:
                    self.on_text(bytes(self.buf[:idx]))
                    del self.buf[:idx]
                if len(self.buf) < len(FRAME_TAG) + 4:
                    return
                length = struct.unpack_from('<I', self.buf, len(FRAME_TAG))[0]
                del self.buf[:len(FRAME_TAG) + 4]
                if length == 0 or length > FRAME_LEN_MAX:
                    self.on_text(b'\n[bad frame length %d]\n' % length)
                    continue
                self.need = length
            if len(self.buf) < self.need:
                return
            frame = bytes(self.buf[:self.need])
            del self.buf[:self.need]
            self.need = None
            self.on_frame(frame)


class Viewer:
    """Decode and show the frames, keep the statistics"""

    def __init__(self, args):
        self.display = not args.no_display
        self.scale = args.scale
        self.save = args.save
        self.frames = collections.deque(maxlen=2)   # Newest frames waiting for display
        self.count = 0
        self.bad = 0
        self.bytes = 0
        self.t0 = None
        self.tlast = None
        self.rate = collections.deque(maxlen=30)    # (time, size) of the last frames
        self.running = True
        self.lock = threading.Lock()
        if self.save:
            os.makedirs(self.save, exist_ok=True)

    def text(self, data):
        if data:
            sys.stdout.write(data.decode('ascii', errors='replace'))
            sys.stdout.flush()

    def frame(self, data):
        """Called by the receiver for each frame"""
        now = time.time()
        with self.lock:
            if self.t0 is None:
                self.t0 = now
            self.tlast = now
            self.count += 1
            self.bytes += len(data)
            self.rate.append((now, len(data)))
            num = self.count
        if self.save:
            end = data.rfind(JPEG_EOI)
            jpeg = data[:end + 2] if end >= 0 else data
            with open(os.path.join(self.save, 'frame%05d.jpg' % num), 'wb') as f:
                f.write(jpeg)
        self.frames.append((num, data))
        if not self.display:
            fps, kbs = self.stats()
            print('frame %d: %d bytes, %.1f fps, %.1f KB/s' % (num, len(data), fps, kbs))

    def stats(self):
        """Frame rate and data rate over the last frames"""
        with self.lock:
            if len(self.rate) < 2:
                return 0.0, 0.0
            dt = self.rate[-1][0] - self.rate[0][0]
            if dt <= 0:
                return 0.0, 0.0
            nbytes = sum(s for _, s in list(self.rate)[1:])
            return (len(self.rate) - 1) / dt, nbytes / dt / 1024.0

    def show(self):
        """Show the newest frame, return False to quit"""
        if not self.display:
            return self.running
        try:
            num, data = self.frames.pop()
            self.frames.clear()
        except IndexError:
            num = None
        if num is not None:
            img = cv2.imdecode(np.frombuffer(data, np.uint8), cv2.IMREAD_COLOR)
            if img is None:
                self.bad += 1
            else:
                if self.scale != 1.0:
                    img = cv2.resize(img, None, fx=self.scale, fy=self.scale, interpolation=cv2.INTER_LINEAR)
                fps, kbs = self.stats()
                h, w = img.shape[:2]
                label = '%dx%d  %.1f fps  %.1f KB/s  %d bytes  #%d  bad %d' % (
                    int(w / self.scale), int(h / self.scale), fps, kbs, len(data), num, self.bad)
                cv2.putText(img, label, (8, 20), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 0), 3, cv2.LINE_AA)
                cv2.putText(img, label, (8, 20), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1, cv2.LINE_AA)
                cv2.imshow(WINDOW_NAME, img)
        key = cv2.waitKey(1) & 0xFF
        if key in (ord('q'), 27):
            self.running = False
        return self.running

    def summary(self):
        dt = self.tlast - self.t0 if self.t0 else 0
        avg = (self.count - 1) / dt if dt > 0 and self.count > 1 else 0.0
        print('\n%d frames, %d bytes, %.1f fps average, %d not decoded' % (self.count, self.bytes, avg, self.bad))


def run_serial(args, viewer, parser):
    import serial

    # Opening sets DTR, the USB CDC example captures while it is set
    port = serial.Serial(args.port, args.baud, timeout=0.2)
    print('Receiving on %s' % args.port)

    def reader():
        while viewer.running:
            try:
                data = port.read(port.in_waiting or 1)
            except serial.SerialException as e:
                print('\nSerial error: %s' % e)
                viewer.running = False
                break
            if data:
                parser.feed(data)

    t = threading.Thread(target=reader, daemon=True)
    t.start()
    try:
        while viewer.show():
            if not viewer.display:
                time.sleep(0.05)
    except KeyboardInterrupt:
        pass
    viewer.running = False
    t.join(1)
    port.close()


async def ble_main(args, viewer, parser):
    from bleak import BleakClient, BleakScanner

    if args.address:
        target = args.address
    else:
        print('Scanning for %s' % args.ble)
        target = await BleakScanner.find_device_by_name(args.ble, timeout=15.0)
        if target is None:
            print('%s not found' % args.ble)
            return

    async with BleakClient(target) as client:
        print('Connected to %s, MTU %d' % (args.address or target.name, client.mtu_size))
        await client.start_notify(BLE_DATA_UUID, lambda _, data: parser.feed(bytes(data)))
        try:
            while viewer.show() and client.is_connected:
                await asyncio.sleep(0.005 if viewer.display else 0.05)
        finally:
            if client.is_connected:
                await client.stop_notify(BLE_DATA_UUID)


def main():
    ap = argparse.ArgumentParser(description='Show the frames of the IOsonata video examples')
    src = ap.add_mutually_exclusive_group(required=True)
    src.add_argument('--port', help='Serial port, USB CDC or UART')
    src.add_argument('--ble', nargs='?', const=BLE_DEVICE_NAME, help='BLE device name (default BleVideo)')
    ap.add_argument('--baud', type=int, default=1000000, help='UART baud rate')
    ap.add_argument('--address', help='BLE address or macOS identifier, skips the scan')
    ap.add_argument('--scale', type=float, default=1.0, help='Window scale factor')
    ap.add_argument('--save', help='Directory to write the frames to')
    ap.add_argument('--no-display', action='store_true', help='Print the frame rate only')
    args = ap.parse_args()

    viewer = Viewer(args)
    parser = FrameParser(viewer.text, viewer.frame)

    try:
        if args.port:
            run_serial(args, viewer, parser)
        else:
            asyncio.run(ble_main(args, viewer, parser))
    except KeyboardInterrupt:
        pass
    finally:
        if viewer.display:
            cv2.destroyAllWindows()
        viewer.summary()


if __name__ == '__main__':
    main()
