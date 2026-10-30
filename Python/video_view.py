""" -------------------------------------------------------------------------
@file   video_view.py

@brief  Show the frames of the IOsonata video examples on the PC

Receives the stream of exemples/video/video_usb_demo.cpp (USB CDC),
video_ble_demo.cpp (BLE) or video_capture_demo.cpp (UART) and shows the JPEG
frames in the web browser. A frame is the 4 characters VFRM, the frame length
in bytes 32 bits little endian, then the frame data. The text between the
frames is printed by line. Lines that are not text are counted and reported
with their first bytes, not printed. When bytes of a frame are lost, the next
frame header is inside the frame data: the short frame is reported, shown as
it is, and the stream continues from that header. Text that follows the short
frame, such as a read error the board prints, is printed. --raw writes every byte received
to a file, to look at the stream when no frame is found.

The script serves the frames on http://127.0.0.1:8080 and opens that page.
The page asks for each frame, the newest one is sent. The browser decodes the
frames, no image or window library is needed. The page shows the frame size,
frame rate and data rate. /stream has the same frames as a motion JPEG stream
for a player. Ctrl+C stops the script.

Serial port, USB CDC or UART:
    python3 video_view.py --port /dev/cu.usbmodem1101
BLE with the Bluetooth of the PC, the board advertises as BleVideo:
    python3 video_view.py --ble
    python3 video_view.py --ble --address XX:XX:XX:XX:XX:XX
BLE with an HciController dongle on USB instead of the Bluetooth of the PC:
    python3 video_view.py --hci
    python3 video_view.py --hci /dev/cu.usbmodem14201

The Bluetooth of the PC sets the link: an older Mac agrees to an ATT MTU of
104 only, without 2M PHY or long link layer packets. With --hci the script is
the Bluetooth host itself over HciController (IOsonata HciController firmware
on an nRF52840 dongle, USB CDC H:4 or native USB HCI). It asks for an ATT MTU
of 247, 251 byte link layer packets, the 2M PHY and a 7.5 to 15 ms connection
interval.

Requires pyserial for --port, bleak for --ble. --hci uses the Python library
of HciController: install it with python3 -m pip install -e
<HciController>/python, or keep the HciController checkout next to the
IOsonata one, or set HCICONTROLLER_PYTHON to <HciController>/python.

@param  --port       : Serial port, USB CDC or UART
        --baud       : UART baud rate (default 1MBaud), not used by USB CDC
        --ble        : Connect over BLE to the device of this name
                       (default BleVideo)
        --hci        : Connect over BLE through HciController: auto (default),
                       usb, serial, or the serial port of the dongle
        --name       : Device name to look for with --hci (default BleVideo)
        --trace      : Print every HCI packet in hex, with --hci
        --address    : BLE address or macOS identifier, skips the scan
        --scale      : Picture scale factor (default 1, 2 for small frames)
        --save       : Directory to write the frames to as JPEG files
        --http       : Port of the page (default 8080)
        --no-browser : Do not open the page, open it by hand
        --no-display : Print the frame rate only, no page
        --raw        : File to write all the received bytes to

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
import http.server
import json
import os
import socket
import struct
import sys
import threading
import time
import webbrowser

FRAME_TAG = b'VFRM'
FRAME_LEN_MAX = 8 * 1024 * 1024     # Larger is a corrupted header
JPEG_SOI = b'\xff\xd8'
JPEG_EOI = b'\xff\xd9'
REPORT_INTERVAL = 1.0               # Seconds between two reports of the same problem
CHECK_INTERVAL = 2.0                # Seconds between two checks for no data or a long frame
TEXT_LINE_MAX = 256                 # Longer without an end of line is not text

# BlueIO UART service, the stream comes in the notifications
BLE_DEVICE_NAME = 'BleVideo'
BLE_SERVICE_UUID = '00000101-287c-11e4-ab74-0002a5d5c51b'
BLE_DATA_UUID = '00000102-287c-11e4-ab74-0002a5d5c51b'

HTTP_PORT = 8080
STREAM_BOUNDARY = b'vfrm'

# BLE through HciController, the script is the Bluetooth host
HCI_ATT_MTU = 247
HCI_DATA_LEN = 251                  # Link layer payload, octets
HCI_DATA_TIME = 2120                # Time of a 251 octet packet on the 1M PHY, usec
HCI_CONN_INTERVAL = (6, 12)         # 7.5 to 15 ms, in 1.25 ms
HCI_SUP_TIMEOUT = 400               # 4 s, in 10 ms
HCI_SCAN_TIME = 15.0                # Seconds
HCI_CONNECT_TIME = 10.0             # Seconds
HCI_RSP_TIME = 5.0                  # Seconds for an ATT response
HCI_QUEUE_REPORT = 64               # Packets waiting in the script, reported above that

# Commands, Core Vol 4 Part E 7.1 and 7.8
HCI_OP_DISCONNECT = 0x0406
HCI_OP_LE_SET_EVENT_MASK = 0x2001
HCI_OP_LE_SET_RANDOM_ADDRESS = 0x2005
HCI_OP_LE_SET_SCAN_PARAMS = 0x200B
HCI_OP_LE_SET_SCAN_ENABLE = 0x200C
HCI_OP_LE_CREATE_CONNECTION = 0x200D
HCI_OP_LE_CREATE_CONNECTION_CANCEL = 0x200E
HCI_OP_LE_CONNECTION_UPDATE = 0x2013
HCI_OP_LE_SET_DATA_LENGTH = 0x2022
HCI_OP_LE_SET_PHY = 0x2032
HCI_OP_LE_SET_EXT_SCAN_PARAMS = 0x2041
HCI_OP_LE_SET_EXT_SCAN_ENABLE = 0x2042

# Events, Core Vol 4 Part E 7.7
HCI_EVT_DISCONNECTION_COMPLETE = 0x05
HCI_EVT_LE_META = 0x3E
HCI_LE_ADVERTISING_REPORT = 0x02
HCI_LE_CONNECTION_UPDATE_COMPLETE = 0x03
HCI_LE_DATA_LENGTH_CHANGE = 0x07
HCI_LE_PHY_UPDATE_COMPLETE = 0x0C
HCI_LE_EXT_ADVERTISING_REPORT = 0x0D

# Every LE event but LE Remote Connection Parameter Request (bit 5). With that
# one unmasked the controller waits for the host to answer a parameter request
# of the peripheral, and the link drops when no answer comes.
HCI_LE_EVENT_MASK = bytes.fromhex('dfff000000000000')

H4_ACL = 0x02
H4_EVENT = 0x04

L2CAP_CID_ATT = 0x0004
L2CAP_CID_SIGNAL = 0x0005
L2CAP_CID_SMP = 0x0006
L2CAP_COMMAND_REJECT = 0x01
L2CAP_CONN_PARAM_REQ = 0x12
L2CAP_CONN_PARAM_RSP = 0x13
# LE signaling requests, Core Vol 3 Part A 4
L2CAP_REQUESTS = (0x06, 0x12, 0x14, 0x17, 0x19)

SMP_PAIRING_REQ = 0x01
SMP_PAIRING_FAILED = 0x05
SMP_SECURITY_REQ = 0x0B
SMP_PAIRING_NOT_SUPPORTED = 0x05

ATT_ERROR_RSP = 0x01
ATT_EXCHANGE_MTU_REQ = 0x02
ATT_EXCHANGE_MTU_RSP = 0x03
ATT_FIND_INFO_REQ = 0x04
ATT_FIND_INFO_RSP = 0x05
ATT_READ_BY_TYPE_REQ = 0x08
ATT_READ_BY_TYPE_RSP = 0x09
ATT_WRITE_REQ = 0x12
ATT_WRITE_RSP = 0x13
ATT_NOTIFICATION = 0x1B
ATT_INDICATION = 0x1D
ATT_CONFIRMATION = 0x1E
# Requests of a client. The PC has no attribute, they are answered with an
# error, the discovery ones with Attribute Not Found.
ATT_REQUESTS = (0x04, 0x06, 0x08, 0x0A, 0x0C, 0x0E, 0x10, 0x12, 0x16, 0x18, 0x20)
ATT_DISCOVERY_REQUESTS = (0x04, 0x06, 0x08, 0x10)
ATT_ERR_REQ_NOT_SUPPORTED = 0x06
ATT_ERR_ATTR_NOT_FOUND = 0x0A

GATT_CHAR_DECL = 0x2803
GATT_CCCD = 0x2902
GATT_CCCD_NOTIFY = 0x0001

# Page : the page asks for each frame, the server answers with the newest
# frame once there is one after the last shown. Each frame is shown when it
# arrives, whatever a browser does with a motion JPEG stream, and a slow
# browser skips frames instead of falling behind. The status line is read
# twice a second.
PAGE = '''<!DOCTYPE html>
<html><head><meta charset="utf-8"><title>IOsonata video</title>
<style>
body { margin: 0; background: #202020; color: #e0e0e0; font: 13px monospace; }
#pic { display: block; }
#status { padding: 4px 8px; white-space: pre; }
</style></head>
<body>
<img id="pic" alt="">
<div id="status">Waiting for frames</div>
<script>
const scale = %s;
const pic = document.getElementById('pic');
const status = document.getElementById('status');
async function update() {
  try {
    const s = await (await fetch('/status')).json();
    if (s.width > 0) {
      pic.style.width = (s.width * scale) + 'px';
    }
    status.textContent = s.text;
  } catch (e) {
    status.textContent = 'video_view.py stopped';
  }
}
setInterval(update, 500);
let last = 0;
async function frames() {
  for (;;) {
    try {
      const r = await fetch('/frame?after=' + last, { cache: 'no-store' });
      if (r.status === 200) {
        last = parseInt(r.headers.get('X-Frame'));
        const old = pic.src;
        pic.src = URL.createObjectURL(await r.blob());
        if (old.startsWith('blob:')) {
          URL.revokeObjectURL(old);
        }
      }
    } catch (e) {
      await new Promise(done => setTimeout(done, 500));
    }
  }
}
frames();
</script>
</body></html>
'''

# Start of frame markers, the frame header holds the picture size
JPEG_SOF = (0xC0, 0xC1, 0xC2, 0xC3, 0xC5, 0xC6, 0xC7, 0xC9, 0xCA, 0xCB, 0xCD, 0xCE, 0xCF)


def jpeg_size(data):
    """Width and height in the JPEG frame header, None when there is none"""
    if data[:2] != JPEG_SOI:
        return None
    i = 2
    while i + 9 <= len(data):
        if data[i] != 0xFF:
            return None
        marker = data[i + 1]
        if marker == 0xFF:
            # Fill byte
            i += 1
            continue
        if marker == 0x01 or 0xD0 <= marker <= 0xD8:
            # No length field
            i += 2
            continue
        if marker in JPEG_SOF:
            return (data[i + 7] << 8) | data[i + 8], (data[i + 5] << 8) | data[i + 6]
        if marker == 0xDA:
            # Scan data before the frame header
            return None
        i += 2 + ((data[i + 2] << 8) | data[i + 3])
    return None


class FrameParser:
    """Split the received stream in text and frames, in any part size"""

    def __init__(self, on_text, on_frame, on_lost):
        self.on_text = on_text
        self.on_frame = on_frame
        self.on_lost = on_lost
        self.buf = bytearray()
        self.need = None            # Frame length expected, None while looking for a tag
        self.tail = b''             # Last bytes of the frame just passed on
        self.tailneed = 0           # Its length in the header

    def feed(self, data):
        self.buf += data
        while True:
            if self.need is None:
                if self.tail:
                    if len(self.buf) < len(FRAME_TAG) - 1:
                        return
                    # A header starting in the last bytes of the frame passed
                    # on means bytes of that frame were lost
                    joined = self.tail + bytes(self.buf[:len(FRAME_TAG) - 1])
                    j = joined.find(FRAME_TAG)
                    if 0 <= j < len(self.tail):
                        self.buf[0:0] = self.tail[j:]
                        self.on_lost(self.tailneed, None)
                    self.tail = b''
                idx = self.buf.find(FRAME_TAG)
                if idx < 0:
                    # Keep the end only when it can be the start of a tag.
                    # Text that ends the stream, such as a line the board
                    # sends once, is passed on whole.
                    keep = 0
                    for k in range(len(FRAME_TAG) - 1, 0, -1):
                        if self.buf.endswith(FRAME_TAG[:k]):
                            keep = k
                            break
                    if len(self.buf) > keep:
                        self.on_text(bytes(self.buf[:len(self.buf) - keep]))
                        del self.buf[:len(self.buf) - keep]
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
            # A frame is passed on as soon as its last byte arrives. A header
            # inside it means bytes were lost, the frame ends at the header.
            if len(self.buf) < self.need:
                return
            frame = bytes(self.buf[:self.need])
            lost = frame.find(FRAME_TAG)
            if lost >= 0:
                # Bytes of this frame were lost, the next frame starts at the tag
                region = bytes(self.buf[:lost])
                del self.buf[:lost]
                expected = self.need
                self.need = None
                # Text the board sent after the short frame ends the region
                t = len(region)
                while t > 0 and (32 <= region[t - 1] < 127 or region[t - 1] in (9, 10, 13)):
                    t -= 1
                if len(region) - t < 8 or b'\n' not in region[t:]:
                    t = len(region)
                self.on_lost(expected, region[:t])
                if t < len(region):
                    self.on_text(region[t:])
                continue
            del self.buf[:self.need]
            self.tail = frame[-(len(FRAME_TAG) - 1):]
            self.tailneed = self.need
            self.need = None
            self.on_frame(frame)


class Viewer:
    """Check and serve the frames, keep the statistics"""

    def __init__(self, args):
        self.display = not args.no_display
        self.scale = args.scale
        self.save = args.save
        self.count = 0
        self.bad = 0                # Frames with no JPEG frame header
        self.notjpeg = 0            # Frames not starting with the JPEG start marker
        self.lost = 0               # Frames dropped because bytes were lost
        self.skipped = 0            # Bytes outside the frames that are not text
        self.skipfirst = None       # First of these bytes, shown in the report
        self.textbuf = bytearray()  # Text line not complete yet
        self.treport = {}           # Last report time of each problem
        self.bytes = 0
        self.t0 = None
        self.tlast = None
        self.rate = collections.deque(maxlen=30)    # (time, size) of the last frames
        self.size = (0, 0)          # Picture size of the last frame served
        self.latest = None          # (number, data) of the last frame served
        self.cond = threading.Condition()
        self.server = None
        self.rxbytes = 0            # Bytes received from the board
        self.tcheck = time.time()
        self.rxcheck = 0            # Bytes received at the last check
        self.countcheck = 0         # Frames received at the last check
        self.idle = False           # Nothing received reported, until data comes again
        self.running = True
        self.lock = threading.Lock()
        if self.save:
            os.makedirs(self.save, exist_ok=True)

    def report(self, kind, msg):
        """Print a problem, at most once per REPORT_INTERVAL for each kind"""
        now = time.time()
        if now - self.treport.get(kind, 0) >= REPORT_INTERVAL:
            self.treport[kind] = now
            print('\n[%s]' % msg)

    def text(self, data):
        """Data between the frames, printed by line. A line that is not text is counted, not printed"""
        self.textbuf += data
        while True:
            nl = self.textbuf.find(b'\n')
            if nl < 0:
                if len(self.textbuf) > TEXT_LINE_MAX:
                    self.skip(bytes(self.textbuf))
                    self.textbuf.clear()
                return
            line = bytes(self.textbuf[:nl + 1])
            del self.textbuf[:nl + 1]
            if all(32 <= b < 127 or b in (9, 10, 13) for b in line):
                sys.stdout.write(line.decode('ascii'))
                sys.stdout.flush()
            else:
                self.skip(line)

    def skip(self, data):
        """Data outside the frames that is not text"""
        self.skipped += len(data)
        if self.skipfirst is None:
            self.skipfirst = data[:32]
        self.report('skip', '%d bytes outside a frame, not text, no VFRM header. Wrong baud rate or lost bytes. '
                    'First bytes: %s' % (self.skipped, self.skipfirst.hex(' ')))

    def check(self, parser):
        """Report every CHECK_INTERVAL when nothing comes or a frame takes long"""
        now = time.time()
        if now - self.tcheck < CHECK_INTERVAL:
            return
        self.tcheck = now
        if self.rxbytes == self.rxcheck:
            # Once for each pause, the board may have nothing more to send
            if not self.idle:
                self.idle = True
                self.report('idle', 'nothing received from the board, %d bytes in total. Reset the board to see '
                            'its start messages' % self.rxbytes)
        else:
            self.idle = False
        if self.rxbytes != self.rxcheck and parser.need is not None and self.count == self.countcheck:
            # No frame ended since the last check
            self.report('long', 'receiving a frame of %d bytes, %d received so far, %d bytes in total'
                        % (parser.need, min(len(parser.buf), parser.need), self.rxbytes))
        self.rxcheck = self.rxbytes
        self.countcheck = self.count

    def lost_bytes(self, expected, data):
        """Called by the receiver when a frame is shorter than its header says,
        data None when the frame was already passed on"""
        self.lost += 1
        if data is None:
            self.report('lost', 'frame of %d bytes was short by up to 3 bytes, bytes lost, %d damaged frames shown'
                        % (expected, self.lost))
            return
        self.report('lost', 'frame of %d bytes ended after %d bytes, bytes lost, %d damaged frames shown'
                    % (expected, len(data), self.lost))
        self.frame(data)

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
        if num == 1:
            print('\n[first frame: %d bytes, begins %s]' % (len(data), data[:8].hex(' ')))
        size = None
        if data[:2] != JPEG_SOI:
            self.notjpeg += 1
            self.report('jpeg', 'frame %d: %d bytes, no JPEG start, begins %s, %d frames not JPEG'
                        % (num, len(data), data[:8].hex(' '), self.notjpeg))
        else:
            size = jpeg_size(data)
            if size is None:
                self.bad += 1
                self.report('bad', 'frame %d: %d bytes, JPEG start but no frame header, %d frames'
                            % (num, len(data), self.bad))
        if self.save:
            end = data.rfind(JPEG_EOI)
            jpeg = data[:end + 2] if end >= 0 else data
            with open(os.path.join(self.save, 'frame%05d.jpg' % num), 'wb') as f:
                f.write(jpeg)
        if not self.display:
            fps, kbs = self.stats()
            print('frame %d: %d bytes, %.1f fps, %.1f KB/s' % (num, len(data), fps, kbs))
        elif size is not None:
            # The browser gets only the frames it can show
            with self.cond:
                self.size = size
                self.latest = (num, data)
                self.cond.notify_all()

    def next_frame(self, last, timeout):
        """Frame after the number last, None when none came within timeout"""
        with self.cond:
            if self.latest is None or self.latest[0] == last:
                self.cond.wait(timeout)
            if self.latest is None or self.latest[0] == last:
                return None
            return self.latest

    def status(self):
        fps, kbs = self.stats()
        with self.cond:
            w, h = self.size
            num = self.latest[0] if self.latest else 0
            size = len(self.latest[1]) if self.latest else 0
        if num == 0:
            text = 'Waiting for frames, %d received' % self.count
        else:
            text = '%dx%d  %.1f fps  %.1f KB/s  %d bytes  #%d  damaged %d  not JPEG %d' % (
                w, h, fps, kbs, size, num, self.lost, self.notjpeg + self.bad)
        return {'width': w, 'height': h, 'text': text}

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

    def serve(self, port, browser):
        """Start the page server, False when the port cannot be used"""
        viewer = self
        page = (PAGE % repr(float(self.scale))).encode()

        class Handler(http.server.BaseHTTPRequestHandler):
            def send_body(self, ctype, body):
                self.send_response(200)
                self.send_header('Content-Type', ctype)
                self.send_header('Content-Length', str(len(body)))
                self.send_header('Cache-Control', 'no-cache')
                self.end_headers()
                self.wfile.write(body)

            def do_GET(self):
                if self.path == '/':
                    self.send_body('text/html; charset=utf-8', page)
                elif self.path == '/status':
                    self.send_body('application/json', json.dumps(viewer.status()).encode())
                elif self.path.startswith('/frame'):
                    self.frame()
                elif self.path == '/stream':
                    self.stream()
                else:
                    self.send_error(404)

            def frame(self):
                """Newest frame after the number in after=, no content when
                none comes within a second"""
                try:
                    last = int(self.path.split('after=', 1)[1])
                except (IndexError, ValueError):
                    last = 0
                item = viewer.next_frame(last, 1.0)
                if item is None:
                    self.send_response(204)
                    self.send_header('Cache-Control', 'no-cache')
                    self.end_headers()
                    return
                num, data = item
                self.send_response(200)
                self.send_header('Content-Type', 'image/jpeg')
                self.send_header('Content-Length', str(len(data)))
                self.send_header('Cache-Control', 'no-cache')
                self.send_header('X-Frame', str(num))
                self.end_headers()
                self.wfile.write(data)

            def stream(self):
                """Motion JPEG, each part replaces the picture shown. Not used
                by the page, for a player that takes a stream URL"""
                # Small writes go out at once, not after the acknowledge of
                # the previous part
                self.connection.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
                self.send_response(200)
                self.send_header('Content-Type', 'multipart/x-mixed-replace; boundary=' + STREAM_BOUNDARY.decode())
                self.send_header('Cache-Control', 'no-cache')
                self.end_headers()
                last = 0
                part = b'\r\n--%s\r\nContent-Type: image/jpeg\r\n\r\n' % STREAM_BOUNDARY
                try:
                    self.wfile.write(part[2:])
                    while viewer.running:
                        item = viewer.next_frame(last, 0.5)
                        if item is None:
                            continue
                        last, data = item
                        # The boundary and the header of the next part follow
                        # the picture at once, the browser shows a picture
                        # when the next part has started
                        self.wfile.write(data + part)
                        self.wfile.flush()
                except OSError:
                    # Page closed
                    pass

            def log_message(self, fmt, *args):
                pass

        class Server(http.server.ThreadingHTTPServer):
            def handle_error(self, request, client_address):
                # The page was closed or reloaded during an answer
                if isinstance(sys.exc_info()[1], (BrokenPipeError, ConnectionResetError)):
                    return
                super().handle_error(request, client_address)

        try:
            self.server = Server(('127.0.0.1', port), Handler)
        except OSError as e:
            print('Port %d: %s, use --http to choose another port' % (port, e))
            return False
        self.server.daemon_threads = True
        threading.Thread(target=self.server.serve_forever, daemon=True).start()
        url = 'http://127.0.0.1:%d/' % port
        print('Video on %s' % url)
        if browser:
            webbrowser.open(url)
        return True

    def close(self):
        self.running = False
        with self.cond:
            self.cond.notify_all()
        if self.server is not None:
            self.server.shutdown()
            self.server.server_close()
            self.server = None

    def summary(self):
        dt = self.tlast - self.t0 if self.t0 else 0
        avg = (self.count - 1) / dt if dt > 0 and self.count > 1 else 0.0
        print('\n%d frames, %d bytes, %.1f fps average' % (self.count, self.bytes, avg))
        if self.notjpeg or self.bad or self.lost or self.skipped:
            print('%d frames without JPEG start, %d without JPEG frame header, %d with lost bytes, '
                  '%d bytes outside the frames' % (self.notjpeg, self.bad, self.lost, self.skipped))


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
        while viewer.running:
            time.sleep(0.1)
            viewer.check(parser)
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
            while viewer.running and client.is_connected:
                await asyncio.sleep(0.1)
                viewer.check(parser)
        finally:
            if client.is_connected:
                await client.stop_notify(BLE_DATA_UUID)


class HciLinkError(Exception):
    pass


def hci_library():
    """hci_ble_test of the HciController Python library, installed or in a
    checkout next to the IOsonata one"""
    try:
        import hcicontroller  # noqa: F401
    except ModuleNotFoundError as e:
        if e.name != 'hcicontroller':
            raise
        here = os.path.dirname(os.path.abspath(__file__))
        found = False
        for d in (os.environ.get('HCICONTROLLER_PYTHON'),
                  os.path.join(here, '..', '..', 'HciController', 'python')):
            if d and os.path.isdir(os.path.join(d, 'hcicontroller')):
                sys.path.insert(0, os.path.abspath(d))
                found = True
                break
        if not found:
            raise HciLinkError('HciController Python library not found. Install it with python3 -m pip install -e '
                               '<HciController>/python, or set HCICONTROLLER_PYTHON to <HciController>/python')
    from hcicontroller import hci_ble_test
    return hci_ble_test


def uuid_bytes(text):
    """128 bit UUID as it is on the air, least significant byte first"""
    return bytes(reversed(bytes.fromhex(text.replace('-', ''))))


def addr_text(raw):
    return ':'.join('%02X' % b for b in reversed(raw))


def adv_name(data):
    """Complete or shortened local name in advertising data, '' when none"""
    i = 0
    while i + 1 < len(data):
        n = data[i]
        if n == 0 or i + 1 + n > len(data):
            break
        if data[i + 1] in (0x08, 0x09):
            return data[i + 2:i + 1 + n].decode('utf-8', 'replace')
        i += n + 1
    return ''


def adv_reports(body):
    """(address type, address, data) of each report of an advertising report
    LE meta event"""
    reports = []
    if body[0] == HCI_LE_EXT_ADVERTISING_REPORT and len(body) >= 2:
        # Vol 4 Part E 7.7.65.13, 24 bytes then the data
        i = 2
        for _ in range(body[1]):
            if i + 24 > len(body):
                break
            n = body[i + 23]
            reports.append((body[i + 2], bytes(body[i + 3:i + 9]), bytes(body[i + 24:i + 24 + n])))
            i += 24 + n
    elif body[0] == HCI_LE_ADVERTISING_REPORT and len(body) >= 12 and body[1] == 1:
        # Vol 4 Part E 7.7.65.2, one report
        n = body[10]
        reports.append((body[3], bytes(body[4:10]), bytes(body[11:11 + n])))
    return reports


class HciLink:
    """One connection as central over HciController: L2CAP reassembly, the ATT
    client requests of the script, answers to what the peripheral asks"""

    def __init__(self, hci, handle, on_notify):
        self.hci = hci
        self.handle = handle
        self.on_notify = on_notify
        self.rx = bytearray()       # L2CAP frame being reassembled
        self.rsp = collections.deque()  # ATT responses to the requests of the script
        self.mtu = 23
        self.down = None            # Disconnection reason once the link is gone

    def send(self, cid, pdu):
        self.hci.send_acl(self.handle, cid, pdu)

    def pump(self, timeout):
        """Handle what the controller sent, False when nothing came within timeout"""
        packet = self.hci.read_packet(timeout)
        if packet is None:
            return False
        # Then what is already there, at most a burst so the caller keeps running
        for _ in range(256):
            self.packet(packet)
            packet = self.hci.read_packet(0)
            if packet is None:
                break
        return True

    def packet(self, packet):
        kind, code, body = packet
        if kind == H4_EVENT:
            self.event(code, body)
        elif kind == H4_ACL and len(body) >= 4:
            hdr, length = struct.unpack_from('<HH', body)
            if hdr & 0x0FFF != self.handle:
                return
            data = body[4:4 + length]
            if (hdr >> 12) & 3 == 1:
                # Continuing fragment
                self.rx += data
            else:
                self.rx = bytearray(data)
            if len(self.rx) >= 4:
                n, cid = struct.unpack_from('<HH', self.rx)
                if len(self.rx) >= 4 + n:
                    pdu = bytes(self.rx[4:4 + n])
                    self.rx = bytearray()
                    self.l2cap(cid, pdu)

    def event(self, code, body):
        if code == HCI_EVT_DISCONNECTION_COMPLETE and len(body) >= 4:
            if struct.unpack_from('<H', body, 1)[0] == self.handle:
                self.down = body[3]
        elif code == HCI_EVT_LE_META and len(body) >= 1:
            sub = body[0]
            if sub == HCI_LE_DATA_LENGTH_CHANGE and len(body) >= 11:
                tx, _, rx, _ = struct.unpack_from('<HHHH', body, 3)
                print('Link layer packets: %d bytes out, %d bytes in' % (tx, rx))
            elif sub == HCI_LE_PHY_UPDATE_COMPLETE and len(body) >= 6:
                if body[1] == 0:
                    print('PHY: %s out, %s in' % (self.phy_name(body[4]), self.phy_name(body[5])))
                else:
                    print('PHY update refused, 0x%02X' % body[1])
            elif sub == HCI_LE_CONNECTION_UPDATE_COMPLETE and len(body) >= 10 and body[1] == 0:
                interval, latency, timeout = struct.unpack_from('<HHH', body, 4)
                print('Connection interval %.2f ms, latency %d, timeout %d ms'
                      % (interval * 1.25, latency, timeout * 10))

    @staticmethod
    def phy_name(phy):
        return {1: '1M', 2: '2M', 3: 'Coded'}.get(phy, str(phy))

    def l2cap(self, cid, pdu):
        if not pdu:
            return
        if cid == L2CAP_CID_ATT:
            self.att(pdu)
        elif cid == L2CAP_CID_SIGNAL:
            self.signal(pdu)
        elif cid == L2CAP_CID_SMP and pdu[0] in (SMP_PAIRING_REQ, SMP_SECURITY_REQ):
            # No security on this link
            self.send(L2CAP_CID_SMP, bytes([SMP_PAIRING_FAILED, SMP_PAIRING_NOT_SUPPORTED]))

    def att(self, pdu):
        op = pdu[0]
        if op == ATT_NOTIFICATION and len(pdu) >= 3:
            self.on_notify(struct.unpack_from('<H', pdu, 1)[0], pdu[3:])
        elif op == ATT_INDICATION and len(pdu) >= 3:
            self.send(L2CAP_CID_ATT, bytes([ATT_CONFIRMATION]))
            self.on_notify(struct.unpack_from('<H', pdu, 1)[0], pdu[3:])
        elif op == ATT_EXCHANGE_MTU_REQ and len(pdu) >= 3:
            # The peripheral asks too, the link MTU is the smaller of the two
            peer = struct.unpack_from('<H', pdu, 1)[0]
            self.send(L2CAP_CID_ATT, struct.pack('<BH', ATT_EXCHANGE_MTU_RSP, HCI_ATT_MTU))
            self.mtu = max(23, min(peer, HCI_ATT_MTU))
        elif op in ATT_REQUESTS:
            if op in ATT_DISCOVERY_REQUESTS and len(pdu) >= 3:
                err, hdl = ATT_ERR_ATTR_NOT_FOUND, struct.unpack_from('<H', pdu, 1)[0]
            else:
                err, hdl = ATT_ERR_REQ_NOT_SUPPORTED, 0
            self.send(L2CAP_CID_ATT, struct.pack('<BBHB', ATT_ERROR_RSP, op, hdl, err))
        elif op & 0x40 or op == ATT_CONFIRMATION:
            # Command, no answer
            pass
        else:
            self.rsp.append(pdu)

    def signal(self, pdu):
        if len(pdu) < 4:
            return
        code, ident, length = struct.unpack_from('<BBH', pdu)
        if code == L2CAP_CONN_PARAM_REQ and length >= 8:
            imin, imax, latency, timeout = struct.unpack_from('<HHHH', pdu, 4)
            self.send(L2CAP_CID_SIGNAL, struct.pack('<BBHH', L2CAP_CONN_PARAM_RSP, ident, 2, 0))
            self.hci.command(HCI_OP_LE_CONNECTION_UPDATE,
                             struct.pack('<HHHHHHH', self.handle, imin, imax, latency, timeout, 0, 0),
                             allow_fail=True)
        elif code in L2CAP_REQUESTS:
            # Command not understood
            self.send(L2CAP_CID_SIGNAL, struct.pack('<BBHH', L2CAP_COMMAND_REJECT, ident, 2, 0))

    def request(self, pdu, expect):
        """Send an ATT request, its response or None for an error response"""
        self.rsp.clear()
        self.send(L2CAP_CID_ATT, pdu)
        deadline = time.time() + HCI_RSP_TIME
        while time.time() < deadline:
            if self.down is not None:
                raise HciLinkError('link lost, reason 0x%02X' % self.down)
            if self.rsp:
                rsp = self.rsp.popleft()
                if rsp[0] == expect:
                    return rsp
                if rsp[0] == ATT_ERROR_RSP and len(rsp) >= 5 and rsp[1] == pdu[0]:
                    return None
                continue
            self.pump(0.1)
        raise HciLinkError('no answer to ATT request 0x%02X' % pdu[0])

    def exchange_mtu(self):
        rsp = self.request(struct.pack('<BH', ATT_EXCHANGE_MTU_REQ, HCI_ATT_MTU), ATT_EXCHANGE_MTU_RSP)
        if rsp is not None and len(rsp) >= 3:
            self.mtu = max(23, min(struct.unpack_from('<H', rsp, 1)[0], HCI_ATT_MTU))
        return self.mtu

    def find_char(self, uuid):
        """Value handle of the characteristic of this UUID and the handle of
        the next characteristic declaration (0x10000 when it is the last),
        None when there is no such characteristic"""
        value = None
        start = 1
        while start <= 0xFFFF:
            rsp = self.request(struct.pack('<BHHH', ATT_READ_BY_TYPE_REQ, start, 0xFFFF, GATT_CHAR_DECL),
                               ATT_READ_BY_TYPE_RSP)
            if rsp is None or len(rsp) < 2 or rsp[1] < 7:
                # Attribute Not Found, no declaration left
                break
            n = rsp[1]
            last = start
            for i in range(2, len(rsp) - n + 1, n):
                decl, _, val = struct.unpack_from('<HBH', rsp, i)
                if value is not None:
                    return value, decl
                if rsp[i + 5:i + n] == uuid:
                    value = val
                last = decl
            if last >= 0xFFFF:
                break
            start = last + 1
        return (value, 0x10000) if value is not None else None

    def find_cccd(self, value, end):
        """Client characteristic configuration descriptor between the value
        and the next declaration, the handle after the value when not found"""
        start = value + 1
        while start < end:
            rsp = self.request(struct.pack('<BHH', ATT_FIND_INFO_REQ, start, end - 1), ATT_FIND_INFO_RSP)
            if rsp is None or len(rsp) < 2:
                break
            step = 4 if rsp[1] == 1 else 18
            last = start
            for i in range(2, len(rsp) - step + 1, step):
                hdl = struct.unpack_from('<H', rsp, i)[0]
                if step == 4 and struct.unpack_from('<H', rsp, i + 2)[0] == GATT_CCCD:
                    return hdl
                last = hdl
            start = last + 1
        return value + 1

    def write(self, handle, value):
        if self.request(struct.pack('<BH', ATT_WRITE_REQ, handle) + value, ATT_WRITE_RSP) is None:
            raise HciLinkError('write to handle 0x%04X refused' % handle)

    def disconnect(self):
        if self.down is not None:
            return
        self.hci.command(HCI_OP_DISCONNECT, struct.pack('<HB', self.handle, 0x13), allow_fail=True)
        deadline = time.time() + 1.0
        while self.down is None and time.time() < deadline:
            self.pump(0.1)


def hci_start(hci):
    """Reset, address of the PC side, LE event mask. Returns the own address type"""
    hci.setup()
    identity, own_type, _ = hci.identity()
    if own_type == 0x01:
        hci.command(HCI_OP_LE_SET_RANDOM_ADDRESS, identity)
    hci.command(HCI_OP_LE_SET_EVENT_MASK, HCI_LE_EVENT_MASK)
    return own_type


def hci_scan(hci, own_type, name):
    """((address type, address) of the device that advertises this name, or
    None when none is seen within HCI_SCAN_TIME, True when extended scanning
    was used). Active scan, the name can be in the scan response."""
    params = bytes([own_type, 0x00, 0x01, 0x01]) + struct.pack('<HH', 0x0060, 0x0030)
    status, _ = hci.command(HCI_OP_LE_SET_EXT_SCAN_PARAMS, params, allow_fail=True)
    extended = status == 0
    if extended:
        hci.command(HCI_OP_LE_SET_EXT_SCAN_ENABLE, bytes([0x01, 0x00]) + struct.pack('<HH', 0, 0))
    else:
        # No extended scanning in the controller
        hci.command(HCI_OP_LE_SET_SCAN_PARAMS, bytes([0x01]) + struct.pack('<HH', 0x0060, 0x0030) +
                    bytes([own_type, 0x00]))
        hci.command(HCI_OP_LE_SET_SCAN_ENABLE, bytes([0x01, 0x00]))

    found = None
    deadline = time.time() + HCI_SCAN_TIME
    while found is None and time.time() < deadline:
        packet = hci.read_packet(0.2)
        if packet is None:
            continue
        kind, code, body = packet
        if kind != H4_EVENT or code != HCI_EVT_LE_META or not body:
            continue
        for addr_type, addr, data in adv_reports(body):
            if adv_name(data) == name:
                found = (addr_type & 1, addr)
                break

    if extended:
        hci.command(HCI_OP_LE_SET_EXT_SCAN_ENABLE, bytes([0x00, 0x00]) + struct.pack('<HH', 0, 0),
                    allow_fail=True)
    else:
        hci.command(HCI_OP_LE_SET_SCAN_ENABLE, bytes([0x00, 0x00]), allow_fail=True)
    return found, extended


def hci_connect(hci, own_type, peer_type, peer):
    """Connection handle, None when the connection was not made"""
    from hcicontroller.hci_events import parse_connection

    params = struct.pack('<HH', 0x0060, 0x0030) + bytes([0x00, peer_type]) + peer + bytes([own_type])
    params += struct.pack('<HHHHHH', HCI_CONN_INTERVAL[0], HCI_CONN_INTERVAL[1], 0, HCI_SUP_TIMEOUT, 0, 0)
    hci.command(HCI_OP_LE_CREATE_CONNECTION, params)
    # Data of the new link can come before its connection event: native USB
    # carries events and data on separate endpoints. The peripheral asks for
    # the MTU at once, and an nRF5 SDK peripheral keeps the 23 MTU, 20 byte
    # notifications, until that request is answered. Such data is kept for
    # the link, in its order, not dropped.
    early = []
    deadline = time.time() + HCI_CONNECT_TIME
    while time.time() < deadline:
        packet = hci.read_packet(0.2)
        if packet is None:
            continue
        kind, code, body = packet
        if kind == H4_ACL:
            early.append(packet)
            continue
        if kind != H4_EVENT or code != HCI_EVT_LE_META:
            continue
        info = parse_connection(body)
        if info is None:
            continue
        status, handle, _, _, interval, latency, timeout = info
        if status != 0:
            print('Connection failed, 0x%02X' % status)
            return None
        print('Connection interval %.2f ms, latency %d, timeout %d ms' % (interval * 1.25, latency, timeout * 10))
        hci.pending[0:0] = early
        return handle
    hci.command(HCI_OP_LE_CREATE_CONNECTION_CANCEL, allow_fail=True)
    print('No connection within %.0f s' % HCI_CONNECT_TIME)
    return None


def run_hci(args, viewer, parser):
    try:
        hb = hci_library()
        from hcicontroller import TransportSpec, discover
    except Exception as e:
        print('HciController: %s' % e)
        return

    try:
        if args.hci in ('auto', 'usb', 'serial'):
            try:
                spec = discover(kind=args.hci)
            except Exception as e:
                # In auto mode the error of the native USB search, such as no
                # PyUSB, is reported when no USB H:4 port was found either
                print('No HciController found (%s). Give its serial port with --hci PORT, '
                      'the dongle in USB H:4 mode' % e)
                return
            if spec is None:
                print('No HciController found. Give its serial port with --hci PORT')
                return
        else:
            spec = TransportSpec('serial', args.hci)
        hci = hb.Hci(spec, raw=args.trace)
    except Exception as e:
        print('HciController: %s' % e)
        return

    print('HciController on %s' % spec)
    link = None
    try:
        own_type = hci_start(hci)
        if args.address:
            peer = bytes(reversed(bytes.fromhex(args.address.replace(':', '').replace('-', ''))))
            peer_type = 1
            extended = False
        else:
            print('Scanning for %s' % args.name)
            found, extended = hci_scan(hci, own_type, args.name)
            if found is None:
                print('%s not found' % args.name)
                return
            peer_type, peer = found
        print('Connecting to %s %s' % (args.name, addr_text(peer)))
        if extended:
            # A controller refuses the legacy connection command once an
            # extended scan command has been used, a reset clears that
            own_type = hci_start(hci)
        handle = hci_connect(hci, own_type, peer_type, peer)
        if handle is None:
            return

        value = [None]

        def notify(hdl, data):
            if hdl == value[0]:
                parser.feed(data)
            else:
                viewer.report('handle', 'notification of %d bytes on handle 0x%04X, the video data is on 0x%04X'
                              % (len(data), hdl, value[0] or 0))

        link = HciLink(hci, handle, notify)
        hci.command(HCI_OP_LE_SET_DATA_LENGTH, struct.pack('<HHH', handle, HCI_DATA_LEN, HCI_DATA_TIME),
                    allow_fail=True)
        hci.command(HCI_OP_LE_SET_PHY, struct.pack('<HBBBH', handle, 0x00, 0x02, 0x02, 0), allow_fail=True)
        print('ATT MTU %d' % link.exchange_mtu())
        found = link.find_char(uuid_bytes(BLE_DATA_UUID))
        if found is None:
            print('No video data characteristic %s on %s' % (BLE_DATA_UUID, args.name))
            return
        value[0], end = found
        cccd = link.find_cccd(value[0], end)
        link.write(cccd, struct.pack('<H', GATT_CCCD_NOTIFY))
        print('Receiving, handle 0x%04X' % value[0])

        tqueue = time.time()
        while viewer.running and link.down is None:
            link.pump(0.05)
            viewer.check(parser)
            if time.time() - tqueue >= CHECK_INTERVAL:
                # Packets received from the dongle and not handled yet. A
                # growing number is a script slower than the link.
                tqueue = time.time()
                queued = len(hci.pending) + len(getattr(hci.transport, '_reader_packets', ()))
                if queued > HCI_QUEUE_REPORT:
                    viewer.report('queue', '%d packets from the dongle not handled yet, the video is late'
                                  % queued)
        if link.down is not None:
            print('\nLink lost, reason 0x%02X' % link.down)
    except KeyboardInterrupt:
        pass
    except (HciLinkError, hb.HciError, hb.HciGone) as e:
        print('\n%s' % e)
    finally:
        try:
            if link is not None:
                link.disconnect()
        except Exception:
            pass
        hci.close()


def main():
    ap = argparse.ArgumentParser(description='Show the frames of the IOsonata video examples')
    src = ap.add_mutually_exclusive_group(required=True)
    src.add_argument('--port', help='Serial port, USB CDC or UART')
    src.add_argument('--ble', nargs='?', const=BLE_DEVICE_NAME, help='BLE device name (default BleVideo)')
    src.add_argument('--hci', nargs='?', const='auto',
                     help='BLE through HciController: auto (default), usb, serial or its serial port')
    ap.add_argument('--name', default=BLE_DEVICE_NAME, help='Device name for --hci (default BleVideo)')
    ap.add_argument('--trace', action='store_true', help='Print every HCI packet in hex (--hci)')
    ap.add_argument('--baud', type=int, default=1000000, help='UART baud rate')
    ap.add_argument('--address', help='BLE address or macOS identifier, skips the scan')
    ap.add_argument('--scale', type=float, default=1.0, help='Picture scale factor')
    ap.add_argument('--save', help='Directory to write the frames to')
    ap.add_argument('--http', type=int, default=HTTP_PORT, help='Port of the page (default %d)' % HTTP_PORT)
    ap.add_argument('--no-browser', action='store_true', help='Do not open the page')
    ap.add_argument('--no-display', action='store_true', help='Print the frame rate only')
    ap.add_argument('--raw', help='File to write all the received bytes to')
    args = ap.parse_args()

    viewer = Viewer(args)
    parser = FrameParser(viewer.text, viewer.frame, viewer.lost_bytes)
    raw = open(args.raw, 'wb') if args.raw else None
    feed = parser.feed

    def feed_count(data):
        # Every byte received, before the parser
        viewer.rxbytes += len(data)
        if raw is not None:
            raw.write(data)
        feed(data)
    parser.feed = feed_count

    if viewer.display and not viewer.serve(args.http, not args.no_browser):
        return

    try:
        if args.port:
            run_serial(args, viewer, parser)
        elif args.hci:
            run_hci(args, viewer, parser)
        else:
            asyncio.run(ble_main(args, viewer, parser))
    except KeyboardInterrupt:
        pass
    finally:
        viewer.close()
        viewer.summary()
        if raw is not None:
            raw.close()
            print('Received bytes in %s' % args.raw)


if __name__ == '__main__':
    main()
