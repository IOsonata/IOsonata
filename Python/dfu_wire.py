#!/usr/bin/env python3
"""Send a signed image to the IOsonata stage 0 boot over its wire protocol.

The boot serves recovery over a UART or USB CDC port; this tool talks to it
the same way on macOS, Windows and Linux, with pyserial alone. The protocol
is docs/architecture/dfu_wire.md: SLIP frames, fixed binary operations, the
image manifest (header and TLVs) first so the boot checks the signature
before it erases or writes anything, then the payload.

    dfu_wire.py --port /dev/cu.usbmodem1101 app_signed.bin
    dfu_wire.py --port COM7 --baud 115200 app_signed.bin
    dfu_wire.py --vid 0x1209 --pid 0x0001 app_signed.bin
    dfu_wire.py --port /dev/ttyACM0 --info

The image is what imgtool sign writes (.bin), or an Intel hex of it when the
intelhex package is there. On macOS use the /dev/cu.* device: /dev/tty.*
waits for carrier.

A lost link is taken up again: the port is reopened (found again by VID,
PID and serial number when given, since a USB replug renames it on macOS),
INFO says where the boot is, and the upload goes on from there when the
bytes it holds match.
"""

import argparse
import struct
import sys
import time
import zlib

import serial
import serial.tools.list_ports

END, ESC, ESC_END, ESC_ESC = 0xC0, 0xDB, 0xDC, 0xDD

OP_INFO, OP_BEGIN, OP_WRITE, OP_FINISH, OP_RESET, OP_ABORT, OP_ENTER = range(1, 8)
OP_RSP = 0x80

STATUS = {
    0: "ok", 1: "not valid in this state", 2: "length", 3: "image header",
    4: "size", 5: "TLV area", 6: "key not the boot's", 7: "signature",
    8: "older than the image in place", 9: "offset", 10: "memory",
    11: "hash, the payload is not the one signed", 12: "entry not for slot 0",
}

STATE = {0: "idle", 1: "receiving", 2: "done"}

IMG_MAGIC = 0x96F3B83D


class WireError(Exception):
    pass


def slip(raw):
    out = bytearray([END])
    for b in raw:
        if b == END:
            out += bytes([ESC, ESC_END])
        elif b == ESC:
            out += bytes([ESC, ESC_ESC])
        else:
            out.append(b)
    out.append(END)
    return bytes(out)


class Link:
    """One request, one response, over a serial port."""

    def __init__(self, port, baud, find, verbose=False):
        self.port_name = port
        self.baud = baud
        self.find = find
        self.verbose = verbose
        self.seq = 0
        self.ser = None
        self.buf = bytearray()
        self.open()

    def locate(self):
        if self.find is None:
            return self.port_name
        vid, pid, sn = self.find
        for p in serial.tools.list_ports.comports():
            if p.vid == vid and p.pid == pid and (sn is None or p.serial_number == sn):
                return p.device
        return None

    def open(self, wait=0.0):
        end = time.monotonic() + wait
        while True:
            name = self.locate()
            if name is not None:
                try:
                    self.ser = serial.Serial(name, self.baud, timeout=0.05,
                                             dsrdtr=False, rtscts=False)
                    # The IOsonata CDC class takes DTR as the port being open.
                    # A pseudo terminal has no modem lines.
                    try:
                        self.ser.dtr = True
                    except (serial.SerialException, OSError):
                        pass
                    self.ser.reset_input_buffer()
                    self.port_name = name
                    self.buf = bytearray()
                    return
                except (serial.SerialException, OSError):
                    pass
            if time.monotonic() >= end:
                raise WireError("port not found or not opened: %s" %
                                (name or "no matching device"))
            time.sleep(0.2)

    def close(self):
        if self.ser is not None:
            try:
                self.ser.close()
            except (serial.SerialException, OSError):
                pass
            self.ser = None

    def read_frame(self, deadline):
        """Next well formed frame, or None at the deadline."""
        while time.monotonic() < deadline:
            data = self.ser.read(256)
            if data:
                self.buf += data
            while END in self.buf:
                i = self.buf.index(END)
                chunk = bytes(self.buf[:i])
                del self.buf[:i + 1]
                raw = bytearray()
                esc = False
                bad = False
                for b in chunk:
                    if esc:
                        if b not in (ESC_END, ESC_ESC):
                            bad = True
                            break
                        raw.append(END if b == ESC_END else ESC)
                        esc = False
                    elif b == ESC:
                        esc = True
                    else:
                        raw.append(b)
                if bad or esc or len(raw) < 6:
                    continue
                if zlib.crc32(raw[:-4]) != struct.unpack("<I", raw[-4:])[0]:
                    continue
                return bytes(raw[:-4])
        return None

    def request(self, op, body=b"", timeout=1.0, tries=4):
        last = None
        for _ in range(tries):
            self.seq = (self.seq + 1) & 0xFF
            raw = bytes([op, self.seq]) + body
            raw += struct.pack("<I", zlib.crc32(raw))
            try:
                self.ser.write(slip(raw))
                self.ser.flush()
                deadline = time.monotonic() + timeout
                while True:
                    f = self.read_frame(deadline)
                    if f is None:
                        break
                    if f[0] == (op | OP_RSP) and f[1] == self.seq:
                        return f[2:]
            except (serial.SerialException, OSError) as e:
                last = e
                self.close()
                self.open(wait=10.0)
        raise WireError("no response to operation %d%s" %
                        (op, (": %s" % last) if last else ""))


def status_of(body):
    return body[0] if body else -1


def check(body, what):
    st = status_of(body)
    if st != 0:
        raise WireError("%s: %s" % (what, STATUS.get(st, "status %d" % st)))


def info(link):
    b = link.request(OP_INFO)
    check(b, "INFO")
    if len(b) < 40:
        raise WireError("INFO: short response")
    (st, ver, state, flags, maxbody, wunit, erasems, manmax, slot, off,
     vmaj, vmin, vrev, vbuild, devid, bootver) = struct.unpack(
        "<BBBBHHHHIIBBHI8sI", b[:40])
    return {
        "proto": ver, "state": state, "downgrade": bool(flags & 1),
        "maxbody": maxbody, "writeunit": wunit, "erasems": erasems,
        "manifestmax": manmax, "slotsize": slot, "offset": off,
        "version": "%d.%d.%d+%d" % (vmaj, vmin, vrev, vbuild),
        "devid": devid.hex(), "bootver": "%08x" % bootver,
    }


def load_image(path):
    if path.lower().endswith(".hex"):
        from intelhex import IntelHex
        ih = IntelHex(path)
        data = ih.tobinstr()
    else:
        with open(path, "rb") as f:
            data = f.read()
    if len(data) < 32:
        raise WireError("not an image: too short")
    magic, _load, hdr, _prot, size = struct.unpack("<IIHHI", data[:16])
    if magic != IMG_MAGIC:
        raise WireError("not an MCUboot image (magic %08x)" % magic)
    if hdr + size > len(data):
        raise WireError("image shorter than its header says")
    manifest = data[:hdr] + data[hdr + size:]
    payload = data[hdr:hdr + size]
    return manifest, payload


def progress(done, total, t0):
    rate = done / max(time.monotonic() - t0, 1e-3) / 1024
    sys.stdout.write("\r  %7d / %d bytes  %5.1f KB/s" % (done, total, rate))
    sys.stdout.flush()


def upload(link, manifest, payload, resume=True, quiet=False):
    inf = info(link)
    if not quiet:
        print("boot %s, protocol %d, slot 0 holds %s, %s" %
              (inf["bootver"], inf["proto"], inf["version"],
               STATE.get(inf["state"], "?")))
    maxbody = inf["maxbody"]
    if len(manifest) > inf["manifestmax"]:
        raise WireError("manifest of %d bytes, the boot takes %d" %
                        (len(manifest), inf["manifestmax"]))
    wr_timeout = inf["erasems"] / 1000.0 + 1.0

    start = 0
    if resume and inf["state"] == 1 and 0 < inf["offset"] <= len(payload):
        # Taken up where it is, when what it holds is what we would send.
        b = link.request(OP_WRITE, struct.pack("<I", 0) + payload[:1],
                         timeout=wr_timeout)
        if status_of(b) == 0 and len(b) >= 9:
            off, crc = struct.unpack("<II", b[1:9])
            if crc == zlib.crc32(payload[:off]):
                start = off
                if not quiet:
                    print("  resuming at %d" % off)

    if start == 0:
        for off in range(0, len(manifest), maxbody):
            piece = manifest[off:off + maxbody]
            b = link.request(OP_BEGIN,
                             struct.pack("<HH", len(manifest), off) + piece,
                             timeout=5.0)
            check(b, "BEGIN")

    t0 = time.monotonic()
    off = start
    while off < len(payload):
        piece = payload[off:off + maxbody]
        b = link.request(OP_WRITE, struct.pack("<I", off) + piece,
                         timeout=wr_timeout)
        st = status_of(b)
        if len(b) >= 9:
            dev_off, crc = struct.unpack("<II", b[1:9])
        else:
            dev_off, crc = None, None
        if st == 9 and dev_off is not None and dev_off < off:
            # The boot is behind: go back to where it is.
            off = dev_off
            continue
        check(b, "WRITE at %d" % off)
        if dev_off != off + len(piece) or crc != zlib.crc32(payload[:dev_off]):
            raise WireError("WRITE at %d: the boot holds other bytes" % off)
        off = dev_off
        if not quiet:
            progress(off, len(payload), t0)
    if not quiet:
        print()

    b = link.request(OP_FINISH, timeout=60.0)
    check(b, "FINISH")
    if not quiet:
        print("  checked and in place")


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("image", nargs="?", help="signed image, .bin or .hex")
    ap.add_argument("--port", help="serial port (/dev/cu.*, COMn, /dev/ttyACMn)")
    ap.add_argument("--baud", type=int, default=115200)
    ap.add_argument("--vid", type=lambda s: int(s, 0), help="find the port by USB id")
    ap.add_argument("--pid", type=lambda s: int(s, 0))
    ap.add_argument("--serial", help="USB serial number, with --vid and --pid")
    ap.add_argument("--info", action="store_true", help="show what the boot says")
    ap.add_argument("--enter", action="store_true",
                    help="ask a running application to restart into recovery first")
    ap.add_argument("--no-start", action="store_true",
                    help="stay in recovery after the upload")
    ap.add_argument("--no-resume", action="store_true")
    ap.add_argument("-q", "--quiet", action="store_true")
    a = ap.parse_args(argv)

    find = None
    if a.vid is not None or a.pid is not None:
        if a.vid is None or a.pid is None:
            ap.error("--vid and --pid go together")
        find = (a.vid, a.pid, a.serial)
    elif a.port is None:
        ap.error("--port or --vid/--pid")
    if not a.info and a.image is None:
        ap.error("an image, or --info")

    try:
        link = Link(a.port, a.baud, find)
        if a.enter:
            b = link.request(OP_ENTER, timeout=2.0)
            check(b, "ENTER")
            link.close()
            time.sleep(0.5)
            link.open(wait=15.0)
        if a.info:
            for k, v in info(link).items():
                print("%-12s %s" % (k, v))
            if a.image is None:
                return 0
        manifest, payload = load_image(a.image)
        upload(link, manifest, payload, resume=not a.no_resume, quiet=a.quiet)
        b = link.request(OP_RESET, bytes([0 if a.no_start else 1]), timeout=2.0)
        check(b, "RESET")
        link.close()
        return 0
    except WireError as e:
        print("error: %s" % e, file=sys.stderr)
        return 1


if __name__ == "__main__":
    sys.exit(main())
