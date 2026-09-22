#!/usr/bin/env python3
"""Stage 0 recovery driven by Python/dfu_wire.py over a pseudo terminal.

dfu_wire_host serves the wire protocol on simulated memory; this runs the
host tool against it as a user would: info, uploads, an older image, a bad
signature, a tampered payload, a link lost in the middle and taken up again,
and checks after each reset what the boot started.

    wire_test.py <dfu_wire_host> <image dir> [nor|rram|ecc16|big256|sect]
"""

import contextlib
import io
import os
import queue
import struct
import subprocess
import sys
import threading
import time

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, "..", "..", "Python"))
import dfu_wire  # noqa: E402

fails = 0


def ok(cond, what):
    global fails
    print("  %s  %s" % ("ok  " if cond else "FAIL", what))
    if not cond:
        fails += 1


def run_tool(*args):
    out = io.StringIO()
    err = io.StringIO()
    with contextlib.redirect_stdout(out), contextlib.redirect_stderr(err):
        rc = dfu_wire.main(list(args))
    return rc, out.getvalue() + err.getvalue()


def main():
    host, imgdir = sys.argv[1], sys.argv[2]
    kind = sys.argv[3] if len(sys.argv) > 3 else "nor"
    p = subprocess.Popen([host, imgdir, kind], stdout=subprocess.PIPE,
                         text=True, bufsize=1)
    port = p.stdout.readline().split()[0]
    lines = queue.Queue()

    def reader():
        for l in p.stdout:
            lines.put(l.strip())

    threading.Thread(target=reader, daemon=True).start()

    def boot_line(timeout=10.0):
        """The line of the boot that ends in recovery, and the start line
        before it when there is one."""
        seen = []
        end = time.monotonic() + timeout
        while time.monotonic() < end:
            try:
                l = lines.get(timeout=0.2)
            except queue.Empty:
                continue
            seen.append(l)
            if "recovery" in l:
                return seen
        return seen

    def started(seen):
        for l in seen:
            if "started slot 0" in l:
                return l.split("started slot 0 ")[1].split(",")[0]
        return None

    print("%s %s" % (port, kind))
    img = lambda n: os.path.join(imgdir, n)

    first = boot_line()
    ok(started(first) is None, "empty device: recovery")

    rc, out = run_tool("--port", port, "--info")
    ok(rc == 0 and "maxbody" in out and "0.0.0+0" in out, "info")

    rc, out = run_tool("--port", port, "-q", img("v1.bin"))
    ok(rc == 0, "v1 upload")
    ok(started(boot_line()) == "v1", "v1 started")

    rc, out = run_tool("--port", port, "-q", img("v2.bin"))
    ok(rc == 0, "v2 upload, padded header")
    ok(started(boot_line()) == "v2", "v2 started")

    rc, out = run_tool("--port", port, "-q", img("v1.bin"))
    ok(rc == 1 and "older" in out, "v1 refused as older: %s" % out.strip())
    rc, out = run_tool("--port", port, "-q", img("v1_badsig.bin"))
    ok(rc == 1 and "signature" in out, "bad signature refused")
    rc, out = run_tool("--port", port, "-q", img("v2_key2.bin"))
    ok(rc == 1 and "key" in out, "other key refused")

    # Nothing was touched: a reset starts v2 again.
    link = dfu_wire.Link(port, 115200, None)
    dfu_wire.check(link.request(dfu_wire.OP_RESET, b"\x01"), "RESET")
    link.close()
    ok(started(boot_line()) == "v2", "v2 still there after the refusals")

    # Tampered payload with a good manifest: refused at FINISH, no image.
    with open(img("v3.bin"), "rb") as f:
        d = bytearray(f.read())
    hdr = struct.unpack("<H", d[8:10])[0]
    d[hdr + 1000] ^= 0x10
    bad = os.path.join(imgdir, "v3_tampered.bin")
    with open(bad, "wb") as f:
        f.write(d)
    rc, out = run_tool("--port", port, "-q", bad)
    ok(rc == 1 and "hash" in out, "tampered payload refused")
    link = dfu_wire.Link(port, 115200, None)
    dfu_wire.check(link.request(dfu_wire.OP_RESET, b"\x01"), "RESET")
    link.close()
    ok(started(boot_line()) is None, "nothing startable after it")

    # Link lost halfway through v3, then taken up again.
    man, pay = dfu_wire.load_image(img("v3.bin"))
    link = dfu_wire.Link(port, 115200, None)
    inf = dfu_wire.info(link)
    mb = inf["maxbody"]
    for off in range(0, len(man), mb):
        dfu_wire.check(link.request(dfu_wire.OP_BEGIN,
                                    struct.pack("<HH", len(man), off) + man[off:off + mb],
                                    timeout=5.0), "BEGIN")
    half = (len(pay) // 2) // mb * mb
    for off in range(0, half, mb):
        dfu_wire.check(link.request(dfu_wire.OP_WRITE,
                                    struct.pack("<I", off) + pay[off:off + mb]),
                       "WRITE")
    # A frame cut short, then garbage, as a cable pulled mid frame.
    link.ser.write(bytes([0xC0, dfu_wire.OP_WRITE, 1, 2, 3]))
    link.ser.write(b"line noise \xdb\x01")
    link.close()
    rc, out = run_tool("--port", port, img("v3.bin"))
    ok(rc == 0 and ("resuming at %d" % half) in out, "resumed at %d" % half)
    ok(started(boot_line()) == "v3", "v3 started")

    rc, out = run_tool("--port", port, "-q", img("v4.bin"))
    ok(rc == 0, "v4 upload")
    ok(started(boot_line()) == "v4", "v4 started")

    # Stay in recovery when asked.
    rc, out = run_tool("--port", port, "-q", "--no-start", img("v4.bin"))
    seen = boot_line()
    ok(rc == 0 and started(seen) is None, "no start: stays in recovery")

    os.remove(bad)
    p.kill()
    print("wire_test %s: %s" % (kind, "PASS" if fails == 0 else "%d FAILED" % fails))
    return 1 if fails else 0


if __name__ == "__main__":
    sys.exit(main())
