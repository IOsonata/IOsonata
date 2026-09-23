#!/usr/bin/env python3
# Python/dfu_prod_hex.py: with a slot 1 layout the image lands at slot 1 and
# the trailer says pending; without, the payload lands at slot 0 and the
# record is what the boot reads. Every layout file in the tree parses, and a
# too large image or a file that is not an image is refused. The slot 0 case
# is also checked by running the boot on it (dfu_prod_boot, see Makefile).
#
# Usage: prod_hex_test.py <image dir> [dfu_prod_boot]

import glob
import os
import struct
import subprocess
import sys
import tempfile

from intelhex import IntelHex

IMG = sys.argv[1]
ROOT = os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "..")
TOOL = os.path.join(ROOT, "Python", "dfu_prod_hex.py")
fail = 0


def check(c, what):
    global fail
    print(("  ok    " if c else "  FAIL  ") + what)
    fail += 0 if c else 1


sys.dont_write_bytecode = True
import importlib.util
spec = importlib.util.spec_from_file_location("tool", TOOL)
tool = importlib.util.module_from_spec(spec)
spec.loader.exec_module(tool)

img = open(os.path.join(IMG, "v2.bin"), "rb").read()
hdr, size, tlv = tool.parse_image(img)

layouts = sorted(glob.glob(os.path.join(ROOT, "ARM", "**", "dfu_layout_*.ld"),
                           recursive=True) +
                 glob.glob(os.path.join(ROOT, "RISCV", "**", "dfu_layout_*.ld"),
                           recursive=True))
check(len(layouts) > 10, "%d layout files found" % len(layouts))

with tempfile.TemporaryDirectory() as d:
    out = os.path.join(d, "o.hex")
    for ld in layouts:
        name = os.path.basename(ld)
        lay = tool.read_layout(ld)
        if lay["slot0_size"] < size:
            continue
        subprocess.run([sys.executable, TOOL, "--layout", ld,
                        os.path.join(IMG, "v2.bin"), out], check=True)
        ih = IntelHex(out)
        if lay["slot1"] is not None:
            s1, n1 = lay["slot1"], lay["slot1_size"]
            ok = ih.tobinstr(s1, s1 + len(img) - 1) == img and \
                struct.unpack("<I", ih.tobinstr(s1 + n1 - 32, s1 + n1 - 29))[0] \
                == 0x444E4550 and len(ih.segments()) == 2
            check(ok, "%s: slot 1, pending" % name)
        else:
            s0, r, u = lay["slot0"], lay["rec"], lay["unit"]
            ok = ih.tobinstr(s0, s0 + size - 1) == img[hdr:hdr + size]
            ok = ok and struct.unpack("<I", ih.tobinstr(r, r + 3))[0] == \
                0x43455244
            hl, tl = struct.unpack("<HH", ih.tobinstr(r + u, r + u + 3))
            ok = ok and hl == hdr and tl == tlv
            ok = ok and ih.tobinstr(r + u + 4, r + u + 4 + hdr - 1) == img[:hdr]
            check(ok, "%s: slot 0 and record, unit %d" % (name, u))

    r = subprocess.run([sys.executable, TOOL, "--slot0", "0x1000",
                        "--slot0-size", "0x1000", "--rec", "0x3000",
                        os.path.join(IMG, "v2.bin"), out])
    check(r.returncode != 0, "too large refused")
    r = subprocess.run([sys.executable, TOOL, "--slot0", "0x10000000",
                        "--slot0-size", "0x10000", "--rec", "0x10010000",
                        os.path.join(IMG, "app1.bin"), out])
    check(r.returncode != 0, "unsigned binary refused")

    # The boot on the host layout of the tests, with no slot 1.
    if len(sys.argv) > 2:
        for name, want in (("v2.bin", 0), ("v1_badsig.bin", 1),
                           ("v1_tamper.bin", 1)):
            subprocess.run([sys.executable, TOOL, "--slot0", "0x10000000",
                            "--slot0-size", "0x10000", "--rec", "0x10010000",
                            os.path.join(IMG, name), out], check=True)
            ih = IntelHex(out)
            dump = os.path.join(d, "dump.bin")
            open(dump, "wb").write(ih.tobinstr(0x10000000, 0x10010FFF))
            r = subprocess.run([sys.argv[2], IMG, dump], capture_output=True,
                               text=True)
            check(r.returncode == want, "boot on %s from the hex: %s" %
                  (name, r.stdout.strip()))

print("prod_hex_test: %s" % ("PASS" if fail == 0 else "%d FAILED" % fail))
sys.exit(1 if fail else 0)
