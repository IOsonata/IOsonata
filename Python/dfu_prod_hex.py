#!/usr/bin/env python3
"""Production hex of a signed image, for a device with the stage 0 DFU boot.

Reads the layout from the target's dfu_layout_*.ld, the same file the boot
and the application link with, so the addresses cannot drift:

  layout with slot 1 (OTA parts)
      The image goes in slot 1 with its trailer marked pending. At the first
      reset the boot checks the signature and installs it, the same way it
      installs an update.

  layout without slot 1 (parts updated by wire)
      The payload goes in slot 0 and the header and TLV areas in the record,
      as stage 0 recovery would have written them. The boot checks the
      signature at every start (bVerifySlot0), so an image that does not
      verify is not started.

Merge the output with the boot hex (and the SoftDevice where there is one).

    dfu_prod_hex.py --layout ARM/Nordic/nRF52/nRF52840/ldscript/dfu_layout_nrf52840_s140.ld \\
        app_signed.bin app_prod.hex
    dfu_prod_hex.py --layout .../dfu_layout_lpc11u35.ld app_signed.bin out.hex

The state unit, 16 or the program unit of the part's memory when that is
larger (256 on LPC, 512 on SAM4), places the trailer words and the record
information. It is read from the layout's __dfu_state_unit, 16 when the
layout has none; --unit gives it for a layout given by hand.

The layout symbols can be given by hand instead of --layout:
    --slot0 A --slot0-size N --rec A [--slot1 A --slot1-size N]
"""

import argparse
import re
import struct
import sys

from intelhex import IntelHex

IMG_MAGIC = 0x96F3B83D
TLV_INFO_MAGIC = 0x6907
TLV_PROT_MAGIC = 0x6908
TRAILER_PENDING = 0x444E4550
REC_MAGIC = 0x43455244
HDR_MAX = 0x800
TLV_MAX = 0x400


def read_layout(path):
    sym = {}
    for k, v in re.findall(r"(__dfu_\w+)\s*=\s*(0x[0-9A-Fa-f]+|\d+)\s*;",
                           open(path).read()):
        sym[k] = int(v, 0)
    lay = {
        "slot0": sym["__dfu_slot0_start"],
        "slot0_size": sym["__dfu_slot0_end"] - sym["__dfu_slot0_start"],
        "rec": sym["__dfu_rec_start"],
        "rec_size": sym["__dfu_rec_end"] - sym["__dfu_rec_start"],
        "slot1": None,
        "slot1_size": 0,
        "unit": sym.get("__dfu_state_unit", 16),
    }
    if sym.get("__dfu_slot1_end", 0) > sym.get("__dfu_slot1_start", 0):
        lay["slot1"] = sym["__dfu_slot1_start"]
        lay["slot1_size"] = sym["__dfu_slot1_end"] - sym["__dfu_slot1_start"]
    return lay


def parse_image(img):
    """Header size, payload size and length of both TLV areas."""
    if len(img) < 32 or struct.unpack_from("<I", img, 0)[0] != IMG_MAGIC:
        sys.exit("not an MCUboot image")
    hdr_size, prot = struct.unpack_from("<HH", img, 8)
    size = struct.unpack_from("<I", img, 12)[0]
    if hdr_size < 32 or hdr_size > HDR_MAX:
        sys.exit("header size %d not taken" % hdr_size)
    off = hdr_size + size
    if prot:
        if struct.unpack_from("<H", img, off)[0] != TLV_PROT_MAGIC:
            sys.exit("protected TLV area missing")
    magic, tot = struct.unpack_from("<HH", img, off + prot)
    if magic != TLV_INFO_MAGIC:
        sys.exit("TLV area missing")
    tlv = prot + tot
    if tlv > TLV_MAX or off + tlv > len(img):
        sys.exit("TLV areas too large or cut")
    return hdr_size, size, tlv


def main():
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--layout", help="dfu_layout_*.ld of the target")
    num = lambda s: int(s, 0)
    ap.add_argument("--slot0", type=num)
    ap.add_argument("--slot0-size", type=num)
    ap.add_argument("--rec", type=num)
    ap.add_argument("--slot1", type=num)
    ap.add_argument("--slot1-size", type=num, default=0)
    ap.add_argument("--unit", type=num)
    ap.add_argument("image", help="image signed by imgtool, binary")
    ap.add_argument("out", help="Intel hex output")
    a = ap.parse_args()

    if a.layout:
        lay = read_layout(a.layout)
    elif a.slot0 is not None and a.slot0_size and a.rec is not None:
        lay = {"slot0": a.slot0, "slot0_size": a.slot0_size, "rec": a.rec,
               "rec_size": None, "slot1": a.slot1,
               "slot1_size": a.slot1_size if a.slot1 is not None else 0}
    else:
        ap.error("give --layout, or the layout symbols by hand")

    unit = a.unit if a.unit is not None else lay.get("unit", 16)
    if a.layout and a.unit is not None and a.unit != lay["unit"]:
        sys.exit("--unit %d differs from the layout's %d" % (a.unit, lay["unit"]))
    if unit < 16 or unit & (unit - 1):
        sys.exit("unit must be a power of 2, 16 or more")

    img = open(a.image, "rb").read()
    hdr_size, size, tlv = parse_image(img)
    if size > lay["slot0_size"]:
        sys.exit("payload %d bytes, slot 0 holds %d" % (size, lay["slot0_size"]))

    ih = IntelHex()
    if lay["slot1"] is not None and lay["slot1_size"]:
        trailer = 2 * unit
        if len(img) > lay["slot1_size"] - trailer:
            sys.exit("%d bytes, slot 1 holds %d" %
                     (len(img), lay["slot1_size"] - trailer))
        ih.frombytes(img, offset=lay["slot1"])
        ih.frombytes(struct.pack("<I", TRAILER_PENDING),
                     offset=lay["slot1"] + lay["slot1_size"] - trailer)
    else:
        body = struct.pack("<HH", hdr_size, tlv) + img[:hdr_size] + \
            img[hdr_size + size:hdr_size + size + tlv]
        if lay["rec_size"] is not None and unit + len(body) > lay["rec_size"]:
            sys.exit("record does not fit")
        ih.frombytes(img[hdr_size:hdr_size + size], offset=lay["slot0"])
        ih.frombytes(struct.pack("<I", REC_MAGIC), offset=lay["rec"])
        ih.frombytes(body, offset=lay["rec"] + unit)
    ih.write_hex_file(a.out)


if __name__ == "__main__":
    main()
