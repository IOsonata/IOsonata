#!/usr/bin/env python3
# Test images for the DFU host tests, signed with MCUboot imgtool.
#
# Usage: mkimages.py <out dir> <slot 0 address>
#
# Writes two P-256 keys, the DER public keys, and a set of images: good ones
# with small and padded headers, one signed with the other key, one with no
# signature, one with a protected TLV, one with a payload byte changed after
# signing and one whose reset vector points outside the payload.

import os
import random
import struct
import subprocess
import sys

from cryptography.hazmat.primitives import serialization

out = sys.argv[1]
slot0 = int(sys.argv[2], 0)
os.makedirs(out, exist_ok=True)


def run(*args):
    subprocess.run(args, check=True, stdout=subprocess.DEVNULL)


def key(name):
    pem = os.path.join(out, name + ".pem")
    if not os.path.exists(pem):
        run("imgtool", "keygen", "-k", pem, "-t", "ecdsa-p256")
    k = serialization.load_pem_private_key(open(pem, "rb").read(), None)
    der = k.public_key().public_bytes(
        serialization.Encoding.DER,
        serialization.PublicFormat.SubjectPublicKeyInfo)
    open(os.path.join(out, name + ".der"), "wb").write(der)
    return pem


def payload(name, size, seed, reset_off=0x101):
    rnd = random.Random(seed)
    data = bytearray(rnd.getrandbits(8) for _ in range(size))
    struct.pack_into("<II", data, 0, 0x20008000, slot0 + reset_off)
    path = os.path.join(out, name + ".bin")
    open(path, "wb").write(data)
    return path


def sign(src, dst, ver, pem=None, hdr="0x20", extra=()):
    args = ["imgtool", "sign", "--header-size", hdr, "--pad-header",
            "--align", "4", "-v", ver, "-S", "0x10FF0", *extra]
    if pem:
        args += ["-k", pem]
    run(*args, src, os.path.join(out, dst))


k1 = key("key1")
k2 = key("key2")

p1 = payload("app1", 8 * 1024 + 3, 1)
p2 = payload("app2", 20 * 1024, 2)
pbad = payload("appvec", 4096, 3, reset_off=0x20001)
p3 = payload("app3", 56 * 1024 + 1, 4)

# A page of ones inside the payload, as a gap filled binary has.
p4 = payload("app4", 12 * 1024, 5)
d4 = bytearray(open(p4, "rb").read())
d4[4096:8192] = b"\xff" * 4096
open(p4, "wb").write(d4)

sign(p1, "v1.bin", "1.0.0", k1)
sign(p2, "v2.bin", "2.1.3+7", k1, hdr="0x200")
sign(p2, "v2_key2.bin", "2.1.3", k2)
sign(p2, "v2_nosig.bin", "2.1.3")
sign(pbad, "vec.bin", "1.0.1", k1)
# Across the 32 KB sector of the SECT memory kind.
sign(p3, "v3.bin", "3.0.0", k1)
sign(p4, "v4.bin", "4.0.0", k1)
# Security counter: a protected TLV, inside the hashed range.
sign(p2, "v2_prot.bin", "2.2.0", k1, extra=("-s", "5"))
# Signature padded to a fixed 72 bytes, zeros after the DER sequence.
for i in range(8):
    sign(p1, "v1_padsig%d.bin" % i, "1.0.%d" % i, k1, extra=("--pad-sig",))

# A payload byte changed after signing: hash no longer matches.
d = bytearray(open(os.path.join(out, "v1.bin"), "rb").read())
d[0x20 + 100] ^= 0x01
open(os.path.join(out, "v1_tamper.bin"), "wb").write(d)

# Signature bytes changed, hash intact.
d = bytearray(open(os.path.join(out, "v1.bin"), "rb").read())
d[-10] ^= 0x01
open(os.path.join(out, "v1_badsig.bin"), "wb").write(d)
