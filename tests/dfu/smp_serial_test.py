#!/usr/bin/env python3
# End to end: stage 0 recovery (dfu_serial_host) driven by smpclient over its
# serial transport, the framing mcumgr and smpmgr use on a UART or USB CDC
# port. Upload straight to slot 0, refused images, reset: the boot starts
# the image, the simulated application asks for recovery, and the client
# finds the boot again with the image in place.
#
# Usage: smp_serial_test.py <dfu_serial_host> <image dir> [nor|rram|ecc16|big256]

import asyncio
import subprocess
import sys

from smpclient import SMPClient
from smpclient.transport.serial import SMPSerialTransport
from smpclient.requests.image_management import ImageStatesRead, ImageErase
from smpclient.requests.os_management import (EchoWrite, ResetWrite,
                                              MCUMgrParametersRead,
                                              BootloaderInformationRead)
from smpclient.generics import success, error

HOST, IMG = sys.argv[1], sys.argv[2]
KIND = sys.argv[3] if len(sys.argv) > 3 else "nor"
failures = 0


def check(cond, what):
    global failures
    print(("  ok    " if cond else "  FAIL  ") + what)
    if not cond:
        failures += 1


def img(name):
    return open(IMG + "/" + name, "rb").read()


def tlv_sha(data):
    hdr_size = int.from_bytes(data[8:10], "little")
    prot = int.from_bytes(data[10:12], "little")
    size = int.from_bytes(data[12:16], "little")
    off = hdr_size + size + prot
    end = off + int.from_bytes(data[off + 2:off + 4], "little")
    off += 4
    while off < end:
        t = int.from_bytes(data[off:off + 2], "little")
        n = int.from_bytes(data[off + 2:off + 4], "little")
        if t == 0x10:
            return data[off + 4:off + 4 + n]
        off += 4 + n
    raise ValueError("no hash")


async def upload(c, data):
    last = 0
    async for off in c.upload(data):
        last = off
    return last


async def refused(c, name):
    try:
        await upload(c, img(name))
    except Exception as e:
        print("         client saw: %s" % type(e).__name__)
        return True
    return False


async def main(port, srv):
    v1, v2 = img("v1.bin"), img("v2.bin")

    async with SMPClient(SMPSerialTransport(), port) as c:
        r = await c.request(EchoWrite(d="IOsonata"))
        check(success(r) and r.r == "IOsonata", "echo over serial")

        r = await c.request(MCUMgrParametersRead())
        check(success(r) and r.buf_size == 1024, "parameters")

        r = await c.request(BootloaderInformationRead())
        check(success(r) and r.bootloader == "MCUboot", "bootloader name")

        r = await c.request(ImageStatesRead())
        check(success(r) and len(r.images) == 1 and not r.images[0].active,
              "nothing in slot 0")

        n = await upload(c, v1)
        check(n == len(v1), "upload v1 straight to slot 0, %d bytes" % n)
        r = await c.request(ImageStatesRead())
        check(success(r) and r.images[0].hash == tlv_sha(v1), "v1 in slot 0")

        for bad in ("v1_badsig.bin", "v1_tamper.bin", "vec.bin"):
            check(await refused(c, bad), "%s refused" % bad)
        r = await c.request(ImageStatesRead())
        check(success(r) and len(r.images) == 1 and
              r.images[0].version == "0.0.0", "refused upload leaves nothing")

        n = await upload(c, v2)
        check(n == len(v2), "upload v2, %d bytes" % n)

        r = await c.request(ResetWrite())
        check(success(r), "reset accepted")
        line = srv.stdout.readline().strip()
        print("         " + line)
        check("started slot 0" in line and "violations 0" in line,
              "boot started v2")
        line = srv.stdout.readline().strip()
        print("         " + line)
        check("recovery (0)" in line, "application asked for recovery")

        r = await c.request(ImageStatesRead())
        check(success(r) and r.images[0].hash == tlv_sha(v2) and
              r.images[0].version == "2.1.3.7", "v2 in place after reset")

        r = await c.request(ImageErase())
        check(success(r), "erase")
        r = await c.request(ResetWrite())
        check(success(r), "reset after erase")
        line = srv.stdout.readline().strip()
        print("         " + line)
        check("recovery (-2)" in line, "nothing to start after erase")


srv = subprocess.Popen([HOST, IMG, KIND], stdout=subprocess.PIPE, text=True)
first = srv.stdout.readline().split()
print(" ".join(first))
srv.stdout.readline()          # boot 1: recovery
try:
    asyncio.run(asyncio.wait_for(main(first[0], srv), 120))
finally:
    srv.terminate()
    srv.communicate(timeout=5)

print("smp_serial_test %s: %s" % (KIND, "PASS" if failures == 0 else
                                  "%d FAILED" % failures))
sys.exit(1 if failures else 0)
