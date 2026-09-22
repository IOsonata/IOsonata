#!/usr/bin/env python3
# End to end: the IOsonata SMP server (dfu_smp_udp_host) driven by smpclient,
# the Python SMP client library, over UDP. Upload, test, reset into the
# stage 0 boot, check the image list; then the same with confirm, and the
# error answers a client sees for a bad image.
#
# Usage: smp_udp_test.py <dfu_smp_udp_host> <image dir> [nor|rram]

import asyncio
import subprocess
import sys

from smpclient import SMPClient
from smpclient.transport.udp import SMPUDPTransport
from smpclient.requests.image_management import (ImageStatesRead,
                                                 ImageStatesWrite,
                                                 ImageErase)
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
    # The SHA-256 TLV value, what the image list reports as the hash.
    hdr_size = int.from_bytes(data[8:10], "little")
    prot = int.from_bytes(data[10:12], "little")
    size = int.from_bytes(data[12:16], "little")
    off = hdr_size + size + prot
    assert int.from_bytes(data[off:off + 2], "little") == 0x6907
    end = off + int.from_bytes(data[off + 2:off + 4], "little")
    off += 4
    while off < end:
        t = int.from_bytes(data[off:off + 2], "little")
        n = int.from_bytes(data[off + 2:off + 4], "little")
        if t == 0x10:
            return data[off + 4:off + 4 + n]
        off += 4 + n
    raise ValueError("no hash")


async def images(c):
    r = await c.request(ImageStatesRead())
    check(success(r), "image list")
    return r.images


async def upload(c, data):
    last = 0
    async for off in c.upload(data):
        last = off
    return last


async def reset(c):
    r = await c.request(ResetWrite())
    check(success(r), "reset accepted")
    await asyncio.sleep(0.3)


async def main(port):
    v1, v2 = img("v1.bin"), img("v2.bin")

    async with SMPClient(SMPUDPTransport(), "127.0.0.1") as c:
        r = await c.request(EchoWrite(d="IOsonata"))
        check(success(r) and r.r == "IOsonata", "echo")

        r = await c.request(MCUMgrParametersRead())
        check(success(r) and r.buf_size == 1024, "parameters")

        r = await c.request(BootloaderInformationRead())
        check(success(r) and r.bootloader == "MCUboot", "bootloader name")

        im = await images(c)
        check(len(im) == 1 and im[0].slot == 0, "empty device lists slot 0")

        # Upload, test, reset: installed.
        n = await upload(c, v1)
        check(n == len(v1), "upload v1, %d bytes" % n)
        im = await images(c)
        check(len(im) == 2 and im[1].hash == tlv_sha(v1) and not im[1].pending,
              "slot 1 holds v1")
        r = await c.request(ImageStatesWrite(hash=tlv_sha(v1), confirm=False))
        check(success(r) and r.images[1].pending, "v1 set for test")
        await reset(c)
        im = await images(c)
        check(len(im) == 1 and im[0].hash == tlv_sha(v1) and im[0].version ==
              "1.0.0" and im[0].active and im[0].confirmed,
              "v1 runs after reset")

        # Confirm the running image, as a client does after a test boot.
        r = await c.request(ImageStatesWrite(confirm=True))
        check(success(r), "confirm running image")

        # Upload v2, confirm directly, reset.
        n = await upload(c, v2)
        check(n == len(v2), "upload v2, %d bytes" % n)
        r = await c.request(ImageStatesWrite(hash=tlv_sha(v2), confirm=True))
        check(success(r), "v2 confirmed")
        await reset(c)
        im = await images(c)
        check(len(im) == 1 and im[0].hash == tlv_sha(v2) and
              im[0].version == "2.1.3.7", "v2 runs after reset")

        # Unknown hash.
        r = await c.request(ImageStatesWrite(hash=bytes(32), confirm=False))
        check(error(r), "unknown hash refused: %s" % (r,))

        # A tampered image: the upload itself reports it.
        bad = False
        try:
            await upload(c, img("v1_tamper.bin"))
        except Exception as e:
            bad = True
            print("         client saw: %s" % type(e).__name__)
        check(bad, "tampered image refused at end of upload")

        r = await c.request(ImageErase())
        check(success(r), "erase slot 1")
        im = await images(c)
        check(len(im) == 1, "slot 1 empty after erase")


port = 1337
srv = subprocess.Popen([HOST, IMG, str(port), KIND], stdout=subprocess.PIPE,
                       text=True)
line = srv.stdout.readline()
print(line.strip())
try:
    asyncio.run(main(port))
finally:
    srv.terminate()
    out = srv.communicate(timeout=5)[0]
    print(out.strip())
    check("nor violations 0" in out and "no application" not in out,
          "server memory log clean")

print("smp_udp_test: %s" % ("PASS" if failures == 0 else
                            "%d FAILED" % failures))
sys.exit(1 if failures else 0)
