#!/usr/bin/env python3
"""USB Mass Storage Bulk-Only Transport validation for IOsonata disks."""

import argparse
import hashlib
import struct
import sys

import usb.core
import usb.util


CBW_SIGNATURE = 0x43425355
CSW_SIGNATURE = 0x53425355


def find_msc(vid, pid):
    for dev in usb.core.find(find_all=True, idVendor=vid, idProduct=pid):
        for cfg in dev:
            for intf in cfg:
                if (intf.bInterfaceClass, intf.bInterfaceSubClass,
                        intf.bInterfaceProtocol) == (0x08, 0x06, 0x50):
                    return dev, cfg, intf
    return None, None, None


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--vid", type=lambda x: int(x, 0), default=0x1209)
    parser.add_argument("--pid", type=lambda x: int(x, 0), default=0x0004)
    parser.add_argument("--write-test", action="store_true",
                        help="write, verify and restore the last sectors")
    parser.add_argument("--write-blocks", type=int, default=16,
                        help="number of end-of-disk sectors to test (1-16; default 16)")
    parser.add_argument("--probe-only", action="store_true",
                        help="read-only INQUIRY, READ CAPACITY, and boot-sector check")
    parser.add_argument("--verbose", action="store_true",
                        help="print the current SCSI command and USB transfer stage")
    args = parser.parse_args()
    if args.write_blocks < 1 or args.write_blocks > 16:
        parser.error("--write-blocks must be between 1 and 16")
    if args.probe_only and args.write_test:
        parser.error("--probe-only and --write-test cannot be used together")

    dev, cfg, intf = find_msc(args.vid, args.pid)
    if dev is None:
        print(f"FAIL: MSC {args.vid:04x}:{args.pid:04x} not found")
        return 1

    detached = False
    try:
        if dev.is_kernel_driver_active(intf.bInterfaceNumber):
            dev.detach_kernel_driver(intf.bInterfaceNumber)
            detached = True
    except NotImplementedError:
        pass
    active = dev.get_active_configuration()
    if active.bConfigurationValue != cfg.bConfigurationValue:
        dev.set_configuration(cfg.bConfigurationValue)
    usb.util.claim_interface(dev, intf.bInterfaceNumber)
    ep_out = usb.util.find_descriptor(
        intf, custom_match=lambda ep: usb.util.endpoint_direction(
            ep.bEndpointAddress) == usb.util.ENDPOINT_OUT)
    ep_in = usb.util.find_descriptor(
        intf, custom_match=lambda ep: usb.util.endpoint_direction(
            ep.bEndpointAddress) == usb.util.ENDPOINT_IN)
    if ep_out is None or ep_in is None:
        print("FAIL: bulk endpoint pair not found")
        usb.util.release_interface(dev, intf.bInterfaceNumber)
        if detached:
            dev.attach_kernel_driver(intf.bInterfaceNumber)
        usb.util.dispose_resources(dev)
        return 1

    tag = 0

    scsi_names = {
        0x00: "TEST UNIT READY", 0x03: "REQUEST SENSE",
        0x12: "INQUIRY", 0x1B: "START STOP UNIT",
        0x1E: "PREVENT ALLOW MEDIUM REMOVAL",
        0x25: "READ CAPACITY (10)", 0x28: "READ (10)",
        0x2A: "WRITE (10)",
    }

    def command(cdb, data_len=0, direction_in=False, data_out=None,
                expected_status=0, expected_residue=0):
        nonlocal tag
        tag += 1
        cdb = bytes(cdb)
        name = scsi_names.get(cdb[0], f"SCSI 0x{cdb[0]:02x}")
        operation = f"{name} (tag {tag})"
        phase = "sending CBW"
        if args.verbose:
            print(f"{operation}: {data_len} bytes, "
                  f"{'IN' if direction_in else 'OUT'}", flush=True)
        try:
            cbw = struct.pack("<IIIBBB16s", CBW_SIGNATURE, tag, data_len,
                              0x80 if direction_in else 0, 0, len(cdb),
                              cdb.ljust(16, b"\0"))
            if ep_out.write(cbw, timeout=2000) != 31:
                raise RuntimeError("short CBW write")
            payload = b""
            if data_out is not None:
                phase = f"sending {len(data_out)} data bytes"
                if ep_out.write(data_out, timeout=5000) != len(data_out):
                    raise RuntimeError("short data write")
            elif data_len:
                phase = f"reading {data_len} data bytes"
                payload = bytes(ep_in.read(data_len, timeout=5000))
                if len(payload) != data_len:
                    raise RuntimeError(
                        f"short data read: {len(payload)}/{data_len} bytes")
            phase = "reading CSW"
            csw = bytes(ep_in.read(13, timeout=2000))
            if len(csw) != 13:
                raise RuntimeError(f"short CSW: {len(csw)}/13 bytes")
            signature, csw_tag, residue, status = struct.unpack("<IIIB", csw)
            if signature != CSW_SIGNATURE or csw_tag != tag:
                raise RuntimeError("invalid CSW")
            if status != expected_status or residue != expected_residue:
                raise RuntimeError(
                    f"command failed: status={status} residue={residue}")
            return payload
        except (usb.core.USBError, RuntimeError, ValueError) as exc:
            raise RuntimeError(f"{operation}, {phase}: {exc}") from exc

    try:
        def rw_cdb(opcode, lba, blocks):
            return [opcode, 0, (lba >> 24) & 0xFF, (lba >> 16) & 0xFF,
                    (lba >> 8) & 0xFF, lba & 0xFF, 0,
                    (blocks >> 8) & 0xFF, blocks & 0xFF, 0]

        inquiry = command([0x12, 0, 0, 0, 36, 0], 36, True)
        capacity = command([0x25] + [0] * 9, 8, True)
        last_lba, sector_size = struct.unpack(">II", capacity)
        boot = command([0x28, 0, 0, 0, 0, 0, 0, 0, 1, 0],
                       sector_size, True)
        if boot[-2:] != b"\x55\xaa":
            raise RuntimeError("boot-sector signature mismatch")
        if args.probe_only:
            print("Vendor/Product :", inquiry[8:32].decode("ascii").rstrip())
            print("Capacity       :", (last_lba + 1) * sector_size, "bytes")
            print("Sector size    :", sector_size)
            print("Boot read      : PASS")
            print("Result         : PASS (read-only probe)")
            return 0
        if args.write_test:
            blocks = min(args.write_blocks, last_lba + 1)
            lba = last_lba + 1 - blocks
            length = blocks * sector_size
            read_cdb = rw_cdb(0x28, lba, blocks)
            write_cdb = rw_cdb(0x2A, lba, blocks)
            original = command(read_cdb, length, True)
            pattern = bytes((n * 29 + 7) & 0xFF for n in range(length))
            command(write_cdb, length, False, pattern)
            readback = command(read_cdb, length, True)
            if hashlib.sha256(readback).digest() != hashlib.sha256(pattern).digest():
                raise RuntimeError("multi-sector write/read hash mismatch")
            command(write_cdb, length, False, original)
            restored = command(read_cdb, length, True)
            if hashlib.sha256(restored).digest() != hashlib.sha256(original).digest():
                raise RuntimeError("multi-sector restore hash mismatch")
        if args.verbose:
            print("Bulk-Only Reset: class control request", flush=True)
        try:
            dev.ctrl_transfer(0x21, 0xFF, 0, intf.bInterfaceNumber, None,
                              timeout=2000)
        except usb.core.USBError as exc:
            raise RuntimeError(f"Bulk-Only Reset control request: {exc}") from exc
        if args.verbose:
            print("Bulk-Only Reset: clear endpoint halts", flush=True)
        try:
            ep_in.clear_halt()
            ep_out.clear_halt()
        except usb.core.USBError as exc:
            raise RuntimeError(f"Bulk-Only Reset clear halts: {exc}") from exc
        command([0x00, 0, 0, 0, 0, 0])

        command([0x1E, 0, 0, 0, 1, 0])
        command([0x1B, 0, 0, 0, 2, 0], expected_status=1)
        sense = command([0x03, 0, 0, 0, 18, 0], 18, True)
        if (sense[2], sense[12], sense[13]) != (0x05, 0x53, 0x02):
            raise RuntimeError("prevented-removal sense mismatch")
        command([0x1E, 0, 0, 0, 0, 0])
        command([0x1B, 0, 0, 0, 2, 0])
        command([0x00, 0, 0, 0, 0, 0], expected_status=1)
        sense = command([0x03, 0, 0, 0, 18, 0], 18, True)
        if (sense[2], sense[12]) != (0x02, 0x3A):
            raise RuntimeError("ejected-medium sense mismatch")
        command([0x1B, 0, 0, 0, 3, 0])
        command([0x00, 0, 0, 0, 0, 0])
        for _ in range(50):
            command([0x00, 0, 0, 0, 0, 0])
        print("Vendor/Product :", inquiry[8:32].decode("ascii").rstrip())
        print("Capacity       :", (last_lba + 1) * sector_size, "bytes")
        print("Sector size    :", sector_size)
        print("Write test     :", "PASS" if args.write_test else "skipped")
        print("Eject/reload   : PASS")
        print("Repeated TUR   : PASS (50)")
        print("Result         : PASS")
        return 0
    except (usb.core.USBError, RuntimeError, ValueError) as exc:
        print("FAIL:", exc)
        return 1
    finally:
        usb.util.release_interface(dev, intf.bInterfaceNumber)
        if detached:
            dev.attach_kernel_driver(intf.bInterfaceNumber)
        usb.util.dispose_resources(dev)


if __name__ == "__main__":
    sys.exit(main())
