#!/usr/bin/env python3
"""USB custom bulk loopback example for IOsonata."""

import argparse
import sys
import time

import usb.core
import usb.util


def parse_int(value):
    return int(value, 0)


def find_custom_interface(device):
    config = device.get_active_configuration()

    for interface in config:
        if interface.bInterfaceClass != 0xFF:
            continue

        ep_out = None
        ep_in = None
        for endpoint in interface:
            if usb.util.endpoint_type(endpoint.bmAttributes) != usb.util.ENDPOINT_TYPE_BULK:
                continue

            if usb.util.endpoint_direction(endpoint.bEndpointAddress) == usb.util.ENDPOINT_OUT:
                ep_out = endpoint
            else:
                ep_in = endpoint

        if ep_out is not None and ep_in is not None:
            return interface, ep_out, ep_in

    raise RuntimeError("No custom bulk interface with IN and OUT endpoints found")


def read_exact(endpoint, length, timeout_ms):
    received = bytearray()

    while len(received) < length:
        data = endpoint.read(length - len(received), timeout=timeout_ms)
        received.extend(data)

    return bytes(received)


def main():
    parser = argparse.ArgumentParser(
        description="Exercise the IOsonata UsbCustomBulkLoopback example"
    )
    parser.add_argument("--vid", type=parse_int, default=0x1209)
    parser.add_argument("--pid", type=parse_int, default=0x0002)
    parser.add_argument("--cycles", type=int, default=100)
    parser.add_argument("--timeout", type=int, default=1000, help="USB timeout in ms")
    args = parser.parse_args()

    device = usb.core.find(idVendor=args.vid, idProduct=args.pid)
    if device is None:
        print(f"Device {args.vid:04x}:{args.pid:04x} not found", file=sys.stderr)
        return 1

    device.set_configuration()
    interface, ep_out, ep_in = find_custom_interface(device)
    interface_no = interface.bInterfaceNumber

    detached = False
    try:
        try:
            if device.is_kernel_driver_active(interface_no):
                device.detach_kernel_driver(interface_no)
                detached = True
        except (NotImplementedError, usb.core.USBError):
            pass

        usb.util.claim_interface(device, interface_no)

        print(
            f"Custom interface {interface_no}, "
            f"OUT 0x{ep_out.bEndpointAddress:02x}, "
            f"IN 0x{ep_in.bEndpointAddress:02x}"
        )

        sizes = (1, 7, 63, 64, 65, 127, 128, 129, 512, 1024)
        total = 0
        start = time.monotonic()

        for cycle in range(args.cycles):
            for size in sizes:
                payload = bytes(((cycle + i) & 0xFF) for i in range(size))
                written = ep_out.write(payload, timeout=args.timeout)
                if written != len(payload):
                    raise RuntimeError(
                        f"Short write: {written} of {len(payload)} bytes"
                    )

                echoed = read_exact(ep_in, len(payload), args.timeout)
                if echoed != payload:
                    raise RuntimeError(
                        f"Data mismatch at cycle {cycle}, size {size}"
                    )
                total += len(payload)

        elapsed = time.monotonic() - start
        rate = total / elapsed if elapsed > 0 else 0.0
        print(f"Bytes          : {total}")
        print(f"Bytes/sec      : {rate:.2f}")
        print("Result         : PASS")
        return 0

    except (usb.core.USBError, RuntimeError) as exc:
        print(f"Error: {exc}", file=sys.stderr)
        print("Result         : FAIL")
        return 1

    finally:
        try:
            usb.util.release_interface(device, interface_no)
        except usb.core.USBError:
            pass

        if detached:
            try:
                device.attach_kernel_driver(interface_no)
            except (NotImplementedError, usb.core.USBError):
                pass

        usb.util.dispose_resources(device)


if __name__ == "__main__":
    raise SystemExit(main())
