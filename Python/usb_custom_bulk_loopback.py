#!/usr/bin/env python3
"""USB custom bulk loopback example for IOsonata."""

import argparse
import sys
import time

import usb.core
import usb.util


DEFAULT_PRODUCT = "IOsonata Custom Bulk Loopback"


def parse_int(value):
    return int(value, 0)


def get_string(device, index):
    if index == 0:
        return ""

    try:
        return usb.util.get_string(device, index) or ""
    except (ValueError, usb.core.USBError):
        return "<unavailable>"


def device_identity(device):
    return {
        "manufacturer": get_string(device, device.iManufacturer),
        "product": get_string(device, device.iProduct),
        "serial": get_string(device, device.iSerialNumber),
    }


def print_candidate(device, identity, prefix=""):
    print(
        f"{prefix}{device.idVendor:04x}:{device.idProduct:04x}  "
        f"manufacturer={identity['manufacturer']!r}  "
        f"product={identity['product']!r}  "
        f"serial={identity['serial']!r}"
    )


def find_device(args):
    devices = list(
        usb.core.find(
            find_all=True,
            idVendor=args.vid,
            idProduct=args.pid,
        )
        or []
    )

    if not devices:
        raise RuntimeError(f"Device {args.vid:04x}:{args.pid:04x} not found")

    identities = [(device, device_identity(device)) for device in devices]
    matches = []

    for device, identity in identities:
        if args.product is not None and identity["product"] != args.product:
            continue
        if args.serial is not None and identity["serial"] != args.serial:
            continue
        matches.append((device, identity))

    if not matches:
        print("VID/PID candidates found:", file=sys.stderr)
        for device, identity in identities:
            print_candidate(device, identity, prefix="  ")
        raise RuntimeError("No device matched the requested product/serial identity")

    if len(matches) > 1:
        print("Multiple matching devices found:", file=sys.stderr)
        for device, identity in matches:
            print_candidate(device, identity, prefix="  ")
        raise RuntimeError("Specify --serial to select one device")

    return matches[0]


def ensure_configuration(device, reconfigure=False):
    if reconfigure:
        device.set_configuration()
        return

    try:
        device.get_active_configuration()
    except usb.core.USBError:
        device.set_configuration()


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
    parser.add_argument(
        "--product",
        default=DEFAULT_PRODUCT,
        help="expected USB product string",
    )
    parser.add_argument(
        "--serial",
        help="select one device by USB serial number",
    )
    parser.add_argument("--cycles", type=int, default=100)
    parser.add_argument("--timeout", type=int, default=1000, help="USB timeout in ms")
    parser.add_argument(
        "--reconfigure",
        action="store_true",
        help="force SET_CONFIGURATION even when the device is already configured",
    )
    args = parser.parse_args()

    try:
        device, identity = find_device(args)
    except RuntimeError as exc:
        print(f"Error: {exc}", file=sys.stderr)
        print("Result         : FAIL")
        return 1

    ensure_configuration(device, args.reconfigure)
    interface, ep_out, ep_in = find_custom_interface(device)
    interface_no = interface.bInterfaceNumber
    interface_name = get_string(device, interface.iInterface)

    detached = False
    try:
        try:
            if device.is_kernel_driver_active(interface_no):
                device.detach_kernel_driver(interface_no)
                detached = True
        except (NotImplementedError, usb.core.USBError):
            pass

        usb.util.claim_interface(device, interface_no)

        print(f"VID:PID        : {device.idVendor:04x}:{device.idProduct:04x}")
        print(f"Manufacturer   : {identity['manufacturer']}")
        print(f"Product        : {identity['product']}")
        print(f"Serial         : {identity['serial']}")
        print(f"Interface      : {interface_name} ({interface_no})")
        print(f"Bulk OUT       : 0x{ep_out.bEndpointAddress:02x}")
        print(f"Bulk IN        : 0x{ep_in.bEndpointAddress:02x}")

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
