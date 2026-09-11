#!/usr/bin/env python3
"""Hardware loopback test for the IOsonata UsbdHid example."""

import argparse
import sys
import time

try:
    import hid
except ImportError:
    hid = None


DEFAULT_VID = 0x1209
DEFAULT_PID = 0x0005
DEFAULT_PRODUCT = "IOsonata USB HID Loopback"
REPORT_SIZE = 64


def parse_int(value):
    return int(value, 0)


def text(value):
    if isinstance(value, bytes):
        return value.decode("utf-8", errors="replace")
    return value or ""


def find_device(args):
    matches = []
    for info in hid.enumerate(args.vid, args.pid):
        if args.product is not None and text(info.get("product_string")) != args.product:
            continue
        if args.serial is not None and text(info.get("serial_number")) != args.serial:
            continue
        matches.append(info)
    if not matches:
        raise RuntimeError(f"Device {args.vid:04x}:{args.pid:04x} not found")
    if len(matches) != 1:
        raise RuntimeError("Multiple HID interfaces matched; use --serial")
    return matches[0]


def payload(sequence):
    data = bytearray(
        ((0xA7 ^ (sequence * 23) ^ (index * 31)) & 0xFF)
        for index in range(REPORT_SIZE)
    )
    data[0:4] = sequence.to_bytes(4, "little")
    return bytes(data)


def exchange(device, data, timeout_ms):
    written = device.write(bytes((0,)) + data)
    if written not in (len(data), len(data) + 1):
        raise RuntimeError(f"Short HID write: {written}")
    reply = bytes(device.read(REPORT_SIZE, timeout_ms))
    if reply != data:
        raise RuntimeError(
            f"Loopback mismatch: sent {data.hex()}, received {reply.hex()}"
        )


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--vid", type=parse_int, default=DEFAULT_VID)
    parser.add_argument("--pid", type=parse_int, default=DEFAULT_PID)
    parser.add_argument("--product", default=DEFAULT_PRODUCT)
    parser.add_argument("--serial")
    parser.add_argument("--rounds", type=int, default=160)
    parser.add_argument("--timeout-ms", type=int, default=1000)
    parser.add_argument("--manual-suspend-wake", action="store_true")
    args = parser.parse_args()

    if hid is None:
        raise RuntimeError("Python hidapi package is required: pip install hidapi")

    info = find_device(args)
    device = hid.device()
    device.open_path(info["path"])
    device.set_nonblocking(0)
    try:
        print(f"USB HID loopback {args.vid:04x}:{args.pid:04x}")
        print(f"Product        : {text(info.get('product_string'))}")
        print(f"Serial         : {text(info.get('serial_number'))}")
        for sequence in range(args.rounds):
            exchange(device, payload(sequence), args.timeout_ms)
        print(f"PASS: {args.rounds} output/input report loopbacks")

        if args.manual_suspend_wake:
            input("Suspend the host/system, wake it, then press Enter to continue.\n")
            time.sleep(0.25)
            exchange(device, payload(args.rounds), args.timeout_ms)
            print("PASS suspend/wake: HID reports resumed on the existing handle")
        else:
            print("Suspend/wake not run; use --manual-suspend-wake for that phase.")
        print("USB HID hardware test: PASS")
        print("Result         : PASS")
        return 0
    finally:
        device.close()


if __name__ == "__main__":
    try:
        sys.exit(main())
    except Exception as error:
        print(f"Error: {error}")
        print("Result         : FAIL")
        sys.exit(1)
