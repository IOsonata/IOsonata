#!/usr/bin/env python3
"""USB interrupt loopback hardware test for IOsonata UsbIntIntrf."""

import argparse
import struct
import sys
import time

try:
    import usb.core
    import usb.util
except ImportError:
    usb = None


DEFAULT_VID = 0x1209
DEFAULT_PID = 0x0004
DEFAULT_PRODUCT = "IOsonata USB Interrupt Loopback"
DEFAULT_ROUNDS = 32
DEFAULT_TIMEOUT_MS = 1000

MPS = 64
INTERVAL_BY_ALT = (1, 4, 16)
DIAG_REQUEST = 0x5B
DIAG_FORMAT = "<10I3H4B"
DIAG_SIZE = struct.calcsize(DIAG_FORMAT)

DIAG_FLAG_OPENED = 1 << 0
DIAG_FLAG_SUSPENDED = 1 << 1
DIAG_FLAG_TX_READY = 1 << 2


def parse_int(value):
    return int(value, 0)


def get_string(device, index):
    if index == 0:
        return ""
    try:
        return usb.util.get_string(device, index) or ""
    except (ValueError, usb.core.USBError):
        return "<unavailable>"


def find_device(args):
    devices = list(
        usb.core.find(find_all=True, idVendor=args.vid, idProduct=args.pid) or []
    )
    if not devices:
        raise RuntimeError(f"Device {args.vid:04x}:{args.pid:04x} not found")

    matches = []
    for device in devices:
        product = get_string(device, device.iProduct)
        serial = get_string(device, device.iSerialNumber)
        if args.product is not None and product != args.product:
            continue
        if args.serial is not None and serial != args.serial:
            continue
        matches.append((device, product, serial))

    if not matches:
        raise RuntimeError("No device matched the requested product/serial identity")
    if len(matches) != 1:
        raise RuntimeError("Multiple devices matched; use --serial to select one")
    return matches[0]


def ensure_configuration(device, reconfigure=False):
    if reconfigure:
        device.set_configuration(1)
        return
    try:
        device.get_active_configuration()
    except usb.core.USBError:
        device.set_configuration(1)


def discover_interrupt_loopback(device, interface_override=None, ep_override=None):
    config = device.get_active_configuration()
    candidates = {}

    for setting in config:
        interface = setting.bInterfaceNumber
        alt = setting.bAlternateSetting
        if interface_override is not None and interface != interface_override:
            continue
        if alt < 1 or alt > len(INTERVAL_BY_ALT):
            continue
        if setting.bInterfaceClass != 0xFF:
            continue

        ep_in = None
        ep_out = None
        for endpoint in setting:
            if usb.util.endpoint_type(endpoint.bmAttributes) != usb.util.ENDPOINT_TYPE_INTR:
                continue
            ep_no = endpoint.bEndpointAddress & 0x0F
            if ep_override is not None and ep_no != ep_override:
                continue
            if usb.util.endpoint_direction(endpoint.bEndpointAddress) == usb.util.ENDPOINT_IN:
                ep_in = endpoint
            else:
                ep_out = endpoint

        if ep_in is None or ep_out is None:
            continue
        in_no = ep_in.bEndpointAddress & 0x0F
        out_no = ep_out.bEndpointAddress & 0x0F
        expected_interval = INTERVAL_BY_ALT[alt - 1]
        if (
            in_no == 0
            or in_no != out_no
            or ep_in.wMaxPacketSize != MPS
            or ep_out.wMaxPacketSize != MPS
            or ep_in.bInterval != expected_interval
            or ep_out.bInterval != expected_interval
        ):
            continue
        candidates.setdefault((interface, in_no), set()).add(alt)

    expected_alts = set(range(1, len(INTERVAL_BY_ALT) + 1))
    matches = [key for key, alts in candidates.items() if alts == expected_alts]
    if not matches:
        raise RuntimeError(
            "No IOsonata interrupt loopback interface matching alt1..alt3"
        )
    if len(matches) != 1:
        raise RuntimeError("Multiple interrupt loopback interfaces matched")
    return matches[0]


def read_diag(device, interface, timeout_ms):
    raw = bytes(
        device.ctrl_transfer(
            0xC1,
            DIAG_REQUEST,
            0,
            interface,
            DIAG_SIZE,
            timeout=timeout_ms,
        )
    )
    if len(raw) != DIAG_SIZE:
        raise RuntimeError(f"Diagnostic reply is {len(raw)} bytes, expected {DIAG_SIZE}")

    values = struct.unpack(DIAG_FORMAT, raw)
    names = (
        "rx",
        "tx_submit",
        "tx_done",
        "tx_fail",
        "loopback_drop",
        "core_rx_drop",
        "rx_error",
        "tx_error",
        "rx_empty",
        "tx_empty",
        "last_rx_length",
        "last_tx_length",
        "mps",
        "interval",
        "alt",
        "flags",
        "reserved",
    )
    return dict(zip(names, values))


def set_alt(device, interface, alt):
    device.set_interface_altsetting(interface=interface, alternate_setting=alt)


def endpoint_pair(device, interface, alt):
    setting = device.get_active_configuration()[(interface, alt)]
    ep_in = None
    ep_out = None
    for endpoint in setting:
        if usb.util.endpoint_direction(endpoint.bEndpointAddress) == usb.util.ENDPOINT_IN:
            ep_in = endpoint
        else:
            ep_out = endpoint
    if ep_in is None or ep_out is None:
        raise RuntimeError(f"Alternate setting {alt} has no endpoint pair")
    return ep_out, ep_in


def payload(alt, length, sequence):
    data = bytearray(
        ((0xA7 ^ (alt * 19) ^ (sequence * 23) ^ (index * 31)) & 0xFF)
        for index in range(length)
    )
    if length >= 4:
        data[0] = alt
        data[1] = sequence & 0xFF
        data[2] = (sequence >> 8) & 0xFF
        data[3] = length
    return bytes(data)


def exchange(ep_out, ep_in, data, timeout_ms):
    written = ep_out.write(data, timeout=timeout_ms)
    if written != len(data):
        raise RuntimeError(f"Short OUT transfer: {written} of {len(data)} bytes")
    echoed = bytes(ep_in.read(MPS, timeout=timeout_ms))
    if echoed != data:
        raise RuntimeError(
            f"Loopback mismatch: sent {data.hex()}, received {echoed.hex()}"
        )


def wait_tx_ready(device, interface, timeout_ms):
    deadline = time.monotonic() + (timeout_ms / 1000.0)
    while time.monotonic() < deadline:
        diag = read_diag(device, interface, timeout_ms)
        if diag["flags"] & DIAG_FLAG_TX_READY:
            return diag
        time.sleep(0.001)
    raise RuntimeError("TX slot did not return to ready state")


def run_busy_transition(device, interface, ep_out, ep_in, alt, timeout_ms):
    first = payload(alt, MPS, 0x100)
    second = payload(alt, MPS, 0x101)
    before = read_diag(device, interface, timeout_ms)

    written = ep_out.write(first, timeout=timeout_ms)
    if written != len(first):
        raise RuntimeError(f"Busy test short first write: {written}")
    busy = read_diag(device, interface, timeout_ms)
    if busy["flags"] & DIAG_FLAG_TX_READY:
        raise RuntimeError("TX slot did not become busy while IN packet was pending")

    written = ep_out.write(second, timeout=timeout_ms)
    if written != len(second):
        raise RuntimeError(f"Busy test short second write: {written}")
    echoed = bytes(ep_in.read(MPS, timeout=timeout_ms))
    if echoed != first:
        raise RuntimeError("Busy test corrupted or reordered the pending IN packet")

    after = wait_tx_ready(device, interface, timeout_ms)
    if after["loopback_drop"] <= before["loopback_drop"]:
        raise RuntimeError("Busy TX slot did not reject the second loopback packet")

    recovery = payload(alt, 17, 0x102)
    exchange(ep_out, ep_in, recovery, timeout_ms)


def validate_diag(diag, alt, interval):
    if not (diag["flags"] & DIAG_FLAG_OPENED):
        raise RuntimeError(f"Alternate setting {alt} did not open UsbIntIntrf")
    if diag["flags"] & DIAG_FLAG_SUSPENDED:
        raise RuntimeError("UsbIntIntrf unexpectedly reports suspended")
    if diag["mps"] != MPS or diag["interval"] != interval or diag["alt"] != alt:
        raise RuntimeError(
            f"Open state mismatch: MPS={diag['mps']} interval={diag['interval']} "
            f"alt={diag['alt']}"
        )


def run_alt(device, interface, alt, interval, rounds, timeout_ms):
    set_alt(device, interface, 0)
    closed = read_diag(device, interface, timeout_ms)
    if closed["flags"] & DIAG_FLAG_OPENED:
        raise RuntimeError("Alternate setting 0 did not close UsbIntIntrf")

    set_alt(device, interface, alt)
    ep_out, ep_in = endpoint_pair(device, interface, alt)
    diag = read_diag(device, interface, timeout_ms)
    validate_diag(diag, alt, interval)

    run_busy_transition(device, interface, ep_out, ep_in, alt, timeout_ms)

    lengths = (0, 1, 7, MPS - 1, MPS)
    for sequence in range(rounds):
        for length in lengths:
            exchange(ep_out, ep_in, payload(alt, length, sequence), timeout_ms)

    diag = wait_tx_ready(device, interface, timeout_ms)
    expected = rounds * len(lengths)
    if diag["tx_fail"] != 0 or diag["rx_error"] != 0 or diag["tx_error"] != 0:
        raise RuntimeError(
            f"Transfer errors: tx_fail={diag['tx_fail']} "
            f"rx_error={diag['rx_error']} tx_error={diag['tx_error']}"
        )
    if diag["core_rx_drop"] != 0:
        raise RuntimeError(f"UsbIntrf dropped {diag['core_rx_drop']} RX packet(s)")
    if (
        diag["rx"] != expected + 3
        or diag["tx_submit"] != expected + 2
        or diag["tx_done"] != expected + 2
        or diag["loopback_drop"] != 1
    ):
        raise RuntimeError(
            f"Unexpected transfer counts: rx={diag['rx']} "
            f"tx_submit={diag['tx_submit']} tx_done={diag['tx_done']} "
            f"loopback_drop={diag['loopback_drop']}"
        )
    if diag["rx_empty"] < rounds or diag["tx_empty"] < rounds:
        raise RuntimeError("Zero-length interrupt packets were not completed")

    print(
        f"PASS alt {alt}: interval={interval}, {expected} loopbacks, "
        "ZLP, TX busy/recovery, no transfer errors"
    )


def manual_suspend_wake(device, interface, alt, timeout_ms):
    set_alt(device, interface, alt)
    ep_out, ep_in = endpoint_pair(device, interface, alt)
    print()
    input("Suspend the host/system, wake it, then press Enter to continue.")
    exchange(ep_out, ep_in, payload(alt, MPS, 0x7000), timeout_ms)
    validate_diag(read_diag(device, interface, timeout_ms), alt, INTERVAL_BY_ALT[alt - 1])
    print("PASS suspend/wake: interrupt endpoint pair resumed on the existing handle")


def main():
    parser = argparse.ArgumentParser(
        description="Exercise the IOsonata generic UsbIntIntrf loopback example"
    )
    parser.add_argument("--vid", type=parse_int, default=DEFAULT_VID)
    parser.add_argument("--pid", type=parse_int, default=DEFAULT_PID)
    parser.add_argument("--product", default=DEFAULT_PRODUCT)
    parser.add_argument("--serial")
    parser.add_argument("--interface", type=int)
    parser.add_argument("--ep", type=int)
    parser.add_argument("--rounds", type=int, default=DEFAULT_ROUNDS)
    parser.add_argument("--timeout", type=int, default=DEFAULT_TIMEOUT_MS)
    parser.add_argument("--reconfigure", action="store_true")
    parser.add_argument("--manual-suspend-wake", action="store_true")
    args = parser.parse_args()

    if usb is None:
        print("Error: PyUSB is required. Install it with:", file=sys.stderr)
        print("  python3 -m pip install pyusb", file=sys.stderr)
        print("Result         : FAIL")
        return 2
    if args.interface is not None and not 0 <= args.interface <= 255:
        parser.error("--interface must be 0..255")
    if args.ep is not None and not 1 <= args.ep <= 15:
        parser.error("--ep must be 1..15")
    if not 1 <= args.rounds <= 1000:
        parser.error("--rounds must be 1..1000")
    if args.timeout <= 0:
        parser.error("--timeout must be positive")

    device = None
    interface = None
    detached = False
    try:
        device, product, serial = find_device(args)
        ensure_configuration(device, args.reconfigure)
        interface, ep = discover_interrupt_loopback(
            device, args.interface, args.ep
        )

        try:
            if device.is_kernel_driver_active(interface):
                device.detach_kernel_driver(interface)
                detached = True
        except (NotImplementedError, usb.core.USBError):
            pass

        usb.util.claim_interface(device, interface)
        print(f"USB interrupt loopback {args.vid:04x}:{args.pid:04x}")
        print(f"Product        : {product}")
        print(f"Serial         : {serial}")
        print(f"Interface      : {interface}")
        print(f"Endpoint pair  : EP{ep}")

        for alt, interval in enumerate(INTERVAL_BY_ALT, start=1):
            run_alt(
                device,
                interface,
                alt,
                interval,
                args.rounds,
                args.timeout,
            )

        if args.manual_suspend_wake:
            manual_suspend_wake(
                device, interface, len(INTERVAL_BY_ALT), args.timeout
            )
        else:
            print(
                "Suspend/wake not run; use --manual-suspend-wake for that "
                "hardware phase."
            )

        set_alt(device, interface, 0)
        print("USB interrupt hardware test: PASS")
        print("Result         : PASS")
        return 0

    except (RuntimeError, usb.core.USBError) as exc:
        print(f"Error: {exc}", file=sys.stderr)
        print("Result         : FAIL")
        return 1
    finally:
        if device is not None and interface is not None:
            try:
                usb.util.release_interface(device, interface)
            except usb.core.USBError:
                pass
            if detached:
                try:
                    device.attach_kernel_driver(interface)
                except (NotImplementedError, usb.core.USBError):
                    pass
            usb.util.dispose_resources(device)


if __name__ == "__main__":
    raise SystemExit(main())
