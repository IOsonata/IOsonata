#!/usr/bin/env python3
"""Read device-side counters from the IOsonata USB ISO loopback example."""

import argparse
import struct
import sys

try:
    import usb1
except ImportError:
    usb1 = None

from usb_iso_loopback import discover_iso_loopback, parse_int

DEFAULT_VID = 0x1209
DEFAULT_PID = 0x0003
ISO_DIAG_REQUEST_TYPE = 0xC1  # device-to-host | vendor | interface
ISO_DIAG_REQUEST = 0x5A
ISO_DIAG_FORMAT = "<10I3H2B"
ISO_DIAG_SIZE = struct.calcsize(ISO_DIAG_FORMAT)


def read_diag(handle, interface):
    data = bytes(
        handle.controlRead(
            ISO_DIAG_REQUEST_TYPE,
            ISO_DIAG_REQUEST,
            0,
            interface,
            ISO_DIAG_SIZE,
            timeout=1000,
        )
    )
    if len(data) != ISO_DIAG_SIZE:
        raise RuntimeError(
            f"ISO diagnostic reply is {len(data)} bytes, expected {ISO_DIAG_SIZE}"
        )

    values = struct.unpack(ISO_DIAG_FORMAT, data)
    names = (
        "sof",
        "rx",
        "tx_submit",
        "tx_done",
        "tx_fail",
        "drop",
        "rx_miss",
        "tx_miss",
        "rx_empty",
        "tx_empty",
        "last_rx",
        "last_tx",
        "mps",
        "alt",
        "flags",
    )
    return dict(zip(names, values))


def main():
    parser = argparse.ArgumentParser(
        description="Read device-side diagnostics from USB ISO loopback"
    )
    parser.add_argument("--vid", type=parse_int, default=DEFAULT_VID)
    parser.add_argument("--pid", type=parse_int, default=DEFAULT_PID)
    parser.add_argument("--interface", type=int, default=None)
    parser.add_argument("--ep", type=int, default=None)
    args = parser.parse_args()

    if usb1 is None:
        print("Error: python-libusb1 is required", file=sys.stderr)
        return 2

    try:
        with usb1.USBContext() as context:
            device = context.getByVendorIDAndProductID(
                args.vid,
                args.pid,
                skip_on_error=True,
            )
            if device is None:
                raise RuntimeError(
                    f"Device {args.vid:04x}:{args.pid:04x} not found"
                )

            interface, ep = discover_iso_loopback(
                device,
                interface_override=args.interface,
                ep_override=args.ep,
            )
            handle = device.open()
            try:
                try:
                    handle.setAutoDetachKernelDriver(True)
                except (AttributeError, usb1.USBError):
                    pass

                with handle.claimInterface(interface):
                    diag = read_diag(handle, interface)
            finally:
                handle.close()

    except (RuntimeError, usb1.USBError) as exc:
        print(f"Error: {exc}", file=sys.stderr)
        return 1

    flags = diag["flags"]
    print(f"USB ISO loopback {args.vid:04x}:{args.pid:04x} interface {interface} EP{ep}")
    print(
        "Device ISO diag: "
        f"SOF={diag['sof']} "
        f"RX={diag['rx']} "
        f"TXsubmit={diag['tx_submit']} "
        f"TXdone={diag['tx_done']} "
        f"TXfail={diag['tx_fail']} "
        f"drop={diag['drop']} "
        f"RxMiss={diag['rx_miss']} "
        f"TxMiss={diag['tx_miss']} "
        f"RxEmpty={diag['rx_empty']} "
        f"TxEmpty={diag['tx_empty']} "
        f"lastRX={diag['last_rx']} "
        f"lastTX={diag['last_tx']} "
        f"MPS={diag['mps']} "
        f"alt={diag['alt']} "
        f"opened={int(bool(flags & 0x01))} "
        f"suspended={int(bool(flags & 0x02))} "
        f"txready={int(bool(flags & 0x04))}"
    )

    if diag["sof"] == 0:
        print("Path stop: no SOF reached the active ISO function")
    elif diag["rx"] == 0:
        print("Path stop: no ISO OUT completion reached UsbIsoIntrf")
    elif diag["tx_submit"] == 0:
        print("Path stop: RX reached the loopback but no ISO IN frame was accepted")
    elif diag["tx_done"] == 0:
        print("Path stop: ISO IN was submitted but never completed by the controller")
    else:
        print("Device path reached ISO IN completion; inspect host/bus service timing")

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
