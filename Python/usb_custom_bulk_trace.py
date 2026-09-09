#!/usr/bin/env python3
"""Trace the IOsonata custom bulk loopback data path."""

import argparse
import struct
import sys
import time

import usb.core
import usb.util

from usb_custom_bulk_loopback import (
    DEFAULT_PRODUCT,
    ensure_configuration,
    find_custom_interface,
    find_device,
    get_string,
    parse_int,
    read_exact,
)


TRACE_GET_REQ = 0x5A
TRACE_RESET_REQ = 0x5B
TRACE_MAGIC = 0x314B4C42
TRACE_WORDS = 16
TRACE_SIZE = TRACE_WORDS * 4

CTRL_IN_VENDOR_INTERFACE = 0xC1
CTRL_OUT_VENDOR_INTERFACE = 0x41


def trace_reset(device, interface_no, timeout_ms):
    device.ctrl_transfer(
        CTRL_OUT_VENDOR_INTERFACE,
        TRACE_RESET_REQ,
        0,
        interface_no,
        None,
        timeout=timeout_ms,
    )


def trace_read(device, interface_no, timeout_ms):
    raw = bytes(
        device.ctrl_transfer(
            CTRL_IN_VENDOR_INTERFACE,
            TRACE_GET_REQ,
            0,
            interface_no,
            TRACE_SIZE,
            timeout=timeout_ms,
        )
    )
    if len(raw) != TRACE_SIZE:
        raise RuntimeError(f"Trace short read: {len(raw)} of {TRACE_SIZE} bytes")

    words = struct.unpack("<16I", raw)
    if words[0] != TRACE_MAGIC:
        raise RuntimeError(f"Trace magic mismatch: 0x{words[0]:08x}")
    if words[1] != 1:
        raise RuntimeError(f"Unsupported trace version: {words[1]}")

    topology = words[2]
    flags = words[4]
    tx_pending = words[7]

    return {
        "config": topology & 0xFF,
        "interface": (topology >> 8) & 0xFF,
        "endpoint": (topology >> 16) & 0xFF,
        "mps": words[3],
        "rx_pending": bool(flags & 0x01),
        "tx_ready": bool(flags & 0x02),
        "rx_drop": (flags >> 16) & 0xFFFF,
        "rx_fifo_used": words[5],
        "rx_fifo_avail": words[6],
        "tx_fifo_used": tx_pending & 0xFFFF,
        "loop_pending": (tx_pending >> 16) & 0xFFFF,
        "rx_data_evt": words[8],
        "rx_fifo_full_evt": words[9],
        "rx_timeout_evt": words[10],
        "tx_timeout_evt": words[11],
        "tx_empty_evt": words[12],
        "app_rx_bytes": words[13],
        "app_tx_bytes": words[14],
        "app_tx_zero": words[15],
    }


def print_trace(label, trace):
    print(
        f"{label:<15}: "
        f"cfg={trace['config']} itf={trace['interface']} ep={trace['endpoint']} "
        f"mps={trace['mps']} rxPending={int(trace['rx_pending'])} "
        f"txReady={int(trace['tx_ready'])} rxDrop={trace['rx_drop']} "
        f"rxFifo={trace['rx_fifo_used']}/{trace['rx_fifo_avail']} "
        f"txUsed={trace['tx_fifo_used']} loopPending={trace['loop_pending']} "
        f"rxEvt={trace['rx_data_evt']} fifoFull={trace['rx_fifo_full_evt']} "
        f"rxTimeout={trace['rx_timeout_evt']} txTimeout={trace['tx_timeout_evt']} "
        f"txEmpty={trace['tx_empty_evt']} appRx={trace['app_rx_bytes']} "
        f"appTx={trace['app_tx_bytes']} txZero={trace['app_tx_zero']}"
    )


def try_print_trace(device, interface_no, timeout_ms, label):
    try:
        print_trace(label, trace_read(device, interface_no, timeout_ms))
    except (usb.core.USBError, RuntimeError) as exc:
        print(f"{label:<15}: unavailable ({exc})")


def main():
    parser = argparse.ArgumentParser(
        description="Trace the IOsonata UsbCustomBulkLoopback example"
    )
    parser.add_argument("--vid", type=parse_int, default=0x1209)
    parser.add_argument("--pid", type=parse_int, default=0x0002)
    parser.add_argument("--product", default=DEFAULT_PRODUCT)
    parser.add_argument("--serial")
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
    active_cycle = -1
    active_size = -1
    active_phase = "setup"

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

        trace_reset(device, interface_no, args.timeout)
        print_trace("Trace initial", trace_read(device, interface_no, args.timeout))

        sizes = (1, 7, 63, 64, 65, 127, 128, 129, 512, 1024)
        total = 0
        start = time.monotonic()

        for cycle in range(args.cycles):
            for size in sizes:
                active_cycle = cycle
                active_size = size
                active_phase = "OUT write"
                payload = bytes(((cycle + i) & 0xFF) for i in range(size))

                written = ep_out.write(payload, timeout=args.timeout)
                if written != len(payload):
                    raise RuntimeError(
                        f"Short write: {written} of {len(payload)} bytes"
                    )

                active_phase = "IN read"
                echoed = read_exact(ep_in, len(payload), args.timeout)
                if echoed != payload:
                    raise RuntimeError(
                        f"Data mismatch at cycle {cycle}, size {size}"
                    )
                total += len(payload)

        elapsed = time.monotonic() - start
        rate = total / elapsed if elapsed > 0 else 0.0
        print_trace("Trace final", trace_read(device, interface_no, args.timeout))
        print(f"Bytes          : {total}")
        print(f"Bytes/sec      : {rate:.2f}")
        print("Result         : PASS")
        return 0

    except (usb.core.USBError, RuntimeError) as exc:
        print(f"Error: {exc}", file=sys.stderr)
        print(
            f"Failure        : phase={active_phase}, "
            f"cycle={active_cycle}, size={active_size}"
        )
        try_print_trace(device, interface_no, args.timeout, "Trace failure")
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
