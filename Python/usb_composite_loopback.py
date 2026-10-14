#!/usr/bin/env python3
"""Check the IOsonata CDC/HID/vendor-bulk composite loopback with PyUSB.

Copyright (c) 2026, I-SYST inc. MIT License.
Requires PyUSB, libusb and permission to claim all three USB interfaces.
Uses packet-sized interleaved traffic; this is a correctness test, not a
throughput benchmark. The firmware uses demo VID/PID 1209:0005.
"""
import argparse
import sys
import time


def exercise(device, cycles, reconfigure, timeout):
    import usb.core
    import usb.util
    detached = []
    claimed = []
    try:
        config = device.get_active_configuration()
        if config.bConfigurationValue != 1:
            device.set_configuration(1)
            config = device.get_active_configuration()
        interfaces = {interface.bInterfaceClass: interface for interface in config}
        for kind in (2, 10, 3, 255):
            if kind not in interfaces:
                raise RuntimeError(f'Missing USB interface class {kind}')
        for interface in config:
            number = interface.bInterfaceNumber
            try:
                if device.is_kernel_driver_active(number):
                    device.detach_kernel_driver(number)
                    detached.append(number)
            except NotImplementedError:
                pass
            usb.util.claim_interface(device, number)
            claimed.append(number)
        transports = []
        for kind in (10, 3, 255):
            interface = interfaces[kind]
            endpoints = {usb.util.endpoint_direction(ep.bEndpointAddress): ep for ep in interface}
            out_ep = endpoints[usb.util.ENDPOINT_OUT]
            in_ep = endpoints[usb.util.ENDPOINT_IN]
            if out_ep.wMaxPacketSize != 64 or in_ep.wMaxPacketSize != 64:
                raise RuntimeError('This example requires full-speed 64-byte endpoints')
            transports.append((kind, out_ep, in_ep))
        for cycle in range(cycles):
            if reconfigure and cycle:
                # Repeat SET_CONFIGURATION while interfaces remain claimed.
                device.ctrl_transfer(0, 9, config.bConfigurationValue, 0, None, timeout)
            device.ctrl_transfer(0x21, 0x22, 3, interfaces[2].bInterfaceNumber, None, timeout)
            for total in (1, 7, 63, 64, 65, 127, 128, 129, 512, 1024):
                for offset in range(0, total, 64):
                    expected = []
                    for kind, out_ep, in_ep in transports:
                        length = 64 if kind == 3 else min(64, total - offset)
                        payload = bytes((cycle + offset + i + kind) & 255 for i in range(length))
                        if out_ep.write(payload, timeout) != length:
                            raise RuntimeError(f'Short OUT write on interface class {kind}')
                        expected.append((kind, in_ep, payload))
                    for kind, in_ep, payload in expected:
                        received = bytearray()
                        deadline = time.monotonic() + timeout / 1000
                        while len(received) < len(payload):
                            remaining = int((deadline - time.monotonic()) * 1000)
                            if remaining <= 0:
                                raise RuntimeError(f'Echo timeout on interface class {kind}')
                            packet = bytes(in_ep.read(64, remaining))
                            if not packet:
                                continue  # An exact-size bulk write may end with a ZLP.
                            received.extend(packet)
                        if received != payload:
                            raise RuntimeError(f'Payload mismatch: cycle {cycle}, class {kind}, offset {offset}')
            print(f'Cycle {cycle + 1}/{cycles}: CDC, HID and vendor bulk PASS', flush=True)
    finally:
        for number in reversed(claimed):
            try:
                usb.util.release_interface(device, number)
            except usb.core.USBError:
                pass
        for number in detached:
            try:
                device.attach_kernel_driver(number)
            except (NotImplementedError, usb.core.USBError):
                pass
        usb.util.dispose_resources(device)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--serial')
    parser.add_argument('--cycles', type=int, default=10)
    parser.add_argument('--timeout', type=int, default=2000, help='USB request timeout in milliseconds')
    parser.add_argument('--reconfigure', action='store_true')
    args = parser.parse_args()
    if args.cycles < 1 or args.timeout < 1:
        parser.error('cycles and timeout must be positive')
    import usb.core
    import usb.util
    devices = []
    for device in usb.core.find(find_all=True, idVendor=0x1209, idProduct=0x0005):
        if usb.util.get_string(device, device.iProduct) != 'IOsonata USB Composite Loopback':
            continue
        if args.serial and usb.util.get_string(device, device.iSerialNumber) != args.serial:
            continue
        devices.append(device)
    if len(devices) != 1:
        raise RuntimeError(f'Found {len(devices)} matching devices; connect one or use --serial')
    exercise(devices[0], args.cycles, args.reconfigure, args.timeout)


if __name__ == '__main__':
    try:
        main()
    except (ImportError, RuntimeError) as error:
        sys.exit(str(error))
