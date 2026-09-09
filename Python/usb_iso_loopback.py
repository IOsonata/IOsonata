#!/usr/bin/env python3
"""USB isochronous loopback hardware test for IOsonata UsbIsoIntrf."""

import argparse
import sys
import time

try:
    import usb1
except ImportError:
    usb1 = None

DEFAULT_VID = 0x1209
DEFAULT_PID = 0x0003
DEFAULT_ROUNDS = 32
DEFAULT_TIMEOUT_MS = 1000

MPS_BY_ALT = (9, 17, 25, 33, 49, 63)
BURST_GUARD_FRAMES = 8
BURST_EXTRA_IN_FRAMES = 16
USB_ENDPOINT_TRANSFER_TYPE_MASK = 0x03
USB_ENDPOINT_TRANSFER_TYPE_ISO = 0x01


def parse_int(value):
    return int(value, 0)


def status_name(status):
    names = {
        usb1.TRANSFER_COMPLETED: "COMPLETED",
        usb1.TRANSFER_ERROR: "ERROR",
        usb1.TRANSFER_TIMED_OUT: "TIMED_OUT",
        usb1.TRANSFER_CANCELLED: "CANCELLED",
        usb1.TRANSFER_STALL: "STALL",
        usb1.TRANSFER_NO_DEVICE: "NO_DEVICE",
        usb1.TRANSFER_OVERFLOW: "OVERFLOW",
    }
    return names.get(status, str(status))


def cancel_transfer(context, transfer):
    if not transfer.isSubmitted():
        return

    try:
        transfer.cancel()
    except usb1.USBError:
        pass

    deadline = time.monotonic() + 0.5
    while transfer.isSubmitted() and time.monotonic() < deadline:
        try:
            context.handleEvents()
        except usb1.USBErrorInterrupted:
            pass
        except usb1.USBError:
            break


def build_payload(alt, length, seq):
    data = bytearray(
        ((0xA5 ^ (alt * 13) ^ (seq * 17) ^ (i * 29)) & 0xFF)
        for i in range(length)
    )
    if length >= 4:
        data[0] = alt
        data[1] = seq & 0xFF
        data[2] = (seq >> 8) & 0xFF
        data[3] = length
    return bytes(data)


def packet_summary(setup):
    counts = {}
    nonzero = 0
    for packet in setup:
        name = status_name(packet["status"])
        counts[name] = counts.get(name, 0) + 1
        if packet["actual_length"]:
            nonzero += 1
    text = ", ".join(f"{name}={count}" for name, count in sorted(counts.items()))
    return f"{text}; nonzero={nonzero}/{len(setup)}"


def discover_iso_loopback(device, interface_override=None, ep_override=None):
    """Find the vendor interface whose alt 1..6 match the loopback descriptors."""
    candidates = {}

    for setting in device.iterSettings():
        interface = setting.getNumber()
        alt = setting.getAlternateSetting()
        if interface_override is not None and interface != interface_override:
            continue
        if alt < 1 or alt > len(MPS_BY_ALT):
            continue
        if setting.getClass() != 0xFF:
            continue

        endpoints = list(setting.iterEndpoints())
        if len(endpoints) != 2:
            continue

        in_ep = None
        out_ep = None
        for endpoint in endpoints:
            if (
                endpoint.getAttributes() & USB_ENDPOINT_TRANSFER_TYPE_MASK
            ) != USB_ENDPOINT_TRANSFER_TYPE_ISO:
                continue

            address = endpoint.getAddress()
            ep = address & 0x0F
            if ep_override is not None and ep != ep_override:
                continue

            if address & usb1.ENDPOINT_IN:
                in_ep = endpoint
            else:
                out_ep = endpoint

        if in_ep is None or out_ep is None:
            continue

        in_no = in_ep.getAddress() & 0x0F
        out_no = out_ep.getAddress() & 0x0F
        if in_no == 0 or in_no != out_no:
            continue

        expected_mps = MPS_BY_ALT[alt - 1]
        if (
            in_ep.getMaxPacketSize() != expected_mps
            or out_ep.getMaxPacketSize() != expected_mps
        ):
            continue

        key = (interface, in_no)
        candidates.setdefault(key, set()).add(alt)

    expected_alts = set(range(1, len(MPS_BY_ALT) + 1))
    matches = [
        key for key, alts in candidates.items()
        if expected_alts.issubset(alts)
    ]

    if not matches:
        detail = ""
        if interface_override is not None:
            detail += f" interface {interface_override}"
        if ep_override is not None:
            detail += f" EP{ep_override}"
        raise RuntimeError(
            "No IOsonata generic ISO loopback interface matching alt1..alt6"
            + detail
        )
    if len(matches) > 1:
        text = ", ".join(f"interface {i} EP{ep}" for i, ep in matches)
        raise RuntimeError(f"Multiple ISO loopback interfaces match: {text}")

    return matches[0]


def run_burst(
    context,
    handle,
    ep,
    alt,
    mps,
    test_length,
    rounds,
    timeout_ms,
    sequence_base,
):
    guard = BURST_GUARD_FRAMES
    out_count = guard + rounds + guard
    in_count = out_count + BURST_EXTRA_IN_FRAMES

    payloads = [
        build_payload(alt, test_length, sequence_base + i)
        for i in range(out_count)
    ]
    out_buffer = b"".join(payloads)
    out_lengths = [test_length] * out_count
    in_lengths = [mps] * in_count

    state = {
        "in_done": False,
        "out_done": False,
        "in_status": None,
        "out_status": None,
        "in_setup": [],
        "out_setup": [],
        "in_packets": [],
        "error": None,
    }

    # python-libusb1 rejects a per-frame ISO length of 0, so a zero-length
    # OUT burst cannot be submitted through this API. For the ZLP case drive
    # the IN endpoint alone and rely on the device ISOINCONFIG=ZeroData idle
    # response to produce the zero-length IN frames the validator checks for.
    zlp = (test_length == 0)

    in_transfer = handle.getTransfer(iso_packets=in_count)
    out_transfer = None if zlp else handle.getTransfer(iso_packets=out_count)

    def fail(message):
        if state["error"] is None:
            state["error"] = message

    def in_complete(transfer):
        state["in_status"] = transfer.getStatus()
        state["in_setup"] = list(transfer.getISOSetupList())
        state["in_packets"] = [
            (packet_status, bytes(packet_data))
            for packet_status, packet_data in transfer.iterISO()
        ]
        state["in_done"] = True

    def out_complete(transfer):
        state["out_status"] = transfer.getStatus()
        state["out_setup"] = list(transfer.getISOSetupList())
        state["out_done"] = True

    in_transfer.setIsochronous(
        usb1.ENDPOINT_IN | ep,
        mps * in_count,
        callback=in_complete,
        timeout=timeout_ms,
        iso_transfer_length_list=in_lengths,
    )
    if zlp:
        # No OUT transfer to send or wait on.
        state["out_done"] = True
    else:
        out_transfer.setIsochronous(
            usb1.ENDPOINT_OUT | ep,
            out_buffer,
            callback=out_complete,
            timeout=timeout_ms,
            iso_transfer_length_list=out_lengths,
        )

    try:
        # Keep IN service scheduled before OUT. ISO has no retry, so a
        # one-transfer-at-a-time bulk-style request/echo test is invalid.
        in_transfer.submit()
        if not zlp:
            out_transfer.submit()

        deadline = time.monotonic() + max(
            3.0,
            (in_count * 0.002) + (timeout_ms / 1000.0) + 1.0,
        )
        while (
            state["error"] is None
            and not (state["in_done"] and state["out_done"])
            and time.monotonic() < deadline
        ):
            try:
                context.handleEvents()
            except usb1.USBErrorInterrupted:
                pass
            except usb1.USBError as exc:
                fail(f"libusb event handling failed: {exc}")

        if state["error"] is None and not (
            state["in_done"] and state["out_done"]
        ):
            fail(
                f"burst timeout: in_done={state['in_done']} "
                f"out_done={state['out_done']}"
            )

        if state["error"] is not None:
            return state["error"], None

        if not zlp:
            if len(state["out_setup"]) != out_count:
                return (
                    f"OUT descriptor count {len(state['out_setup'])}, "
                    f"expected {out_count}",
                    None,
                )

            for index, packet in enumerate(state["out_setup"]):
                if (
                    packet["status"] != usb1.TRANSFER_COMPLETED
                    or packet["actual_length"] != out_lengths[index]
                ):
                    return (
                        f"OUT packet {index}: status={status_name(packet['status'])}, "
                        f"requested={out_lengths[index]}, "
                        f"actual={packet['actual_length']}; "
                        f"transfer={status_name(state['out_status'])}",
                        None,
                    )

        if test_length == 0:
            completed_zero = [
                index
                for index, (packet_status, packet_data)
                in enumerate(state["in_packets"])
                if packet_status == usb1.TRANSFER_COMPLETED and len(packet_data) == 0
            ]
            if len(completed_zero) < rounds:
                return (
                    f"zero-length echo count {len(completed_zero)}, "
                    f"expected at least {rounds}; "
                    f"IN transfer={status_name(state['in_status'])}; "
                    f"{packet_summary(state['in_setup'])}",
                    None,
                )

            stats = {
                "received": len(completed_zero),
                "in_slots": in_count,
                "matched": min(len(completed_zero), rounds),
                "out_packets": out_count,
                "in_status": status_name(state["in_status"]),
            }
            return None, stats

        received = []
        bad_packets = []
        for index, (packet_status, packet_data) in enumerate(state["in_packets"]):
            if packet_data:
                if packet_status != usb1.TRANSFER_COMPLETED:
                    bad_packets.append(
                        f"{index}:{status_name(packet_status)}/{len(packet_data)}"
                    )
                else:
                    received.append(packet_data)

        if bad_packets:
            return (
                "IN packets with data but error status: " + ", ".join(bad_packets[:8]),
                None,
            )

        # Match non-empty echoes in order. Guard frames absorb the normal
        # host/device phase offset; every frame in the validation window must
        # survive or the test reports a drop/corruption.
        matched = set()
        next_tx = 0
        unknown = []
        for packet_data in received:
            found = None
            for tx_index in range(next_tx, len(payloads)):
                if payloads[tx_index] == packet_data:
                    found = tx_index
                    break
            if found is None:
                unknown.append(packet_data.hex())
                continue
            matched.add(found)
            next_tx = found + 1

        if unknown:
            return (
                "unexpected IN payload(s): " + ", ".join(unknown[:4]),
                None,
            )

        first_test = guard
        last_test = guard + rounds
        missing = [
            index - first_test
            for index in range(first_test, last_test)
            if index not in matched
        ]
        if missing:
            return (
                f"missing validation frame(s) {missing[:16]}; "
                f"IN transfer={status_name(state['in_status'])}; "
                f"{packet_summary(state['in_setup'])}",
                None,
            )

        stats = {
            "received": len(received),
            "in_slots": in_count,
            "matched": len(matched),
            "out_packets": out_count,
            "in_status": status_name(state["in_status"]),
        }
        return None, stats

    except usb1.USBError as exc:
        return f"submit failed: {exc}", None
    finally:
        cancel_transfer(context, in_transfer)
        if out_transfer is not None:
            cancel_transfer(context, out_transfer)


def run_alt(context, handle, interface, ep, alt, mps, rounds, timeout_ms):
    lengths = (mps, mps - 1, 1, 0)

    for mode, length in enumerate(lengths):
        handle.setInterfaceAltSetting(interface, 0)
        handle.setInterfaceAltSetting(interface, alt)
        time.sleep(0.005)

        error, stats = run_burst(
            context,
            handle,
            ep,
            alt,
            mps,
            length,
            rounds,
            timeout_ms,
            sequence_base=mode * 512,
        )
        if error is not None:
            print(
                f"FAIL alt {alt} MPS {mps} length {length}: {error}",
                file=sys.stderr,
            )
            return False

        label = "zero-length ISO packets" if length == 0 else "validation frames"
        print(
            f"PASS alt {alt} MPS {mps} length {length}: "
            f"{rounds} {label} "
            f"(RX {stats['received']}/{stats['in_slots']} ISO slots, "
            f"IN transfer {stats['in_status']})"
        )

    return True


def manual_suspend_wake(context, handle, interface, ep, rounds, timeout_ms):
    handle.setInterfaceAltSetting(interface, 0)
    handle.setInterfaceAltSetting(interface, 6)
    time.sleep(0.005)

    print()
    print(f"Suspend/wake phase: alt 6 is open on interface {interface}, EP{ep}.")
    input("Put the host into real USB/system suspend now. After wake, press Enter.")

    error, stats = run_burst(
        context,
        handle,
        ep,
        6,
        MPS_BY_ALT[5],
        MPS_BY_ALT[5],
        max(1, min(rounds, 8)),
        timeout_ms,
        sequence_base=0x4000,
    )
    if error is not None:
        print(f"FAIL suspend/wake resume: {error}", file=sys.stderr)
        return False

    print(
        "PASS suspend/wake: existing handle and alt-6 ISO path resumed "
        f"(RX {stats['received']}/{stats['in_slots']} ISO slots)"
    )
    return True


def main():
    parser = argparse.ArgumentParser(
        description="Exercise the IOsonata generic UsbIsoIntrf loopback example"
    )
    parser.add_argument("--vid", type=parse_int, default=DEFAULT_VID)
    parser.add_argument("--pid", type=parse_int, default=DEFAULT_PID)
    parser.add_argument(
        "--interface",
        type=int,
        default=None,
        help="optional interface override; default discovers descriptor topology",
    )
    parser.add_argument(
        "--ep",
        type=int,
        default=None,
        help="optional endpoint-number override; default discovers descriptor topology",
    )
    parser.add_argument(
        "--rounds",
        type=int,
        default=DEFAULT_ROUNDS,
        help="validated frames per packet length and alternate setting",
    )
    parser.add_argument(
        "--timeout",
        type=int,
        default=DEFAULT_TIMEOUT_MS,
        help="per-transfer timeout in ms",
    )
    parser.add_argument("--manual-suspend-wake", action="store_true")
    args = parser.parse_args()

    if usb1 is None:
        print("Error: python-libusb1 is required. Install it with:", file=sys.stderr)
        print("  python3 -m pip install libusb1", file=sys.stderr)
        print("Result         : FAIL")
        return 2

    if args.interface is not None and (args.interface < 0 or args.interface > 255):
        parser.error("--interface must be 0..255")
    if args.ep is not None and (args.ep <= 0 or args.ep > 15):
        parser.error("--ep must be 1..15")
    if args.rounds <= 0 or args.rounds > 200:
        parser.error("--rounds must be 1..200")
    if args.timeout <= 0:
        parser.error("--timeout must be positive")

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
                    print(
                        f"USB ISO loopback {args.vid:04x}:{args.pid:04x} "
                        f"interface {interface} EP{ep}"
                    )

                    for alt, mps in enumerate(MPS_BY_ALT, start=1):
                        if not run_alt(
                            context,
                            handle,
                            interface,
                            ep,
                            alt,
                            mps,
                            args.rounds,
                            args.timeout,
                        ):
                            print("Result         : FAIL")
                            return 1

                    if args.manual_suspend_wake:
                        if not manual_suspend_wake(
                            context,
                            handle,
                            interface,
                            ep,
                            args.rounds,
                            args.timeout,
                        ):
                            print("Result         : FAIL")
                            return 1
                    else:
                        print(
                            "Suspend/wake not run; use --manual-suspend-wake "
                            "for full hardware validation."
                        )

                    handle.setInterfaceAltSetting(interface, 0)
            finally:
                handle.close()

    except (RuntimeError, usb1.USBError) as exc:
        print(f"Error: {exc}", file=sys.stderr)
        print("Result         : FAIL")
        return 1

    print(
        "USB ISO hardware test: PASS"
        + (" including suspend/wake" if args.manual_suspend_wake else "")
    )
    print("Result         : PASS")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
