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
DEFAULT_INTERFACE = 0
DEFAULT_EP = 8
DEFAULT_ROUNDS = 32
DEFAULT_TIMEOUT_MS = 1000

MPS_BY_ALT = (9, 17, 25, 33, 49, 63)
BURST_GUARD_FRAMES = 8
BURST_EXTRA_IN_FRAMES = 16


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
    out_lengths = [len(payload) for payload in payloads]
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

    in_transfer = handle.getTransfer(iso_packets=in_count)
    out_transfer = handle.getTransfer(iso_packets=out_count)

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
    out_transfer.setIsochronous(
        usb1.ENDPOINT_OUT | ep,
        out_buffer,
        callback=out_complete,
        timeout=timeout_ms,
        iso_transfer_length_list=out_lengths,
    )

    try:
        # Keep a continuous IN schedule in place before the OUT stream starts.
        # Unlike bulk, ISO has no retry. A one-frame request/echo transaction can
        # lose the only echo merely because the host and device chose different
        # service frames. Multi-packet transfers keep both directions scheduled
        # for consecutive frames and test the transport as an ISO stream.
        in_transfer.submit()
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

        # Match received non-empty frames against the transmitted stream in
        # order. The leading/trailing guard frames absorb normal host/device
        # phase differences. Every frame in the middle validation window must
        # survive; otherwise this is a real ISO drop or corruption.
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
            in_summary = packet_summary(state["in_setup"])
            return (
                f"missing validation frame(s) {missing[:16]}; "
                f"IN transfer={status_name(state['in_status'])}; {in_summary}",
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
        cancel_transfer(context, out_transfer)


def run_alt(context, handle, interface, ep, alt, mps, rounds, timeout_ms):
    lengths = (mps, max(1, mps - 1), 1)

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

        print(
            f"PASS alt {alt} MPS {mps} length {length}: "
            f"{rounds} validation frames "
            f"(RX {stats['received']}/{stats['in_slots']} ISO slots, "
            f"IN transfer {stats['in_status']})"
        )

    return True


def manual_suspend_wake(context, handle, args):
    handle.setInterfaceAltSetting(args.interface, 0)
    handle.setInterfaceAltSetting(args.interface, 6)
    time.sleep(0.005)

    print()
    print(f"Suspend/wake phase: alt 6 is open and EP{args.ep} OUT is armed.")
    input("Put the host into real USB/system suspend now. After wake, press Enter.")

    error, stats = run_burst(
        context,
        handle,
        args.ep,
        6,
        MPS_BY_ALT[5],
        MPS_BY_ALT[5],
        max(1, min(args.rounds, 8)),
        args.timeout,
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
        description="Exercise the IOsonata UsbIsoLoopback example"
    )
    parser.add_argument("--vid", type=parse_int, default=DEFAULT_VID)
    parser.add_argument("--pid", type=parse_int, default=DEFAULT_PID)
    parser.add_argument("--interface", type=int, default=DEFAULT_INTERFACE)
    parser.add_argument("--ep", type=int, default=DEFAULT_EP)
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

    if args.ep <= 0 or args.ep > 15:
        parser.error("--ep must be 1..15")
    if args.rounds <= 0 or args.rounds > 200:
        parser.error("--rounds must be 1..200")
    if args.timeout <= 0:
        parser.error("--timeout must be positive")

    try:
        with usb1.USBContext() as context:
            handle = context.openByVendorIDAndProductID(
                args.vid,
                args.pid,
                skip_on_error=True,
            )
            if handle is None:
                raise RuntimeError(
                    f"Device {args.vid:04x}:{args.pid:04x} not found"
                )

            try:
                try:
                    handle.setAutoDetachKernelDriver(True)
                except (AttributeError, usb1.USBError):
                    pass

                with handle.claimInterface(args.interface):
                    print(
                        f"USB ISO loopback {args.vid:04x}:{args.pid:04x} "
                        f"interface {args.interface} EP{args.ep}"
                    )

                    for alt, mps in enumerate(MPS_BY_ALT, start=1):
                        if not run_alt(
                            context,
                            handle,
                            args.interface,
                            args.ep,
                            alt,
                            mps,
                            args.rounds,
                            args.timeout,
                        ):
                            print("Result         : FAIL")
                            return 1

                    if args.manual_suspend_wake:
                        if not manual_suspend_wake(context, handle, args):
                            print("Result         : FAIL")
                            return 1
                    else:
                        print(
                            "Suspend/wake not run; use --manual-suspend-wake "
                            "for full hardware validation."
                        )

                    handle.setInterfaceAltSetting(args.interface, 0)
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
