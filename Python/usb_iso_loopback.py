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
ROUND_TIMEOUT_S = 3.0

MPS_BY_ALT = (9, 17, 25, 33, 49, 63)


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


def transfer_detail(label, transfer):
    setup = transfer.getISOSetupList()
    if not setup:
        return f"{label}: transfer={status_name(transfer.getStatus())}, no ISO packet"

    packet = setup[0]
    return (
        f"{label}: transfer={status_name(transfer.getStatus())}, "
        f"packet={status_name(packet['status'])}, "
        f"requested={packet['length']}, actual={packet['actual_length']}"
    )


def cancel_transfer(context, transfer):
    if not transfer.isSubmitted():
        return

    try:
        transfer.cancel()
    except usb1.USBError:
        pass

    while transfer.isSubmitted():
        try:
            context.handleEvents()
        except usb1.USBErrorInterrupted:
            pass
        except usb1.USBError:
            break


def build_payload(alt, mps, seq):
    if seq % 3 == 0:
        length = mps
    elif seq % 3 == 1:
        length = max(1, mps - 1)
    else:
        length = 1

    data = bytearray((0xA5 ^ alt ^ seq ^ i) & 0xFF for i in range(length))
    if length >= 4:
        data[0] = alt
        data[1] = seq & 0xFF
        data[2] = (seq >> 8) & 0xFF
        data[3] = (seq >> 16) & 0xFF
    return bytes(data)


def run_round(context, handle, ep, alt, mps, seq, timeout_ms):
    payload = build_payload(alt, mps, seq)
    state = {
        "in_done": False,
        "out_done": False,
        "echo": b"",
        "missed_in": 0,
        "last_in": None,
        "error": None,
    }

    in_transfer = handle.getTransfer(iso_packets=1)
    out_transfer = handle.getTransfer(iso_packets=1)

    def fail(message):
        if state["error"] is None:
            state["error"] = message

    def in_complete(transfer):
        setup = transfer.getISOSetupList()
        if len(setup) != 1:
            fail(transfer_detail("IN", transfer))
            state["in_done"] = True
            return

        packet = setup[0]
        transfer_status = transfer.getStatus()
        packet_status = packet["status"]
        actual = packet["actual_length"]
        state["last_in"] = transfer_detail("IN", transfer)

        # A full-speed ISO IN token can arrive before the loopback firmware has
        # received the matching OUT frame and queued its echo. On macOS the
        # Darwin libusb backend reports that missed service interval as an
        # overall TRANSFER_ERROR while leaving the packet descriptor COMPLETED
        # with actual_length == 0. It is not a fatal transport error here: keep
        # the IN request alive until the OUT frame has produced an echo.
        if (
            actual == 0
            and packet_status == usb1.TRANSFER_COMPLETED
            and transfer_status in (usb1.TRANSFER_COMPLETED, usb1.TRANSFER_ERROR)
            and state["error"] is None
        ):
            state["missed_in"] += 1
            try:
                transfer.submit()
            except usb1.USBError as exc:
                fail(f"IN resubmit failed: {exc}")
                state["in_done"] = True
            return

        if (
            transfer_status != usb1.TRANSFER_COMPLETED
            or packet_status != usb1.TRANSFER_COMPLETED
        ):
            fail(transfer_detail("IN", transfer))
            state["in_done"] = True
            return

        packets = list(transfer.iterISO())
        if len(packets) != 1:
            fail(f"IN: expected one ISO packet, got {len(packets)}")
            state["in_done"] = True
            return

        iter_status, packet_data = packets[0]
        if iter_status != usb1.TRANSFER_COMPLETED:
            fail(
                f"IN: iterISO packet={status_name(iter_status)} "
                f"({transfer_detail('IN', transfer)})"
            )
        else:
            state["echo"] = bytes(packet_data)
        state["in_done"] = True

    def out_complete(transfer):
        setup = transfer.getISOSetupList()
        if (
            transfer.getStatus() != usb1.TRANSFER_COMPLETED
            or len(setup) != 1
            or setup[0]["status"] != usb1.TRANSFER_COMPLETED
        ):
            fail(transfer_detail("OUT", transfer))
        elif setup[0]["actual_length"] != len(payload):
            fail(
                f"{transfer_detail('OUT', transfer)}, "
                f"expected actual={len(payload)}"
            )
        state["out_done"] = True

    in_transfer.setIsochronous(
        usb1.ENDPOINT_IN | ep,
        mps,
        callback=in_complete,
        timeout=timeout_ms,
        iso_transfer_length_list=[mps],
    )
    out_transfer.setIsochronous(
        usb1.ENDPOINT_OUT | ep,
        payload,
        callback=out_complete,
        timeout=timeout_ms,
        iso_transfer_length_list=[len(payload)],
    )

    try:
        # Submit IN first. The first IN interval may be missed because the
        # device cannot echo data until its OUT transaction has completed.
        # Resubmission keeps IN pending while OUT is active, preserving the
        # simultaneous bidirectional ISO test.
        in_transfer.submit()
        out_transfer.submit()

        deadline = time.monotonic() + ROUND_TIMEOUT_S
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
            detail = state["last_in"] or "no IN completion"
            fail(
                f"round timeout: in_done={state['in_done']} "
                f"out_done={state['out_done']}, "
                f"missed IN intervals={state['missed_in']}, last {detail}"
            )

        if state["error"] is None and state["echo"] != payload:
            fail(
                f"data mismatch: sent={payload.hex()} "
                f"received={state['echo'].hex()}"
            )

    except usb1.USBError as exc:
        fail(f"submit failed: {exc}")
    finally:
        cancel_transfer(context, in_transfer)
        cancel_transfer(context, out_transfer)

    return state["error"], state["missed_in"]


def run_alt(context, handle, interface, ep, alt, mps, rounds, timeout_ms):
    handle.setInterfaceAltSetting(interface, 0)
    handle.setInterfaceAltSetting(interface, alt)

    missed_total = 0
    for seq in range(rounds):
        error, missed_in = run_round(
            context, handle, ep, alt, mps, seq, timeout_ms
        )
        missed_total += missed_in
        if error is not None:
            print(
                f"FAIL alt {alt} MPS {mps} round {seq}: {error}",
                file=sys.stderr,
            )
            return False

    print(
        f"PASS alt {alt} MPS {mps}: {rounds} simultaneous IN/OUT rounds "
        f"(missed/empty IN intervals {missed_total})"
    )
    return True


def manual_suspend_wake(context, handle, args):
    handle.setInterfaceAltSetting(args.interface, 0)
    handle.setInterfaceAltSetting(args.interface, 6)

    print()
    print(f"Suspend/wake phase: alt 6 is open and EP{args.ep} OUT is armed.")
    input("Put the host into real USB/system suspend now. After wake, press Enter.")

    error, missed_in = run_round(
        context,
        handle,
        args.ep,
        6,
        MPS_BY_ALT[5],
        0xA55A,
        args.timeout,
    )
    if error is not None:
        print(f"FAIL suspend/wake resume: {error}", file=sys.stderr)
        return False

    print(
        "PASS suspend/wake: existing handle and alt-6 ISO path resumed "
        f"(missed/empty IN intervals {missed_in})"
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
    parser.add_argument("--rounds", type=int, default=DEFAULT_ROUNDS)
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
    if args.rounds <= 0:
        parser.error("--rounds must be positive")
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
