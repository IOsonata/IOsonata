#!/usr/bin/env python3
"""Oversized ISO OUT frame recovery test for IOsonata UsbIsoIntrf.

Companion to Python/usb_iso_loopback.py. It requires the loopback firmware
built with ISO_TEST_RX_CLAMP defined (for example ISO_TEST_RX_CLAMP=9), so
the device opens the endpoint with a smaller MPS than the descriptor
advertises. A host frame of the full descriptor MPS is then oversized at the
controller and must be dropped for that frame only.

The test streams one continuous OUT burst where every frame carries the
clamped length except a single mid-burst frame of the full alternate-setting
MPS. Expected behavior: the oversized frame is not echoed, and every fitting
frame after it still loops back. The historical failure mode is that all
frames after the oversized one are lost until the stream gaps.
"""

import argparse
import struct
import sys
import time

try:
    import usb1
except ImportError:
    usb1 = None

from usb_iso_loopback import (
    DEFAULT_VID,
    DEFAULT_PID,
    MPS_BY_ALT,
    BURST_GUARD_FRAMES,
    BURST_EXTRA_IN_FRAMES,
    build_payload,
    cancel_transfer,
    discover_iso_loopback,
    packet_summary,
    parse_int,
    status_name,
)

DEFAULT_ALT = 2
DEFAULT_CLAMP = 9
DEFAULT_ROUNDS = 24
DEFAULT_TIMEOUT_MS = 1000

ISO_REQ_GET_DIAG = 0x5A
ISO_DIAG_LENGTH = 48


def read_diag(handle, interface, timeout_ms):
    request_type = (
        usb1.ENDPOINT_IN | usb1.TYPE_VENDOR | usb1.RECIPIENT_INTERFACE
    )
    data = handle.controlRead(
        request_type, ISO_REQ_GET_DIAG, 0, interface,
        ISO_DIAG_LENGTH, timeout_ms,
    )
    if len(data) != ISO_DIAG_LENGTH:
        raise RuntimeError(f"diag reply length {len(data)}")
    fields = struct.unpack("<10I3H2B", data)
    return {
        "RxCnt": fields[1],
        "TxSubmitCnt": fields[2],
        "TxDoneCnt": fields[3],
        "TxFailCnt": fields[4],
        "LoopbackDropCnt": fields[5],
        "RxMissCnt": fields[6],
        "Mps": fields[12],
        "Alt": fields[13],
    }


def run_oversize_burst(
    context, handle, ep, alt, mps, clamp, rounds, timeout_ms
):
    guard = BURST_GUARD_FRAMES
    out_count = guard + rounds + guard
    in_count = out_count + BURST_EXTRA_IN_FRAMES
    oversize_index = guard + rounds // 2

    out_lengths = [clamp] * out_count
    out_lengths[oversize_index] = mps
    payloads = [
        build_payload(alt, out_lengths[i], i) for i in range(out_count)
    ]
    out_buffer = b"".join(payloads)
    in_lengths = [mps] * in_count

    state = {
        "in_done": False,
        "out_done": False,
        "in_status": None,
        "out_status": None,
        "in_setup": [],
        "in_packets": [],
        "error": None,
    }

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
        state["out_done"] = True

    in_transfer = handle.getTransfer(iso_packets=in_count)
    out_transfer = handle.getTransfer(iso_packets=out_count)
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
        # IN service scheduled first, exactly like the loopback bench.
        in_transfer.submit()
        out_transfer.submit()

        deadline = time.monotonic() + max(
            3.0, (in_count * 0.002) + (timeout_ms / 1000.0) + 1.0
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

        received = []
        for packet_status, packet_data in state["in_packets"]:
            if packet_data:
                if packet_status != usb1.TRANSFER_COMPLETED:
                    return (
                        f"IN packet with data but status "
                        f"{status_name(packet_status)}",
                        None,
                    )
                received.append(packet_data)

        # In-order matching against the transmitted payloads, as in the
        # loopback bench. Sequence bytes make every payload unique.
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

        window = range(guard, guard + rounds)
        missing = [i for i in window if i not in matched]
        stats = {
            "oversize_index": oversize_index,
            "missing": missing,
            "received": len(received),
            "in_status": status_name(state["in_status"]),
            "summary": packet_summary(state["in_setup"]),
        }
        return None, stats

    except usb1.USBError as exc:
        return f"submit failed: {exc}", None
    finally:
        cancel_transfer(context, in_transfer)
        cancel_transfer(context, out_transfer)


def main():
    parser = argparse.ArgumentParser(
        description=(
            "Oversized ISO OUT frame recovery test; requires loopback "
            "firmware built with ISO_TEST_RX_CLAMP"
        )
    )
    parser.add_argument("--vid", type=parse_int, default=DEFAULT_VID)
    parser.add_argument("--pid", type=parse_int, default=DEFAULT_PID)
    parser.add_argument("--interface", type=int, default=None)
    parser.add_argument("--ep", type=int, default=None)
    parser.add_argument(
        "--alt",
        type=int,
        default=DEFAULT_ALT,
        help="alternate setting to test; its MPS must exceed the clamp",
    )
    parser.add_argument(
        "--clamp",
        type=int,
        default=DEFAULT_CLAMP,
        help="ISO_TEST_RX_CLAMP value the firmware was built with",
    )
    parser.add_argument("--rounds", type=int, default=DEFAULT_ROUNDS)
    parser.add_argument("--timeout", type=int, default=DEFAULT_TIMEOUT_MS)
    args = parser.parse_args()

    if usb1 is None:
        print("Error: python-libusb1 is required. Install it with:",
              file=sys.stderr)
        print("  python3 -m pip install libusb1", file=sys.stderr)
        print("Result         : FAIL")
        return 2
    if args.alt < 1 or args.alt > len(MPS_BY_ALT):
        parser.error(f"--alt must be 1..{len(MPS_BY_ALT)}")
    mps = MPS_BY_ALT[args.alt - 1]
    if args.clamp < 4 or args.clamp >= mps:
        parser.error("--clamp must be >= 4 and smaller than the alt MPS")
    if args.rounds < 8 or args.rounds > 200:
        parser.error("--rounds must be 8..200")

    try:
        with usb1.USBContext() as context:
            device = context.getByVendorIDAndProductID(
                args.vid, args.pid, skip_on_error=True
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
                        f"USB ISO oversize test {args.vid:04x}:{args.pid:04x} "
                        f"interface {interface} EP{ep} alt {args.alt} "
                        f"mps {mps} clamp {args.clamp}"
                    )
                    handle.setInterfaceAltSetting(interface, args.alt)

                    diag = read_diag(handle, interface, args.timeout)
                    if diag["Mps"] != args.clamp:
                        raise RuntimeError(
                            f"device opened MPS {diag['Mps']}, expected the "
                            f"clamp {args.clamp}; firmware not built with "
                            f"ISO_TEST_RX_CLAMP={args.clamp}?"
                        )

                    error, stats = run_oversize_burst(
                        context, handle, ep, args.alt, mps,
                        args.clamp, args.rounds, args.timeout,
                    )

                    diag = read_diag(handle, interface, args.timeout)
                    handle.setInterfaceAltSetting(interface, 0)

                    if error is not None:
                        print(f"Error: {error}")
                        print("Result         : FAIL")
                        return 1

                    oversize = stats["oversize_index"]
                    missing = stats["missing"]
                    print(
                        f"oversized frame index {oversize}; "
                        f"missing window frames {missing}; "
                        f"IN {stats['in_status']}; {stats['summary']}"
                    )
                    print(
                        f"device diag: RxCnt={diag['RxCnt']} "
                        f"RxMissCnt={diag['RxMissCnt']} "
                        f"LoopbackDropCnt={diag['LoopbackDropCnt']}"
                    )

                    if missing == [oversize]:
                        print(
                            "Oversized frame dropped alone; reception "
                            "recovered on the next frame."
                        )
                        print("Result         : PASS")
                        return 0
                    if missing and missing[0] == oversize and (
                        missing == list(range(oversize, missing[-1] + 1))
                    ):
                        print(
                            "REGRESSION: every frame after the oversized "
                            "one was lost; ISO OUT arming did not retry."
                        )
                    print("Result         : FAIL")
                    return 1
            finally:
                handle.close()

    except (RuntimeError, usb1.USBError) as exc:
        print(f"Error: {exc}", file=sys.stderr)
        print("Result         : FAIL")
        return 1


if __name__ == "__main__":
    raise SystemExit(main())
