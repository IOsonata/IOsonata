#!/usr/bin/env python3
"""Concurrent firmware stress test for exemples/usb/usb_combo_stress.cpp.

All workers run for the same interval. Host handles are deliberately independent;
the purpose is to keep every device-side USB path busy at once, not to benchmark
Python-side lock contention.
"""

import argparse
import os
import shutil
import subprocess
import sys
import re
import struct
import threading
import time

try:
    import serial
    import serial.tools.list_ports
except ImportError:
    serial = None

try:
    import hid
except ImportError:
    hid = None

try:
    import usb1
except ImportError:
    usb1 = None

import usb_iso_loopback as iso_test


DEFAULT_VID = 0x1209
DEFAULT_PID = 0x0008
DEFAULT_PRODUCT = "IOsonata USB Combo Stress"

HID_REPORT_SIZE = 64
INT_MPS = 64
INT_ALT = 1
ISO_ALT = 6
ISO_MPS = 63

USB_ENDPOINT_TRANSFER_TYPE_MASK = 0x03
USB_ENDPOINT_TRANSFER_TYPE_INT = 0x03

# After a system sleep the process can run again before the host has resumed
# the bus. A failure this long after the wake belongs to the sleep.
HOST_WAKE_GRACE_S = 10.0

ISO_DIAG_REQUEST = 0x5A
ISO_DIAG_FORMAT = "<5I"
ISO_DIAG_SIZE = struct.calcsize(ISO_DIAG_FORMAT)
# Controller-level ISO scheduling counters. Only the TinyUSB comparison
# firmware answers this request; the IOsonata firmware stalls it.
ISO_TRACE_REQUEST = 0x5C
ISO_TRACE_HDR_FORMAT = "<4H"
ISO_TRACE_ENTRY_FORMAT = "<HHBBBBHH"
ISO_TRACE_ENTRY_SIZE = struct.calcsize(ISO_TRACE_ENTRY_FORMAT)
ISO_TRACE_COUNT = 128
ISO_TRACE_SIZE = (
    struct.calcsize(ISO_TRACE_HDR_FORMAT) + ISO_TRACE_COUNT * ISO_TRACE_ENTRY_SIZE
)
# Flag bits of a trace entry, in the order they print. Lower case marks the
# OUT direction.
ISO_TRACE_FLAGS = (
    (0x01, "O"),  # IN frame offered at SOF
    (0x02, "R"),  # offer refused, previous IN still pending
    (0x04, "B"),  # DMA channel busy at the offer
    (0x08, "E"),  # ... held by EP0
    (0x10, "S"),  # STARTISOIN issued
    (0x20, "D"),  # ENDISOIN retired
    (0x40, "s"),  # STARTISOOUT issued
    (0x80, "d"),  # ENDISOOUT retired
)
ISO_DCD_DIAG_REQUEST = 0x5B
ISO_DCD_DIAG_FORMAT = "<11I"
ISO_DCD_DIAG_SIZE = struct.calcsize(ISO_DCD_DIAG_FORMAT)
ISO_DCD_DIAG_NAMES = (
    "in_idle", "in_wait", "in_carry", "out_idle", "out_carry", "out_unarmed",
    "in_spin_out", "in_start_max_us", "in_end_max_us", "in_end_slow",
    "cbi_held"
)


BANNER = b"IOsonata USB Combo Stress"


def parse_int(value):
    return int(value, 0)


def prbs8(curval):
    newbit = ((curval >> 6) ^ (curval >> 5)) & 1
    return ((curval << 1) | newbit) & 0x7f


def make_prbs_cycle():
    # The existing 7-bit pattern repeats after 127 nonzero states.
    state = 0x7f
    data = bytearray()
    for _ in range(127):
        state = prbs8(state)
        data.append(state)
    return bytes(data)


PRBS_CYCLE = make_prbs_cycle()
PRBS_NEXT = bytes(prbs8(value) for value in range(256))


def make_prbs_block(state, length):
    if length <= 0:
        return bytes(length), state
    first = prbs8(state)
    if first == 0:
        return bytes(length), 0
    offset = PRBS_CYCLE.index(first)
    cycle = PRBS_CYCLE[offset:] + PRBS_CYCLE[:offset]
    whole, tail = divmod(length, len(cycle))
    data = cycle * whole + cycle[:tail]
    return data, data[-1]


def check_stream(data, expected):
    if not data:
        return expected, 0
    # Translate and compare whole blocks in C on the clean path. Each next
    # expectation still follows the received byte, including corrupt bytes.
    following = data.translate(PRBS_NEXT)
    errors = int(data[0] != expected)
    received = data[1:]
    wanted = following[:-1]
    if received != wanted:
        errors += sum(value != next_value
                      for value, next_value in zip(received, wanted))
    return following[-1], errors


def check_prbs_stream(data, expected):
    # Zero is a target RX error marker and does not advance the PRBS state.
    target_errors = data.count(0)
    if target_errors:
        data = data.replace(b"\x00", b"")
    if not data:
        return expected, 0, target_errors
    if expected is None:
        expected = data[0]
    expected, errors = check_stream(data, expected)
    return expected, errors, target_errors


def hid_payload(sequence):
    data = bytearray(
        ((0xA7 ^ (sequence * 23) ^ (index * 31)) & 0xFF)
        for index in range(HID_REPORT_SIZE)
    )
    data[0:4] = sequence.to_bytes(4, "little")
    return bytes(data)


def int_payload(sequence):
    data = bytearray(
        ((0x5D ^ (sequence * 29) ^ (index * 17)) & 0xFF)
        for index in range(INT_MPS)
    )
    data[0:4] = sequence.to_bytes(4, "little")
    return bytes(data)


def fetch_iso_trace(handle, interface, timeout_ms):
    """Controller per-frame ISO trace as a list of dicts, oldest first.

    Returns (entries, None) or (None, reason).
    """
    try:
        raw = bytes(
            handle.controlRead(
                0xC1,
                ISO_TRACE_REQUEST,
                0,
                interface,
                ISO_TRACE_SIZE,
                timeout=timeout_ms,
            )
        )
    except usb1.USBError as exc:
        return None, f"trace read failed: {exc}"
    if len(raw) != ISO_TRACE_SIZE:
        return None, f"trace length {len(raw)}/{ISO_TRACE_SIZE}"

    hdr_size = struct.calcsize(ISO_TRACE_HDR_FORMAT)
    nxt, count, entry_size, version = struct.unpack(
        ISO_TRACE_HDR_FORMAT, raw[:hdr_size]
    )
    if version != 2 or entry_size != ISO_TRACE_ENTRY_SIZE or count != ISO_TRACE_COUNT:
        return None, f"trace layout v{version} entry {entry_size} count {count}"

    entries = []
    for i in range(count):
        index = (nxt + i) % count
        offset = hdr_size + index * entry_size
        frame, sof_us, start_us, end_us, in_len, flags, out_len, _ = struct.unpack(
            ISO_TRACE_ENTRY_FORMAT, raw[offset:offset + entry_size]
        )
        entries.append({
            "frame": frame, "sof_us": sof_us, "start_us": start_us,
            "end_us": end_us, "in": in_len, "out": out_len, "flags": flags,
        })
    return entries, None


def format_iso_trace(entries):
    """One text line per frame.

    Columns: bus frame number, microseconds since the previous SOF mark
    (about 1000; 2000 means the device saw no SOF for one frame, more than
    that means the interrupt was held off past the next SOF), SOF to
    STARTISOIN in us, SOF to ENDISOIN in us (0 when it did not happen in
    that frame), IN bytes offered, OUT bytes read, flags (see
    ISO_TRACE_FLAGS, '-' when clear). A line ending in '<< gap' follows a
    jump in the frame number.
    """
    lines = ["frame sof_us start_us end_us in out flags"]
    previous = None
    for e in entries:
        text = "".join(
            letter if e["flags"] & bit else "-" for bit, letter in ISO_TRACE_FLAGS
        )
        gap = ""
        if previous is not None and ((e["frame"] - previous) & 0x7FF) != 1:
            gap = "  << gap"
        previous = e["frame"]
        lines.append(
            f"{e['frame']:5d} {e['sof_us']:6d} {e['start_us']:8d} "
            f"{e['end_us']:6d} {e['in']:3d} {e['out']:3d} {text}{gap}"
        )
    return lines


MISSING_RE = re.compile(r"^missing validation frame\(s\) \[([0-9, ]*)\]")


def link_loss_frames(error, entries, guard, mps):
    """Classify every missing validation frame of a failed burst as a link
    loss, or return None when any one of them is not explained that way.

    The failed burst is the last run of trace entries that read OUT data.
    OUT packet k of the burst is read at the SOF of frame first + k, so a
    missing validation index m belongs to frame first + guard + m. That
    frame is a link loss when either:
      "no SOF":    it has no trace entry, the device raised no SOF for it and
                   the packet was replaced by the next one unread;
      "no packet": the device saw the SOF and read the OUT buffer, and the
                   hardware reported no full packet, while the burst
                   around it carried full packets. ISO OUT has no
                   handshake, so the host reports these as sent.
    Returns a list of (frame, kind).
    """
    match = MISSING_RE.match(error or "")
    if match is None or not entries:
        return None
    missing = [int(v) for v in match.group(1).split(",") if v.strip()]
    if not missing or len(missing) >= 16:
        return None     # empty, or the list may have been truncated

    # The burst: from the first entry that read a full packet to the last,
    # allowing single entries inside it that read nothing.
    last = None
    for i in range(len(entries) - 1, -1, -1):
        if entries[i]["out"] == mps:
            last = i
            break
    if last is None:
        return None
    first = last
    while first > 0 and (entries[first - 1]["out"] == mps or (
            first > 1 and entries[first - 2]["out"] == mps)):
        first -= 1
    while entries[first]["out"] != mps:
        first += 1
    start = entries[first]["frame"]
    span = (entries[last]["frame"] - start) & 0x7FF
    by_frame = {e["frame"]: e for e in entries[first:last + 1]}

    result = []
    for m in missing:
        frame = (start + guard + m) & 0x7FF
        if ((frame - start) & 0x7FF) > span:
            return None
        entry = by_frame.get(frame)
        if entry is None:
            result.append((frame, "no SOF"))
        elif entry["out"] == 0 and 900 <= entry["sof_us"] <= 1100:
            result.append((frame, "no packet"))
        else:
            return None
    return result


def read_iso_diag(handle, interface, timeout_ms):
    raw = bytes(
        handle.controlRead(
            0xC1,
            ISO_DIAG_REQUEST,
            0,
            interface,
            ISO_DIAG_SIZE,
            timeout=timeout_ms,
        )
    )
    if len(raw) != ISO_DIAG_SIZE:
        return f"diag length {len(raw)}/{ISO_DIAG_SIZE}"
    rx_miss, tx_miss, loop_drop, rx_empty, tx_empty = struct.unpack(
        ISO_DIAG_FORMAT, raw
    )
    text = (
        f"rx_miss={rx_miss} tx_miss={tx_miss} "
        f"loop_drop={loop_drop} rx_empty={rx_empty} tx_empty={tx_empty}"
    )
    try:
        raw = bytes(
            handle.controlRead(
                0xC1,
                ISO_DCD_DIAG_REQUEST,
                0,
                interface,
                ISO_DCD_DIAG_SIZE,
                timeout=timeout_ms,
            )
        )
    except usb1.USBError:
        return text
    if len(raw) != ISO_DCD_DIAG_SIZE:
        return text
    values = struct.unpack(ISO_DCD_DIAG_FORMAT, raw)
    return text + "; dcd " + " ".join(
        f"{name}={value}" for name, value in zip(ISO_DCD_DIAG_NAMES, values)
    )




class Stats:
    NAMES = ("loop_tx", "loop_rx", "prbs_rx", "hid", "int", "iso")

    def __init__(self):
        self.lock = threading.Lock()
        self.count = {name: 0 for name in self.NAMES}
        self.last = {name: None for name in self.NAMES}
        self.loop_errors = 0
        self.prbs_errors = 0
        self.target_errors = 0
        self.path_errors = {"hid": 0, "int": 0, "iso": 0}
        self.iso_host_misses = 0
        self.iso_host_skews = 0
        self.iso_host_unsent = 0
        self.iso_sof_losses = 0
        self.iso_out_losses = 0
        self.host_pauses = 0
        self.host_pause_sec = 0.0
        self.host_pause_windows = []
        self.failure = None
        self.failure_at = None

    def add_host_pause(self, seconds, start, end, asleep=False):
        """The host process was not scheduled for `seconds`.

        No path could make progress during that time, so the pause is not
        counted against any of them: every progress timestamp moves forward
        by the pause. `asleep` marks a system sleep.
        """
        with self.lock:
            self.host_pauses += 1
            self.host_pause_sec += seconds
            self.host_pause_windows.append((start, end, seconds, asleep))
            for name, when in self.last.items():
                if when is not None:
                    self.last[name] = when + seconds
            return self.host_pauses

    def host_pause_at(self, when, grace):
        """The pause, if any, during which or right after which `when` falls.

        A worker blocked in a transfer when the host stops times out as soon as
        the host runs again, so a failure up to `grace` seconds after the end
        of a pause belongs to that pause. After a system sleep the process can
        run before the host has resumed the bus, so that window is at least
        HOST_WAKE_GRACE_S long.
        """
        with self.lock:
            for start, end, seconds, asleep in self.host_pause_windows:
                limit = max(grace, HOST_WAKE_GRACE_S) if asleep else grace
                if start <= when <= end + limit:
                    return seconds
            return None

    def host_pause_summary(self):
        with self.lock:
            return self.host_pauses, self.host_pause_sec

    def add_iso_host_miss(self):
        with self.lock:
            self.iso_host_misses += 1
            return self.iso_host_misses

    def iso_host_miss_count(self):
        with self.lock:
            return self.iso_host_misses

    def add_iso_host_skew(self):
        with self.lock:
            self.iso_host_skews += 1
            return self.iso_host_skews

    def iso_host_skew_count(self):
        with self.lock:
            return self.iso_host_skews

    def add_iso_host_unsent(self, frames):
        with self.lock:
            self.iso_host_unsent += frames
            return self.iso_host_unsent

    def iso_host_unsent_count(self):
        with self.lock:
            return self.iso_host_unsent

    def add_iso_link_loss(self, losses):
        with self.lock:
            for _, kind in losses:
                if kind == "no SOF":
                    self.iso_sof_losses += 1
                else:
                    self.iso_out_losses += 1
            return self.iso_sof_losses + self.iso_out_losses

    def iso_sof_loss_count(self):
        with self.lock:
            return self.iso_sof_losses

    def iso_out_loss_count(self):
        with self.lock:
            return self.iso_out_losses

    def add(self, name, count, errors=0, target_errors=0):
        with self.lock:
            self.count[name] += count
            self.last[name] = time.monotonic()
            if name == "loop_rx":
                self.loop_errors += errors
            elif name == "prbs_rx":
                self.prbs_errors += errors
                self.target_errors += target_errors

    def fail(self, name, error):
        with self.lock:
            key = name.lower()
            if key in self.path_errors:
                self.path_errors[key] += 1
            if self.failure is None:
                self.failure = f"{name}: {error}"
                self.failure_at = time.monotonic()

    def failure_timestamp(self):
        with self.lock:
            return self.failure_at

    def snapshot(self):
        with self.lock:
            return (
                dict(self.count),
                dict(self.last),
                self.loop_errors,
                self.prbs_errors,
                self.target_errors,
                dict(self.path_errors),
                self.failure,
            )


def open_cdc(port, baud):
    return serial.Serial(
        port=port,
        baudrate=baud,
        timeout=0.1,
        write_timeout=1.0,
        rtscts=False,
        dsrdtr=False,
    )


def prepare_cdc(loop_comm, prbs_comm):
    for comm in (loop_comm, prbs_comm):
        try:
            comm.dtr = False
        except (OSError, serial.SerialException):
            pass
        comm.reset_input_buffer()
        comm.reset_output_buffer()

    try:
        loop_comm.dtr = True
    except (OSError, serial.SerialException):
        pass

    deadline = time.monotonic() + 1.0
    received = bytearray()
    while time.monotonic() < deadline:
        waiting = loop_comm.in_waiting
        data = loop_comm.read(waiting if waiting > 0 else 1)
        if data:
            received.extend(data)
            if BANNER in received:
                break
    loop_comm.reset_input_buffer()

    try:
        prbs_comm.dtr = True
    except (OSError, serial.SerialException):
        pass


def port_serial(port):
    """USB serial number of the device behind a serial port, None if unknown."""
    path = os.path.realpath(port)
    for info in serial.tools.list_ports.comports():
        if info.device == port or os.path.realpath(info.device) == path:
            return info.serial_number
    return None


def find_hid(vid, pid, product, serial_number):
    matches = []
    for info in hid.enumerate(vid, pid):
        if product is not None and (info.get("product_string") or "") != product:
            continue
        if serial_number is not None and (info.get("serial_number") or "") != serial_number:
            continue
        matches.append(info)
    if len(matches) != 1:
        found = ", ".join(
            info.get("serial_number") or "?" for info in matches
        )
        raise RuntimeError(
            f"expected one HID interface, found {len(matches)}"
            + (f" (serial {found})" if matches else "")
        )
    return matches[0]


def find_usb(context, vid, pid, serial_number):
    """The one USB device with this VID, PID and, when given, serial number."""
    matches = []
    for device in context.getDeviceIterator(skip_on_error=True):
        if device.getVendorID() != vid or device.getProductID() != pid:
            device.close()
            continue
        if serial_number is not None:
            try:
                number = device.getSerialNumber()
            except usb1.USBError:
                number = None
            if number != serial_number:
                device.close()
                continue
        matches.append(device)
    if len(matches) != 1:
        for device in matches:
            device.close()
        raise RuntimeError(
            f"expected one USB device {vid:04x}:{pid:04x}"
            + (f" serial {serial_number}" if serial_number else "")
            + f", found {len(matches)}"
        )
    return matches[0]


def discover_interrupt_loopback(device):
    candidates = {}
    for setting in device.iterSettings():
        interface = setting.getNumber()
        alt = setting.getAlternateSetting()
        if alt < 1 or alt > 3 or setting.getClass() != 0xFF:
            continue

        ep_in = None
        ep_out = None
        for endpoint in setting.iterEndpoints():
            if (
                endpoint.getAttributes() & USB_ENDPOINT_TRANSFER_TYPE_MASK
            ) != USB_ENDPOINT_TRANSFER_TYPE_INT:
                continue
            if endpoint.getAddress() & usb1.ENDPOINT_IN:
                ep_in = endpoint
            else:
                ep_out = endpoint

        if ep_in is None or ep_out is None:
            continue

        in_no = ep_in.getAddress() & 0x0F
        out_no = ep_out.getAddress() & 0x0F
        if (
            in_no == 0
            or in_no != out_no
            or ep_in.getMaxPacketSize() != INT_MPS
            or ep_out.getMaxPacketSize() != INT_MPS
        ):
            continue
        candidates.setdefault((interface, in_no), set()).add(alt)

    matches = [
        key for key, alts in candidates.items()
        if {1, 2, 3}.issubset(alts)
    ]
    if len(matches) != 1:
        raise RuntimeError(
            f"expected one raw interrupt loopback interface, found {len(matches)}"
        )
    return matches[0]


def loop_tx_worker(comm, start, stop, stats, block_size, tx_stop):
    state = 0xFF
    pending = b""
    start.wait()
    try:
        while not stop.is_set() and not tx_stop.is_set():
            if not pending:
                pending, state = make_prbs_block(state, block_size)
            written = comm.write(pending)
            if written > 0:
                stats.add("loop_tx", written)
                pending = pending[written:]
    except Exception as exc:
        stats.fail("CDC loop TX", exc)
        stop.set()


def loop_rx_worker(comm, start, stop, stats, read_size):
    expected = prbs8(0xFF)
    start.wait()
    try:
        while not stop.is_set():
            data = comm.read(read_size)
            if data:
                expected, errors = check_stream(data, expected)
                stats.add("loop_rx", len(data), errors=errors)
    except Exception as exc:
        stats.fail("CDC loop RX", exc)
        stop.set()


def prbs_rx_worker(comm, start, stop, stats, read_size):
    expected = None
    start.wait()
    try:
        while not stop.is_set():
            data = comm.read(read_size)
            if data:
                expected, errors, target_errors = check_prbs_stream(
                    data, expected
                )
                stats.add(
                    "prbs_rx", len(data), errors=errors,
                    target_errors=target_errors
                )
    except Exception as exc:
        stats.fail("CDC PRBS RX", exc)
        stop.set()


def hid_worker(vid, pid, product, serial_number, start, stop, stats, timeout_ms):
    device = None
    try:
        info = find_hid(vid, pid, product, serial_number)
        device = hid.device()
        device.open_path(info["path"])
        device.set_nonblocking(0)
        sequence = 0
        start.wait()
        while not stop.is_set():
            data = hid_payload(sequence)
            written = device.write(bytes((0,)) + data)
            if written not in (len(data), len(data) + 1):
                raise RuntimeError(f"short write {written}")
            reply = bytes(device.read(HID_REPORT_SIZE, timeout_ms))
            if reply != data:
                raise RuntimeError("loopback mismatch")
            stats.add("hid", len(data))
            sequence += 1
    except Exception as exc:
        stats.fail("HID", exc)
        stop.set()
    finally:
        if device is not None:
            device.close()


def int_worker(vid, pid, serial_number, start, stop, stats, timeout_ms):
    try:
        with usb1.USBContext() as context:
            device = find_usb(context, vid, pid, serial_number)
            interface, ep = discover_interrupt_loopback(device)
            handle = device.open()
            try:
                with handle.claimInterface(interface):
                    handle.setInterfaceAltSetting(interface, INT_ALT)
                    sequence = 0
                    start.wait()
                    while not stop.is_set():
                        data = int_payload(sequence)
                        written = handle.interruptWrite(
                            ep, data, timeout=timeout_ms
                        )
                        if written != len(data):
                            raise RuntimeError(
                                f"short write {written}/{len(data)}"
                            )
                        reply = bytes(
                            handle.interruptRead(
                                usb1.ENDPOINT_IN | ep,
                                INT_MPS,
                                timeout=timeout_ms,
                            )
                        )
                        if reply != data:
                            raise RuntimeError("loopback mismatch")
                        stats.add("int", len(data))
                        sequence += 1
                    handle.setInterfaceAltSetting(interface, 0)
            finally:
                handle.close()
    except Exception as exc:
        stats.fail("INT", exc)
        stop.set()


def iso_worker(vid, pid, serial_number, start, stop, stats, timeout_ms, rounds):
    try:
        with usb1.USBContext() as context:
            device = find_usb(context, vid, pid, serial_number)
            interface, ep = iso_test.discover_iso_loopback(device)
            handle = device.open()
            try:
                with handle.claimInterface(interface):
                    handle.setInterfaceAltSetting(interface, ISO_ALT)
                    sequence = 0
                    start.wait()
                    misses = 0
                    while not stop.is_set():
                        error, result = iso_test.run_burst(
                            context,
                            handle,
                            ep,
                            ISO_ALT,
                            ISO_MPS,
                            ISO_MPS,
                            rounds,
                            timeout_ms,
                            sequence,
                        )
                        if iso_test.is_host_sched_error(error):
                            # The host refused or failed the request, did not
                            # send some of its validation frames (miss), or
                            # ran OUT late against IN so the IN window closed
                            # before the last echoes (skew). Count it, move
                            # the sequence on so any queued echoes read as
                            # stale, and resubmit.
                            misses += 1
                            if iso_test.is_host_sched_skew_error(error):
                                total = stats.add_iso_host_skew()
                            else:
                                total = stats.add_iso_host_miss()
                            print(
                                f"ISO {error} (#{total}, "
                                f"{time.strftime('%H:%M:%S')}); resubmitting",
                                file=sys.stderr,
                                flush=True,
                            )
                            sequence += rounds + (
                                2 * iso_test.BURST_GUARD_FRAMES
                            )
                            if misses <= iso_test.HOST_SCHED_MISS_RETRIES:
                                continue
                            error = (
                                f"{misses} consecutive host scheduling "
                                f"misses; last: {error}"
                            )
                        else:
                            misses = 0
                        if error is not None:
                            # The controller's last 128 frames, oldest
                            # first; the failed burst is inside them.
                            entries, trace_error = fetch_iso_trace(
                                handle, interface, timeout_ms
                            )
                            lost = link_loss_frames(
                                error, entries, iso_test.BURST_GUARD_FRAMES,
                                ISO_MPS
                            )
                            if lost is not None:
                                # The device never saw the SOF, or never
                                # received the packet, for those frames. A
                                # link-level loss, not a stack failure.
                                total = stats.add_iso_link_loss(lost)
                                text = ", ".join(
                                    f"{frame} {kind}" for frame, kind in lost
                                )
                                print(
                                    f"ISO link loss: frame(s) {text} "
                                    f"(#{total}, {time.strftime('%H:%M:%S')})",
                                    file=sys.stderr,
                                    flush=True,
                                )
                                stats.add(
                                    "iso", (rounds - len(lost)) * ISO_MPS
                                )
                                sequence += rounds + (
                                    2 * iso_test.BURST_GUARD_FRAMES
                                )
                                continue
                            try:
                                diag = read_iso_diag(
                                    handle, interface, timeout_ms
                                )
                            except Exception as diag_exc:
                                diag = f"diag read failed: {diag_exc}"
                            print("ISO controller frame trace:", file=sys.stderr)
                            lines = (format_iso_trace(entries)
                                     if entries is not None else [trace_error])
                            for line in lines:
                                print("  " + line, file=sys.stderr)
                            sys.stderr.flush()
                            raise RuntimeError(f"{error}; {diag}")
                        stats.add("iso", result["matched"] * ISO_MPS)
                        if result.get("out_unsent"):
                            # Guard frames the host did not send; the
                            # validation frames all went out and came back.
                            stats.add_iso_host_unsent(result["out_unsent"])
                        sequence += rounds + (
                            2 * iso_test.BURST_GUARD_FRAMES
                        )
                    handle.setInterfaceAltSetting(interface, 0)
            finally:
                handle.close()
    except Exception as exc:
        stats.fail("ISO", exc)
        stop.set()


def keep_host_awake():
    """Keep a macOS host from idle or system sleep while this process runs.

    A host that sleeps in the middle of a run stops every path at once and the
    run fails with timeouts that say nothing about the device. caffeinate
    exits with this process. Closing a laptop lid still sleeps the host.
    """
    if sys.platform != "darwin" or shutil.which("caffeinate") is None:
        return None
    try:
        return subprocess.Popen(
            ["caffeinate", "-i", "-m", "-s", "-w", str(os.getpid())],
            stdout=subprocess.DEVNULL,
            stderr=subprocess.DEVNULL,
        )
    except OSError:
        return None


def stop_workers(workers, tx_stop, stop, stats, iso_stop=None):
    # Like the dual-CDC test, keep RX and the other interfaces running until
    # the final CDC write returns. Stopping RX first can block that write
    # through loopback backpressure and manufacture a shutdown timeout.
    stop_started = time.monotonic()
    tx_stop.set()
    workers[0].join(timeout=1.5)  # Longer than the serial write timeout.
    if workers[0].is_alive():
        stats.fail("CDC loop TX", "writer did not stop")
    test_end = time.monotonic()
    # ISO next, before the INT worker selects alternate setting 0. Closing
    # an OUT endpoint with a transfer armed can need a global OUT NAK on the
    # device, and ISO OUT data arriving meanwhile is dropped, so the last
    # ISO burst must finish first.
    if iso_stop is not None:
        iso_stop.set()
        for worker in workers[1:]:
            if worker.name == "ISO":
                worker.join(timeout=2.0)
    stop.set()
    for worker in workers[1:]:
        worker.join(timeout=2.0)
        if worker.is_alive():
            stats.fail(worker.name, "worker did not stop")
    return stop_started, test_end


def main():
    parser = argparse.ArgumentParser(
        description="Stress all IOsonata USB combo functions concurrently"
    )
    parser.add_argument("--loop-port", required=True)
    parser.add_argument("--prbs-port", required=True)
    parser.add_argument("--vid", type=parse_int, default=DEFAULT_VID)
    parser.add_argument("--pid", type=parse_int, default=DEFAULT_PID)
    parser.add_argument("--product", default=DEFAULT_PRODUCT)
    parser.add_argument("--serial")
    parser.add_argument("--baud", type=int, default=1_000_000)
    parser.add_argument("--duration", type=float, default=60.0)
    parser.add_argument("--block", type=int, default=4096)
    parser.add_argument("--read-size", type=int, default=4096)
    parser.add_argument("--timeout", type=int, default=1000)
    parser.add_argument("--iso-rounds", type=int, default=32)
    parser.add_argument("--report", type=float, default=1.0)
    parser.add_argument("--stall-timeout", type=float, default=3.0)
    parser.add_argument(
        "--allow-sleep",
        action="store_true",
        help="do not keep a macOS host awake during the run",
    )
    args = parser.parse_args()

    missing = []
    if serial is None:
        missing.append("pyserial")
    if hid is None:
        missing.append("hidapi")
    if usb1 is None or iso_test.usb1 is None:
        missing.append("libusb1")
    if missing:
        print("Missing Python package(s): " + ", ".join(missing), file=sys.stderr)
        return 2

    if (
        args.loop_port == args.prbs_port
        or args.duration <= 0
        or args.block <= 0
        or args.read_size <= 0
        or args.timeout <= 0
        or args.iso_rounds <= 0
        or args.report <= 0
        or args.stall_timeout <= 0
    ):
        parser.error("invalid argument")

    # Every path runs on the board behind --loop-port: several boards with
    # the same VID, PID and product can be attached at once.
    if args.serial is None:
        args.serial = port_serial(args.loop_port)
        prbs_serial = port_serial(args.prbs_port)
        if args.serial is not None and prbs_serial not in (None, args.serial):
            parser.error("--loop-port and --prbs-port are on different devices")
    print(f"Device serial   : {args.serial or 'unknown, one board only'}")

    awake = None if args.allow_sleep else keep_host_awake()
    if awake is not None:
        print("Host sleep      : held off with caffeinate for the run")

    loop_comm = None
    prbs_comm = None
    stop = threading.Event()
    tx_stop = threading.Event()
    # The ISO worker stops on its own event, ahead of the other interfaces.
    iso_stop = threading.Event()
    start = threading.Event()
    stats = Stats()

    try:
        loop_comm = open_cdc(args.loop_port, args.baud)
        prbs_comm = open_cdc(args.prbs_port, args.baud)
        prepare_cdc(loop_comm, prbs_comm)

        workers = [
            threading.Thread(
                name="CDC-loop-TX",
                target=loop_tx_worker,
                args=(loop_comm, start, stop, stats, args.block, tx_stop),
                daemon=True,
            ),
            threading.Thread(
                name="CDC-loop-RX",
                target=loop_rx_worker,
                args=(loop_comm, start, stop, stats, args.read_size),
                daemon=True,
            ),
            threading.Thread(
                name="CDC-PRBS-RX",
                target=prbs_rx_worker,
                args=(prbs_comm, start, stop, stats, args.read_size),
                daemon=True,
            ),
            threading.Thread(
                name="HID",
                target=hid_worker,
                args=(
                    args.vid,
                    args.pid,
                    args.product,
                    args.serial,
                    start,
                    stop,
                    stats,
                    args.timeout,
                ),
                daemon=True,
            ),
            threading.Thread(
                name="INT",
                target=int_worker,
                args=(
                    args.vid,
                    args.pid,
                    args.serial,
                    start,
                    stop,
                    stats,
                    args.timeout,
                ),
                daemon=True,
            ),
            threading.Thread(
                name="ISO",
                target=iso_worker,
                args=(
                    args.vid,
                    args.pid,
                    args.serial,
                    start,
                    iso_stop,
                    stats,
                    args.timeout,
                    args.iso_rounds,
                ),
                daemon=True,
            ),
        ]

        for worker in workers:
            worker.start()

        # Give workers time to open/claim their independent host interfaces,
        # then release them together into the firmware stress phase.
        time.sleep(0.25)
        test_start = time.monotonic()
        report_start = test_start
        previous = {name: 0 for name in Stats.NAMES}
        start.set()

        tick = time.monotonic()
        wall_tick = time.time()
        # A worker timeout shorter than this is a host pause long enough to
        # fail a transfer.
        pause_limit = min(args.stall_timeout, args.timeout / 1000.0, 1.0) / 2
        while time.monotonic() - test_start < args.duration:
            time.sleep(0.02)
            now = time.monotonic()
            wall_now = time.time()
            # This loop only sleeps 20 ms per pass. A longer pass means the
            # host process itself was not running (system sleep, App Nap,
            # scheduler); no path could move, device or not. The monotonic
            # clock may not count system sleep, the wall clock does.
            gap = max(now - tick, wall_now - wall_tick)
            # Only a system sleep moves the wall clock past the monotonic one.
            asleep = (wall_now - wall_tick) - (now - tick) >= pause_limit
            pause_start = tick
            tick = now
            wall_tick = wall_now
            if gap >= pause_limit:
                total = stats.add_host_pause(gap, pause_start, now, asleep)
                print(
                    f"Host paused {gap:.1f} s (#{total}, "
                    f"{time.strftime('%H:%M:%S')}"
                    f"{', system sleep' if asleep else ''}); "
                    "not counted as a stall",
                    file=sys.stderr,
                    flush=True,
                )
                # Checked after the pause is recorded: a worker that timed
                # out because of the pause has already stopped the run.
                if stop.is_set():
                    break
                continue

            if stop.is_set():
                break

            count, last, loop_errors, prbs_errors, target_errors, path_errors, failure = (
                stats.snapshot()
            )

            if failure is not None:
                break

            if now - test_start >= args.stall_timeout:
                for name in Stats.NAMES:
                    when = last[name]
                    if when is None or now - when >= args.stall_timeout:
                        ages = ", ".join(
                            f"{other} "
                            + ("never" if last[other] is None
                               else f"{now - last[other]:.1f} s")
                            for other in Stats.NAMES
                        )
                        print(
                            f"Stall: {name}; time since last progress: {ages}",
                            file=sys.stderr,
                            flush=True,
                        )
                        stats.fail(name, "stalled")
                        stop.set()
                        break

            if now - report_start >= args.report:
                elapsed = now - report_start
                rates = {
                    name: (count[name] - previous[name]) / elapsed
                    for name in Stats.NAMES
                }
                previous = count
                report_start = now
                print(
                    "CDC loop %.0f/%.0f B/s err %d | PRBS %.0f B/s err %d/%d | "
                    "HID %.0f B/s err %d | INT %.0f B/s err %d | "
                    "ISO %.0f B/s err %d"
                    % (
                        rates["loop_tx"],
                        rates["loop_rx"],
                        loop_errors,
                        rates["prbs_rx"],
                        prbs_errors,
                        target_errors,
                        rates["hid"],
                        path_errors["hid"],
                        rates["int"],
                        path_errors["int"],
                        rates["iso"],
                        path_errors["iso"],
                    ),
                    flush=True,
                )

        stop_started, test_end = stop_workers(
            workers, tx_stop, stop, stats, iso_stop
        )

        count, _, loop_errors, prbs_errors, target_errors, path_errors, failure = (
            stats.snapshot()
        )
        pending = count["loop_tx"] - count["loop_rx"]
        elapsed = test_end - test_start
        rates = {
            name: count[name] / elapsed
            for name in Stats.NAMES
        }
        total_bytes = sum(count.values())
        total_rate = total_bytes / elapsed

        print()
        print(f"Elapsed sec     : {elapsed:.2f}")
        print(f"Loop TX bytes   : {count['loop_tx']}")
        print(f"Loop TX B/sec   : {rates['loop_tx']:.2f}")
        print(f"Loop RX bytes   : {count['loop_rx']}")
        print(f"Loop RX B/sec   : {rates['loop_rx']:.2f}")
        print(f"Loop pending    : {pending}")
        print(f"Loop errors     : {loop_errors}")
        print(f"PRBS RX bytes   : {count['prbs_rx']}")
        print(f"PRBS RX B/sec   : {rates['prbs_rx']:.2f}")
        print(f"PRBS errors     : {prbs_errors}")
        print(f"Target RX errors: {target_errors}")
        print(f"HID bytes       : {count['hid']}")
        print(f"HID B/sec       : {rates['hid']:.2f}")
        print(f"HID errors      : {path_errors['hid']}")
        print(f"INT bytes       : {count['int']}")
        print(f"INT B/sec       : {rates['int']:.2f}")
        print(f"INT errors      : {path_errors['int']}")
        print(f"ISO bytes       : {count['iso']}")
        print(f"ISO B/sec       : {rates['iso']:.2f}")
        print(f"ISO errors      : {path_errors['iso']}")
        print(f"ISO host misses : {stats.iso_host_miss_count()}")
        print(f"ISO host skews  : {stats.iso_host_skew_count()}")
        print(f"ISO host unsent : {stats.iso_host_unsent_count()}")
        print(f"ISO SOF losses  : {stats.iso_sof_loss_count()}")
        print(f"ISO OUT losses  : {stats.iso_out_loss_count()}")
        pauses, pause_sec = stats.host_pause_summary()
        print(f"Host pauses     : {pauses} ({pause_sec:.1f} s)")
        print(f"Total bytes     : {total_bytes}")
        print(f"Total B/sec     : {total_rate:.2f}")
        host_paused = None
        if failure is not None:
            print(f"Failure         : {failure}")
            failure_at = stats.failure_timestamp()
            phase = ("traffic" if failure_at < stop_started else
                     "final CDC write" if failure_at < test_end else
                     "interface shutdown")
            print(f"Failure time    : {failure_at - test_start:.2f} s ({phase})")
            host_paused = stats.host_pause_at(
                failure_at, args.timeout / 1000.0 + 1.0
            )
            if host_paused is not None:
                print(
                    f"Failure cause   : host paused {host_paused:.1f} s; "
                    "every transfer in flight timed out, not a device result"
                )

        passed = (
            failure is None
            and loop_errors == 0
            and prbs_errors == 0
            and target_errors == 0
            and all(value == 0 for value in path_errors.values())
            and pending >= 0
            and count["loop_rx"] > 0
            and count["prbs_rx"] > 0
            and count["hid"] > 0
            and count["int"] > 0
            and count["iso"] > 0
        )
        if not passed and host_paused is not None:
            print("Result          : INCONCLUSIVE (host paused)")
            return 3
        print("Result          : " + ("PASS" if passed else "FAIL"))
        return 0 if passed else 1

    finally:
        stop.set()
        iso_stop.set()
        if loop_comm is not None:
            loop_comm.close()
        if prbs_comm is not None:
            prbs_comm.close()
        if awake is not None:
            awake.terminate()


if __name__ == "__main__":
    raise SystemExit(main())
