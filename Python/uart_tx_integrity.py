#!/usr/bin/env python3
"""Host receiver for exemples/uart/uart_tx_integrity_test.cpp.

Fixed 64-byte IOTX frame: magic[4], sequence LE32, deterministic payload[54],
CRC16-CCITT-FALSE LE16. This checks received byte identity, not just line
formatting. Start this receiver before resetting the board.

Examples:
  python3 Python/uart_tx_integrity.py --port /dev/cu.usbmodem... --baud 1000000
  python3 Python/uart_tx_integrity.py --port /dev/cu.usbmodem... --baud 115200
  python3 Python/uart_tx_integrity.py --self-test
"""
import argparse
import collections
import sys
import time

MAGIC = b"IOTX"
FRAME_SIZE = 64
PAYLOAD_END = 62
DEFAULT_FRAMES = 2048


def crc16(data):
    crc = 0xFFFF
    for value in data:
        crc ^= value << 8
        for _ in range(8):
            crc = ((crc << 1) ^ (0x1021 if crc & 0x8000 else 0)) & 0xFFFF
    return crc


def frame_for(seq):
    data = bytearray(FRAME_SIZE)
    data[:4] = MAGIC
    data[4:8] = seq.to_bytes(4, "little")
    state = (seq ^ 0xA5C3D29F) & 0xFFFFFFFF
    for pos in range(8, PAYLOAD_END):
        state ^= (state << 13) & 0xFFFFFFFF
        state ^= state >> 17
        state ^= (state << 5) & 0xFFFFFFFF
        state &= 0xFFFFFFFF
        data[pos] = state & 0xFF
    data[62:64] = crc16(data[:62]).to_bytes(2, "little")
    return bytes(data)


class FrameChecker:
    def __init__(self, expected_frames):
        self.expected_frames = expected_frames
        self.pending = bytearray()
        self.seen = set()
        self.valid = 0
        self.duplicates = 0
        self.bad_frames = 0
        self.unframed_bytes = 0
        self.received_bytes = 0
        self.last_seq = -1
        self.backwards = 0
        self.bad_examples = []

    def feed(self, data):
        self.received_bytes += len(data)
        self.pending.extend(data)
        while True:
            index = self.pending.find(MAGIC)
            if index < 0:
                # Keep a possible partial magic across serial reads.
                skip = max(0, len(self.pending) - len(MAGIC) + 1)
                self.unframed_bytes += skip
                del self.pending[:skip]
                return
            if index:
                self.unframed_bytes += index
                del self.pending[:index]
            if len(self.pending) < FRAME_SIZE:
                return
            seq = int.from_bytes(self.pending[4:8], "little")
            candidate = bytes(self.pending[:FRAME_SIZE])
            if seq >= self.expected_frames or candidate != frame_for(seq):
                self.bad_frames += 1
                if len(self.bad_examples) < 8:
                    self.bad_examples.append((self.received_bytes - len(self.pending), seq))
                self.unframed_bytes += 1
                del self.pending[0]
                continue
            if seq in self.seen:
                self.duplicates += 1
            else:
                self.seen.add(seq)
            if seq < self.last_seq:
                self.backwards += 1
            self.last_seq = seq
            self.valid += 1
            del self.pending[:FRAME_SIZE]

    def result(self):
        missing = sorted(set(range(self.expected_frames)) - self.seen)
        spans = []
        for seq in missing:
            if spans and seq == spans[-1][1] + 1:
                spans[-1] = (spans[-1][0], seq)
            else:
                spans.append((seq, seq))
        return {
            "frames": self.valid,
            "unique": len(self.seen),
            "missing": len(missing),
            "missing_spans": spans,
            "duplicates": self.duplicates,
            "out_of_order": self.backwards,
            "bad_candidates": self.bad_frames,
            "unframed_bytes": self.unframed_bytes,
            "received_bytes": self.received_bytes,
        }


def self_test():
    frames = [frame_for(i) for i in range(12)]
    normal = FrameChecker(12)
    stream = b"".join(frames)
    for i in range(0, len(stream), 11):
        normal.feed(stream[i:i + 11])
    assert normal.result()["missing"] == 0
    assert normal.result()["bad_candidates"] == 0

    damaged = FrameChecker(12)
    stream = b"".join(frames[:3] + frames[4:7] + [frames[7], frames[7]] + frames[8:])
    stream = stream[:9 * FRAME_SIZE + 15] + stream[9 * FRAME_SIZE + 16:]
    for i in range(0, len(stream), 37):
        damaged.feed(stream[i:i + 37])
    result = damaged.result()
    assert 3 in [n for lo, hi in result["missing_spans"] for n in range(lo, hi + 1)]
    assert result["duplicates"] >= 1
    assert result["missing"] >= 2
    assert result["bad_candidates"] >= 1
    print("uart_tx_integrity self-test: PASS")


def receive(port, baud, frames, start_timeout, idle_timeout):
    try:
        import serial
    except ImportError as exc:
        raise SystemExit("pyserial required: python3 -m pip install pyserial") from exc

    checker = FrameChecker(frames)
    print("Listening on %s at %d 8N1; reset the board now." % (port, baud), flush=True)
    start = time.monotonic()
    last_data = start
    first_data = None
    with serial.Serial(port, baudrate=baud, timeout=0.1) as comm:
        while True:
            data = comm.read(4096)
            now = time.monotonic()
            if data:
                if first_data is None:
                    first_data = now
                checker.feed(data)
                last_data = now
            if len(checker.seen) == frames:
                break
            if first_data is None and now - start >= start_timeout:
                break
            if first_data is not None and now - last_data >= idle_timeout:
                break

    result = checker.result()
    print("Received %d raw bytes, %d valid frames (%d unique) / %d expected"
          % (result["received_bytes"], result["frames"], result["unique"], frames))
    print("Missing frames: %d; duplicates: %d; out-of-order: %d"
          % (result["missing"], result["duplicates"], result["out_of_order"]))
    print("Corrupt candidates: %d; unframed bytes: %d"
          % (result["bad_candidates"], result["unframed_bytes"]))
    if result["missing_spans"]:
        spans = result["missing_spans"][:16]
        print("Missing ranges (first 16): " +
              ", ".join(str(lo) if lo == hi else "%d-%d" % (lo, hi)
                        for lo, hi in spans))
    if checker.bad_examples:
        print("Corrupt candidate positions / decoded sequences: %s"
              % checker.bad_examples)
    ok = (result["missing"] == 0 and result["duplicates"] == 0
          and result["out_of_order"] == 0 and result["bad_candidates"] == 0
          and result["unframed_bytes"] == 0
          and result["received_bytes"] == FRAME_SIZE * frames)
    print("UART TX integrity: %s" % ("PASS" if ok else "FAIL"))
    return 0 if ok else 1


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--port", help="Serial port to receive binary test output")
    parser.add_argument("--baud", type=int, default=1000000)
    parser.add_argument("--frames", type=int, default=DEFAULT_FRAMES)
    parser.add_argument("--start-timeout", type=float, default=30.0)
    parser.add_argument("--idle-timeout", type=float, default=3.0)
    parser.add_argument("--self-test", action="store_true")
    args = parser.parse_args()
    if args.self_test:
        self_test()
        return 0
    if not args.port:
        parser.error("--port is required unless --self-test is selected")
    if args.frames <= 0 or args.baud <= 0:
        parser.error("--frames and --baud must be positive")
    return receive(args.port, args.baud, args.frames,
                   args.start_timeout, args.idle_timeout)


if __name__ == "__main__":
    sys.exit(main())
