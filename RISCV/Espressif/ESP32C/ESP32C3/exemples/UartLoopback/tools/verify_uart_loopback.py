#!/usr/bin/env python3
"""ESP32-C3 UART0 host-driven loopback acceptance at 1 Mbaud.

Uses the board's existing CP2102N USB/UART bridge. The firmware reuses
IOsonata's generic exemples/uart/uart_loopback.cpp unchanged. No jumper,
second serial port, or board-specific UART driver is required.

Requires pyserial; no ESP-IDF dependency.
"""
import argparse
import random
import sys
import time


class LoopbackFailure(RuntimeError):
    pass


def verify_frame(port, expected, timeout=2.0):
    """Send one binary frame and check that the echo matches byte-for-byte."""
    if not expected:
        raise ValueError("expected must not be empty")

    n = port.write(expected)
    if n != len(expected):
        raise LoopbackFailure(f"short write: {n} of {len(expected)} bytes")

    received = bytearray()
    deadline = time.monotonic() + timeout
    while len(received) < len(expected):
        data = port.read(len(expected) - len(received))
        if data:
            received.extend(data)
        elif time.monotonic() >= deadline:
            raise LoopbackFailure(
                f"RX timeout: received {len(received)}/{len(expected)} bytes, "
                f"first bytes={bytes(received[:16]).hex()}"
            )

    if received != expected:
        index = next(i for i, (a, b) in enumerate(zip(expected, received))
                     if a != b)
        raise LoopbackFailure(
            f"RX mismatch at byte {index}/{len(expected)}: "
            f"expected 0x{expected[index]:02x}, received 0x{received[index]:02x}; "
            f"expected start={expected[:16].hex()}, "
            f"actual start={received[:16].hex()}"
        )

    return len(expected)


def sync_loopback(port):
    """Discard boot-ROM chatter and wait for firmware echo readiness."""
    probe = b"\x55\xa5\x00\xff"
    for attempt in range(10):
        port.reset_input_buffer()
        try:
            verify_frame(port, probe, timeout=0.5)
            print("  [PASS] UART0 echo synchronization")
            return
        except LoopbackFailure:
            time.sleep(0.2)
    raise LoopbackFailure("UART0 did not echo the synchronization sequence")


def acceptance(port, seconds=30, chunk=64, stress=False, timeout=2.0):
    """Run deterministic binary echo cases followed by continuous frames."""
    rng = random.Random(0xC32026)
    fixed = [
        b"\x00",
        b"\xff",
        b"\x55",
        b"\xaa",
        b"\x00\xff\x0d\x0a\x80\x7f",
        bytes(range(16)),
        bytes(range(64)),
    ]
    sizes = [1, 2, 3, 7, 15, 16, 17, 31, 32, 63, 64, 65, 127, 128, 255]
    if stress:
        sizes.extend([256, 511, 1024])

    count = 0
    for frame in fixed:
        count += verify_frame(port, frame, timeout)
    print(f"  [PASS] {len(fixed)} fixed binary echo patterns")

    for n in sizes:
        data = bytes(rng.getrandbits(8) for _ in range(n))
        count += verify_frame(port, data, timeout)
    print(f"  [PASS] {len(sizes)} variable-length frames "
          f"(1..{max(sizes)} bytes, unaligned lengths)")

    if seconds == 0:
        return count, 0, 0.0

    start = time.monotonic()
    last_report = start
    frames = 0
    traffic = 0
    while time.monotonic() - start < seconds:
        data = bytes(rng.getrandbits(8) for _ in range(chunk))
        traffic += verify_frame(port, data, timeout)
        frames += 1
        now = time.monotonic()
        if now - last_report >= 5.0:
            print(f"  elapsed {now - start:.1f}s: "
                  f"{frames} frames, {traffic} bytes, 0 mismatches")
            last_report = now

    elapsed = time.monotonic() - start
    print(f"  [PASS] sustained echo: {frames} frames, {traffic} bytes, "
          f"0 mismatches in {elapsed:.1f}s")
    print(f"  Stop-and-wait echo rate: {traffic / elapsed:.0f} B/s "
          f"(not maximum UART throughput)")
    return count + traffic, frames, elapsed


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--port", required=True,
                        help="CP2102N serial port, e.g. /dev/cu.SLAB_USBtoUART or COM5")
    parser.add_argument("--baud", type=int, default=1_000_000)
    parser.add_argument("--seconds", type=int, default=30,
                        help="sustained duration; 0 to test fixed frames only")
    parser.add_argument("--chunk", type=int, default=64,
                        help="bytes per sustained echo frame (1..256)")
    parser.add_argument("--timeout", type=float, default=2.0,
                        help="per-frame receive deadline in seconds")
    parser.add_argument("--stress", action="store_true",
                        help="also try 256, 511, 1024-byte unpaced frames")
    args = parser.parse_args(argv)
    if not 0 <= args.seconds <= 3600:
        parser.error("--seconds must be between 0 and 3600")
    if not 1 <= args.chunk <= 256:
        parser.error("--chunk must be between 1 and 256")
    if args.timeout <= 0:
        parser.error("--timeout must be greater than zero")

    try:
        import serial
    except ImportError:
        print("pyserial is required: python3 -m pip install pyserial",
              file=sys.stderr)
        return 2

    print(f"ESP32-C3 UART0 binary echo test: {args.port}, {args.baud} baud")
    print("Expected firmware: generic IOsonata UartLoopback; no console output")
    port = serial.Serial(port=None, baudrate=args.baud, timeout=0.05,
                         write_timeout=args.timeout, rtscts=False,
                         dsrdtr=False, xonxoff=False)
    try:
        # Avoid intentionally asserting auto-program/reset control lines.
        port.dtr = False
        port.rts = False
        port.port = args.port
        port.open()
        # Opening some bridges resets the target. Allow its application
        # to start before discarding ROM boot messages.
        time.sleep(1.0)
        sync_loopback(port)
        acceptance(port, args.seconds, args.chunk, args.stress, args.timeout)
    except (LoopbackFailure, serial.SerialException, OSError) as error:
        print(f"  [FAIL] {error}", file=sys.stderr)
        return 1
    finally:
        if port.is_open:
            port.close()

    print("PASS: UART0 loopback, 0 errors")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
