""" -------------------------------------------------------------------------
@file   uartprbs_lb.py

@brief  PRBS UART loopback test with one or two serial ports

Sends the PRBS7 stream to the board running exemples/uart/uart_loopback.cpp
and checks the echo. At most --window bytes are sent and not yet accounted
for. Each echoed byte is matched to its position in the sent stream:

    errors  : discontinuities in the echo (loss, corrupt or repeat events)
    lost    : bytes that were sent and never echoed correctly. A byte that
              matches a later position, confirmed by the byte after it,
              means the bytes in between were lost. They are written off so
              that they no longer hold the window. A corrupted byte also
              counts as lost, since its correct value never came back.
    corrupt : echoed bytes that match no position nearby
    repeat  : echoed bytes that match a position already passed
    pending : bytes sent and not yet accounted for

Counters that are 0 are not printed, except errors and pending.

If nothing is echoed for --timeout seconds, the pending bytes are written off
as lost and the test goes on. A second timeout in a row ends the test.

@author Hoang Nguyen Hoan
@date   July 27, 2019

@license

MIT License

Copyright (c) 2019 I-SYST inc. All rights reserved.

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.

----------------------------------------------------------------------------
"""
import argparse
import sys
import threading
import time

import serial


def prbs8(curval):
    newbit = ((curval >> 6) ^ (curval >> 5)) & 1
    return ((curval << 1) | newbit) & 0x7f


PRBS_PERIOD = 127
# Echo position search. A byte up to MAX_LOSS positions ahead means the bytes
# in between were lost, up to MAX_REPEAT behind means bytes were echoed
# twice. The two ranges do not overlap within the 127-byte PRBS period.
MAX_LOSS = 110
MAX_REPEAT = 16
# Unmatched bytes forming a PRBS run before the position is taken from them
RESYNC_BYTES = 8
# Unmatched bytes kept for the resync check
RECENT_MAX = 64


class EchoCheck:
    """Match each echoed byte to its position in the sent PRBS stream.

    pos is the stream position of the next expected echo. All positions
    before it are accounted for, either received or written off as lost. A
    jump in position is confirmed by the next byte before it is counted.
    Bytes that match no position are kept until the stream matches again,
    then counted as corrupt.
    """

    def __init__(self, pattern):
        self.pattern = pattern
        self.phase = {b: i for i, b in enumerate(pattern)}
        self.pos = 0
        self.received = 0
        self.errors = 0
        self.lost = 0
        self.corrupt = 0
        self.repeat = 0
        self.held = None
        self.recent = []

    def _offset(self, b, pos, sent):
        # Position jump from pos to the byte, None if it is not a nearby one
        ph = self.phase.get(b)
        if ph is None:
            return None
        d = (ph - pos) % PRBS_PERIOD
        if 0 < d <= MAX_LOSS and pos + d < sent:
            return d
        if d >= PRBS_PERIOD - MAX_REPEAT:
            # A repeat must refer to a byte in this run. If that position
            # would be negative, only the forward loss can be valid.
            if pos + d >= PRBS_PERIOD:
                return d - PRBS_PERIOD
            if pos + d < sent:
                return d
        return None

    def _matched(self, newpos, jump):
        # The stream matches again at newpos. Unmatched bytes before were
        # corrupt. jump is the confirmed position change, 0 if none.
        if self.recent or jump:
            self.errors += 1
        self.corrupt += len(self.recent)
        self.recent = []
        if jump > 0:
            self.lost += jump
        elif jump < 0:
            self.repeat += -jump
        self.pos = newpos

    def _resync(self, sent):
        # The last unmatched bytes form a PRBS run: a long gap or an echo
        # from before. Take the position of the run from its last byte.
        run = self.recent[-RESYNC_BYTES:]
        for prev, cur in zip(run, run[1:]):
            if cur != prbs8(prev) or cur not in self.phase:
                return False
        last = sent - 1
        p = last - ((last - self.phase[run[-1]]) % PRBS_PERIOD)
        first = p - RESYNC_BYTES + 1
        if first < 0:
            # No complete matching run has been sent yet.
            return False
        self.recent = self.recent[:-RESYNC_BYTES]
        # Positive: the bytes before the run were lost. Negative: the run
        # repeats positions already passed.
        self._matched(p + 1, first - self.pos)
        return True

    def _byte(self, b, sent):
        if self.held is not None:
            d = self.held
            self.held = None
            nxt = self.pos + d + 1
            if b == self.pattern[nxt % PRBS_PERIOD]:
                # Jump confirmed by the next byte
                self._matched(nxt + 1, d)
                return
            d2 = self._offset(b, nxt, sent)
            if d > 0 and d2 is not None and d2 > 0:
                # The held byte is valid, more bytes lost after it
                self._matched(nxt, d)
            else:
                # The held byte matched a position by chance
                self.recent.append(self.pattern[(self.pos + d) % PRBS_PERIOD])

        if b == self.pattern[self.pos % PRBS_PERIOD]:
            if self.recent:
                self._matched(self.pos + 1, 0)
            else:
                self.pos += 1
            return

        d = self._offset(b, self.pos, sent)
        if d is not None:
            # Wait for the next byte to confirm the jump
            self.held = d
            return

        self.recent.append(b)
        if len(self.recent) >= RESYNC_BYTES and self._resync(sent):
            return
        if len(self.recent) > RECENT_MAX:
            self.corrupt += len(self.recent) - RECENT_MAX
            self.recent = self.recent[-RECENT_MAX:]

    def feed(self, data, sent):
        for b in data:
            self._byte(b, sent)
        self.received += len(data)

    def pending(self, sent):
        return max(0, sent - self.pos)

    def write_off(self, sent):
        # Nothing echoed for a while: the pending bytes are lost
        n = self.pending(sent)
        if n > 0 or self.held is not None or self.recent:
            self.errors += 1
        self.corrupt += len(self.recent) + (1 if self.held is not None else 0)
        self.lost += n
        self.pos = max(self.pos, sent)
        self.held = None
        self.recent = []
        return n


def drain(port, quiet=0.05, limit=1.0):
    # Discard bytes still echoed from an earlier run until the line is quiet
    end = time.perf_counter() + limit
    last = time.perf_counter()
    port.reset_input_buffer()
    while time.perf_counter() < end:
        if port.in_waiting:
            port.reset_input_buffer()
            last = time.perf_counter()
        elif time.perf_counter() - last >= quiet:
            break
        time.sleep(0.005)


def parse_args():
    parser = argparse.ArgumentParser(description="Threaded PRBS UART loopback")
    parser.add_argument("--port", required=True, help="Transmit serial port")
    parser.add_argument("--rx-port", help="Separate receive port (default: --port)")
    parser.add_argument("--baud", type=int, default=1000000)
    parser.add_argument("--timeout", type=float, default=1.0,
                        help="No-receive timeout in seconds (default: 1)")
    parser.add_argument("--report-interval", type=float, default=1.0)
    parser.add_argument("--block-size", type=int, default=16,
                        help="Transmitted bytes per write (default: 16)")
    parser.add_argument("--window", type=int, default=128,
                        help="Maximum unacknowledged bytes (default: 128)")
    return parser.parse_args()


def main():
    args = parse_args()
    if (args.baud <= 0 or args.timeout <= 0 or args.report_interval <= 0 or
            args.block_size <= 0 or args.window < args.block_size):
        print("ERROR: invalid baud, timeout, interval, block size or window",
              file=sys.stderr)
        return 2

    tx = None
    rx = None
    worker = None
    stop = threading.Event()
    condition = threading.Condition()
    state = {"sent": 0, "failure": None}

    # PRBS7 repeats every 127 bytes; generate the stream once for efficient
    # block transmission, then match every received byte to its position.
    sequence = bytearray()
    value = prbs8(0xff)
    for _ in range(PRBS_PERIOD):
        sequence.append(value)
        value = prbs8(value)
    pattern = bytes(sequence)
    check = EchoCheck(pattern)

    def transmit():
        try:
            while not stop.is_set():
                with condition:
                    condition.wait_for(
                        lambda: stop.is_set() or
                        check.pending(state["sent"]) < args.window,
                        timeout=0.1)
                    if stop.is_set():
                        return
                    available = args.window - check.pending(state["sent"])
                    if available <= 0:
                        continue
                    offset = state["sent"]
                    length = min(args.block_size, available)
                block = bytes(pattern[(offset + i) % PRBS_PERIOD]
                              for i in range(length))
                written = tx.write(block)
                if written != length:
                    raise serial.SerialTimeoutException(
                        "Short UART write: %d of %d" % (written, length))
                with condition:
                    state["sent"] += written
                    condition.notify_all()
        except (serial.SerialException, OSError) as exc:
            with condition:
                state["failure"] = exc
                condition.notify_all()
            stop.set()

    def report_line(rate, sent):
        fields = ["Rx B/s : %.2f" % rate, "errors %d" % check.errors]
        if check.lost:
            fields.append("lost %d" % check.lost)
        if check.corrupt:
            fields.append("corrupt %d" % check.corrupt)
        if check.repeat:
            fields.append("repeat %d" % check.repeat)
        fields.append("pending %d" % check.pending(sent))
        return ", ".join(fields)

    try:
        tx = serial.Serial(port=args.port, baudrate=args.baud,
                           timeout=0.1, write_timeout=args.timeout,
                           rtscts=False)
        rx = (serial.Serial(port=args.rx_port, baudrate=args.baud,
                            timeout=0.1, rtscts=False)
              if args.rx_port and args.rx_port != args.port else tx)
        # Bytes from an earlier run may still be echoed
        drain(rx)

        worker = threading.Thread(target=transmit, name="uart-prbs-tx",
                                  daemon=True)
        worker.start()
        start = time.perf_counter()
        last_report = start
        last_received_at = start
        prior_received = 0
        written_off = False

        while not stop.is_set():
            # Read data already available without waiting for a full window.
            # The transmit window is also args.window, so requesting that
            # many bytes causes a timeout-driven send/receive feedback loop
            # whenever fewer bytes are outstanding.
            ready = rx.in_waiting
            data = rx.read(min(ready, args.window) if ready else 1)
            now = time.perf_counter()
            if data:
                with condition:
                    check.feed(data, state["sent"])
                    condition.notify_all()
                last_received_at = now
                written_off = False

            with condition:
                sent = state["sent"]
                received = check.received
                failure = state["failure"]

            if failure is not None:
                raise failure
            if now - last_report >= args.report_interval:
                elapsed = now - last_report
                rate = (received - prior_received) / elapsed
                with condition:
                    line = report_line(rate, state["sent"])
                print(line, flush=True)
                prior_received = received
                last_report = now
            if now - last_received_at >= args.timeout:
                if written_off:
                    raise serial.SerialTimeoutException(
                        "No UART echo for %.2f seconds (%d sent, %d received)" %
                        (args.timeout, sent, received))
                with condition:
                    n = check.write_off(state["sent"])
                    condition.notify_all()
                print("No UART echo for %.2f seconds, %d pending bytes written off as lost" %
                      (args.timeout, n), flush=True)
                written_off = True
                last_received_at = now

    except KeyboardInterrupt:
        print("KeyboardInterrupt. Exiting.")
    except (serial.SerialException, OSError) as exc:
        print("ERROR: %s" % exc, file=sys.stderr)
        return 2
    finally:
        stop.set()
        with condition:
            condition.notify_all()
        if worker is not None:
            worker.join(timeout=args.timeout + 0.2)
        if rx is not None and rx is not tx:
            rx.close()
        if tx is not None:
            tx.close()
    return 0


if __name__ == "__main__":
    sys.exit(main())
