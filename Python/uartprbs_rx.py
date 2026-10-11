""" -------------------------------------------------------------------------
@file   uartprbs_rx.py

@brief  PRBS receive test

Receives the PRBS8 stream sent by exemples/uart/uart_prbs_tx.cpp and checks
each byte against the PRBS successor of the byte before it.

Printed each report interval, counters are totals since start:
    Bytes/sec    : bytes received during the last interval per second
    drop         : bytes that are not the successor of the byte before them
                   (same count as earlier versions of this script)
    corrupt      : bytes replaced or inserted, the stream then continues
    lost         : jumps in the sequence, data missing between two bytes,
                   with the number of bytes missing. The sequence repeats
                   every 127 bytes, so a loss of a multiple of 127 bytes is
                   not seen
    out of frame : bytes above 0x7F, which the transmitter never sends
                   after its first 0xFF. They mean that the receiver splits
                   the bit stream at the wrong place, usually because it
                   started in the middle of a stream with no idle time
                   between characters. Reset the board with this script
                   running.
    restart      : the transmitter started again (0xFF then 0x7E)
    average      : bytes per second since the first byte

A report is printed even when no data arrives.

@param  --port : Serial port to use
        --baud : Baud rate (default 1MBaud)
        --read-size : Serial read block size (default 4096)
        --report-interval : Status report interval in seconds (default 1.0)
        --dtr : DTR state while the port is open, on or off (default on)
        --rts : RTS state while the port is open, on or off (default on)

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
import time

import serial

# The transmitter sends 0xFF first, then the 7-bit PRBS values 0x01 to 0x7F
PRBS_START = 0xFF
PRBS_PERIOD = 127


def prbs8(curval):
    newbit = (((curval >> 6) ^ (curval >> 5)) & 1)
    return ((curval << 1) | newbit) & 0x7f


# Successor of every byte value, and the position of each PRBS value in the
# sequence
NEXT = [prbs8(v) for v in range(256)]
INDEX = {}
_v = NEXT[PRBS_START]
for _i in range(PRBS_PERIOD):
    INDEX[_v] = _i
    _v = NEXT[_v]


# Require enough consecutive valid PRBS bytes to establish a reliable
# phase before accepting measurements after a damaged stream.
SYNC_BYTES = 8
MAX_RUN = 16
MAX_ESTIMATED_LOSS = 16


class PrbsCheck:
    """Validate PRBS continuity and reacquire phase after sustained errors.

    Drop counts raw adjacent-byte discontinuities independently of recovery.
    Corrupt and lost counts describe only observations that can be classified
    with confidence. The 127-byte PRBS period limits loss estimation.
    """

    def __init__(self):
        self.count = 0
        self.drop = 0
        self.corrupt = 0
        self.lost = 0
        self.lost_bytes = 0
        self.out_of_frame = 0
        self.restart = 0
        self.uncertain = 0
        self.recovered = 0
        self.last = None
        self.prev = None
        self.synced = False
        self.had_sync = False
        self.run = []
        self.anchor = None

    @staticmethod
    def valid(c):
        return c in INDEX or c == PRBS_START

    @staticmethod
    def confirmed(values):
        if len(values) != SYNC_BYTES or not PrbsCheck.valid(values[0]):
            return False
        for previous, current in zip(values, values[1:]):
            if current != NEXT[previous] or current not in INDEX:
                return False
        return True

    def _recover(self):
        if len(self.run) < SYNC_BYTES:
            return

        candidate = self.run[-SYNC_BYTES:]
        if not self.confirmed(candidate):
            if len(self.run) > MAX_RUN:
                # Bound both memory and speculative accounting. Retain the
                # last SYNC_BYTES - 1 bytes for a candidate across reads.
                flushed = len(self.run) - (SYNC_BYTES - 1)
                if self.had_sync:
                    self.corrupt += flushed
                self.run = self.run[-(SYNC_BYTES - 1):]
                self.anchor = None
            return

        prefix = len(self.run) - SYNC_BYTES
        first = candidate[0]

        if self.had_sync:
            self.corrupt += prefix

            if first == PRBS_START:
                self.restart += 1
            elif self.anchor is not None and self.anchor in INDEX:
                expected = NEXT[self.anchor]
                # Extra received bytes cannot be distinguished from a long
                # jump around the 127-byte PRBS period. Only report bounded
                # forward losses; preserve other events as uncertain.
                distance = (INDEX[first] - INDEX[expected]) % PRBS_PERIOD
                missing = distance - prefix
                if missing == 0:
                    pass
                elif 0 < missing <= MAX_ESTIMATED_LOSS:
                    self.lost += 1
                    self.lost_bytes += missing
                else:
                    self.uncertain += 1
            elif self.anchor is not None:
                self.uncertain += 1

        if self.had_sync:
            self.recovered += 1

        self.had_sync = True
        self.synced = True
        self.prev = candidate[-1]
        self.run = []
        self.anchor = None

    def feed(self, data):
        for c in data:
            if c > 0x7F and c != PRBS_START:
                self.out_of_frame += 1

            if self.last is not None and c != NEXT[self.last]:
                self.drop += 1
            self.last = c

            if self.synced:
                if c == NEXT[self.prev]:
                    self.prev = c
                    continue

                self.synced = False
                self.anchor = self.prev
                self.run = [c]
            else:
                self.run.append(c)

            self._recover()

        self.count += len(data)

    def finish(self):
        """Include an unresolved tail in final corruption statistics."""
        if self.run and self.had_sync:
            self.corrupt += len(self.run)
            self.uncertain += 1
        self.run = []


def parse_args() -> argparse.Namespace:
    p = argparse.ArgumentParser(description="PRBS serial receiver")
    p.add_argument("--port", required=True,
                   help="Serial port (e.g., /dev/ttyUSB0, /dev/cu.usbmodemXXX, COM5)")
    p.add_argument("--baud", type=int, default=1_000_000,
                   help="Baud rate (default: 1000000)")
    p.add_argument("--read-size", type=int, default=4096,
                   help="Bytes per serial read (default: 4096)")
    p.add_argument("--report-interval", type=float, default=1.0,
                   help="Status report interval in seconds (default: 1.0)")
    p.add_argument("--dtr", choices=("on", "off"), default="on",
                   help="DTR state while the port is open (default: on)")
    p.add_argument("--rts", choices=("on", "off"), default="on",
                   help="RTS state while the port is open (default: on)")
    return p.parse_args()


def report(chk, rate, average, oof_interval, count_interval):
    fields = ["Bytes/sec : %.2f" % rate]

    if chk.drop:
        fields.append("drop %d" % chk.drop)
    if chk.corrupt:
        fields.append("corrupt %d" % chk.corrupt)
    if chk.lost or chk.lost_bytes:
        fields.append("lost %d (%d bytes)" % (chk.lost, chk.lost_bytes))
    if chk.out_of_frame:
        fields.append("out of frame %d" % chk.out_of_frame)
    if chk.restart:
        fields.append("restart %d" % chk.restart)
    if chk.uncertain:
        fields.append("uncertain %d" % chk.uncertain)
    if chk.recovered:
        fields.append("recovered %d" % chk.recovered)

    fields.append("average %.2f" % average)
    print(", ".join(fields))

    if count_interval == 0:
        print("  no data")
    elif oof_interval * 100 > count_interval:
        print("  receiver out of frame: reset the board with this script running")


def main():
    args = parse_args()

    if args.read_size <= 0:
        print("ERROR: --read-size must be greater than zero", file=sys.stderr)
        return 2

    if args.report_interval <= 0:
        print("ERROR: --report-interval must be greater than zero", file=sys.stderr)
        return 2

    try:
        # DTR and RTS are set before the port opens, a board may use them
        # for its reset or boot pins
        comm = serial.Serial()
        comm.port = args.port
        comm.baudrate = args.baud
        comm.timeout = 0.1
        comm.dtr = args.dtr == "on"
        comm.rts = args.rts == "on"
        comm.open()
    except Exception as e:
        print(f"ERROR: cannot open {args.port} at {args.baud} baud: {e}", file=sys.stderr)
        return 2

    comm.reset_input_buffer()
    chk = PrbsCheck()

    try:
        # The average starts after the first read that returns data
        start_time = None
        start_count = 0
        last_report = time.perf_counter()
        last_count = 0
        last_oof = 0

        while True:
            data = comm.read(args.read_size)
            now = time.perf_counter()
            if data:
                chk.feed(data)
                if start_time is None:
                    start_time = now
                    start_count = chk.count

            if now - last_report >= args.report_interval:
                interval = now - last_report
                count_interval = chk.count - last_count
                average = 0.0
                if start_time is not None and now > start_time:
                    average = (chk.count - start_count) / (now - start_time)
                report(chk, count_interval / interval, average,
                       chk.out_of_frame - last_oof, count_interval)
                last_report = now
                last_count = chk.count
                last_oof = chk.out_of_frame

    except KeyboardInterrupt:
        chk.finish()
        print("KeyboardInterrupt. Final counters:")
        average = 0.0
        if start_time is not None:
            elapsed = time.perf_counter() - start_time
            if elapsed > 0:
                average = (chk.count - start_count) / elapsed
        report(chk, 0.0, average, 0, -1)
    finally:
        comm.close()

    return 0


if __name__ == '__main__':
    sys.exit(main())
