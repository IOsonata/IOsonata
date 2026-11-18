""" -------------------------------------------------------------------------
@file   sam4l_uart_wake.py
@brief  PowerOffDemo console with automatic SAM4L wake-baud switching.

@license

MIT License

Copyright (c) 2026 I-SYST inc. All rights reserved.

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


def send_byte(port, timeout, value=b"U"):
    if port.write(value) != len(value):
        raise serial.SerialTimeoutException("Wake/test byte was not queued")
    deadline = time.monotonic() + timeout
    while port.out_waiting:
        if time.monotonic() >= deadline:
            raise serial.SerialTimeoutException("Adapter TX did not drain")
        time.sleep(0.001)
    # USB adapters may report an empty queue before the stop bit leaves TX.
    time.sleep(0.003)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--port", required=True)
    parser.add_argument("--baud", type=int, default=1000000)
    parser.add_argument("--cycles", type=int, default=3,
                        help="Number of complete UART wake cycles to check")
    parser.add_argument("--wake-delay", type=float, default=1.25,
                        help="Seconds after FTDI switches to 9600 before wake U "
                             "(minimum 1.25; increase for MCU sleep-current measurements)")
    parser.add_argument("--timeout", type=float, default=30,
                        help="Seconds without a complete console line")
    parser.add_argument("--total-timeout", type=float, default=300,
                        help="Maximum duration of the complete test in seconds")
    args = parser.parse_args()
    if args.timeout <= 0 or args.total_timeout <= 0 or args.cycles <= 0:
        parser.error("timeouts and cycles must be greater than zero")
    if not 1.25 <= args.wake_delay < args.total_timeout:
        parser.error("--wake-delay must be at least 1.25 seconds and less "
                     "than --total-timeout")
    with serial.Serial(args.port, args.baud, timeout=0.2, write_timeout=2) as port:
        pending = bytearray()
        started = time.monotonic()
        progress = started
        stage = "waiting for PowerOffDemo output"
        cycles = 0
        rx_ok = False
        rate_ok = False
        clock_ok = False
        resumed_ok = False
        normal_rx_ok = False
        setup_acked = False
        last_line = "(none)"
        print("[host] Opened %s at %d baud. Reset the board now." %
              (args.port, args.baud), flush=True)
        print("[host] Use the adapter wired to PB00/PB01, not EDBG. "
              "PowerOffDemo needs POWER_DEMO_UART_WAKE=1.", flush=True)
        while True:
            now = time.monotonic()
            if now - progress >= args.timeout or now - started >= args.total_timeout:
                print("\n[host] Timeout: %s. Last line: %s" %
                      (stage, last_line), file=sys.stderr)
                if pending:
                    print("[host] Partial input: %r" % bytes(pending[-160:]),
                          file=sys.stderr)
                elif cycles == 0 and stage == "waiting for PowerOffDemo output":
                    print("[host] No USART0 console received. Check EXT1/EXT4 "
                          "pin 13 = PB00 MCU RX (adapter TX), pin 14 = PB01 "
                          "MCU TX (adapter RX), pin 19 = GND. EXT2 pin 13/14 "
                          "is USART1, not USART0. Also check that the UART-wake "
                          "firmware was flashed.", file=sys.stderr)
                return 2
            data = port.read_until(b"\n")
            if not data:
                continue
            sys.stdout.buffer.write(data)
            sys.stdout.buffer.flush()
            pending.extend(data)
            if not pending.endswith(b"\n"):
                if len(pending) > 4096:
                    print("\n[host] No line ending in 4096 bytes; check baud rate.",
                          file=sys.stderr)
                    return 2
                continue
            line = bytes(pending)
            pending.clear()
            progress = time.monotonic()
            last_line = line.decode("ascii", errors="replace").strip()
            if b"UART wake byte: FAIL" in line or b"UART rate after resume: FAIL" in line or \
                    b"CPU/APB clock after resume: FAIL" in line or b"SRAM sample FAIL" in line or \
                    b"UART wake test stopped" in line or b"UART wake setup: FAIL" in line:
                print("[host] UART wake cycle failed; stopping without another attempt.",
                      file=sys.stderr)
                return 1
            if b"UART wake setup: send R" in line:
                if not normal_rx_ok or (clock_ok and not resumed_ok):
                    print("[host] Normal USART0 RX test was not confirmed.",
                          file=sys.stderr)
                    return 1
                if clock_ok and resumed_ok:
                    print("[host] Missing SRAM result for previous cycle.",
                          file=sys.stderr)
                    return 1
                stage = "waiting for 9600 baud switch prompt"
                # Allow MCU console TX draining to finish before replying.
                time.sleep(0.05)
                send_byte(port, 2, b"R")
                setup_acked = True
                print("[host] Sent R at %d baud; waiting for MCU setup prompt." %
                      args.baud, flush=True)
            elif b"UART sleep preparation: switch host to 9600 baud" in line:
                if not setup_acked:
                    print("[host] Missing 1 Mbaud wake setup handshake.",
                          file=sys.stderr)
                    return 1
                setup_acked = False
                stage = "waiting for UART wake result"
                rx_ok = False
                rate_ok = False
                clock_ok = False
                resumed_ok = False
                # The MCU waits 750 ms after the prompt before calling
                # SystemSleep(). Keep FTDI at 9600 for the requested interval
                # so sleep current can be measured without altering firmware.
                try:
                    port.baudrate = 9600
                    print("[host] Adapter switched to 9600 before PICOUART "
                          "activation.", flush=True)
                    if args.wake_delay > 1.25:
                        print("[host] Wake deferred %.2f s for sleep-current "
                              "measurement." % args.wake_delay, flush=True)
                    time.sleep(args.wake_delay)
                    if time.monotonic() - started >= args.total_timeout:
                        print("[host] Total timeout reached during wake delay.",
                              file=sys.stderr)
                        return 2
                    early = port.read(port.in_waiting) if port.in_waiting else b""
                    if early:
                        print("[host] Unexpected data during the 9600-baud "
                              "quiet interval before sending U: %r. "
                              "The MCU may have refused sleep or woken early." %
                              early[-160:], file=sys.stderr)
                        return 1
                    send_byte(port, 2)
                finally:
                    port.baudrate = args.baud
                progress = time.monotonic()
                print("[host] Sent U at 9600; adapter restored to %d baud." %
                      args.baud, flush=True)
            elif b"Entering UART sleep;" in line:
                print("[host] Old PowerOffDemo firmware. Rebuild and flash "
                      "the version with the 1 Mbaud R handshake and "
                      "9600-baud setup prompt.", file=sys.stderr)
                return 2
            elif b"UART RX: send U" in line:
                stage = "waiting for normal UART RX result"
                send_byte(port, 2)
                print("[host] Sent U at %d baud." % args.baud, flush=True)
            elif b"UART RX normal: PASS" in line:
                normal_rx_ok = True
                stage = "waiting for UART wake setup handshake"
            elif b"UART RX normal: FAIL" in line:
                print("[host] USART0 RX could not receive U at normal baud. "
                      "Check adapter TX to PB00 and shared ground.",
                      file=sys.stderr)
                return 1
            elif b"UART wake unavailable for this configuration" in line:
                print("[host] UARTEnableWake rejected the USART0 profile.",
                      file=sys.stderr)
                return 1
            elif b"Entering Standby" in line:
                print("[host] Another wake profile is selected. Enable "
                      "POWER_DEMO_UART_WAKE and rebuild PowerOffDemo.",
                      file=sys.stderr)
                return 2
            elif b"UART wake byte: PASS" in line:
                rx_ok = True
            elif b"UART rate after resume: PASS" in line:
                rate_ok = True
            elif b"CPU/APB clock after resume: PASS" in line:
                if not rx_ok or not rate_ok:
                    print("[host] Incomplete UART wake cycle.", file=sys.stderr)
                    return 1
                clock_ok = True
                stage = "waiting for MCU resume counter"
            elif b"Resumed from Standby, count 0x" in line:
                if not clock_ok or resumed_ok:
                    print("[host] Unexpected MCU resume counter.", file=sys.stderr)
                    return 1
                try:
                    count = int(line.rsplit(b"0x", 1)[1], 16)
                except (IndexError, ValueError):
                    print("[host] Invalid MCU resume counter.", file=sys.stderr)
                    return 1
                if count != cycles + 1:
                    print("[host] MCU resume count %d, expected %d." %
                          (count, cycles + 1), file=sys.stderr)
                    return 1
                resumed_ok = True
                stage = "waiting for SRAM retention result"
            elif b"SRAM sample PASS:" in line:
                if not (rx_ok and rate_ok and clock_ok and resumed_ok):
                    print("[host] SRAM result without a complete wake cycle.",
                          file=sys.stderr)
                    return 1
                cycles += 1
                print("[host] UART wake cycle %d/%d PASS, including SRAM." %
                      (cycles, args.cycles), flush=True)
                if cycles >= args.cycles:
                    return 0
                clock_ok = False
                resumed_ok = False
                stage = "waiting for the next PowerOffDemo sleep"


if __name__ == "__main__":
    try:
        sys.exit(main())
    except KeyboardInterrupt:
        sys.exit(130)
    except (serial.SerialException, OSError) as error:
        print(error, file=sys.stderr)
        sys.exit(1)
