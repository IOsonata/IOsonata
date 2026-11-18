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


def send_byte(port, timeout):
    if port.write(b"U") != 1:
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
    parser.add_argument("--timeout", type=float, default=30,
                        help="Seconds without a complete console line")
    parser.add_argument("--total-timeout", type=float, default=300,
                        help="Maximum duration of the complete test in seconds")
    args = parser.parse_args()
    if args.timeout <= 0 or args.total_timeout <= 0 or args.cycles <= 0:
        parser.error("timeouts and cycles must be greater than zero")
    with serial.Serial(args.port, args.baud, timeout=0.2, write_timeout=2) as port:
        pending = bytearray()
        started = time.monotonic()
        progress = started
        stage = "waiting for PowerOffDemo output"
        cycles = 0
        rx_ok = False
        rate_ok = False
        normal_rx_ok = False
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
                    b"CPU/APB clock after resume: FAIL" in line:
                print("[host] UART wake cycle failed; stopping without another attempt.",
                      file=sys.stderr)
                return 1
            if b"Entering UART sleep;" in line:
                if not normal_rx_ok:
                    print("[host] Normal USART0 RX test was not confirmed.",
                          file=sys.stderr)
                    return 1
                stage = "waiting for UART wake result"
                rx_ok = False
                rate_ok = False
                # The MCU prints a FAIL around 100 ms after an immediate
                # SystemSleep return. Check for that before sending a byte,
                # otherwise the result can be mistaken for a failed decode.
                time.sleep(0.35)
                early = port.read(port.in_waiting) if port.in_waiting else b""
                if early:
                    sys.stdout.buffer.write(early)
                    sys.stdout.buffer.flush()
                    print("[host] MCU responded before the 9600-baud wake "
                          "byte was sent. Sleep was refused or another "
                          "event woke it. Inspect the SAM4L diagnostic.",
                          file=sys.stderr)
                    return 1
                try:
                    port.baudrate = 9600
                    send_byte(port, 2)
                finally:
                    port.baudrate = args.baud
                print("[host] Sent U at 9600; adapter restored to %d baud." %
                      args.baud, flush=True)
            elif b"UART RX: send U" in line:
                stage = "waiting for normal UART RX result"
                send_byte(port, 2)
                print("[host] Sent U at %d baud." % args.baud, flush=True)
            elif b"UART RX normal: PASS" in line:
                normal_rx_ok = True
                stage = "waiting for PICOUART sleep prompt"
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
                cycles += 1
                print("[host] UART wake cycle %d/%d PASS." %
                      (cycles, args.cycles), flush=True)
                if cycles >= args.cycles:
                    return 0
                stage = "waiting for the next PowerOffDemo sleep"



if __name__ == "__main__":
    try:
        sys.exit(main())
    except KeyboardInterrupt:
        sys.exit(130)
    except (serial.SerialException, OSError) as error:
        print(error, file=sys.stderr)
        sys.exit(1)
