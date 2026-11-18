""" -------------------------------------------------------------------------
@file   sam4l_uart_wake.py
@brief  ClockDemo console with automatic SAM4L wake-baud switching.

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


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--port", required=True)
    parser.add_argument("--baud", type=int, default=1000000)
    args = parser.parse_args()
    with serial.Serial(args.port, args.baud, timeout=1, write_timeout=2) as port:
        pending = bytearray()
        while True:
            data = port.read_until(b"\n")
            if not data:
                continue
            sys.stdout.buffer.write(data)
            sys.stdout.buffer.flush()
            pending.extend(data)
            if not pending.endswith(b"\n"):
                if len(pending) > 4096:
                    pending.clear()
                continue
            line = bytes(pending)
            pending.clear()
            if b"Entering UART sleep;" in line:
                # Let the MCU arm PICOUART after draining its prompt.
                time.sleep(0.1)
                try:
                    port.baudrate = 9600
                    port.write(b"U")
                    port.flush()
                    # Allow the 8-N-1 frame to leave the adapter shift register.
                    time.sleep(0.003)
                finally:
                    port.baudrate = args.baud
            elif b"UART RX: send U" in line:
                port.write(b"U")
                port.flush()
            elif b"ClockDemo complete:" in line:
                return 0 if b"complete: 0 failures" in line else 1


if __name__ == "__main__":
    try:
        sys.exit(main())
    except KeyboardInterrupt:
        sys.exit(130)
    except (serial.SerialException, OSError) as error:
        print(error, file=sys.stderr)
        sys.exit(1)
