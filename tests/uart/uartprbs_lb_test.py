#!/usr/bin/env python3
"""-------------------------------------------------------------------------
@file   uartprbs_lb_test.py

@brief  UART PRBS echo accounting checks without a serial device

Run from the repository root with python3 tests/uart/uartprbs_lb_test.py.
Requires pyserial, as does Python/uartprbs_lb.py.

@author Hoang Nguyen Hoan
@date   Oct. 14, 2026

@license

MIT License

Copyright (c) 2026, I-SYST inc., all rights reserved.

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
-------------------------------------------------------------------------"""
import sys
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[2] / "Python"))
from uartprbs_lb import EchoCheck, PRBS_PERIOD, prbs8


class EchoCheckTest(unittest.TestCase):
    def setUp(self):
        sequence = bytearray()
        value = prbs8(0xff)
        for _ in range(PRBS_PERIOD):
            sequence.append(value)
            value = prbs8(value)
        self.pattern = bytes(sequence)

    def test_clean_stream(self):
        check = EchoCheck(self.pattern)
        stream = self.pattern * 3
        for offset in range(0, len(stream), 16):
            check.feed(stream[offset:offset + 16], len(stream))
        self.assertEqual(check.pos, len(stream))
        self.assertEqual((check.errors, check.lost, check.corrupt, check.repeat), (0, 0, 0, 0))

    def test_startup_loss_is_not_a_repeat_before_byte_zero(self):
        for corrupt in (5, 6):
            for chunk in (1, 2, 8, 16, 128):
                for lost in range(111, 127):
                    with self.subTest(corrupt=corrupt, chunk=chunk, lost=lost):
                        check = EchoCheck(self.pattern)
                        echo = bytes([0xff]) * corrupt + self.pattern[lost:] + self.pattern[:1]
                        for offset in range(0, len(echo), chunk):
                            check.feed(echo[offset:offset + chunk], 128)
                        self.assertEqual(check.pos, 128)
                        self.assertEqual((check.errors, check.lost, check.corrupt, check.repeat),
                                         (1, lost, corrupt, 0))
                        # Subsequent clean data must not add errors or retain
                        # a phantom 127-byte backlog after resynchronization.
                        check.feed(self.pattern[1:], 254)
                        self.assertEqual(check.pending(254), 0)
                        self.assertEqual(check.errors, 1)

    def test_stale_run_before_any_transmission(self):
        check = EchoCheck(self.pattern)
        check.feed(self.pattern[-8:], 0)
        self.assertEqual(check.pos, 0)
        self.assertEqual(check.repeat, 0)
        check.feed(self.pattern[:8], 8)
        self.assertEqual(check.pos, 8)
        self.assertEqual((check.errors, check.lost, check.corrupt, check.repeat), (1, 0, 8, 0))

    def test_actual_repeat(self):
        check = EchoCheck(self.pattern)
        check.feed(self.pattern[:32] + self.pattern[16:32] + self.pattern[32:64], 64)
        self.assertEqual(check.pos, 64)
        self.assertEqual((check.errors, check.lost, check.corrupt, check.repeat), (1, 0, 0, 16))

    def test_short_loss(self):
        check = EchoCheck(self.pattern)
        check.feed(self.pattern[27:80], 80)
        self.assertEqual(check.pos, 80)
        self.assertEqual((check.errors, check.lost, check.corrupt, check.repeat), (1, 27, 0, 0))


if __name__ == "__main__":
    unittest.main()
