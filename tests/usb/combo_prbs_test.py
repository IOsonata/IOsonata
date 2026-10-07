#!/usr/bin/env python3
"""Keep combo-stress PRBS generation and error counts equivalent to scalar code."""
from pathlib import Path
import random
import sys
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parents[2] / 'Python'))
import usb_combo_stress as bench


def scalar_next(value):
    return ((value << 1) | (((value >> 6) ^ (value >> 5)) & 1)) & 0x7f


def scalar_block(state, length):
    data = bytearray(length)
    for index in range(length):
        state = scalar_next(state)
        data[index] = state
    return bytes(data), state


def scalar_check(data, expected, markers=False):
    errors = 0
    target_errors = 0
    for value in data:
        if markers and value == 0:
            target_errors += 1
            continue
        if (not markers or expected is not None) and value != expected:
            errors += 1
        expected = scalar_next(value)
    if markers:
        return expected, errors, target_errors
    return expected, errors


class ComboPrbsTest(unittest.TestCase):
    def test_generator_all_byte_states_and_packet_boundaries(self):
        for state in range(256):
            for length in (0, 1, 2, 63, 64, 126, 127, 128, 255, 511, 512, 4096):
                with self.subTest(state=state, length=length):
                    self.assertEqual(bench.make_prbs_block(state, length),
                                     scalar_block(state, length))
        with self.assertRaises(ValueError):
            bench.make_prbs_block(255, -1)

    def test_generator_preserves_state_across_blocks(self):
        lengths = (1, 0, 126, 512, 3, 4096, 127, 64)
        for initial in (0, 1, 126, 127, 128, 255):
            state = initial
            blocks = []
            for length in lengths:
                block, state = bench.make_prbs_block(state, length)
                blocks.append(block)
            self.assertEqual((b''.join(blocks), state),
                             scalar_block(initial, sum(lengths)))

    def test_checker_matches_every_byte_transition(self):
        # Include high-bit corruption and zero, not just valid PRBS states.
        data = bytes(value for left in range(256) for right in range(256)
                     for value in (left, right))
        for expected in (None, 0, 1, 126, 255):
            self.assertEqual(bench.check_stream(data, expected),
                             scalar_check(data, expected))
            self.assertEqual(bench.check_prbs_stream(data, expected),
                             scalar_check(data, expected, markers=True))

    def test_clean_and_damaged_streams_across_read_boundaries(self):
        clean, _ = scalar_block(255, 8192)
        rng = random.Random(42)
        streams = (
            clean,
            clean[:512] + clean[1024:],       # Drop one HS packet.
            clean[:127] + b'\xff' + clean[127:],
            clean[:4095] + b'\x00\x80' + clean[4097:],
            bytes(rng.randrange(256) for _ in range(8192)),
        )
        for data in streams:
            for chunk_size in (1, 63, 127, 512, 4096):
                for markers in (False, True):
                    expected = None if markers else scalar_next(255)
                    errors = 0
                    target_errors = 0
                    for offset in range(0, len(data), chunk_size):
                        chunk = data[offset:offset + chunk_size]
                        if markers:
                            expected, count, target_count = bench.check_prbs_stream(
                                chunk, expected)
                            target_errors += target_count
                        else:
                            expected, count = bench.check_stream(chunk, expected)
                        errors += count
                    initial = None if markers else scalar_next(255)
                    result = ((expected, errors, target_errors) if markers
                              else (expected, errors))
                    self.assertEqual(result, scalar_check(data, initial, markers))

    def test_target_markers_do_not_advance_pattern(self):
        clean, state = scalar_block(255, 512)
        marked = b'\x00\x00' + b'\x00'.join(bytes([v]) for v in clean) + b'\x00'
        self.assertEqual(bench.check_prbs_stream(marked, None),
                         (scalar_next(state), 0, 514))
        for expected in (None, 0, 126):
            self.assertEqual(bench.check_stream(b'', expected), (expected, 0))
            self.assertEqual(bench.check_prbs_stream(b'', expected), (expected, 0, 0))
            self.assertEqual(bench.check_prbs_stream(b'\x00' * 512, expected),
                             (expected, 0, 512))


if __name__ == '__main__':
    unittest.main()
