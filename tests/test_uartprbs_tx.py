"""Verify host PRBS continuity and SLIP escaping without serial hardware."""
import importlib.util
from pathlib import Path
import unittest

spec = importlib.util.spec_from_file_location('uartprbs_tx', Path(__file__).resolve().parents[1] / 'Python/uartprbs_tx.py')
tx = importlib.util.module_from_spec(spec)
spec.loader.exec_module(tx)


class PrbsTx(unittest.TestCase):
    def test_continuity(self):
        state, first = tx.payload(0xff, 92)
        _, second = tx.payload(state, 92)
        _, whole = tx.payload(0xff, 184)
        self.assertEqual(first + second, whole)
        self.assertEqual(whole[:4], bytes([126, 124, 120, 112]))
        self.assertEqual(whole[:57], whole[127:])

    def test_slip_escaping(self):
        self.assertEqual(tx.slip_encode(bytes([0, 0xc0, 0xdb, 255])),
                         bytes([0xc0, 0, 0xdb, 0xdc, 0xdb, 0xdd, 255, 0xc0]))


if __name__ == '__main__':
    unittest.main()
