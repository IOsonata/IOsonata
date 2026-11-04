"""Check the exact, division-helper-free R9A02 microsecond delay arithmetic."""

from pathlib import Path
import random
import unittest


SOURCE = (Path(__file__).resolve().parents[1]
          / "RISCV/Renesas/R9A02/R9A02G021/lib/src/system_r9a02.c")


def cycles_without_long_division(us, hz):
    mhz, rhz = divmod(hz, 1000000)
    sec, rem = divmod(us, 1000000)
    a, b = divmod(rem, 1000)
    c, d = divmod(rhz, 1000)
    frac = (a * d + b * c) * 1000 + b * d
    assert frac + 999999 < 2**31
    return us * mhz + sec * rhz + a * c + (frac + 999999) // 1000000


class R9A02DelayMathTest(unittest.TestCase):
    def test_exact_ceiling_for_short_and_long_delays(self):
        rnd = random.Random(0x9A02)
        inputs = [0, 1, 2, 31, 999, 1000, 1001, 999999,
                  1000000, 4294967295]
        clocks = [0, 32768, 500000, 1000000, 8000000,
                  48000000, 4294967295]
        for us in inputs:
            for hz in clocks:
                with self.subTest(us=us, hz=hz):
                    expected = (us * hz + 999999) // 1000000
                    self.assertEqual(cycles_without_long_division(us, hz),
                                     expected)
        for _ in range(2000):
            us = rnd.getrandbits(32)
            hz = rnd.getrandbits(32)
            expected = (us * hz + 999999) // 1000000
            self.assertEqual(cycles_without_long_division(us, hz), expected)

    def test_source_uses_32_bit_divisors(self):
        source = SOURCE.read_text(encoding="utf-8")
        func = source.split("void R9a02DelayUs(uint32_t Us)", 1)[1]
        func = func.split("static volatile uint32_t *R9a02MstpReg", 1)[0]
        self.assertIn("frac = (a * d + b * c) * 1000U + b * d;", func)
        self.assertIn("(frac + 999999U) / 1000000U", func)
        self.assertNotIn("/ 1000000ULL", func)


if __name__ == "__main__":
    unittest.main()
