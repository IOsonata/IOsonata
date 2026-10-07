#!/usr/bin/env python3
"""Exercise combo-stress failure attribution to host pauses and sleep."""
from pathlib import Path
import sys
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parents[2] / 'Python'))
import usb_combo_stress as bench


class HostPauseTest(unittest.TestCase):
    GRACE = 2.0

    def test_scheduling_pause_uses_the_transfer_grace(self):
        stats = bench.Stats()
        stats.add_host_pause(1.0, 100.0, 101.0)
        self.assertEqual(stats.host_pause_at(100.5, self.GRACE), 1.0)
        self.assertEqual(stats.host_pause_at(102.9, self.GRACE), 1.0)
        self.assertIsNone(stats.host_pause_at(103.1, self.GRACE))
        self.assertIsNone(stats.host_pause_at(99.9, self.GRACE))

    def test_sleep_covers_the_bus_resume(self):
        # The monotonic clock stops during sleep, so the window itself is one
        # loop pass wide; the failure comes after the process runs again.
        stats = bench.Stats()
        stats.add_host_pause(14.9, 100.0, 100.02, asleep=True)
        wake = 100.02
        self.assertEqual(stats.host_pause_at(wake + 5.0, self.GRACE), 14.9)
        self.assertEqual(
            stats.host_pause_at(wake + bench.HOST_WAKE_GRACE_S, self.GRACE),
            14.9,
        )
        self.assertIsNone(
            stats.host_pause_at(wake + bench.HOST_WAKE_GRACE_S + 0.1,
                                self.GRACE)
        )

    def test_pause_moves_progress_forward(self):
        stats = bench.Stats()
        stats.add("iso", 63)
        before = stats.snapshot()[1]["iso"]
        stats.add_host_pause(3.0, before, before + 3.0, asleep=True)
        self.assertAlmostEqual(stats.snapshot()[1]["iso"], before + 3.0)
        self.assertEqual(stats.host_pause_summary(), (1, 3.0))


if __name__ == '__main__':
    unittest.main()
