#!/usr/bin/env python3
"""Exercise CDC+ISO result accounting and drain order without USB hardware."""
from pathlib import Path
import sys
import threading
import unittest
sys.path.insert(0, str(Path(__file__).resolve().parents[2] / 'Python'))
import usb_cdc_iso_stress as runner


class RunnerTest(unittest.TestCase):
    def test_pass_requires_both_paths_and_complete_echo(self):
        stats = runner.Stats()
        self.assertFalse(runner.passed(stats))
        stats.add('loop_tx', 128)
        stats.add('loop_rx', 64)
        stats.add('iso', 63)
        self.assertFalse(runner.passed(stats))
        stats.add('loop_rx', 64)
        self.assertTrue(runner.passed(stats))
        stats.add('loop_rx', 1)
        self.assertFalse(runner.passed(stats))

    def test_corruption_and_iso_failure_fail(self):
        for corrupt in (True, False):
            stats = runner.Stats()
            stats.add('loop_tx', 64)
            stats.add('loop_rx', 64, errors=int(corrupt))
            stats.add('iso', 63)
            if not corrupt:
                stats.fail('ISO', 'packet mismatch')
            self.assertFalse(runner.passed(stats))

    def test_last_write_drains_before_readers_stop(self):
        stats = runner.Stats()
        stop, tx_stop, iso_stop = (threading.Event() for _ in range(3))
        stats.add('loop_tx', 64)
        class Worker:
            def __init__(self, name):
                self.name = name
                self.joined = False
            def join(self, timeout=None):
                if self.name == 'TX' and not self.joined:
                    assert tx_stop.is_set() and not stop.is_set()
                    stats.add('loop_rx', 64)
                    self.joined = True
                elif self.name == 'ISO':
                    assert iso_stop.is_set()
            def is_alive(self): return False
        workers = [Worker('TX'), Worker('RX'), Worker('ISO')]
        runner.stop_workers(workers, tx_stop, stop, iso_stop, stats, 0.1)
        self.assertTrue(stop.is_set())
        self.assertIsNone(stats.snapshot()[-1])


if __name__ == '__main__':
    unittest.main()
