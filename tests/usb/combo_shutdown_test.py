#!/usr/bin/env python3
"""Exercise combo-stress shutdown with a backpressured CDC loopback."""
from pathlib import Path
import queue
import sys
import threading
import time
import unittest
from unittest.mock import Mock, patch

sys.path.insert(0, str(Path(__file__).resolve().parents[2] / 'Python'))
import usb_combo_stress as bench


class SerialLoop:
    """A bounded echo path whose writer needs the reader to keep draining."""
    def __init__(self):
        self.data = queue.Queue(maxsize=1)
        self.reading = threading.Event()
        self.writing = threading.Event()
        self.release_read = threading.Event()

    def read(self, size):
        self.reading.set()
        if not self.release_read.wait(1.0):
            raise RuntimeError('test did not release reader')
        try:
            return self.data.get(timeout=0.05)
        except queue.Empty:
            return b''

    def write(self, data):
        self.writing.set()
        for offset in range(0, len(data), 4):
            try:
                self.data.put(data[offset:offset + 4], timeout=0.2)
            except queue.Full:
                raise TimeoutError('Write timeout')
        return len(data)


class ComboShutdownTest(unittest.TestCase):
    def setUp(self):
        self.start = threading.Event()
        self.stop = threading.Event()
        self.tx_stop = threading.Event()
        self.stats = bench.Stats()
        self.comm = SerialLoop()
        self.workers = [
            threading.Thread(target=bench.loop_tx_worker, args=(
                self.comm, self.start, self.stop, self.stats, 12, self.tx_stop)),
            threading.Thread(target=bench.loop_rx_worker, args=(
                self.comm, self.start, self.stop, self.stats, 4)),
        ]

    def tearDown(self):
        self.tx_stop.set()
        self.stop.set()
        self.start.set()
        self.comm.release_read.set()
        for worker in self.workers:
            if worker.ident is not None:
                worker.join(timeout=1.0)
                self.assertFalse(worker.is_alive())

    def start_blocked_write(self):
        for worker in self.workers:
            worker.start()
        self.start.set()
        self.assertTrue(self.comm.reading.wait(1.0))
        self.assertTrue(self.comm.writing.wait(1.0))
        self.assertIsNone(self.stats.snapshot()[-1])

    def test_simultaneous_stop_reproduces_late_write_timeout(self):
        self.start_blocked_write()
        stopped = time.monotonic()
        self.stop.set()
        self.comm.release_read.set()
        for worker in self.workers:
            worker.join(timeout=1.0)
        self.assertEqual(self.stats.snapshot()[-1], 'CDC loop TX: Write timeout')
        self.assertGreaterEqual(self.stats.failure_timestamp(), stopped)

    def test_writer_finishes_while_receiver_remains_active(self):
        self.start_blocked_write()
        # Release the pending read only after shutdown has requested TX stop.
        def release():
            if self.tx_stop.wait(1.0):
                self.comm.release_read.set()
        gate = threading.Thread(target=release)
        gate.start()
        stop_started, test_end = bench.stop_workers(
            self.workers, self.tx_stop, self.stop, self.stats)
        gate.join(timeout=1.0)
        count, _, errors, _, _, _, failure = self.stats.snapshot()
        self.assertIsNone(failure)
        self.assertIsNone(self.stats.failure_timestamp())
        self.assertEqual(errors, 0)
        self.assertEqual(count['loop_tx'], 12)
        self.assertGreaterEqual(count['loop_rx'], 8)
        self.assertGreaterEqual(test_end, stop_started)
        self.assertTrue(all(not worker.is_alive() for worker in self.workers))

    def test_real_timeout_during_final_write_still_fails(self):
        self.start.set()
        def fail_write(data):
            self.tx_stop.set()
            raise TimeoutError('Write timeout')
        self.comm.write = fail_write
        bench.loop_tx_worker(self.comm, self.start, self.stop,
                             self.stats, 12, self.tx_stop)
        self.assertTrue(self.stop.is_set())
        self.assertEqual(self.stats.snapshot()[-1], 'CDC loop TX: Write timeout')
        self.assertEqual(self.stats.snapshot()[0]['loop_tx'], 0)

    def test_writer_that_never_exits_is_a_failure(self):
        writer = Mock()
        writer.is_alive.return_value = True
        bench.stop_workers([writer], self.tx_stop, self.stop, self.stats)
        self.assertEqual(self.stats.snapshot()[-1], 'CDC loop TX: writer did not stop')
        self.assertTrue(self.stop.is_set())

    def test_first_failure_and_time_are_preserved(self):
        with patch.object(bench.time, 'monotonic', return_value=10):
            self.stats.fail('CDC loop TX', 'Write timeout')
        with patch.object(bench.time, 'monotonic', return_value=20):
            self.stats.fail('ISO', 'shutdown error')
        self.assertEqual(self.stats.snapshot()[-1], 'CDC loop TX: Write timeout')
        self.assertEqual(self.stats.failure_timestamp(), 10)


if __name__ == '__main__':
    unittest.main()
