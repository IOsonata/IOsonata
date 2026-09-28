#!/usr/bin/env python3
"""Exercise ISO burst classification with completed host transfer results."""

import importlib.util
from pathlib import Path
from types import SimpleNamespace
import unittest
from unittest.mock import patch


SOURCE = Path(__file__).resolve().parents[2] / "Python/usb_iso_loopback.py"
SPEC = importlib.util.spec_from_file_location("iso_loopback", SOURCE)
iso = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(iso)

USB = SimpleNamespace(
    ENDPOINT_IN=0x80,
    ENDPOINT_OUT=0,
    TRANSFER_COMPLETED=0,
    TRANSFER_ERROR=1,
    TRANSFER_TIMED_OUT=2,
    TRANSFER_CANCELLED=3,
    TRANSFER_STALL=4,
    TRANSFER_NO_DEVICE=5,
    TRANSFER_OVERFLOW=6,
    USBError=type("USBError", (Exception,), {}),
    USBErrorInterrupted=type("USBErrorInterrupted", (Exception,), {}),
)


class Clock:
    now = 0.0

    def monotonic(self):
        return self.now


class Transfer:
    def __init__(self, packets, done, status=USB.TRANSFER_COMPLETED):
        self.packets = packets
        self.done = done
        self.status = status
        self.submitted = False

    def setIsochronous(self, endpoint, buffer, callback, **kwargs):
        self.callback = callback
        assert len(kwargs["iso_transfer_length_list"]) == len(self.packets)

    def submit(self):
        self.submitted = True

    def isSubmitted(self):
        return self.submitted

    def getStatus(self):
        return self.status

    def getISOSetupList(self):
        return [
            {"status": USB.TRANSFER_COMPLETED, "actual_length": len(packet)}
            for packet in self.packets
        ]

    def iterISO(self):
        return [(USB.TRANSFER_COMPLETED, packet) for packet in self.packets]


class Host:
    def __init__(self, clock, received, out_done, in_status):
        self.clock = clock
        payloads = [iso.build_payload(6, 63, i) for i in range(48)]
        packets = [payloads[i] for i in received]
        packets += [b""] * (64 - len(packets))
        self.transfers = [
            Transfer(packets, 0.067, in_status),
            Transfer(payloads, out_done),
        ]
        self.allocated = 0

    def getTransfer(self, iso_packets):
        transfer = self.transfers[self.allocated]
        self.allocated += 1
        assert len(transfer.packets) == iso_packets
        return transfer

    def handleEvents(self):
        transfer = min(
            (item for item in self.transfers if item.submitted),
            key=lambda item: item.done,
        )
        self.clock.now = transfer.done
        transfer.submitted = False
        transfer.callback(transfer)


class IsoHostSkewTest(unittest.TestCase):
    def burst(self, received, out_done=0.071, in_status=USB.TRANSFER_COMPLETED):
        clock = Clock()
        host = Host(clock, received, out_done, in_status)
        with patch.object(iso, "time", clock), patch.object(iso, "usb1", USB):
            return iso.run_burst(host, host, 8, 6, 63, 63, 32, 1000, 0)

    def test_complete_validation(self):
        error, result = self.burst(range(48))
        self.assertIsNone(error)
        self.assertEqual(result["matched"], 48)

    def test_reported_late_out_tail(self):
        error, result = self.burst(range(22), out_done=0.0895)
        self.assertTrue(iso.is_host_sched_skew_error(error))
        self.assertIn("14-31", error)
        self.assertIsNone(result)

    def test_later_guards_disprove_missing_tail_is_capture_end(self):
        # Validation indices 30/31 are OUT indices 38/39. The captured
        # guard echoes 40..43 establish that IN covered the missing tail.
        for missing in ((39,), (38, 39)):
            with self.subTest(missing=missing):
                error, result = self.burst(
                    [i for i in range(44) if i not in missing]
                )
                self.assertTrue(error.startswith("missing validation frame(s)"))
                self.assertFalse(iso.is_host_sched_error(error))
                self.assertIsNone(result)

    def test_interior_loss_stays_a_failure(self):
        error, _ = self.burst([i for i in range(44) if i != 20])
        self.assertIn("missing validation frame(s) [12]", error)
        self.assertFalse(iso.is_host_sched_error(error))

    def test_earlier_out_completion_is_not_skew(self):
        error, _ = self.burst(range(22), out_done=0.050)
        self.assertTrue(error.startswith("missing validation frame(s)"))
        self.assertFalse(iso.is_host_sched_error(error))

    def test_failed_in_transfer_is_not_skew(self):
        error, _ = self.burst(range(22), in_status=USB.TRANSFER_TIMED_OUT)
        self.assertTrue(error.startswith("missing validation frame(s)"))
        self.assertFalse(iso.is_host_sched_error(error))


if __name__ == "__main__":
    unittest.main()
