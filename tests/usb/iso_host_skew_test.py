#!/usr/bin/env python3
"""Exercise ISO burst classification of host transfer results."""

import importlib.util
from pathlib import Path
from types import SimpleNamespace
import unittest
from unittest.mock import patch


SOURCE = Path(__file__).resolve().parents[2] / "Python/usb_iso_loopback.py"
SPEC = importlib.util.spec_from_file_location("iso_loopback", SOURCE)
iso = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(iso)

USBError = type("USBError", (Exception,), {})

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
    USBError=USBError,
    USBErrorInterrupted=type("USBErrorInterrupted", (USBError,), {}),
    USBErrorOther=type("USBErrorOther", (USBError,), {}),
    USBErrorNoDevice=type("USBErrorNoDevice", (USBError,), {}),
)


class Clock:
    now = 0.0

    def monotonic(self):
        return self.now


class Transfer:
    def __init__(self, packets, done, status=USB.TRANSFER_COMPLETED,
                 actual=None, submit_error=None):
        self.packets = packets
        self.done = done
        self.status = status
        # Host-reported byte count per frame, where it differs from the data.
        self.actual = actual or {}
        self.submit_error = submit_error
        self.submitted = False

    def setIsochronous(self, endpoint, buffer, callback, **kwargs):
        self.callback = callback
        assert len(kwargs["iso_transfer_length_list"]) == len(self.packets)

    def submit(self):
        if self.submit_error is not None:
            raise self.submit_error
        self.submitted = True

    def cancel(self):
        self.submitted = False

    def isSubmitted(self):
        return self.submitted

    def getStatus(self):
        return self.status

    def getISOSetupList(self):
        return [
            {
                "status": USB.TRANSFER_COMPLETED,
                "actual_length": self.actual.get(index, len(packet)),
            }
            for index, packet in enumerate(self.packets)
        ]

    def iterISO(self):
        return [(USB.TRANSFER_COMPLETED, packet) for packet in self.packets]


class Host:
    def __init__(self, clock, received, out_done, in_status, out_actual=None,
                 out_submit_error=None):
        self.clock = clock
        payloads = [iso.build_payload(6, 63, i) for i in range(48)]
        packets = [payloads[i] for i in received]
        packets += [b""] * (64 - len(packets))
        self.transfers = [
            Transfer(packets, 0.067, in_status),
            Transfer(payloads, out_done, actual=out_actual,
                     submit_error=out_submit_error),
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


class BurstCase(unittest.TestCase):
    def burst(self, received, out_done=0.071, in_status=USB.TRANSFER_COMPLETED,
              out_actual=None, out_submit_error=None):
        clock = Clock()
        host = Host(clock, received, out_done, in_status, out_actual,
                    out_submit_error)
        with patch.object(iso, "time", clock), patch.object(iso, "usb1", USB):
            return iso.run_burst(host, host, 8, 6, 63, 63, 32, 1000, 0)


class IsoHostSkewTest(BurstCase):
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


class IsoHostUnsentTest(BurstCase):
    """OUT frames the host reports as not sent, and refused submits."""

    def test_unsent_leading_guard_frame_is_tolerated(self):
        # The host reports OUT frame 0 sent with 0 bytes, so the device has
        # nothing to echo for it. Every validation frame still goes through.
        error, result = self.burst(range(1, 48), out_actual={0: 0})
        self.assertIsNone(error)
        self.assertEqual(result["out_unsent"], 1)
        self.assertEqual(result["matched"], 47)

    def test_unsent_validation_frame_is_host_miss(self):
        unsent = range(0, 10)
        error, result = self.burst(
            range(10, 48), out_actual={index: 0 for index in unsent}
        )
        self.assertTrue(iso.is_host_sched_miss_error(error))
        self.assertIn("not sent by host (10 of 48)", error)
        self.assertIsNone(result)

    def test_short_out_frame_stays_a_failure(self):
        error, _ = self.burst(range(48), out_actual={20: 10})
        self.assertTrue(error.startswith("OUT packet 20:"))
        self.assertFalse(iso.is_host_sched_error(error))

    def test_refused_submit_is_host_miss(self):
        error, result = self.burst(
            range(48), out_submit_error=USB.USBErrorOther("LIBUSB_ERROR_OTHER")
        )
        self.assertTrue(iso.is_host_sched_miss_error(error))
        self.assertIn("submit refused", error)
        self.assertIsNone(result)

    def test_other_submit_error_stays_a_failure(self):
        error, _ = self.burst(
            range(48),
            out_submit_error=USB.USBErrorNoDevice("LIBUSB_ERROR_NO_DEVICE"),
        )
        self.assertTrue(error.startswith("submit failed:"))
        self.assertFalse(iso.is_host_sched_error(error))


if __name__ == "__main__":
    unittest.main()
