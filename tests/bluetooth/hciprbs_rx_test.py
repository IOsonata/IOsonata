"""Host tests for PRBS notification reception; no radio throughput claims."""
import collections
import contextlib
import io
from pathlib import Path
import struct
import sys
import unittest
from unittest.mock import patch

sys.path.insert(0, str(Path(__file__).resolve().parents[2] / 'Python'))
import hciprbs_rx as rx
from uartprbs_rx import PrbsCheck, NEXT

API = rx.hci_library()


class Wire:
    def __init__(self):
        self.packets = collections.deque()
        self.sent = []

    def read_packet(self, timeout):
        return self.packets.popleft() if self.packets else None

    def send_acl(self, handle, cid, pdu):
        self.sent.append((handle, cid, pdu))


def sequence(size, state=0xFF):
    data = bytearray()
    for _ in range(size):
        state = NEXT[state]
        data.append(state)
    return bytes(data)


def acl(data, handle=1, continuing=False):
    return API.H4_ACL, None, struct.pack('<HH', handle | (0x1000 if continuing else 0x2000), len(data)) + data


def notification(data, handle=0x12):
    pdu = struct.pack('<BH', API.ATT_HANDLE_VALUE_NTF, handle) + data
    return struct.pack('<HH', len(pdu), API.CID_ATT) + pdu


class PrbsReceiverTest(unittest.TestCase):
    def setUp(self):
        self.wire = Wire()
        self.now = 0.0
        self.receiver = rx.Receiver(PrbsCheck(), lambda: self.now)
        self.receiver.handle = 0x12
        self.link = rx.PrbsLink(self.wire, 1, self.receiver, API)

    def test_fragmented_notifications_keep_prbs_phase(self):
        data = sequence(488)
        first = notification(data[:244])
        self.link.packet(acl(first[:32]))
        self.link.packet(acl(first[32:], continuing=True))
        self.now = 1.0
        self.link.packet(acl(notification(data[244:])))
        self.assertEqual(self.receiver.check.count, 488)
        self.assertFalse(self.receiver.failed())

    def test_burst_boundary_does_not_drop_packet_257(self):
        data = sequence(257 * 20)
        self.wire.packets.extend(acl(notification(data[i:i + 20])) for i in range(0, len(data), 20))
        self.link.pump(0)
        self.assertEqual(len(self.wire.packets), 1)
        self.link.pump(0)
        self.assertEqual(self.receiver.check.count, len(data))
        self.assertFalse(self.receiver.failed())

    def test_wrong_handle_is_not_counted(self):
        self.link.packet(acl(notification(sequence(244), 0x13)))
        self.assertEqual(self.receiver.check.count, 0)
        self.assertEqual(self.receiver.other, 1)
        self.assertTrue(self.receiver.failed())

    def test_corruption_fails_even_after_recovery(self):
        data = bytearray(sequence(488))
        data[100] = 0x80
        self.receiver.notify(0x12, data)
        self.assertTrue(self.receiver.failed())
        self.assertGreater(self.receiver.check.drop, 0)
        self.assertEqual(self.receiver.check.out_of_frame, 1)

    def test_idle_time_is_included_in_throughput(self):
        self.receiver.notify(0x12, sequence(244))
        self.now = 2.0
        output = io.StringIO()
        with contextlib.redirect_stdout(output):
            self.receiver.report()
        self.assertIn('average: 122.00', output.getvalue())

    def test_phy_change_rejects_mislabeled_measurement(self):
        self.link.required_phy = 2
        body = struct.pack('<BBHBB', 0x0C, 0, 1, 1, 1)
        with self.assertRaisesRegex(RuntimeError, 'PHY changed'):
            self.link.packet((API.H4_EVENT, API.EVT_LE_META, body))

    def test_read_phy_confirms_both_directions(self):
        self.wire.command = lambda opcode, *args, **kwargs: (0, struct.pack('<HBB', 1, 2, 2))
        rx.configure_phy(self.link, '2M')
        self.assertEqual(self.link.required_phy, 2)

    def test_phy_fallback_cannot_pass(self):
        self.wire.command = lambda opcode, *args, **kwargs: (0, struct.pack('<HBB', 1, 1, 1))
        with patch.object(rx.time, 'monotonic', side_effect=[0, 0, 6]):
            with self.assertRaisesRegex(RuntimeError, 'not negotiated'):
                rx.configure_phy(self.link, '2M')

    def test_orphan_fragment_fails(self):
        with self.assertRaisesRegex(RuntimeError, 'continuation'):
            self.link.packet(acl(b'bad', continuing=True))

    def test_subscription_discovers_actual_cccd(self):
        characteristic = (
            bytes([API.ATT_READ_BY_TYPE_RSP, 21]) + struct.pack('<HBH', 0x11, 0x10, 0x12) + API.uuid128(rx.DATA_UUID))
        responses = iter([struct.pack('<BH', API.ATT_EXCHANGE_MTU_RSP, 247), characteristic, None,
                          struct.pack('<BBHH', API.ATT_FIND_INFORMATION_RSP, 1, 0x15, API.UUID_CCCD), bytes([API.ATT_WRITE_RSP])])
        requests = []
        def request(pdu, expected):
            requests.append(pdu)
            return next(responses)
        self.link.request = request
        self.assertEqual(self.link.subscribe(), 0x12)
        self.assertEqual(requests[-1], struct.pack('<BHH', API.ATT_WRITE_REQ, 0x15, 1))

    def test_missing_cccd_fails_without_guessing(self):
        characteristic = bytes([9, 21]) + struct.pack('<HBH', 0x11, 0x10, 0x12) + API.uuid128(rx.DATA_UUID)
        responses = iter([struct.pack('<BH', 3, 247), characteristic, None, None])
        self.link.request = lambda *args: next(responses)
        with self.assertRaisesRegex(RuntimeError, 'CCCD not found'):
            self.link.subscribe()

    def test_defaults_compare_both_phys(self):
        args = rx.parse_args([])
        self.assertEqual(args.phy, 'both')
        self.assertEqual(args.duration, 60)
        self.assertEqual(args.interval, 15)


if __name__ == '__main__':
    unittest.main()
