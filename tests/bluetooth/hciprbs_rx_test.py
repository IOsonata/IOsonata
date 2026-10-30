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

    def test_matching_phy_does_not_start_another_procedure(self):
        commands = []
        def command(opcode, *args, **kwargs):
            commands.append(opcode)
            return 0, struct.pack('<HBB', 1, 1, 1)
        self.wire.command = command
        rx.configure_phy(self.link, '1M')
        self.assertEqual(commands, [0x2030])

    def test_changed_phy_waits_for_completion_before_readback(self):
        commands = []
        def command(opcode, *args, **kwargs):
            commands.append(opcode)
            if opcode == 0x2032:
                self.wire.packets.append((API.H4_EVENT, API.EVT_LE_META,
                                         struct.pack('<BBHBB', 0x0C, 0, 1, 2, 2)))
                return 0, b''
            if len(commands) == 1:
                return 0, struct.pack('<HBB', 1, 1, 1)
            self.assertEqual(self.link.phy, (2, 2))
            return 0, struct.pack('<HBB', 1, 2, 2)
        self.wire.command = command
        rx.configure_phy(self.link, '2M')
        self.assertEqual(commands, [0x2030, 0x2032, 0x2030])
        self.assertEqual(self.link.required_phy, 2)

    def test_readback_cannot_replace_missing_completion(self):
        reads = []
        def command(opcode, *args, **kwargs):
            if opcode == 0x2030:
                reads.append(opcode)
                return 0, struct.pack('<HBB', 1, 1 if len(reads) == 1 else 2, 1 if len(reads) == 1 else 2)
            return 0, b''
        self.wire.command = command
        with patch.object(rx.time, 'monotonic', side_effect=[0, 0, 6]):
            with self.assertRaisesRegex(RuntimeError, 'not negotiated'):
                rx.configure_phy(self.link, '2M')
        self.assertEqual(len(reads), 1)

    def test_phy_fallback_cannot_pass(self):
        self.wire.command = lambda opcode, *args, **kwargs: (0, struct.pack('<HBB', 1, 1, 1))
        with patch.object(rx.time, 'monotonic', side_effect=[0, 0, 6]):
            with self.assertRaisesRegex(RuntimeError, 'not negotiated'):
                rx.configure_phy(self.link, '2M')

    def test_att_disconnect_reports_reason(self):
        self.link.down = 0x08
        with self.assertRaisesRegex(RuntimeError, 'ATT request, reason 0x08'):
            self.link.request(b'\x02\xf7\x00', 3)

    def test_phy_disconnect_reports_reason(self):
        self.wire.command = lambda *args, **kwargs: (0, b'')
        self.link.down = 0x3E
        with self.assertRaisesRegex(RuntimeError, 'PHY negotiation, reason 0x3E'):
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

    def test_both_phys_restore_controller_event_budget(self):
        for phy in ('1M', '2M'):
            with self.subTest(phy=phy):
                self.check_scan_name('BlePrbs_S132', ['--phy', phy])

    def test_scanning_accepts_all_prbs_variants(self):
        for name in rx.PRBS_NAMES:
            with self.subTest(name=name):
                self.check_scan_name(name)

    def test_explicit_name_filters_other_variants(self):
        self.check_scan_name('BlePrbs_S132', ['--name', 'BlePrbs_S132'],
                             ignored='BlePrbs_SDC')

    def check_scan_name(self, name, argv=None, ignored='Unrelated'):
        peer = bytes.fromhex('0102030405c0')
        def advertisement(label):
            data = bytes([len(label) + 1, 9]) + label.encode()
            body = bytes([2, 1, 0, 1]) + peer + bytes([len(data)]) + data + bytes([200])
            return API.H4_EVENT, API.EVT_LE_META, body
        self.wire.setup = lambda: None
        self.wire.identity = lambda: (peer, 1, None)
        self.wire.pending = []
        commands = []
        self.wire.command = lambda *args, **kwargs: (commands.append(args) or (0, b''))
        self.wire.packets.extend([advertisement(ignored), advertisement(name),
                                 (API.H4_EVENT, API.EVT_LE_META, b'connection')])
        with patch.object(API, 'parse_connection', return_value=(0, 1, 0, peer, 12, 0, 400)):
            self.assertEqual(rx.connect(self.wire, API, rx.parse_args(argv or [])), (1, 15))
        self.assertFalse(self.wire.packets)
        self.assertEqual(commands[0], (0xFD05, struct.pack('<I', 7500)))

    def test_defaults_compare_both_phys(self):
        args = rx.parse_args([])
        self.assertIsNone(args.name)
        self.assertEqual(args.phy, 'both')
        self.assertEqual(args.duration, 60)
        self.assertEqual(args.interval, 15)


if __name__ == '__main__':
    unittest.main()
