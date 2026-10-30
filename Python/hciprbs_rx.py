#!/usr/bin/env python3
"""-------------------------------------------------------------------------
@file   hciprbs_rx.py
@brief  Receive and validate BleIntrfPrbsTx through HciController.

Run from IOsonata:
    python3 Python/hciprbs_rx.py --duration 60
    python3 Python/hciprbs_rx.py --name BlePrbs_S140 --interval 15
    python3 Python/hciprbs_rx.py --hci /dev/cu.usbmodemXXXX

Uses HciController/python directly and sibling uartprbs_rx.py PrbsCheck.
Requires the HciController Python package (or a sibling checkout), pyserial,
and PyUSB/libusb for native USB. No camera or display packages are required.

Rates count received characteristic payload only. PRBS repeats every 127
bytes: sequence checking cannot detect removal of complete 127-byte periods.
Reported loss is an estimate, not an exact missing-byte count. Host arrival
bursts do not establish packets per radio connection event.

@author Hoang Nguyen Hoan
@date   Oct. 30, 2026
@license MIT License, (c) 2026 I-SYST inc. See LICENSE.
----------------------------------------------------------------------------"""
import argparse
import collections
import copy
import os
import math
import struct
import sys
import time

# HCI framing, transport, command completion and ACL credits belong to the
# hcicontroller package. This client supplies only the PRBS GATT procedure.
DATA_UUID = '00000102-287c-11e4-ab74-0002a5d5c51b'
PRBS_NAMES = ('BlePrbs', 'BlePrbs_S132', 'BlePrbs_S140', 'BlePrbs_SDC')


def hci_library():
    try:
        import hcicontroller
    except ModuleNotFoundError as error:
        if error.name != 'hcicontroller':
            raise
        here = os.path.dirname(os.path.abspath(__file__))
        for path in (os.environ.get('HCICONTROLLER_PYTHON'),
                     os.path.join(here, '..', '..', 'HciController', 'python')):
            if path and os.path.isdir(os.path.join(path, 'hcicontroller')):
                sys.path.insert(0, os.path.abspath(path))
                break
    from hcicontroller import hci_ble_test
    return hci_ble_test


class PrbsLink:
    def __init__(self, hci, handle, receiver, api):
        self.hci = hci
        self.handle = handle
        self.receiver = receiver
        self.api = api
        self.rx = bytearray()
        self.responses = collections.deque()
        self.down = None
        self.mtu = 23
        self.phy = None
        self.required_phy = None
        self.interval = None

    def pump(self, timeout):
        received = False
        for index in range(256):
            packet = self.hci.read_packet(timeout if index == 0 else 0)
            if packet is None:
                break
            self.packet(packet)
            received = True
        return received

    def packet(self, packet):
        kind, code, body = packet
        if kind == self.api.H4_EVENT:
            if code == 0x05 and len(body) >= 4 and struct.unpack_from('<H', body, 1)[0] == self.handle:
                self.down = body[3]
            elif code == self.api.EVT_LE_META and len(body) >= 3:
                if body[0] == 0x0C and len(body) >= 6 and struct.unpack_from('<H', body, 2)[0] == self.handle:
                    if body[1]:
                        raise RuntimeError('PHY update failed: 0x%02X' % body[1])
                    self.phy = (body[4], body[5])
                    print('PHY: %s out, %s in' % tuple({1: '1M', 2: '2M', 3: 'Coded'}.get(x, str(x)) for x in self.phy))
                    if self.required_phy is not None and self.phy != (self.required_phy,) * 2:
                        raise RuntimeError('PHY changed during measurement')
                elif body[0] == 0x07 and len(body) >= 11 and struct.unpack_from('<H', body, 1)[0] == self.handle:
                    tx, _, rx, _ = struct.unpack_from('<HHHH', body, 3)
                    print('Link layer packets: %d bytes out, %d bytes in' % (tx, rx))
                elif body[0] == 0x03 and len(body) >= 10 and body[1] == 0 and struct.unpack_from('<H', body, 2)[0] == self.handle:
                    self.interval = struct.unpack_from('<H', body, 4)[0] * 1.25
                    print('Connection interval updated: %.2f ms' % self.interval)
            return
        if kind != self.api.H4_ACL or len(body) < 4:
            return
        header, size = struct.unpack_from('<HH', body)
        if header & 0xFFF != self.handle:
            return
        if len(body) != size + 4:
            raise RuntimeError('invalid ACL length')
        if (header >> 12) & 3 == 1:
            if not self.rx:
                raise RuntimeError('ACL continuation without a first fragment')
            self.rx.extend(body[4:])
        else:
            if self.rx:
                raise RuntimeError('incomplete L2CAP packet replaced by new packet')
            self.rx = bytearray(body[4:])
        if len(self.rx) < 4:
            return
        size, cid = struct.unpack_from('<HH', self.rx)
        if size > 517:
            raise RuntimeError('unexpected L2CAP payload size')
        if len(self.rx) < size + 4:
            return
        if len(self.rx) != size + 4:
            raise RuntimeError('invalid L2CAP length')
        pdu = bytes(self.rx[4:])
        self.rx.clear()
        if not pdu:
            return
        if cid == self.api.CID_ATT:
            if pdu[0] == self.api.ATT_HANDLE_VALUE_NTF and len(pdu) >= 3:
                self.receiver.notify(struct.unpack_from('<H', pdu, 1)[0], pdu[3:])
            elif pdu[0] == self.api.ATT_EXCHANGE_MTU_REQ and len(pdu) == 3:
                self.mtu = max(23, min(247, struct.unpack_from('<H', pdu, 1)[0]))
                self.hci.send_acl(self.handle, cid, struct.pack('<BH', self.api.ATT_EXCHANGE_MTU_RSP, 247))
            elif pdu[0] in (self.api.ATT_ERROR_RSP, self.api.ATT_EXCHANGE_MTU_RSP,
                            self.api.ATT_READ_BY_TYPE_RSP, self.api.ATT_FIND_INFORMATION_RSP,
                            self.api.ATT_WRITE_RSP):
                self.responses.append(pdu)
            elif pdu[0] & 0x40 == 0:
                handle = struct.unpack_from('<H', pdu, 1)[0] if len(pdu) >= 3 else 0
                self.hci.send_acl(self.handle, cid, struct.pack('<BBHB', 1, pdu[0], handle, 6))
        elif cid == self.api.CID_SMP and pdu[0] in (1, 11):
            self.hci.send_acl(self.handle, cid, bytes([5, 5]))
        elif cid == self.api.CID_SIGNALING and len(pdu) >= 4:
            # Keep the requested test interval; reject peripheral changes.
            code, ident, length = struct.unpack_from('<BBH', pdu)
            if code == 0x12 and length == 8 and len(pdu) == 12:
                self.hci.send_acl(self.handle, cid, struct.pack('<BBHH', 0x13, ident, 2, 1))

    def request(self, pdu, expected):
        self.responses.clear()
        self.hci.send_acl(self.handle, self.api.CID_ATT, pdu)
        deadline = time.monotonic() + 5
        while time.monotonic() < deadline:
            if self.down is not None:
                raise RuntimeError('disconnected during ATT request')
            if self.responses:
                response = self.responses.popleft()
                if response[0] == expected:
                    return response
                if response[0] == 1 and len(response) == 5 and response[1] == pdu[0]:
                    if response[4] == 0x0A:
                        return None
                    raise RuntimeError('ATT request 0x%02X failed: 0x%02X' % (pdu[0], response[4]))
            self.pump(0.05)
        raise RuntimeError('ATT request timed out: 0x%02X' % pdu[0])

    def subscribe(self):
        api = self.api
        response = self.request(struct.pack('<BH', api.ATT_EXCHANGE_MTU_REQ, 247), api.ATT_EXCHANGE_MTU_RSP)
        if response is None or len(response) != 3:
            raise RuntimeError('MTU exchange failed')
        self.mtu = max(23, min(247, struct.unpack_from('<H', response, 1)[0]))
        print('ATT MTU %d' % self.mtu)
        start, value, end = 1, None, 0x10000
        while start <= 0xFFFF:
            response = self.request(struct.pack('<BHHH', api.ATT_READ_BY_TYPE_REQ, start, 0xFFFF, api.UUID_CHARACTERISTIC), api.ATT_READ_BY_TYPE_RSP)
            if response is None:
                break
            if len(response) < 2 or response[1] not in (7, 21) or (len(response) - 2) % response[1]:
                raise RuntimeError('invalid characteristic discovery response')
            last = start - 1
            for offset in range(2, len(response), response[1]):
                declaration, properties, handle = struct.unpack_from('<HBH', response, offset)
                if declaration <= last:
                    raise RuntimeError('non-increasing characteristic handles')
                last = declaration
                if value is not None:
                    end = declaration
                    break
                if response[offset + 5:offset + response[1]] == api.uuid128(DATA_UUID):
                    if properties & 0x10 == 0:
                        raise RuntimeError('PRBS characteristic does not support notifications')
                    value = handle
            if end != 0x10000:
                break
            start = last + 1
        if value is None:
            raise RuntimeError('BlueIO UART notification characteristic not found')
        start, cccd = value + 1, None
        while start < end:
            response = self.request(struct.pack('<BHH', api.ATT_FIND_INFORMATION_REQ, start, end - 1), api.ATT_FIND_INFORMATION_RSP)
            if response is None:
                break
            if len(response) < 2 or response[1] not in (1, 2):
                raise RuntimeError('invalid descriptor discovery response')
            step = 4 if response[1] == 1 else 18
            if (len(response) - 2) % step or len(response) == 2:
                raise RuntimeError('invalid descriptor list')
            last = start - 1
            for offset in range(2, len(response), step):
                handle = struct.unpack_from('<H', response, offset)[0]
                if handle <= last:
                    raise RuntimeError('non-increasing descriptor handles')
                last = handle
                if step == 4 and struct.unpack_from('<H', response, offset + 2)[0] == api.UUID_CCCD:
                    cccd = handle
            if cccd is not None:
                break
            start = last + 1
        if cccd is None:
            raise RuntimeError('notification CCCD not found')
        self.receiver.handle = value
        response = self.request(struct.pack('<BHH', api.ATT_WRITE_REQ, cccd, 1), api.ATT_WRITE_RSP)
        if response != bytes([api.ATT_WRITE_RSP]):
            raise RuntimeError('notification subscription failed')
        return value

    def disconnect(self):
        if self.down is None:
            self.hci.command(self.api.OP_DISCONNECT, struct.pack('<HB', self.handle, 0x13), allow_fail=True)
            deadline = time.monotonic() + 2
            while self.down is None and time.monotonic() < deadline:
                self.pump(0.05)


def connect(hci, api, args):
    hci.setup()
    identity, own_type, _ = hci.identity()
    if own_type == 1:
        hci.command(api.OP_LE_SET_RANDOM_ADDRESS, identity)
    # Connection, data length and PHY events; controller handles remote LL
    # connection parameter requests itself (bit 5 remains masked).
    hci.command(api.OP_LE_SET_EVENT_MASK, bytes.fromhex('dfff000000000000'))
    if args.address:
        peer_type, peer = int(args.address_type == 'random'), args.address
    else:
        names = (args.name,) if args.name else PRBS_NAMES
        print('Scanning for %s' % ', '.join(names))
        hci.command(api.OP_LE_SET_SCAN_PARAMS, struct.pack('<BHHBB', 1, 96, 48, own_type, 0))
        hci.command(api.OP_LE_SET_SCAN_ENABLE, bytes([1, 0]))
        peer = None
        deadline = time.monotonic() + 15
        try:
            while peer is None and time.monotonic() < deadline:
                packet = hci.read_packet(0.2)
                if packet is None:
                    continue
                kind, code, body = packet
                if kind != api.H4_EVENT or code != api.EVT_LE_META or len(body) < 2 or body[0] != 2:
                    continue
                offset = 2
                for _ in range(body[1]):
                    if offset + 9 > len(body):
                        break
                    length = body[offset + 8]
                    if offset + 10 + length > len(body):
                        break
                    data = body[offset + 9:offset + 9 + length]
                    name = api.local_name(data)
                    if name in names:
                        print('Found %s' % name)
                        peer_type, peer = body[offset + 1], bytes(body[offset + 2:offset + 8])
                        break
                    offset += 10 + length
        finally:
            hci.command(api.OP_LE_SET_SCAN_ENABLE, bytes([0, 0]), allow_fail=True)
        if peer is None:
            raise RuntimeError('%s not found' % ', '.join(names))
    print('Connecting to %s; requesting %s, %.2f ms' % (api.addr_str(peer), args.phy, args.interval))
    interval = int(args.interval / 1.25)
    params = struct.pack('<HHBB', 96, 48, 0, peer_type) + peer + bytes([own_type])
    params += struct.pack('<HHHHHH', interval, interval, 0, 400, 0, 0)
    hci.command(api.OP_LE_CREATE_CONNECTION, params)
    early = []
    deadline = time.monotonic() + 10
    while time.monotonic() < deadline:
        packet = hci.read_packet(0.1)
        if packet is None:
            continue
        kind, code, body = packet
        info = api.parse_connection(body) if kind == api.H4_EVENT and code == api.EVT_LE_META else None
        if info is not None:
            status, handle, _, _, units, latency, timeout = info
            if status:
                raise RuntimeError('connection failed: 0x%02X' % status)
            print('Connection interval %.2f ms, latency %d, timeout %d ms' % (units * 1.25, latency, timeout * 10))
            hci.pending[0:0] = early
            return handle, units * 1.25
        early.append(packet)
    hci.command(api.OP_LE_CREATE_CONNECTION_CANCEL, allow_fail=True)
    raise RuntimeError('connection timed out')


def configure_phy(link, phy):
    number = 1 if phy == '1M' else 2
    link.hci.command(0x2032, struct.pack('<HBBBH', link.handle, 0, number, number, 0))
    deadline = time.monotonic() + 5
    while time.monotonic() < deadline:
        link.pump(0.1)
        if link.down is not None:
            raise RuntimeError('disconnected during PHY negotiation')
        status, data = link.hci.command(0x2030, struct.pack('<H', link.handle), allow_fail=True)
        if status == 0 and len(data) == 4 and tuple(data[2:]) == (number, number):
            link.required_phy = number
            print('Verified PHY: %s in both directions' % phy)
            return
    raise RuntimeError('%s PHY was not negotiated; refusing a mislabeled result' % phy)


class Receiver:
    def __init__(self, checker, clock=time.monotonic):
        self.check = checker
        self.clock = clock
        self.handle = None
        self.first = None
        self.last = None
        self.report_time = None
        self.report_bytes = 0
        self.notifications = 0
        self.other = 0

    def notify(self, handle, data):
        if handle != self.handle:
            self.other += 1
            return
        if not data:
            return
        now = self.clock()
        if self.first is None:
            self.first = now
            self.report_time = now
        self.last = now
        self.notifications += 1
        self.check.feed(data)

    def failed(self):
        return (not self.check.had_sync or self.check.drop != 0 or
                self.check.corrupt != 0 or self.check.out_of_frame != 0 or
                self.check.restart != 0 or self.check.uncertain != 0)

    def report(self, queued=0, final=False):
        now = self.clock()
        elapsed = now - self.first if self.first is not None else 0.0
        window = now - self.report_time if self.report_time is not None else 0.0
        rate = (self.check.count - self.report_bytes) / window if window > 0 else 0.0
        average = self.check.count / elapsed if elapsed > 0 else 0.0
        print('%sBytes/sec: %.2f, average: %.2f (%.2f kbit/s), bytes: %d, '
              'notifications: %d, drop: %d, corrupt: %d, lost_est: %d, '
              'out_of_frame: %d, restart: %d, uncertain: %d, queued: %d, other: %d'
              % ('Final ' if final else '', rate, average, average * 8 / 1000,
                 self.check.count, self.notifications, self.check.drop,
                 self.check.corrupt, self.check.lost_bytes,
                 self.check.out_of_frame, self.check.restart,
                 self.check.uncertain, queued, self.other), flush=True)
        self.report_time = now
        self.report_bytes = self.check.count


def positive(value):
    number = float(value)
    if not math.isfinite(number) or number <= 0:
        raise argparse.ArgumentTypeError('must be a finite positive number')
    return number


def parse_args(argv=None):
    parser = argparse.ArgumentParser(description='BLE PRBS receiver through HciController')
    parser.add_argument('--hci', nargs='?', const='usb', default='usb',
                        help='usb (default), auto, serial, or a serial port')
    parser.add_argument('--name', help='exact advertised name (default: any BlePrbs variant)')
    parser.add_argument('--address', help='BLE address, skips scanning')
    parser.add_argument('--address-type', choices=('random', 'public'), default='random')
    parser.add_argument('--phy', choices=('1M', '2M', 'both'), default='both')
    parser.add_argument('--interval', type=positive, default=15.0,
                        help='requested connection interval in ms (default 15)')
    parser.add_argument('--duration', type=positive, default=60.0,
                        help='seconds per PHY after subscription (default 60)')
    parser.add_argument('--report-interval', type=positive, default=1.0)
    parser.add_argument('--idle-timeout', type=positive, default=10.0)
    parser.add_argument('--trace', action='store_true', help='HCI packet trace; affects performance')
    args = parser.parse_args(argv)
    units = args.interval / 1.25
    if not 6 <= units <= 3200 or not units.is_integer():
        parser.error('--interval must be 7.5..4000 ms in multiples of 1.25 ms')
    # The existing helper requests a 4-second supervision timeout.
    if args.interval >= 2000:
        parser.error('--interval must be below 2000 ms with the 4-second supervision timeout')
    if args.address:
        try:
            args.address = bytes.fromhex(args.address.replace(':', '').replace('-', ''))[::-1]
            if len(args.address) != 6:
                raise ValueError()
        except ValueError:
            parser.error('--address must contain six hexadecimal bytes')
    return args


def run(args):
    from uartprbs_rx import PrbsCheck
    hb = hci_library()
    from hcicontroller import TransportSpec, discover

    spec = (discover(kind=args.hci) if args.hci in ('auto', 'usb', 'serial')
            else TransportSpec('serial', args.hci))
    if spec is None:
        raise RuntimeError('no HciController found')
    receiver = Receiver(PrbsCheck())
    hci = hb.Hci(spec, raw=args.trace)
    link = None
    subscribed = False
    failure = False
    interrupted = False
    print('HciController on %s' % spec)
    try:
        handle, interval = connect(hci, hb, args)
        link = PrbsLink(hci, handle, receiver, hb)
        link.interval = interval
        configure_phy(link, args.phy)
        hci.command(0x2022, struct.pack('<HHH', handle, 251, 2120))
        link.subscribe()
        subscribed = True
        start = time.monotonic()
        report = start
        print('Receiving PRBS on handle 0x%04X; Ctrl-C stops' % receiver.handle)
        while True:
            link.pump(0.05)
            now = time.monotonic()
            if link.down is not None:
                raise RuntimeError('link lost, reason 0x%02X' % link.down)
            if now - (receiver.last if receiver.last is not None else start) >= args.idle_timeout:
                raise RuntimeError('no PRBS data for %.1f seconds' % args.idle_timeout)
            if now - report >= args.report_interval:
                queued = len(hci.pending) + len(getattr(hci.transport, '_reader_packets', ()))
                receiver.report(queued)
                report = now
            if args.duration is not None and now - start >= args.duration:
                break
    except KeyboardInterrupt:
        interrupted = True
        failure = not subscribed
    except Exception as error:
        print('ERROR: %s' % error, file=sys.stderr)
        failure = True
    finally:
        receiver.check.finish()
        receiver.report(final=True)
        result = failure or receiver.failed()
        print('%s RESULT: %s' % (args.phy, ('FAIL' if result else 'PASS (received PRBS sequence)')))
        try:
            if link is not None:
                receiver.handle = None
                link.disconnect()
        except Exception as error:
            print('Disconnect: %s' % error, file=sys.stderr)
        hci.close()
    return 130 if interrupted else int(result)


def main():
    args = parse_args()
    result = 0
    for phy in (('1M', '2M') if args.phy == 'both' else (args.phy,)):
        trial = copy.copy(args)
        trial.phy = phy
        print('\n=== %s PRBS test ===' % phy, flush=True)
        try:
            status = run(trial)
        except KeyboardInterrupt:
            return 130
        except Exception as error:
            print('%s ERROR: %s' % (phy, error), file=sys.stderr)
            status = 1
        if status == 130:
            return status
        result |= status
    return result


if __name__ == '__main__':
    sys.exit(main())
