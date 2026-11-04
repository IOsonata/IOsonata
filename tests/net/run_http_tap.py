#!/usr/bin/env python3
# MIT License
#
# Copyright (c) 2026, I-SYST inc., all rights reserved
#
# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:
#
# The above copyright notice and this permission notice shall be included in all
# copies or substantial portions of the Software.
#
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
# SOFTWARE.

"""Real HTTP clients against exemples/net/http_server.cpp on a TAP interface.

Linux, as root, with /dev/net/tun. LWIP_ROOT selects the lwIP checkout. Runs the
example with the real server, IP service and lwIP under ASan and UBSan. The timing
shown is of a computer and says nothing of an MCU. This is not a hardware test.
"""
import concurrent.futures as cf
import hashlib
import http.client
import json
import os
from pathlib import Path
import shutil
import signal
import socket
import struct
import subprocess
import tempfile
import time

ROOT = Path(__file__).resolve().parents[2]
LWIP = Path(os.environ.get('LWIP_ROOT', ROOT.parent / 'external/lwip'))
HOST = '10.99.0.2'
BIG_SIZE = 300000
BIG = bytes(((i * 2654435761) & 0xFFFFFFFF) >> 24 for i in range(BIG_SIZE))


def build(out):
    sources = sorted((LWIP / 'src/core').glob('*.c')) + sorted((LWIP / 'src/core/ipv4').glob('*.c'))
    sources += [LWIP / 'src/netif/ethernet.c']
    sources += [ROOT / s for s in ('src/device_intrf.cpp', 'src/device.cpp', 'src/net/ip_lwip.cpp', 'src/net/http_server.cpp',
                                   'src/app_evt_handler.cpp', 'src/app_run.cpp', 'src/syslog.cpp', 'src/cfifo.c',
                                   'tests/net/http_tap_host.cpp')]
    inc = ['-I' + str(ROOT / p) for p in ('tests/net/http_stubs', 'tests/net/hostport', 'include', 'include/net/lwip')]
    inc += ['-I' + str(LWIP / 'src/include')]
    san = ['-g', '-O1', '-fsanitize=address,undefined', '-fno-sanitize-recover=all']

    def compile_one(job):
        i, source = job
        obj = out / (str(i) + '.o')
        cpp = source.suffix == '.cpp'
        lang = ['-std=gnu++23', '-fno-exceptions', '-fno-rtti'] if cpp else ['-std=gnu17']
        p = subprocess.run(['g++' if cpp else 'gcc', *lang, *san, *inc, '-c', str(source), '-o', str(obj)],
                           text=True, stdout=subprocess.PIPE, stderr=subprocess.STDOUT)
        if p.returncode:
            raise SystemExit(str(source) + '\n' + p.stdout)
        return obj

    with cf.ThreadPoolExecutor(8) as ex:
        objects = list(ex.map(compile_one, enumerate(sources)))
    exe = out / 'http_tap_host'
    p = subprocess.run(['g++', *san, *map(str, objects), '-o', str(exe)], text=True,
                       stdout=subprocess.PIPE, stderr=subprocess.STDOUT)
    if p.returncode:
        raise SystemExit(p.stdout)
    return exe


def request(path, port=80, method='GET', body=None, timeout=10):
    conn = http.client.HTTPConnection(HOST, port, timeout=timeout)
    try:
        conn.request(method, path, body=body)
        rsp = conn.getresponse()
        return rsp.status, dict(rsp.getheaders()), rsp.read()
    finally:
        conn.close()


def raw(data, port=80, timeout=5, shut=False):
    s = socket.create_connection((HOST, port), timeout=timeout)
    try:
        s.sendall(data)
        if shut:
            s.shutdown(socket.SHUT_WR)
        out = b''
        while True:
            d = s.recv(65536)
            if not d:
                return out
            out += d
    finally:
        s.close()


def status_of(rsp):
    return int(rsp.split(b' ', 2)[1]) if rsp.startswith(b'HTTP/1.1 ') else None


def curl(*args):
    p = subprocess.run(['curl', '-sS', '--noproxy', '*', '--max-time', '20', *args], capture_output=True)
    assert p.returncode == 0, (args, p.stderr)
    return p.stdout


def test_example():
    status, head, body = request('/')
    assert status == 200 and head['Content-Type'] == 'text/html' and body.startswith(b'<!DOCTYPE html>')
    assert b'<title>IOsonata</title>' in body and int(head['Content-Length']) == len(body)
    assert head['Connection'] == 'close'
    status, head, body = request('/status')
    info = json.loads(body)
    assert status == 200 and head['Content-Type'] == 'application/json' and head['Cache-Control'] == 'no-store'
    assert info['address'] == HOST and info['link_mbps'] == 100 and info['uptime_ms'] > 0
    status, head, body = request('/hello?name=H%C3%B4+an')
    assert status == 200 and body.decode() == 'Hello H\u00f4 an\n'
    assert request('/hello')[2] == b'Hello world\n'
    assert request('/hello?name=' + 'x' * 40)[2] == b'Hello world\n'
    status, head, body = request('/led', method='POST')
    assert status == 204 and body == b'' and 'Content-Length' not in head
    status, head, _ = request('/led')
    assert status == 405 and head['Allow'] == 'POST'
    assert request('/', method='HEAD')[2] == b''
    assert request('/missing')[0] == 404
    assert b'Hello curl' in curl('http://' + HOST + '/hello?name=curl')
    assert curl('-o', '/dev/null', '-w', '%{http_code}', '-X', 'POST', 'http://' + HOST + '/led') == b'204'
    print('example routes, http.client and curl: PASS', flush=True)


def test_big():
    start = time.monotonic()
    status, head, body = request('/big', 8081)
    took = time.monotonic() - start
    assert status == 200 and int(head['Content-Length']) == BIG_SIZE
    assert hashlib.sha256(body).digest() == hashlib.sha256(BIG).digest()
    data = curl('http://' + HOST + ':8081/big')
    assert hashlib.sha256(data).digest() == hashlib.sha256(BIG).digest()
    print(f'{BIG_SIZE} byte page twice, same bytes, {BIG_SIZE / 1024 / took:.0f} KiB/s here: PASS', flush=True)


def test_post():
    for size in (0, 1, 100, 500, 900):
        body = bytes(i & 0xFF for i in range(size))
        status, _, rsp = request('/echo', 8081, 'POST', body)
        assert status == 200 and rsp == body, size
    # Refused while the client is still sending: the response must arrive, not a reset
    for size in (1000, 5000, 100000):
        status, _, rsp = request('/echo', 8081, 'POST', b'z' * size)
        assert status == 413 and rsp == b'Content Too Large', size
    p = subprocess.run(['curl', '-sS', '--noproxy', '*', '--max-time', '20', '-o', '/dev/null', '-w', '%{http_code}',
                        '--data-binary', '@-', 'http://' + HOST + ':8081/echo'], input=b'q' * 200000,
                       capture_output=True)
    assert p.stdout == b'413', p
    print('POST echo up to 900 bytes, 413 reaches a client still sending: PASS', flush=True)


def test_parallel():
    def one(i):
        status, _, body = request('/hello?name=' + str(i), timeout=30)
        assert status == 200 and body == b'Hello ' + str(i).encode() + b'\n'
        return i

    start = time.monotonic()
    with cf.ThreadPoolExecutor(8) as ex:
        done = list(ex.map(one, range(200)))
    assert len(done) == 200
    print(f'200 requests from 8 clients on 2 connections, {time.monotonic() - start:.1f} s: PASS', flush=True)


def test_churn():
    start = time.monotonic()
    for i in range(400):
        assert request('/hello?name=x')[0] == 200, i
    print(f'400 connections in turn, {time.monotonic() - start:.1f} s: PASS', flush=True)


def test_slow():
    # Nothing sent: 408 at the limit of the test server, 1 s
    start = time.monotonic()
    s = socket.create_connection((HOST, 8081), timeout=5)
    rsp = b''
    while True:
        d = s.recv(4096)
        if not d:
            break
        rsp += d
    s.close()
    took = time.monotonic() - start
    assert status_of(rsp) == 408 and 0.9 < took < 2, (rsp, took)

    # A byte now and then does not extend it
    start = time.monotonic()
    s = socket.create_connection((HOST, 8081), timeout=5)
    rsp = b''
    try:
        for c in b'GET / HTTP/1.1\r\nHost':
            s.sendall(bytes([c]))
            time.sleep(0.2)
    except OSError:
        pass
    try:
        while True:
            d = s.recv(4096)
            if not d:
                break
            rsp += d
    except OSError:
        pass
    s.close()
    took = time.monotonic() - start
    assert status_of(rsp) == 408, rsp
    print('no request and a slow request: 408 at the time limit: PASS', flush=True)


def test_stalled_reader():
    # A client not reading a large page holds one connection. The other keeps serving,
    # and the first is dropped at the time limit.
    s = socket.socket()
    s.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, 4096)
    s.settimeout(10)
    s.connect((HOST, 8081))
    s.sendall(b'GET /big HTTP/1.1\r\nHost: t\r\n\r\n')
    time.sleep(0.3)
    worst = 0
    for i in range(20):
        start = time.monotonic()
        status, _, body = request('/echo', 8081, 'POST', b'still here')
        worst = max(worst, time.monotonic() - start)
        assert status == 200 and body == b'still here'
    time.sleep(1.5)
    got = 0
    try:
        while True:
            d = s.recv(65536)
            if not d:
                break
            got += len(d)
    except OSError:
        pass
    s.close()
    assert got < BIG_SIZE, got
    print(f'client not reading: others served, worst {worst * 1000:.0f} ms, it got {got} bytes then dropped: PASS', flush=True)


def test_reset():
    # A client resetting in the middle of a page
    for i in range(10):
        s = socket.create_connection((HOST, 8081), timeout=5)
        s.sendall(b'GET /big HTTP/1.1\r\nHost: t\r\n\r\n')
        s.recv(1000)
        s.setsockopt(socket.SOL_SOCKET, socket.SO_LINGER, struct.pack('ii', 1, 0))
        s.close()
    assert request('/big', 8081)[0] == 200
    print('clients resetting during a page: PASS', flush=True)


def test_junk():
    cases = [
        (b'\x16\x03\x01\x02\x00\x01\x00\x01\xfc\x03\x03' + bytes(500), None),
        (b'PRI * HTTP/2.0\r\n\r\nSM\r\n\r\n', 505),
        (b'GET / HTTP/1.1\r\n\r\n', 400),
        (b'GET / HTTP/9.9\r\nHost: t\r\n\r\n', 505),
        (b'OPTIONS / HTTP/1.1\r\nHost: t\r\n\r\n', 501),
        (b'GET /' + b'a' * 3000 + b' HTTP/1.1\r\nHost: t\r\n\r\n', 431),
        (b'POST /echo HTTP/1.1\r\nHost: t\r\nTransfer-Encoding: chunked\r\n\r\n', 501),
        (bytes(range(256)) * 8, None),
    ]
    for data, want in cases:
        try:
            rsp = raw(data, 8081)
        except ConnectionResetError:
            rsp = b''
        if want is not None:
            assert status_of(rsp) == want, (data[:40], rsp[:60])
        else:
            assert rsp == b'' or status_of(rsp) is not None, (data[:40], rsp[:60])
    assert request('/')[0] == 200
    print('bad requests and bytes that are not HTTP: PASS', flush=True)


def test_half_close():
    # The documented limit: no response once the client has closed its sending side.
    rsp = raw(b'GET / HTTP/1.1\r\nHost: t\r\n\r\n', 80, shut=True)
    assert request('/')[0] == 200
    print(f'client closing its sending side: {len(rsp)} bytes back, server fine: PASS', flush=True)


def main():
    if not (LWIP / 'src/include/lwip/init.h').is_file():
        raise SystemExit('Set LWIP_ROOT to the SDK lwIP checkout (tested with 2.2.0).')
    if os.geteuid() != 0 or not os.path.exists('/dev/net/tun') or shutil.which('curl') is None:
        print('SKIP: needs root, /dev/net/tun and curl')
        return
    with tempfile.TemporaryDirectory(prefix='iosonata-http-tap-') as td:
        exe = build(Path(td))
        log = open(Path(td) / 'log', 'w+')
        env = dict(os.environ, ASAN_OPTIONS='detect_leaks=0')
        proc = subprocess.Popen([str(exe)], stdout=log, stderr=subprocess.STDOUT, env=env)
        ok = False
        try:
            for i in range(100):
                try:
                    if request('/', timeout=1)[0] == 200:
                        ok = True
                        break
                except OSError:
                    time.sleep(0.1)
            assert ok, 'no answer from the server'
            test_example()
            test_big()
            test_post()
            test_parallel()
            test_churn()
            test_slow()
            test_stalled_reader()
            test_reset()
            test_junk()
            test_half_close()
            assert proc.poll() is None, 'server stopped'
        finally:
            proc.send_signal(signal.SIGTERM)
            try:
                proc.wait(5)
            except subprocess.TimeoutExpired:
                proc.kill()
            log.seek(0)
            text = log.read()
            print(text[-3000:] if (not ok or 'ERROR' in text or 'runtime error' in text) else '', end='')
        assert proc.returncode == 0 and 'ERROR' not in text and 'runtime error' not in text, proc.returncode
        assert 'Open http://10.99.0.2/' in text, text
        assert 'LED toggles 2' in text, text
        print('PASS')


if __name__ == '__main__':
    main()
