#!/usr/bin/env python3
# SPDX-License-Identifier: MIT
# Copyright (c) 2026 I-SYST inc.
"""CoAP O0/O2 sanitizer tests and optional independent aiocoap interoperability.

Requires Linux, gcc and g++. Install aiocoap separately to run interoperability;
this runner never downloads dependencies. All test sockets bind to localhost.
Use --require-interop to make a missing aiocoap installation an error.
"""
import argparse
import importlib.util
import os
from pathlib import Path
import select
import socket
import subprocess
import sys
import tempfile
import threading

ROOT = Path(__file__).resolve().parents[2]
ENV = dict(os.environ, ASAN_OPTIONS='detect_leaks=0:abort_on_error=1',
           UBSAN_OPTIONS='halt_on_error=1:print_stacktrace=1')
SOURCES = ['src/net/coap.cpp', 'src/device_intrf.cpp', 'src/device.cpp']


class LossProxy:
    """Drop one piggyback response and one ACK to a separate response."""

    def __init__(self, server_port):
        self.server = ('127.0.0.1', server_port)
        self.client = None
        self.socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.socket.bind(('127.0.0.1', 0))
        self.port = self.socket.getsockname()[1]
        self.stopped = threading.Event()
        self.thread = threading.Thread(target=self.run, daemon=True)
        self.dropped_response = False
        self.dropped_ack = False
        self.confirmable_responses = 0
        self.error = None

    def run(self):
        try:
            while not self.stopped.is_set():
                if not select.select([self.socket], [], [], 0.05)[0]:
                    continue
                packet, source = self.socket.recvfrom(65535)
                kind = (packet[0] >> 4) & 3 if len(packet) >= 4 else -1
                code = packet[1] if len(packet) >= 4 else -1
                if source == self.server:
                    if kind == 0 and code >= 64:
                        self.confirmable_responses += 1
                    if kind == 2 and code >= 64 and not self.dropped_response:
                        self.dropped_response = True
                        continue
                    if self.client is not None:
                        self.socket.sendto(packet, self.client)
                else:
                    self.client = source
                    if kind == 2 and code == 0 and not self.dropped_ack:
                        self.dropped_ack = True
                        continue
                    self.socket.sendto(packet, self.server)
        except Exception as error:
            self.error = error

    def close(self):
        self.stopped.set()
        self.thread.join(timeout=2)
        self.socket.close()
        if self.error is not None:
            raise self.error
        if self.thread.is_alive():
            raise RuntimeError('UDP loss proxy did not stop')


def interop(binary):
    with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as probe:
        probe.bind(('127.0.0.1', 0))
        port = probe.getsockname()[1]
    command = [sys.executable, '-u', str(ROOT / 'Python/coap_server.py'),
               '--host', '127.0.0.1', '--port', str(port), '--delay', '0.3']
    with tempfile.TemporaryFile() as errors:
        server = subprocess.Popen(command, stdout=subprocess.PIPE, stderr=errors, env=ENV)
        try:
            if not select.select([server.stdout], [], [], 5)[0]:
                raise RuntimeError('aiocoap server did not start')
            line = server.stdout.readline()
            if b'CoAP server ready:' not in line:
                raise RuntimeError('Unexpected aiocoap startup: ' + repr(line))
            subprocess.run([str(binary), str(port)], check=True, env=ENV, timeout=30)
            proxy = LossProxy(port)
            proxy.thread.start()
            try:
                subprocess.run([str(binary), str(proxy.port), 'loss'],
                               check=True, env=ENV, timeout=40)
            finally:
                proxy.close()
            if not (proxy.dropped_response and proxy.dropped_ack and
                    proxy.confirmable_responses >= 2):
                raise RuntimeError('Loss test did not exercise both retransmissions')
            print('aiocoap UDP: direct and lost-response/lost-ACK sessions PASS', flush=True)
        except Exception:
            errors.seek(0)
            print(errors.read().decode(errors='replace'), flush=True)
            raise
        finally:
            server.terminate()
            try:
                server.wait(timeout=5)
            except subprocess.TimeoutExpired:
                server.kill()
                server.wait(timeout=5)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--require-interop', action='store_true')
    args = parser.parse_args()
    available = importlib.util.find_spec('aiocoap') is not None
    if args.require_interop and not available:
        parser.error('aiocoap is required; install it separately in the active Python environment')
    with tempfile.TemporaryDirectory(prefix='iosonata-coap-') as tmp:
        for opt in ('-O0', '-O2'):
            flags = ['-std=gnu++17', opt, '-g', '-Wall', '-Wextra', '-Werror',
                     '-fno-exceptions', '-fno-rtti', '-fsanitize=address,undefined',
                     '-fno-omit-frame-pointer', '-fno-pie', '-no-pie',
                     '-ffunction-sections', '-fdata-sections', '-I' + str(ROOT / 'include')]
            binary = Path(tmp) / 'coap_test'
            subprocess.run(['g++', *flags, str(ROOT / 'tests/net/coap_test.cpp'),
                            *[str(ROOT / p) for p in SOURCES],
                            '-Wl,--gc-sections', '-o', str(binary)], check=True)
            subprocess.run([str(binary)], check=True, env=ENV, timeout=30)
            print(opt + ': CoAP wire and exchange tests PASS', flush=True)
        subprocess.run(['gcc', '-std=c11', '-Wall', '-Wextra', '-Werror', '-fsyntax-only',
                        '-I' + str(ROOT / 'include'), '-x', 'c', '-'],
                       input='#include "net/coap.h"\nCoapDev_t dev;\n', text=True, check=True)
        print('CoAP C header: PASS', flush=True)
        if available:
            binary = Path(tmp) / 'coap_host'
            subprocess.run(['g++', *flags, str(ROOT / 'tests/net/coap_host.cpp'),
                            *[str(ROOT / p) for p in SOURCES],
                            '-Wl,--gc-sections', '-o', str(binary)], check=True)
            interop(binary)
        else:
            print('aiocoap interoperability: SKIP (install aiocoap separately)', flush=True)


if __name__ == '__main__':
    main()
