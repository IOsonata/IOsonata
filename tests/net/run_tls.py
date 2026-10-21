#!/usr/bin/env python3
# SPDX-License-Identifier: MIT
# Copyright (c) 2026 I-SYST inc.
"""Native TLS tests: real IOsonata crypto, malformed peers, OpenSSL interoperability.
Requires Linux, g++, OpenSSL 3 with s_server -allow_no_dhe_kex. No network downloads.
All network traffic is localhost. No ARM toolchain is required for these host tests.
"""
import os
from pathlib import Path
import select
import socket
import subprocess
import tempfile

ROOT = Path(__file__).resolve().parents[2]
SOURCES = ['src/net/tls.cpp', 'src/crypto/crypto_softaes.cpp',
           'src/crypto/crypto_softsha256.cpp', 'src/crypto/crypto_p256.cpp',
           'src/device.cpp', 'src/device_intrf.cpp']
ENV = dict(os.environ, ASAN_OPTIONS='detect_leaks=0:abort_on_error=1')


def interop(binary, chunk, bad=False):
    with socket.socket() as probe:
        probe.bind(('127.0.0.1', 0))
        port = probe.getsockname()[1]
    command = ['openssl', 's_server', '-accept', f'127.0.0.1:{port}',
               '-nocert', '-tls1_3', '-allow_no_dhe_kex', '-psk',
               bytes(range(32)).hex(), '-psk_identity', 'IOsonata',
               '-ciphersuites', 'TLS_AES_128_GCM_SHA256', '-rev', '-naccept', '2']
    with tempfile.TemporaryFile() as errors:
        server = subprocess.Popen(command, stdout=subprocess.PIPE, stderr=errors)
        try:
            # Wait for the server's ACCEPT line without consuming its one connection.
            if not select.select([server.stdout], [], [], 5)[0]:
                raise RuntimeError('OpenSSL did not start')
            while True:
                line = server.stdout.readline()
                if b'ACCEPT' in line:
                    break
                if not line:
                    errors.seek(0)
                    raise RuntimeError(errors.read().decode())
            subprocess.run([str(binary), str(port), str(chunk)] +
                           (['bad-key'] if bad else []), check=True, env=ENV, timeout=40)
        finally:
            server.terminate()
            server.wait(timeout=5)


def main():
    with tempfile.TemporaryDirectory(prefix='iosonata-tls-') as temp:
        for optimization in ['-O0', '-O2']:
            flags = ['-std=gnu++17', optimization, '-g', '-Wall', '-Wextra',
                     '-fno-exceptions', '-fno-rtti', '-ffunction-sections',
                     '-fdata-sections', '-fsanitize=address,undefined',
                     '-fno-omit-frame-pointer', '-fno-pie', '-no-pie',
                     '-I' + str(ROOT / 'include')]
            for name in ['tls_test', 'tls_host']:
                binary = Path(temp) / name
                subprocess.run(['g++', *flags, str(ROOT / f'tests/net/{name}.cpp'),
                                *[str(ROOT / p) for p in SOURCES],
                                '-Wl,--gc-sections', '-o', str(binary)], check=True)
                if name == 'tls_test':
                    subprocess.run([str(binary)], check=True, env=ENV)
                else:
                    for chunk, bad in [(65536, False), (1, False), (7, True), (7, False)]:
                        # Each successful process reuses its TLS context for two connections.
                        interop(binary, chunk, bad)
            print(f'{optimization}: TLS host tests PASS', flush=True)
        subprocess.run(['gcc', '-std=c11', '-Werror', '-fsyntax-only',
                        '-I' + str(ROOT / 'include'), '-x', 'c', '-'],
                       input='#include "net/tls.h"\nTlsDev_t dev;\n', text=True, check=True)
    print('TLS C header and O0/O2 ASan/UBSan checks PASS')


if __name__ == '__main__':
    main()
