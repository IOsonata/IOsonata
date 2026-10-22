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
           'src/device.cpp', 'src/device_intrf.cpp', 'src/crypto/crypto_uecc.cpp']
UECC = Path(os.environ.get('UECC_ROOT', ROOT / 'micro-ecc'))
ENV = dict(os.environ, ASAN_OPTIONS='detect_leaks=0:abort_on_error=1')


def interop(binary, chunk, bad=False, dhe=False, connections=2):
    with socket.socket() as probe:
        probe.bind(('127.0.0.1', 0))
        port = probe.getsockname()[1]
    command = ['openssl', 's_server', '-accept', f'127.0.0.1:{port}',
               '-nocert', '-tls1_3', '-psk',
               bytes(range(32)).hex(), '-psk_identity', 'IOsonata',
               '-ciphersuites', 'TLS_AES_128_GCM_SHA256', '-rev', '-naccept', str(connections)]
    command += ['-groups', 'P-256'] if dhe else ['-allow_no_dhe_kex']
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
            subprocess.run([str(binary), str(port), str(chunk), 'dhe' if dhe else 'psk'] +
                           ['bad-key' if bad else 'good-key', str(connections)],
                           check=True, env=ENV, timeout=max(40, connections * 10))
        finally:
            server.terminate()
            server.wait(timeout=5)


def main():
    with tempfile.TemporaryDirectory(prefix='iosonata-tls-') as temp:
        for optimization in ['-O0', '-O2']:
            ecc = Path(temp) / 'uecc.o'
            subprocess.run(['gcc', optimization, '-g', '-fsanitize=address,undefined',
                            '-fno-pie', '-DuECC_ENABLE_VLI_API=1', '-I' + str(UECC), '-c', str(UECC / 'uECC.c'),
                            '-o', str(ecc)], check=True)
            flags = ['-std=gnu++17', '-DuECC_ENABLE_VLI_API=1', optimization, '-g', '-Wall', '-Wextra',
                     '-fno-exceptions', '-fno-rtti', '-ffunction-sections',
                     '-fdata-sections', '-fsanitize=address,undefined',
                     '-fno-omit-frame-pointer', '-fno-pie', '-no-pie',
                     '-I' + str(ROOT / 'include'), '-I' + str(UECC)]
            for name in ['tls_test', 'tls_host']:
                binary = Path(temp) / name
                subprocess.run(['g++', *flags, str(ROOT / f'tests/net/{name}.cpp'),
                                *[str(ROOT / p) for p in SOURCES],
                                str(ecc), '-Wl,--gc-sections', '-o', str(binary)], check=True)
                if name == 'tls_test':
                    subprocess.run([str(binary)], check=True, env=ENV)
                else:
                    for chunk, bad in [(65536, False), (1, False), (7, True), (7, False)]:
                        # Each successful process reuses its TLS context for two connections.
                        interop(binary, chunk, bad)
                        interop(binary, chunk, bad, True)
                    if optimization == '-O2':
                        # Keep one OpenSSL process alive across repeated DHE sessions.
                        interop(binary, 65536, dhe=True, connections=20)
                        print('20 PSK-DHE reconnects to one OpenSSL process: PASS', flush=True)
            print(f'{optimization}: TLS host tests PASS', flush=True)
        subprocess.run(['gcc', '-std=c11', '-Werror', '-fsyntax-only',
                        '-I' + str(ROOT / 'include'), '-x', 'c', '-'],
                       input='#include "net/tls.h"\nTlsDev_t dev;\n', text=True, check=True)
    print('TLS C header and O0/O2 ASan/UBSan checks PASS')


if __name__ == '__main__':
    main()
