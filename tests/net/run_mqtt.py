#!/usr/bin/env python3
# SPDX-License-Identifier: MIT
# Copyright (c) 2026 I-SYST inc.
"""MQTT O0/O2 sanitizer tests. Optional Mosquitto TCP/TLS interoperability.
Linux host; set MOSQUITTO and UECC_ROOT if they are not in the default locations.
No network downloads. The optional broker listens only on localhost.
"""
import os
from pathlib import Path
import pwd
import select
import shutil
import socket
import subprocess
import tempfile
import time

ROOT = Path(__file__).resolve().parents[2]
ENV = dict(os.environ, ASAN_OPTIONS='detect_leaks=0:abort_on_error=1')


def interop(broker, binary, directory, secure):
    with socket.socket() as probe:
        probe.bind(('127.0.0.1', 0))
        port = probe.getsockname()[1]
    cfg = directory / 'broker.conf'
    config = (f'user {pwd.getpwuid(os.geteuid()).pw_name}\nlistener {port} 127.0.0.1\n'
              'allow_anonymous true\npersistence false\n')
    cfg.write_text(config)
    with tempfile.TemporaryFile() as log:
        server = subprocess.Popen([broker, '-c', str(cfg)], stdout=log, stderr=log, env=ENV)
        try:
            for _ in range(100):
                try:
                    with socket.create_connection(('127.0.0.1', port), timeout=0.1):
                        break
                except OSError:
                    time.sleep(0.02)
            else:
                raise RuntimeError('Mosquitto failed to listen')
            if secure:
                tls_interop(binary, port)
            else:
                subprocess.run([str(binary), str(port)], check=True, env=ENV, timeout=40)
        except Exception:
            log.seek(0)
            print(log.read().decode(), flush=True)
            raise
        finally:
            server.terminate()
            server.wait(timeout=5)


def tls_interop(binary, broker_port):
    # Mosquitto 2.0 PSK listeners cap TLS at 1.2. Terminate TLS 1.3 in OpenSSL
    # and forward its application stream to the real localhost MQTT broker.
    with socket.socket() as probe:
        probe.bind(('127.0.0.1', 0))
        port = probe.getsockname()[1]
    command = ['openssl', 's_server', '-quiet', '-ign_eof', '-naccept', '1',
               '-accept', f'127.0.0.1:{port}', '-nocert', '-tls1_3', '-groups', 'P-256',
               '-ciphersuites', 'TLS_AES_128_GCM_SHA256', '-psk_identity', 'IOsonata',
               '-psk', bytes(range(32)).hex()]
    with socket.create_connection(('127.0.0.1', broker_port)) as peer, tempfile.TemporaryFile() as log:
        server = subprocess.Popen(command, stdin=subprocess.PIPE, stdout=subprocess.PIPE, stderr=log)
        client = subprocess.Popen([str(binary), str(port), 'tls'], env=ENV)
        streams = [peer, server.stdout]
        deadline = time.monotonic() + 35
        try:
            while client.poll() is None and time.monotonic() < deadline:
                ready, _, _ = select.select(streams, [], [], 0.1)
                for stream in ready:
                    data = os.read(stream.fileno(), 4096)
                    if not data:
                        streams.remove(stream)
                    elif stream is peer:
                        server.stdin.write(data)
                        server.stdin.flush()
                    else:
                        peer.sendall(data)
            if client.poll() != 0:
                log.seek(0)
                print(log.read().decode(), flush=True)
                raise RuntimeError('MQTT over native TLS failed or timed out')
        finally:
            if client.poll() is None:
                client.kill()
            client.wait(timeout=5)
            server.terminate()
            server.wait(timeout=5)
            server.stdin.close()
            server.stdout.close()


def main():
    with tempfile.TemporaryDirectory(prefix='iosonata-mqtt-') as tmp:
        for opt in ('-O0', '-O2'):
            flags = ['-std=gnu++17', opt, '-g', '-Wall', '-Wextra', '-Werror',
                     '-fno-exceptions', '-fno-rtti', '-fsanitize=address,undefined',
                     '-fno-omit-frame-pointer', '-fno-pie', '-no-pie',
                     '-ffunction-sections', '-fdata-sections', '-I' + str(ROOT / 'include')]
            sources = [str(ROOT / s) for s in ('src/net/mqtt.cpp', 'src/device_intrf.cpp')]
            binary = Path(tmp) / 'mqtt_test'
            subprocess.run(['g++', *flags, str(ROOT / 'tests/net/mqtt_test.cpp'),
                            *sources, '-Wl,--gc-sections', '-o', str(binary)], check=True)
            subprocess.run([str(binary)], check=True, env=ENV, timeout=30)
            print(opt + ': MQTT tests PASS', flush=True)
        subprocess.run(['gcc', '-std=c11', '-Werror', '-fsyntax-only',
                        '-I' + str(ROOT / 'include'), '-x', 'c', '-'],
                       input='#include "net/mqtt.h"\nMqttDev_t dev;\n', text=True, check=True)
        broker = os.environ.get('MOSQUITTO') or shutil.which('mosquitto')
        if not broker:
            print('Mosquitto interoperability: SKIP (set MOSQUITTO or install mosquitto)')
            return
        uecc = Path(os.environ.get('UECC_ROOT', ROOT / 'micro-ecc'))
        ecc = Path(tmp) / 'uecc.o'
        subprocess.run(['gcc', '-O2', '-g', '-fsanitize=address,undefined', '-fno-pie',
                        '-DuECC_ENABLE_VLI_API=1', '-I' + str(uecc), '-c', str(uecc / 'uECC.c'),
                        '-o', str(ecc)], check=True)
        flags += ['-DuECC_ENABLE_VLI_API=1', '-I' + str(uecc)]
        sources += [str(ROOT / s) for s in ('src/net/tls.cpp', 'src/device.cpp',
                    'src/crypto/crypto_softaes.cpp', 'src/crypto/crypto_softsha256.cpp',
                    'src/crypto/crypto_p256.cpp', 'src/crypto/crypto_uecc.cpp')]
        binary = Path(tmp) / 'mqtt_host'
        subprocess.run(['g++', *flags, str(ROOT / 'tests/net/mqtt_host.cpp'),
                        *sources, str(ecc), '-Wl,--gc-sections', '-o', str(binary)], check=True)
        for secure in (False, True):
            interop(broker, binary, Path(tmp), secure)


if __name__ == '__main__':
    main()
