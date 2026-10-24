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
import signal
import shutil
import socket
import subprocess
import sys
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
    command = [sys.executable, str(ROOT / 'Python/mqtt_tls_bridge.py'),
               '--host', '127.0.0.1', '--port', str(port),
               '--broker-port', str(broker_port)]
    with tempfile.TemporaryFile() as log:
        server = subprocess.Popen(command, stdout=log, stderr=log)
        try:
            for _ in range(2):
                subprocess.run([str(binary), str(port), 'tls'],
                               check=True, env=ENV, timeout=40)
                time.sleep(0.5)
            print('MQTT TLS helper: two independent sessions PASS', flush=True)
        except Exception:
            log.seek(0)
            print(log.read().decode(), flush=True)
            raise
        finally:
            server.send_signal(signal.SIGINT)
            try:
                server.wait(timeout=5)
            except subprocess.TimeoutExpired:
                server.kill()
                server.wait(timeout=5)


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
                    'src/crypto/x509.cpp',
                    'src/crypto/crypto_softaes.cpp', 'src/crypto/crypto_aes_soft.cpp', 'src/crypto/crypto_softsha256.cpp',
                    'src/crypto/crypto_p256.cpp', 'src/crypto/crypto_uecc.cpp')]
        binary = Path(tmp) / 'mqtt_host'
        subprocess.run(['g++', *flags, str(ROOT / 'tests/net/mqtt_host.cpp'),
                        *sources, str(ecc), '-Wl,--gc-sections', '-o', str(binary)], check=True)
        for secure in (False, True):
            interop(broker, binary, Path(tmp), secure)


if __name__ == '__main__':
    main()
