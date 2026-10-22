#!/usr/bin/env python3
# SPDX-License-Identifier: MIT
# Copyright (c) 2026 I-SYST inc.
"""Exercise the actual nRF91 socket port with host Nordic API substitutes.

No SDK, modem, cellular network or remote server is needed. The test covers
socket setup, timeouts, address-family fallback and error cleanup; it does not
claim an ARM build or establish compatibility with any modem firmware.
"""
import argparse
import os
from pathlib import Path
import subprocess
import tempfile

ROOT = Path(__file__).resolve().parents[2]
ENV = dict(os.environ, ASAN_OPTIONS='detect_leaks=0:abort_on_error=1',
           UBSAN_OPTIONS='halt_on_error=1:print_stacktrace=1')


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--source', type=Path,
                        default=ROOT / 'ARM/Nordic/nRF91/src/sock_intrf_nrf91.cpp',
                        help='alternate socket source, for regression negative controls')
    args = parser.parse_args()
    with tempfile.TemporaryDirectory(prefix='iosonata-nrf91-socket-') as tmp:
        flags = ['-std=gnu++17', '-Wall', '-Wextra', '-Werror',
                 '-Wno-missing-field-initializers', '-fno-exceptions', '-fno-rtti',
                 '-I' + str(ROOT / 'tests/net/nrf91_socket_stubs'),
                 '-I' + str(ROOT / 'include')]
        for opt in ('-O0', '-O2'):
            binary = Path(tmp, 'nrf91_socket_test')
            subprocess.run(['g++', *flags, opt, '-g', '-fsanitize=address,undefined',
                            '-fno-omit-frame-pointer', '-fno-pie', '-no-pie',
                            '-ffunction-sections', '-fdata-sections',
                            str(ROOT / 'tests/net/nrf91_socket_test.cpp'),
                            str(args.source), str(ROOT / 'src/device_intrf.cpp'),
                            '-Wl,--gc-sections', '-o', str(binary)], check=True)
            subprocess.run([str(binary)], check=True, env=ENV, timeout=20)
            print(opt + ': nRF91 socket setup under ASan/UBSan PASS', flush=True)


if __name__ == '__main__':
    main()
