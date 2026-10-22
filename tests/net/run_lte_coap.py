#!/usr/bin/env python3
# SPDX-License-Identifier: MIT
# Copyright (c) 2026 I-SYST inc.
"""Host checks for the LTE CoAP example using the actual Thingy:91 board header.

Requires gcc/g++; no SDK, modem, network or Python packages are needed. The
Nordic register header is replaced by an empty host header: the application
uses only the real Nordic RNG declaration, never registers. These checks do
not replace an ARM build or a test on an nRF91 with an active SIM.
"""
import os
from pathlib import Path
import subprocess
import tempfile

ROOT = Path(__file__).resolve().parents[2]
BOARD = ROOT / 'ARM/Nordic/nRF91/nRF9160/exemples/LteCoapDemo/src'
ENV = dict(os.environ, ASAN_OPTIONS='detect_leaks=0:abort_on_error=1',
           UBSAN_OPTIONS='halt_on_error=1:print_stacktrace=1')


def main():
    with tempfile.TemporaryDirectory(prefix='iosonata-lte-coap-') as tmp:
        Path(tmp, 'nrf.h').write_text('/* Host: no Nordic registers used. */\n')
        flags = ['-std=gnu++17', '-Wall', '-Wextra', '-Werror',
                 '-Wno-missing-field-initializers', '-fno-exceptions', '-fno-rtti',
                 '-DNRF91_SERIES=1', '-DIRQ_PRIO_LOW=5', '-I' + tmp,
                 '-I' + str(ROOT / 'include'), '-I' + str(ROOT / 'ARM/Nordic/include'),
                 '-I' + str(BOARD)]
        subprocess.run(['g++', *flags, '-fsyntax-only',
                        str(ROOT / 'exemples/lte/lte_coap.cpp')], check=True)
        print('LTE CoAP application with Thingy:91 board header: host syntax PASS', flush=True)
        for opt in ('-O0', '-O2'):
            binary = Path(tmp, 'lte_coap_demo_test')
            subprocess.run(['g++', *flags, opt, '-g', '-fsanitize=address,undefined',
                            '-fno-omit-frame-pointer', '-fno-pie', '-no-pie',
                            '-ffunction-sections', '-fdata-sections',
                            str(ROOT / 'tests/net/lte_coap_demo_test.cpp'),
                            str(ROOT / 'src/net/coap.cpp'),
                            str(ROOT / 'src/device_intrf.cpp'),
                            str(ROOT / 'src/device.cpp'), '-Wl,--gc-sections',
                            '-o', str(binary)], check=True)
            subprocess.run([str(binary)], check=True, env=ENV, timeout=20)
            print(opt + ': LTE CoAP application recovery under ASan/UBSan PASS', flush=True)


if __name__ == '__main__':
    main()
