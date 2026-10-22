#!/usr/bin/env python3
# SPDX-License-Identifier: MIT
# Copyright (c) 2026 I-SYST inc.
"""Host checks for the LTE MQTT example using its Thingy:91 board header.

The actual shared MQTT client runs through a simulated stream socket, modem,
timer and application event queue. Requires g++; no SDK, radio, broker or Python
packages are needed. These checks do not replace an ARM build or a test on an
nRF91 with an active SIM.
"""
import os
from pathlib import Path
import subprocess
import tempfile

ROOT = Path(__file__).resolve().parents[2]
BOARD = ROOT / 'ARM/Nordic/nRF91/nRF9160/exemples/LteMqttDemo/src'
ENV = dict(os.environ, ASAN_OPTIONS='detect_leaks=0:abort_on_error=1',
           UBSAN_OPTIONS='halt_on_error=1:print_stacktrace=1')


def main():
    with tempfile.TemporaryDirectory(prefix='iosonata-lte-mqtt-') as tmp:
        Path(tmp, 'nrf.h').write_text('/* Host: no Nordic registers used. */\n')
        flags = ['-std=gnu++17', '-Wall', '-Wextra', '-Werror',
                 '-Wno-missing-field-initializers', '-fno-exceptions', '-fno-rtti',
                 '-DNRF91_SERIES=1', '-DIRQ_PRIO_LOW=5', '-I' + tmp,
                 '-I' + str(ROOT / 'include'), '-I' + str(ROOT / 'ARM/Nordic/include'),
                 '-I' + str(BOARD)]
        subprocess.run(['g++', *flags, '-fsyntax-only',
                        str(ROOT / 'exemples/lte/lte_mqtt.cpp')], check=True)
        print('LTE MQTT application with Thingy:91 board header: host syntax PASS', flush=True)
        for opt in ('-O0', '-O2'):
            binary = Path(tmp, 'lte_mqtt_demo_test')
            subprocess.run(['g++', *flags, opt, '-g', '-fsanitize=address,undefined',
                            '-fno-omit-frame-pointer', '-fno-pie', '-no-pie',
                            '-ffunction-sections', '-fdata-sections',
                            str(ROOT / 'tests/net/lte_mqtt_demo_test.cpp'),
                            str(ROOT / 'src/net/mqtt.cpp'),
                            str(ROOT / 'src/device_intrf.cpp'),
                            str(ROOT / 'src/device.cpp'), '-Wl,--gc-sections',
                            '-o', str(binary)], check=True)
            subprocess.run([str(binary)], check=True, env=ENV, timeout=20)
            print(opt + ': LTE MQTT application recovery under ASan/UBSan PASS', flush=True)


if __name__ == '__main__':
    main()
