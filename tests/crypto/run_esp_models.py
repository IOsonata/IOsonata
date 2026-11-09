#!/usr/bin/env python3
"""-------------------------------------------------------------------------
@file   run_esp_models.py

@brief  Build and execute ESP32-C3/C6 register models.

@author IOsonata contributors
@date   Nov. 8, 2026

@license

MIT License

Copyright (c) 2026, I-SYST inc., all rights reserved

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.

-------------------------------------------------------------------------"""
from pathlib import Path
import os
import subprocess
import tempfile

ROOT = Path(__file__).resolve().parents[2]
SOURCES = ['src/device.cpp', 'src/device_intrf.cpp', 'src/coredev/timer.cpp',
    'tests/crypto/timer_fixture.cpp', 'src/crypto/crypto.cpp',
    'src/crypto/crypto_aes_modes.cpp', 'src/crypto/crypto_aes_soft.cpp',
    'src/crypto/crypto_softsha256.cpp', 'src/crypto/crypto_softrng.cpp',
    'src/crypto/crypto_uecc.cpp', 'src/crypto/crypto_p256.cpp', 'micro-ecc/uECC.c',
    'RISCV/Espressif/src/crypto_esp32.cpp', 'exemples/crypto/crypto_engine_test.cpp',
    'tests/crypto/esp_model.cpp']

with tempfile.TemporaryDirectory(prefix='crypto-esp-') as temp:
    out = Path(temp)
    (out / 'coredev').mkdir()
    (out / 'coredev/interrupt.h').write_text('#pragma once\n#include <stdint.h>\n'
        'inline uintptr_t DisableInterrupt() { return 0; }\n'
        'inline void EnableInterrupt(uintptr_t) {}\n')
    for opt in ('-O0', '-O2'):
        for target in ('ESP32C3', 'ESP32C6'):
            flags = [opt, '-g', '-fsanitize=address,undefined', '-fno-omit-frame-pointer',
                '-fno-pie', '-DuECC_ENABLE_VLI_API=1', '-DCRYPTO_TEST_NO_MAIN']
            flags += ['-D' + target, '-DESP32_CRYPTO_POLL_MAX=32', '-include', str(ROOT/'tests/crypto/esp_model.h')]
            if target == 'ESP32C3':
                flags.append('-DESP32_RNG_MODEL_DISABLED')
            inc = ['-I' + str(p) for p in (out, ROOT/'include', ROOT/'micro-ecc',
                ROOT/'RISCV/Espressif/include', ROOT/'tests/crypto')]
            objects = []
            sources = SOURCES
            for n, source in enumerate(sources):
                obj = out / f'{n}.o'
                compiler = ['gcc', '-std=gnu17'] if source.endswith('.c') else [
                    'g++', '-std=gnu++23', '-fno-rtti', '-fno-exceptions', '-Wall', '-Wextra', '-Werror']
                subprocess.run([*compiler, *flags, *inc, '-c', str(ROOT/source), '-o', str(obj)], check=True)
                objects.append(str(obj))
            exe = out/'model'
            subprocess.run(['g++', *flags, '-no-pie', *objects, '-lcrypto', '-o', str(exe)], check=True)
            result = subprocess.run([str(exe)], text=True, capture_output=True,
                env={**os.environ, "ASAN_OPTIONS": "detect_leaks=0"})
            print(target, opt, result.stdout, result.stderr, flush=True)
            assert result.returncode == 0
            assert result.stdout.count('ns/op') == 28
