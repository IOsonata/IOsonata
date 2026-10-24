#!/usr/bin/env python3
"""Exercise CryptoMaster DMA and AES modes with an OpenSSL model."""
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
    'src/crypto/cryptomaster.cpp', 'exemples/crypto/crypto_engine_test.cpp',
    'tests/crypto/cm_model.cpp']

with tempfile.TemporaryDirectory(prefix='crypto-sam-') as temp:
    out = Path(temp)
    (out / 'coredev').mkdir()
    (out / 'coredev/interrupt.h').write_text('#pragma once\n#include <stdint.h>\n'
        'inline uintptr_t DisableInterrupt() { return 0; }\n'
        'inline void EnableInterrupt(uintptr_t) {}\n')
    (out/'idelay.h').write_text('#pragma once\n#include <stdint.h>\nextern "C" void usDelay(uint32_t);\n')
    for opt in ('-O0', '-O2'):
        for target in ('cryptomaster',):
            flags = [opt, '-g', '-fsanitize=address,undefined', '-fno-omit-frame-pointer',
                '-fno-pie', '-DuECC_ENABLE_VLI_API=1', '-DCRYPTO_TEST_NO_MAIN']
            inc = ['-I' + str(p) for p in (out, ROOT/'include', ROOT/'micro-ecc',
                 ROOT/'tests/crypto')]
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

