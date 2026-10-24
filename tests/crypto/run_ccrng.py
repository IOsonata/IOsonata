#!/usr/bin/env python3
"""Check production CryptoCell DRBG against independent Python HMAC/SHA-256."""
from pathlib import Path
import hashlib
import hmac
import os
import subprocess
import tempfile

ROOT = Path(__file__).resolve().parents[2]
SOURCES = ['src/device.cpp', 'src/device_intrf.cpp', 'src/coredev/timer.cpp',
    'tests/crypto/timer_fixture.cpp', 'src/crypto/crypto.cpp', 'src/crypto/crypto_default.cpp',
    'src/crypto/crypto_aes_modes.cpp', 'src/crypto/crypto_aes_soft.cpp',
    'src/crypto/crypto_softsha256.cpp', 'src/crypto/crypto_softrng.cpp',
    'src/crypto/crypto_uecc.cpp', 'src/crypto/crypto_p256.cpp', 'micro-ecc/uECC.c',
    'ARM/src/cc3xx_intrf.cpp', 'ARM/Nordic/src/rng_nrfx.cpp', 'tests/crypto/ccrng_test.cpp']

def reference():
    key, value, calls, used, serial = bytes(32), bytes([1])*32, 0, 0, 0
    def mac(k, msg):
        return hmac.new(k, msg, hashlib.sha256).digest()
    def update(k, v, seed=b''):
        k = mac(k, v+b'\x00'+seed)
        v = mac(k, v)
        if seed:
            k = mac(k, v+b'\x01'+seed)
            v = mac(k, v)
        return k, v
    result = bytearray()
    for length in [1, 16, 33, 1024, 65536, 70000] + [1]*1030:
        while length:
            if not calls or calls > 1024 or used == 65536:
                seed = bytes((serial*73+i) & 255 for i in range(48))
                serial += 1
                key, value = update(key, value, seed)
                calls, used = 1, 0
            count = min(length, 65536-used)
            generated = bytearray()
            while len(generated) < count:
                value = mac(key, value)
                generated += value
            result += generated[:count]
            key, value = update(key, value)
            calls += 1
            used += count
            length -= count
    assert serial == 4
    return result

with tempfile.TemporaryDirectory(prefix='ccrng-') as temp:
    out = Path(temp)
    (out/'coredev').mkdir()
    (out/'coredev/interrupt.h').write_text('#pragma once\n#include <stdint.h>\n'
        'inline uintptr_t DisableInterrupt() { return 0; }\ninline void EnableInterrupt(uintptr_t) {}\n')
    (out/'nrf.h').write_text('#pragma once\n')
    (out/'crypto_cc3xx_sym.h').write_text('#include "ccrng_hash_fixture.h"\n')
    for target in ('NRF5340_XXAA_APPLICATION', 'NRF91_SERIES'):
        for opt in ('-O0', '-Os', '-O2'):
            flags = [opt, '-g', '-fsanitize=address,undefined', '-fno-omit-frame-pointer',
                '-fno-pie', '-DuECC_ENABLE_VLI_API=1', '-D'+target]
            inc = ['-I'+str(p) for p in (out, ROOT/'include', ROOT/'micro-ecc',
                ROOT/'ARM/include', ROOT/'ARM/Nordic/include', ROOT/'tests/crypto')]
            objects = []
            for n, source in enumerate(SOURCES):
                obj = out/f'{n}.o'
                compiler = ['gcc', '-std=gnu17'] if source.endswith('.c') else [
                    'g++', '-std=gnu++23', '-fno-rtti', '-fno-exceptions', '-Wall', '-Wextra', '-Werror']
                subprocess.run([*compiler, *flags, *inc, '-c', str(ROOT/source), '-o', str(obj)], check=True)
                objects.append(str(obj))
            exe = out/'test'
            subprocess.run(['g++', *flags, '-no-pie', *objects, '-lcrypto', '-o', str(exe)], check=True)
            subprocess.run([str(exe), str(out/'output')], check=True,
                env={**os.environ, 'ASAN_OPTIONS': 'detect_leaks=0'})
            assert (out/'output').read_bytes() == reference()
            print(target, opt, 'independent HMAC_DRBG comparison passed', flush=True)
