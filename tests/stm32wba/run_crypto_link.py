#!/usr/bin/env python3
"""Link production WBA crypto without software P-256 or PRNG objects.

Host register storage tests construction and failed initialization only.
No AES/PKA arithmetic, entropy, ARM instructions or silicon timing is modeled.
"""
from pathlib import Path
import argparse
import re
import subprocess
import tempfile

ROOT = Path(__file__).resolve().parents[2]
parser = argparse.ArgumentParser(description=__doc__)
parser.add_argument('--device-header', type=Path,
    default=ROOT / 'ARM/ST/STM32WBAxx/include/stm32wba65xx.h')
args = parser.parse_args()
source = (ROOT / 'ARM/ST/STM32WBAxx/src/crypto_stm32wba.cpp').read_text()
header = args.device_header.read_text()
# Keep register layouts and values from the real device header. Replace only
# CMSIS CPU dependencies and MMIO addresses with host storage.
structs = dict((name, body) for body, name in re.findall(
    r'(typedef struct\s*\{[\s\S]*?\}\s*(\w+_TypeDef);)', header))
names = ('AES', 'PKA', 'RCC', 'RNG')
macros = {m[1]: m[0] for m in re.finditer(r'^#define\s+(\w+)[^\n]*', header, re.M)}
pending = list(set(re.findall(r'\b(?:AES|PKA|RCC|RNG)_\w+\b', source)))
selected = {}
while pending:
    name = pending.pop()
    if name in selected or name not in macros:
        continue
    selected[name] = macros[name]
    pending.extend(n for n in re.findall(r'\b\w+\b', macros[name]) if n in macros and n not in selected)
shim = '#pragma once\n#include <stdint.h>\n#define __IO volatile\n#define __I volatile const\n#define __O volatile\n'
shim += '\n'.join(structs[n + '_TypeDef'] for n in names) + '\n'
shim += '\n'.join(selected.values()) + '\n'
for n in names:
    shim += f'extern {n}_TypeDef g_{n};\n#undef {n}\n#define {n} (&g_{n})\n'
shim += '''#define SET_BIT(r,b) ((r) |= (b))
#define CLEAR_BIT(r,b) ((r) &= ~(b))
#define WRITE_REG(r,v) ((r) = (v))
#define MODIFY_REG(r,c,s) WRITE_REG((r), (((r) & ~(c)) | (s)))
'''
main = '''#include <cassert>
#include <cstring>
#include "stm32wbaxx.h"
#include "crypto/crypto.h"
AES_TypeDef g_AES = {};
PKA_TypeDef g_PKA = {};
RCC_TypeDef g_RCC = {};
RNG_TypeDef g_RNG = {};
int main()
{
	Crypto crypto;
	uint8_t out[64];

	assert(!crypto.Valid());
	// No ready clock in host storage: Init must fail without software retry.
	assert(!crypto.Init());
	assert(!crypto.IsSecure());
	memset(out, 0xa5, sizeof(out));
	assert(crypto.Random(out, sizeof(out)) == CRYPTO_STATUS_FAIL);
	assert(P256IsZero(out, sizeof(out)));
	memset(out, 0xa5, sizeof(out));
	assert(crypto.Hash(CRYPTO_HASH_SHA256, nullptr, 0, out) == CRYPTO_STATUS_FAIL);
	assert(P256IsZero(out, 32));
	assert(crypto.KeyGen(CRYPTO_CURVE_P256, nullptr, out) == CRYPTO_STATUS_FAIL);
	assert(P256IsZero(out, sizeof(out)));
	return 0;
}
'''
sources = ['src/device.cpp', 'src/device_intrf.cpp', 'src/crypto/crypto.cpp',
    'src/crypto/crypto_aes_modes.cpp', 'src/crypto/crypto_aes_soft.cpp',
    'src/crypto/crypto_softsha256.cpp', 'src/crypto/crypto_p256.cpp',
    'ARM/ST/src/crypto_default_stm32.cpp', 'ARM/ST/STM32WBAxx/src/crypto_stm32wba.cpp']
# Deliberately omit crypto_uecc.cpp, uECC.c and crypto_softrng.cpp. A dependency
# on any of their operations must fail the link, even if size checks change.
with tempfile.TemporaryDirectory(prefix='wba-crypto-link-') as temp:
    out = Path(temp)
    (out / 'stm32wbaxx.h').write_text(shim)
    (out / 'stm32.h').write_text('#define IOSONATA_STM32_HAS_RNG 1\n')
    (out / 'main.cpp').write_text(main)
    for opt in ('-O0', '-Os'):
        flags = ['g++', '-std=gnu++23', opt, '-Wall', '-Wextra', '-Werror',
            '-fno-rtti', '-fno-exceptions', '-ffunction-sections', '-fdata-sections',
            '-DSTM32WBA65xx', '-DCRYPTO_WBA_RNG_POLL_MAX=4',
            '-I' + str(out), '-I' + str(ROOT / 'include'), '-I' + str(ROOT / 'ARM/ST/include')]
        objects = []
        for i, path in enumerate(sources):
            obj = out / f'{i}.o'
            subprocess.run([*flags, '-c', str(ROOT / path), '-o', str(obj)], check=True)
            objects.append(str(obj))
        archive = out / 'libcrypto.a'
        subprocess.run(['ar', 'rcs', str(archive), *objects], check=True)
        exe = out / 'test'
        subprocess.run([*flags, str(out / 'main.cpp'), str(archive), '-Wl,--gc-sections', '-o', str(exe)], check=True)
        subprocess.run([str(exe)], check=True)
        symbols = subprocess.check_output(['nm', '-C', str(exe)], text=True)
        for forbidden in ('uECC_', 'CryptoSoftRng', 'CryptoUecc::Sign', 'CryptoUecc::Verify',
                          'CryptoUecc::KeyGen', 'CryptoUecc::KeyImport', 'CryptoUecc::Agree'):
            assert forbidden not in symbols, forbidden
        for required in ('CryptoWbaPka::Sign', 'CryptoWbaPka::Verify', 'CryptoWbaAes::Cipher',
                         'CryptoSoftAes::Cipher', 'CryptoSoftSha256Hash::Hash'):
            assert required in symbols, required
        print(opt, 'WBA target: no software P-256 or PRNG; required AES-256/SHA retained', flush=True)
