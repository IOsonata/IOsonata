#!/usr/bin/env python3
"""STM32WBA65 crypto engine test on an instruction set emulator.

Run: python3 tests/stm32wba/run_crypto.py

Needs arm-none-eabi-g++, and the python packages unicorn, pyelftools and
cryptography. Without them the test prints NOT RUN.

crypto_fw.cpp is built for the Cortex-M33 together with the generic test
exemples/crypto/crypto_engine_test.cpp, the board.h of the CryptoTest project
and ARM/ST/STM32WBAxx/src/crypto_stm32wba.cpp, and run in unicorn. The AES,
PKA and RNG blocks, and the RCC bits that clock them, are a model written from
the reference HAL (stm32wbaxx_hal_cryp.c, stm32wbaxx_hal_pka.c, the RNG
bring-up of the HAL):

- AES: configuration only with the block disabled, key and IV registers
  written before the enable, ECB and CBC decryption refuse to produce a
  correct result unless the key derivation mode ran first (the key registers
  then hold the last round key), the block disables itself at the end of the
  key derivation, input written over an unread output sets WRERR, an output
  read with none ready sets RDERR. CTR runs with a 32 bit and with a 128 bit
  counter increment, the engine has to give the same result with both. The
  key and IV registers must be zero at the end.
- PKA: operands checked for the end marker word, the field prime and the
  curve constants taken from RAM, the Montgomery parameter of the point check
  compared with R^2 mod p, the input point of a scalar multiplication checked
  to be on the curve, no RAM or mode access while busy. The result is computed
  with python integers, the end flag comes after a random number of status
  reads. The RAM must be zero at the end.
- RNG: clocked only from HSI16 with the block enabled, health test register
  written only during the conditioning reset, data read only with DRDY set,
  a seed error injected during the run which the engine has to recover from.
- every access to a block whose clock is off is a violation.

The model is a reading of the HAL and the reference manual, so it finds
errors of logic, layout and sequence, and not differences of the silicon.

Not hardware validation.

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
"""
import collections
from pathlib import Path
import random
import shutil
import sys
import tempfile

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
import run_clock as rc  # noqa: E402  header helpers and the build runner

ROOT = rc.ROOT
hdr = rc.hdr
reg_off = rc.reg_off

BOARD_DIR = ROOT / 'ARM/ST/STM32WBAxx/STM32WBA65/exemples/CryptoTest/src'

# Number of rounds of the soak part, kept low for the emulator
SOAK_PKA = 2
SOAK_AES = 24


def build(tmp):
    cpu = ['-mcpu=cortex-m33', '-mthumb', '-mfloat-abi=soft', '-Os', '-DSTM32WBA65xx', '-DIOSONATA_STM32_WBA',
           '-ffunction-sections', '-fdata-sections', '-g']
    inc = ['-I' + str(p) for p in (ROOT / 'include', ROOT / 'ARM/include', ROOT / 'ARM/CMSIS/Core/Include',
                                   ROOT / 'ARM/ST/include', ROOT / 'ARM/ST/STM32WBAxx/include',
                                   ROOT / 'micro-ecc', BOARD_DIR)]
    cxx = ['arm-none-eabi-g++', *cpu, '-std=gnu++23', '-Wall', '-Werror', '-fno-exceptions', '-fno-rtti', *inc]
    srcs = [
        (HERE / 'crypto_fw.cpp', []),
        (ROOT / 'exemples/crypto/crypto_engine_test.cpp',
         ['-DCRYPTO_TEST_NO_MAIN', '-DCRYPTO_TEST_PKA_DIAG_ALWAYS=1', f'-DCRYPTO_TEST_SOAK_PKA={SOAK_PKA}', f'-DCRYPTO_TEST_SOAK_AES={SOAK_AES}']),
        (ROOT / 'ARM/ST/STM32WBAxx/src/crypto_stm32wba.cpp', []),
        (ROOT / 'src/crypto/crypto_softaes.cpp', []),
        (ROOT / 'src/crypto/crypto_softrng.cpp', []),
        (ROOT / 'src/crypto/crypto_p256.cpp', []),
        (ROOT / 'src/crypto/crypto_uecc.cpp', []),
        (ROOT / 'src/device.cpp', []),
    ]
    objs = []
    for src, extra in srcs:
        obj = tmp / (src.stem + '.o')
        rc.run([*cxx, *extra, '-c', src, '-o', obj])
        objs.append(obj)
    obj = tmp / 'uECC.o'
    rc.run(['arm-none-eabi-gcc', *cpu, '-std=gnu17', *inc, '-c', ROOT / 'micro-ecc/uECC.c', '-o', obj])
    objs.append(obj)
    elf = tmp / 'crypto_fw.elf'
    rc.run(['arm-none-eabi-g++', *cpu, '-nostartfiles', '-Wl,--gc-sections', '-Wl,-T,' + str(HERE / 'crypto_fw.ld'),
            '--specs=nosys.specs', '--specs=nano.specs', *objs, '-o', elf])
    return elf


# ---------------------------------------------------------------------------
# AES helpers

def _sbox():
    box = [0] * 256
    p = q = 1
    while True:
        p = p ^ ((p << 1) & 0xFF) ^ (0x1B if p & 0x80 else 0)
        q ^= q << 1
        q ^= q << 2
        q ^= q << 4
        q &= 0xFF
        if q & 0x80:
            q ^= 0x09
        x = q ^ (q << 1 | q >> 7) & 0xFF ^ (q << 2 | q >> 6) & 0xFF ^ (q << 3 | q >> 5) & 0xFF ^ (q << 4 | q >> 4) & 0xFF
        box[p] = (x ^ 0x63) & 0xFF
        if p == 1:
            break
    box[0] = 0x63
    return box


SBOX = _sbox()
RCON = [0x01, 0x02, 0x04, 0x08, 0x10, 0x20, 0x40, 0x80, 0x1B, 0x36]


def _g(w, rnd):
    w = w[1:] + w[:1]
    w = [SBOX[b] for b in w]
    w[0] ^= RCON[rnd]
    return w


def last_round_key(key):
    """Round key 10 of an AES-128 key."""
    w = [list(key[4 * i:4 * i + 4]) for i in range(4)]
    for i in range(4, 44):
        t = w[i - 1]
        if i % 4 == 0:
            t = _g(t, i // 4 - 1)
        w.append([a ^ b for a, b in zip(w[i - 4], t)])
    return bytes(sum(w[40:44], []))


def key_from_last_round_key(rk):
    """Inverse of the key schedule: the cipher key behind round key 10."""
    w = [None] * 44
    for i in range(4):
        w[40 + i] = list(rk[4 * i:4 * i + 4])
    for i in range(43, 3, -1):
        t = _g(w[i - 1], i // 4 - 1) if i % 4 == 0 else w[i - 1]
        w[i - 4] = [a ^ b for a, b in zip(w[i], t)]
    return bytes(sum(w[0:4], []))


def be_words_to_bytes(words):
    return b''.join(w.to_bytes(4, 'big') for w in words)


def bytes_to_be_words(b):
    return [int.from_bytes(b[i:i + 4], 'big') for i in range(0, len(b), 4)]


# ---------------------------------------------------------------------------
# Peripheral models

class Env:
    def __init__(self, seed, ctr_bits, rng_fault):
        self.viol = []
        self.rnd = random.Random(seed)
        self.ctr_bits = ctr_bits
        self.rng_fault = rng_fault
        self.stats = collections.Counter()

    def v(self, msg):
        if len(self.viol) < 200:
            self.viol.append(msg)


class Rcc:
    def __init__(self, env):
        self.env = env
        self.reg = collections.defaultdict(int)
        self.off = {n: reg_off('RCC_TypeDef', n) for n in ('CR', 'AHB2ENR', 'AHB2RSTR', 'CCIPR2')}
        self.pka_reset = None
        self.aes_reset = None
        self.rng_clock_hook = None

    def aes_clk(self):
        return bool(self.reg[self.off['AHB2ENR']] & (1 << hdr('RCC_AHB2ENR_AESEN_Pos')))

    def pka_clk(self):
        return bool(self.reg[self.off['AHB2ENR']] & (1 << hdr('RCC_AHB2ENR_PKAEN_Pos')))

    def rng_clk(self):
        ok = bool(self.reg[self.off['AHB2ENR']] & (1 << hdr('RCC_AHB2ENR_RNGEN_Pos')))
        sel = (self.reg[self.off['CCIPR2']] >> hdr('RCC_CCIPR2_RNGSEL_Pos')) & 3
        return ok, sel

    def rd(self, off):
        v = self.reg[off]
        if off == self.off['CR']:
            if v & (1 << hdr('RCC_CR_HSION_Pos')):
                v |= 1 << hdr('RCC_CR_HSIRDY_Pos')
            else:
                v &= ~(1 << hdr('RCC_CR_HSIRDY_Pos'))
        return v

    def wr(self, off, val):
        if off == self.off['AHB2RSTR']:
            old = self.reg[off]
            for bit, cb in ((hdr('RCC_AHB2RSTR_PKARST_Pos'), self.pka_reset),
                            (hdr('RCC_AHB2RSTR_AESRST_Pos'), self.aes_reset)):
                if (val >> bit) & 1 and not (old >> bit) & 1 and cb:
                    cb()
        self.reg[off] = val


class Aes:
    CR_EN = 1
    MODE_SHIFT = 3
    CHMOD_SHIFT = 5

    def __init__(self, env, rcc):
        self.env = env
        self.rcc = rcc
        o = {n: reg_off('AES_TypeDef', n) for n in (
            'CR', 'SR', 'DINR', 'DOUTR', 'KEYR0', 'KEYR1', 'KEYR2', 'KEYR3', 'IVR0', 'IVR1', 'IVR2', 'IVR3',
            'IER', 'ISR', 'ICR')}
        self.o = o
        self.cr = 0
        self.key = [0] * 4
        self.key_mask = 0
        self.iv = [0] * 4
        self.iv_mask = 0
        self.din = []
        self.dout = []
        self.isr = 0
        self.sr_err = 0
        self.derived = False
        rcc.aes_reset = self.reset

    def reset(self):
        self.cr = 0
        self.key = [0] * 4
        self.iv = [0] * 4
        self.key_mask = 0
        self.iv_mask = 0
        self.din = []
        self.dout = []
        self.isr = 0
        self.sr_err = 0
        self.derived = False

    def mode(self):
        return (self.cr >> self.MODE_SHIFT) & 3

    def chmod(self):
        return ((self.cr >> self.CHMOD_SHIFT) & 3) | (((self.cr >> 16) & 1) << 2)

    def key_bytes(self):
        return be_words_to_bytes([self.key[3], self.key[2], self.key[1], self.key[0]])

    def iv_bytes(self):
        return be_words_to_bytes([self.iv[3], self.iv[2], self.iv[1], self.iv[0]])

    def set_iv(self, b):
        w = bytes_to_be_words(b)
        self.iv = [w[3], w[2], w[1], w[0]]

    def sr(self):
        v = 0
        if self.isr & 1:
            v |= 1
        if self.sr_err & 1:
            v |= 2
        if self.sr_err & 2:
            v |= 4
        if self.key_mask == 0xF:
            v |= 1 << 7
        return v

    def rd(self, off):
        if not self.rcc.aes_clk():
            self.env.v('AES register read with the clock off')
            return 0
        o = self.o
        if off == o['CR']:
            return self.cr
        if off == o['SR']:
            return self.sr()
        if off == o['ISR']:
            return (self.isr & ~2) | (2 if self.sr_err else 0)
        if off == o['IER']:
            return 0
        if off == o['DOUTR']:
            if not self.dout:
                self.sr_err |= 1
                self.env.v('AES DOUTR read with no output ready')
                return 0
            return self.dout.pop(0)
        return 0

    def wr(self, off, val):
        env = self.env
        if not self.rcc.aes_clk():
            env.v('AES register write with the clock off')
            return
        o = self.o
        if off == o['CR']:
            self.write_cr(val)
        elif off == o['ICR']:
            if val & 1:
                self.isr &= ~1
            if val & 2:
                self.sr_err = 0
            if val & 4:
                self.isr &= ~4
        elif off == o['DINR']:
            self.write_din(val)
        elif off == o['IER']:
            if val:
                env.v('AES interrupt enabled in a polled engine')
        else:
            for i in range(4):
                if off == o[f'KEYR{i}']:
                    if self.cr & self.CR_EN:
                        env.v('AES key register written while enabled')
                        return
                    self.key[i] = val
                    self.key_mask |= 1 << i
                    self.derived = False
                    return
                if off == o[f'IVR{i}']:
                    if self.cr & self.CR_EN:
                        env.v('AES IV register written while enabled')
                        return
                    self.iv[i] = val
                    self.iv_mask |= 1 << i
                    return

    def write_cr(self, val):
        env = self.env
        old = self.cr
        if old & self.CR_EN and val & self.CR_EN and (old ^ val) & ~self.CR_EN:
            env.v('AES configuration changed while enabled')
            return
        if val & (1 << 31):
            self.reset()
            return
        if val == 0:
            # full clear: the key and IV have to be written again
            self.key_mask = 0
            self.iv_mask = 0
        if (val >> 1) & 3:
            env.v('AES DATATYPE not zero, the engine layout assumes no swap')
        if (val >> 18) & 1 or (val >> 24) & 7:
            env.v('AES key size or key mode bits set')
        self.cr = val
        if not (old & self.CR_EN) and val & self.CR_EN:
            self.start()
        elif old & self.CR_EN and not val & self.CR_EN:
            self.din = []
            self.dout = []

    def start(self):
        env = self.env
        if self.key_mask != 0xF:
            env.v('AES enabled with the key not fully written')
        ch = self.chmod()
        if ch not in (0, 1, 2):
            env.v(f'AES chaining mode {ch} not used by the engine')
        if ch in (1, 2) and self.mode() != 1 and self.iv_mask != 0xF:
            env.v('AES enabled with the IV not fully written')
        if self.mode() == 1:
            if ch not in (0, 1):
                env.v('AES key derivation with a chaining mode other than ECB or CBC')
            rk = last_round_key(self.key_bytes())
            w = bytes_to_be_words(rk)
            self.key = [w[3], w[2], w[1], w[0]]
            self.derived = True
            self.isr |= 1
            self.cr &= ~self.CR_EN
            env.stats['aes_keyder'] += 1

    def write_din(self, val):
        env = self.env
        if not self.cr & self.CR_EN:
            env.v('AES DINR written while disabled')
            return
        if self.mode() == 1:
            env.v('AES DINR written in key derivation mode')
            return
        if not self.din and self.dout:
            self.sr_err |= 2
            env.v('AES input written over an unread output')
            return
        self.din.append(val)
        if len(self.din) == 4:
            blk = be_words_to_bytes(self.din)
            self.din = []
            self.dout = bytes_to_be_words(self.compute(blk))
            self.isr |= 1
            env.stats['aes_blocks'] += 1

    def compute(self, blk):
        from cryptography.hazmat.primitives.ciphers import Cipher, algorithms, modes
        env = self.env
        ch = self.chmod()
        mode = self.mode()
        kb = self.key_bytes()
        if ch == 2:
            # CTR: the keystream is the encryption of the counter in both modes
            ks = Cipher(algorithms.AES(kb), modes.ECB()).encryptor().update(self.iv_bytes())
            out = bytes(a ^ b for a, b in zip(blk, ks))
            ctr = int.from_bytes(self.iv_bytes(), 'big')
            if env.ctr_bits == 32:
                ctr = (ctr & ~0xFFFFFFFF) | ((ctr + 1) & 0xFFFFFFFF)
            else:
                ctr = (ctr + 1) & ((1 << 128) - 1)
            self.set_iv(ctr.to_bytes(16, 'big'))
            return out
        if mode == 0:
            if ch == 1:
                blk = bytes(a ^ b for a, b in zip(blk, self.iv_bytes()))
            out = Cipher(algorithms.AES(kb), modes.ECB()).encryptor().update(blk)
            if ch == 1:
                self.set_iv(out)
            return out
        if mode == 2:
            if not self.derived:
                env.v('AES decryption without the key derivation')
            # The key registers hold round key 10 after the derivation
            ck = key_from_last_round_key(kb)
            out = Cipher(algorithms.AES(ck), modes.ECB()).decryptor().update(blk)
            if ch == 1:
                out = bytes(a ^ b for a, b in zip(out, self.iv_bytes()))
                self.set_iv(blk)
            return out
        env.v(f'AES mode {mode} not valid')
        return bytes(16)


class Rng:
    CR_RNGEN = 1 << 2
    CR_CED = 1 << 5
    CR_CONDRST = 1 << 30

    def __init__(self, env, rcc):
        self.env = env
        self.rcc = rcc
        o = {'CR': reg_off('RNG_TypeDef', 'CR'), 'SR': reg_off('RNG_TypeDef', 'SR'),
             'DR': reg_off('RNG_TypeDef', 'DR'), 'HTCR': 0x10}
        self.o = o
        self.cr = 0
        self.sr = 0
        self.htcr = 0
        self.wait = 0
        self.rd_count = 0
        self.cond_reads = 0
        self.fault_done = False
        self.nist = hdr('RNG_CR_NIST_VALUE')
        self.htnist = hdr('RNG_HTCR_NIST_VALUE')
        self.gen = random.Random(env.rnd.random())

    def clk_ok(self):
        ok, sel = self.rcc.rng_clk()
        if not ok:
            self.env.v('RNG register access with the clock off')
            return False
        if sel != 2:
            self.env.v('RNG kernel clock is not HSI16')
        return True

    def rd(self, off):
        if not self.clk_ok():
            return 0
        o = self.o
        if off == o['CR']:
            v = self.cr
            if self.cr & self.CR_CONDRST == 0 and self.cond_reads > 0:
                # the reset is released a little after the write
                self.cond_reads -= 1
                v |= self.CR_CONDRST
            return v
        if off == o['SR']:
            if self.cr & self.CR_RNGEN and not self.cr & self.CR_CONDRST and not self.sr & 0x66 and self.sr & 1 == 0:
                if self.wait > 0:
                    self.wait -= 1
                else:
                    self.sr |= 1
            return self.sr
        if off == o['DR']:
            if not self.sr & 1:
                self.env.v('RNG DR read without DRDY')
                return 0
            self.rd_count += 1
            self.sr &= ~1
            self.wait = self.env.rnd.randint(0, 6)
            if self.env.rng_fault and not self.fault_done and self.rd_count == 7:
                # seed error after this word: it is still valid, the next ones are not
                self.fault_done = True
                self.sr |= 1 << 6
                self.env.stats['rng_seis'] += 1
            self.env.stats['rng_words'] += 1
            return self.gen.getrandbits(32)
        if off == o['HTCR']:
            return self.htcr
        return 0

    def wr(self, off, val):
        if not self.clk_ok():
            return
        o = self.o
        if off == o['CR']:
            old = self.cr
            if old & self.CR_CONDRST == 0 and val & self.CR_CONDRST == 0 and (old ^ val) & 0x3FFFFF00 \
                    and not (old & self.CR_RNGEN):
                self.env.v('RNG configuration written without the conditioning reset')
            if (old ^ val) & 0x3FFFFF00 and old & self.CR_RNGEN:
                self.env.v('RNG configuration changed while enabled')
            if old & self.CR_CONDRST and not val & self.CR_CONDRST:
                self.cond_reads = 2
                if self.htcr != self.htnist:
                    self.env.v(f'RNG health test register {self.htcr:#x} is not the NIST value')
                if (val & 0x3FFFFF00) != (self.nist & 0x3FFFFF00):
                    self.env.v(f'RNG configuration {val:#x} is not the NIST value')
                self.wait = self.env.rnd.randint(1, 8)
            if val & self.CR_RNGEN and not old & self.CR_RNGEN:
                if val & self.CR_CONDRST:
                    self.env.v('RNG enabled during the conditioning reset')
                self.wait = self.env.rnd.randint(1, 8)
            if not val & self.CR_RNGEN:
                self.sr &= ~1
            self.cr = val
        elif off == o['SR']:
            # CEIS and SEIS clear by writing zero, the others are read only
            for bit in (5, 6):
                if not (val >> bit) & 1:
                    self.sr &= ~(1 << bit)
        elif off == o['HTCR']:
            if not self.cr & self.CR_CONDRST:
                self.env.v('RNG health test register written outside the conditioning reset')
                return
            self.htcr = val


class Pka:
    def __init__(self, env, rcc):
        self.env = env
        self.rcc = rcc
        o = {n: reg_off('PKA_TypeDef', n) for n in ('CR', 'SR', 'CLRFR')}
        self.o = o
        self.ram_base = 0x400
        self.ram = [0] * 1334
        self.cr = 0
        self.initok_wait = None
        self.initok = False
        self.busy_polls = 0
        self.procend = False
        self.err = 0
        rcc.pka_reset = self.reset

    def reset(self):
        self.ram = [0] * 1334
        self.cr = 0
        self.initok = False
        self.initok_wait = None
        self.busy_polls = 0
        self.procend = False
        self.err = 0

    def sr(self):
        v = 0
        if self.initok_wait is not None:
            if self.initok_wait > 0:
                self.initok_wait -= 1
            else:
                self.initok = True
                self.initok_wait = None
        if self.initok:
            v |= 1
        if self.busy_polls > 0:
            self.busy_polls -= 1
            if self.busy_polls == 0:
                self.finish()
            else:
                v |= 1 << 16
        if self.procend:
            v |= 1 << 17
        v |= self.err
        return v

    def rd(self, off):
        if not self.rcc.pka_clk():
            self.env.v('PKA register read with the clock off')
            return 0
        if off == self.o['CR']:
            return self.cr
        if off == self.o['SR']:
            return self.sr()
        if off >= self.ram_base:
            if self.busy_polls > 0:
                self.env.v('PKA RAM read while busy')
            return self.ram[(off - self.ram_base) // 4]
        return 0

    def wr(self, off, val):
        env = self.env
        if not self.rcc.pka_clk():
            env.v('PKA register write with the clock off')
            return
        if off == self.o['CR']:
            old = self.cr
            if self.busy_polls > 0:
                env.v('PKA CR written while busy')
                return
            if val & ((1 << 17) | (1 << 19) | (1 << 20) | (1 << 21)):
                env.v('PKA interrupt enabled in a polled engine')
            if not old & 1 and val & 1:
                self.ram = [0] * 1334
                self.initok = False
                self.initok_wait = env.rnd.randint(2, 20)
            if old & 1 and not val & 1:
                self.reset()
                self.cr = val
                return
            self.cr = val & ~2
            if val & 2:
                self.start((val >> 8) & 0x3F)
        elif off == self.o['CLRFR']:
            if val & (1 << 17):
                self.procend = False
            self.err &= ~(val & ((1 << 19) | (1 << 20) | (1 << 21)))
        elif off >= self.ram_base:
            if self.busy_polls > 0:
                env.v('PKA RAM written while busy')
                return
            if not self.initok:
                env.v('PKA RAM written before INITOK')
            self.ram[(off - self.ram_base) // 4] = val

    # --- operations -------------------------------------------------------
    def start(self, mode):
        env = self.env
        if not self.cr & 1 or not self.initok:
            env.v('PKA started without EN and INITOK')
            self.err |= 1 << 21
            return
        self.mode = mode
        self.busy_polls = env.rnd.randint(3, 40)

    def word(self, name):
        return hdr(name)

    def num(self, idx, check_end=True):
        w = self.ram[idx:idx + 8]
        if check_end and self.ram[idx + 8] != 0:
            self.env.v(f'PKA operand at word {idx:#x} has no end marker word')
        return sum(x << (32 * i) for i, x in enumerate(w))

    def put(self, idx, val):
        for i in range(8):
            self.ram[idx + i] = (val >> (32 * i)) & 0xFFFFFFFF
        self.ram[idx + 8] = 0
        self.ram[idx + 9] = 0

    def finish(self):
        env = self.env
        m = self.mode
        if m == 0x01:
            self.do_montgomery()
        elif m == 0x20:
            self.do_mul()
        elif m == 0x24:
            self.do_sign()
        elif m == 0x26:
            self.do_verify()
        elif m == 0x28:
            self.do_check()
        else:
            env.v(f'PKA mode {m:#x} not used by the engine')
            self.err |= 1 << 21
        self.procend = True
        self.cr &= ~2
        env.stats[f'pka_mode_{m:#x}'] += 1

    # curve arithmetic over python integers
    @staticmethod
    def add(P, Q, p, a):
        if P is None:
            return Q
        if Q is None:
            return P
        if P[0] == Q[0]:
            if (P[1] + Q[1]) % p == 0:
                return None
            lam = (3 * P[0] * P[0] + a) * pow(2 * P[1], -1, p) % p
        else:
            lam = (Q[1] - P[1]) * pow(Q[0] - P[0], -1, p) % p
        x = (lam * lam - P[0] - Q[0]) % p
        return (x, (lam * (P[0] - x) - P[1]) % p)

    @classmethod
    def mul(cls, k, P, p, a):
        R = None
        while k:
            if k & 1:
                R = cls.add(R, P, p, a)
            P = cls.add(P, P, p, a)
            k >>= 1
        return R

    def curve(self, a_sign, a_idx, p_idx, b_idx=None):
        p = self.num(p_idx)
        a_abs = self.num(a_idx)
        a = (-a_abs) % p if self.ram[a_sign] == 1 else a_abs
        b = self.num(b_idx) if b_idx is not None else None
        return p, a, b

    def on_curve(self, P, p, a, b):
        x, y = P
        return x < p and y < p and (y * y - (x * x * x + a * x + b)) % p == 0

    def do_mul(self):
        h = hdr
        env = self.env
        r = self.ram
        if r[h('PKA_ECC_SCALAR_MUL_IN_EXP_NB_BITS')] != 256 or r[h('PKA_ECC_SCALAR_MUL_IN_OP_NB_BITS')] != 256:
            env.v('PKA scalar multiplication bit counts are not 256')
        p, a, b = self.curve(h('PKA_ECC_SCALAR_MUL_IN_A_COEFF_SIGN'), h('PKA_ECC_SCALAR_MUL_IN_A_COEFF'),
                             h('PKA_ECC_SCALAR_MUL_IN_MOD_GF'), h('PKA_ECC_SCALAR_MUL_IN_B_COEFF'))
        k = self.num(h('PKA_ECC_SCALAR_MUL_IN_K'))
        P = (self.num(h('PKA_ECC_SCALAR_MUL_IN_INITIAL_POINT_X')), self.num(h('PKA_ECC_SCALAR_MUL_IN_INITIAL_POINT_Y')))
        n = self.num(h('PKA_ECC_SCALAR_MUL_IN_N_PRIME_ORDER'))
        if p != P256_P or b != P256_B or a != (-3) % p or n != P256_N:
            env.v('PKA scalar multiplication curve constants are not P-256')
        if not self.on_curve(P, p, a, b):
            env.v('PKA scalar multiplication on a point off the curve')
        env.stats['pka_mul'] += 1
        if k == 0 or k >= n:
            self.ram[h('PKA_ECC_SCALAR_MUL_OUT_ERROR')] = 0xCBC9
            return
        R = self.mul(k, P, p, a)
        if R is None:
            self.ram[h('PKA_ECC_SCALAR_MUL_OUT_ERROR')] = 0xCBC9
            return
        self.put(h('PKA_ECC_SCALAR_MUL_OUT_RESULT_X'), R[0])
        self.put(h('PKA_ECC_SCALAR_MUL_OUT_RESULT_Y'), R[1])
        self.ram[h('PKA_ECC_SCALAR_MUL_OUT_ERROR')] = 0xD60D

    def do_montgomery(self):
        h = hdr
        env = self.env
        r = self.ram
        if r[h('PKA_MONTGOMERY_PARAM_IN_MOD_NB_BITS')] != 256:
            env.v('PKA Montgomery parameter bit count is not 256')
        p = self.num(h('PKA_MONTGOMERY_PARAM_IN_MODULUS'))
        if p != P256_P:
            env.v('PKA Montgomery parameter modulus is not the P-256 prime')
        env.stats['pka_montgomery'] += 1
        res = pow(2, 512, p)
        for i in range(8):
            r[h('PKA_MONTGOMERY_PARAM_OUT_PARAMETER') + i] = (res >> (32 * i)) & 0xFFFFFFFF

    def do_check(self):
        h = hdr
        env = self.env
        r = self.ram
        if r[h('PKA_POINT_CHECK_IN_MOD_NB_BITS')] != 256:
            env.v('PKA point check bit count is not 256')
        p, a, b = self.curve(h('PKA_POINT_CHECK_IN_A_COEFF_SIGN'), h('PKA_POINT_CHECK_IN_A_COEFF'),
                             h('PKA_POINT_CHECK_IN_MOD_GF'), h('PKA_POINT_CHECK_IN_B_COEFF'))
        x = self.num(h('PKA_POINT_CHECK_IN_INITIAL_POINT_X'))
        y = self.num(h('PKA_POINT_CHECK_IN_INITIAL_POINT_Y'))
        r2 = self.num(h('PKA_POINT_CHECK_IN_MONTGOMERY_PARAM'))
        if p != P256_P or b != P256_B or a != (-3) % p:
            env.v('PKA point check curve constants are not P-256')
        if r2 != pow(2, 512, p):
            env.v('PKA point check Montgomery parameter is not R^2 mod p')
        if x >= p or y >= p:
            env.v('PKA point check coordinate not below the prime')
        ok = self.on_curve((x, y), p, a, b)
        r[h('PKA_POINT_CHECK_OUT_ERROR')] = 0xD60D if ok else 0xA3B7

    def do_sign(self):
        h = hdr
        env = self.env
        r = self.ram
        if r[h('PKA_ECDSA_SIGN_IN_ORDER_NB_BITS')] != 256 or r[h('PKA_ECDSA_SIGN_IN_MOD_NB_BITS')] != 256:
            env.v('PKA sign bit counts are not 256')
        p, a, b = self.curve(h('PKA_ECDSA_SIGN_IN_A_COEFF_SIGN'), h('PKA_ECDSA_SIGN_IN_A_COEFF'),
                             h('PKA_ECDSA_SIGN_IN_MOD_GF'), h('PKA_ECDSA_SIGN_IN_B_COEFF'))
        k = self.num(h('PKA_ECDSA_SIGN_IN_K'))
        G = (self.num(h('PKA_ECDSA_SIGN_IN_INITIAL_POINT_X')), self.num(h('PKA_ECDSA_SIGN_IN_INITIAL_POINT_Y')))
        e = self.num(h('PKA_ECDSA_SIGN_IN_HASH_E'))
        d = self.num(h('PKA_ECDSA_SIGN_IN_PRIVATE_KEY_D'))
        n = self.num(h('PKA_ECDSA_SIGN_IN_ORDER_N'))
        if p != P256_P or b != P256_B or a != (-3) % p or n != P256_N or G != (P256_GX, P256_GY):
            env.v('PKA sign curve constants are not P-256')
        if e >= n:
            env.v('PKA sign hash not reduced below the order')
        if not 0 < d < n:
            env.v('PKA sign private key out of range')
        err = h('PKA_ECDSA_SIGN_OUT_ERROR')
        if not 0 < k < n:
            r[err] = 0xCBC9
            return
        R = self.mul(k, G, p, a)
        rr = R[0] % n
        if rr == 0:
            r[err] = 0xA3B7
            return
        s = pow(k, -1, n) * (e + rr * d) % n
        if s == 0:
            r[err] = 0xF946
            return
        self.put(h('PKA_ECDSA_SIGN_OUT_SIGNATURE_R'), rr)
        self.put(h('PKA_ECDSA_SIGN_OUT_SIGNATURE_S'), s)
        r[err] = 0xD60D

    def do_verify(self):
        h = hdr
        env = self.env
        r = self.ram
        if r[h('PKA_ECDSA_VERIF_IN_ORDER_NB_BITS')] != 256 or r[h('PKA_ECDSA_VERIF_IN_MOD_NB_BITS')] != 256:
            env.v('PKA verify bit counts are not 256')
        p = self.num(h('PKA_ECDSA_VERIF_IN_MOD_GF'))
        a_abs = self.num(h('PKA_ECDSA_VERIF_IN_A_COEFF'))
        a = (-a_abs) % p if r[h('PKA_ECDSA_VERIF_IN_A_COEFF_SIGN')] == 1 else a_abs
        G = (self.num(h('PKA_ECDSA_VERIF_IN_INITIAL_POINT_X')), self.num(h('PKA_ECDSA_VERIF_IN_INITIAL_POINT_Y')))
        Q = (self.num(h('PKA_ECDSA_VERIF_IN_PUBLIC_KEY_POINT_X')), self.num(h('PKA_ECDSA_VERIF_IN_PUBLIC_KEY_POINT_Y')))
        rr = self.num(h('PKA_ECDSA_VERIF_IN_SIGNATURE_R'))
        s = self.num(h('PKA_ECDSA_VERIF_IN_SIGNATURE_S'))
        e = self.num(h('PKA_ECDSA_VERIF_IN_HASH_E'))
        n = self.num(h('PKA_ECDSA_VERIF_IN_ORDER_N'))
        if p != P256_P or a != (-3) % p or n != P256_N or G != (P256_GX, P256_GY):
            env.v('PKA verify curve constants are not P-256')
        if e >= n:
            env.v('PKA verify hash not reduced below the order')
        res = h('PKA_ECDSA_VERIF_OUT_RESULT')
        if not (0 < rr < n and 0 < s < n):
            r[res] = 0xA3B7
            return
        if not self.on_curve(Q, p, a, P256_B):
            r[res] = 0xCBC9
            return
        w = pow(s, -1, n)
        pt = self.add(self.mul(e * w % n, G, p, a), self.mul(rr * w % n, Q, p, a), p, a)
        r[res] = 0xD60D if pt is not None and pt[0] % n == rr else 0xCBC9


P256_P = 0xFFFFFFFF00000001000000000000000000000000FFFFFFFFFFFFFFFFFFFFFFFF
P256_B = 0x5AC635D8AA3A93E7B3EBBD55769886BC651D06B0CC53B0F63BCE3C3E27D2604B
P256_N = 0xFFFFFFFF00000000FFFFFFFFFFFFFFFFBCE6FAADA7179E84F3B9CAC2FC632551
P256_GX = 0x6B17D1F2E12C4247F8BCE6E563A440F277037D812DEB33A0F4A13945D898C296
P256_GY = 0x4FE342E2FE1A7F9B8EE7EB4A7C0F9E162BCE33576B315ECECBB6406837BF51F5


def model_self_check():
    """The curve code of the model against the cryptography package."""
    from cryptography.hazmat.primitives.asymmetric import ec
    a = (-3) % P256_P
    for k in (3, 0xDEADBEEF, P256_N - 1, 1 << 200):
        pt = Pka.mul(k, (P256_GX, P256_GY), P256_P, a)
        num = ec.derive_private_key(k, ec.SECP256R1()).public_key().public_numbers()
        assert pt == (num.x, num.y), f'model scalar multiplication differs for {k:#x}'
    key = bytes(range(16))
    rk = last_round_key(key)
    assert key_from_last_round_key(rk) == key, 'model key schedule inversion'
    from cryptography.hazmat.primitives.ciphers import Cipher, algorithms, modes
    e = Cipher(algorithms.AES(bytes.fromhex('000102030405060708090a0b0c0d0e0f')), modes.ECB()).encryptor()
    assert e.update(bytes.fromhex('00112233445566778899aabbccddeeff')).hex() == '69c4e0d86a7b0430d8cdb78070b4c55a'
    assert rk.hex() == '13111d7fe3944a17f307a78b4d2b30c5', 'model last round key'


# ---------------------------------------------------------------------------
# emulator

def run_fw(elf_path, seed, ctr_bits, rng_fault):
    from unicorn import Uc, UC_ARCH_ARM, UC_MODE_THUMB, UC_MODE_MCLASS, UC_HOOK_CODE
    from unicorn.arm_const import UC_CPU_ARM_CORTEX_M33
    from unicorn.arm_const import UC_ARM_REG_R0, UC_ARM_REG_R1, UC_ARM_REG_SP, UC_ARM_REG_PC
    from elftools.elf.elffile import ELFFile

    elf = ELFFile(open(elf_path, 'rb'))
    syms = {s.name: s['st_value'] for s in elf.get_section_by_name('.symtab').iter_symbols()}

    uc = Uc(UC_ARCH_ARM, UC_MODE_THUMB | UC_MODE_MCLASS)
    uc.ctl_set_cpu_model(UC_CPU_ARM_CORTEX_M33)
    uc.mem_map(0x08000000, 0x200000)
    uc.mem_map(0x20000000, 0x80000)
    uc.mem_map(0xE0000000, 0x100000)
    for seg in elf.iter_segments():
        if seg['p_type'] == 'PT_LOAD' and seg['p_filesz']:
            uc.mem_write(seg['p_paddr'], seg.data())

    env = Env(seed, ctr_bits, rng_fault)
    rcc = Rcc(env)
    aes = Aes(env, rcc)
    rng = Rng(env, rcc)
    pka = Pka(env, rcc)

    # The device header selects the secure or the non-secure alias of each
    # block by the build; both views are mapped to the same model.
    views = []
    for view in ('S', 'NS'):
        views.append((hdr(f'RCC_BASE_{view}'), hdr(f'AES_BASE_{view}'), hdr(f'RNG_BASE_{view}'),
                      hdr(f'PKA_BASE_{view}')))
    plain = collections.defaultdict(int)

    def dispatch(a):
        for rcc_base, aes_base, rng_base, pka_base in views:
            if rcc_base <= a < rcc_base + 0x400:
                return rcc, a - rcc_base
            if aes_base <= a < aes_base + 0x400:
                return aes, a - aes_base
            if rng_base <= a < rng_base + 0x400:
                return rng, a - rng_base
            if pka_base <= a < pka_base + 0x2000:
                return pka, a - pka_base
        return None, 0

    def rd(uc, off, size, base):
        a = base + off
        dev, o = dispatch(a)
        if dev is None:
            return plain[a]
        if size != 4:
            env.v(f'{type(dev).__name__} access of {size} bytes at {o:#x}')
        return dev.rd(o) & 0xFFFFFFFF

    def wr(uc, off, size, val, base):
        a = base + off
        dev, o = dispatch(a)
        if dev is None:
            plain[a] = val
            return
        if size != 4:
            env.v(f'{type(dev).__name__} access of {size} bytes at {o:#x}')
        dev.wr(o, val & 0xFFFFFFFF)

    for rcc_base, aes_base, rng_base, pka_base in views:
        for b, size in ((aes_base & ~0xFFF, 0x1000), (pka_base, 0x2000), (rcc_base & ~0xFFF, 0x1000)):
            uc.mmio_map(b, size, rd, b, wr, b)

    console = bytearray()

    def on_console(uc, addr, size, user):
        p = uc.reg_read(UC_ARM_REG_R0)
        n = uc.reg_read(UC_ARM_REG_R1)
        console.extend(bytes(uc.mem_read(p, n)))

    state = {'done': False}

    def on_done(uc, addr, size, user):
        state['done'] = True
        uc.emu_stop()

    for name, cb in (('CryptoFwConsole', on_console), ('CryptoFwDone', on_done)):
        a = syms[name] & ~1
        uc.hook_add(UC_HOOK_CODE, cb, begin=a, end=a)

    vec = bytes(uc.mem_read(0x08000000, 8))
    uc.reg_write(UC_ARM_REG_SP, int.from_bytes(vec[0:4], 'little'))
    pc = int.from_bytes(vec[4:8], 'little')
    try:
        uc.emu_start(pc, 0xFFFFFFFF, count=4_000_000_000)
    except Exception as ex:
        text = console.decode('ascii', 'replace')
        raise SystemExit(f'FAIL emulator stopped at {uc.reg_read(UC_ARM_REG_PC):#x}: {ex}\n{text[-1500:]}')
    if not state['done']:
        raise SystemExit('FAIL firmware did not finish\n' + console.decode('ascii', 'replace')[-1500:])

    result = int.from_bytes(uc.mem_read(syms['g_CryptoFwResult'], 4), 'little', signed=True)
    # Clean-up state at the end
    left = []
    if any(pka.ram):
        left.append('PKA RAM not cleared')
    if any(aes.key) or any(aes.iv):
        left.append('AES key or IV registers not cleared')
    if aes.cr & 1:
        left.append('AES left enabled')
    return result, console.decode('ascii', 'replace'), env, left


def main():
    arm = shutil.which('arm-none-eabi-g++')
    try:
        import unicorn  # noqa: F401
        import elftools  # noqa: F401
        import cryptography  # noqa: F401
    except ImportError as e:
        print(f'NOT RUN: {e}. pip install unicorn pyelftools cryptography')
        return
    if not arm:
        print('NOT RUN: arm-none-eabi-g++ not found')
        return

    model_self_check()
    errors = []
    total_checks = 0
    with tempfile.TemporaryDirectory(prefix='wba-crypto-') as tmp:
        elf = build(Path(tmp))
        # Counter increment of the peripheral: 32 bits, then 128 bits. The
        # seed error of the RNG is injected in the first run.
        for seed, ctr_bits, fault in ((1, 32, True), (2, 128, False)):
            result, text, env, left = run_fw(elf, seed, ctr_bits, fault)
            passed = text.count('[PASS]')
            failed = text.count('[FAIL]')
            total_checks += passed + failed
            print(f'run seed {seed}, AES counter {ctr_bits} bits: {passed} passed, {failed} failed, '
                  f'result {result}')
            for line in text.splitlines():
                if '[FAIL]' in line:
                    errors.append(f'seed {seed}: {line.strip()}')
            if result != 0:
                errors.append(f'seed {seed}: the test returned {result}')
            for want in ('pka point check 1 err d60d, constant 1 err d60d',
                         'pka mul 1, verify 1, R2 same 1',
                         'pka R2 00000003 00000000 ffffffff fffffffb fffffffe ffffffff fffffffd 00000004'):
                if want not in text:
                    errors.append(f'seed {seed}: Diag line missing: {want}')
            if 'engines construct' not in text:
                errors.append(f'seed {seed}: no test output')
            for v in env.viol[:10]:
                errors.append(f'seed {seed}: model violation: {v}')
            if len(env.viol) > 10:
                errors.append(f'seed {seed}: {len(env.viol)} model violations in all')
            for m in left:
                errors.append(f'seed {seed}: {m}')
            if fault and env.stats['rng_seis'] == 0:
                errors.append(f'seed {seed}: the seed error was never injected')
            print('  model counters:', dict(sorted(env.stats.items())))
            if '-v' in sys.argv:
                print(text)

    for m in errors:
        print('FAIL:', m)
    if errors:
        raise SystemExit(f'STM32WBA65 crypto: {len(errors)} problems')
    print(f'STM32WBA65 crypto engines on the emulator: {total_checks} checks, 0 failures')
    print('Hardware not tested.')


if __name__ == '__main__':
    main()
