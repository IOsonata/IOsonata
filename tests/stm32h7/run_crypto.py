#!/usr/bin/env python3
"""Execute H753 RNG failure paths and crypto vectors with real Arm instructions.

The register model supplies deterministic test words, not physical entropy.
Requires GNU Arm GCC, unicorn and pyelftools.

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
from pathlib import Path
import struct
import subprocess
import tempfile
from elftools.elf.elffile import ELFFile
from unicorn import Uc, UC_ARCH_ARM, UC_MODE_THUMB, UC_MODE_MCLASS, UC_HOOK_MEM_READ, UC_HOOK_MEM_WRITE
from unicorn.arm_const import UC_ARM_REG_SP, UC_ARM_REG_LR, UC_ARM_REG_R0, UC_ARM_REG_PC

ROOT = Path(__file__).resolve().parents[2]
SOURCES = ('ARM/ST/src/rng_stm32.cpp', 'src/crypto/crypto_p256.cpp',
           'src/crypto/crypto_softaes.cpp', 'src/crypto/crypto_softsha256.cpp',
           'src/crypto/crypto_uecc.cpp', 'micro-ecc/uECC.c', 'src/device.cpp',
           'src/device_intrf.cpp', 'src/CppRuntimeOverload.cpp', 'tests/stm32h7/crypto_fw.cpp')


def build(out, opt):
    flags = ['-mcpu=cortex-m7', '-mthumb', '-DSTM32H753xx', opt, '-g',
             '-DuECC_ENABLE_VLI_API=1', '-ffunction-sections', '-fdata-sections']
    flags += ['-I' + str(ROOT / p) for p in ('include', 'ARM/include',
              'ARM/CMSIS/Core/Include', 'ARM/ST/include', 'ARM/ST/STM32H7xx/include', 'micro-ecc')]
    objects = []
    for src in SOURCES:
        obj = out / (Path(src).stem + '.o')
        cpp = src.endswith('.cpp')
        language = ['-std=gnu++23', '-fno-exceptions', '-fno-rtti'] if cpp else ['-std=gnu17']
        subprocess.run(['arm-none-eabi-g++' if cpp else 'arm-none-eabi-gcc', *flags,
                        *language, '-c', str(ROOT / src), '-o', str(obj)], check=True)
        objects.append(str(obj))
    script = out / 'test.ld'
    # GCC 15 turns on the libstdc++ assertions at -O0. Link with g++ for the
    # assertion handler, and give the newlib sbrk its heap start, end, since
    # the handler failure path links abort and malloc.
    script.write_text('MEMORY { FLASH (rx) : ORIGIN = 0x08000000, LENGTH = 2M\n'
                      'RAM (rwx) : ORIGIN = 0x20000000, LENGTH = 128K }\n'
                      'SECTIONS { .text : { *(.text*) *(.rodata*) } > FLASH\n'
                      '.data : { *(.data*) } > RAM\n .bss : { *(.bss*) *(COMMON) } > RAM\n'
                      'end = .;\n'
                      '/DISCARD/ : { *(.ARM.exidx*) } }\n')
    elf = out / 'crypto.elf'
    subprocess.run(['arm-none-eabi-g++', '-mcpu=cortex-m7', '-mthumb', '-nostartfiles',
                    '-specs=nano.specs', '-specs=nosys.specs', '-T' + str(script),
                    '-Wl,--gc-sections,-e,RunRngTest,-u,RunCryptoVectors', *objects,
                    '-o', str(elf)], check=True)
    return elf


class Model:
    def __init__(self, path):
        self.u = Uc(UC_ARCH_ARM, UC_MODE_THUMB | UC_MODE_MCLASS)
        for addr, size in ((0x08000000, 0x200000), (0x20000000, 0x20000),
                           (0x40000000, 0x10000000), (0x58000000, 0x100000),
                           (0xe0000000, 0x100000)):
            self.u.mem_map(addr, size)
        with path.open('rb') as f:
            e = ELFFile(f)
            for seg in e.iter_segments():
                if seg['p_type'] == 'PT_LOAD' and seg['p_filesz']:
                    self.u.mem_write(seg['p_vaddr'], seg.data())
            self.syms = {s.name: s['st_value'] for s in e.get_section_by_name('.symtab').iter_symbols()}
        self.words = 0
        self.put(0x58024454, 0xA5300300)  # unrelated clock mux fields must survive
        self.u.hook_add(UC_HOOK_MEM_READ, self.read)
        self.u.hook_add(UC_HOOK_MEM_WRITE, self.write)

    def get(self, addr):
        return struct.unpack('<I', self.u.mem_read(addr, 4))[0]

    def put(self, addr, value):
        self.u.mem_write(addr, struct.pack('<I', value))

    def read(self, uc, access, addr, size, value, ctx):
        fault = self.get(self.syms['g_RngFault'])
        if addr == 0x58024400:
            cr = self.get(addr)
            self.put(addr, cr | (1 << 13) if cr & (1 << 12) and fault != 1 else cr & ~(1 << 13))
        elif addr == 0x48021804:
            errors = {2: 2, 3: 4, 4: 32, 5: 64}
            sr = 0 if fault == 6 else 1 | errors.get(fault, 0)
            if fault == 7 and self.words >= 2:
                sr |= 64
            self.put(addr, sr)
        elif addr == 0x48021808:
            self.words += 1
            self.put(addr, (self.words * 0x9E3779B9) & 0xFFFFFFFF)

    def write(self, uc, access, addr, size, value, ctx):
        if addr == 0x58024484 and value & (1 << 6):
            self.put(0x48021800, 0)  # peripheral reset

    def run(self, name, arg=0):
        self.u.reg_write(UC_ARM_REG_SP, 0x2001f000)
        self.u.reg_write(UC_ARM_REG_LR, 0x080ff001)
        self.u.reg_write(UC_ARM_REG_R0, arg)
        self.u.emu_start(self.syms[name] | 1, 0x080ff000, count=150000000)
        assert self.u.reg_read(UC_ARM_REG_PC) == 0x080ff000, name + ' did not return'
        result = self.u.reg_read(UC_ARM_REG_R0)
        assert result == 0, f'{name} case {arg}: firmware check line {result}'


def main():
    with tempfile.TemporaryDirectory(prefix='h7-crypto-') as td:
        for opt in ('-O0', '-Os'):
            elf = build(Path(td), opt)
            for case in range(8):
                model = Model(elf)
                model.run('RunRngTest', case)
                print(f'{opt} RNG case {case}: PASS', flush=True)
            Model(elf).run('RunCryptoVectors')
            print(opt + ' AES/CMAC, SHA-256/HMAC, P-256 vectors: PASS', flush=True)
    print('H753 crypto register/instruction tests PASS; physical entropy untested.')


if __name__ == '__main__':
    main()
