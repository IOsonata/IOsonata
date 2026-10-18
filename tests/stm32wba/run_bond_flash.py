#!/usr/bin/env python3
"""STM32WBA bond store over the flash controller model, on an emulator.

Run: python3 tests/stm32wba/run_bond_flash.py

Needs arm-none-eabi-g++, and the python packages unicorn and pyelftools.
Without them the test prints NOT RUN.

bond_flash_fw.cpp is built for the Cortex-M33 with the real bond store
(ARM/ST/STM32WBAxx/src/bt_smp_bond_stm32wba.cpp), the real PDS store
(src/bluetooth/bt_pds.cpp, ARM/ST/STM32WBAxx/src/bt_pds_stm32wba.cpp), the
real flash driver (ARM/ST/src/nvm_stm32.cpp) and the NVM region and Nvm
code, and run in unicorn. The flash controller is a model written from the
reference manual and the ST HAL (stm32wbaxx_hal_flash.c):

- NSCR1 locked until the two key writes, a wrong key is a violation;
- a quad word is programmed when its fourth 32 bit word is written with PG
  set, aligned on 16 bytes; a write without PG, of another size or out of
  order is a violation;
- a quad word that is not erased cannot be programmed (PROGERR);
- page erase by bank and page number with PER and STRT;
- writes and erases of a protected page are refused with WRPERR. The test
  turns this protection on and off where a board would need an option byte
  change and a reset, so a retry can be watched succeeding mid wait.

The flash content is kept between the boots of the firmware, so a boot is a
reset. The one-second tick of the application is a call of
BtSmpBondWbaPoll; the emulator counts those calls and checks each retry line
against that count: tries after 1, 2, 4 .. 64 s, the save dropped after the
64 s wait (127 polls in total), and no retry from idle-loop calls while
that wait is active. The initial save still runs from the application loop.
A second image is linked without the bond store region and checks that a missing
region is reported and never retried.

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
import re
from pathlib import Path
import shutil
import sys
import tempfile

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
import run_clock as rc  # noqa: E402  header helpers and the build runner

ROOT = rc.ROOT
hdr = rc.hdr
reg_off = rc.reg_off

# Layout of the WBA65 2 MB part, as gcc_stm32wba65_i.ld places the store
FLASH_BASE = 0x08000000
FLASH_KB = 2048
NVM_BASE = 0x081F8000
NVM_SIZE = 0x4000
NVM_MAP = 0x8000			# NVM0 and NVM1, the top 32 KB
PAGE = 0x2000
QWORD = 16

LD = '''/* Link layout of the STM32WBA bond flash emulator test firmware */
MEMORY
{
	FLASH (rx) : ORIGIN = 0x08000000, LENGTH = 0x1F8000
	NVM0 (rw) : ORIGIN = 0x081F8000, LENGTH = 0x4000
	RAM (rwx) : ORIGIN = 0x20000000, LENGTH = 0x80000
}

ENTRY(ResetHandler)

SECTIONS
{
	.isr_vector : { KEEP(*(.isr_vector)) } > FLASH
	.text : { *(.text*) *(.rodata*) *(.init) *(.fini) . = ALIGN(4); } > FLASH
	.ARM.exidx : { *(.ARM.exidx*) } > FLASH
	.init_array :
	{
		__init_array_start = .;
		KEEP(*(SORT(.init_array.*)))
		KEEP(*(.init_array))
		__init_array_end = .;
	} > FLASH
%(nvm)s	_sidata = LOADADDR(.data);
	.data : { _sdata = .; *(.data*) . = ALIGN(4); _edata = .; } > RAM AT > FLASH
	.bss : { _sbss = .; *(.bss*) *(COMMON) . = ALIGN(4); _ebss = .; } > RAM
	.noinit (NOLOAD) : { *(.noinit*) } > RAM
	end = .;
	_estack = ORIGIN(RAM) + LENGTH(RAM);
}
'''

LD_NVM = '''	nvm0 (NOLOAD) :
	{
		__start_nvm0 = .;
		KEEP(*(nvm0))
		. = ORIGIN(NVM0) + LENGTH(NVM0);
		__stop_nvm0 = .;
	} > NVM0
'''

BOOTS = 5				# boots of the image with the store region
BOOT_NO_REGION = 5		# boot of the image without it


def build(tmp, with_region):
    name = 'bond_flash_fw' if with_region else 'bond_flash_noregion_fw'
    out = tmp / name
    out.mkdir()
    ld = out / 'fw.ld'
    ld.write_text(LD % {'nvm': LD_NVM if with_region else ''})
    cpu = ['-mcpu=cortex-m33', '-mthumb', '-mfloat-abi=soft', '-Os', '-DSTM32WBA65xx', '-DIOSONATA_STM32_WBA',
           '-ffunction-sections', '-fdata-sections', '-g']
    inc = [ROOT / 'include', ROOT / 'ARM/include', ROOT / 'ARM/CMSIS/Core/Include', ROOT / 'ARM/ST/include',
           ROOT / 'ARM/ST/STM32WBAxx/include', ROOT / 'ARM/ST/STM32WBAxx/STM32WBA65/include']
    inc = ['-I' + str(p) for p in inc]
    # Warnings are errors in the test firmware only; the library sources are
    # built as the library project builds them.
    cxx = ['arm-none-eabi-g++', *cpu, '-std=gnu++23', '-Wall', '-fno-exceptions', '-fno-rtti', *inc]
    fw = out / 'bond_flash_fw.o'
    rc.run([*cxx, '-Werror', '-c', HERE / 'bond_flash_fw.cpp', '-o', fw])
    srcs = [
        ROOT / 'ARM/ST/STM32WBAxx/src/bt_smp_bond_stm32wba.cpp',
        ROOT / 'ARM/ST/STM32WBAxx/src/bt_pds_stm32wba.cpp',
        ROOT / 'src/bluetooth/bt_pds.cpp',
        ROOT / 'ARM/ST/src/nvm_stm32.cpp',
        ROOT / 'src/storage/nvm.cpp',
        ROOT / 'src/storage/nvm_region.cpp',
        ROOT / 'src/device.cpp',
        ROOT / 'src/device_intrf.cpp',
    ]
    objs = [fw]
    for src in srcs:
        obj = out / (src.stem + '.o')
        rc.run([*cxx, '-c', src, '-o', obj])
        objs.append(obj)
    obj = out / 'crc.o'
    rc.run(['arm-none-eabi-gcc', *cpu, '-std=gnu17', *inc, '-c', ROOT / 'src/crc.c', '-o', obj])
    objs.append(obj)
    elf = out / (name + '.elf')
    rc.run(['arm-none-eabi-g++', *cpu, '-nostartfiles', '-Wl,--gc-sections', '-Wl,-T,' + str(ld),
            '--specs=nosys.specs', '--specs=nano.specs', *objs, '-o', elf])
    return elf


class Flash:
    """STM32WBA flash controller, non-secure registers, and the memory of
    the bond store region."""

    def __init__(self, mem):
        self.mem = mem			# bytearray of NVM_MAP bytes from NVM_BASE - kept across boots
        self.off_keyr = reg_off('FLASH_TypeDef', 'NSKEYR')
        self.off_sr = reg_off('FLASH_TypeDef', 'NSSR')
        self.off_cr = reg_off('FLASH_TypeDef', 'NSCR1')
        self.PG = hdr('FLASH_NSCR1_PG')
        self.PER = hdr('FLASH_NSCR1_PER')
        self.MER = hdr('FLASH_NSCR1_MER1') | hdr('FLASH_NSCR1_MER2')
        self.PNB_POS = hdr('FLASH_NSCR1_PNB_Pos')
        self.PNB_MSK = hdr('FLASH_NSCR1_PNB_Msk')
        self.BKER = hdr('FLASH_NSCR1_BKER')
        self.STRT = hdr('FLASH_NSCR1_STRT')
        self.EOPIE = hdr('FLASH_NSCR1_EOPIE')
        self.LOCK = hdr('FLASH_NSCR1_LOCK')
        self.EOP = hdr('FLASH_NSSR_EOP')
        self.PROGERR = hdr('FLASH_NSSR_PROGERR')
        self.WRPERR = hdr('FLASH_NSSR_WRPERR')
        self.PGAERR = hdr('FLASH_NSSR_PGAERR')
        self.PGSERR = hdr('FLASH_NSSR_PGSERR')
        self.ERR = (hdr('FLASH_NSSR_OPERR') | self.PROGERR | self.WRPERR | self.PGAERR |
                    hdr('FLASH_NSSR_SIZERR') | self.PGSERR | hdr('FLASH_NSSR_OPTWERR'))
        self.cr = self.LOCK
        self.sr = 0
        self.key = 0
        self.qw = []			# words of the quad word being written: (addr, value)
        self.prot = False		# bond store region refuses writes and erases
        self.arm = 0			# BtPdsWrite calls left before prot turns on, 0 none
        self.viol = []
        self.stats = {'programs': 0, 'erases': 0, 'wrperr': 0, 'progerr': 0}

    def v(self, msg):
        if msg not in self.viol:
            self.viol.append(msg)

    def protected(self, addr):
        return self.prot and NVM_BASE <= addr < NVM_BASE + NVM_SIZE

    def done(self, err):
        if err:
            self.sr |= err
        elif self.cr & self.EOPIE:
            self.sr |= self.EOP

    # registers
    def rd(self, off):
        if off == self.off_cr:
            return self.cr
        if off == self.off_sr:
            return self.sr			# BSY and WDW never set: the model is instant
        return 0

    def wr(self, off, val):
        if off == self.off_keyr:
            if self.key == 0 and val == 0x45670123:
                self.key = 1
            elif self.key == 1 and val == 0xCDEF89AB:
                self.key = 0
                self.cr &= ~self.LOCK
            else:
                self.v(f'wrong flash key {val:#x}')
                self.key = 0
            return
        if off == self.off_sr:
            self.sr &= ~(val & (self.ERR | self.EOP))
            return
        if off == self.off_cr:
            if self.cr & self.LOCK:
                self.v('NSCR1 written while locked')
                return
            if (val & self.PG) and (val & (self.PER | self.MER)):
                self.v('PG set together with an erase')
            if not (val & self.PG) and self.qw:
                self.v('PG cleared in the middle of a quad word')
                self.qw = []
            self.cr = val & ~self.STRT
            if val & self.STRT:
                if val & self.MER:
                    self.v('mass erase')
                elif val & self.PER:
                    self.erase(val)
                else:
                    self.v('STRT without an operation')
            return
        self.v(f'write of flash register {off:#x}')

    def erase(self, cr):
        if self.sr & self.ERR:
            self.v('erase started with error flags still set')
        page = (cr & self.PNB_MSK) >> self.PNB_POS
        bank = 1 if cr & self.BKER else 0
        addr = FLASH_BASE + bank * (FLASH_KB * 1024 // 2) + page * PAGE
        if not (NVM_BASE <= addr < NVM_BASE + NVM_MAP):
            self.v(f'erase of page {addr:#x} outside the store')
            self.done(self.WRPERR)
            return
        if self.protected(addr):
            self.stats['wrperr'] += 1
            self.done(self.WRPERR)
            return
        o = addr - NVM_BASE
        self.mem[o:o + PAGE] = b'\xff' * PAGE
        self.stats['erases'] += 1
        self.done(0)

    # memory of the store region
    def mem_rd(self, off, size):
        return int.from_bytes(self.mem[off:off + size], 'little')

    def mem_wr(self, off, size, val):
        addr = NVM_BASE + off
        if not (self.cr & self.PG):
            self.v(f'flash write at {addr:#x} without PG')
            self.done(self.PGSERR)
            return
        if size != 4:
            self.v(f'flash write of {size} bytes at {addr:#x}')
            self.done(self.PGSERR)
            return
        if not self.qw:
            if addr % QWORD:
                self.v(f'quad word start not aligned at {addr:#x}')
                self.done(self.PGAERR)
                return
        elif addr != self.qw[-1][0] + 4:
            self.v(f'quad word written out of order at {addr:#x}')
            self.qw = []
            self.done(self.PGSERR)
            return
        self.qw.append((addr, val & 0xFFFFFFFF))
        if len(self.qw) < QWORD // 4:
            return
        base = self.qw[0][0]
        data = b''.join(w.to_bytes(4, 'little') for _, w in self.qw)
        self.qw = []
        if self.protected(base):
            self.stats['wrperr'] += 1
            self.done(self.WRPERR)
            return
        o = base - NVM_BASE
        if self.mem[o:o + QWORD] != b'\xff' * QWORD:
            self.stats['progerr'] += 1
            self.done(self.PROGERR)
            return
        self.mem[o:o + QWORD] = data
        self.stats['programs'] += 1
        self.done(0)


def run_boot(elf_path, boot, mem):
    from unicorn import Uc, UC_ARCH_ARM, UC_MODE_THUMB, UC_MODE_MCLASS, UC_HOOK_CODE
    from unicorn.arm_const import UC_CPU_ARM_CORTEX_M33
    from unicorn.arm_const import UC_ARM_REG_R0, UC_ARM_REG_R1, UC_ARM_REG_SP, UC_ARM_REG_PC
    from elftools.elf.elffile import ELFFile

    elf = ELFFile(open(elf_path, 'rb'))
    syms = {s.name: s['st_value'] for s in elf.get_section_by_name('.symtab').iter_symbols()}

    uc = Uc(UC_ARCH_ARM, UC_MODE_THUMB | UC_MODE_MCLASS)
    uc.ctl_set_cpu_model(UC_CPU_ARM_CORTEX_M33)
    uc.mem_map(FLASH_BASE, NVM_BASE - FLASH_BASE)
    uc.mem_map(0x20000000, 0x80000)
    uc.mem_map(0xE0000000, 0x100000)
    for seg in elf.iter_segments():
        if seg['p_type'] == 'PT_LOAD' and seg['p_filesz']:
            uc.mem_write(seg['p_paddr'], seg.data())

    flash = Flash(mem)

    def mem_rd(uc, off, size, user):
        return flash.mem_rd(off, size)

    def mem_wr(uc, off, size, val, user):
        flash.mem_wr(off, size, val)

    uc.mmio_map(NVM_BASE, NVM_MAP, mem_rd, None, mem_wr, None)

    # Both views of the controller go to the same model. The map starts on
    # the page holding the block, user data is the block address.
    def reg_rd(uc, off, size, base):
        reg = (base & ~0xFFF) + off - base
        if size != 4:
            flash.v(f'flash register access of {size} bytes at {reg:#x}')
        return flash.rd(reg) & 0xFFFFFFFF

    def reg_wr(uc, off, size, val, base):
        reg = (base & ~0xFFF) + off - base
        if size != 4:
            flash.v(f'flash register access of {size} bytes at {reg:#x}')
        flash.wr(reg, val & 0xFFFFFFFF)

    for view in ('FLASH_R_BASE_NS', 'FLASH_R_BASE_S'):
        base = hdr(view)
        uc.mmio_map(base & ~0xFFF, 0x1000, reg_rd, base, reg_wr, base)

    size_reg = hdr('FLASHSIZE_BASE')

    def size_rd(uc, off, size, user):
        return FLASH_KB if (size_reg & ~0xFFF) + off == size_reg else 0

    uc.mmio_map(size_reg & ~0xFFF, 0x1000, size_rd, None, lambda *a: flash.v('write of the flash size'), None)

    st = {'poll': 0, 'done': False, 'init': 0, 'lines': [], 'buf': ''}

    def on_console(uc, addr, size, user):
        p = uc.reg_read(UC_ARM_REG_R0)
        n = uc.reg_read(UC_ARM_REG_R1)
        st['buf'] += bytes(uc.mem_read(p, n)).decode('ascii', 'replace')
        while '\n' in st['buf']:
            line, st['buf'] = st['buf'].split('\n', 1)
            st['lines'].append((st['poll'], line.rstrip('\r')))

    def on_protect(uc, addr, size, user):
        mode = uc.reg_read(UC_ARM_REG_R0)
        mode = mode - (1 << 32) if mode & 0x80000000 else mode
        flash.prot = mode == 0
        flash.arm = mode if mode > 0 else 0

    def on_pds_write(uc, addr, size, user):
        if flash.arm:
            flash.arm -= 1
            if flash.arm == 0:
                flash.prot = True

    def on_poll(uc, addr, size, user):
        st['poll'] += 1

    def on_init(uc, addr, size, user):
        st['init'] += 1

    def on_done(uc, addr, size, user):
        st['done'] = True
        uc.emu_stop()

    for name, cb in (('BondFwConsole', on_console), ('BondFwProtect', on_protect), ('BondFwDone', on_done),
                     ('BtPdsWrite', on_pds_write), ('BtSmpBondWbaPoll', on_poll), ('BtPdsWbaInit', on_init)):
        if name not in syms:
            raise SystemExit(f'FAIL symbol {name} not in the firmware')
        a = syms[name] & ~1
        uc.hook_add(UC_HOOK_CODE, cb, begin=a, end=a)

    from unicorn import UC_HOOK_MEM_UNMAPPED
    bad = []

    def on_unmapped(uc, access, addr, size, val, user):
        pc = uc.reg_read(UC_ARM_REG_PC)
        fn = max(((a & ~1, n) for n, a in syms.items() if (a & ~1) <= pc and n), default=(0, '?'))[1]
        bad.append(f'unmapped access at {addr:#x} ({size} bytes) from {fn}')
        return False

    uc.hook_add(UC_HOOK_MEM_UNMAPPED, on_unmapped)
    uc.mem_write(syms['g_BondFwBoot'], boot.to_bytes(4, 'little'))
    vec = bytes(uc.mem_read(FLASH_BASE, 8))
    uc.reg_write(UC_ARM_REG_SP, int.from_bytes(vec[0:4], 'little'))
    pc = int.from_bytes(vec[4:8], 'little')
    try:
        uc.emu_start(pc, 0xFFFFFFFF, count=2_000_000_000)
    except Exception as ex:
        text = '\n'.join(f'[{p}] {line}' for p, line in st['lines'])
        raise SystemExit(f'FAIL boot {boot}: emulator stopped at {uc.reg_read(UC_ARM_REG_PC):#x}: {ex} {bad}\n'
                         f'{text[-2000:]}')
    if not st['done']:
        raise SystemExit(f'FAIL boot {boot}: firmware did not finish')
    result = int.from_bytes(uc.mem_read(syms['g_BondFwResult'], 4), 'little', signed=True)
    return result, st, flash


RETRY = re.compile(r'WBA bond: save failed \((-?\d+)\), retry in (\d+) s')
DROP = re.compile(r'WBA bond: save dropped \((-?\d+)\)')
SAVED = re.compile(r'WBA bond: saved (\d+) bytes')
RESTORED = re.compile(r'WBA bond: restored (\d+) bytes')


def events(st):
    """Retry, drop and saved lines with the poll count when printed."""
    ev = []
    for poll, line in st['lines']:
        m = RETRY.search(line)
        if m:
            ev.append((poll, 'retry', int(m.group(2)), int(m.group(1))))
            continue
        m = DROP.search(line)
        if m:
            ev.append((poll, 'drop', 0, int(m.group(1))))
            continue
        m = SAVED.search(line)
        if m:
            ev.append((poll, 'saved', int(m.group(1)), 0))
    return ev


def expect_events(errors, boot, got, want):
    got = [e[:3] for e in got]
    if got != want:
        errors.append(f'boot {boot}: retry events {got}, expected {want}')


def main():
    if not shutil.which('arm-none-eabi-g++'):
        print('NOT RUN: arm-none-eabi-g++ not found')
        return
    try:
        import unicorn  # noqa: F401
        import elftools  # noqa: F401
    except ImportError as e:
        print(f'NOT RUN: {e}. pip install unicorn pyelftools')
        return

    errors = []
    passed = failed = 0
    totals = {'programs': 0, 'erases': 0, 'wrperr': 0, 'progerr': 0}
    with tempfile.TemporaryDirectory(prefix='wba-bond-flash-') as td:
        tmp = Path(td)
        elf = build(tmp, True)
        elf_none = build(tmp, False)

        mem = bytearray(b'\xff' * NVM_MAP)
        runs = [(elf, b) for b in range(BOOTS)] + [(elf_none, BOOT_NO_REGION)]
        for path, boot in runs:
            if path is elf_none:
                mem = bytearray(b'\xff' * NVM_MAP)
            result, st, flash = run_boot(path, boot, mem)
            text = [line for _, line in st['lines']]
            passed += sum('[PASS]' in t for t in text)
            failed += sum('[FAIL]' in t for t in text)
            for t in text:
                if '[FAIL]' in t:
                    errors.append(f'boot {boot}: {t}')
            if result != 0:
                errors.append(f'boot {boot}: the firmware counted {result} failures')
            for v in flash.viol:
                errors.append(f'boot {boot}: model violation: {v}')
            for k in totals:
                totals[k] += flash.stats[k]
            ev = events(st)
            if '-v' in sys.argv:
                print(f'--- boot {boot}')
                for p, line in st['lines']:
                    print(f'  [{p}] {line}')

            if boot == 0:
                expect_events(errors, boot, ev, [(0, 'saved', 312)])
                if not any(RESTORED.search(t) and RESTORED.search(t).group(1) == '0' for t in text):
                    errors.append('boot 0: no "restored 0 bytes" line')
            elif boot == 1:
                want = [(0, 'retry', 1), (1, 'retry', 2), (3, 'retry', 4), (7, 'retry', 8),
                        (15, 'retry', 16), (31, 'retry', 32), (63, 'retry', 64), (127, 'drop', 0),
                        (150, 'saved', 312)]
                expect_events(errors, boot, ev, want)
                codes = {e[3] for e in ev if e[1] != 'saved'}
                if not codes or any(c >= 0 for c in codes):
                    errors.append(f'boot 1: error codes of the refused saves {sorted(codes)}')
                if flash.stats['wrperr'] == 0:
                    errors.append('boot 1: the protection was never hit')
            elif boot == 2:
                expect_events(errors, boot, ev, [(0, 'retry', 1), (1, 'retry', 2), (3, 'saved', 312)])
            elif boot == 3:
                expect_events(errors, boot, ev, [(0, 'retry', 1), (1, 'saved', 312)])
                if not any('chunk 1 save failed' in t for t in text):
                    errors.append('boot 3: the second chunk was not the one refused')
            elif boot == 4:
                if not any(RESTORED.search(t) and RESTORED.search(t).group(1) == '312' for t in text):
                    errors.append('boot 4: no "restored 312 bytes" line')
            elif boot == BOOT_NO_REGION:
                expect_events(errors, boot, ev, [(0, 'drop', 0)])
                if ev and ev[0][3] != -19:
                    errors.append(f'boot {boot}: missing region dropped with {ev[0][3]}, -19 expected')
                # Mount tries: the restore at boot, the one save, and the
                # final check of the firmware after the ten seconds; none
                # from the polls in between.
                stamps = [p for p, t in st['lines'] if 'no NVM0 region' in t]
                if stamps != [0, 0, 10] or st['init'] != 3:
                    errors.append(f'boot {boot}: store mounted {st["init"]} times, region message at polls '
                                  f'{stamps}, expected 3 at [0, 0, 10]')
                if flash.stats['programs'] or flash.stats['erases']:
                    errors.append(f'boot {boot}: flash written without a region')

    print(f'{passed} passed, {failed} failed')
    print('  model counters:', totals)
    if totals['progerr']:
        errors.append(f'{totals["progerr"]} quad words programmed over data')
    if totals['erases'] == 0:
        errors.append('no page erase: garbage collection never ran')
    print(f'STM32WBA bond store over the flash model: {passed + failed} checks, {len(errors)} failures')
    for e in errors:
        print('  FAIL', e)
    print('Hardware not tested.')
    if errors:
        sys.exit(1)


if __name__ == '__main__':
    main()
