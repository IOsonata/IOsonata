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
  change and a reset, so a retry can be watched succeeding mid wait;
- a read of a quad word left half programmed or half erased by a power cut
  is a double ECC error: ECCR records it and the NMI follows the read, as on
  the silicon (stm32wbaxx_hal_flash_ex.c). The firmware links the WBA65
  vector table, so the port's NMI_Handler runs; a boot that ends in
  Default_Handler is a device stopped at boot and fails the test.

The flash content is kept between the boots of the firmware, so a boot is a
reset. The one-second tick of the application is a call of
BtSmpBondWbaPoll; the emulator counts those calls and checks each retry line
against that count: tries after 1, 2, 4 .. 64 s, the save dropped after the
64 s wait (127 polls in total), and no retry from idle-loop calls while
that wait is active. The initial save still runs from the application loop.
A second image is linked without the bond store region and checks that a missing
region is reported and never retried. A read error that passes, two failed
reads of the header of the sector holding the image, must leave the sector
and the image in place: PDS tries a read three times before it takes the
data as damaged.

With --powercut it also cuts the power at every program and erase of one
save, whole or left half done, for a bank switch in each direction and for
a save that runs the PDS garbage collection, then boots: the restore must
give the old or the new image and the store must take the next save. A
half done cut leaves a damaged quad word, so the next boot takes the flash
ECC NMI, which must not stop it. The sweep runs about 15 minutes.

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

ENTRY(ResetEntry)

SECTIONS
{
	.ivector : { KEEP(*(.ivector)) } > FLASH
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
	__StackTop = _estack;
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
    for src in (ROOT / 'src/crc.c', ROOT / 'ARM/ST/STM32WBAxx/STM32WBA65/src/vectors_stm32wba65.c'):
        obj = out / (src.stem + '.o')
        rc.run(['arm-none-eabi-gcc', *cpu, '-std=gnu17', '-Wall', *inc, '-c', src, '-o', obj])
        objs.append(obj)
    elf = out / (name + '.elf')
    rc.run(['arm-none-eabi-g++', *cpu, '-nostartfiles', '-Wl,--gc-sections', '-Wl,-T,' + str(ld),
            '--specs=nosys.specs', '--specs=nano.specs', *objs, '-o', elf])
    return elf


class Flash:
    """STM32WBA flash controller, non-secure registers, and the memory of
    the bond store region."""

    def __init__(self, mem, torn=None, transient=None):
        self.mem = mem			# bytearray of NVM_MAP bytes from NVM_BASE - kept across boots
        # Offsets of quad words a power cut left part programmed or part
        # erased, kept across boots. The silicon checks every flash read
        # against the quad word ECC; such a word can fail it, and on the WBA
        # a double ECC error raises the NMI (stm32wbaxx_hal_flash_ex.c).
        self.torn = torn if torn is not None else set()
        # Quad words whose next reads fail their ECC check, then read well:
        # offset -> reads left to fail.
        self.transient = transient if transient is not None else {}
        self.ecc_reads = []		# reads that touched a torn quad word
        self.cut_at = None		# flash operation the power cut stops, None for no cut
        self.tear = False		# the cut operation is left half done
        self.cut_done = False
        self.ops = 0			# program and erase operations started
        self.uc = None
        self.off_keyr = reg_off('FLASH_TypeDef', 'NSKEYR')
        self.off_sr = reg_off('FLASH_TypeDef', 'NSSR')
        self.off_cr = reg_off('FLASH_TypeDef', 'NSCR1')
        self.off_eccr = reg_off('FLASH_TypeDef', 'ECCR')
        self.ECCD = hdr('FLASH_ECCR_ECCD')
        self.ECCC = hdr('FLASH_ECCR_ECCC')
        self.ECCIE = hdr('FLASH_ECCR_ECCIE')
        self.BK_ECC = hdr('FLASH_ECCR_BK_ECC')
        self.ADDR_ECC = hdr('FLASH_ECCR_ADDR_ECC_Msk')
        self.eccr = 0
        self.nmi_pending = False		# a double ECC error waits for the NMI
        # The emulator stops on the faulting load, so the return from the NMI
        # runs that load again; on the silicon the read completed and the NMI
        # came once. While that instruction runs again, the quad words that
        # already raised the NMI raise nothing.
        self.skip_pc = None
        self.skip = set()
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

    def cut_here(self):
        """Count one program or erase; True when the power cut stops it."""
        if self.cut_at is not None and self.ops == self.cut_at:
            self.cut_done = True
            self.uc.emu_stop()
            return True
        self.ops += 1
        return False

    def done(self, err):
        if err:
            self.sr |= err
        elif self.cr & self.EOPIE:
            self.sr |= self.EOP

    # registers
    def rd(self, off):
        if off == self.off_eccr:
            return self.eccr
        if off == self.off_cr:
            return self.cr
        if off == self.off_sr:
            return self.sr			# BSY and WDW never set: the model is instant
        return 0

    def wr(self, off, val):
        if off == self.off_eccr:
            # ECCD and ECCC clear by writing 1, ECCIE is a setting
            self.eccr &= ~(val & (self.ECCD | self.ECCC))
            self.eccr = (self.eccr & ~self.ECCIE) | (val & self.ECCIE)
            return
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
        if self.cut_here():
            if self.tear:
                # Half erased; the other half is left in an unknown state
                self.mem[o:o + PAGE // 2] = b'\xff' * (PAGE // 2)
                self.torn.update(range(o + PAGE // 2, o + PAGE, QWORD))
            return
        self.mem[o:o + PAGE] = b'\xff' * PAGE
        self.torn.difference_update(range(o, o + PAGE, QWORD))
        self.stats['erases'] += 1
        self.done(0)

    # memory of the store region
    def mem_rd(self, off, size):
        if self.skip_pc is not None or self.transient or self.torn:
            from unicorn.arm_const import UC_ARM_REG_PC
            pc = self.uc.reg_read(UC_ARM_REG_PC)
            if self.skip_pc is not None and pc != self.skip_pc:
                self.skip_pc = None
                self.skip.clear()
            for q in sorted({off - off % QWORD, (off + size - 1) - (off + size - 1) % QWORD}):
                if q in self.skip:
                    continue
                if self.transient.get(q, 0) > 0:
                    self.transient[q] -= 1
                elif q not in self.torn:
                    continue
                self.ecc_error(pc, q)
                break
        return int.from_bytes(self.mem[off:off + size], 'little')

    def ecc_error(self, pc, q):
        """A read of a torn quad word: double ECC error, NMI after the read.
        The first error stays recorded until ECCD is cleared."""
        addr = NVM_BASE + q
        if self.skip_pc != pc:
            self.skip.clear()
        self.skip_pc = pc
        self.skip.add(q)
        self.ecc_reads.append(addr)
        if not (self.eccr & self.ECCD):
            bank_size = FLASH_KB * 1024 // 2
            rel = addr - FLASH_BASE
            bank = rel >= bank_size
            rel -= bank_size if bank else 0
            self.eccr = (self.eccr & self.ECCIE) | self.ECCD | ((rel >> 3) & self.ADDR_ECC) | \
                (self.BK_ECC if bank else 0)
        self.nmi_pending = True
        self.uc.emu_stop()

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
        if self.cut_here():
            if self.tear:
                # Half the quad word programmed
                for i in range(QWORD // 2):
                    self.mem[o + i] &= data[i]
                self.torn.add(o)
            return
        if self.mem[o:o + QWORD] != b'\xff' * QWORD:
            self.stats['progerr'] += 1
            self.done(self.PROGERR)
            return
        self.mem[o:o + QWORD] = data
        self.stats['programs'] += 1
        self.done(0)


def run_boot(elf_path, boot, mem, torn=None, cut=None, tear=False, args=None, transient=None):
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

    flash = Flash(mem, torn, transient)
    flash.uc = uc
    flash.cut_at = cut
    flash.tear = tear

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

    st = {'poll': 0, 'done': False, 'init': 0, 'lines': [], 'buf': '', 'nmis': 0, 'hang': False}

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
    for i, v in enumerate(args or ()):
        uc.mem_write(syms['g_BondFwArg'] + 4 * i, v.to_bytes(4, 'little'))
    vec = bytes(uc.mem_read(FLASH_BASE, 8))
    uc.reg_write(UC_ARM_REG_SP, int.from_bytes(vec[0:4], 'little'))
    pc = int.from_bytes(vec[4:8], 'little')
    # The NMI of a double ECC error is entered by hand, the way
    # run_bt_crypto.py enters its interrupt: registers saved, the handler run
    # with the trap as its return address, registers restored at the trap.
    # NMI_Handler is the port's (vectors_stm32wba65.c); a handler that ends
    # in Default_Handler is a device stopped at boot.
    from unicorn.arm_const import (UC_ARM_REG_R2, UC_ARM_REG_R3, UC_ARM_REG_R4, UC_ARM_REG_R5,
                                   UC_ARM_REG_R6, UC_ARM_REG_R7, UC_ARM_REG_R8, UC_ARM_REG_R9,
                                   UC_ARM_REG_R10, UC_ARM_REG_R11, UC_ARM_REG_R12, UC_ARM_REG_LR,
                                   UC_ARM_REG_XPSR)
    regs = [UC_ARM_REG_R0, UC_ARM_REG_R1, UC_ARM_REG_R2, UC_ARM_REG_R3, UC_ARM_REG_R4, UC_ARM_REG_R5,
            UC_ARM_REG_R6, UC_ARM_REG_R7, UC_ARM_REG_R8, UC_ARM_REG_R9, UC_ARM_REG_R10, UC_ARM_REG_R11,
            UC_ARM_REG_R12, UC_ARM_REG_SP, UC_ARM_REG_LR, UC_ARM_REG_XPSR]
    nmi = {'saved': None, 'resume': 0, 'ret': False}

    def on_trap(uc, addr, size, user):
        nmi['ret'] = True
        uc.emu_stop()

    def on_default(uc, addr, size, user):
        st['hang'] = True
        uc.emu_stop()

    for name, cb in (('BondFwNmiTrap', on_trap), ('Default_Handler', on_default)):
        a = syms[name] & ~1
        uc.hook_add(UC_HOOK_CODE, cb, begin=a, end=a)

    start = pc
    try:
        while True:
            uc.emu_start(start | 1, 0xFFFFFFFF, count=2_000_000_000)
            if st['done'] or st['hang'] or flash.cut_done:
                break
            if nmi['ret']:
                nmi['ret'] = False
                if flash.eccr & flash.ECCD:
                    flash.v('NMI returned with ECCD still set')
                for r, val in zip(regs, nmi['saved']):
                    uc.reg_write(r, val)
                nmi['saved'] = None
                start = nmi['resume']
                continue
            if flash.nmi_pending and nmi['saved'] is None:
                flash.nmi_pending = False
                nmi['saved'] = [uc.reg_read(r) for r in regs]
                nmi['resume'] = uc.reg_read(UC_ARM_REG_PC)
                uc.reg_write(UC_ARM_REG_LR, syms['BondFwNmiTrap'] | 1)
                start = syms['NMI_Handler'] & ~1
                st['nmis'] += 1
                if st['nmis'] > 20000:
                    raise RuntimeError('more than 20000 NMI in one boot')
                continue
            raise RuntimeError('emulator stopped without a reason')
    except Exception as ex:
        text = '\n'.join(f'[{p}] {line}' for p, line in st['lines'])
        raise SystemExit(f'FAIL boot {boot}: emulator stopped at {uc.reg_read(UC_ARM_REG_PC):#x}: {ex} {bad}\n'
                         f'{text[-2000:]}')
    if flash.cut_done:
        return None, st, flash
    if st['hang']:
        return -1, st, flash
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


BOOT_CUT_PREPARE = 6
BOOT_CUT_SAVE = 7
BOOT_CUT_CHECK = 8


def boot_ok(errors, label, result, st, flash):
    if st['hang']:
        errors.append(f'{label}: stopped in Default_Handler after {st["nmis"]} NMI')
        return
    for _, t in st['lines']:
        if '[FAIL]' in t:
            errors.append(f'{label}: {t}')
    if result != 0:
        errors.append(f'{label}: the firmware counted {result} failures')
    for v in flash.viol:
        errors.append(f'{label}: model violation: {v}')
    if flash.stats['progerr']:
        errors.append(f'{label}: {flash.stats["progerr"]} quad words programmed over data')


def powercut(elf, errors):
    """Cut the power at every program and erase of one save, whole or half
    done, then boot: the restore must give the old or the new image and the
    store must take the next save."""
    # old seed, old words, new seed, new words, fillers, old image in bank 1
    cases = [
        ('first bank switch, 312 to 512 bytes', [11, 39, 12, 64, 0, 0]),
        ('switch back to bank 0, 512 to 312 bytes', [13, 64, 14, 39, 0, 1]),
        ('save through PDS garbage collection', [15, 39, 16, 64, None, 0]),
    ]
    total = 0
    damaged = 0
    for name, args in cases:
        if args[4] is None:
            # Fill the active sector until the save has to collect it
            for fillers in range(25, 45):
                args[4] = fillers
                mem = bytearray(b'\xff' * NVM_MAP)
                run_boot(elf, BOOT_CUT_PREPARE, mem, args=args)
                _, _, fl = run_boot(elf, BOOT_CUT_SAVE, mem, args=args)
                if fl.stats['erases']:
                    break
            else:
                errors.append(f'{name}: no filler count makes the save collect a sector')
                continue
        mem = bytearray(b'\xff' * NVM_MAP)
        result, st, fl = run_boot(elf, BOOT_CUT_PREPARE, mem, args=args)
        boot_ok(errors, f'{name}, prepare', result, st, fl)
        start = bytes(mem)
        ref = bytearray(start)
        result, st, fl = run_boot(elf, BOOT_CUT_SAVE, ref, args=args)
        boot_ok(errors, f'{name}, uncut save', result, st, fl)
        steps = fl.ops
        counts = {'old': 0, 'new': 0, 'neither': 0}
        reads = 0
        stops = 0
        for cut in range(steps):
            for tear in (False, True):
                mem = bytearray(start)
                torn = set()
                _, st, fl = run_boot(elf, BOOT_CUT_SAVE, mem, torn, cut, tear, args)
                if not fl.cut_done:
                    errors.append(f'{name}: cut {cut} was not reached')
                    continue
                label = f'{name}, cut {cut}{" torn" if tear else ""}'
                result, st, fl = run_boot(elf, BOOT_CUT_CHECK, mem, torn, args=args)
                boot_ok(errors, label, result, st, fl)
                stops += st['hang']
                for _, t in st['lines']:
                    m = re.search(r'cut sweep: restored (\w+)', t)
                    if m:
                        counts[m.group(1)] += 1
                if fl.ecc_reads:
                    reads += 1
                total += 1
        damaged += reads
        print(f'  {name}: {steps} flash operations, {2 * steps} cuts, restored old {counts["old"]}, '
              f'new {counts["new"]}, neither {counts["neither"]}; '
              f'{reads} of {steps} half done cuts took a flash ECC NMI at the next boot, '
              f'{stops} stopped there')
    return total, damaged


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
            if st['hang'] or st['nmis']:
                errors.append(f'boot {boot}: {st["nmis"]} NMI, stopped {st["hang"]}')
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

        # A read error that passes: the first quad word of the store, the
        # header of the sector holding the image, fails twice at boot, fewer
        # times than PDS tries a read. The sector is kept and the image comes
        # back; a store that took one failed read as damage would erase it.
        args = [21, 39, 22, 64, 0, 0]
        mem = bytearray(b'\xff' * NVM_MAP)
        result, st, fl = run_boot(elf, BOOT_CUT_PREPARE, mem, args=args)
        boot_ok(errors, 'passing read error, prepare', result, st, fl)
        if mem[0:4] == b'\xff' * 4:
            errors.append('passing read error: the image is not in the first sector')
        transient = {0: 2}
        result, st, fl = run_boot(elf, BOOT_CUT_CHECK, mem, args=args, transient=transient)
        boot_ok(errors, 'passing read error', result, st, fl)
        if not any('cut sweep: restored old' in t for _, t in st['lines']) or st['nmis'] != 2:
            errors.append(f'passing read error: {st["nmis"]} NMI, the image was not restored')
        else:
            passed += 1
            print('passing read error on a sector header: sector kept, image restored, 2 NMI')

        if '--powercut' in sys.argv:
            print('power cut sweep:')
            cuts, damaged = powercut(elf, errors)
            print(f'  {cuts} power cuts checked')
            print(f'  {damaged} half done cuts left a quad word that raised the flash ECC NMI at the next boot')

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
