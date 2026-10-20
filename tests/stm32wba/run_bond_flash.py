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
  Default_Handler is a device stopped at boot and fails the test;
- ECCR as RM0515 gives it: ADDR_ECC is the byte offset of the word in its
  bank, and an error is recorded only while ECCC and ECCD are both clear.
  A double error read with a flag already set raises nothing and the load
  returns the damaged bytes; a read the driver ends without reading such a
  word again fails the test;
- ICACHE, disabled or enabled: then a read of the store after
  a program or an erase without an invalidation in between, or an ECC NMI
  that returns without one, fails the test. A masked double error can fill
  a cached word: clearing ECCC without invalidating it serves those same
  bytes without a new ECC check. Raw read tests cover this and corrections
  from elsewhere on both retry attempts;
- corrected errors in the cases the driver cannot see from ECCR alone: the
  banks swapped, ADDR_ECC giving the address of the load rather than of its
  quad word, and a corrected error on every instruction fetched from flash
  between the ECCC clear and the load of the word. The last one stands for
  a weak cell in code run in that window; the driver copies each word from
  RAM with interrupts masked, so nothing is fetched there.

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

Bond deletion, the four boots of BtSmpBondErase on a store of its own: the
stored image is erased from the application loop and stays erased across a
reset; a save request left from before the clear is dropped, one made by the
clear is saved after the erase; a clear the stack refuses keeps the image and
the waiting save; an erase the medium refuses is tried after 1, 2 and 4 s
until it succeeds, and the next save is stored.

With --powercut it also cuts the power at every program and erase of one
save, whole or left half done, for a bank switch in each direction and for
a save that runs the PDS garbage collection, then boots: the restore must
give the old or the new image and the store must take the next save. A
half done cut leaves a damaged quad word, so the next boot takes the flash
ECC NMI, which must not stop it. The sweep runs about 15 minutes. It also
cuts every program and erase of a bond deletion: the restore must give the
old image or nothing, and a deleted image must not come back at a later cut.
--powercut-erase runs only the deletion sweep.

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
	.data : { _sdata = .; *(.data*) *(.fastrun*) . = ALIGN(4); _edata = .; } > RAM AT > FLASH
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
NMI_BUDGET = 5_000_000	# instructions an NMI handler may run before it counts as stopped
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

    def __init__(self, mem, torn=None, transient=None, swap=False, corrected=None, icache=False,
                 foreign=None, foreign_always=False, ecc_at_load=False, weak_fetch=False):
        self.mem = mem			# bytearray of NVM_MAP bytes from NVM_BASE - kept across boots
        # Offsets of quad words a power cut left part programmed or part
        # erased, kept across boots. The silicon checks every flash read
        # against the quad word ECC; such a word can fail it, and on the WBA
        # a double ECC error raises the NMI (stm32wbaxx_hal_flash_ex.c).
        self.torn = torn if torn is not None else set()
        # Quad words whose next reads fail their ECC check, then read well:
        # offset -> reads left to fail.
        self.transient = transient if transient is not None else {}
        # Quad words with a single bit error the ECC corrects on every read:
        # ECCC is set, no NMI.
        self.corrected = corrected if corrected is not None else set()
        # SWAP_BANK option: the second physical bank is mapped first. BKER
        # names the physical bank; BK_ECC is given as the physical bank too.
        self.swap = swap
        self.off_optr = reg_off('FLASH_TypeDef', 'OPTR')
        self.SWAP_BANK = hdr('FLASH_OPTR_SWAP_BANK')
        self.ecc_reads = []		# reads that touched a torn quad word
        # Reads of a damaged quad word that raised nothing because ECCC or
        # ECCD was already set: the load returned the damaged bytes.
        self.masked = []
        # The last of those not yet read again, and the damaged quad words a
        # read moved past without reading them again: bytes the driver handed
        # on as data.
        self.unchecked = None
        self.returned = []
        # Quad words whose first load after an ECCC clear finds ECCC set by
        # something else, an interrupt or a code fetch with a corrected error
        # between the clear and the load. Every other time, so a word read
        # again after one more clear is read without it.
        self.foreign = foreign if foreign is not None else set()
        self.foreign_always = foreign_always
        # ADDR_ECC of a corrected error gives the address of the load rather
        # than of its quad word, should the silicon record it that way.
        self.ecc_at_load = ecc_at_load
        # Every instruction fetched from flash between an ECCC clear and the
        # next load of the store has a corrected error: the worst case of a
        # weak cell in code that runs in that window. Needs the code hook of
        # run_boot, so only for short boots.
        self.weak_fetch = weak_fetch
        self.window = False		# ECCC cleared, no load of the store since
        self.weak_fetches = 0
        self.flip = {}
        self.loaded = set()		# quad words loaded since ECCC was last cleared
        # In the NMI handler. Its clear of the flags is not a clear before a
        # load: the load it returns to ran once on the silicon, before the NMI.
        self.in_nmi = False
        # ICACHE enabled: the driver has to invalidate it after a program, an
        # erase and a double ECC error it takes. Invalidations counted.
        self.icache = icache
        self.icache_inv = 0
        self.stale = False		# programmed or erased since the last invalidation
        # A masked double error can fill a cache line with invalid data.
        # Keep those words until invalidation: flag clears alone do not make
        # a later load go back to flash and check ECC again.
        self.masked_cache = {}
        self.masked_cache_hits = 0
        self.masked_cache_clear = {}
        self.ecc_clears = 0
        self.masked_cache_reuses = 0
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
        # came once. That one load, the first after the return, raises nothing
        # for the quad words that already raised the NMI. Any later load does,
        # even by the same instruction, as in a byte copy loop.
        self.skip = set()
        self.resumed = False		# the next load is the one the NMI interrupted
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
        if off == self.off_optr:
            return self.SWAP_BANK if self.swap else 0
        return 0

    def wr(self, off, val):
        if off == self.off_eccr:
            # ECCD and ECCC clear by writing 1, ECCIE is a setting
            if (val & self.ECCC) and not self.in_nmi:
                self.loaded.clear()
                self.ecc_clears += 1
                self.window = True
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
        if self.swap:
            bank ^= 1			# physical bank to the mapped one
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
        self.stale = True
        self.done(0)

    # memory of the store region
    def mem_rd(self, off, size):
        if self.icache and self.stale:
            self.v('read of the store through the enabled ICACHE not invalidated after a program or an erase')
        quads = sorted({off - off % QWORD, (off + size - 1) - (off + size - 1) % QWORD})
        self.window = False
        if self.unchecked is not None and self.unchecked not in quads:
            self.returned.append(NVM_BASE + self.unchecked)
            self.unchecked = None
        for q in quads:
            if q in self.foreign and q not in self.loaded:
                self.flip[q] = self.foreign_always or not self.flip.get(q, False)
                if self.flip[q]:
                    self.ecc_record((q + PAGE) % NVM_MAP, self.ECCC)
            self.loaded.add(q)
        if self.icache and all(q in self.masked_cache for q in quads):
            self.masked_cache_hits += 1
            if any(self.masked_cache_clear[q] < self.ecc_clears for q in quads):
                self.masked_cache_reuses += 1
            data = bytes(self.masked_cache[a - a % QWORD][a % QWORD]
                         for a in range(off, off + size))
            return int.from_bytes(data, 'little')
        resumed = self.resumed
        self.resumed = False
        if not resumed:
            self.skip.clear()
        if self.transient or self.torn or self.corrected:
            from unicorn.arm_const import UC_ARM_REG_PC
            pc = self.uc.reg_read(UC_ARM_REG_PC)
            for q in quads:
                if q in self.corrected:
                    self.ecc_record(q, self.ECCC, max(off, q) if self.ecc_at_load else q)
            for q in quads:
                if q in self.skip:
                    continue
                if self.transient.get(q, 0) > 0:
                    self.transient[q] -= 1
                elif q not in self.torn:
                    continue
                self.ecc_error(pc, q)
                break
        return int.from_bytes(self.mem[off:off + size], 'little')

    def ecc_record(self, q, flag, at=None):
        """Record a corrected (ECCC) or double (ECCD) ECC error. ECCR takes
        an error only while ECCC and ECCD are both clear: a later error sets
        no flag and records no address until software clears them (RM0515
        FLASH_ECCR), so a double error behind a corrected one raises no NMI.
        ADDR_ECC is the byte offset of the word from the start of its bank,
        or of At when given. Q may be below the store, for code. True when
        the error was recorded."""
        if self.eccr & (self.ECCD | self.ECCC):
            return False
        bank_size = FLASH_KB * 1024 // 2
        rel = NVM_BASE + (q if at is None else at) - FLASH_BASE
        bank = rel >= bank_size
        rel -= bank_size if bank else 0
        if self.swap:
            bank = not bank		# the physical bank
        self.eccr = (self.eccr & self.ECCIE) | (rel & self.ADDR_ECC) | \
            (self.BK_ECC if bank else 0) | flag
        return True

    def ecc_error(self, pc, q):
        """A read of a torn quad word: double ECC error, NMI after the read,
        unless a flag already set keeps the controller from recording it."""
        addr = NVM_BASE + q
        self.ecc_reads.append(addr)
        if not self.ecc_record(q, self.ECCD):
            self.masked.append(addr)
            self.unchecked = q
            if self.icache:
                self.masked_cache[q] = bytes(self.mem[q:q + QWORD])
                self.masked_cache_clear[q] = self.ecc_clears
            return
        if self.unchecked == q:
            self.unchecked = None
        self.skip.add(q)
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
        self.stale = True
        self.done(0)


def run_boot(elf_path, boot, mem, torn=None, cut=None, tear=False, args=None, transient=None,
             swap=False, corrected=None, icache=False, foreign=None, foreign_always=False,
             ecc_at_load=False, weak_fetch=False):
    from unicorn import Uc, UC_ARCH_ARM, UC_MODE_THUMB, UC_MODE_MCLASS, UC_HOOK_CODE
    from unicorn.arm_const import UC_CPU_ARM_CORTEX_M33
    from unicorn.arm_const import UC_ARM_REG_R0, UC_ARM_REG_R1, UC_ARM_REG_SP, UC_ARM_REG_PC
    from elftools.elf.elffile import ELFFile

    elf = ELFFile(open(elf_path, 'rb'))
    symtab = list(elf.get_section_by_name('.symtab').iter_symbols())
    syms = {s.name: s['st_value'] for s in symtab}
    sizes = {s.name: s['st_size'] for s in symtab}

    uc = Uc(UC_ARCH_ARM, UC_MODE_THUMB | UC_MODE_MCLASS)
    uc.ctl_set_cpu_model(UC_CPU_ARM_CORTEX_M33)
    uc.mem_map(FLASH_BASE, NVM_BASE - FLASH_BASE)
    uc.mem_map(0x20000000, 0x80000)
    uc.mem_map(0xE0000000, 0x100000)
    for seg in elf.iter_segments():
        if seg['p_type'] == 'PT_LOAD' and seg['p_filesz']:
            uc.mem_write(seg['p_paddr'], seg.data())

    flash = Flash(mem, torn, transient, swap, corrected, icache, foreign, foreign_always,
                  ecc_at_load, weak_fetch)
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

    # ICACHE, disabled unless asked for: the driver reads CR to know, and an
    # enabled cache is invalidated after a program, an erase and a double ECC
    # error it takes. The invalidation is instant: SR never shows BUSYF.
    icache = hdr('ICACHE_BASE_NS')
    icache_cr = icache - (icache & ~0xFFF) + reg_off('ICACHE_TypeDef', 'CR')
    ic_en = hdr('ICACHE_CR_EN')
    ic_inv = hdr('ICACHE_CR_CACHEINV')

    def icache_rd(uc, off, size, user):
        return ic_en if flash.icache and off == icache_cr else 0

    def icache_wr(uc, off, size, val, user):
        if not flash.icache:
            flash.v('write of ICACHE while it is disabled')
        elif off != icache_cr or (val & ~(ic_en | ic_inv)) or not (val & ic_en):
            flash.v(f'ICACHE write {val:#x} at {off:#x}')
        elif val & ic_inv:
            flash.icache_inv += 1
            flash.stale = False
            flash.masked_cache.clear()
            flash.masked_cache_clear.clear()

    uc.mmio_map(icache & ~0xFFF, 0x1000, icache_rd, None, icache_wr, None)

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

    if weak_fetch:
        def on_fetch(uc, addr, size, user):
            if flash.window and not flash.in_nmi:
                flash.weak_fetches += 1
                flash.ecc_record(addr - NVM_BASE, flash.ECCC)

        uc.hook_add(UC_HOOK_CODE, on_fetch, begin=FLASH_BASE, end=NVM_BASE - 1)
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
    nmi = {'saved': None, 'resume': 0, 'ret': False, 'inv': 0}

    def on_trap(uc, addr, size, user):
        nmi['ret'] = True
        uc.emu_stop()

    def on_default(uc, addr, size, user):
        st['hang'] = True
        uc.emu_stop()

    for name, cb in (('BondFwNmiTrap', on_trap), ('Default_Handler', on_default)):
        a = syms[name] & ~1
        uc.hook_add(UC_HOOK_CODE, cb, begin=a, end=a + max(sizes[name], 2) - 1)

    start = pc
    try:
        while True:
            # The NMI handler gets a budget: the compiler can inline the
            # Default_Handler loop into it, and a handler that does not
            # return within it is a device stopped there.
            uc.emu_start(start | 1, 0xFFFFFFFF,
                         count=NMI_BUDGET if nmi['saved'] is not None else 2_000_000_000)
            if st['done'] or st['hang'] or flash.cut_done:
                break
            if nmi['ret']:
                nmi['ret'] = False
                if flash.eccr & flash.ECCD:
                    flash.v('NMI returned with ECCD still set')
                if flash.icache and flash.icache_inv == nmi['inv']:
                    flash.v('ECC NMI returned without invalidating the enabled ICACHE')
                for r, val in zip(regs, nmi['saved']):
                    uc.reg_write(r, val)
                nmi['saved'] = None
                flash.in_nmi = False
                flash.resumed = True
                start = nmi['resume']
                continue
            if flash.nmi_pending and nmi['saved'] is None:
                flash.nmi_pending = False
                nmi['saved'] = [uc.reg_read(r) for r in regs]
                nmi['resume'] = uc.reg_read(UC_ARM_REG_PC)
                nmi['inv'] = flash.icache_inv
                flash.in_nmi = True
                uc.reg_write(UC_ARM_REG_LR, syms['BondFwNmiTrap'] | 1)
                start = syms['NMI_Handler'] & ~1
                st['nmis'] += 1
                if st['nmis'] > 20000:
                    raise RuntimeError('more than 20000 NMI in one boot')
                continue
            if nmi['saved'] is not None:
                st['hang'] = True
                break
            raise RuntimeError('emulator stopped without a reason')
    except Exception as ex:
        text = '\n'.join(f'[{p}] {line}' for p, line in st['lines'])
        raise SystemExit(f'FAIL boot {boot}: emulator stopped at {uc.reg_read(UC_ARM_REG_PC):#x}: {ex} {bad}\n'
                         f'{text[-2000:]}')
    if flash.unchecked is not None:
        flash.returned.append(NVM_BASE + flash.unchecked)
        flash.unchecked = None
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
DEL_RETRY = re.compile(r'WBA bond: delete failed \((-?\d+)\), retry in (\d+) s')
DELETED = re.compile(r'WBA bond: stored bonds deleted')


def events(st):
    """Retry, drop, saved, delete retry and deleted lines with the poll count
    when printed."""
    ev = []
    for poll, line in st['lines']:
        m = RETRY.search(line)
        if m:
            ev.append((poll, 'retry', int(m.group(2)), int(m.group(1))))
            continue
        m = DEL_RETRY.search(line)
        if m:
            ev.append((poll, 'delretry', int(m.group(2)), int(m.group(1))))
            continue
        if DELETED.search(line):
            ev.append((poll, 'deleted', 0, 0))
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
BOOT_ERASE = 9			# first of the four bond deletion boots
BOOT_CUT_ERASE = 13
BOOT_CUT_ERASE_CHECK = 14
BOOT_PROGRAM_ERROR = 15
BOOT_ECC_RETRY = 16


def boot_ok(errors, label, result, st, flash, read_rejected=False):
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
    # The normal boots must not pass on a masked word. The raw failure
    # test instead checks the negative Read return in the firmware itself;
    # its discarded buffer may contain bytes from the rejected read.
    if flash.returned and not read_rejected:
        errors.append(f'{label}: {len(flash.returned)} reads returned a damaged quad word that raised no NMI '
                      f'and was not read again, first at {flash.returned[0]:#x} '
                      f'({len(flash.masked)} loads raised nothing)')


def gc_fillers(elf, args, **kw):
    """Fill the active sector until the save has to collect it: the filler
    count goes in args[4]. False when no count from 25 to 44 does."""
    for fillers in range(25, 45):
        args[4] = fillers
        mem = bytearray(b'\xff' * NVM_MAP)
        run_boot(elf, BOOT_CUT_PREPARE, mem, args=args, **kw)
        _, _, fl = run_boot(elf, BOOT_CUT_SAVE, mem, args=args, **kw)
        if fl.stats['erases']:
            return True
    return False


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
        if args[4] is None and not gc_fillers(elf, args):
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


def powercut_erase(elf, errors):
    """Cut the power at every program and erase of one bond deletion, whole or
    half done, then boot: the restore must give the old image or nothing, and
    once a cut gives nothing every later cut must too, so a deleted image never
    comes back."""
    # old seed, old words, seed and words of the save after the cut, fillers,
    # old image in bank 1
    cases = [
        ('deletion of an image in bank 0', [41, 39, 42, 39, 0, 0]),
        ('deletion of an image in bank 1', [43, 64, 44, 39, 0, 1]),
        ('deletion with a sector full of other records', [45, 39, 46, 64, 30, 0]),
    ]
    total = 0
    for name, args in cases:
        mem = bytearray(b'\xff' * NVM_MAP)
        result, st, fl = run_boot(elf, BOOT_CUT_PREPARE, mem, args=args)
        boot_ok(errors, f'{name}, prepare', result, st, fl)
        start = bytes(mem)
        ref = bytearray(start)
        result, st, fl = run_boot(elf, BOOT_CUT_ERASE, ref, args=args)
        boot_ok(errors, f'{name}, uncut deletion', result, st, fl)
        steps = fl.ops
        if steps == 0:
            errors.append(f'{name}: the deletion wrote nothing')
            continue
        counts = {'old': 0, 'none': 0, 'neither': 0}
        for tear in (False, True):
            seen_none = False
            for cut in range(steps):
                mem = bytearray(start)
                torn = set()
                _, st, fl = run_boot(elf, BOOT_CUT_ERASE, mem, torn, cut, tear, args)
                if not fl.cut_done:
                    errors.append(f'{name}: cut {cut} was not reached')
                    continue
                label = f'{name}, cut {cut}{" torn" if tear else ""}'
                result, st, fl = run_boot(elf, BOOT_CUT_ERASE_CHECK, mem, torn, args=args)
                boot_ok(errors, label, result, st, fl)
                got = None
                for _, t in st['lines']:
                    m = re.search(r'erase cut: restored (\w+)', t)
                    if m:
                        got = m.group(1)
                if got is None:
                    errors.append(f'{label}: no restore line')
                    continue
                counts[got] += 1
                if got == 'none':
                    seen_none = True
                elif got == 'old' and seen_none:
                    errors.append(f'{label}: the deleted image came back after a later cut')
                total += 1
        print(f'  {name}: {steps} flash operations, {2 * steps} cuts, restored old {counts["old"]}, '
              f'nothing {counts["none"]}, neither {counts["neither"]}')
    return total



def boot_or_error(errors, label, *a, **kw):
    """run_boot, with an emulator that stops for no reason reported as a
    failure of the run instead of ending the test."""
    try:
        return run_boot(*a, **kw)
    except SystemExit as e:
        errors.append(f'{label}: {str(e).splitlines()[0]}')
        st = {'lines': [], 'hang': True, 'nmis': 0}
        return -1, st, Flash(bytearray(NVM_MAP))


def driver_runs(elf, errors):
    """Runs of the flash driver on its own. Returns how many passed."""
    passed = 0

    # A program the controller refuses fails the write, a full 64 byte block
    # as well as a shorter one: three refused programs in boot 15.
    n = len(errors)
    mem = bytearray(b'\xff' * NVM_MAP)
    result, st, fl = boot_or_error(errors, 'program error', elf, BOOT_PROGRAM_ERROR, mem)
    for _, t in st['lines']:
        if '[FAIL]' in t:
            errors.append(f'program error: {t}')
    if result != 0 or st['hang'] or st['nmis']:
        errors.append(f'program error: {result} failures, {st["nmis"]} NMI, stopped {st["hang"]}')
    for v in fl.viol:
        errors.append(f'program error: model violation: {v}')
    if fl.stats['progerr'] != 3:
        errors.append(f'program error: {fl.stats["progerr"]} programs refused, 3 expected')
    if len(errors) == n:
        passed += 1
        print('programs the controller refuses: every write fails, 64 byte blocks included')

    # Banks swapped: the erases of the deletion boots go to the page of the
    # store, not to the other bank, and an ECC error is taken.
    n = len(errors)
    mem = bytearray(b'\xff' * NVM_MAP)
    for boot in range(BOOT_ERASE, BOOT_ERASE + 4):
        result, st, fl = boot_or_error(errors, f'banks swapped, boot {boot}', elf, boot, mem, swap=True)
        boot_ok(errors, f'banks swapped, boot {boot}', result, st, fl)
        if boot == BOOT_ERASE and fl.stats['erases'] == 0:
            errors.append('banks swapped: the deletion erased nothing')
    args = [23, 39, 24, 64, 0, 0]
    mem = bytearray(b'\xff' * NVM_MAP)
    result, st, fl = boot_or_error(errors, 'banks swapped, prepare', elf, BOOT_CUT_PREPARE, mem,
                                   args=args, swap=True)
    boot_ok(errors, 'banks swapped, prepare', result, st, fl)
    result, st, fl = boot_or_error(errors, 'banks swapped, read error', elf, BOOT_CUT_CHECK, mem,
                                   args=args, transient={0: 2}, swap=True)
    boot_ok(errors, 'banks swapped, read error', result, st, fl)
    if not any('cut sweep: restored old' in t for _, t in st['lines']) or st['nmis'] != 2:
        errors.append(f'banks swapped, read error: {st["nmis"]} NMI, the image was not restored')
    if len(errors) == n:
        passed += 1
        print('banks swapped: erases stay in the store, ECC errors are taken')

    # Corrected errors. ECCR takes no error while ECCC is set, so a corrected
    # error left set hides a double error read after it: no NMI, and the read
    # returns the damaged bytes. The last record written has a torn quad word
    # (ECCD); a quad word with a single bit error (ECCC) is read first, in an
    # earlier read (the first record header of the store) and in the same
    # read (the quad word before the torn one).
    args = [25, 39, 26, 39, 20, 0]
    mem = bytearray(b'\xff' * NVM_MAP)
    result, st, fl = boot_or_error(errors, 'corrected error, prepare', elf, BOOT_CUT_PREPARE, mem,
                                   args=args)
    boot_ok(errors, 'corrected error, prepare', result, st, fl)
    last = max(q for q in range(0, PAGE, QWORD) if mem[q:q + QWORD] != b'\xff' * QWORD)
    for where, corrected in (('in an earlier read', {QWORD}), ('in the same read', {last - QWORD})):
        n = len(errors)
        label = f'corrected error {where}'
        result, st, fl = boot_or_error(errors, label, elf, BOOT_CUT_CHECK, bytearray(mem), {last},
                                       args=args, corrected=set(corrected))
        boot_ok(errors, label, result, st, fl)
        if not any('cut sweep: restored old' in t for _, t in st['lines']) or st['nmis'] == 0:
            errors.append(f'{label}: {st["nmis"]} NMI, the image was not restored')
        if len(errors) == n:
            passed += 1
            print(f'corrected error {where}: the double error at {last:#x} is taken, {st["nmis"]} NMI, '
                  'image restored')

    # A corrected error from elsewhere, an interrupt or a code fetch between
    # the ECCC clear and the load, sets ECCC before the torn quad word is
    # read: that load raises nothing. Seeing ECCC set after the word, the
    # driver reads it again with ECCC clear, and that read raises the NMI.
    n = len(errors)
    label = 'corrected error from elsewhere'
    result, st, fl = boot_or_error(errors, label, elf, BOOT_CUT_CHECK, bytearray(mem), {last},
                                   args=args, foreign={last})
    boot_ok(errors, label, result, st, fl)
    if not any('cut sweep: restored old' in t for _, t in st['lines']) or st['nmis'] == 0 or not fl.masked:
        errors.append(f'{label}: {st["nmis"]} NMI, {len(fl.masked)} loads raised nothing, image restored '
                      f'{any("cut sweep: restored old" in t for _, t in st["lines"])}')
    if len(errors) == n:
        passed += 1
        print(f'corrected error from elsewhere before the torn quad word: {len(fl.masked)} loads raised nothing, '
              f'each read again, {st["nmis"]} NMI, image restored')

    # Raw driver regressions. A correction on both attempts must fail even
    # on readable data, and a masked error cached on the first attempt must
    # be evicted before the next attempt. Also keep persistent single-bit
    # corrections usable, rather than treating every ECCC as damaged data.
    cases = [
        ('foreign correction on both attempts', dict(foreign={16}, foreign_always=True), True),
        ('masked double error on both attempts', dict(torn={16}, foreign={16}, foreign_always=True), True),
        ('cached masked double error', dict(torn={16}, foreign={16}, icache=True), True),
        ('persistent single-bit correction', dict(corrected={16}), False),
        ('cached valid data after a foreign correction', dict(foreign={16}, icache=True), False),
        # With the banks swapped BK_ECC names either bank, so the word's own
        # correction is taken with both readings, as the NMI takes a hit.
        ('persistent correction, banks swapped', dict(corrected={16}, swap=True), False),
        # ADDR_ECC at the address of the load; the read starts at byte 24,
        # inside the corrected quad word at 16.
        ('correction recorded at the load address', dict(corrected={16}, ecc_at_load=True), False, 24),
        # A corrected error on every flash fetch between the ECCC clear and
        # the load. The word copy runs from RAM with interrupts masked, so
        # nothing is fetched from flash there and the data is read.
        ('flash fetch between the clear and the load', dict(weak_fetch=True), False),
    ]
    for case in cases:
        label, options, rejected = case[:3]
        offset = case[3] if len(case) > 3 else 0
        n = len(errors)
        result, st, fl = boot_or_error(errors, label, elf, BOOT_ECC_RETRY,
                                       bytearray(b'\xa5' * NVM_MAP), args=[int(rejected), offset],
                                       **options)
        boot_ok(errors, label, result, st, fl, read_rejected=rejected)
        if options.get('torn') and not fl.masked:
            errors.append(f'{label}: no double error was masked')
        if options.get('foreign_always') and fl.flip.get(16) is not True:
            errors.append(f'{label}: no foreign correction was injected')
        if label == 'cached masked double error' and fl.masked_cache_reuses:
            errors.append(f'{label}: damaged cache survived an ECC clear {fl.masked_cache_reuses} times')
        if len(errors) == n:
            passed += 1
            print(f'{label}: {"read rejected" if rejected else "valid data kept"}, '
                  f'{st["nmis"]} NMI, {fl.icache_inv} cache invalidations')

    # The same weak flash fetch through a whole mount. A read refused on
    # every attempt makes PDS take the sector header as damaged and erase the
    # sector holding the image; the word copy from RAM keeps every fetch out
    # of the window, so the store mounts and the image comes back.
    n = len(errors)
    label = 'saved image, flash fetch between the clear and the load'
    args = [21, 39, 22, 64, 0, 0]
    mem = bytearray(b'\xff' * NVM_MAP)
    result, st, fl = boot_or_error(errors, f'{label}, prepare', elf, BOOT_CUT_PREPARE, mem, args=args)
    boot_ok(errors, f'{label}, prepare', result, st, fl)
    result, st, fl = boot_or_error(errors, label, elf, BOOT_CUT_CHECK, mem, args=args, weak_fetch=True)
    boot_ok(errors, label, result, st, fl)
    if not any('cut sweep: restored old' in t for _, t in st['lines']) or fl.stats['erases'] or fl.weak_fetches:
        errors.append(f'{label}: {fl.stats["erases"]} erases, {fl.weak_fetches} flash fetches in the window, '
                      f'image restored {any("cut sweep: restored old" in t for _, t in st["lines"])}')
    if len(errors) == n:
        passed += 1
        print(f'{label}: no fetch from flash in the window, store mounted, image restored')

    # ICACHE enabled. It is invalidated after every program and erase, and in
    # the NMI of a double error the driver takes: a line filled by the failed
    # read would give the damaged bytes again, with no NMI, to the read that
    # retries it. The save collects a sector, so erases run too.
    n = len(errors)
    args = [27, 39, 28, 64, None, 0]
    if not gc_fillers(elf, args, icache=True):
        errors.append('ICACHE enabled: no filler count makes the save collect a sector')
    mem = bytearray(b'\xff' * NVM_MAP)
    result, st, fl = boot_or_error(errors, 'ICACHE enabled, prepare', elf, BOOT_CUT_PREPARE, mem,
                                   args=args, icache=True)
    boot_ok(errors, 'ICACHE enabled, prepare', result, st, fl)
    result, st, fl = boot_or_error(errors, 'ICACHE enabled, save', elf, BOOT_CUT_SAVE, mem,
                                   args=args, icache=True)
    boot_ok(errors, 'ICACHE enabled, save', result, st, fl)
    if fl.stats['erases'] == 0 or fl.icache_inv == 0:
        errors.append(f'ICACHE enabled, save: {fl.stats["erases"]} erases, {fl.icache_inv} invalidations')
    last = max(q for q in range(0, NVM_MAP, QWORD) if mem[q:q + QWORD] != b'\xff' * QWORD)
    result, st, fl = boot_or_error(errors, 'ICACHE enabled, double error', elf, BOOT_CUT_CHECK, mem, {last},
                                   args=args, transient={0: 2}, icache=True)
    boot_ok(errors, 'ICACHE enabled, double error', result, st, fl)
    if st['nmis'] < 3 or not any(re.search(r'cut sweep: restored (old|new)', t) for _, t in st['lines']):
        errors.append(f'ICACHE enabled, double error: {st["nmis"]} NMI, nothing restored')
    if len(errors) == n:
        passed += 1
        print(f'ICACHE enabled: invalidated after programs, erases and {st["nmis"]} ECC NMI')
    return passed

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
        runs = [(elf, b) for b in range(BOOTS)] + [(elf_none, BOOT_NO_REGION)] + \
            [(elf, b) for b in range(BOOT_ERASE, BOOT_ERASE + 4)]
        for path, boot in runs:
            if path is elf_none or boot == BOOT_ERASE:
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
            elif boot == BOOT_ERASE:
                expect_events(errors, boot, ev, [(0, 'saved', 312), (0, 'deleted', 0)])
            elif boot == BOOT_ERASE + 1:
                # The save request made before the clear never reaches the medium
                expect_events(errors, boot, ev, [(0, 'saved', 312), (0, 'deleted', 0),
                                                 (0, 'saved', 312), (0, 'deleted', 0),
                                                 (0, 'saved', 160)])
            elif boot == BOOT_ERASE + 2:
                want = [(0, 'saved', 312), (0, 'saved', 312), (0, 'delretry', 1), (1, 'delretry', 2),
                        (3, 'delretry', 4), (7, 'deleted', 0), (8, 'saved', 312)]
                expect_events(errors, boot, ev, want)
                codes = {e[3] for e in ev if e[1] == 'delretry'}
                if not codes or any(c >= 0 for c in codes):
                    errors.append(f'boot {boot}: error codes of the refused erases {sorted(codes)}')
                if flash.stats['wrperr'] == 0:
                    errors.append(f'boot {boot}: the protection was never hit')
            elif boot == BOOT_ERASE + 3:
                if not any(RESTORED.search(t) and RESTORED.search(t).group(1) == '312' for t in text):
                    errors.append(f'boot {boot}: no "restored 312 bytes" line')

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

        passed += driver_runs(elf, errors)

        if '--powercut' in sys.argv:
            print('power cut sweep:')
            cuts, damaged = powercut(elf, errors)
            print(f'  {cuts} power cuts checked')
            print(f'  {damaged} half done cuts left a quad word that raised the flash ECC NMI at the next boot')
        if '--powercut' in sys.argv or '--powercut-erase' in sys.argv:
            print('power cut sweep of a bond deletion:')
            cuts = powercut_erase(elf, errors)
            print(f'  {cuts} power cuts checked')

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
