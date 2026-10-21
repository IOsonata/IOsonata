#!/usr/bin/env python3
"""Compile H753 shared I2C/SPI and exercise all instances in a register model.

Requires GNU Arm GCC, unicorn and pyelftools. Reuses the common I2C v2 and
SPI CFG register models from STM32WBA, plus DMA1/2 and BDMA requests.
Physical timing and cache coherency are not modeled.
"""
import collections
from pathlib import Path
import subprocess
import sys
import tempfile
from elftools.elf.elffile import ELFFile
from unicorn import Uc, UC_ARCH_ARM, UC_MODE_THUMB, UC_MODE_MCLASS, UC_HOOK_BLOCK
from unicorn.arm_const import UC_ARM_REG_SP, UC_ARM_REG_LR, UC_ARM_REG_R0, UC_ARM_REG_PC, UC_ARM_REG_PRIMASK

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / 'tests/stm32wba'))
from run_spi import Spi, Log
from run_i2c import I2c, Slave
from dma_model import Dma


class H7Spi(Spi):
    """H753 TXP/TXC can interrupt even with SPE clear (ES0392)."""

    def wr_reg(self, off, size, value):
        if off == 0 and value & 0x200 and not self.slave():
            # ES0392: CRC disabled still requires the reset CRCSIZE=7
            # when receiving a finite-length transfer.
            assert (self.reg[8] >> 16) & 31 == 7, f'{self.name}: unsafe CRCSIZE'
        if off == 0x10 and self.slave() and not self.spe():
            # TXP is set while disabled; TSIZE=0 also sets TXC, whose
            # interrupt is selected by EOTIE, not SR bit 12 in IER.
            unsafe = value & 2 or (value & 8 and not (self.reg[4] & 0xffff))
            assert not unsafe, f'{self.name}: TXP/TXC IRQ enabled before SPE'
        super().wr_reg(off, size, value)


def run(args):
    subprocess.run([str(a) for a in args], check=True)


def emulate(path):
    u = Uc(UC_ARCH_ARM, UC_MODE_THUMB | UC_MODE_MCLASS)
    for base, size in ((0x08000000, 0x200000), (0x20000000, 0x20000), (0x38000000, 0x10000), (0xe0000000, 0x100000)):
        u.mem_map(base, size)
    with path.open('rb') as f:
        elf = ELFFile(f)
        for seg in elf.iter_segments():
            if seg['p_type'] == 'PT_LOAD':
                u.mem_write(seg['p_vaddr'], seg.data())
        syms = {s.name: s['st_value'] for s in elf.get_section_by_name('.symtab').iter_symbols()}
    log, violations = Log(), []
    spi = {a: H7Spi('SPI' + str(i + 1), log, violations) for i, a in enumerate(
        (0x40013000, 0x40003800, 0x40003c00, 0x40013400, 0x40015000, 0x58001400))}
    i2c = {a: I2c('I2C' + str(i + 1), a, log, violations) for i, a in enumerate(
        (0x40005400, 0x40005800, 0x40005c00, 0x58001c00))}
    for d in i2c.values():
        d.slaves[0x50] = Slave(0x50)
        for addr, at in ((0x51, 3), (0x52, 70), (0x53, 255)):
            d.slaves[addr] = Slave(addr, refuse_at=at)
    dma = Dma(u)
    for d, (tx, rx) in zip(spi.values(), ((38,37),(40,39),(62,61),(84,83),(86,85),(12,11))):
        d.req_tx, d.req_rx, d.gpdma = tx, rx, dma
    for d, (tx, rx) in zip(i2c.values(), ((34,33),(36,35),(74,73),(14,13))):
        d.req_tx, d.req_rx, d.gpdma = tx, rx, dma
    regs = collections.defaultdict(int)
    regs[0x58024400] = 5  # HSI on and ready, divide by one

    def read(uc, off, size, base):
        a = base + off
        if dma.decode(a): return dma.read(a)
        if a & ~255 in spi:
            return spi[a & ~255].rd_reg(a & 255, size)
        if a & ~255 in i2c:
            return i2c[a & ~255].rd_reg(a & 255)
        return regs[a]

    irq_active = False
    pending_irq = None
    phase = 0
    irq_bytes = collections.defaultdict(list)

    def write(uc, off, size, value, base):
        a = base + off
        if dma.decode(a):
            dma.write(a, value)
            return
        if a & ~255 in spi:
            spi[a & ~255].wr_reg(a & 255, size, value)
        elif a & ~255 in i2c:
            if phase and (a & 255) == 0x28:
                assert irq_active, 'interrupt TX wrote TXDR from polling code'
                irq_bytes[phase].append(value & 255)
            i2c[a & ~255].wr_reg(a & 255, value)
        else:
            regs[a] = value

    for base, size in ((0x40000000, 0x30000), (0x52005000, 0x1000), (0x58000000, 0x30000)):
        u.mmio_map(base, size, read, base, write, base)
    u.reg_write(UC_ARM_REG_SP, 0x2001fff0)
    u.reg_write(UC_ARM_REG_LR, 0x080ff001)
    # Dispatch enabled I2C vectors between foreground basic blocks. Save and
    # restore the interrupted CPU context, preserving the handler's RAM writes.
    # This models preemption, not NVIC timing or Cortex-M exception entry.
    ticks = 0
    slave_case = 0
    slave_checks = []
    def dispatch(uc, address, size, ctx):
        nonlocal pending_irq, phase, ticks, slave_case
        if irq_active:
            return
        case = int.from_bytes(uc.mem_read(syms['DmaSlaveCase'],4),'little')
        if case and case != slave_case:
            slave_case = case
            bus, index = divmod(case-1000, 100)
            n, part = divmod(index,2)
            length = 600 if part else 31
            payload = bytes((k*3+9)&255 for k in range(length))
            dev = list(spi.values() if bus else i2c.values())[n]
            if bus:
                dev.script = dev.slave_script([payload])
                slave_checks.append((dev, True, len(dev.sent_by_slave), length))
            else:
                dev.script = dev.master_script([('W',0x30,payload,'P'),('R',0x30,length,'P')])
                slave_checks.append((dev, False, len(dev.got), length))
        if ticks % 64 == 0:
            for d in [*spi.values(),*i2c.values()]: d.step()
        for d in i2c.values(): d.dma_service()
        for d in spi.values():
            d.polls += 1
            d.dma_service()
        phase = int.from_bytes(uc.mem_read(syms['I2cIrqTestPhase'], 4), 'little')
        ticks += 1
        dma_active = int.from_bytes(uc.mem_read(syms['DmaTestActive'], 4), 'little')
        if (not phase and not dma_active) or ticks % 16 or uc.reg_read(UC_ARM_REG_PRIMASK):
            return
        if dma_active:
            vector = dma.pending()
            if vector:
                pending_irq = syms[vector]
                uc.emu_stop()
                return
            for idx, dev in enumerate(spi.values()):
                if dev.reg[0] & 1 and dev.rd_reg(0x14, 4) & dev.reg[0x10]:
                    pending_irq = syms[f'SPI{idx+1}_IRQHandler']
                    uc.emu_stop()
                    return
        for idx, dev in enumerate(i2c.values()):
            cr1, flags = dev.reg[0], dev.f
            if ((flags['txis'] and cr1 & 2) or (flags['rxne'] and cr1 & 4) or
                (flags['stopf'] and cr1 & 32) or (flags['nackf'] and cr1 & 16) or
                ((flags['tc'] or flags['tcr']) and cr1 & 64) or (flags['addr'] and cr1 & 8)):
                pending_irq = syms[f'I2C{idx + 1}_EV_IRQHandler']
                uc.emu_stop()
                return
    u.hook_add(UC_HOOK_BLOCK, dispatch)
    pc = syms['main'] | 1
    for _ in range(100000):
        pending_irq = None
        u.emu_start(pc, 0x080ff000, count=50000000)
        if pending_irq is None:
            break
        saved = u.context_save()
        irq_active = True
        u.mem_write(syms['I2cIrqTestInside'], (1).to_bytes(4, 'little'))
        u.reg_write(UC_ARM_REG_LR, 0x080fe001)
        u.emu_start(pending_irq | 1, 0x080fe000, count=100000)
        assert u.reg_read(UC_ARM_REG_PC) == 0x080fe000, 'IRQ did not return'
        u.mem_write(syms['I2cIrqTestInside'], bytes(4))
        irq_active = False
        u.context_restore(saved)
        pc = u.reg_read(UC_ARM_REG_PC) | 1
    else:
        raise AssertionError('IRQ dispatch budget exhausted')
    for n in range(1, 5):
        for length in (1, 63, 64, 65, 255, 256, 600):
            expected = [0x55] + [(7 * k + 3) & 255 for k in range(length - 1)]
            assert irq_bytes[n * 1000 + length] == expected, (n, length, 'TX bytes differ')
    for n in range(1, 5):
        for case, at in enumerate((3, 70, 255)):
            expected = [0x55] + [(7 * k + 3) & 255 for k in range(at - 1)]
            assert irq_bytes[n * 1000 + 700 + case] == expected, (n, case, 'NACK bytes differ')
    assert u.reg_read(UC_ARM_REG_PC) == 0x080ff000, 'firmware timed out'
    assert u.reg_read(UC_ARM_REG_R0) == 0, f'bus_fw.cpp failed at line {u.reg_read(UC_ARM_REG_R0)}'
    assert not violations, violations[:10]
    for dev in spi.values():
        sent = [e[3] for e in log.ev if e[1:3] == (dev.name, 'X')]
        expected = [0xA5]*600
        for _ in range(2):
            expected += [0xA5]*1200 + [0x55] + [(k*3+1)&255 for k in range(1199)]
        assert sent == expected, (dev.name, 'master TX differs')
    for dev, spi_slave, idx, length in slave_checks:
        actual = dev.sent_by_slave[idx] if spi_slave else dev.got[idx]
        assert bytes(actual) == bytes((k*11+5)&255 for k in range(length)), (dev.name, 'slave TX differs')
    assert sum(dma.moved[:16]) > 10000 and sum(dma.moved[16:]) > 1000, dma.moved
    assert all(any(e[1] == d.name for e in log.ev) for d in [*spi.values(), *i2c.values()])


def main():
    with tempfile.TemporaryDirectory(prefix='h753-bus-') as tmp:
        tmp = Path(tmp)
        ld = tmp / 'bus.ld'
        # end is the heap start for the newlib sbrk. GCC 15 turns on the
        # libstdc++ assertions at -O0, and their failure path links abort and
        # malloc.
        ld.write_text('ENTRY(main)\nMEMORY { FLASH(rx): ORIGIN=0x08000000,LENGTH=2M\nRAM(rwx): ORIGIN=0x20000000,LENGTH=128K\nRAM4(rwx): ORIGIN=0x38000000,LENGTH=64K }\n'
                      'SECTIONS { .text : { KEEP(*(.text.DMA*IRQHandler)) KEEP(*(.text.BDMA*IRQHandler)) *(.text*) *(.rodata*) } > FLASH\n'
                      '.ARM.exidx : { *(.ARM.exidx*) } > FLASH\n'
                      '.bus_dma (NOLOAD) : ALIGN(32) { *(.bus_dma) } > RAM4\n.data : { *(.data*) } > RAM\n.bss : { *(.bss*) *(COMMON) } > RAM\nend=.; }')
        # Spaces around linker-script assignments are required by GNU ld.
        ld.write_text(ld.read_text().replace('=', ' = ').replace(',', ', '))
        for opt in ('-O0', '-O3'):
            flags = ['-mcpu=cortex-m7', '-mthumb', '-DSTM32H753xx', opt,
                     '-ffunction-sections', '-fdata-sections']
            inc = ['-I' + str(ROOT / p) for p in ('include', 'ARM/include', 'ARM/ST/include',
                   'ARM/ST/STM32H7xx/include', 'ARM/CMSIS/Core/Include')]
            objs = []
            for src in ('tests/stm32h7/bus_fw.cpp', 'ARM/ST/src/spi_stm32.cpp',
                        'ARM/ST/src/i2c_stm32.cpp', 'ARM/ST/src/dma_stm32.cpp', 'ARM/ST/src/quadspi_stm32.cpp',
                        'ARM/ST/src/iopincfg_stm32.cpp',
                        'src/coredev/spi.cpp', 'src/coredev/i2c.cpp', 'src/device_intrf.cpp'):
                obj = tmp / (Path(src).stem + '.o')
                run(['arm-none-eabi-g++', *flags, *inc, '-std=gnu++23', '-Wall', '-Werror',
                     '-fno-exceptions', '-fno-rtti', '-c', ROOT / src, '-o', obj])
                objs.append(obj)
            elf = tmp / 'bus.elf'
            run(['arm-none-eabi-g++', *flags, '-nostartfiles', '-Wl,--gc-sections',
                 '-T', ld, '--specs=nosys.specs', '--specs=nano.specs', *objs, '-o', elf])
            emulate(elf)
            print(opt + ': H753 I2C1-4/SPI1-6 polling/IRQ/DMA master and slave, DTCM staging, partial frames, reload/NACK PASS', flush=True)
    print('Hardware validation pending.')


if __name__ == '__main__':
    main()
