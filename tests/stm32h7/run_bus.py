#!/usr/bin/env python3
"""Compile H753 shared I2C/SPI and exercise all instances in a register model.

Requires GNU Arm GCC, unicorn and pyelftools. Reuses the common I2C v2 and
SPI CFG register models from STM32WBA; no DMA or physical timing is modeled.
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


def run(args):
    subprocess.run([str(a) for a in args], check=True)


def emulate(path):
    u = Uc(UC_ARCH_ARM, UC_MODE_THUMB | UC_MODE_MCLASS)
    for base, size in ((0x08000000, 0x200000), (0x20000000, 0x20000), (0xe0000000, 0x100000)):
        u.mem_map(base, size)
    with path.open('rb') as f:
        elf = ELFFile(f)
        for seg in elf.iter_segments():
            if seg['p_type'] == 'PT_LOAD':
                u.mem_write(seg['p_vaddr'], seg.data())
        syms = {s.name: s['st_value'] for s in elf.get_section_by_name('.symtab').iter_symbols()}
    log, violations = Log(), []
    spi = {a: Spi('SPI' + str(i + 1), log, violations) for i, a in enumerate(
        (0x40013000, 0x40003800, 0x40003c00, 0x40013400, 0x40015000, 0x58001400))}
    i2c = {a: I2c('I2C' + str(i + 1), a, log, violations) for i, a in enumerate(
        (0x40005400, 0x40005800, 0x40005c00, 0x58001c00))}
    for d in i2c.values():
        d.slaves[0x50] = Slave(0x50)
        for addr, at in ((0x51, 3), (0x52, 70), (0x53, 255)):
            d.slaves[addr] = Slave(addr, refuse_at=at)
    regs = collections.defaultdict(int)
    regs[0x58024400] = 5  # HSI on and ready, divide by one

    def read(uc, off, size, base):
        a = base + off
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
        if a & ~255 in spi:
            spi[a & ~255].wr_reg(a & 255, size, value)
        elif a & ~255 in i2c:
            if phase and (a & 255) == 0x28:
                assert irq_active, 'interrupt TX wrote TXDR from polling code'
                irq_bytes[phase].append(value & 255)
            i2c[a & ~255].wr_reg(a & 255, value)
        else:
            regs[a] = value

    for base, size in ((0x40000000, 0x20000), (0x52005000, 0x1000), (0x58000000, 0x30000)):
        u.mmio_map(base, size, read, base, write, base)
    u.reg_write(UC_ARM_REG_SP, 0x2001fff0)
    u.reg_write(UC_ARM_REG_LR, 0x080ff001)
    # Dispatch enabled I2C vectors between foreground basic blocks. Save and
    # restore the interrupted CPU context, preserving the handler's RAM writes.
    # This models preemption, not NVIC timing or Cortex-M exception entry.
    ticks = 0
    def dispatch(uc, address, size, ctx):
        nonlocal pending_irq, phase, ticks
        if irq_active:
            return
        phase = int.from_bytes(uc.mem_read(syms['I2cIrqTestPhase'], 4), 'little')
        ticks += 1
        if not phase or ticks % 16 or uc.reg_read(UC_ARM_REG_PRIMASK):
            return
        for idx, dev in enumerate(i2c.values()):
            cr1, flags = dev.reg[0], dev.f
            if ((flags['txis'] and cr1 & 2) or (flags['rxne'] and cr1 & 4) or
                (flags['stopf'] and cr1 & 32) or (flags['nackf'] and cr1 & 16) or
                ((flags['tc'] or flags['tcr']) and cr1 & 64)):
                pending_irq = syms[f'I2C{idx + 1}_EV_IRQHandler']
                uc.emu_stop()
                return
    u.hook_add(UC_HOOK_BLOCK, dispatch)
    pc = syms['main'] | 1
    for _ in range(20000):
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
    assert all(any(e[1] == d.name for e in log.ev) for d in [*spi.values(), *i2c.values()])


def main():
    with tempfile.TemporaryDirectory(prefix='h753-bus-') as tmp:
        tmp = Path(tmp)
        ld = tmp / 'bus.ld'
        # end is the heap start for the newlib sbrk. GCC 15 turns on the
        # libstdc++ assertions at -O0, and their failure path links abort and
        # malloc.
        ld.write_text('ENTRY(main)\nMEMORY { FLASH(rx): ORIGIN=0x08000000,LENGTH=2M\nRAM(rwx): ORIGIN=0x20000000,LENGTH=128K }\n'
                      'SECTIONS { .text : { *(.text*) *(.rodata*) } > FLASH\n'
                      '.ARM.exidx : { *(.ARM.exidx*) } > FLASH\n'
                      '.data : { *(.data*) } > RAM\n.bss : { *(.bss*) *(COMMON) } > RAM\nend=.; }')
        # Spaces around linker-script assignments are required by GNU ld.
        ld.write_text(ld.read_text().replace('=', ' = ').replace(',', ', '))
        for opt in ('-O0', '-O3'):
            flags = ['-mcpu=cortex-m7', '-mthumb', '-DSTM32H753xx', opt,
                     '-ffunction-sections', '-fdata-sections']
            inc = ['-I' + str(ROOT / p) for p in ('include', 'ARM/include', 'ARM/ST/include',
                   'ARM/ST/STM32H7xx/include', 'ARM/CMSIS/Core/Include')]
            objs = []
            for src in ('tests/stm32h7/bus_fw.cpp', 'ARM/ST/src/spi_stm32.cpp',
                        'ARM/ST/src/i2c_stm32.cpp', 'ARM/ST/src/quadspi_stm32.cpp',
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
            print(opt + ': H753 I2C1-4/SPI1-6 polling, interrupts, rates and I2C reload/NACK and IRQ TX lengths 1..600 PASS', flush=True)
    print('Hardware validation pending.')


if __name__ == '__main__':
    main()
