#!/usr/bin/env python3
"""Exercise the interrupt-mask checks in the compiled sDelay entry point."""
from pathlib import Path
import subprocess
import tempfile

from elftools.elf.elffile import ELFFile
from unicorn import Uc, UC_ARCH_ARM, UC_MODE_MCLASS, UC_MODE_THUMB
from unicorn.arm_const import (UC_ARM_REG_BASEPRI, UC_ARM_REG_FAULTMASK,
                              UC_ARM_REG_PRIMASK, UC_ARM_REG_IPSR, UC_ARM_REG_PC,
                              UC_ARM_REG_LR, UC_ARM_REG_R0, UC_ARM_REG_SP)

ROOT = Path(__file__).resolve().parents[2]
RETURN = 0x8000



def check(cpu, directory):
    image = directory / (cpu + '.elf')
    # Stop the unmasked control at the first call into the timer driver.
    stubs = directory / 'driver.S'
    symbols = ('TimerGetLowFreqDevCount', 'TimerGetHighFreqDevCount', 'TimerGetHighFreqDevNo',
               'TimerInit', 'SystemLowFreqClockEnable', 'SystemLowFreqClockDisable',
               'SystemLowFreqClockStartTime', 'SystemHighFreqClockEnable',
               'SystemHighFreqClockDisable', 'SystemHighFreqClockStartTime')
    stubs.write_text('.thumb\n' + ''.join(
        f'.global {name}\n.thumb_func\n{name}: bkpt #0\n' for name in symbols))
    subprocess.run([
        'arm-none-eabi-g++', '-mcpu=' + cpu, '-mthumb', '-Os', '-nostdlib', '-fno-exceptions',
        '-Iinclude', '-IARM/include', '-IARM/CMSIS/Core/Include',
        'src/sdelay.cpp', stubs, '-Wl,-e,sDelay,-Ttext=0x1000,-Tbss=0x6000',
        '-lc', '-lgcc',
        '-o', image], cwd=ROOT, check=True)
    with image.open('rb') as stream:
        elf = ELFFile(stream)
        code = elf.get_section_by_name('.text').data()
        entry = elf.header.e_entry
        query = elf.get_section_by_name('.symtab').get_symbol_by_name('TimerGetLowFreqDevCount')[0]
        first_timer_query = query.entry.st_value & ~1

    cases = [(None, 0, 1), (UC_ARM_REG_PRIMASK, 1, 1), (UC_ARM_REG_IPSR, 16, 1)]
    if cpu in ('cortex-m4', 'cortex-m33'):
        cases.extend([(UC_ARM_REG_BASEPRI, 0x20, 1), (UC_ARM_REG_FAULTMASK, 1, 1),
                      (UC_ARM_REG_BASEPRI, 0x20, 0)])

    for register, mask, seconds in cases:
        arm = Uc(UC_ARCH_ARM, UC_MODE_MCLASS | UC_MODE_THUMB)
        arm.mem_map(0x1000, 0x9000)
        arm.mem_write(0x1000, code)
        arm.reg_write(UC_ARM_REG_SP, 0x7800)
        arm.reg_write(UC_ARM_REG_LR, RETURN | 1)
        arm.reg_write(UC_ARM_REG_R0, seconds)
        if register is not None:
            arm.reg_write(register, mask)
        stop = RETURN if register is not None else first_timer_query
        arm.emu_start(entry | 1, stop, count=250)
        assert arm.reg_read(UC_ARM_REG_PC) == stop, (cpu, register, seconds)
        if stop == RETURN:
            assert arm.reg_read(UC_ARM_REG_R0) == int(seconds == 0)
            assert arm.reg_read(register) == mask
    print(cpu, 'PASS')


if __name__ == '__main__':
    with tempfile.TemporaryDirectory() as folder:
        for core in ('cortex-m0plus', 'cortex-m23', 'cortex-m4', 'cortex-m33'):
            check(core, Path(folder))
