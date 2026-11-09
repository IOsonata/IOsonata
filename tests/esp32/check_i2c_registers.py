#!/usr/bin/env python3
"""Compare HP I2C definitions with a local ESP-IDF v5.5.1 checkout.

Usage: python3 tests/esp32/check_i2c_registers.py /path/to/esp-idf
Uses C and C++ compile-time assertions; does not access hardware or the network.
The reference checkout is an independent source, not a copy of IOsonata constants.
"""
import argparse
from pathlib import Path
import re
import subprocess
import tempfile

ROOT = Path(__file__).resolve().parents[2]
INCLUDE = ROOT / 'RISCV/Espressif/include'


def number(text, name):
    match = re.search(r'^#define\s+' + re.escape(name) + r'\s+\(?\s*(0x[0-9a-fA-F]+|[0-9]+)[UuLl]*\)?(?:\s|$)', text, re.M)
    if match is None:
        raise ValueError('Missing reference definition: ' + name)
    return int(match[1], 0)


def offset(text, name):
    match = re.search(r'^#define\s+' + re.escape(name) + r'(?:\([^\n]*?\))?\s+[^\n]*?\+\s*(0x[0-9a-fA-F]+)', text, re.M)
    if match is None:
        raise ValueError('Missing reference offset: ' + name)
    return int(match[1], 16)


def irq(text, name):
    value = -1
    for entry, explicit in re.findall(r'^\s*(ETS_\w+)(?:\s*=\s*(\d+))?\s*,', text, re.M):
        value = int(explicit) if explicit else value + 1
        if entry == name:
            return value
    raise ValueError('Missing interrupt: ' + name)


def check(chip, idf, temporary, prefix):
    target = idf / 'components/soc' / chip
    registers = (target / 'register/soc/i2c_reg.h').read_text()
    caps = (target / 'include/soc/soc_caps.h').read_text()
    signals = (target / 'include/soc/gpio_sig_map.h').read_text()
    interrupts = (target / 'include/soc/interrupts.h').read_text()
    lowlevel = (idf / 'components/hal' / chip / 'include/hal/i2c_ll.h').read_text()
    pcr = (target / 'register/soc/pcr_reg.h').read_text() if chip not in ('esp32c2', 'esp32c3') else ''
    macros = subprocess.check_output([prefix + 'gcc', '-E', '-dM', '-D' + chip.upper(), '-I' + str(INCLUDE),
        '-include', 'esp32xx_i2c.h', '-x', 'c', '/dev/null'], text=True)
    names = re.findall(r'^#define (ESP32_I2C_\w+)(?:\s|\()', macros, re.M)
    expected = {}
    for name in names:
        short = name.removeprefix('ESP32_I2C_')
        if short.endswith('_OFFSET'):
            reg = short.removesuffix('_OFFSET') + '_REG'
            if reg.startswith('PCR_'):
                expected[name] = offset(pcr, reg.replace('PCR_', 'PCR_I2C_', 1))
            else:
                expected[name] = offset(registers, 'I2C_' + reg)
        elif short.endswith(('_Pos', '_Msk')):
            field = re.sub(r'^(CTR|SR|FIFO_CONF|CLK_CONF)_', '', short[:-4])
            if field.startswith('COMD_'):
                continue
            reference = pcr if field.startswith('PCR_') else registers
            field = field.replace('PCR_', 'PCR_I2C_', 1) if field.startswith('PCR_') else 'I2C_' + field
            shift = number(reference, field + '_S')
            expected[name] = shift if short.endswith('_Pos') else number(reference, field + '_V') << shift
        elif short.startswith('INT_') and not short.endswith('_REG'):
            expected[name] = 1 << number(registers, 'I2C_' + short[4:] + '_INT_RAW_S')
        elif short.startswith('OPCODE_'):
            field = short[7:].replace('RSTART', 'RESTART')
            expected[name] = number(lowlevel, 'I2C_LL_CMD_' + field)
    expected.update({
        'ESP32_I2C0_BASE': number((target / 'register/soc/reg_base.h').read_text(), 'DR_REG_I2C_EXT_BASE' if chip in ('esp32c2', 'esp32c3', 'esp32c6') else 'DR_REG_I2C_BASE'),
        'ESP32_I2C_FIFO_DEPTH': number(caps, 'SOC_I2C_FIFO_LEN'),
        'ESP32_I2C_CMD_QUEUE_DEPTH': number(caps, 'SOC_I2C_CMD_REG_NUM'),
        'ESP32_I2C_INTR_SOURCE_I2C0': irq(interrupts, 'ETS_I2C_EXT0_INTR_SOURCE'),
    })
    for line in ('SDA', 'SCL'):
        expected['ESP32_I2C_' + line + '_SIGNAL'] = number(signals, 'I2CEXT0_' + line + '_IN_IDX')
        assert number(signals, 'I2CEXT0_' + line + '_IN_IDX') == number(signals, 'I2CEXT0_' + line + '_OUT_IDX')
    if pcr:
        for field in ('CLK_EN', 'RST_EN', 'SCLK_EN'):
            expected['ESP32_I2C_PCR_' + field] = 1 << number(pcr, 'PCR_I2C_' + field + '_S')
    if 'ESP32_I2C_INTR_SOURCE_LP_I2C' in names:
        expected['ESP32_I2C_INTR_SOURCE_LP_I2C'] = irq(interrupts, 'ETS_LP_I2C_INTR_SOURCE')
    # LL command structure: byte count, ACK check/expected/value, opcode, reserved, done.
    for field, value in {'BYTE_NUM_Pos': 0, 'BYTE_NUM_Msk': 255, 'ACK_EN': 256,
                         'ACK_EXP': 512, 'ACK_VAL': 1024, 'OPCODE_Pos': 11,
                         'OPCODE_Msk': 7 << 11, 'COMMAND_Pos': 0,
                         'COMMAND_Msk': 0x3fff, 'DONE_Pos': 31, 'DONE_Msk': 1 << 31}.items():
        expected['ESP32_I2C_COMD_' + field] = value
    for language, compiler, suffix in (('c', 'gcc', '.c'), ('c++', 'g++', '.cpp')):
        assertion = '_Static_assert' if language == 'c' else 'static_assert'
        source = '#include "esp32xx_i2c.h"\n'
        source += '\n'.join(f'{assertion}({name} == {value}UL, "{name}");' for name, value in sorted(expected.items()))
        path = temporary / (chip + suffix)
        path.write_text(source + '\n')
        subprocess.run([prefix + compiler, '-Werror', '-fsyntax-only', '-D' + chip.upper(), '-I' + str(INCLUDE), str(path)], check=True)
    print(chip, len(expected), 'register assertions PASS in C and C++')


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('idf', type=Path, help='ESP-IDF v5.5.1 source directory')
    parser.add_argument('--toolchain-prefix', default='', help='Compiler path/prefix ending in a hyphen')
    args = parser.parse_args()
    with tempfile.TemporaryDirectory(prefix='esp32-i2c-registers-') as directory:
        for chip in ('esp32c2', 'esp32c3', 'esp32c5', 'esp32c6', 'esp32c61'):
            check(chip, args.idf, Path(directory), args.toolchain_prefix)


if __name__ == '__main__':
    main()
