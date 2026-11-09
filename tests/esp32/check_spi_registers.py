#!/usr/bin/env python3
"""Check GP-SPI2 definitions against a local ESP-IDF v5.5.1 checkout.

Usage: python3 tests/esp32/check_spi_registers.py /path/to/esp-idf
Compiles independent reference values as C and C++ assertions. No hardware access.
"""
import argparse
from pathlib import Path
import re
import subprocess
import tempfile

from check_i2c_registers import number, offset, irq

ROOT = Path(__file__).resolve().parents[2]
INCLUDE = ROOT / 'RISCV/Espressif/include'


def check(chip, idf, temporary, prefix):
    target = idf / 'components/soc' / chip
    registers = (target / 'register/soc/spi_reg.h').read_text()
    signals = (target / 'include/soc/gpio_sig_map.h').read_text()
    interrupts = (target / 'include/soc/interrupts.h').read_text()
    caps = (target / 'include/soc/soc_caps.h').read_text()
    pcr = (target / 'register/soc/pcr_reg.h').read_text() if chip not in ('esp32c2', 'esp32c3') else ''
    macros = subprocess.check_output([prefix + 'gcc', '-E', '-dM', '-D' + chip.upper(), '-I' + str(INCLUDE),
        '-include', 'esp32xx_spi.h', '-x', 'c', '/dev/null'], text=True)
    names = re.findall(r'^#define (ESP32_SPI_\w+)(?:\s|\()', macros, re.M)
    declared = set(re.findall(r'^#\s*define (ESP32_SPI_\w+)', (INCLUDE / 'esp32xx_spi.h').read_text(), re.M))
    names = [name for name in names if name in declared]
    expected = {}
    aliases = {
        'MST_WFULL_ERR_END': 'MST_WFULL_ERR_END_EN',
        'MST_REMPTY_ERR_END': 'MST_REMPTY_ERR_END_EN',
        'INFIFO_FULL_ERR': 'DMA_INFIFO_FULL_ERR',
        'OUTFIFO_EMPTY_ERR': 'DMA_OUTFIFO_EMPTY_ERR',
        'DMA_SEG_TRANS': 'DMA_SEG_TRANS_DONE',
        'BUF_ADDR_ERR': 'SLV_BUF_ADDR_ERR',
        'CMD_ERR': 'SLV_CMD_ERR',
    }
    for name in names:
        short = name.removeprefix('ESP32_SPI_')
        if short.endswith('_OFFSET'):
            reg = short.removesuffix('_OFFSET') + '_REG'
            expected[name] = offset(pcr, reg.replace('PCR_', 'PCR_SPI2_', 1)) if reg.startswith('PCR_') else offset(registers, 'SPI_' + reg)
        elif short.endswith(('_Pos', '_Msk')):
            field = re.sub(r'^(CMD|CTRL|CLOCK|USER1?|USER2|MS_DLEN|MISC|CLK_GATE)_', '', short[:-4])
            field = aliases.get(field, field)
            reference = pcr if field.startswith('PCR_') else registers
            field = field.replace('PCR_', 'PCR_SPI2_', 1) if field.startswith('PCR_') else 'SPI_' + field
            shift = number(reference, field + '_S')
            expected[name] = shift if short.endswith('_Pos') else number(reference, field + '_V') << shift
        elif short.startswith('DMA_INT_') and not short.endswith('_REG'):
            field = aliases.get(short[8:], short[8:])
            expected[name] = 1 << number(registers, 'SPI_' + field + '_INT_RAW_S')
    source = {'esp32c2': 'ETS_SPI2_INTR_SOURCE', 'esp32c3': 'ETS_SPI2_INTR_SOURCE',
              'esp32c5': 'ETS_GPSPI2_INTR_SOURCE', 'esp32c6': 'ETS_GSPI2_INTR_SOURCE',
              'esp32c61': 'ETS_GPSPI2_INTR_SOURCE'}[chip]
    expected.update({
        'ESP32_SPI2_BASE': number((target / 'register/soc/reg_base.h').read_text(), 'DR_REG_SPI2_BASE'),
        'ESP32_SPI_INTR_SOURCE_SPI2': irq(interrupts, source),
        'ESP32_SPI_W_FIFO_BYTES': number(caps, 'SOC_SPI_MAXIMUM_BUFFER_SIZE'),
        'ESP32_SPI_W_REG_COUNT': (offset(registers, 'SPI_W15_REG') - offset(registers, 'SPI_W0_REG')) // 4 + 1,
    })
    for line, field in (('SCK', 'FSPICLK'), ('MISO', 'FSPIQ'), ('MOSI', 'FSPID')):
        expected['ESP32_SPI_' + line + '_SIGNAL'] = number(signals, field + '_IN_IDX')
        assert number(signals, field + '_IN_IDX') == number(signals, field + '_OUT_IDX')
    unchecked = set(names) - expected.keys()
    unchecked -= {name for name in names if name.endswith('_REG')}
    if unchecked:
        raise ValueError('Unchecked SPI definitions: ' + str(sorted(unchecked)))
    for language, compiler, suffix in (('c', 'gcc', '.c'), ('c++', 'g++', '.cpp')):
        assertion = '_Static_assert' if language == 'c' else 'static_assert'
        source = '#include "esp32xx_spi.h"\n'
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
    with tempfile.TemporaryDirectory(prefix='esp32-spi-registers-') as directory:
        for chip in ('esp32c2', 'esp32c3', 'esp32c5', 'esp32c6', 'esp32c61'):
            check(chip, args.idf, Path(directory), args.toolchain_prefix)


if __name__ == '__main__':
    main()
