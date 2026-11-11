#!/usr/bin/env python3
"""Check C6 LP I2C and pin routing against a local ESP-IDF v5.5.1 checkout."""
import argparse
from pathlib import Path
import subprocess
import tempfile
from check_i2c_registers import number, offset, ROOT, INCLUDE


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('idf', type=Path)
    parser.add_argument('--toolchain-prefix', default='')
    args = parser.parse_args()
    soc = args.idf / 'components/soc/esp32c6/register/soc'
    regs = (soc / 'lp_i2c_reg.h').read_text()
    pins = (soc / 'lp_io_reg.h').read_text()
    aon = (soc / 'lp_aon_reg.h').read_text()
    clocks = (soc / 'lp_clkrst_reg.h').read_text()
    peri = (soc / 'lpperi_reg.h').read_text()
    bases = (soc / 'reg_base.h').read_text()
    caps = (args.idf / 'components/soc/esp32c6/include/soc/soc_caps.h').read_text()
    expected = {'ESP32_LP_I2C_BASE': number(bases, 'DR_REG_LP_I2C_BASE'),
                'ESP32_LP_IOMUX_BASE': number(bases, 'DR_REG_LP_IO_BASE'),
                'ESP32_LP_I2C_FIFO_DEPTH': number(caps, 'SOC_LP_I2C_FIFO_LEN')}
    for reg in ('SCL_LOW_PERIOD', 'CTR', 'SR', 'TO', 'FIFO_CONF', 'DATA',
                'INT_RAW', 'INT_CLR', 'INT_ENA', 'INT_STATUS', 'SDA_HOLD',
                'SDA_SAMPLE', 'SCL_HIGH_PERIOD', 'SCL_START_HOLD',
                'SCL_RSTART_SETUP', 'SCL_STOP_HOLD', 'SCL_STOP_SETUP',
                'FILTER_CFG', 'COMD0'):
        expected['ESP32_I2C_' + reg + '_OFFSET'] = offset(regs, 'LP_I2C_' + reg + '_REG')
    for group, fields in {
        'CTR': ('SDA_FORCE_OUT', 'SCL_FORCE_OUT', 'TRANS_START', 'CLK_EN',
                'ARBITRATION_EN', 'FSM_RST', 'CONF_UPGATE'),
        'SR': ('BUS_BUSY',),
        'FIFO_CONF': ('RX_FIFO_RST', 'TX_FIFO_RST', 'FIFO_PRT_EN'),
    }.items():
        for field in fields:
            expected[f'ESP32_I2C_{group}_{field}_Msk'] = (
                number(regs, 'LP_I2C_' + field + '_V') << number(regs, 'LP_I2C_' + field + '_S'))
    for field in ('NACK', 'ARBITRATION_LOST', 'TIME_OUT', 'SCL_ST_TO',
                  'SCL_MAIN_ST_TO', 'RXFIFO_OVF', 'MST_TXFIFO_UDF',
                  'END_DETECT', 'TRANS_COMPLETE'):
        expected['ESP32_I2C_INT_' + field] = 1 << number(regs, 'LP_I2C_' + field + '_INT_RAW_S')
    for ours, ref, text in (
        ('ESP32_LP_I2C_CLK_EN_Msk', 'LPPERI_LP_EXT_I2C_CK_EN', peri),
        ('ESP32_LP_I2C_RESET_EN_Msk', 'LPPERI_LP_EXT_I2C_RESET_EN', peri),
        ('ESP32_LP_IO_CLK_EN_Msk', 'LPPERI_LP_IO_CK_EN', peri),
        ('ESP32_LP_I2C_CLK_SEL_Msk', 'LP_CLKRST_LP_I2C_CLK_SEL', clocks),
        ('ESP32_LP_IO_PAD_DRIVER_Msk', 'LP_IO_LP_GPIO6_PAD_DRIVER', pins),
        ('ESP32_LP_IO_FUN_WPD_Msk', 'LP_IO_LP_GPIO6_FUN_WPD', pins),
        ('ESP32_LP_IO_FUN_WPU_Msk', 'LP_IO_LP_GPIO6_FUN_WPU', pins),
        ('ESP32_LP_IO_FUN_IE_Msk', 'LP_IO_LP_GPIO6_FUN_IE', pins),
        ('ESP32_LP_IO_FUN_DRV_Msk', 'LP_IO_LP_GPIO6_FUN_DRV', pins),
    ):
        expected[ours] = number(text, ref + '_V') << number(text, ref + '_S')
    expected['ESP32_LP_IO_FUN_DRV_Pos'] = number(pins, 'LP_IO_LP_GPIO6_FUN_DRV_S')
    expected['ESP32_LP_IO_MCU_SEL_Pos'] = number(pins, 'LP_IO_LP_GPIO6_MCU_SEL_S')
    # Turn register access macros into address expressions for these checks.
    for ours, base, reg, text in (
        ('ESP32_LP_PERI_CLK_EN_REG', 'DR_REG_LPPERI_BASE', 'LPPERI_CLK_EN_REG', peri),
        ('ESP32_LP_PERI_RESET_EN_REG', 'DR_REG_LPPERI_BASE', 'LPPERI_RESET_EN_REG', peri),
        ('ESP32_LP_CLKRST_PERI_REG', 'DR_REG_LP_CLKRST_BASE', 'LP_CLKRST_LPPERI_REG', clocks),
        ('ESP32_LP_AON_GPIO_MUX_REG', 'DR_REG_LP_AON_BASE', 'LP_AON_GPIO_MUX_REG', aon),
        ('ESP32_LP_IO_OUT_W1TS_REG', 'DR_REG_LP_IO_BASE', 'LP_IO_OUT_DATA_W1TS_REG', pins),
        ('ESP32_LP_IO_ENABLE_W1TS_REG', 'DR_REG_LP_IO_BASE', 'LP_IO_OUT_ENABLE_W1TS_REG', pins),
        ('ESP32_LP_IO_ENABLE_W1TC_REG', 'DR_REG_LP_IO_BASE', 'LP_IO_OUT_ENABLE_W1TC_REG', pins),
    ):
        expected[ours] = number(bases, base) + offset(text, reg)
    for pin in (6, 7):
        for ours, reg in (('PIN', 'PIN'), ('MUX', 'GPIO')):
            expected[f'ESP32_LP_IO_{ours}_REG({pin})'] = number(bases, 'DR_REG_LP_IO_BASE') + offset(pins, f'LP_IO_{reg}{pin}_REG')
    with tempfile.TemporaryDirectory(prefix='esp32-lp-registers-') as directory:
        for suffix, compiler, assertion in (('c', 'gcc', '_Static_assert'), ('cpp', 'g++', 'static_assert')):
            source = '#define ESP32_REG32(addr) (addr)\n#include "esp32xx_i2c.h"\n'
            source += '\n'.join(f'{assertion}({name} == {value}UL, "{name}");' for name, value in expected.items())
            path = Path(directory) / ('lp.' + suffix)
            path.write_text(source + '\n')
            subprocess.run([args.toolchain_prefix + compiler, '-Werror', '-fsyntax-only', '-DESP32C6', '-I' + str(INCLUDE), str(path)], check=True)
    print(len(expected), 'C6 LP I2C and GPIO register assertions PASS in C and C++')


if __name__ == '__main__':
    main()
