#!/usr/bin/env python3
"""Run Nordic caller-buffer checks with host register/task models.

Only the function bodies are extracted; their source is not rewritten. The
model checks IRQ ordering and buffer ownership, not MCU register layout or
electrical bus behavior. build.py remains the Arm/MDK integration check.
"""
from pathlib import Path
import argparse
import re
import subprocess
import tempfile

root = Path(__file__).resolve().parents[2]
parser = argparse.ArgumentParser()
parser.add_argument('--cxx', default='g++')
args = parser.parse_args()

def function(text, name):
    match = re.search(r'^.*\b' + name + r'\([^;]*?\)\s*\{', text, re.M)
    if not match:
        raise RuntimeError('Function not found: ' + name)
    end = text.index('{', match.start()) + 1
    depth = 1
    while depth:
        depth += (text[end] == '{') - (text[end] == '}')
        end += 1
    return text[match.start():end] + '\n'

i2c = (root / 'ARM/Nordic/src/i2c_nrfx.cpp').read_text()
spi = (root / 'ARM/Nordic/src/spi_nrfx.cpp').read_text()
parts = []
for text, names in [(i2c, [
    'nRFxI2CAsyncTxBuff', 'nRFxI2CStartRx', 'nRFxI2CDmaNextBlock',
    'nRFxI2CRxDataDMA', 'nRFxI2CRxData', 'nRFxI2CStopRx', 'nRFxI2CStartTx',
    'nRFxI2CTxDataDMA', 'nRFxI2CTxData', 'nRFxI2CStopTx', 'nRFxI2CAsyncEnd',
    'nRFxI2CTwimIrq', 'nRFxI2CTwiIrq']), (spi, [
    'nRFxSPIDmaTimeout', 'nRFxSPIAsyncTxBuff', 'nRFxSPIAsyncTxWait',
    'nRFxSPIStartRx', 'nRFxSPIRxData', 'nRFxSPIDmaNextBlock', 'nRFxSPIDmaEnd',
    'nRFxSPIRxDataDma', 'nRFxSPIStopRx', 'nRFxSPIStartTx', 'nRFxSPITxData',
    'nRFxSPITxDataDma', 'nRFxSPIStopTx', 'nRFxSPIAsyncTxAbortEnd', 'SPI_IRQHandler'])]:
    parts += [function(text, name) for name in names]

with tempfile.TemporaryDirectory() as directory:
    tmp = Path(directory)
    (tmp / 'driver_lifetime.inc').write_text('\n'.join(parts))
    for maximum, capacity in [(255, 64), (65535, 64), (255, 512)]:
        for opt in ['-O0', '-O2']:
            binary = tmp / 'lifetime'
            command = [args.cxx, '-std=gnu++17', opt, '-g', '-fpermissive',
                       '-fno-pie', '-no-pie', '-fsanitize=address,undefined',
                       '-fno-omit-frame-pointer', f'-DTEST_DMA_MAX={maximum}',
                       f'-DNRFX_I2C_TXBUF_SIZE={capacity}', '-I' + str(tmp),
                       '-I' + str(root / 'include'),
                       str(root / 'tests/nordic_master_irq/lifetime_test.cpp'),
                       str(root / 'src/device_intrf.cpp'), '-o', str(binary)]
            result = subprocess.run(command, capture_output=True, text=True)
            if result.returncode:
                raise RuntimeError(result.stderr)
            result = subprocess.run([str(binary)], capture_output=True, text=True)
            print(f'MAXCNT={maximum}, TXBUF={capacity}, {opt}: {result.stdout.strip()}')
            if result.returncode:
                raise RuntimeError(result.stderr or 'Host lifetime check failed')
