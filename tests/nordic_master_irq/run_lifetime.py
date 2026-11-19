#!/usr/bin/env python3
"""Run Nordic caller-buffer checks with host register/task models.

Production functions are extracted for host testing; 32-bit pointer-to-DMA
casts are adapted to the host pointer width in the temporary test copy.
The model checks IRQ ordering and buffer ownership, not MCU register layout or
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
    'nRFxSPIStartRx', 'nRFxSPIRxData', 'nRFxSPIRxBlockSize', 'nRFxSPIDmaNextBlock', 'nRFxSPIDmaEnd',
    'nRFxSPIRxDataDma', 'nRFxSPIStopRx', 'nRFxSPIStartTx', 'nRFxSPITxData',
    'nRFxSPITxDataDma', 'nRFxSPIStopTx', 'nRFxSPIAsyncTxAbortEnd', 'SPI_IRQHandler'])]:
    parts += [function(text, name) for name in names]

# Hardware DMA PTR is 32-bit. The host model stores a native pointer so
# address comparisons work on 64-bit macOS and Linux. Only pointer-to-DMA
# casts are adjusted in the generated host copy, never in production sources.
parts = [re.sub(r'\(uint32_t\)(?=\s*(?:&|pData\b|pBuff\b|pDev->TxBuf\b|dev->TxBuf\b))',
                '(uintptr_t)', part) for part in parts]

with tempfile.TemporaryDirectory() as directory:
    tmp = Path(directory)
    (tmp / 'driver_lifetime.inc').write_text('\n'.join(parts))
    for maximum, capacity in [(255, 64), (65535, 64), (255, 512)]:
        for opt in ['-O0', '-O2']:
            binary = tmp / 'lifetime'
            command = [args.cxx, '-std=gnu++17', opt, '-g', '-fpermissive',
                       '-fno-pie', '-fsanitize=address,undefined',
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
