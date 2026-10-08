#!/usr/bin/env python3
"""Run the shared SLIP RX example with the production decoder and a fake UART."""
from pathlib import Path
import subprocess, tempfile
root = Path(__file__).resolve().parents[2]
with tempfile.TemporaryDirectory(prefix='f030-slip-example-') as tmp:
    out = Path(tmp)
    (out / 'board.h').write_text('''#include "coredev/iopincfg.h"
#define UART_DEVNO 0
#define UART_PINS {{0, 10, 0x12, IOPINDIR_INPUT, IOPINRES_NONE, IOPINTYPE_NORMAL}}
''')
    exe = out / 'test'
    subprocess.run(['g++', '-std=c++23', '-O1', '-g', '-fsanitize=undefined',
                    '-fno-sanitize-recover=all', '-ffunction-sections', '-fdata-sections',
                    '-I' + str(out), '-I' + str(root / 'include'),
                    str(root / 'tests/stm32f0/slip_rx_example_test.cpp'),
                    str(root / 'src/slip_intrf.cpp'), str(root / 'src/device_intrf.cpp'),
                    str(root / 'src/prbs.c'), '-Wl,--gc-sections', '-o', str(exe)], check=True)
    subprocess.run([str(exe)], check=True)
    subprocess.run([str(exe), 'fragment'], check=True)
