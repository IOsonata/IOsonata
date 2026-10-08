#!/usr/bin/env python3
"""Production drivers/vector wrappers with ST definitions and a host register model."""
from pathlib import Path
import subprocess, tempfile
root=Path(__file__).resolve().parents[2]
with tempfile.TemporaryDirectory() as work:
    work=Path(work)
    vector=(root/'ARM/ST/STM32F0xx/STM32F030x8/lib/src/Vectors_STM32F030x8.c').read_text()
    # Compile the production weak IRQ wrappers, excluding the target-address vector table.
    vector=vector[:vector.index('/**\n * This interrupt vector')]
    (work/'vectors.c').write_text(vector)
    subprocess.run(['gcc','-c',str(work/'vectors.c'),'-o',str(work/'vectors.o')],check=True)
    for override in [False, True]:
        exe=str(work/('timer-override' if override else 'timer'))
        opts=['-DTEST_IRQ_OVERRIDE'] if override else []
        subprocess.run(['g++','-std=c++17','-O2','-fsanitize=undefined','-fno-sanitize-recover=all',
            '-DSTM32F030x8','-include','initializer_list',*opts,
            '-I'+str(root/'tests/stm32f0/timer_shim'),'-I'+str(root/'tests/stm32f0/shim'),'-I'+str(root/'include'),
            '-I'+str(root/'ARM/ST/STM32F0xx/include'),str(root/'tests/stm32f0/timer_test.cpp'),
            str(work/'vectors.o'),'-o',exe],check=True)
        subprocess.run([exe],check=True)
