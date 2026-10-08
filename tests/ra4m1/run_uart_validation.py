#!/usr/bin/env python3
"""RA4M1 UART host/model and reduced-header ARM compile checks; no hardware.

Run from a full IOsonata checkout to additionally use the production CFifo.
The standalone package uses a clearly labeled FIFO test double.
"""
import os
from pathlib import Path
import shutil
import sys
from run_validation import ROOT, HERE, TARGET, BUILD, run


def main():
    cc = os.environ.get('CLANG') or shutil.which('clang')
    cxx = os.environ.get('CLANGXX') or shutil.which('clang++')
    if not cc or not cxx:
        raise SystemExit('Clang and Clang++ are required.')
    BUILD.mkdir(exist_ok=True)
    print(run([sys.executable, HERE/'run_validation.py']), end='')
    includes = ['-I'+str(HERE/'include'), '-I'+str(TARGET/'include')]
    common = ['-std=c++11', '-Wall', '-Wextra', '-Werror', *includes]
    modes = [('O0', ['-O0']), ('Os', ['-Os']), ('O2', ['-O2']),
             ('sanitized', ['-O1', '-g', '-fsanitize=address,undefined',
                            '-fno-sanitize-recover=all', '-fno-omit-frame-pointer'])]
    for label, flags in modes:
        out = BUILD/('uart_model_'+label)
        run([cxx, *common, *flags, HERE/'uart_model.cpp', '-o', out])
        log = run([out]); (BUILD/(out.name+'.log')).write_text(log)
        print(label, 'CFifo test double:', log.splitlines()[-1])
        production = ROOT/'src/cfifo.c'
        if production.exists():
            obj = BUILD/('production_cfifo_'+label+'.o')
            run([cc, '-std=c17', '-Wall', '-Wextra', '-Werror', *flags,
                 '-I'+str(ROOT/'include'), '-c', production, '-o', obj])
            real = BUILD/('uart_real_cfifo_'+label)
            run([cxx, *common, *flags, '-DRA4M1_TEST_REAL_CFIFO',
                 HERE/'uart_model.cpp', obj, '-o', real])
            log = run([real]); (BUILD/(real.name+'.log')).write_text(log)
            print(label, 'production CFifo:', log.splitlines()[-1])
    if not (ROOT/'src/cfifo.c').exists():
        print('NOT RUN: production CFifo source is absent from this standalone package.')
    for abi in ('soft', 'hard'):
        for label, flags in modes[:3]:
            run([cxx, *common, *flags, '--target=arm-none-eabi', '-mcpu=cortex-m4',
                 '-mthumb', '-mfpu=fpv4-sp-d16', '-mfloat-abi='+abi,
                 '-ffreestanding', '-fno-builtin', '-fno-exceptions', '-fno-rtti',
                 '-ffunction-sections', '-fdata-sections', '-c',
                 TARGET/'src/uart_ra4m1.cpp', '-o', BUILD/('uart_'+abi+'_'+label+'.o')])
        print('PASS: UART Cortex-M4 C++11', abi, 'ABI at O0/Os/O2 (reduced headers)')
    print('UART checks complete. Production GNU/newlib build, IOC managed build and hardware NOT tested.')


if __name__ == '__main__':
    main()
