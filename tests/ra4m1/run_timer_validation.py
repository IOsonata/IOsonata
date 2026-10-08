#!/usr/bin/env python3
"""RA4M1 timer/ICU model, ARM compile and inherited port regressions.

The model uses actual timer/ICU code and reduced headers. It is not a CPU
emulator, a production GNU/newlib link, or a physical timing test.
"""
import os
from pathlib import Path
import shutil
import sys
from run_validation import ROOT, HERE, TARGET, BUILD, run


def main():
    cxx = os.environ.get('CLANGXX') or shutil.which('clang++')
    if not cxx:
        raise SystemExit('Clang++ is required.')
    BUILD.mkdir(exist_ok=True)
    print(run([sys.executable, HERE/'run_uart_validation.py']), end='')
    includes = ['-I'+str(HERE/'include'), '-I'+str(TARGET/'include')]
    common = ['-std=c++11', '-Wall', '-Wextra', '-Werror', *includes]
    modes = [('O0', ['-O0']), ('Os', ['-Os']), ('O2', ['-O2']),
             ('sanitized', ['-O1', '-g', '-fsanitize=address,undefined',
                            '-fno-sanitize-recover=all', '-fno-omit-frame-pointer'])]
    for label, flags in modes:
        out = BUILD/('timer_model_'+label)
        run([cxx, *common, *flags, HERE/'timer_model.cpp', '-o', out])
        log = run([out]); (BUILD/(out.name+'.log')).write_text(log)
        print(label, log.strip())
    for abi in ('soft', 'hard'):
        for label, flags in modes[:3]:
            run([cxx, *common, *flags, '--target=arm-none-eabi', '-mcpu=cortex-m4',
                 '-mthumb', '-mfpu=fpv4-sp-d16', '-mfloat-abi='+abi,
                 '-ffreestanding', '-fno-builtin', '-fno-exceptions', '-fno-rtti',
                 '-ffunction-sections', '-fdata-sections', '-c',
                 TARGET/'src/timer_ra4m1.cpp', '-o', BUILD/('timer_'+abi+'_'+label+'.o')])
        print('PASS: timer Cortex-M4 C++11', abi, 'ABI at O0/Os/O2 (reduced headers)')
    print('Timer checks complete. Production GNU/newlib build, IOC managed build and hardware NOT tested.')


if __name__ == '__main__':
    main()
