#!/usr/bin/env python3
"""STM32 timer host tests and GNU Arm checks of the shared timer driver.

Run: python3 tests/stm32/run_timer.py

The driver is ARM/ST/src/timer_stm32.cpp (TimerInit), timer_lptim_stm32.cpp
(LPTIM) and timer_tim_stm32.cpp (TIM), with ARM/ST/include/timer_stm32.h.

- timer_tim_test.cpp runs TimerInit and the TIM driver with a model of the
  TIM registers for the parts of TIM_PARTS, at -O0 and -O2 with the
  undefined behavior sanitizer, then built with TEST_VECTOR_OVERRIDE: an
  application handler on one timer vector, TimerInit refuses its timers.
- timer_lptim_test.cpp runs the three sources for the L4 and G4 parts of
  LPTIM_PARTS, peripheral registers plain memory mapped at their addresses,
  and with TEST_VECTOR_OVERRIDE. Linux only. The WBA LPTIM is tested by
  tests/stm32wba/run.py.
- the vector file of each STM32 library aliases to Stm32TimerVector the
  timer vectors timer_tim_test.cpp lists for the part, and no other.

When arm-none-eabi-g++ is found:
- the three sources are compiled for every device of the F0, F4, L4, G4 and
  WBA family headers that stm32.h selects, at -O0 and -Os, warnings as
  errors
- an F030x8, L476 and WBA65 image is linked from an archive of the driver
  and the vector file: without TimerInit the timer driver is not linked and
  the timer vectors are Stm32TimerVector; with it Stm32TimerIRQHandler is
  linked; an application TIM3_IRQHandler takes the place of the alias.

Not hardware validation: no timing, no electrical test.

MIT License

Copyright (c) 2026, I-SYST inc., all rights reserved

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
"""
import concurrent.futures
import os
from pathlib import Path
import re
import shutil
import subprocess
import sys
import tempfile

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[1]
SRC = ROOT / 'ARM/ST/src'
TIMER = [SRC / 'timer_stm32.cpp', SRC / 'timer_lptim_stm32.cpp', SRC / 'timer_tim_stm32.cpp']

TIM_PARTS = [
    ('STM32F030x8', 'STM32F0xx'), ('STM32F072xB', 'STM32F0xx'), ('STM32F401xC', 'STM32F4xx'),
    ('STM32F446xx', 'STM32F4xx'), ('STM32L432xx', 'STM32L4xx'), ('STM32L476xx', 'STM32L4xx'),
    ('STM32L496xx', 'STM32L4xx'), ('STM32L4S9xx', 'STM32L4xx'), ('STM32G474xx', 'STM32G4xx'),
    ('STM32WBA65xx', 'STM32WBAxx'),
]
LPTIM_PARTS = [
    ('STM32L432xx', 'STM32L4xx'), ('STM32L476xx', 'STM32L4xx'), ('STM32L4S9xx', 'STM32L4xx'),
    ('STM32G474xx', 'STM32G4xx'),
]

# Family directory, family header, Cortex-M core and floating point flags
FAMILIES = [
    ('STM32F0xx', 'stm32f0xx.h', 'cortex-m0', []),
    ('STM32F4xx', 'stm32f4xx.h', 'cortex-m4', ['-mfloat-abi=hard', '-mfpu=fpv4-sp-d16']),
    ('STM32L4xx', 'stm32l4xx.h', 'cortex-m4', ['-mfloat-abi=hard', '-mfpu=fpv4-sp-d16']),
    ('STM32G4xx', 'stm32g4xx.h', 'cortex-m4', ['-mfloat-abi=hard', '-mfpu=fpv4-sp-d16']),
    ('STM32WBAxx', 'stm32wbaxx.h', 'cortex-m33', ['-mfloat-abi=hard', '-mfpu=fpv5-sp-d16']),
]

# STM32 libraries: part, family directory and vector file
LIBRARIES = [
    ('STM32F030x8', 'STM32F0xx', 'ARM/ST/STM32F0xx/STM32F030x8/lib/src/Vectors_STM32F030x8.c'),
    ('STM32F401xC', 'STM32F4xx', 'ARM/ST/STM32F4xx/STM32F401/STM32F401xc/lib/src/Vectors_STM32F401xC.c'),
    ('STM32L432xx', 'STM32L4xx', 'ARM/ST/STM32L4xx/STM32L432/lib/src/Vector_STM32L432xx.c'),
    ('STM32L476xx', 'STM32L4xx', 'ARM/ST/STM32L4xx/STM32L476/lib/src/Vector_STM32L476xx.c'),
    ('STM32L496xx', 'STM32L4xx', 'ARM/ST/STM32L4xx/STM32L496/lib/src/Vector_STM32L496xx.c'),
    ('STM32L4S9xx', 'STM32L4xx', 'ARM/ST/STM32L4xx/STM32L4S9/lib/src/Vector_STM32L4S9xx.c'),
    ('STM32G474xx', 'STM32G4xx', 'ARM/ST/STM32G4xx/STM32G474/lib/src/Vector_STM32G474xx.c'),
    ('STM32WBA65xx', 'STM32WBAxx', 'ARM/ST/STM32WBAxx/STM32WBA65/src/vectors_stm32wba65.c'),
]


def run(args, check=True):
    p = subprocess.run([str(a) for a in args], cwd=ROOT, text=True,
                       stdout=subprocess.PIPE, stderr=subprocess.STDOUT)
    if check and p.returncode != 0:
        raise SystemExit(f'FAIL ({p.returncode}): {" ".join(map(str, args))}\n{p.stdout}')
    return p.stdout


def family_of(directory):
    return next(f for f in FAMILIES if f[0] == directory)


def arm_flags(directory):
    _, _, cpu, fpu = family_of(directory)
    return [f'-mcpu={cpu}', '-mthumb', *fpu]


def arm_inc(directory):
    return ['-I' + str(ROOT / 'include'), '-I' + str(ROOT / 'ARM/include'),
            '-I' + str(ROOT / 'ARM/CMSIS/Core/Include'), '-I' + str(ROOT / 'ARM/ST/include'),
            '-I' + str(ROOT / 'ARM/ST' / directory / 'include')]


def host_tests(cxx):
    vectors = {}
    with tempfile.TemporaryDirectory(prefix='stm32timer-') as tmp:
        tmp = Path(tmp)
        for part, family in TIM_PARTS:
            inc = ['-I' + str(HERE / 'timer_shim'), '-I' + str(HERE / 'shim'), '-I' + str(ROOT / 'include'),
                   '-I' + str(ROOT / 'ARM/ST/include'), '-I' + str(ROOT / 'ARM/ST' / family / 'include')]
            for label, opt in (('O0', ['-O0']), ('O2', ['-O2']), ('override', ['-O2', '-DTEST_VECTOR_OVERRIDE'])):
                exe = tmp / f'tim_{part}_{label}'
                run([cxx, '-std=gnu++23', '-fno-exceptions', '-fno-rtti', *opt, '-g', '-fsanitize=undefined',
                     '-fno-sanitize-recover=all', '-D' + part, *inc, HERE / 'timer_tim_test.cpp', '-o', exe])
                print(f'TIM {part} {label}: {run([exe]).strip().splitlines()[-1]}', flush=True)
            vectors[part] = set(run([exe, '--vectors']).split())

        for part, family in LPTIM_PARTS:
            inc = ['-I' + str(HERE / 'shim'), '-I' + str(ROOT / 'include'),
                   '-I' + str(ROOT / 'ARM/ST/include'), '-I' + str(ROOT / 'ARM/ST' / family / 'include')]
            for label, opt in (('O0', ['-O0']), ('O2', ['-O2']), ('override', ['-O2', '-DTEST_VECTOR_OVERRIDE'])):
                exe = tmp / f'lptim_{part}_{label}'
                run([cxx, '-std=gnu++23', '-fno-exceptions', '-fno-rtti', *opt, '-g', '-fsanitize=undefined',
                     '-fno-sanitize-recover=all', '-no-pie', '-D' + part, *inc, HERE / 'timer_lptim_test.cpp',
                     *TIMER, '-o', exe])
                print(f'LPTIM {part} {label}: {run([exe]).strip().splitlines()[-1]}', flush=True)

    # Vector files: the timer vectors of the part, and no other, aliased to
    # Stm32TimerVector, which calls the weak Stm32TimerIRQHandler
    for part, _, vector in LIBRARIES:
        text = (ROOT / vector).read_text()
        aliased = set(re.findall(r'(\w+)_IRQHandler\(void\)\s*__attribute__\(\(weak, alias\("Stm32TimerVector"\)\)\)', text)) | \
                  set(re.findall(r'__attribute__\(\(weak, alias\("Stm32TimerVector"\)\)\)\s*void\s+(\w+)_IRQHandler', text))
        if aliased != vectors[part]:
            raise SystemExit(f'FAIL {vector}: aliased {sorted(aliased)}, timer vectors {sorted(vectors[part])}')
        if not re.search(r'__attribute__\(\(weak\)\)\s*void Stm32TimerIRQHandler\(int IrqNo\);', text) or \
           not re.search(r'\nvoid Stm32TimerVector\(void\)\n', text.replace('\r\n', '\n')):
            raise SystemExit(f'FAIL {vector}: Stm32TimerVector or the weak Stm32TimerIRQHandler missing')
        print(f'{part}: {len(aliased)} timer vectors on Stm32TimerVector in the vector file', flush=True)


def devices():
    res = []
    for directory, header, _, _ in FAMILIES:
        text = (ROOT / 'ARM/ST' / directory / 'include' / header).read_text()
        for macro, _ in re.findall(r'defined\s*\(\s*(STM32\w+)\s*\)\s*\n\s*#include\s+"(stm32\w+\.h)"', text):
            # stm32.h does not select the WBA2x
            if not macro.startswith('STM32WBA2'):
                res.append((macro, directory))
    return res


def compile_all(arm):
    jobs = [(m, d, label, src) for m, d in devices() for label in ('O0', 'Os') for src in TIMER]

    def one(job):
        macro, directory, label, src = job
        return job, run([arm, *arm_flags(directory), '-D' + macro, f'-{label}', '-std=gnu++23', '-Wall', '-Werror',
                         '-fno-exceptions', '-fno-rtti', '-ffunction-sections', '-fdata-sections',
                         *arm_inc(directory), '-c', src, '-o', os.devnull], check=False)

    bad = []
    with concurrent.futures.ThreadPoolExecutor(os.cpu_count() or 4) as ex:
        for job, out in ex.map(one, jobs):
            if out.strip():
                bad.append(f'{job[0]} {job[2]} {job[3].name}\n{out}')
    if bad:
        raise SystemExit('FAIL GNU Arm compile\n' + '\n'.join(bad))
    print(f'GNU Arm compile of the timer sources for {len(jobs) // 6} devices at -O0 and -Os: PASS')


def check_link(arm, tmp):
    nm = arm.replace('g++', 'nm')
    ar = arm.replace('g++', 'ar')
    for part, directory, vector in (LIBRARIES[0], LIBRARIES[3], LIBRARIES[7]):
        objs = []
        for src in [*TIMER, ROOT / vector]:
            obj = tmp / f'link_{part}_{src.stem}.o'
            lang = ['-x', 'c', '-std=gnu17'] if src.suffix == '.c' else ['-std=gnu++23', '-fno-exceptions', '-fno-rtti']
            run([arm, *arm_flags(directory), '-D' + part, '-Os', *lang, '-ffunction-sections', '-fdata-sections',
                 *arm_inc(directory), '-c', src, '-o', obj])
            objs.append(obj)
        lib = tmp / f'libtimer_{part}.a'
        run([ar, 'rcs', lib, *objs])
        for variant in ('systick', 'timer', 'override'):
            main = tmp / f'main_{part}_{variant}.cpp'
            init = 'TimerDev_t t{}; TimerCfg_t cfg{0, TIMER_CLKSRC_DEFAULT, 0, 2, nullptr, false}; TimerInit(&t, &cfg);' \
                   if variant != 'systick' else ''
            app = 'void TIM3_IRQHandler(void) {}\n' if variant == 'override' else ''
            main.write_text('#include "coredev/timer.h"\n#include "coredev/system_core_clock.h"\n'
                            'uint32_t SystemCoreClock = 16000000;\n'
                            'McuOsc_t g_McuOsc = { { OSC_TYPE_RC, 16000000, 20 }, { OSC_TYPE_RC, 32000, 20 }, false };\n'
                            'extern "C" {\nuint32_t SystemPeriphClockGet(int) { return SystemCoreClock; }\n'
                            'void SystemInit(void) {}\n' + app +
                            'void ResetEntry(void) {\n' + init + '\nwhile (1) {}\n}\n}\n')
            exe = tmp / f'{part}_{variant}.elf'
            run([arm, *arm_flags(directory), '-Os', '-fno-exceptions', '-fno-rtti', '-ffunction-sections',
                 *arm_inc(directory), '-nostartfiles', main, '-Wl,-u,__Vectors', '-Wl,-e,ResetEntry',
                 '-Wl,--defsym=__StackTop=0x20002000', '-Wl,--defsym=__stack=0x20002000', '-Wl,--gc-sections',
                 lib, '-Wl,--start-group', '-lc', '-lgcc', '-lnosys', '-Wl,--end-group', '-o', exe])
            syms = {}
            addr = {}
            for line in run([nm, exe]).splitlines():
                f = line.split()
                if len(f) == 3:
                    syms[f[2]] = f[1]
                    addr[f[2]] = int(f[0], 16)
                elif len(f) == 2:
                    syms[f[1]] = f[0]
            vec = addr['Stm32TimerVector']
            has_init = syms.get('TimerInit') == 'T'
            handler = syms.get('Stm32TimerIRQHandler')
            tim3 = addr['TIM3_IRQHandler'] == vec
            ok = syms.get('TIM3_IRQHandler') in ('W', 'T')
            if variant == 'systick':
                ok = ok and not has_init and handler in (None, 'w', 'v') and tim3
            elif variant == 'timer':
                ok = ok and has_init and handler == 'T' and tim3
            else:
                ok = ok and has_init and handler == 'T' and not tim3 and syms['TIM3_IRQHandler'] == 'T'
            if not ok:
                raise SystemExit(f'FAIL {part} {variant} link: TimerInit {has_init}, Stm32TimerIRQHandler {handler}, '
                                 f'TIM3_IRQHandler {syms.get("TIM3_IRQHandler")} on Stm32TimerVector {tim3}')
            print(f'{part} {variant} image: PASS', flush=True)


def main():
    cxx = os.environ.get('CXX') or shutil.which('g++')

    if not sys.platform.startswith('linux'):
        print('NOT RUN: the host tests need Linux')
    elif not cxx:
        raise SystemExit('A host C++ compiler is required.')
    else:
        host_tests(cxx)

    arm = shutil.which('arm-none-eabi-g++')
    if arm:
        compile_all(arm)
        with tempfile.TemporaryDirectory(prefix='stm32timer-arm-') as tmp:
            check_link(arm, Path(tmp))
    else:
        print('NOT RUN: arm-none-eabi-g++ not found, no Arm checks')
    print('STM32 timer checks complete. Hardware not tested.')


if __name__ == '__main__':
    main()
