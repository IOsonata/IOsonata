#!/usr/bin/env python3
"""Exercise the production CC3xx register interface; no silicon timing claim."""
from pathlib import Path
import os
import subprocess
import tempfile
ROOT = Path(__file__).resolve().parents[2]
with tempfile.TemporaryDirectory(prefix='ccintrf-') as temp:
    out = Path(temp)
    (out/'coredev').mkdir()
    (out/'coredev/interrupt.h').write_text('#pragma once\n#include <stdint.h>\n'
        'inline uintptr_t DisableInterrupt() { return 0; }\ninline void EnableInterrupt(uintptr_t) {}\n')
    for opt in ('-O0', '-Os', '-O2'):
        exe = out/'test'
        subprocess.run(['g++', '-std=gnu++23', '-fno-rtti', '-fno-exceptions',
            '-Wall', '-Wextra', '-Werror', opt, '-g', '-fsanitize=address,undefined',
            '-fno-pie', '-no-pie', '-I'+str(out), '-I'+str(ROOT/'include'),
            '-I'+str(ROOT/'ARM/include'),
            *[str(ROOT/p) for p in ('src/device.cpp', 'src/device_intrf.cpp',
                'src/coredev/timer.cpp', 'tests/crypto/timer_fixture.cpp',
                'ARM/src/cc3xx_intrf.cpp', 'tests/crypto/ccintrf_test.cpp')],
            '-o', str(exe)], check=True)
        print(opt, flush=True)
        subprocess.run([str(exe)], check=True, env={**os.environ, 'ASAN_OPTIONS':'detect_leaks=0'})

    # Compile the actual vendor wrapper for both MDK alias forms.
    for secure in (False, True):
        (out/'nrf.h').write_text("""#pragma once
#include <stdint.h>
struct TestReg {
    uint32_t Value = 0, Writes = 0;
    bool Fail = false;
    operator uint32_t() const { return Value; }
    TestReg &operator=(uint32_t value) {
        ++Writes;
        if (!Fail) Value = value;
        return *this;
    }
};
struct TestWrapper { TestReg ENABLE; };
extern TestWrapper g_Wrapper;
""" + ('#define NRF_CRYPTOCELL_S (&g_Wrapper)\n'
       '#define NRF_CC_HOST_RGF_S_BASE 0x50845000U\n'
       '#define TEST_CC_BASE NRF_CC_HOST_RGF_S_BASE\n' if secure else
       '#define NRF_CRYPTOCELL (&g_Wrapper)\n'
       '#define NRF_CC_HOST_RGF_BASE 0x5002B000U\n'
       '#define TEST_CC_BASE NRF_CC_HOST_RGF_BASE\n'))
        subprocess.run(['g++', '-std=gnu++23', '-Os', '-Wall', '-Wextra', '-Werror',
            '-I'+str(out), '-I'+str(ROOT/'include'), '-I'+str(ROOT/'ARM/include'),
            str(ROOT/'ARM/Nordic/src/cc3xx_intrf_nrfx.cpp'),
            str(ROOT/'tests/crypto/ccpower_test.cpp'), '-o', str(out/'power')], check=True)
        subprocess.run([str(out/'power')], check=True)
    print('Nordic wrapper: repeated enable, power-off, failure and recovery passed')
