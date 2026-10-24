#!/usr/bin/env python3
from pathlib import Path
import os
import subprocess
import tempfile
ROOT = Path(__file__).resolve().parents[2]
with tempfile.TemporaryDirectory(prefix='stm32-rng-') as temp:
    out = Path(temp)
    (out/'stm32.h').write_text('#include "stm32_rng_model.h"\n')
    for opt in ('-O0','-O2'):
        sources = ['src/crypto/crypto_p256.cpp','src/device.cpp','src/device_intrf.cpp','ARM/ST/src/rng_stm32.cpp','tests/crypto/stm32_rng_model.cpp']
        inc = ['-I'+str(p) for p in (out, ROOT/'include', ROOT/'ARM/ST/include', ROOT/'tests/crypto')]
        subprocess.run(['g++','-std=gnu++23',opt,'-Wall','-Wextra','-Werror','-fno-rtti','-fno-exceptions',
            '-fsanitize=address,undefined','-fno-pie','-no-pie','-DRNG_STM32_TIMEOUT=8',*inc,
            *[str(ROOT/s) for s in sources],'-o',str(out/'test')], check=True)
        subprocess.run([str(out/'test')],check=True,env={**os.environ,'ASAN_OPTIONS':'detect_leaks=0'})
