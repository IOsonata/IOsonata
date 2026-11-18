#!/usr/bin/env python3
"""Exercise the production SAMD timer core with deterministic counter operations."""
from pathlib import Path
import subprocess
import tempfile
ROOT = Path(__file__).resolve().parents[2]
with tempfile.TemporaryDirectory(prefix='samd-timer-') as directory:
    out = Path(directory)
    for source, name in [('ARM/Microchip/SAMD/include/timer_samd.h', 'timer_private.h'),
                         ('ARM/Microchip/SAMD/include/evsys_samd.h', 'evsys_private.h'),
                         ('ARM/Microchip/SAMD/src/timer_samd.cpp', 'timer_driver.h')]:
        (out / name).write_text('\n'.join(line for line in (ROOT / source).read_text().splitlines()
                                        if not line.startswith('#include')))
    for opt in ('-O0', '-Os'):
        binary = out / ('timer' + opt)
        subprocess.run(['g++', '-std=gnu++23', opt, '-Wall', '-Wextra', '-Werror',
                        '-Wno-missing-field-initializers', '-fsanitize=undefined',
                        '-fno-sanitize-recover=all', '-I' + str(out), '-I' + str(ROOT / 'include'),
                        str(ROOT / 'tests/samd/timer_test.cpp'), '-o', str(binary)], check=True)
        subprocess.run([str(binary)], check=True)
