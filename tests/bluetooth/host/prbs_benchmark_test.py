#!/usr/bin/env python3
"""Verify byte-identical benchmark output from the C and C++ implementations."""
from pathlib import Path
import subprocess
import tempfile

ROOT = Path(__file__).resolve().parents[3]
expected = None
with tempfile.TemporaryDirectory(prefix='prbs-common-') as tmp:
    exe = Path(tmp) / 'test'
    for compiler, standard in [('cc', 'c11'), ('g++', 'c++17')]:
        for opt in ('-O0', '-O2'):
            subprocess.run([compiler, '-std=' + standard, opt, '-Wall', '-Wextra',
                            '-fsanitize=undefined', '-fno-sanitize-recover=all',
                            '-I' + str(ROOT / 'include'),
                            '-I' + str(ROOT / 'exemples/bluetooth'),
                            str(ROOT / 'tests/bluetooth/host/prbs_benchmark_test.c'),
                            str(ROOT / 'src/cfifo.c'), '-o', str(exe)], check=True)
            output = subprocess.check_output([str(exe)])
            if expected is None:
                expected = output
            assert output == expected, (compiler, opt)
    assert b'PHY=1 RESULT: PASS' in expected
    assert b'PHY=2 RESULT: PASS' in expected
print('PASS shared C/C++ notification processing and byte-identical reports (O0/O2 UBSan)')
