#!/usr/bin/env python3
"""Run the production SAMD UART with register/DMAC stand-ins and real CFIFOs."""
from pathlib import Path
import subprocess
import tempfile
ROOT = Path(__file__).resolve().parents[2]
with tempfile.TemporaryDirectory(prefix='samd-uart-') as directory:
    out = Path(directory)
    source = (ROOT / 'ARM/Microchip/SAMD/src/uart_samd.cpp').read_text()
    (out / 'uart_driver.h').write_text('\n'.join(line for line in source.splitlines()
                                             if not line.startswith('#include')))
    for opt in ('-O0', '-Os'):
        binary = out / ('uart' + opt)
        flags = [opt, '-fsanitize=undefined', '-fno-sanitize-recover=all',
                 '-I' + str(out), '-I' + str(ROOT / 'include'),
                 '-I' + str(ROOT / 'ARM/Microchip/SAMD/SAMD11/include')]
        subprocess.run(['gcc', '-std=gnu17', *flags, '-c', str(ROOT / 'src/cfifo.c'),
                        '-o', str(out / 'cfifo.o')], check=True)
        subprocess.run(['g++', '-std=gnu++23', *flags, '-Wall', '-Wextra', '-Werror',
                        '-Wno-class-memaccess', '-Wno-missing-field-initializers',
                        str(ROOT / 'tests/samd/uart_test.cpp'), str(out / 'cfifo.o'),
                        '-o', str(binary)], check=True)
        subprocess.run([str(binary)], check=True)
