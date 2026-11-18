#!/usr/bin/env python3
"""Run extracted production Standby functions in a host register model.

Models NMI arrival at entry boundaries; it does not execute Arm instructions
or validate physical wake timing. Cortex-M0+ compilation is checked separately.
"""
from pathlib import Path
import subprocess
import tempfile
from run_standby_source import body

ROOT = Path(__file__).resolve().parents[2]
eic = (ROOT / 'ARM/Microchip/SAMD/src/iopincfg_samd.c').read_text()
sleep = (ROOT / 'ARM/Microchip/SAMD/src/system_sleep_samd.c').read_text()
functions = [(eic, 'void SamdStandbyWakeArm(void)'),
             (eic, 'void SamdStandbyWakeDone(void)'),
             (eic, 'void NMI_Handler(void)'),
             (sleep, 'void SystemSleep(void)')]
with tempfile.TemporaryDirectory(prefix='samd-standby-entry-') as tmp:
    out = Path(tmp)
    (out / 'standby_functions.h').write_text('\n'.join(
        signature + '\n{' + body(source, signature) + '}\n'
        for source, signature in functions))
    for opt in ('-O0', '-O2'):
        binary = out / ('entry' + opt)
        subprocess.run(['gcc', '-std=c11', opt, '-Wall', '-Wextra', '-Werror',
                        '-fsanitize=undefined', '-fno-sanitize-recover=all',
                        '-D__SAMD11D14AM__', '-I' + tmp,
                        str(ROOT / 'tests/samd/standby_entry_test.c'),
                        '-o', str(binary)], check=True)
        subprocess.run([str(binary)], check=True)
