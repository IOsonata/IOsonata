#!/usr/bin/env python3
# SPDX-License-Identifier: MIT
# Copyright (c) 2026, I-SYST inc.
"""Exercise the SDK TLS pinning helper without repository network access."""
import json
import os
from pathlib import Path
import shutil
import subprocess
import tempfile
import unittest

ROOT = Path(__file__).resolve().parents[2]
PIN = '0a8fda272a5a0abef3b47c91bed37185d5a726b1'
CRYPTO = 'a0632be94d883daa0295ac1eabf70359ad94f91b'


class MbedTlsInstallerTest(unittest.TestCase):
    def test_pinning(self):
        for platform, shell in [('linux', 'bash'), ('macos', 'zsh')]:
            if shutil.which(shell) is None:
                continue
            script = (ROOT / f'Installer/clone_iosonata_sdk_{platform}.sh').read_text()
            helper = script.split('clone_mbedtls() {', 1)[1].split('\n}\n', 1)[0]
            helper = 'clone_mbedtls() {' + helper + '\n}\nclone_mbedtls\n'
            for scenario in ('new', 'existing', 'wrong_head', 'dirty', 'wrong_link',
                             'update_failed', 'wrong_crypto'):
                with self.subTest(platform=platform, scenario=scenario):
                    with tempfile.TemporaryDirectory() as td:
                        root = Path(td)
                        target = root / 'external/mbedtls'
                        if scenario != 'new':
                            target.mkdir(parents=True)
                            (target / 'keep.txt').write_text('preserve me')
                        fake = root / 'git'
                        fake.write_text('''#!/usr/bin/env python3
import json, os, sys
from pathlib import Path
a = sys.argv[1:]
with open(os.environ['CALL_LOG'], 'a') as out:
    out.write(json.dumps(a) + '\\n')
s = os.environ['SCENARIO']
if a[0] == 'clone':
    assert a[1:5] == ['--depth=1', '--branch', 'mbedtls-4.1.1', 'https://github.com/Mbed-TLS/mbedtls.git']
    Path(a[-1]).mkdir(parents=True)
elif a[0] == '-C':
    if a[2:] == ['rev-parse', 'HEAD']:
        crypto = a[1].endswith('/tf-psa-crypto')
        wrong = s == ('wrong_crypto' if crypto else 'wrong_head')
        print('bad' if wrong else os.environ['CRYPTO_PIN' if crypto else 'MAIN_PIN'])
    elif a[2:] == ['status', '--porcelain', '--ignore-submodules=none']:
        print(' M local.cpp' if s == 'dirty' else '', end='')
    elif a[2:] == ['rev-parse', 'HEAD:tf-psa-crypto']:
        print('bad' if s == 'wrong_link' else os.environ['CRYPTO_PIN'])
    elif a[2:] == ['submodule', 'update', '--init', '--recursive', '--depth', '1']:
        sys.exit(1 if s == 'update_failed' else 0)
    else:
        sys.exit(99)
else:
    sys.exit(99)
''')
                        fake.chmod(0o755)
                        log = root / 'calls.jsonl'
                        env = dict(os.environ, PATH=str(root) + os.pathsep + os.environ['PATH'],
                                   EXT=str(root / 'external'), MODE='force', CALL_LOG=str(log),
                                   SCENARIO=scenario, MAIN_PIN=PIN, CRYPTO_PIN=CRYPTO)
                        result = subprocess.run([shell, '-c', helper], env=env,
                                                capture_output=True, text=True)
                        self.assertEqual(result.returncode == 0, scenario in ('new', 'existing'),
                                         result.stdout + result.stderr)
                        if scenario != 'new':
                            self.assertEqual((target / 'keep.txt').read_text(), 'preserve me')
                        calls = [json.loads(line) for line in log.read_text().splitlines()]
                        if scenario in ('wrong_head', 'dirty', 'wrong_link'):
                            self.assertFalse(any('submodule' in a for a in calls))
                        if scenario == 'existing':
                            self.assertFalse(any('clone' in a for a in calls))


if __name__ == '__main__':
    unittest.main()
