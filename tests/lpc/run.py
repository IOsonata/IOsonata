#!/usr/bin/env python3
"""Run all LPC host regression suites."""
from pathlib import Path
import subprocess
import sys

directory = Path(__file__).resolve().parent
for suite in ['clock', 'gpio', 'uart', 'ssp', 'i2c', 'timer',
              'lpc546xx', 'startup', 'iap', 'iap_shared', 'usb']:
    print(f'Running LPC {suite}', flush=True)
    subprocess.run([sys.executable, str(directory / suite / 'run.py')], check=True)
