#!/usr/bin/env python3
"""Run the clock display time and rendering tests on the host."""
from pathlib import Path
import os
import subprocess
import tempfile

ROOT = Path(__file__).resolve().parents[2]


def main():
    with tempfile.TemporaryDirectory(prefix='clock-display-') as directory:
        binary = Path(directory) / 'clock_test'
        subprocess.run(['g++', '-std=c++17', '-Os', '-Wall', '-Wextra', '-Werror',
                        '-Wno-missing-field-initializers', '-fsanitize=address,undefined',
                        '-fno-sanitize-recover=all', '-I' + str(ROOT / 'exemples/display'),
                        str(ROOT / 'tests/display/clock_display_test.cpp'), '-o', str(binary)], check=True)
        subprocess.run([str(binary)], check=True, env=os.environ.copy())


if __name__ == '__main__':
    main()
