#!/usr/bin/env python3
# MIT License
#
# Copyright (c) 2026, I-SYST inc., all rights reserved
#
# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:
#
# The above copyright notice and this permission notice shall be included in all
# copies or substantial portions of the Software.
#
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
# SOFTWARE.

"""Run the sensor drivers against register models of the devices.

Each test builds the production driver with its test on the host, with
AddressSanitizer and UBSan at -O0 and -O2, then compiles the driver for a
Cortex-M4 with GNU Arm when arm-none-eabi-g++ is found. The models check the
register configuration and the data conversion, not the bus timing or the
devices themselves.
"""
import os
from pathlib import Path
import shutil
import subprocess
import tempfile

ROOT = Path(__file__).resolve().parents[2]

# The base sensor classes hide some Device overloads, which GCC 13 and later
# report with -Wall.
WARN = ['-Wall', '-Wextra', '-Werror', '-Wno-overloaded-virtual']
COMMON = ['src/device.cpp', 'src/device_intrf.cpp']

TESTS = {
	'lsm6dsl': ['src/sensors/ag_lsm6dsl.cpp', 'src/sensors/accel_sensor.cpp',
				'src/sensors/gyro_sensor.cpp'],
	'hts221': ['src/sensors/humi_hts221.cpp'],
}


def main():
	cxx = os.environ.get('CXX', 'g++')
	arm = shutil.which('arm-none-eabi-g++')
	with tempfile.TemporaryDirectory(prefix='sensors-') as tmp:
		for name, sources in TESTS.items():
			for opt in ('-O0', '-O2'):
				exe = Path(tmp) / f'{name}{opt}'
				cmd = [cxx, '-std=gnu++23', opt, '-g', *WARN, '-fno-exceptions', '-fno-rtti',
					   '-fsanitize=address,undefined', '-fno-sanitize-recover=all',
					   '-I' + str(ROOT / 'include'), str(ROOT / 'tests/sensors' / f'{name}_test.cpp')]
				cmd += [str(ROOT / s) for s in sources + COMMON]
				subprocess.run(cmd + ['-o', str(exe)], check=True)
				out = subprocess.run([str(exe)], capture_output=True, text=True)
				print(f'{name} {opt}: {out.stdout.strip().splitlines()[-1]}', flush=True)
				if out.returncode != 0:
					print(out.stdout)
					raise SystemExit(1)
			if arm:
				for opt in ('-O0', '-Os'):
					subprocess.run([arm, '-mcpu=cortex-m4', '-mthumb', '-std=gnu++23', opt, *WARN,
									'-fno-exceptions', '-fno-rtti', '-I' + str(ROOT / 'include'),
									'-I' + str(ROOT / 'ARM/include'), '-I' + str(ROOT / 'ARM/CMSIS/Core/Include'),
									'-c', str(ROOT / sources[0]), '-o', str(Path(tmp) / f'{name}.o')],
								   check=True)
				print(f'{name}: GNU Arm compile at -O0 and -Os PASS', flush=True)
			else:
				print(f'{name}: NOT RUN, arm-none-eabi-g++ not found, no Arm compile', flush=True)
	print('Sensor driver models complete. Hardware not tested.')


if __name__ == '__main__':
	main()
