#!/usr/bin/env python3
"""Offline tests for the managed firmware tools; never install host packages."""
import os
from pathlib import Path
import re
import subprocess
import tempfile
import unittest

INSTALLER = Path(__file__).resolve().parents[2] / 'Installer'


class ToolTests(unittest.TestCase):
	def functions(self, platform):
		source = (INSTALLER / f'install_iocdevtools_{platform}.sh').read_text()
		return '\n'.join(re.findall(r'^install_python_tools\(\) \{.*?^\}', source, re.M | re.S))

	def test_venv_pip_and_failure_propagation(self):
		for platform in ['linux', 'macos']:
			for fail in ['0', '19']:
				with self.subTest(platform=platform, fail=fail), tempfile.TemporaryDirectory() as d:
					root = Path(d)
					venv = root / 'Tools with spaces' / 'Python'
					(venv / 'bin').mkdir(parents=True)
					bin_dir = root / 'bin'
					bin_dir.mkdir()
					for path, code in [
						(bin_dir / 'python3', '#!/bin/bash\necho used >> "$HOST_LOG"\nexit 99\n'),
						(venv / 'bin/python3', '#!/bin/bash\n[[ "$1" == "-c" ]] && exit 0\nprintf "%s\\n" "$*" >> "$LOG"\nexit "$FAIL"\n'),
						(venv / 'bin/pyocd', '#!/bin/bash\necho pyocd-test\n')]:
						path.write_text(code)
						path.chmod(0o755)
					# Existing unrelated command must remain untouched.
					(bin_dir / 'pyocd').write_text('unrelated')
					env = dict(os.environ, PATH=str(bin_dir) + os.pathsep + os.environ['PATH'],
						PYOCD_DIR=str(venv), TOOL_BIN=str(root / 'Tools' / 'bin'), PYOCD=str(venv / 'bin/pyocd'), BIN=str(bin_dir), LOG=str(root / 'log'), HOST_LOG=str(root / 'host-python'), FAIL=fail)
					result = subprocess.run(['bash', '-c', 'set -e\n' + self.functions(platform) + '\ninstall_python_tools'], env=env, capture_output=True)
					self.assertEqual(result.returncode == 0, fail == '0')
					self.assertEqual((root / 'log').read_text().strip(), '-m pip install --upgrade pyocd intelhex==2.3.0')
					self.assertFalse((root / 'host-python').exists())
					self.assertEqual((bin_dir / 'pyocd').read_text(), 'unrelated')
		windows = (INSTALLER / 'install_iocdevtools_win.ps1').read_text()
		self.assertIn('if (Test-Path $ManagedPython)', windows)
		self.assertIn('pip install --upgrade pyocd intelhex==2.3.0', windows)
		self.assertIn('$PythonExecutablePath = $ManagedPython', windows)
		macos = (INSTALLER / 'install_iocdevtools_macos.sh').read_text()
		self.assertIn('install_macos_python() {', macos)
		self.assertIn('python-3.12.4-macos11.pkg', macos)
		self.assertIn('pkgutil --check-signature "$pkg"', macos)
		self.assertIn('bootstrap_python=$(install_macos_python)', macos)
		linux = (INSTALLER / 'install_iocdevtools_linux.sh').read_text()
		self.assertIn("sys.version_info < (3, 9)", linux)
		self.assertIn("Ubuntu 20.04's default Python 3.8 is not sufficient", linux)

	def test_idap_bad_download_preserves_existing_tool(self):
		for platform, folder in [('macos', 'IDAP_DIR'), ('linux', 'TOOLS_DIR')]:
			for failure in ['download', 'extract', 'missing', 'none']:
				with self.subTest(platform=platform, failure=failure), tempfile.TemporaryDirectory() as d:
					root = Path(d)
					dest = root / 'tools'
					dest.mkdir()
					program = dest / 'IDAPnRFProg'
					program.write_text('original')
					program.chmod(0o755)
					for name, script in {
						'curl': '[[ "$FAILURE" != download ]]',
						'unzip': '[[ "$FAILURE" != extract ]] || exit 1; mkdir -p "$5"; [[ "$FAILURE" == missing ]] || echo replacement > "$5/IDAPnRFProg"',
					}.items():
						tool = root / name
						tool.write_text('#!/bin/bash\n' + script + '\n')
						tool.chmod(0o755)
					source = (INSTALLER / f'install_iocdevtools_{platform}.sh').read_text()
					start = source.index('IDAP_PROG="$' + folder + '/IDAPnRFProg"')
					end_marker = '# Install Eclipse Embedded CDT' if platform == 'macos' else '# Check if udev rules already installed in system'
					block = source[start:source.index(end_marker, start)]
					env = dict(os.environ, PATH=str(root) + os.pathsep + os.environ['PATH'], MODE='force',
						FAILURE=failure)
					env[folder] = str(dest)
					result = subprocess.run(['bash', '-c', 'set -e\n' + block], env=env, text=True, capture_output=True)
					self.assertEqual(result.returncode == 0, platform == 'linux' or failure == 'none', result.stderr)
					self.assertEqual(program.read_text().strip(), 'replacement' if failure == 'none' else 'original')
					self.assertTrue((dest / 'python3').exists() is False)

	def test_intelhex_installed_without_srecord(self):
		for platform in ['macos', 'linux']:
			source = (INSTALLER / f'install_iocdevtools_{platform}.sh').read_text()
			self.assertIn('pyocd intelhex==2.3.0', source)
			self.assertNotIn('install_srecord', source)
			self.assertNotIn('SRECORD_DIR', source)
		windows = (INSTALLER / 'install_iocdevtools_win.ps1').read_text()
		self.assertIn('pyocd intelhex==2.3.0', windows)
		self.assertNotIn('Install-SRecord', windows)
		self.assertNotIn('SRECORD_DIR', windows)


if __name__ == '__main__':
	unittest.main(verbosity=2)
