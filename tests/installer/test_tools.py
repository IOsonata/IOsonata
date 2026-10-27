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
		return '\n'.join(re.findall(r'^install_(?:python_tools|srecord)\(\) \{.*?^\}', source, re.M | re.S))

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
						PYOCD_DIR=str(venv), PYOCD=str(venv / 'bin/pyocd'), BIN=str(bin_dir), LOG=str(root / 'log'), HOST_LOG=str(root / 'host-python'), FAIL=fail)
					result = subprocess.run(['bash', '-c', 'set -e\n' + self.functions(platform) + '\ninstall_python_tools'], env=env, capture_output=True)
					self.assertEqual(result.returncode == 0, fail == '0')
					self.assertEqual((root / 'log').read_text().strip(), '-m pip install --upgrade pyocd')
					self.assertFalse((root / 'host-python').exists())
					self.assertEqual((bin_dir / 'pyocd').read_text(), 'unrelated')
		windows = (INSTALLER / 'install_iocdevtools_win.ps1').read_text()
		self.assertIn('if (Test-Path $ManagedPython)', windows)
		self.assertIn('$PythonExecutablePath = $ManagedPython', windows)
		macos = (INSTALLER / 'install_iocdevtools_macos.sh').read_text()
		self.assertIn('Homebrew has no SRecord bottle for $bottle_tag', macos)
		self.assertIn('data.get("formulae")', macos)
		self.assertLess(macos.index('command -v port'), macos.index('brew_cmd=$(command -v brew'))

	def test_srecord_links_repeated_runs_and_no_self_links(self):
		for platform in ['linux', 'macos']:
			with self.subTest(platform=platform), tempfile.TemporaryDirectory() as d:
				root = Path(d)
				bin_dir = root / 'native tools'
				bin_dir.mkdir()
				for tool in ['srec_cat', 'srec_cmp', 'srec_info']:
					path = bin_dir / tool
					path.write_text('#!/bin/bash\necho SRecord-test\n')
					path.chmod(0o755)
				dest = root / 'Tools' / 'SRecord'
				env = dict(os.environ, PATH=str(bin_dir) + os.pathsep + os.environ['PATH'], SRECORD_DIR=str(dest))
				for _ in range(2):
					result = subprocess.run(['bash', '-c', 'set -e\n' + self.functions(platform) + '\ninstall_srecord'], env=env, capture_output=True)
					self.assertEqual(result.returncode, 0, result.stderr)
					env['PATH'] = str(dest / 'bin') + os.pathsep + env['PATH']
				self.assertEqual((dest / 'bin/srec_cat').resolve(), bin_dir / 'srec_cat')

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
					block = source[start:source.index('\nfi', start) + 3]
					env = dict(os.environ, PATH=str(root) + os.pathsep + os.environ['PATH'], MODE='force', FAILURE=failure)
					env[folder] = str(dest)
					result = subprocess.run(['bash', '-c', 'set -e\n' + block], env=env, text=True, capture_output=True)
					self.assertEqual(result.returncode == 0, failure == 'none', result.stderr)
					self.assertEqual(program.read_text().strip(), 'replacement' if failure == 'none' else 'original')
		windows = (INSTALLER / 'install_iocdevtools_win.ps1').read_text()
		self.assertIn('.idap-stage-', windows)
		self.assertIn('IDAPnRFProg extraction failed.', windows)
		self.assertIn('missing or empty IDAPnRFProg.exe', windows)
		self.assertIn('Move-Item $IDAP_DIR $IDAP_BACKUP', windows)
		self.assertIn('.xp-stage-', windows)
		self.assertIn('curl.exe -fL -o $zip $url', windows)
		self.assertIn('if ($LASTEXITCODE -ne 0) { throw "$Name extraction failed." }', windows)
		self.assertIn('archive is missing bin\\$Tool.exe', windows)
		self.assertIn('function Test-Python39', windows)
		self.assertIn('.python-stage-', windows)
		self.assertIn('Python installer failed: $($process.ExitCode)', windows)
		self.assertIn('.7zip-stage-', windows)
		self.assertIn('7-Zip installer failed: $($process.ExitCode)', windows)
		self.assertIn("$needsInstall = ($MODE -eq 'force')", windows)
		self.assertIn("'.srecord-stage-'", windows)
		self.assertIn('Missing or empty SRecord tool: $tool', windows)
		self.assertIn('Staged SRecord verification failed.', windows)
		self.assertIn('Move-Item $SRECORD_DIR $backup', windows)


	def test_srecord_verification_failure(self):
		with tempfile.TemporaryDirectory() as d:
			root = Path(d)
			for tool in ['srec_cat', 'srec_cmp', 'srec_info']:
				path = root / tool
				path.write_text('#!/bin/bash\nexit 7\n')
				path.chmod(0o755)
			env = dict(os.environ, PATH=str(root) + os.pathsep + os.environ['PATH'], SRECORD_DIR=str(root / 'managed'))
			result = subprocess.run(['bash', '-c', 'set -e\n' + self.functions('linux') + '\ninstall_srecord'], env=env)
			self.assertNotEqual(result.returncode, 0)


if __name__ == '__main__':
	unittest.main(verbosity=2)
