#!/usr/bin/env python3
"""Offline installer regressions. No downloads or system changes are performed."""
import os
from pathlib import Path
import re
import shutil
import subprocess
import tempfile
import unittest

INSTALLER = Path(__file__).resolve().parents[2] / 'Installer'


class InstallerTests(unittest.TestCase):
	def shells(self):
		for platform, shell in [('linux', 'bash'), ('macos', 'zsh')]:
			if shutil.which(shell):
				yield platform, shell

	def run_helper(self, platform, shell, helper, mode='normal', tag='', fail='', cube_state=''):
		with tempfile.TemporaryDirectory() as directory:
			root = Path(directory)
			target = root / 'STM32CubeWBA'
			target.mkdir()
			if cube_state:
				header = target / 'Drivers/STM32WBAxx_HAL_Driver/Inc/stm32wbaxx_hal.h'
				header.parent.mkdir(parents=True)
				header.touch()
			if cube_state == 'complete':
				for folder in ['Projects/Common/WPAN/Interfaces', 'Utilities/lpm/tiny_lpm']:
					(target / folder).mkdir(parents=True)
				platform_file = target / 'Projects/NUCLEO-WBA65RI/Applications/BLE/BLE_p2pServer/STM32_WPAN/Target/linklayer_plat.c'
				platform_file.parent.mkdir(parents=True)
				platform_file.touch()
			sentinel = target / 'user-work.txt'
			sentinel.write_text('keep me')
			git = root / 'git'
			git.write_text('''#!/bin/bash
printf '%s\\n' "$*" >> "$CALLS"
case "$1" in
 describe) printf '%s\\n' "$TAG" ;;
 pull) exit 42 ;;
 clone) mkdir -p "${@: -1}" ;;
esac
[[ "$1" != "$FAIL" ]]
''')
			git.chmod(0o755)
			source = (INSTALLER / f'clone_iosonata_sdk_{platform}.sh').read_text()
			functions = '\n'.join(re.findall(r'^clone_\w+\(\) \{.*?^\}', source, re.M | re.S))
			args = {'clone_or_update_repo': 'url "$EXT/STM32CubeWBA"',
				'clone_pinned_repo': 'url "$EXT/STM32CubeWBA" v1.10.0',
				'clone_cube_wba': ''}[helper]
			env = dict(os.environ, PATH=str(root) + os.pathsep + os.environ['PATH'],
				EXT=str(root), MODE=mode, TAG=tag, FAIL=fail, CALLS=str(root / 'calls'))
			result = subprocess.run([shell, '-c', 'set -e\n' + functions + '\n' + helper + ' ' + args],
				env=env, text=True, capture_output=True)
			calls = (root / 'calls').read_text() if (root / 'calls').exists() else ''
			return result, sentinel.exists(), calls

	def test_failed_update_preserves_checkout(self):
		for platform, shell in self.shells():
			with self.subTest(platform=platform):
				result, preserved, calls = self.run_helper(platform, shell, 'clone_or_update_repo')
				self.assertNotEqual(result.returncode, 0)
				self.assertTrue(preserved)
				self.assertNotIn('clone ', calls)

	def test_pinned_mismatch_and_incomplete_cube_preserve_checkout(self):
		for platform, shell in self.shells():
			for helper in ['clone_pinned_repo', 'clone_cube_wba']:
				for tag in ['', 'other', 'v1.10.0']:
					if helper == 'clone_pinned_repo' and tag == 'v1.10.0':
						continue
					with self.subTest(platform=platform, helper=helper, tag=tag):
						result, preserved, calls = self.run_helper(platform, shell, helper, tag=tag)
						self.assertNotEqual(result.returncode, 0)
						self.assertTrue(preserved)
						self.assertNotIn('clone ', calls)

	def test_cube_uses_local_cmsis_and_requires_platform_files(self):
		for platform, shell in self.shells():
			for state in ['hal-only', 'complete']:
				with self.subTest(platform=platform, state=state):
					result, preserved, calls = self.run_helper(platform, shell, 'clone_cube_wba', tag='v1.10.0', cube_state=state)
					self.assertEqual(result.returncode == 0, state == 'complete')
					self.assertTrue(preserved)
					self.assertNotIn('clone ', calls)

	def test_matching_pin_is_kept(self):
		for platform, shell in self.shells():
			result, preserved, calls = self.run_helper(platform, shell, 'clone_pinned_repo', tag='v1.10.0')
			self.assertEqual(result.returncode, 0, result.stderr)
			self.assertTrue(preserved)
			self.assertNotIn('clone ', calls)

	def test_force_replaces_checkout_and_propagates_clone_failure(self):
		for platform, shell in self.shells():
			for helper in ['clone_pinned_repo', 'clone_cube_wba']:
				result, preserved, calls = self.run_helper(platform, shell, helper, mode='force', fail='clone')
				self.assertNotEqual(result.returncode, 0)
				self.assertFalse(preserved)
				self.assertIn('clone ', calls)


	def test_build_failures_keep_separate_logs(self):
		for platform in ['linux', 'macos']:
			with tempfile.TemporaryDirectory() as directory:
				root = Path(directory)
				for name in ['.project', '.cproject']:
					(root / name).touch()
				ide = root / 'ide'
				ide.write_text('#!/bin/bash\necho compiler-failure\nexit 23\n')
				ide.chmod(0o755)
				source = (INSTALLER / f'build_iosonata_lib_{platform}.sh').read_text()
				function = re.search(r'^build_project\(\) \{.*?^\}', source, re.M | re.S).group()
				command = function + '\nfor name in first second; do if build_project "$PROJECT" "$name"; then exit 99; fi; done'
				result = subprocess.run(['bash', '-c', command], text=True, capture_output=True,
					env=dict(os.environ, IDE_BIN=str(ide), IDE_NAME='mock', PROJECT=str(root)))
				self.assertEqual(result.returncode, 0, result.stderr)
				logs = re.findall(r'Log: (\S+)', result.stdout)
				try:
					self.assertEqual(len(set(logs)), 2)
					for log in logs:
						self.assertIn('compiler-failure', Path(log).read_text())
				finally:
					for log in logs:
						Path(log).unlink(missing_ok=True)

	def test_help_and_invalid_arguments_have_no_side_effects(self):
		for platform, shell in self.shells():
			with tempfile.TemporaryDirectory() as directory:
				env = dict(os.environ, HOME=directory, PATH=directory)
				for args, expected in [(['--help'], 0), (['--home'], 1), (['--mode'], 1), (['--mode', 'typo'], 1)]:
					result = subprocess.run([shutil.which(shell), str(INSTALLER / f'clone_iosonata_sdk_{platform}.sh'), *args],
						env=env, text=True, capture_output=True, timeout=5)
					self.assertEqual(result.returncode, expected, result.stderr)
				self.assertEqual(list(Path(directory).iterdir()), [])
		with tempfile.TemporaryDirectory() as directory:
			Path(directory, 'cat').symlink_to(shutil.which('cat'))
			for arg in ['--help', '--version']:
				result = subprocess.run([shutil.which('bash'), str(INSTALLER / 'install_iocdevtools_linux.sh'), arg],
					env=dict(os.environ, HOME=directory, PATH=directory), text=True, capture_output=True)
				self.assertEqual(result.returncode, 0, result.stderr)
			self.assertEqual([f.name for f in Path(directory).iterdir()], ['cat'])


if __name__ == '__main__':
	unittest.main(verbosity=2)
