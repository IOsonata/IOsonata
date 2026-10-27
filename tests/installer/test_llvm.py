#!/usr/bin/env python3
"""Test LLVM target checks offline; fake compilers do not validate firmware."""
import os
from pathlib import Path
import subprocess
import tempfile
import unittest

INSTALLER = Path(__file__).resolve().parents[2] / 'Installer'


class LLVMTests(unittest.TestCase):
	def source(self, platform):
		source = (INSTALLER / f'install_iocdevtools_{platform}.sh').read_text()
		return source[source.index('verify_llvm() ('):source.index('# Firmware tools share')]

	def test_optional_llvm_skip_success_and_failure(self):
		for platform in ['linux', 'macos']:
			for enabled, status in [('0', 0), ('1', 0), ('1', 9)]:
				with self.subTest(platform=platform, enabled=enabled, status=status):
					command = 'set -eu\n' + self.source(platform) + '\n'
					command += f'WITH_LLVM={enabled}; LLVM_DIR=/test/llvm; LLVM_STATUS=skipped\n'
					command += f'install_llvm() {{ echo attempted; return {status}; }}\n'
					command += 'install_optional_llvm\necho "continued: $LLVM_STATUS"\n'
					result = subprocess.run(['bash', '-c', command], text=True, capture_output=True)
					self.assertEqual(result.returncode, 0, result.stderr)
					self.assertIn('continued:', result.stdout)
					self.assertEqual('attempted' in result.stdout, enabled == '1')
					if enabled == '1':
						self.assertIn('unavailable' if status else 'verified', result.stdout)


	def fixture(self, root):
		bin_dir = root / 'LLVM path' / 'bin'
		bin_dir.mkdir(parents=True)
		for tool in ['clang', 'clang++', 'ld.lld', 'llvm-ar', 'llvm-objcopy', 'llvm-size']:
			file = bin_dir / tool
			file.write_text('''#!/bin/bash
printf '%s %s\\n' "$(basename "$0")" "$*" >> "$LOG"
[[ -z "$FAIL_TARGET" || "$*" != *"$FAIL_TARGET"* ]] || exit 9
''')
			file.chmod(0o755)
		return dict(os.environ, LLVM_DIR=str(bin_dir.parent), LOG=str(root / 'log'), FAIL_TARGET='')

	def test_existing_distribution_checks_both_targets_c_and_cpp(self):
		for platform in ['linux', 'macos']:
			with self.subTest(platform=platform), tempfile.TemporaryDirectory() as d:
				root = Path(d)
				env = self.fixture(root)
				result = subprocess.run(['bash', '-c', 'set -e\n' + self.source(platform) + '\ninstall_llvm'], env=env, capture_output=True)
				self.assertEqual(result.returncode, 0, result.stderr)
				log = (root / 'log').read_text()
				for compiler in ['clang', 'clang++']:
					for target in ['arm-none-eabi', 'riscv32-unknown-elf']:
						self.assertIn(compiler + ' --target=' + target, log)
				self.assertEqual(log.count('llvm-objcopy -O ihex'), 4)
				self.assertIn('-mcpu=cortex-m4', log)
				self.assertIn('-march=rv32imac -mabi=ilp32', log)
				self.assertTrue((root / 'LLVM path/bin/clang').exists())

	def test_failed_riscv_check_is_not_reported_as_success(self):
		with tempfile.TemporaryDirectory() as d:
			root = Path(d)
			env = self.fixture(root)
			env['FAIL_TARGET'] = 'riscv32-unknown-elf'
			result = subprocess.run(['bash', '-c', 'set -e\n' + self.source('linux') + '\ninstall_llvm'], env=env, capture_output=True)
			self.assertNotEqual(result.returncode, 0)
			self.assertTrue((root / 'LLVM path/bin/clang').exists())

	def test_missing_linker_fails_before_compile(self):
		with tempfile.TemporaryDirectory() as d:
			root = Path(d)
			env = self.fixture(root)
			(root / 'LLVM path/bin/ld.lld').unlink()
			result = subprocess.run(['bash', '-c', 'set -e\n' + self.source('macos') + '\ninstall_llvm'], env=env, capture_output=True)
			self.assertNotEqual(result.returncode, 0)
			self.assertFalse((root / 'log').exists())


if __name__ == '__main__':
	unittest.main(verbosity=2)
