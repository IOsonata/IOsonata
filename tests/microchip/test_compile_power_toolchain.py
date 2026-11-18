#!/usr/bin/env python3
"""Test compiler discovery without requiring an installed Arm toolchain."""

import importlib.util
import os
from pathlib import Path
import tempfile
import unittest
from unittest import mock

spec = importlib.util.spec_from_file_location(
    'compile_power', Path(__file__).with_name('compile_power.py'))
m = importlib.util.module_from_spec(spec)
spec.loader.exec_module(m)


def make_compilers(bin_dir):
    bin_dir.mkdir(parents=True, exist_ok=True)
    for compiler in ('gcc', 'g++'):
        path = bin_dir / ('arm-none-eabi-' + compiler)
        path.write_text('#!/bin/sh\nexit 0\n')
        path.chmod(0o755)
    return str(bin_dir / 'arm-none-eabi-')


class TestResolve(unittest.TestCase):
    def test_explicit_prefix(self):
        with tempfile.TemporaryDirectory() as tmp:
            prefix = make_compilers(Path(tmp) / 'bin')
            self.assertEqual(m.resolve_toolchain(prefix), prefix)

    def test_invalid_override_does_not_fallback(self):
        with tempfile.TemporaryDirectory() as tmp:
            prefix = make_compilers(Path(tmp) / 'bin')
            with self.assertRaises(SystemExit):
                m.resolve_toolchain(prefix + 'typo-')

    def test_in_path(self):
        with tempfile.TemporaryDirectory() as tmp:
            make_compilers(Path(tmp) / 'bin')
            with mock.patch.dict(os.environ,
                                 {'PATH': str(Path(tmp) / 'bin')}, clear=True):
                self.assertEqual(m.resolve_toolchain(), 'arm-none-eabi-')

    def test_env_home(self):
        with tempfile.TemporaryDirectory() as tmp:
            folder = Path(tmp) / 'custom'
            prefix = make_compilers(folder)
            with mock.patch.dict(os.environ,
                                 {'PATH': '', 'ARM_GCC_HOME': str(folder)},
                                 clear=True):
                self.assertEqual(m.resolve_toolchain(), prefix)

    def test_env_root(self):
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp) / 'root'
            prefix = make_compilers(root / 'bin')
            with mock.patch.dict(os.environ,
                                 {'PATH': '', 'ARM_GCC_ROOT': str(root)},
                                 clear=True):
                self.assertEqual(m.resolve_toolchain(), prefix)

    def test_makefile(self):
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp) / 'repo'
            root.mkdir()
            folder = Path(tmp) / 'compiler' / 'bin'
            prefix = make_compilers(folder)
            (root / 'makefile_path.mk').write_text(
                'ARM_GCC_BIN = ' + str(folder) + '\n')
            with mock.patch.dict(os.environ, {'PATH': ''}, clear=True), \
                    mock.patch.object(m, 'ROOT', root), \
                    mock.patch.object(m, 'XPACK_ROOTS', ()):
                self.assertEqual(m.resolve_toolchain(), prefix)

    def test_xpack_newer_version(self):
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp) / 'xPacks'
            make_compilers(root / 'xpack-arm-none-eabi-9.9.9' / 'bin')
            prefix = make_compilers(
                root / 'xpack-arm-none-eabi-15.2.1' / 'bin')
            with mock.patch.dict(os.environ, {'PATH': ''}, clear=True), \
                    mock.patch.object(m, 'XPACK_ROOTS', (root,)), \
                    mock.patch.object(m, 'ROOT', root):
                self.assertEqual(m.resolve_toolchain(), prefix)

    def test_no_compiler_message(self):
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp)
            with mock.patch.dict(os.environ, {'PATH': ''}, clear=True), \
                    mock.patch.object(m, 'ROOT', root), \
                    mock.patch.object(m, 'XPACK_ROOTS', (root,)):
                with self.assertRaisesRegex(SystemExit, 'not found'):
                    m.resolve_toolchain()


if __name__ == '__main__':
    unittest.main()
