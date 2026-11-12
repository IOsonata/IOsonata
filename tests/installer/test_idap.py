"""IDAP package upgrades and firmware downloads; no probe is accessed."""
import contextlib
import importlib.util
import io
import json
import os
import subprocess
import sys
from pathlib import Path
import tempfile
import unittest
from unittest.mock import patch
import zipfile

ROOT = Path(__file__).resolve().parents[2]
spec = importlib.util.spec_from_file_location('install_idap', ROOT / 'Installer/install_idap.py')
idap = importlib.util.module_from_spec(spec)
spec.loader.exec_module(idap)


class IdapTests(unittest.TestCase):
    def payloads(self, platform):
        folder, suffix, _ = idap.TOOLS[platform]
        data = {}
        hashes = []
        for tool in ('IDAPnRFProg', 'IDAPSetTarget'):
            buffer = io.BytesIO()
            with zipfile.ZipFile(buffer, 'w') as archive:
                archive.writestr(tool + ('.exe' if platform == 'windows' else ''), b'new ' + tool.encode())
            path = folder + '/' + tool + suffix + '_' + idap.VERSION + '.zip'
            data[path] = buffer.getvalue()
            hashes.append(idap.digest(data[path]))
        firmware = {}
        for board in idap.FIRMWARE:
            path = 'Firmware/' + board + '/build_' + idap.VERSION + '/firmware.bin'
            data[path] = b'firmware ' + board.encode()
            firmware[board] = idap.digest(data[path])
        return data, {platform: (folder, suffix, hashes)}, firmware

    def test_upgrade_skip_repair_and_force_all_platforms(self):
        for platform in idap.TOOLS:
            with self.subTest(platform=platform), tempfile.TemporaryDirectory() as tmp:
                root = Path(tmp) / 'Tools home with spaces'
                name = 'IDAPnRFProg' + ('.exe' if platform == 'windows' else '')
                program = root / 'Tools/bin' / name
                program.parent.mkdir(parents=True)
                program.write_bytes(b'old')
                data, tools, firmware = self.payloads(platform)
                def fetch(url, timeout):
                    return io.BytesIO(data[url.removeprefix(idap.BASE_URL)])
                with patch.object(idap, 'TOOLS', tools), patch.object(idap, 'FIRMWARE', firmware), patch.object(idap.urllib.request, 'urlopen', side_effect=fetch) as request, contextlib.redirect_stdout(io.StringIO()):
                    idap.install(root, platform)
                    self.assertEqual(request.call_count, 5)
                    self.assertEqual(program.read_bytes(), b'new IDAPnRFProg')
                    receipt = json.loads((root / 'Tools/IDAP/installed.json').read_text())
                    self.assertEqual(receipt[name]['version'], idap.VERSION)
                    idap.install(root, platform)
                    self.assertEqual(request.call_count, 5)
                    program.write_bytes(b'damaged')
                    idap.install(root, platform)
                    self.assertEqual(request.call_count, 6)
                    program.unlink()
                    idap.install(root, platform)
                    self.assertEqual(request.call_count, 7)
                    idap.install(root, platform, force=True)
                    self.assertEqual(request.call_count, 12)
                    for board in firmware:
                        self.assertEqual((root / 'Tools/IDAP/Firmware' / board / ('build_' + idap.VERSION) / 'firmware.bin').read_bytes(), b'firmware ' + board.encode())

    def test_failed_download_or_checksum_preserves_tools_and_receipt(self):
        for failure in ('network', 'checksum', 'archive'):
            with self.subTest(failure=failure), tempfile.TemporaryDirectory() as tmp:
                root = Path(tmp)
                program = root / 'Tools/bin/IDAPnRFProg'
                program.parent.mkdir(parents=True)
                program.write_bytes(b'original')
                receipt = root / 'Tools/IDAP/installed.json'
                receipt.parent.mkdir(parents=True)
                receipt.write_text('{}')
                data, tools, firmware = self.payloads('linux')
                calls = 0
                def fetch(url, timeout):
                    nonlocal calls
                    calls += 1
                    if calls == 2:
                        if failure == 'network':
                            raise OSError('download failed')
                        return io.BytesIO(b'invalid')
                    return io.BytesIO(data[url.removeprefix(idap.BASE_URL)])
                if failure == 'archive':
                    tools['linux'][2][1] = idap.digest(b'invalid')
                with patch.object(idap, 'TOOLS', tools), patch.object(idap, 'FIRMWARE', firmware), patch.object(idap.urllib.request, 'urlopen', side_effect=fetch):
                    with self.assertRaises((OSError, ValueError, zipfile.BadZipFile)):
                        idap.install(root, 'linux')
                self.assertEqual(program.read_bytes(), b'original')
                self.assertEqual(receipt.read_text(), '{}')

    def test_failed_staged_write_does_not_truncate_existing_file(self):
        with tempfile.TemporaryDirectory() as tmp:
            target = Path(tmp) / 'tool'
            target.write_bytes(b'old')
            with patch.object(idap.os, 'replace', side_effect=OSError('locked file')):
                with self.assertRaises(OSError):
                    idap.replace_file(target, b'new', True)
            self.assertEqual(target.read_bytes(), b'old')
            self.assertEqual(list(Path(tmp).iterdir()), [target])

    def test_piped_shell_installer_downloads_helper_and_passes_arguments(self):
        for platform in ('macos', 'linux'):
            with self.subTest(platform=platform), tempfile.TemporaryDirectory() as tmp:
                root = Path(tmp)
                curl = root / 'curl'
                curl.write_text('#!/bin/bash\ncp "$FIXTURE" "$4"\n')
                curl.chmod(0o755)
                fixture = root / 'helper.py'
                fixture.write_text('import sys,json,os\nopen(os.environ["ARGS"],"w").write(json.dumps(sys.argv[1:]))\n')
                source = (ROOT / 'Installer' / ('install_iocdevtools_' + platform + '.sh')).read_text()
                start = source.index('install_idap_tools() (')
                end = source.index('\n)', start) + 2
                block = source[start:end]
                env = dict(os.environ, PATH=str(root) + os.pathsep + os.environ['PATH'],
                           ROOT=str(root / 'SDK home'), MODE='force', PYTHON3=sys.executable,
                           FIXTURE=str(fixture), ARGS=str(root / 'args'))
                result = subprocess.run(['bash'], input='set -euo pipefail\n' + block + '\ninstall_idap_tools\n',
                                        env=env, text=True, capture_output=True)
                self.assertEqual(result.returncode, 0, result.stderr)
                self.assertEqual(json.loads((root / 'args').read_text()),
                                 ['--root', str(root / 'SDK home'), '--platform', platform, '--force'])

    def test_installer_entry_points(self):
        for platform in ('macos', 'linux'):
            source = (ROOT / 'Installer' / ('install_iocdevtools_' + platform + '.sh')).read_text()
            self.assertIn('--platform ' + platform, source)
            self.assertIn('"$PYTHON3" "$helper" "${args[@]}"', source)
            self.assertIn('args+=(--force)', source)
            self.assertNotIn('2_1_240807', source)
        source = (ROOT / 'Installer/install_iocdevtools_win.ps1').read_text()
        self.assertIn("'--platform', 'windows'", source)
        self.assertIn('& $PythonExecutablePath $idapHelper @idapArgs', source)
        self.assertNotIn('2_1_240807', source)


if __name__ == '__main__':
    unittest.main()
