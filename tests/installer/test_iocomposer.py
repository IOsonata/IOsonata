#!/usr/bin/env python3
"""Offline IOcomposer setup and website handoff tests; no system installation."""
import json
import os
from pathlib import Path
import shutil
import sys
import subprocess
import tempfile
import unittest
import zipfile

INSTALLER = Path(__file__).resolve().parents[2] / 'Installer'


class IOcomposerTests(unittest.TestCase):
	def fixture(self, root):
		bin_dir = root / 'bin'
		bin_dir.mkdir()
		for name, content in {
			'sudo': '#!/bin/bash\nexec "$@"\n',
			'curl': '''#!/bin/bash
url=""
output=""
while [[ $# -gt 0 ]]; do
 case "$1" in
  -o) output="$2"; shift 2 ;;
  https:*) url="$1"; shift ;;
  *) shift ;;
 esac
done
printf '%s\\n' "$url" >> "$FIXTURE/calls"
[[ -z "$FAIL_URL" || "$url" != *"$FAIL_URL"* ]] || exit 22
case "$url" in
 *contents/plugin*) cp "$FIXTURE/listing.json" "$output" ;;
 *.jar) cp "$FIXTURE/plugin.jar" "$output" ;;
 *.bmp) printf 'splash' > "$output" ;;
 *) cp "$FIXTURE/child.sh" "$output" ;;
esac
''',
		}.items():
			file = bin_dir / name
			file.write_text(content)
			file.chmod(0o755)
		with zipfile.ZipFile(root / 'plugin.jar', 'w') as archive:
			archive.writestr('META-INF/MANIFEST.MF', 'Manifest-Version: 1.0\n')
		(root / 'listing.json').write_text(json.dumps([
			{'name': f'com.iocomposer.embedcdt.{kind}_{version}.jar'}
			for kind in ['ai', 'ui'] for version in ['0.0.9', '0.0.10', '0.0.2']
		]))
		# The wizard comes from the IOsonata clone of the SDK root, not from the website
		self.wizard(root, '0.1.7')
		return dict(os.environ, PATH=str(bin_dir) + os.pathsep + os.environ['PATH'],
			FIXTURE=str(root), FAIL_URL='', IOCOMPOSER_AI_PLUGIN_URL='', PYTHON3=sys.executable,
			ROOT=str(root / 'sdk'))

	def wizard(self, root, version):
		folder = root / 'sdk' / 'IOsonata' / 'Installer' / 'eclipse_plugin'
		folder.mkdir(parents=True, exist_ok=True)
		for jar in folder.glob('org.iosonata.embedcdt.templates.wizard_*.jar'):
			jar.unlink()
		with zipfile.ZipFile(folder / f'org.iosonata.embedcdt.templates.wizard_{version}.jar', 'w') as archive:
			archive.writestr('META-INF/MANIFEST.MF', f'Manifest-Version: 1.0\nBundle-Version: {version}\n')

	def setup_runtime(self, root, platform):
		runtime = root / 'custom IDE path'
		if platform == 'macos':
			runtime = runtime / 'Contents' / 'Eclipse'
		(runtime / 'plugins' / 'org.eclipse.epp.package.embedcpp').mkdir(parents=True)
		(runtime / 'dropins').mkdir()
		(runtime / 'eclipse.ini').write_text('-startup\nlauncher.jar\n-name\nEclipse\n-pluginCustomization\nold.ini\n-vmargs\n-Diosonata_loc=/my/sdk\n')
		(runtime / 'plugin_customization.ini').write_text('org.eclipse.core.resources/pathvariable.IOSONATA_LOC=/my/sdk\n')
		for kind in ['ai', 'ui']:
			(runtime / 'dropins' / f'com.iocomposer.embedcdt.{kind}.jar').write_text('existing-plugin')
		return runtime

	def run_setup(self, platform, runtime, env):
		source = (INSTALLER / f'install_iocdevtools_{platform}.sh').read_text()
		start = source.index('iocomposer_plugin_url() {')
		end = source.index('\n# ---------------------------------------------------------\n# Install xPack toolchain', start)
		return subprocess.run(['bash', '-c', 'set -euo pipefail\n' + source[start:end] + '\ninstall_iocomposer "$RUNTIME"'],
			env=dict(env, RUNTIME=str(runtime)), text=True, capture_output=True)

	def test_plugins_preferences_and_repeat_install(self):
		for platform in ['linux', 'macos']:
			with self.subTest(platform=platform), tempfile.TemporaryDirectory() as directory:
				root = Path(directory)
				env = self.fixture(root)
				runtime = self.setup_runtime(root, platform)
				for _ in range(2):
					result = self.run_setup(platform, runtime, env)
					self.assertEqual(result.returncode, 0, result.stderr)
				ini = (runtime / 'eclipse.ini').read_text()
				self.assertEqual(ini.count('-pluginCustomization\n'), 1)
				self.assertEqual(ini.count('-name\nIOcomposer\n'), 1)
				self.assertIn('-Diosonata_loc=/my/sdk', ini)
				self.assertLess(ini.index('-name'), ini.index('-vmargs'))
				prefs = (runtime / 'plugin_customization.ini').read_text()
				self.assertIn('IOSONATA_LOC=/my/sdk', prefs)
				self.assertEqual(prefs.count('defaultPerspectiveId='), 1)
				self.assertIn('_0.0.10.jar', (root / 'calls').read_text())
				self.assertNotIn('templates.wizard', (root / 'calls').read_text())
				with zipfile.ZipFile(runtime / 'dropins' / 'org.iosonata.embedcdt.templates.wizard.jar') as archive:
					self.assertIn('Bundle-Version: 0.1.7', archive.read('META-INF/MANIFEST.MF').decode())
				self.assertNotIn('embedcdt.bletest', (root / 'calls').read_text())
				if platform == 'macos':
					self.assertIn('../Eclipse/plugin_customization.ini', ini)

	def test_failed_required_plugin_keeps_both_existing_plugins(self):
		for platform in ['linux', 'macos']:
			with self.subTest(platform=platform), tempfile.TemporaryDirectory() as directory:
				root = Path(directory)
				env = self.fixture(root)
				env['FAIL_URL'] = 'com.iocomposer.embedcdt.ui_'
				runtime = self.setup_runtime(root, platform)
				result = self.run_setup(platform, runtime, env)
				self.assertNotEqual(result.returncode, 0)
				for jar in (runtime / 'dropins').glob('*.jar'):
					self.assertEqual(jar.read_text(), 'existing-plugin')
				self.assertIn('-name\nEclipse', (runtime / 'eclipse.ini').read_text())

	def test_old_wizard_keeps_existing_plugins(self):
		for platform in ['linux', 'macos']:
			with self.subTest(platform=platform), tempfile.TemporaryDirectory() as directory:
				root = Path(directory)
				env = self.fixture(root)
				self.wizard(root, '0.1.6')
				runtime = self.setup_runtime(root, platform)
				result = self.run_setup(platform, runtime, env)
				self.assertNotEqual(result.returncode, 0)
				self.assertIn('requires wizard 0.1.7 or newer', result.stderr)
				for jar in (runtime / 'dropins').glob('*.jar'):
					self.assertEqual(jar.read_text(), 'existing-plugin')

	def test_invalid_archive_keeps_existing_plugins(self):
		with tempfile.TemporaryDirectory() as directory:
			root = Path(directory)
			env = self.fixture(root)
			(root / 'plugin.jar').write_text('<html>not a plugin</html>')
			runtime = self.setup_runtime(root, 'linux')
			self.assertNotEqual(self.run_setup('linux', runtime, env).returncode, 0)
			self.assertEqual((runtime / 'dropins/com.iocomposer.embedcdt.ai.jar').read_text(), 'existing-plugin')



if __name__ == '__main__':
	unittest.main(verbosity=2)
