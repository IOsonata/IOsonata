"""Validate managed C2/C5/C61 project links, flags, and generic application use."""
from pathlib import Path
import re
import unittest
import xml.etree.ElementTree as ET

ROOT = Path(__file__).resolve().parents[1]
BASE = ROOT / 'RISCV/Espressif/ESP32C'


class StartupProjects(unittest.TestCase):
    def test_managed_projects(self):
        for chip in ('ESP32C2', 'ESP32C5', 'ESP32C61'):
            for sub in ('lib', 'exemples/UartPrbsTxTest', 'exemples/UartLoopback'):
                project = BASE / chip / sub / 'ioc'
                with self.subTest(chip=chip, project=sub):
                    links = ET.parse(project / '.project').findall('.//link')
                    paths = []
                    for link in links:
                        uri = link.findtext('locationURI')
                        if uri == 'virtual:/virtual':
                            continue
                        match = re.fullmatch(r'PARENT-(\d+)-PROJECT_LOC/(.+)', uri)
                        self.assertIsNotNone(match)
                        path = project.parents[int(match[1]) - 1] / match[2]
                        self.assertTrue(path.is_file(), path)
                        paths.append(path.resolve())
                    if sub == 'lib':
                        self.assertIn(ROOT / 'RISCV/src/ResetEntry.c', paths)
                        self.assertIn(ROOT / 'RISCV/Espressif/src/Vectors_esp32.c', paths)
                        self.assertEqual(len(paths), len(set(paths)))
                    else:
                        source = 'uart_prbs_tx.cpp' if sub.endswith('UartPrbsTxTest') else 'uart_loopback.cpp'
                        self.assertIn(ROOT / 'exemples/uart' / source, paths)
                    configs = ET.parse(project / '.cproject').findall('.//cconfiguration')
                    self.assertEqual(len(configs), 2)
                    for cfg in configs:
                        config = cfg.find('.//configuration')
                        build = project / config.get('name')
                        options = {o.get('superClass').split('.option.')[-1]: o
                                   for o in cfg.findall('.//option')}
                        self.assertEqual(options['target.isa.extensions'].get('value'), 'zicsr')
                        self.assertTrue(options['target.isa.base'].get('value').endswith('rv32i'))
                        self.assertEqual(options['target.isa.atomic'].get('value'), 'false')
                        self.assertTrue(options['target.abi.integer'].get('value').endswith('ilp32'))
                        for lang in ('c', 'cpp'):
                            defs = [x.get('value') for x in options[lang + '.compiler.defs']]
                            self.assertIn(chip, defs)
                            for inc in options[lang + '.compiler.include.paths']:
                                self.assertTrue((build / inc.get('value').strip('"')).resolve().is_dir())
                            if sub != 'lib':
                                scripts = list(options[lang + '.linker.scriptfile'])
                                self.assertEqual(len(scripts), 1)
                                path = (build / scripts[0].get('value').strip('"')).resolve()
                                self.assertEqual(path, BASE / 'ldscript' / ('gcc_' + chip.lower() + '.ld'))
                                self.assertTrue(path.is_file())
                                libs = [x.get('value') for x in options[lang + '.linker.libs']]
                                self.assertEqual(libs, ['IOsonata_' + chip])


if __name__ == '__main__':
    unittest.main()
