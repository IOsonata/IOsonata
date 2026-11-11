#!/usr/bin/env python3
"""Check the C3/C6 managed LED and GPIO test projects."""
from pathlib import Path
import re
import unittest
import xml.etree.ElementTree as ET

ROOT = Path(__file__).resolve().parents[1]
BASE = ROOT / 'RISCV/Espressif/ESP32C'


class GpioExamples(unittest.TestCase):
    def test_generic_projects(self):
        for chip in ('ESP32C3', 'ESP32C6'):
            self.assertFalse((BASE / chip / 'exemples/Blinky/ioc/.project').exists())
            for name, source in (('LedDemo', 'led_demo.cpp'), ('PulseTrain', 'pulse_train_test.c')):
                with self.subTest(chip=chip, project=name):
                    project = BASE / chip / 'exemples' / name / 'ioc'
                    paths = []
                    for link in ET.parse(project / '.project').findall('.//link'):
                        uri = link.findtext('locationURI')
                        if uri == 'virtual:/virtual':
                            continue
                        match = re.fullmatch(r'PARENT-(\d+)-PROJECT_LOC/(.+)', uri)
                        self.assertIsNotNone(match)
                        path = (project.parents[int(match[1]) - 1] / match[2]).resolve()
                        self.assertTrue(path.is_file(), path)
                        paths.append(path)
                    self.assertEqual(set(paths), {ROOT / 'exemples/misc' / source,
                                                project.parent / 'src/board.h'})
                    configs = ET.parse(project / '.cproject').findall('.//cconfiguration')
                    self.assertEqual(len(configs), 2)
                    for cfg in configs:
                        config = cfg.find('.//configuration')
                        self.assertIn(config.get('artifactName'), (name, '${ProjName}'))
                        build = project / config.get('name')
                        for option in cfg.findall('.//option'):
                            key = option.get('superClass', '')
                            if key.endswith('target.isa.extensions'):
                                self.assertEqual(option.get('value'), 'zicsr')
                            if key.endswith('linker.libs'):
                                self.assertEqual([x.get('value') for x in option], ['IOsonata_' + chip])
                            if key.endswith('linker.scriptfile'):
                                self.assertEqual([(build / x.get('value').strip('"')).resolve() for x in option],
                                                 [BASE / 'ldscript' / ('gcc_' + chip.lower() + '.ld')])


if __name__ == '__main__':
    unittest.main()
