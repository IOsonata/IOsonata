#!/usr/bin/env python3
"""Check ESP32 managed serial examples and their generic source dependencies."""
from pathlib import Path
import re
import unittest
import xml.etree.ElementTree as ET
from urllib.parse import unquote

ROOT = Path(__file__).resolve().parents[1]
BASE = ROOT / 'RISCV/Espressif/ESP32C'
CHIPS = ('ESP32C2', 'ESP32C3', 'ESP32C5', 'ESP32C6', 'ESP32C61')


EXAMPLES = {'UartPrbsRxTest': 'uart/uart_prbs_rx.cpp', 'UartRetarget': 'uart/uart_retarget_demo.cpp', 'UartSlipPrbsTx': 'uart/uart_slip_prbs_tx.cpp', 'UartSlipPrbsRx': 'uart/uart_slip_prbs_rx.cpp', 'I2cMaster': 'i2c/i2c_master_demo.cpp', 'SpiMaster': 'spi/spi_master_demo.cpp'}


class SerialExamples(unittest.TestCase):
    def test_managed_projects(self):
        for chip in CHIPS:
            examples = dict(EXAMPLES)
            examples['SpiMasterSlave'] = 'spi/spi_master_slave.cpp'
            if chip != 'ESP32C2':
                examples['I2cSlave'] = 'i2c/i2c_slave_demo.cpp'
            if chip == 'ESP32C6':
                examples['I2cMasterSlave'] = 'i2c/i2c_master_slave.cpp'
            for app, source in examples.items():
                with self.subTest(chip=chip, app=app):
                    project = BASE / chip / ('exemples/' + app + '/ioc')
                    paths = []
                    for link in ET.parse(project / '.project').findall('.//link'):
                        uri = unquote(link.findtext('locationURI'))
                        if uri == 'virtual:/virtual':
                            continue
                        match = re.fullmatch(r'PARENT-(\d+)-PROJECT_LOC/(.+)', uri)
                        self.assertIsNotNone(match)
                        path = (project.parents[int(match[1]) - 1] / match[2]).resolve()
                        self.assertTrue(path.is_file(), path)
                        paths.append(path)
                    self.assertIn(ROOT / 'exemples' / source, paths)
                    self.assertEqual(sum(p.suffix == '.cpp' for p in paths), 1)
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
                            self.assertIn(chip, [x.get('value') for x in options[lang + '.compiler.defs']])
                            for inc in options[lang + '.compiler.include.paths']:
                                self.assertTrue((build / inc.get('value').strip('"')).resolve().is_dir())
                            scripts = list(options[lang + '.linker.scriptfile'])
                            self.assertEqual(len(scripts), 1)
                            path = (build / scripts[0].get('value').strip('"')).resolve()
                            self.assertEqual(path, BASE / 'ldscript' / ('gcc_' + chip.lower() + '.ld'))
                            self.assertEqual([x.get('value') for x in options[lang + '.linker.libs']],
                                             ['IOsonata_' + chip])

    def test_console_dependencies(self):
        for chip in CHIPS:
            links = ET.parse(BASE / chip / 'lib/ioc/.project').findall('.//link')
            uris = [x.findtext('locationURI') for x in links]
            self.assertEqual(uris.count('PARENT-4-PROJECT_LOC/src/spi_esp32.cpp'), 1)
            self.assertEqual(uris.count('PARENT-6-PROJECT_LOC/src/coredev/spi.cpp'), 1)
            self.assertEqual(uris.count('PARENT-6-PROJECT_LOC/src/coredev/spi_soft.cpp'), 1)
            for filename in ('stddev.c', 'uart_retarget.c', 'prbs.c', 'slip_intrf.cpp'):
                self.assertEqual(uris.count('PARENT-6-PROJECT_LOC/src/' + filename), 1)


if __name__ == '__main__':
    unittest.main()
