"""Check the C6 managed UART examples link shared code and matching libraries."""
from pathlib import Path
import unittest
import xml.etree.ElementTree as ET

ROOT = Path(__file__).resolve().parents[1]
C6 = ROOT / 'RISCV/Espressif/ESP32C/ESP32C6'


class C6UartProjects(unittest.TestCase):
    def test_shared_sources_and_build_settings(self):
        for name, source in (('UartPrbsTxTest', 'uart_prbs_tx.cpp'),
                             ('UartLoopback', 'uart_loopback.cpp')):
            project = C6 / 'exemples' / name / 'ioc'
            tree = ET.parse(project / '.project')
            self.assertEqual(tree.findtext('name'), name)
            links = {x.findtext('name'): x.findtext('locationURI')
                     for x in tree.findall('.//link')}
            self.assertEqual(links['src/' + source],
                             'PARENT-7-PROJECT_LOC/exemples/uart/' + source)
            self.assertEqual(set(links), {'src', 'src/board.h', 'src/' + source})
            configs = ET.parse(project / '.cproject').findall('.//cconfiguration')
            self.assertEqual(len(configs), 2)
            for cfg in configs:
                config = cfg.find('.//configuration')
                build = project / config.get('name')
                self.assertEqual(config.get('artifactExtension'), 'elf')
                self.assertIn('-O binary', config.get('postbuildStep'))
                options = {x.get('superClass').split('.option.')[-1]: x
                           for x in cfg.findall('.//option')}
                self.assertEqual(options['target.isa.extensions'].get('value'), 'zicsr')
                self.assertTrue(options['target.abi.integer'].get('value').endswith('ilp32'))
                for language in ('c', 'cpp'):
                    values = lambda key: [x.get('value').strip('"') for x in
                        options[language + '.' + key].findall('listOptionValue')]
                    self.assertIn('ESP32C6', values('compiler.defs'))
                    for include in values('compiler.include.paths'):
                        self.assertTrue((build / include).resolve().is_dir())
                    self.assertEqual(values('linker.libs'), ['IOsonata_ESP32C6'])
                    self.assertEqual((build / values('linker.paths')[0]).resolve(),
                                     (C6 / 'lib/ioc' / config.get('name')).resolve())
                    self.assertTrue((build / values('linker.scriptfile')[0]).resolve().is_file())

    def test_library_includes_atomic_support(self):
        project = C6 / 'lib/ioc'
        links = {x.findtext('name'): x.findtext('locationURI')
                 for x in ET.parse(project / '.project').findall('.//link')}
        self.assertEqual(links['src/iatomic.c'], 'PARENT-5-PROJECT_LOC/src/iatomic.c')


if __name__ == '__main__':
    unittest.main()
