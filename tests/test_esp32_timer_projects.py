#!/usr/bin/env python3
"""All C-series libraries must link one shared timer implementation."""
from pathlib import Path
import unittest
import xml.etree.ElementTree as ET

ROOT = Path(__file__).resolve().parents[1]
PORT = ROOT / 'RISCV/Espressif'


class Esp32TimerProjects(unittest.TestCase):
    def test_shared_driver_and_generic_wrapper(self):
        for chip in ('ESP32C2', 'ESP32C3', 'ESP32C5', 'ESP32C6', 'ESP32C61'):
            with self.subTest(chip=chip):
                project = PORT / 'ESP32C' / chip / 'lib/ioc/.project'
                links = [x.findtext('locationURI') for x in
                         ET.parse(project).findall('.//linkedResources/link')]
                self.assertEqual(links.count('PARENT-4-PROJECT_LOC/src/timer_esp32.cpp'), 1)
                self.assertEqual(links.count('PARENT-6-PROJECT_LOC/src/coredev/timer.cpp'), 1)
                self.assertFalse(any('timer_esp32c' in x for x in links))


if __name__ == '__main__':
    unittest.main()
