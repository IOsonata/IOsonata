#!/usr/bin/env python3
"""ESP32-C3 Timer Group source and interrupt ownership checks."""
from pathlib import Path
import unittest
import xml.etree.ElementTree as ET

ROOT = Path(__file__).resolve().parents[1]
PORT = ROOT / "RISCV" / "Espressif"
SRC = PORT / "src" / "timer_esp32c3.cpp"


class Esp32c3TimerTests(unittest.TestCase):
    def test_project_links_hardware_timer(self):
        project = ET.parse(PORT / "ESP32C" / "ESP32C3" / "lib" / "ioc" / ".project")
        links = [
            x for x in project.findall(".//linkedResources/link")
            if x.findtext("name") == "src/coredev/timer_esp32c3.cpp"
        ]
        self.assertEqual(len(links), 1)
        self.assertEqual(links[0].findtext("locationURI"),
                         "PARENT-4-PROJECT_LOC/src/timer_esp32c3.cpp")
        self.assertTrue(SRC.is_file())

    def test_two_timg_counters_and_interrupts(self):
        content = SRC.read_text()
        self.assertIn("#define ESP32C3_TIMER_COUNT\t\t2", content)
        self.assertIn("ESP32C3_TIMG0_BASE", content)
        self.assertIn("ESP32C3_TIMG1_BASE", content)
        self.assertIn("Esp32EnableSourceIrq(source, cpuint", content)
        self.assertIn("ESP32C3_TG0_T0_SOURCE\t32UL", content)
        self.assertIn("ESP32C3_TG1_T0_SOURCE\t34UL", content)
        self.assertIn("ESP32C3_T0_ALARM_EN", content)
        self.assertIn("ESP32C3_T0_UPDATE_TRIG", content)
        self.assertIn("Esp32c3ReadCount(", content)
        self.assertIn("bool TimerInit(", content)
        self.assertIn("return false;", content)
        self.assertNotIn("malloc(", content)


if __name__ == "__main__":
    unittest.main()
