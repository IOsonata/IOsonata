#!/usr/bin/env python3
"""ESP32-C6 two Timer Groups, clock source and source-vector ownership."""
from pathlib import Path
import unittest
import xml.etree.ElementTree as ET

ROOT = Path(__file__).resolve().parents[1]
ESP = ROOT / "RISCV" / "Espressif"
SOURCE = ESP / "src" / "timer_esp32c6.cpp"
PROJECT = ESP / "ESP32C" / "ESP32C6" / "lib" / "ioc" / ".project"


class Esp32c6TimerTests(unittest.TestCase):
    def test_timer_is_linked_in_managed_library(self):
        links = {
            x.findtext("name", ""): x.findtext("locationURI", "")
            for x in ET.parse(PROJECT).findall(".//linkedResources/link")
        }
        self.assertEqual(links.get("src/coredev/timer_esp32c6.cpp"),
                         "PARENT-4-PROJECT_LOC/src/timer_esp32c6.cpp")
        self.assertTrue(SOURCE.is_file())

    def test_real_timg_devices_and_interrupts(self):
        text = SOURCE.read_text()
        self.assertIn("#define ESP32C6_TIMER_COUNT\t\t2", text)
        for identifier in ("ESP32C6_TIMG0_BASE", "ESP32C6_TIMG1_BASE",
                           "ESP32C6_T0_ALARM_EN", "Esp32c6ReadCount",
                           "Esp32EnableSourceIrq", "bool TimerInit(",
                           "int TimerGetHighFreqDevCount(void)"):
            self.assertIn(identifier, text)
        for pair in (
            ("ESP32C6_TG0_T0_SOURCE", "51UL"),
            ("ESP32C6_TG1_T0_SOURCE", "54UL"),
            ("ESP32C6_TG0_T0_CPUINT", "8UL"),
            ("ESP32C6_TG1_T0_CPUINT", "9UL"),
        ):
            self.assertRegex(text, r"(?m)^#define\s+" + pair[0] +
                             r"\s+" + pair[1] + r"\s*$")
        self.assertNotIn("malloc(", text)

    def test_pcr_clock_selected_before_timer_configuration(self):
        text = SOURCE.read_text()
        self.assertIn("ESP32C6_PCR_BASE", text)
        self.assertIn("(1UL << 20) | (1UL << 22)", text)
        self.assertLess(text.index("Esp32c6EnableTimerClock(pCfg->DevNo)"),
                        text.index("config = ESP32C6_T0_INCREASE"))


if __name__ == "__main__":
    unittest.main()
