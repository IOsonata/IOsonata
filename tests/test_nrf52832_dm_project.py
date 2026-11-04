#!/usr/bin/env python3
"""nRF52832 distance measurement reflector must link actual Nordic radio code."""
from pathlib import Path
import unittest
import xml.etree.ElementTree as ET

ROOT = Path(__file__).resolve().parents[1] / "ARM" / "Nordic" / "nRF52" / "nRF52832" / "exemples" / "DistMeasureRxDemo"


class NrfDmProjectTests(unittest.TestCase):
    def test_reflector_uses_complete_sdk_configuration(self):
        text = (ROOT / "src" / "DistMeasureRxDemo.cpp").read_text()
        self.assertIn("nrf_dm_init(&s_PpiCfg, &s_AntennaCfg, NRF_TIMER2)", text)
        self.assertIn("NRF_DM_ROLE_REFLECTOR", text)
        self.assertIn("NRF_DM_RANGING_MODE_MCPD", text)
        self.assertIn("nrf_dm_configure(&s_DmCfg)", text)
        self.assertIn("nrf_dm_proc_execute(", text)
        self.assertIn("nrf_dm_populate_report(&s_LastReport)", text)
        self.assertNotIn("nrf_dm_init(&s_PpiCfg, );", text)

    def test_all_linkers_select_supplied_hard_float_library(self):
        for config in ET.parse(ROOT / "ioc" / ".cproject").findall(".//cconfiguration"):
            name = config.find(".//configuration").get("name")
            with self.subTest(configuration=name):
                paths = [v.get("value", "") for v in config.findall('.//option[@valueType="libPaths"]/listOptionValue')]
                libs = [v.get("value", "") for v in config.findall('.//option[@valueType="libs"]/listOptionValue')]
                self.assertEqual(sum("nrf_dm/lib/nrf52832/hard-float" in x for x in paths), 2)
                self.assertEqual(libs.count("nrf_dm"), 2)


if __name__ == "__main__":
    unittest.main()
