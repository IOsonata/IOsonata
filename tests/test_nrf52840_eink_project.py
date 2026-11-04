"""Validate nRF52840 E-Ink SDK headers and build configuration."""
from pathlib import Path
import unittest
import xml.etree.ElementTree as ET


ROOT = Path(__file__).resolve().parents[1]
DEMO = ROOT / "ARM/Nordic/nRF52/nRF52840/exemples/EInkDisplayDemo"


class EInkProjectTest(unittest.TestCase):
    def test_all_compiler_configs_find_nordic_error_headers(self):
        tree = ET.parse(DEMO / "ioc/.cproject")
        configs = tree.findall(".//configuration")
        self.assertEqual({cfg.get("name") for cfg in configs},
                         {"Debug", "Release"})
        for config in configs:
            with self.subTest(config=config.get("name")):
                include_options = [option for option in config.findall(".//option")
                                   if option.get("name") == "Include paths (-I)"]
                self.assertEqual(len(include_options), 2)
                for option in include_options:
                    paths = [entry.get("value", "")
                             for entry in option.findall("listOptionValue")]
                    self.assertTrue(any("external/nRF5_SDK/components/libraries/util" in value
                                        for value in paths))
                    self.assertTrue(any("external/nRF5_SDK/components/softdevice/s140/headers" in value
                                        for value in paths))

    def test_eink_sources_declare_required_functions(self):
        source = (DEMO / "src/ED037TC1.c").read_text(encoding="utf-8")
        for declaration in ("void InitialDisplay(void);",
                            "void Upload_Temperature_LUT(void);"):
            self.assertIn(declaration, source)

    def test_gfx_api_uses_c_linkage_and_links_sdk_source(self):
        source = (DEMO / "src/eink_display_demo.cpp").read_text(encoding="utf-8")
        self.assertIn('extern "C" {\n#include "nrf_lcd.h"\n#include "nrf_gfx.h"\n}', source)
        project = ET.parse(DEMO / "ioc/.project")
        links = {
            link.findtext("name", ""): link.findtext("locationURI", "")
            for link in project.findall(".//linkedResources/link")
        }
        self.assertEqual(
            links.get("src/libraries/gfx/nrf_gfx.c"),
            "PARENT-8-PROJECT_LOC/external/nRF5_SDK/components/libraries/gfx/nrf_gfx.c"
        )

    def test_paper_error_and_assert_headers(self):
        source = (DEMO / "src/paper.c").read_text(encoding="utf-8")
        for header in ("nrf_error.h", "nrf_assert.h", "app_error.h", "app_util.h"):
            self.assertIn('#include "' + header + '"', source)


if __name__ == "__main__":
    unittest.main()
