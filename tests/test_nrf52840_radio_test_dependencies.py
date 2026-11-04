"""Guard Nordic SDK and nrfx integration dependencies in radio_test."""

from pathlib import Path
import unittest
import xml.etree.ElementTree as ET


ROOT = Path(__file__).resolve().parents[1]
PROJECT = ROOT / "ARM/Nordic/nRF52/nRF52840/exemples/radio_test"
NRFX_GLUE = ROOT / "ARM/Nordic/include/nrfx_glue.h"


class RadioTestDependencies(unittest.TestCase):
    def test_required_legacy_sdk_headers_have_include_path(self):
        tree = ET.parse(PROJECT / "ioc/.cproject")
        configs = tree.findall(".//configuration")
        self.assertEqual({cfg.get("name") for cfg in configs},
                         {"Debug", "Release"})
        for cfg in configs:
            with self.subTest(config=cfg.get("name")):
                path_options = [opt for opt in cfg.findall(".//option")
                                if opt.get("name") == "Include paths (-I)"]
                self.assertTrue(path_options)
                for opt in path_options:
                    values = [item.get("value", "")
                              for item in opt.findall("listOptionValue")]
                    self.assertTrue(any(
                        "external/nRF5_SDK/integration/nrfx/legacy" in value
                        for value in values))
                    self.assertTrue(any(
                        "external/nrfx/drivers/include" in value
                        for value in values))

    def test_radio_test_includes_usb_driver_header(self):
        text = (PROJECT / "src/main.c").read_text(encoding="utf-8")
        self.assertIn('#include "nrf_drv_usbd.h"', text)
        self.assertIn('#include "nrf_drv_clock.h"', text)

    def test_iosonata_nrfx_glue_supports_v4_timer(self):
        text = NRFX_GLUE.read_text(encoding="utf-8")
        self.assertIn("#define NRFX_CTZ(value)", text)


if __name__ == "__main__":
    unittest.main()
