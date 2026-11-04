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

    def test_nrfx_stable_mdk_compiler_abstraction_header_path(self):
        tree = ET.parse(PROJECT / "ioc/.cproject")
        for cfg in tree.findall(".//configuration"):
            with self.subTest(config=cfg.get("name")):
                path_options = [opt for opt in cfg.findall(".//option")
                                if opt.get("name") == "Include paths (-I)"]
                self.assertEqual(len(path_options), 2)
                for option in path_options:
                    values = [item.get("value", "")
                              for item in option.findall("listOptionValue")]
                    self.assertTrue(any(
                        value.strip('"').endswith("external/nrfx/bsp/stable/mdk")
                        for value in values
                    ))
                    self.assertTrue(any(
                        value.strip('"').endswith("external/nrfx/bsp/stable/mdk/common")
                        for value in values
                    ))
                    self.assertTrue(any(
                        value.strip('"').endswith("external/nrfx/bsp/stable")
                        for value in values
                    ))

    def test_usb_sources_and_cli_linker_layout(self):
        project = ET.parse(PROJECT / "ioc/.project")
        links = {
            link.findtext("name", ""): link.findtext("locationURI", "")
            for link in project.findall(".//linkedResources/link")
        }
        required = (
            "app_usbd.c",
            "app_usbd_core.c",
            "app_usbd_string_desc.c",
        )
        for name in required:
            self.assertEqual(
                links.get("src/nRF5_SDK/libraries/usbd/" + name),
                "PARENT-8-PROJECT_LOC/external/nRF5_SDK/components/libraries/usbd/" + name
            )
        self.assertEqual(
            links.get("src/nRF5_SDK/libraries/usbd/class/cdc/acm/app_usbd_cdc_acm.c"),
            "PARENT-8-PROJECT_LOC/external/nRF5_SDK/components/libraries/usbd/class/cdc/acm/app_usbd_cdc_acm.c"
        )
        self.assertNotIn("src/nRF5_SDK/libraries/util/app_util_platform.c", links)
        self.assertNotIn("src/nRF5_SDK/libraries/usbd/app_usbd_serial_num.c", links)
        self.assertNotIn("src/nrf_cli_cdc_acm.c", links)
        self.assertIn("src/nRF5_SDK/libraries/cli/cdc_acm/nrf_cli_cdc_acm.c", links)
        settings = ET.parse(PROJECT / "ioc/.cproject")
        source_configs = settings.findall(".//cconfiguration")
        self.assertEqual(len(source_configs), 2)
        for cfg in source_configs:
            entries = cfg.findall(".//sourceEntries/entry")
            self.assertEqual(len(entries), 1)
            excluded = entries[0].get("excluding", "").split("|")
            self.assertIn("src/nrf_cli_cdc_acm.c", excluded)
        configs = settings.findall(".//configuration")
        self.assertEqual({cfg.get("name") for cfg in configs},
                         {"Debug", "Release"})
        for cfg in configs:
            scripts = [
                item.get("value", "")
                for opt in cfg.findall(".//option")
                if opt.get("name") == "Script files (-T)"
                for item in opt.findall("listOptionValue")
            ]
            self.assertTrue(scripts)
            self.assertTrue(all("gcc_nrf52840_xxaa_cli.ld" in v for v in scripts))

    def test_sdk_usb_cdc_acm_configuration(self):
        source = (PROJECT / "src/sdk_config.h").read_text(encoding="utf-8")
        self.assertIn("#define USBD_CONFIG_IRQ_PRIORITY 6", source)
        self.assertIn("#define APP_USBD_CDC_ACM_ENABLED 1", source)

    def test_radio_test_includes_usb_driver_header(self):
        text = (PROJECT / "src/main.c").read_text(encoding="utf-8")
        self.assertIn('#include "nrf_drv_usbd.h"', text)
        self.assertIn('#include "nrf_drv_clock.h"', text)

    def test_iosonata_nrfx_glue_supports_v4_timer(self):
        text = NRFX_GLUE.read_text(encoding="utf-8")
        self.assertIn("#define NRFX_CTZ(value)", text)


if __name__ == "__main__":
    unittest.main()
