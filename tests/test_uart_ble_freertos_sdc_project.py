#!/usr/bin/env python3
"""Validate UartBleFreeRTOS nRF52832 SoftDevice Controller configurations."""
from pathlib import Path
import unittest
import xml.etree.ElementTree as ET

ROOT = Path(__file__).resolve().parents[1]
PROJECT = (ROOT / "ARM" / "Nordic" / "nRF52" / "nRF52832"
           / "exemples" / "UartBleFreeRTOS" / "ioc" / ".cproject")
SCRIPT = ROOT / "ARM" / "Nordic" / "nRF52" / "ldscript" / "gcc_nrf52832_xxaa.ld"
SOURCE_PROJECT = PROJECT.with_name(".project")
SDC_PATHS = (
    "mpsl/lib/nrf52/hard-float",
    "mpsl/fem/common/lib/nrf52/hard-float",
    "softdevice_controller/lib/nrf52/hard-float",
)


def option_values(config, suffix):
    return [
        child.get("value", "").strip('"')
        for opt in config.iter("option")
        if opt.get("superClass", "").endswith(suffix)
        for child in opt.findall("listOptionValue")
    ]


class UartBleFreeRtosSdcProjectTests(unittest.TestCase):
    def test_sdc_build_configurations(self):
        self.assertTrue(SCRIPT.is_file())
        root = ET.parse(PROJECT).getroot()
        configurations = {}
        for cfg in root.findall(".//cconfiguration"):
            config = cfg.find(".//configuration")
            if config is not None:
                configurations[config.get("name")] = cfg

        linked = {
            item.findtext("name", ""): item.findtext("locationURI", "")
            for item in ET.parse(SOURCE_PROJECT).findall(".//linkedResources/link")
        }
        sdk_compat = "src/FreeRTOS/sdk_compat"
        self.assertEqual(linked[sdk_compat], "virtual:/virtual")
        for src in ("app_error_weak.c", "nrf_drv_clock.c"):
            name = sdk_compat + "/" + src
            self.assertIn(name, linked)
            self.assertIn("external/nRF5_SDK/", linked[name])
        for name in ("Debug", "Release"):
            cfg = configurations[name]
            excludes = [item for entry in cfg.findall(".//sourceEntries/entry")
                        for item in entry.get("excluding", "").split("|")]
            self.assertIn(sdk_compat, excludes)

        for name in ("Debug_SDC", "Release_SDC"):
            with self.subTest(config=name):
                cfg = configurations[name]
                exclusions = [item for entry in cfg.findall(".//sourceEntries/entry")
                              for item in entry.get("excluding", "").split("|")]
                self.assertNotIn(sdk_compat, exclusions)
                builder = cfg.find(".//builder")
                self.assertIsNotNone(builder)
                self.assertTrue(builder.get("buildPath", "").endswith("/" + name))
                for kind in ("c", "cpp"):
                    defs = option_values(cfg, "." + kind + ".compiler.defs")
                    self.assertIn("NRFXLIB_SDC", defs)
                    self.assertIn("NRF52832_XXAA", defs)
                    for outdated in ("S132", "SOFTDEVICE_PRESENT"):
                        self.assertNotIn(outdated, defs)

                    # The SDK FreeRTOS port still requires Nordic utility,
                    # scheduling, logging and S132 header declarations even
                    # when the linked radio controller is NRFXLIB_SDC.
                    include_paths = option_values(cfg, "." + kind + ".compiler.include.paths")
                    for suffix in (
                        "components/device",
                        "components/libraries/util",
                        "components/libraries/strerror",
                        "components/libraries/scheduler",
                        "components/libraries/log",
                        "components/libraries/log/src",
                        "components/libraries/experimental_section_vars",
                        "components/softdevice/s132/headers",
                        "components/softdevice/common",
                    ):
                        self.assertTrue(any(p.endswith("/external/nRF5_SDK/" + suffix)
                                            for p in include_paths), (name, kind, suffix))

                    scripts = option_values(cfg, "." + kind + ".linker.scriptfile")
                    self.assertEqual(scripts, ["../../../../../ldscript/gcc_nrf52832_xxaa.ld"])
                    self.assertEqual(
                        (PROJECT.parent / name / scripts[0]).resolve(),
                        SCRIPT.resolve(),
                    )
                    libraries = option_values(cfg, "." + kind + ".linker.libs")
                    self.assertIn("IOsonata_nRF52832", libraries)
                    self.assertNotIn("IOsonata_nRF52832_FPU", libraries)
                    paths = option_values(cfg, "." + kind + ".linker.paths")
                    self.assertIn("../../../../lib/ioc/" + name, paths)
                    for suffix in SDC_PATHS:
                        self.assertTrue(any(path.endswith("/" + suffix)
                                            for path in paths), (name, kind, suffix))
                cpp_libs = option_values(cfg, ".cpp.linker.libs")
                for required in ("mpsl", "mpsl_fem_common",
                                 "softdevice_controller_multirole"):
                    self.assertIn(required, cpp_libs)


if __name__ == "__main__":
    unittest.main()
