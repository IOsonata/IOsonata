"""Regression checks for Nordic Mesh 5 examples on nrfx 4.x."""

from pathlib import Path
import unittest
import xml.etree.ElementTree as ET


ROOT = Path(__file__).resolve().parents[1]
MESH_PARTS = ("nRF52832", "nRF52840")
MESH_APPS = ("MeshLgtSwClient", "MeshLgtSwServer")
MESH_RTT = "external/nRF5_SDK_Mesh/external/rtt/include"
SDK_RTT = "external/nRF5_SDK/external/segger_rtt"
LOCAL_HAL = "PARENT-4-PROJECT_LOC/src/mesh_compat/simple_hal.c"


class MeshProjectCompatibilityTest(unittest.TestCase):
    def test_mesh_examples_select_local_gpiote_compatibility(self):
        for part in MESH_PARTS:
            for app in MESH_APPS:
                with self.subTest(part=part, app=app):
                    ioc = (ROOT / "ARM/Nordic/nRF52" / part /
                           "exemples" / app / "ioc")
                    root = ET.parse(ioc / ".project").getroot()
                    links = {
                        item.findtext("name"): item.findtext("locationURI")
                        for item in root.findall(".//linkedResources/link")
                    }
                    self.assertEqual(
                        links["src/nRF5_SDK/examples/common/src/simple_hal.c"],
                        LOCAL_HAL
                    )

    def test_mesh_rtt_headers_precede_sdk_rtt_headers(self):
        for part in MESH_PARTS:
            for app in MESH_APPS:
                with self.subTest(part=part, app=app):
                    path = (ROOT / "ARM/Nordic/nRF52" / part /
                            "exemples" / app / "ioc/.cproject")
                    root = ET.parse(path).getroot()
                    options = root.findall(".//option")
                    checked = 0
                    for option in options:
                        if option.get("name") != "Include paths (-I)":
                            continue
                        paths = [entry.get("value", "")
                                 for entry in option.findall("listOptionValue")]
                        mesh = [i for i, value in enumerate(paths)
                                if MESH_RTT in value]
                        sdk = [i for i, value in enumerate(paths)
                               if SDK_RTT in value]
                        if not sdk:
                            continue
                        self.assertEqual(len(mesh), 1)
                        self.assertEqual(len(sdk), 1)
                        self.assertLess(mesh[0], sdk[0])
                        checked += 1
                    self.assertEqual(checked, 2)

    def test_local_helper_uses_nrfx4_api(self):
        path = ROOT / "ARM/Nordic/nRF52/src/mesh_compat/simple_hal.c"
        text = path.read_text(encoding="utf-8")
        self.assertIn("nrfx_gpiote_input_configure(&m_gpiote", text)
        self.assertIn("nrfx_gpiote_trigger_enable(&m_gpiote", text)
        for removed in ("nrfx_gpiote_out_init(", "nrfx_gpiote_in_init(",
                        "nrfx_gpiote_is_init(", "nrfx_gpiote_in_event_enable("):
            self.assertNotIn(removed, text)


if __name__ == "__main__":
    unittest.main()
