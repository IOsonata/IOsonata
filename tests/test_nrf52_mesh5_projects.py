#!/usr/bin/env python3
"""Guard the four Nordic Mesh v5.0.0 light-switch Eclipse source selections.

Nordic's upstream reference manifests are
examples/light_switch/{client,server}/light_switch_*_7_2_0.emProject
in IOsonata/nRF5_SDK_Mesh, commit 526975f6. The Mesh core implementation,
not merely its headers or application example helpers, must be linked.
"""
from pathlib import Path
import unittest
import xml.etree.ElementTree as ET

ROOT = Path(__file__).resolve().parents[1]
NRF = ROOT / "ARM" / "Nordic" / "nRF52"


class MeshSourceSelectionTests(unittest.TestCase):
    def test_both_boards_and_roles_have_complete_vendor_source_set(self):
        for part in ("nRF52832", "nRF52840"):
            for role in ("Client", "Server"):
                with self.subTest(part=part, role=role):
                    project = NRF / part / "exemples" / ("MeshLgtSw" + role) / "ioc"
                    links = [
                        (link.findtext("name", ""), link.findtext("locationURI", ""))
                        for link in ET.parse(project / ".project").findall(".//linkedResources/link")
                    ]
                    paths = [uri for _, uri in links if uri.endswith(".c")]
                    # One app main and 102/115 upstream mesh implementation units.
                    minimum = 102 if role == "Client" else 115
                    mesh = [
                        uri for uri in paths
                        if "/external/nRF5_SDK_Mesh/" in uri
                    ]
                    self.assertGreaterEqual(len(set(mesh)), minimum)
                    self.assertEqual(len(set(paths)), len(paths))
                    self.assertIn("src/main.c", [name for name, _ in links])
                    self.assertFalse(any(
                        uri.endswith("/examples/light_switch/" + role.lower() + "/src/main.c")
                        for uri in paths
                    ))
                    for required in (
                        "/mesh/core/src/nrf_mesh.c",
                        "/mesh/core/src/transport.c",
                        "/mesh/stack/src/mesh_stack.c",
                        "/mesh/access/src/access.c",
                        "/mesh/bearer/src/bearer_handler.c",
                    ):
                        self.assertTrue(any(uri.endswith(required) for uri in mesh), required)
                    sdk = [
                        uri for uri in paths
                        if "/external/nRF5_SDK/" in uri
                    ]
                    self.assertGreaterEqual(len(set(sdk)), 12)
                    self.assertTrue(any(
                        uri.endswith("/components/softdevice/common/nrf_sdh_ble.c")
                        for uri in sdk
                    ))
                    self.assertFalse(any(uri.endswith("/system_nrf52.c") for uri in sdk))
                    self.assertFalse(any(uri.endswith("/system_nrf52840.c") for uri in sdk))

    def test_softdevice_api_and_mesh_compiler_configuration(self):
        for part in ("nRF52832", "nRF52840"):
            for role in ("Client", "Server"):
                with self.subTest(part=part, role=role):
                    path = NRF / part / "exemples" / ("MeshLgtSw" + role) / "ioc" / ".cproject"
                    root = ET.parse(path).getroot()
                    configs = root.findall(".//cconfiguration")
                    self.assertEqual(len(configs), 2)
                    for config in configs:
                        macros = [
                            item.get("value", "")
                            for opt in config.findall(".//option")
                            if opt.get("valueType") == "definedSymbols"
                            for item in opt.findall("listOptionValue")
                        ]
                        for required in (
                            "NRF_SD_BLE_API_VERSION=7",
                            "SOFTDEVICE_PRESENT",
                            "NO_VTOR_CONFIG",
                            "USE_APP_CONFIG",
                            "CONFIG_APP_IN_CORE",
                        ):
                            self.assertIn(required, macros)
                        self.assertNotIn("NRF_SD_BLE_API_VERSION=6", macros)
                        includes = [
                            item.get("value", "")
                            for opt in config.findall(".//option")
                            if opt.get("valueType") == "includePath"
                            for item in opt.findall("listOptionValue")
                        ]
                        for required in (
                            "external/nRF5_SDK_Mesh/examples/light_switch/" + role.lower() + "/include",
                            "external/nRF5_SDK_Mesh/mesh/core/include",
                            "external/nRF5_SDK_Mesh/mesh/access/include",
                            "external/nRF5_SDK_Mesh/mesh/dfu/include",
                            "external/nRF5_SDK_Mesh/external/micro-ecc",
                        ):
                            self.assertTrue(any(required in value for value in includes), required)


if __name__ == "__main__":
    unittest.main()
