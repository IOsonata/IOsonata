#!/usr/bin/env python3
"""Validate IOsonata MCU wizard descriptors against checked-in project files."""

import json
from pathlib import Path
import unittest
import xml.etree.ElementTree as ET

ROOT = Path(__file__).resolve().parents[2]

# These projects cannot yet create a standalone wizard application.
# Every other Arm/RISC-V library project must have a descriptor.
KNOWN_INCOMPLETE = {
    "ARM/RPI/RP20/RP2040/lib/ioc": "No application linker script",
    "ARM/NXP/LPC546xx/LPC54605/lib/ioc":
        "Only DFU linker layouts; no standalone application layout",
    "ARM/Nordic/nRF54/nRF54H20/nRF54H20_App/lib/ioc":
        "No application linker script, UART or timer driver",
    "ARM/Nordic/nRF54/nRF54H20/nRF54H20_Net/lib/ioc":
        "No application linker script, UART or timer driver",
    "RISCV/Nordic/nRF54/nRF54H20/lib/ioc": "Unfinished RISC-V port",
}


def descriptors():
    return sorted(ROOT.glob("ARM/**/lib/ioc/wizard.json")) + sorted(
        ROOT.glob("RISCV/**/lib/ioc/wizard.json"))


class WizardDescriptorTests(unittest.TestCase):
    def test_descriptor_paths(self):
        self.assertTrue(descriptors(), "No IOsonata wizard descriptors found")

        for descriptor in descriptors():
            relative = descriptor.relative_to(ROOT).as_posix()
            with self.subTest(descriptor=relative):
                data = json.loads(descriptor.read_text(encoding="utf-8"))
                self.assertEqual(data.get("schema_version"), 1)
                self.assertTrue(data.get("mcu"))
                self.assertTrue(data.get("cpu"))
                self.assertTrue(data.get("lib"))
                self.assertTrue(data.get("linker_script"))

                library = data.get("lib_path", "").replace(
                    "${iosonata}/", "").replace("/${config}", "")
                self.assertTrue(library, "Missing lib_path")
                self.assertTrue((ROOT / library).is_dir(),
                                f"Missing library project: {library}")

                scripts = {"base": data["linker_script"]}
                scripts.update({
                    name: feature["linker_script"]
                    for name, feature in data.get("features", {}).items()
                    if feature.get("linker_script")
                })
                for name, reference in scripts.items():
                    self.assertTrue(reference.startswith("${iosonata}/"),
                                    f"Unexpected linker path token: {reference}")
                    linker = ROOT / reference.removeprefix("${iosonata}/")
                    self.assertTrue(linker.is_file(),
                                    f"Missing {name} linker script: {linker}")

    def test_library_project_coverage(self):
        projects = sorted(ROOT.glob("ARM/**/lib/ioc/.cproject")) + sorted(
            ROOT.glob("RISCV/**/lib/ioc/.cproject"))
        seen_exceptions = set()
        for project in projects:
            relative = project.parent.relative_to(ROOT).as_posix()
            with self.subTest(project=relative):
                descriptor = project.parent / "wizard.json"
                if relative in KNOWN_INCOMPLETE:
                    seen_exceptions.add(relative)
                    self.assertFalse(descriptor.exists(), KNOWN_INCOMPLETE[relative])
                else:
                    self.assertTrue(descriptor.is_file(),
                                    "Library is invisible to the wizard: " + relative)
        self.assertEqual(seen_exceptions, set(KNOWN_INCOMPLETE))

    def test_library_names(self):
        for descriptor in descriptors():
            with self.subTest(descriptor=descriptor.relative_to(ROOT)):
                data = json.loads(descriptor.read_text(encoding="utf-8"))
                self.assertEqual(data["lib_path"], "${iosonata}/" +
                                 descriptor.parent.relative_to(ROOT).as_posix() +
                                 "/${config}")
                project_name = ET.parse(descriptor.parent / ".project").findtext("name")
                project = ET.parse(descriptor.parent / ".cproject")
                configurations = {c.get("name"): c for c in project.findall(
                    ".//storageModule[@moduleId='cdtBuildSystem']/configuration")}
                self.assertTrue(configurations)
                selected = [config for name, config in configurations.items()
                            if name in ("Debug", "Release")]
                if not selected:
                    selected = list(configurations.values())
                for config in selected:
                    artifact = config.get("artifactName", "").replace(
                        "${ProjName}", project_name)
                    self.assertEqual(data["lib"], artifact)

    def test_local_includes_and_unique_targets(self):
        targets = set()
        for descriptor in descriptors():
            with self.subTest(descriptor=descriptor.relative_to(ROOT)):
                data = json.loads(descriptor.read_text(encoding="utf-8"))
                token = data["mcu"].upper()
                self.assertNotIn(token, targets)
                targets.add(token)
                includes = data.get("includes", [])[:]
                for feature in data.get("features", {}).values():
                    includes.extend(feature.get("extra_includes", []))
                for include in includes:
                    if include.startswith("${iosonata}/"):
                        self.assertTrue((ROOT / include.removeprefix(
                            "${iosonata}/")).is_dir(), include)

    def test_nordic_target_matches_library(self):
        for descriptor in descriptors():
            data = json.loads(descriptor.read_text(encoding="utf-8"))
            if not data["mcu"].startswith("NRF"):
                continue
            with self.subTest(descriptor=descriptor.relative_to(ROOT)):
                project = ET.parse(descriptor.parent / ".cproject")
                for config in project.findall(".//configuration"):
                    if config.get("name") not in ("Debug", "Release"):
                        continue
                    definitions = {v.get("value") for o in config.findall(".//option")
                                   if o.get("superClass", "").endswith(".compiler.defs")
                                   for v in o.findall("listOptionValue")}
                    self.assertIn(data["mcu"], definitions)

    def test_incomplete_rp2040_has_no_wizard_descriptor(self):
        self.assertFalse(
            (ROOT / "ARM/RPI/RP20/RP2040/lib/ioc/wizard.json").exists(),
            "RP2040 wizard must not advertise an unimplemented linker layout"
        )


if __name__ == "__main__":
    unittest.main(verbosity=2)
