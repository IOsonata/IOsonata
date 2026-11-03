#!/usr/bin/env python3
"""Validate IOsonata MCU wizard descriptors against checked-in project files."""

import json
from pathlib import Path
import unittest

ROOT = Path(__file__).resolve().parents[2]

# Source ports that do not yet provide these linker variants.
KNOWN_INCOMPLETE = {
    "ARM/RPI/RP20/RP2040/lib/ioc/wizard.json": {"base"},
}


class WizardDescriptorTests(unittest.TestCase):
    def test_descriptor_paths(self):
        descriptors = sorted(ROOT.glob("ARM/**/lib/ioc/wizard.json"))
        descriptors += sorted(ROOT.glob("RISCV/**/lib/ioc/wizard.json"))
        self.assertTrue(descriptors, "No IOsonata wizard descriptors found")

        for descriptor in descriptors:
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
                    if name in KNOWN_INCOMPLETE.get(relative, set()):
                        continue
                    self.assertTrue(reference.startswith("${iosonata}/"),
                                    f"Unexpected linker path token: {reference}")
                    linker = ROOT / reference.removeprefix("${iosonata}/")
                    self.assertTrue(linker.is_file(),
                                    f"Missing {name} linker script: {linker}")

    def test_known_incomplete_targets_are_explicit(self):
        for descriptor, features in KNOWN_INCOMPLETE.items():
            self.assertTrue((ROOT / descriptor).is_file(), descriptor)
            self.assertTrue(features)


if __name__ == "__main__":
    unittest.main(verbosity=2)
