#!/usr/bin/env python3
"""Audit Nordic Eclipse managed example library configuration linkage.

Check every Nordic .cproject example, not a manually maintained target list.
Debug variants must use Debug libraries and Release variants Release libraries.
Vendor SDK/NCS builds are outside the Eclipse managed project scope.
"""

import pathlib
import re
import unittest

ROOT = pathlib.Path(__file__).resolve().parents[2]
NORDIC = ROOT / "ARM/Nordic"
CONFIG = re.compile(
    r'<storageModule buildSystemId="org.eclipse.cdt.managedbuilder.'
    r'core.configurationDataProvider"[^>]*name="([^"]+)"'
)
LIB_PATH = re.compile(r'lib/ioc/(Debug\w*|Release\w*)')


class ManagedLibraryPaths(unittest.TestCase):
    def test_nordic_managed_examples_link_matching_library(self):
        projects = sorted(NORDIC.glob("**/exemples/*/ioc/.cproject"))
        self.assertGreater(len(projects), 300)
        checked = 0
        for project in projects:
            relative = project.relative_to(ROOT)
            target = project.parents[3].name
            expected_library = "IOsonata_" + target
            source = project.read_text(encoding="utf-8")
            configurations = list(CONFIG.finditer(source))
            with self.subTest(project=str(relative)):
                self.assertGreater(len(configurations), 0)
                self.assertTrue(project.with_name(".project").is_file())
            for index, match in enumerate(configurations):
                config = match.group(1)
                end = (configurations[index + 1].start()
                       if index + 1 < len(configurations) else len(source))
                block = source[match.start():end]
                paths = LIB_PATH.findall(block)
                with self.subTest(project=str(relative), config=config):
                    self.assertRegex(config, r'^(Debug|Release)')
                    self.assertIn(expected_library, block)
                    self.assertGreater(
                        len(paths), 0,
                        "Missing managed library search directory in %s:%s" %
                        (relative, config)
                    )
                    expected = ("Release" if config.startswith("Release")
                                else "Debug")
                    for linked in paths:
                        self.assertTrue(
                            linked.startswith(expected),
                            "Configuration %s links %s in %s" %
                            (config, linked, relative)
                        )
                checked += 1
        print("Nordic managed example configurations checked:", checked)

    def test_nordic_managed_libraries_have_eclipse_metadata(self):
        libraries = sorted(NORDIC.glob("**/lib/ioc/.cproject"))
        self.assertGreater(len(libraries), 10)
        for path in libraries:
            with self.subTest(project=str(path.relative_to(ROOT))):
                self.assertTrue(path.with_name(".project").is_file())


if __name__ == "__main__":
    unittest.main()
