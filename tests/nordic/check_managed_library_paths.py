#!/usr/bin/env python3
"""Validate nRF53/nRF54 Eclipse managed example library link configurations."""

import pathlib
import re
import unittest

ROOT = pathlib.Path(__file__).resolve().parents[2]

TARGETS = (
    "ARM/Nordic/nRF53/nRF5340_App",
    "ARM/Nordic/nRF53/nRF5340_Net",
    "ARM/Nordic/nRF54/nRF54L15",
    "ARM/Nordic/nRF54/nRF54LM20A",
    "ARM/Nordic/nRF54/nRF54LM20B",
)

CONFIG = re.compile(
    r'<storageModule buildSystemId="org.eclipse.cdt.managedbuilder.'
    r'core.configurationDataProvider"[^>]*name="([^"]+)"'
)
LIB_PATH = re.compile(r'lib/ioc/(Debug(?:_SDC)?|Release(?:_SDC)?)')


class ManagedLibraryPaths(unittest.TestCase):
    def test_example_configs_link_matching_library(self):
        for relative in TARGETS:
            target = ROOT / relative
            self.assertTrue(target.is_dir(), str(target))
            projects = sorted(target.glob("exemples/*/ioc/.cproject"))
            self.assertGreater(len(projects), 0, str(target))
            expected_library = "IOsonata_" + target.name

            for project in projects:
                source = project.read_text(encoding="utf-8")
                configs = list(CONFIG.finditer(source))
                self.assertGreater(len(configs), 0, str(project))
                for index, match in enumerate(configs):
                    config = match.group(1)
                    end = (configs[index + 1].start()
                           if index + 1 < len(configs) else len(source))
                    block = source[match.start():end]
                    libraries = LIB_PATH.findall(block)
                    with self.subTest(project=str(project.relative_to(ROOT)),
                                      config=config):
                        self.assertIn(expected_library, block)
                        self.assertGreater(len(libraries), 0)
                        expected = ("Release" if config.startswith("Release")
                                    else "Debug")
                        for linked in libraries:
                            self.assertTrue(
                                linked.startswith(expected),
                                "Configuration %s links %s library in %s" %
                                (config, linked, project)
                            )


if __name__ == "__main__":
    unittest.main()
