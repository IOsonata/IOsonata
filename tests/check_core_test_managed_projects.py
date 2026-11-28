#!/usr/bin/env python3
"""Verify generic CoreClockTest/CorePowerTest Eclipse project migration."""
from pathlib import Path
import unittest
import xml.etree.ElementTree as ET

ROOT = Path(__file__).resolve().parents[1]


class CoreTestProjects(unittest.TestCase):
    def check_build_paths(self, cproject, project_name):
        configs = ET.fromstring(cproject).findall(".//storageModule[@moduleId='cdtBuildSystem']/configuration")
        self.assertTrue(configs, project_name)
        for config in configs:
            name = config.attrib["name"]
            builders = config.findall(".//builder")
            self.assertTrue(builders, (project_name, name))
            for builder in builders:
                self.assertIn(builder.get("buildPath"), (
                    "${ProjDirPath}/" + name,
                    "${workspace_loc:/" + project_name + "}/" + name,
                ), (project_name, name))

    def test_legacy_managed_projects_removed(self):
        for old in ("ClockDemo", "PowerOffDemo"):
            found = list(ROOT.glob("ARM/**/exemples/" + old + "/ioc/.project"))
            found += list(ROOT.glob("RISCV/**/exemples/" + old + "/ioc/.project"))
            self.assertFalse(found, "Legacy projects: " + repr(found))

    def test_generic_managed_projects_are_paired_and_linked(self):
        roots = []
        for arch in ("ARM", "RISCV"):
            roots += list((ROOT / arch).glob("**/exemples/CoreClockTest/ioc/.project"))
        self.assertGreaterEqual(len(roots), 31)
        for clock_project in roots:
            examples = clock_project.parents[2]
            with self.subTest(examples=examples):
                power_project = examples / "CorePowerTest/ioc/.project"
                self.assertTrue(power_project.is_file())
                for name, source in (("CoreClockTest", "core_clock_test.cpp"),
                                     ("CorePowerTest", "core_power_test.cpp")):
                    folder = examples / name / "ioc"
                    project = (folder / ".project").read_text(encoding="utf-8")
                    cproject = (folder / ".cproject").read_text(encoding="utf-8")
                    self.assertIn("<name>" + name + "</name>", project)
                    self.assertIn("exemples/misc/" + source, project)
                    self.assertIn("src/" + source, project)
                    self.check_build_paths(cproject, name)
                    self.assertNotIn("ClockDemo", project + cproject)
                    self.assertNotIn("PowerOffDemo", project + cproject)

    def test_timer_test_projects_link_the_shared_test(self):
        roots = []
        for arch in ("ARM", "RISCV"):
            roots += list((ROOT / arch).glob("**/exemples/TimerTest/ioc/.project"))
        self.assertGreaterEqual(len(roots), 13)
        for project_file in roots:
            folder = project_file.parent
            with self.subTest(project=folder):
                project = project_file.read_text(encoding="utf-8")
                cproject = (folder / ".cproject").read_text(encoding="utf-8")
                board = (folder.parent / "src/board.h").read_text(encoding="utf-8")
                self.assertIn("<name>TimerTest</name>", project)
                self.assertIn("exemples/misc/timer_test.cpp", project)
                self.assertIn("src/timer_test.cpp", project)
                self.check_build_paths(cproject, "TimerTest")
                self.assertNotIn("CoreClockTest", project + cproject + board)
                self.assertNotIn("core_clock_test", project + cproject)
                self.assertNotIn("CORE_CLOCK_TEST", board)
                self.assertIn("#define UART_DEVNO", board)


if __name__ == "__main__":
    unittest.main()
