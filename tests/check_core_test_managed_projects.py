#!/usr/bin/env python3
"""Verify generic CoreClockTest/CorePowerTest Eclipse project migration."""
from pathlib import Path
import unittest

ROOT = Path(__file__).resolve().parents[1]


class CoreTestProjects(unittest.TestCase):
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
                    self.assertIn("${workspace_loc:/" + name + "}", cproject)
                    self.assertNotIn("ClockDemo", project + cproject)
                    self.assertNotIn("PowerOffDemo", project + cproject)


if __name__ == "__main__":
    unittest.main()
