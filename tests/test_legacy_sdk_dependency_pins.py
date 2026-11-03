#!/usr/bin/env python3
"""Check exact SDK versions isolated for legacy Nordic reference examples."""
from pathlib import Path
import unittest
import xml.etree.ElementTree as ET

ROOT = Path(__file__).resolve().parents[1]
NRF52 = ROOT / "ARM" / "Nordic" / "nRF52"
LM20 = ROOT / "ARM" / "Nordic" / "nRF54" / "nRF54LM20x"
LVGL_REV = "69aa421acd78e0d3809200b9bc1f21c8586f5cf0"
TINYUSB_REV = "064afa3774cbbb2093d6af9805bdaa15b4e0f751"


class LegacySdkDependencyTests(unittest.TestCase):
    def test_lvgl_projects_use_isolated_legacy_checkout(self):
        for target in ("nRF52832", "nRF52840"):
            for example in ("LvglDemo", "LvglDualDIsplayDemo"):
                with self.subTest(target=target, example=example):
                    project = NRF52 / target / "exemples" / example / "ioc"
                    links = [
                        link.findtext("locationURI", "")
                        for link in ET.parse(project / ".project").findall(".//linkedResources/link")
                    ]
                    self.assertGreater(sum("external/lvgl_legacy/" in uri for uri in links), 300)
                    self.assertFalse(any("external/lvgl/" in uri for uri in links))
                    settings = (project / ".cproject").read_text()
                    self.assertIn("external/lvgl_legacy/", settings)
                    self.assertNotIn("external/lvgl/", settings)

    def test_tinyusb_lm20_uses_isolated_iso_revision(self):
        project = LM20 / "exemples" / "TinyUsbComboStress" / "ioc"
        links = [
            link.findtext("locationURI", "")
            for link in ET.parse(project / ".project").findall(".//linkedResources/link")
        ]
        self.assertGreaterEqual(sum("external/tinyusb_lm20/" in uri for uri in links), 6)
        self.assertFalse(any("external/tinyusb/" in uri for uri in links))
        settings = (project / ".cproject").read_text()
        self.assertIn("external/tinyusb_lm20/", settings)
        self.assertNotIn("external/tinyusb/", settings)

    def test_installer_clones_both_exact_commits(self):
        for system, suffix in (("linux", ".sh"), ("macos", ".sh"), ("win", ".ps1")):
            with self.subTest(system=system):
                installer = (ROOT / "Installer" / ("clone_iosonata_sdk_" + system + suffix)).read_text()
                self.assertIn("lvgl_legacy", installer)
                self.assertIn("tinyusb_lm20", installer)
                self.assertIn(LVGL_REV, installer)
                self.assertIn(TINYUSB_REV, installer)
                self.assertIn("https://github.com/lvgl/lvgl.git", installer)
                self.assertIn("https://github.com/hathach/tinyusb.git", installer)
                if system == "win":
                    self.assertIn("function Sync-PinnedCommitRepository", installer)
                    self.assertIn("fetch -q --depth=1 origin $Commit", installer)
                    self.assertIn("Pinned checkout preserved:", installer)
                else:
                    self.assertIn("clone_pinned_commit_repo()", installer)
                    self.assertIn('fetch -q --depth=1 origin "$revision"', installer)
                    self.assertIn("Pinned checkout preserved:", installer)


if __name__ == "__main__":
    unittest.main()
