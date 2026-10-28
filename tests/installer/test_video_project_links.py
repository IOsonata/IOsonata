#!/usr/bin/env python3
"""Verify video device links in precompiled IOsonata library projects."""
from pathlib import Path
import re
import unittest
import xml.etree.ElementTree as ET

ROOT = Path(__file__).resolve().parents[2]
VIDEO_FILES = (
    "include/video/videodev.h",
    "include/video/cam_arducam_mega.h",
    "src/video/cam_arducam_mega.cpp",
)


class VideoLibraryProjects(unittest.TestCase):
    def test_all_managed_libraries_link_video_category(self):
        projects = sorted(ROOT.glob("**/lib/ioc/.project"))
        projects.append(ROOT / "Linux/lib/.project")
        self.assertGreaterEqual(len(projects), 38)
        for project in projects:
            with self.subTest(project=str(project.relative_to(ROOT))):
                tree = ET.parse(project)
                links = {}
                for link in tree.findall(".//linkedResources/link"):
                    name = link.findtext("name")
                    if name in VIDEO_FILES:
                        links.setdefault(name, []).append(link.findtext("locationURI"))
                for name in VIDEO_FILES:
                    self.assertEqual(len(links.get(name, [])), 1, name)
                    uri = links[name][0]
                    match = re.fullmatch(r"PARENT-(\d+)-PROJECT_LOC/(.+)", uri)
                    self.assertIsNotNone(match, uri)
                    level = int(match.group(1))
                    target = project.parent
                    for _ in range(level):
                        target = target.parent
                    self.assertEqual((target / match.group(2)).resolve(),
                                     (ROOT / name).resolve())
                    self.assertTrue((target / match.group(2)).is_file())

    def test_camera_source_enabled_in_managed_configurations(self):
        projects = sorted(ROOT.glob("**/lib/ioc/.cproject"))
        self.assertGreaterEqual(len(projects), 37)
        for project in projects:
            with self.subTest(project=str(project.relative_to(ROOT))):
                tree = ET.parse(project)
                configs = tree.findall(".//cconfiguration")
                self.assertGreaterEqual(len(configs), 2)
                for config in configs:
                    entries = config.findall(".//sourceEntries/entry")
                    self.assertTrue(entries, "Missing source entries")
                    for entry in entries:
                        exclusions = entry.get("excluding", "").split("|")
                        for excluded in exclusions:
                            self.assertFalse(
                                excluded == "src/video" or
                                excluded == "src/video/cam_arducam_mega.cpp" or
                                (excluded and "src/video/cam_arducam_mega.cpp".startswith(excluded + "/")),
                                "Video driver excluded from build: " + excluded)

    def test_windows_visual_studio_project(self):
        project = ROOT / "Win/lib/IOsonata_Win.vcxproj"
        tree = ET.parse(project)
        paths = [node.get("Include", "").replace("\\", "/")
                 for node in tree.iter()]
        for name in VIDEO_FILES:
            self.assertEqual(sum(path.endswith(name) for path in paths), 1, name)

    def test_macos_xcode_project(self):
        project = ROOT / "OSX/lib/IOsonata_OSX.xcodeproj/project.pbxproj"
        data = project.read_text()
        for name in VIDEO_FILES:
            self.assertIn("../../" + name, data)
        self.assertIn("cam_arducam_mega.cpp in Sources", data)
        self.assertIn("videodev.h in Headers", data)
        self.assertIn("cam_arducam_mega.h in Headers", data)


if __name__ == "__main__":
    unittest.main(verbosity=2)
