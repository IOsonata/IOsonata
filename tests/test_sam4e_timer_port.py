#!/usr/bin/env python3
"""SAM4E timer port source/project static checks."""
from pathlib import Path
import unittest
import xml.etree.ElementTree as ET

ROOT = Path(__file__).resolve().parents[1]
PORT = ROOT / "ARM" / "Microchip" / "SAM4E"


class Sam4eTimerPortTests(unittest.TestCase):
    def test_lib_project_links_real_timer(self):
        project = ET.parse(PORT / "SAM4E16E" / "lib" / "ioc" / ".project")
        link = [
            element for element in project.findall(".//linkedResources/link")
            if element.findtext("name") == "src/coredev/timer_sam4e.cpp"
        ]
        self.assertEqual(len(link), 1)
        self.assertEqual(link[0].findtext("locationURI"),
                         "PARENT-3-PROJECT_LOC/src/timer_sam4e.cpp")
        self.assertTrue((PORT / "src" / "timer_sam4e.cpp").is_file())

    def test_all_nine_channels_and_required_api(self):
        source = (PORT / "src" / "timer_sam4e.cpp").read_text()
        self.assertIn("SAM4E_TIMER_COUNT\t\t9", source)
        self.assertIn("SAM4E_TC2", source)
        self.assertIn("bool TimerInit(", source)
        self.assertIn("int TimerGetHighFreqDevCount(void)", source)
        self.assertIn("Sam4eSampleCount(", source)
        self.assertIn("TC_IER_COVFS", source)
        self.assertIn("TC_SR_COVFS", source)
        self.assertIn("TC_IER_CPAS", source)
        self.assertIn("TC_IER_CPBS", source)
        self.assertIn("TC_IER_CPCS", source)
        self.assertIn("Sam4eCheckPassed(", source)
        self.assertIn("TC_CMR_WAVSEL_UP", source)
        self.assertIn("return false;", source)
        for i in range(9):
            self.assertIn(f'extern "C" void TC{i}_Handler(void)', source)

    def test_missing_or_unsupported_features_are_not_fake_success(self):
        source = (PORT / "src" / "timer_sam4e.cpp").read_text()
        self.assertIn("pCfg->bTickInt", source)
        self.assertIn("Sam4eEnableExtTrigger(", source)
        self.assertIn("return false;", source)
        self.assertNotIn("new Sam4e", source)
        self.assertNotIn("malloc(", source)


if __name__ == "__main__":
    unittest.main()
