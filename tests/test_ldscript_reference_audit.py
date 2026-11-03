#!/usr/bin/env python3
"""Offline regression tests for Eclipse linker-script reference validation."""

import importlib.util
from pathlib import Path
import tempfile
import unittest


AUDIT = Path(__file__).with_name("ldscript_reference_audit.py")
SPEC = importlib.util.spec_from_file_location("ldscript_reference_audit", AUDIT)
audit = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(audit)


class LinkerReferenceAuditTests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.addCleanup(self.temp.cleanup)
        self.root = Path(self.temp.name)
        self.project = self.root / "ARM" / "Vendor" / "Family" / "Part"
        self.ioc = self.project / "exemples" / "Blinky" / "ioc"
        self.ioc.mkdir(parents=True)
        self.linkers = self.project.parent / "ldscript"
        self.linkers.mkdir()
        self.script = self.linkers / "gcc_part.ld"
        self.script.write_text("SECTIONS { .text : { *(.text*) } }\n", encoding="utf-8")
        self.reference = "../../../../ldscript/gcc_part.ld"

    def write_project(self, reference=None):
        if reference is None:
            reference = self.reference
        path = self.ioc / ".cproject"
        path.write_text(
            '<?xml version="1.0" encoding="UTF-8"?>\n'
            '<cproject><cconfiguration>'
            '<storageModule><configuration name="Debug">'
            '<toolChain><tool><option superClass="cross.linker.scriptfile" '
            'valueType="stringList"><listOptionValue builtIn="false" '
            'value="' + reference + '"/></option></tool></toolChain>'
            '</configuration></storageModule>'
            '</cconfiguration></cproject>\n',
            encoding="utf-8",
        )
        return path

    def test_existing_script_from_generated_build_directory(self):
        path = self.write_project()
        checked, skipped, errors = audit.check_project(path)
        self.assertEqual((checked, skipped, errors), (1, 0, []))

    def test_missing_script_is_reported(self):
        path = self.write_project("../../../../ldscript/not_found.ld")
        checked, skipped, errors = audit.check_project(path)
        self.assertEqual((checked, skipped), (1, 0))
        self.assertEqual(len(errors), 1)
        self.assertIn("not_found.ld", errors[0])

    def test_dynamic_script_is_not_assumed_missing(self):
        path = self.write_project("${LINKER_SCRIPT}")
        checked, skipped, errors = audit.check_project(path)
        self.assertEqual((checked, skipped, errors), (0, 1, []))

    def test_malformed_xml_is_reported(self):
        path = self.ioc / ".cproject"
        path.write_text("<cproject>", encoding="utf-8")
        checked, skipped, errors = audit.check_project(path)
        self.assertEqual((checked, skipped), (0, 0))
        self.assertIn("invalid XML", errors[0])


if __name__ == "__main__":
    unittest.main()
