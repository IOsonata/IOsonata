#!/usr/bin/env python3
"""Regression tests for the GNU ld non-PEI link-isolation checker."""

import contextlib
import io
import pathlib
import tempfile
import unittest

from check_no_pei_link import check


class NoPeiLinkTests(unittest.TestCase):
    def check_map(self, archive="", discarded="", linked=""):
        data = (
            "Archive member included to satisfy reference by file (symbol)\n"
            + archive + "\n"
            + "Discarded input sections\n" + discarded + "\n"
            + "Linker script and memory map\n" + linked + "\n"
        )
        with tempfile.TemporaryDirectory(prefix="pei-map-check-") as directory:
            path = pathlib.Path(directory) / "firmware.map"
            path.write_text(data, encoding="utf-8")
            with contextlib.redirect_stdout(io.StringIO()):
                return check(path)

    def test_clean_timer_map(self):
        self.assertEqual(self.check_map(
            archive="libIOsonata.a(timer_hf_nrfx.o)"), 0)

    def test_plain_matrix_archive_member(self):
        self.assertEqual(self.check_map(
            archive="libIOsonata_nRF52832.a(pei.o)"), 1)

    def test_plain_backend_archive_member(self):
        self.assertEqual(self.check_map(
            archive="libIOsonata_nRF52832.a(pei_nrf52.o)"), 1)

    def test_path_qualified_archive_member(self):
        self.assertEqual(self.check_map(
            archive="libIOsonata.a(ARM/Nordic/nRF52/src/pei_nrf52.cpp.o)"), 1)

    def test_discarded_pei_sections_are_not_linked(self):
        self.assertEqual(self.check_map(discarded=(
            " .text.PeiConnect 0x00000000\n"
            " .bss.s_nRfxRtc0Pei 0x00000000"
        )), 0)

    def test_linked_pei_code_is_rejected(self):
        self.assertEqual(self.check_map(
            linked=" .text.PeiConnect 0x00001000 0x34"), 1)

    def test_linked_gpio_port_pei_source_is_rejected(self):
        self.assertEqual(self.check_map(
            linked=" .text.IOPinEnablePortEvent 0x00002000 0x20"), 1)

    def test_discarded_gpio_port_pei_source_is_allowed(self):
        self.assertEqual(self.check_map(
            discarded=" .text.IOPinDisablePortEvent 0x00000000 0x20"), 0)

    def test_linked_adc_pei_code_is_rejected(self):
        self.assertEqual(self.check_map(
            linked=" .text.AdcNrf52PeiConnectTask 0x00002000 0x30"), 1)

    def test_discarded_adc_pei_code_is_allowed(self):
        self.assertEqual(self.check_map(
            discarded=" .text.AdcNrf52PeiBindEvent 0x00000000 0x20"), 0)

    def test_linked_comparator_pei_code_is_rejected(self):
        self.assertEqual(self.check_map(
            linked=" .text.AnalogCompPeiConnectTask 0x00002000 0x30"), 1)

    def test_discarded_comparator_pei_code_is_allowed(self):
        self.assertEqual(self.check_map(
            discarded=" .text.AnalogCompPeiBindEvent 0x00000000 0x20"), 0)

    def test_linked_rtc_pei_state_is_rejected(self):
        self.assertEqual(self.check_map(
            linked=" .bss.s_nRfxRtcRaw 0x20000000 0x50"), 1)


if __name__ == "__main__":
    unittest.main()
