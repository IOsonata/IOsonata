#!/usr/bin/env python3
"""Source contracts for Nordic HFXO ownership and request accounting."""

from pathlib import Path
import unittest

ROOT = Path(__file__).resolve().parents[2]
CLOCK = ROOT / "ARM/Nordic/src/system_clock_nrfx.cpp"
MPSL = ROOT / "ARM/Nordic/src/nrf_mpsl.cpp"


def function(source, name, next_name):
    begin = source.index(name)
    end = source.index(next_name, begin)
    return source[begin:end]


class HfxoOwnershipTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.clock = CLOCK.read_text(encoding="utf-8")
        cls.mpsl = MPSL.read_text(encoding="utf-8")

    def test_public_api_uses_first_acquire_last_release(self):
        source = function(self.clock, "bool SystemCoreClockSelect(",
                          "// Starts LFCLK from Src")
        self.assertIn("s_nRFxHfxoReqCnt != 0U", source)
        self.assertIn("s_nRFxHfxoReqCnt++", source)
        self.assertIn("s_nRFxHfxoReqCnt > 1U", source)
        self.assertIn("s_nRFxHfxoReqCnt--", source)
        self.assertIn("s_bNRFxHfxoTransition = true", source)
        self.assertIn("s_bNRFxHfxoTransition = false", source)
        self.assertIn("success = nRFxHfxoStart(&owner)", source)
        self.assertIn("success = nRFxHfxoStop(owner)", source)
        self.assertIn("if (ClkSrc == OSC_TYPE_RC && success)", source)
        self.assertIn("else if (success)", source)
        self.assertNotIn("std::atomic", source)

    def test_release_uses_original_backend(self):
        source = function(self.clock, "static bool nRFxHfxoStop(",
                          "bool SystemCoreClockSelect(")
        self.assertIn("nRFxMpslHfxo(false) == 0", source)
        self.assertIn("sd_clock_hfclk_release() == NRF_SUCCESS", source)
        self.assertIn("if (nRFxMpslActive() || nRFxClockSdEnabled())", source)
        guarded = source.split(
            "if (nRFxMpslActive() || nRFxClockSdEnabled())", 1)[1]
        self.assertLess(guarded.index("return false;"),
                        guarded.index("NRFX_HFXO_STOP = 1U"))
        self.assertIn("NRFX_HFXO_OWNER_NONE", self.clock)

    def test_failed_start_releases_backend(self):
        source = function(self.clock, "static bool nRFxHfxoStart(",
                          "// Release only through the backend")
        self.assertIn("sd_clock_hfclk_request()", source)
        self.assertGreaterEqual(source.count("sd_clock_hfclk_release()"), 2)
        self.assertIn("NRFX_HFXO_OWNER_SOFTDEVICE", source)
        self.assertIn("NRFX_HFXO_OWNER_MPSL", source)

        mpsl = function(self.mpsl, "int nRFxMpslHfxo(", 'extern "C"')
        self.assertIn("mpsl_clock_hfclk_src_request(", mpsl)
        self.assertGreaterEqual(
            mpsl.count("mpsl_clock_hfclk_src_release("), 3)


if __name__ == "__main__":
    unittest.main()
