#!/usr/bin/env python3
"""Guard E-Ink example source and driver initialization ordering."""
from pathlib import Path
import unittest

ROOT = Path(__file__).resolve().parents[1]


class EinkExampleTests(unittest.TestCase):
    def test_display_demo_uses_existing_interface(self):
        source = (ROOT / "ARM" / "Nordic" / "nRF52" / "nRF52840"
                  / "exemples" / "EInkDisplayDemo" / "src"
                  / "eink_display_demo.cpp").read_text()
        self.assertIn('#include "display/eink_intrf.h"', source)
        self.assertNotIn('#include "display/eink_vb3300.h"', source)
        self.assertIn(".pSpiDev = nullptr", source)

    def test_init_sets_pin_map_before_using_it(self):
        source = (ROOT / "src" / "display" / "eink_intrf.cpp").read_text()
        start = source.index("bool EInkIntrfInit(")
        end = source.index("bool EInkIntrf::Init(", start)
        init = source[start:end]
        self.assertLess(init.index("pDev->pIOPinMap = pCfgData->pIOPinMap"),
                        init.index("IOPinSet(pDev->pIOPinMap"))
        self.assertLess(init.index("NbIOPins <= EIINTRF_CS_PIN_IDX"),
                        init.index("IOPinCfg(pDev->pIOPinMap"))
        self.assertIn("pCfgData->pIOPinMap == NULL", init)

    def test_legacy_paper_nordic_macros(self):
        example = ROOT / "ARM" / "Nordic" / "nRF52" / "nRF52840" / "exemples" / "EInkDisplayDemo" / "src"
        paper = (example / "paper.c").read_text()
        controller = (example / "ED037TC1.c").read_text()
        self.assertIn('#include "app_util_platform.h"', paper)
        self.assertIn('#include "nordic_common.h"', paper)
        self.assertIn("void Upload_Temperature_LUT(void);", paper)
        self.assertIn("void InitialDisplay(void);", controller)
        self.assertIn("void Upload_Temperature_LUT(void);", controller)
        self.assertIn("void Upload_Temperature_LUT(void)", controller)

    def test_c_implementation_matches_public_write_signature(self):
        header = (ROOT / "include" / "display" / "eink_intrf.h").read_text()
        source = (ROOT / "src" / "display" / "eink_intrf.cpp").read_text()
        signature = ("int EInkIntrfWrite(EInkIntrfDev_t * const pDev, "
                     "uint8_t *pCmd, int CmdLen, const uint8_t *pData, int DataLen)")
        self.assertIn(signature, header)
        self.assertIn(signature, source)


if __name__ == "__main__":
    unittest.main()
