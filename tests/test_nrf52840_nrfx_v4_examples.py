#!/usr/bin/env python3
"""Track nRF52840 Nordic SDK comparison projects using the current nrfx API."""
from pathlib import Path
import unittest
import xml.etree.ElementTree as ET

ROOT = Path(__file__).resolve().parents[1]
DEMOS = ROOT / "ARM" / "Nordic" / "nRF52" / "nRF52840" / "exemples"


class Nrf52840NrfxV4Tests(unittest.TestCase):
    def test_qspi_driver_api_and_pins(self):
        demo = DEMOS / "QSpiTest"
        main = (demo / "src" / "main.c").read_text()
        self.assertIn("nrfx_qspi_config_t config = NRFX_QSPI_DEFAULT_CONFIG(", main)
        self.assertIn("BSP_QSPI_SCK_PIN", main)
        self.assertIn("nrfx_qspi_init(&config", main)
        self.assertIn("nrfx_qspi_erase(", main)
        self.assertNotIn("nrf_drv_qspi_", main)
        self.assertNotIn("APP_ERROR_CHECK(", main)
        project = ET.parse(demo / "ioc" / ".project")
        names = [x.findtext("name", "") for x in project.findall(".//linkedResources/link")]
        self.assertIn("src/modules/nrfx/drivers/src/nrfx_qspi.c", names)
        settings = (demo / "ioc" / ".cproject").read_text()
        self.assertIn("external/nrfx/bsp/stable/mdk/common", settings)
        self.assertNotIn("external/nrfx/mdk", settings)
        self.assertEqual(
            settings.count('external/nrfx/bsp/stable&quot;'), 4
        )
        self.assertIn('#include "nrf_gpio.h"', (demo / "src" / "board.h").read_text())

    def test_radio_timer_nrfx_v4_instance(self):
        demo = DEMOS / "radio_test"
        source = (demo / "src" / "radio_test.c").read_text()
        self.assertIn("static nrfx_timer_t m_timer = NRFX_TIMER_INSTANCE(NRF_TIMER0);", source)
        self.assertIn(".frequency = 1000000UL", source)
        self.assertIn("if (err != 0)", source)
        self.assertNotIn("NRFX_TIMER_INSTANCE(0)", source)
        project = ET.parse(demo / "ioc" / ".project")
        names = [x.findtext("name", "") for x in project.findall(".//linkedResources/link")]
        self.assertIn("src/nrfx/drivers/src/nrfx_timer.c", names)


if __name__ == "__main__":
    unittest.main()
