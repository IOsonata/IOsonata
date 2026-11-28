#!/usr/bin/env python3
"""Source checks for Nordic delay calibration and CPU frequency refresh."""
from pathlib import Path
import unittest

ROOT = Path(__file__).resolve().parents[2]
NRF = (
	"ARM/Nordic/nRF52/src/system_nrf52.c",
	"ARM/Nordic/nRF53/nRF5340_App/lib/src/system_nrf5340_application.c",
	"ARM/Nordic/nRF53/nRF5340_Net/lib/src/system_nrf5340_network.c",
	"ARM/Nordic/nRF54/src/system_nrf54l.c",
	"ARM/Nordic/nRF54/src/system_nrf54h.c",
	"ARM/Nordic/nRF91/src/system_nrf91.c",
)


class NordicDelayArchitecture(unittest.TestCase):
	def test_all_system_clock_updates_have_default_factors(self):
		for path in NRF:
			with self.subTest(path=path):
				source = (ROOT / path).read_text(encoding="utf-8")
				self.assertIn('#include "idelay.h"', source)
				self.assertIn("void SystemCoreClockUpdate(void)", source)
				self.assertTrue(
					"ArmDelayClockSetDivisors(" in source or
					"CalcDelayFactor(" in source
				)
				# All production clock-update paths only update loop factors.
				# DWT calibration runs explicitly in the diagnostic.
				self.assertNotIn("ArmDelayCalibrate(", source)

	def test_nrf5340_frequency_switch_uses_measured_factors(self):
		app = (ROOT / NRF[1]).read_text(encoding="utf-8")
		self.assertIn("NRF_CLOCK_NS->HFCLKCTRL", app)
		self.assertIn("NRF_CLOCK_S->HFCLKCTRL", app)
		self.assertIn("SystemMicroSecLoopCnt = CalcDelayFactor(cpuHz,", app)
		self.assertIn("SystemnsDelayFactor = CalcDelayFactor(cpuHz,", app)
		self.assertIn("64000000UL, 2908624UL, 128000000UL, 6399488UL", app)
		self.assertIn("64000000UL, 21330560UL, 128000000UL, 42664106UL", app)
		net = (ROOT / NRF[2]).read_text(encoding="utf-8")
		self.assertIn("23003229UL, 3000329UL", net)
		self.assertIn("ArmDelayClockSetDivisors(__SYSTEM_CLOCK_DEFAULT,", net)
		self.assertNotIn("ArmDelayCalibrate(", net)

	def test_dwt_calibration_uses_only_cpu_local_hardware(self):
		delay = (ROOT / "ARM/include/system_core_clock_calibration.h").read_text(encoding="utf-8")
		test = (ROOT / "exemples/misc/core_clock_test.cpp").read_text(encoding="utf-8")
		self.assertIn("ArmDelayCalibrate(&measured, cpuHz)", delay)
		start = delay.index("static inline bool ArmDelayCalibrate(")
		end = delay.index("static inline bool DelayCalibrationMeasure(", start)
		cal = delay[start:end]
		for forbidden in ("TimerInit(", "TimerEnable(", "TimerGetMicrosecond(",
			"SystemHighFreqClockEnable(", "SystemLowFreqClockEnable("):
			self.assertNotIn(forbidden, cal)
		self.assertIn("DelayCalibrationMeasure(pSample)", test)
		self.assertIn("DelayCalibrationFromTimer(", test)
		self.assertNotIn("ArmDelayCalibrate(", test)
		self.assertNotIn("DWT->", test)
		self.assertNotIn("__arm__", test)
		self.assertNotIn("__thumb__", test)


if __name__ == "__main__":
	unittest.main()
