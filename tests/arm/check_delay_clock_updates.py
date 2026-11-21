#!/usr/bin/env python3
"""Check CPU-frequency delay factor updates without requiring DWT hardware."""
from pathlib import Path
import unittest

ROOT = Path(__file__).resolve().parents[2]
HELPER = ROOT / "ARM/include/idelay_calibration.h"
DELAY = ROOT / "ARM/include/idelay.h"

PORTS = (
	"ARM/ST/STM32G4xx/src/system_stm32g4xx.c",
	"ARM/ST/STM32L4xx/src/system_stm32l4xx.c",
	"ARM/ST/STM32L4xx/src/system_stm32l4plus.c",
	"ARM/ST/STM32F0xx/src/system_stm32f0xx.c",
	"ARM/ST/STM32F4xx/src/system_stm32f4xx.c",
	"ARM/ST/STM32H7xx/src/system_stm32h7xx.c",
	"ARM/ST/STM32L1xx/src/system_stm32l1xx.c",
	"ARM/ST/STM32WBAxx/src/system_stm32wbaxx.c",
	"ARM/Microchip/SAM4L/src/system_sam4l.c",
	"ARM/Microchip/SAM4E/src/system_sam4e.c",
	"ARM/Microchip/SAMD/src/system_samd.c",
	"ARM/NXP/LPC11xx/src/system_LPC11Uxx.c",
	"ARM/NXP/LPC17xx/src/system_LPC17xx.c",
	"ARM/NXP/LPC546xx/src/system_LPC546xx.c",
	"ARM/Renesas/RA4/RA4M1/src/system_ra4m1.c",
	"ARM/Renesas/RE01/src/system_re01.c",
	"ARM/Nordic/nRF52/src/system_nrf52.c",
	"ARM/Nordic/nRF91/src/system_nrf91.c",
	"ARM/Nordic/nRF54/src/system_nrf54l.c",
	"ARM/Nordic/nRF54/src/system_nrf54h.c",
)


class ArmClockDelayUpdates(unittest.TestCase):
	def test_all_port_factors_share_one_frequency_update_function(self):
		for path in PORTS:
			source = (ROOT / path).read_text(encoding="utf-8")

			with self.subTest(path=path):
				self.assertIn('#include "idelay_calibration.h"', source)
				self.assertIn("ArmDelayClockSetDivisors(SystemCoreClock,", source)
				self.assertNotIn(
					"SystemMicroSecLoopCnt = ((uint64_t)SystemCoreClock << 16)",
					source)
				self.assertNotIn(
					"SystemnsDelayFactor = ((uint64_t)SystemCoreClock << 16)",
					source)
				self.assertIn("SystemCoreClockUpdate(void)", source)

	def test_sam4l_changes_both_factors_in_direct_transitions(self):
		source = (ROOT / "ARM/Microchip/SAM4L/src/system_sam4l.c").read_text(
			encoding="utf-8")
		self.assertEqual(source.count("ArmDelayClockSetDivisors(SystemCoreClock,"), 3)
		self.assertIn("SystemCoreClock = RCSYS_FREQ;", source)
		self.assertIn("SystemCoreClock = Sam4lApplyClkDiv(mainRate, sel);", source)
		self.assertIn("SystemCoreClock = Sam4lApplyClkDiv(s_MainClock,", source)

	def test_no_dwt_fallback_preserves_original_arithmetic(self):
		source = HELPER.read_text(encoding="utf-8")
		self.assertIn("bool ArmDelayClockSetDivisors(", source)
		self.assertIn("CpuHz == 0U || UsDivisor == 0U || NsDivisor == 0U", source)
		self.assertIn("usFactor = ((uint64_t)CpuHz << 16) / UsDivisor", source)
		self.assertIn("nsFactor = ((uint64_t)CpuHz << 16) / NsDivisor", source)
		for clock in (1000000, 4000000, 16000000, 64000000, 128000000):
			for usdiv, nsdiv in ((13000000, 3000000), (14000000, 3000000),
				(16000000, 3000000), (16000000, 2720000)):
				with self.subTest(cpu=clock, usdiv=usdiv, nsdiv=nsdiv):
					usfactor = clock * 65536 // usdiv
					nsfactor = clock * 65536 // nsdiv
					self.assertGreater(usfactor, 0)
					self.assertGreater(nsfactor, 0)
					self.assertEqual(usfactor, (clock << 16) // usdiv)
					self.assertEqual(nsfactor, (clock << 16) // nsdiv)

	def test_measured_cost_is_cached_for_frequency_changes(self):
		source = HELPER.read_text(encoding="utf-8")
		start = source.index("static inline bool ArmDelayClockUpdate(")
		end = source.index("static inline bool ArmDelayClockInit(", start)
		clock_update = source[start:end]

		self.assertIn("pCal->UsCycles", clock_update)
		self.assertIn("pCal->NsCycles", clock_update)
		self.assertIn("pCal->CpuHz = CpuHz;", clock_update)
		self.assertNotIn("DWT->", clock_update)
		self.assertNotIn("usDelay(", clock_update)
		self.assertNotIn("nsDelay(", clock_update)
		self.assertNotIn("__disable_irq()", clock_update)

	def test_delay_implementation_is_unchanged(self):
		source = DELAY.read_text(encoding="utf-8")
		self.assertIn("void usDelay(uint32_t cnt)", source)
		self.assertIn("void nsDelay(uint32_t cnt)", source)
		self.assertIn("void msDelay(uint32_t ms)", source)


if __name__ == "__main__":
	unittest.main()
