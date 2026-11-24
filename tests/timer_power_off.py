#!/usr/bin/env python3
"""Source contracts for the generic IOsonata TimerPowerOff lifecycle."""
from pathlib import Path
import unittest

ROOT = Path(__file__).resolve().parents[1]
HEADER = ROOT / "include/coredev/timer.h"
NORDIC = ROOT / "ARM/Nordic/src"

class TimerPowerOffContract(unittest.TestCase):
	def test_generic_c_and_cpp_api(self):
		source = HEADER.read_text(encoding="utf-8")
		self.assertIn("bool (*PowerOff)(TimerDev_t * const pTimerDev);", source)
		self.assertIn("static inline bool TimerPowerOff(TimerDev_t * const pTimer)", source)
		self.assertIn("pTimer->PowerOff(pTimer) : false;", source)
		self.assertIn("virtual bool PowerOff() { return TimerPowerOff(&vTimer); }", source)
		self.assertIn("vTimer.PowerOff = NULL;", source)
		self.assertIn("static inline void TimerDisable(", source)

	def test_nordic_owners_released_only_by_power_off(self):
		hf = (NORDIC / "timer_hf_nrfx.cpp").read_text(encoding="utf-8")
		rtc = (NORDIC / "timer_lf_nrfx.cpp").read_text(encoding="utf-8")
		for src, cb, key, ptr in (
			(hf, "nRFxTimerPowerOff", "s_pnRFxTimerDev[devno]", "pTimer->PowerOff"),
			(rtc, "nRFxRtcPowerOff", "s_pnRfxRtcDev[devno]", "pTimer->PowerOff"),
		):
			self.assertIn("static bool " + cb + "(", src)
			start = src.index("static bool " + cb + "(")
			end = src.index("\n}\n", start)
			body = src[start:end]
			self.assertIn(key + " = nullptr;", body)
			self.assertIn(ptr + " = nullptr;", body)
			self.assertIn("return true;", body)
			self.assertIn("return false;", body)
			self.assertIn("pTimer->PowerOff = " + cb + ";", src)
			self.assertIn("TIMER_TRIG_PEI_SINGLE", body)
			self.assertIn("TIMER_TRIG_PEI_CONTINUOUS", body)
		self.assertIn("SystemHighFreqClockDisable();", hf)
		self.assertIn("SystemLowFreqClockDisable();", rtc)

	def test_grtc_shutdown_preserves_shared_syscounter(self):
		src = (ROOT / "ARM/Nordic/nRF54/src/timer_lf_nrf54.cpp").read_text(encoding="utf-8")
		start = src.index("static bool nRFxGrtcPowerOff(")
		end = src.index("\n}\n", start)
		body = src[start:end]
		self.assertIn("s_pGrtcOwner[devno] = nullptr;", body)
		self.assertIn("pRtc->pTimer = nullptr;", body)
		self.assertIn("NRF_GRTC->CC[idx].CCEN = 0U;", body)
		self.assertNotIn("NRF_GRTC->TASKS_STOP", body)
		self.assertIn("pTimer->PowerOff = nRFxGrtcPowerOff;", src)

	def test_other_implemented_family_callbacks(self):
		paths = {
			"ARM/Microchip/SAMD/src/timer_samd.cpp": "SamdTimerPowerOff",
			"ARM/Microchip/SAM4L/src/timer_sam4l.cpp": "PowerOff",
			"ARM/Microchip/SAM4E/src/timer_sam4e.cpp": "Sam4ePowerOff",
			"ARM/ST/src/timer_stm32.cpp": "Stm32TimerPowerOff",
		}
		for path, callback in paths.items():
			with self.subTest(path=path):
				src = (ROOT / path).read_text(encoding="utf-8")
				self.assertIn("static bool " + callback + "(", src)
				self.assertIn("->PowerOff = " + callback + ";", src)

	def test_unported_family_callbacks_explicitly_unsupported(self):
		paths = (
			"ARM/NXP/src/timer_lpcxx.cpp",
			"ARM/Renesas/RA4/RA4M1/src/timer_ra4m1.cpp",
			"ARM/Renesas/RE01/src/timer_re01_agt.cpp",
			"ARM/Renesas/RE01/src/timer_re01_gpt.cpp",
			"ARM/Renesas/RE01/src/timer_re01_tmr.cpp",
			"ARM/src/timer_systick.cpp",
			"RISCV/Espressif/src/timer_esp32.cpp",
			"RISCV/Renesas/R9A02/src/timer_r9a02.cpp",
		)
		for path in paths:
			with self.subTest(path=path):
				src = (ROOT / path).read_text(encoding="utf-8")
				self.assertIn("->PowerOff = nullptr;", src)

if __name__ == "__main__":
	unittest.main()
