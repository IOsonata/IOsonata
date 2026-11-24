#!/usr/bin/env python3
"""Check the TimerPowerOff C/C++ declarations and MCU timer drivers."""
from pathlib import Path
import unittest

ROOT = Path(__file__).resolve().parents[1]
HEADER = ROOT / "include/coredev/timer.h"

class TimerPowerOffChecks(unittest.TestCase):
	def read(self, path):
		return (ROOT / path).read_text(encoding="utf-8")

	def test_timer_interface(self):
		src = HEADER.read_text(encoding="utf-8")
		self.assertIn("bool (*PowerOff)(TimerDev_t * const pTimerDev);", src)
		self.assertIn("static inline bool TimerPowerOff(TimerDev_t * const pTimer)", src)
		self.assertIn("pTimer->PowerOff(pTimer) : false;", src)
		self.assertIn("virtual bool PowerOff() { return TimerPowerOff(&vTimer); }", src)
		self.assertIn("vTimer.PowerOff = NULL;", src)
		self.assertIn("static inline void TimerDisable(", src)
		self.assertNotIn("TimerDeinit", src)

	def test_nordic_timer_shutdown(self):
		for path, callback, owner, guard in (
			("ARM/Nordic/src/timer_hf_nrfx.cpp", "nRFxTimerPowerOff",
				"s_pnRFxTimerDev[devno]", "nRFxTimerOwned"),
			("ARM/Nordic/src/timer_lf_nrfx.cpp", "nRFxRtcPowerOff",
				"s_pnRfxRtcDev[devno]", "nRFxRtcOwned"),
		):
			with self.subTest(timer=path):
				src = self.read(path)
				self.assertIn("static bool " + callback + "(", src)
				begin = src.index("static bool " + callback + "(")
				end = src.index("\n}\n", begin)
				body = src[begin:end]
				self.assertIn(owner + " = nullptr;", body)
				self.assertIn("pTimer->DevNo = -1;", body)
				self.assertIn("pTimer->PowerOff = nullptr;", body)
				self.assertIn("TIMER_TRIG_PEI_SINGLE", body)
				self.assertIn("TIMER_TRIG_PEI_CONTINUOUS", body)
				self.assertIn("pTimer->PowerOff = " + callback + ";", src)
				self.assertIn("static bool " + guard + "(", src)
				self.assertIn("!" + guard + "(pTimer)", src)

	def test_nordic_shared_clock_protection(self):
		hf = self.read("ARM/Nordic/src/timer_hf_nrfx.cpp")
		rtc = self.read("ARM/Nordic/src/timer_lf_nrfx.cpp")
		grtc = self.read("ARM/Nordic/nRF54/src/timer_lf_nrf54.cpp")
		self.assertIn("SystemHighFreqClockDisable();", hf)
		self.assertIn("SystemLowFreqClockDisable();", rtc)
		self.assertIn("static bool nRFxGrtcPowerOff(", grtc)
		begin = grtc.index("static bool nRFxGrtcPowerOff(")
		end = grtc.index("\n}\n", begin)
		body = grtc[begin:end]
		self.assertIn("s_pGrtcOwner[devno] = nullptr;", body)
		self.assertIn("NRF_GRTC->CC[idx].CCEN = 0U;", body)
		self.assertNotIn("NRF_GRTC->TASKS_STOP", body)

	def test_microchip_and_stm32(self):
		targets = {
			"ARM/Microchip/SAMD/src/timer_samd.cpp": "SamdTimerPowerOff",
			"ARM/Microchip/SAM4L/src/timer_sam4l.cpp": "PowerOff",
			"ARM/Microchip/SAM4E/src/timer_sam4e.cpp": "Sam4ePowerOff",
			"ARM/ST/src/timer_stm32.cpp": "Stm32TimerPowerOff",
		}
		for path, callback in targets.items():
			with self.subTest(timer=path):
				src = self.read(path)
				self.assertIn("static bool " + callback + "(", src)
				self.assertIn("->PowerOff = " + callback + ";", src)

	def test_nxp_lpc_timer_clock_stop(self):
		header = self.read("ARM/NXP/include/timer_lpcxx.h")
		driver = self.read("ARM/NXP/src/timer_lpcxx.cpp")
		self.assertIn("void LpcTimerClockDisable(", header)
		self.assertIn("static bool LpcTimerPowerOff(", driver)
		self.assertIn("LpcTimerClockDisable(dev);", driver)
		self.assertIn("dev->pTimer = NULL;", driver)
		for path in ("ARM/NXP/LPC11xx/src/timer_lpc11uxx.cpp",
				"ARM/NXP/LPC17xx/src/timer_lpc17xx.cpp",
				"ARM/NXP/LPC546xx/src/timer_lpc546xx.cpp"):
			with self.subTest(timer=path):
				self.assertIn("void LpcTimerClockDisable(", self.read(path))

	def test_esp32_timer_interrupt_and_clock_stop(self):
		driver = self.read("RISCV/Espressif/src/timer_esp32.cpp")
		self.assertIn("static bool Esp32PowerOff(", driver)
		self.assertIn("Esp32DisableSourceIrq(pData->IrqSource, pData->CpuInt);", driver)
		self.assertIn("Esp32DisableTimerClock(devno);", driver)
		self.assertIn("pData->pTimer = NULL;", driver)
		self.assertIn("pTimer->PowerOff = Esp32PowerOff;", driver)

	def test_renesas_timer_shutdown(self):
		ra = self.read("ARM/Renesas/RA4/RA4M1/src/timer_ra4m1.cpp")
		re = self.read("ARM/Renesas/RE01/src/timer_re01.cpp")
		r9 = self.read("RISCV/Renesas/R9A02/src/timer_r9a02.cpp")
		self.assertIn("static bool PowerOff(TimerDev_t *t)", ra)
		self.assertIn("Ra4m1UnregisterIntHandler(", ra)
		self.assertIn("ModuleBit(s_Timer[i]) == mask", ra)
		self.assertIn("t->PowerOff = PowerOff;", ra)
		self.assertIn("static bool Re01TimerPowerOff(", re)
		self.assertIn("Re01UnregisterIntHandler(", re)
		self.assertIn("MSTP->MSTPCRD |= mask;", re)
		self.assertIn("pTimer->PowerOff = Re01TimerPowerOff;", re)
		for name in ("R9a02MtimerPowerOff", "R9a02TauPowerOff"):
			self.assertIn("static bool " + name + "(", r9)
			self.assertIn("pTimer->PowerOff = " + name + ";", r9)
		self.assertIn("R9a02ModuleStop(R9A02_MSTPCR_D, R9A02_MSTPCRD_TAU);", r9)

	def test_systick_power_off(self):
		src = self.read("ARM/src/timer_systick.cpp")
		self.assertIn("static bool SystickPowerOff(", src)
		self.assertIn("pTimer->PowerOff = SystickPowerOff;", src)
		self.assertIn("SYSTICK_ICSR = SCB_ICSR_PENDSTCLR;", src)
		self.assertIn("d->pTimer = nullptr;", src)
		self.assertIn("d.pTimer != nullptr && d.pTimer != pTimer", src)
		self.assertIn("SysTickTimerPowerOff(", self.read("ARM/include/timer_systick.h"))
		self.assertIn("return SystickPowerOff(pTimer);", src)
		self.assertIn("SysTickTimerPowerOff(pTimer)",
			self.read("ARM/Microchip/SAMD/src/timer_samd.cpp"))

	def test_generic_example_uses_timer_power_off(self):
		src = self.read("exemples/misc/core_clock_test.cpp")
		begin = src.index("static void CoreClockTestTimerPowerOff(void)")
		end = src.index("int main(void)", begin)
		test = src[begin:end]
		self.assertIn("TimerPowerOff(&s_RetainedTimer);", test)
		self.assertIn("TimerInit(&s_PowerOffTimer, &cfg);", test)
		self.assertIn("TimerPowerOff(&s_PowerOffTimer);", test)
		self.assertIn("!TimerPowerOff(&s_PowerOffTimer)", test)
		self.assertIn("CoreClockTestTimerPowerOff();", src)
		self.assertNotIn("NRF91_SERIES", test)

if __name__ == "__main__":
	unittest.main()
