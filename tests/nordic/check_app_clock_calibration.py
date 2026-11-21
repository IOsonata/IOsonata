#!/usr/bin/env python3
"""Check that periodic RC calibration remains an opt-in AppRun subsystem."""
from pathlib import Path
import re
import unittest

ROOT = Path(__file__).resolve().parents[2]
HEADER = ROOT / "include/app_clock_calibration.h"
HELPER = ROOT / "src/app_clock_calibration.cpp"
CORE = ROOT / "ARM/Nordic/src/system_clock_nrfx.cpp"
APP_RUN = ROOT / "src/app_run.cpp"
RTC = ROOT / "ARM/Nordic/src/timer_lf_nrfx.cpp"
DOC = ROOT / "docs/architecture/clock-calibration-app-run.md"


class ClockCalibrationAppRunTests(unittest.TestCase):
	def test_core_has_no_timer_or_app_run_dependency(self):
		source = CORE.read_text(encoding="utf-8")
		start = source.index("bool SystemLowFreqClockCalibrate(void)")
		end = source.index("// The clock manager alone starts", start)
		body = source[start:end]
		self.assertNotIn("TimerInit(", body)
		self.assertNotIn("AppEvtHandlerQue(", body)
		self.assertNotIn("AppRun(", body)
		self.assertIn("nRFxLfclkCalibrate()", body)

	def test_deferred_work_is_not_called_from_irq(self):
		source = HELPER.read_text(encoding="utf-8")
		start = source.index("static void AppClockCalibrationTrigger(")
		end = source.index("bool AppClockCalibrationStart(", start)
		irq = source[start:end]
		self.assertIn("AppEvtHandlerQue(0U, pCal, AppClockCalibrationWork)", irq)
		self.assertNotIn("SystemLowFreqClockCalibrate()", irq)
		work = source[source.index("static void AppClockCalibrationWork("):start]
		self.assertIn("SystemLowFreqClockCalibrate()", work)
		self.assertIn("SystemHighFreqClockCalibrate()", work)

	def test_trigger_uses_application_owned_timer(self):
		source = HELPER.read_text(encoding="utf-8")
		self.assertIn("nsTimerEnableTrigger(pTimer, TriggerNo,", source)
		self.assertIn("TIMER_TRIG_TYPE_CONTINUOUS, AppClockCalibrationTrigger, pCal", source)
		self.assertIn("TimerDisableTrigger(pCal->pTimer, pCal->TriggerNo)", source)
		for operation in ("TimerInit(", "TimerEnable(", "TimerDisable("):
			self.assertNotIn(operation, source)
		self.assertIn("pCal->pTimer != pTimer", source)
		self.assertIn("pCal->TriggerNo != TriggerNo", source)

	def test_coalesces_and_handles_queue_refusal(self):
		source = HELPER.read_text(encoding="utf-8")
		self.assertIn("pCal->Queued = true;", source)
		self.assertIn("pCal->Queued = false;", source)
		self.assertIn("pCal->QueueRefused++;", source)
		self.assertRegex(source, r"pCal->Active\s*\|\|\s*pCal->Queued")
		self.assertIn("if (!pCal->Active)", source)
		self.assertIn("pCal->Failed++;", source)
		self.assertIn("pCal->Completed++;", source)

	def test_app_run_dispatches_deferred_work(self):
		source = APP_RUN.read_text(encoding="utf-8")
		self.assertIn("AppEvtHandlerExec()", source)
		self.assertIn("AppEvtHandlerPending()", source)
		source = RTC.read_text(encoding="utf-8")
		self.assertIn("handler(timer, i, ctx)", source)

	def test_documentation_uses_public_interfaces(self):
		source = DOC.read_text(encoding="utf-8")
		self.assertIn("AppClockCalibrationStart(&s_Calibration", source)
		self.assertIn("AppClockCalibrationStop(&s_Calibration)", source)
		self.assertIn("AppRun();", source)
		self.assertNotIn("NRF_CLOCK->", source)
		self.assertIn("src/app_clock_calibration.cpp", source)
		header = HEADER.read_text(encoding="utf-8")
		self.assertIn("AppClockCalibrationStart(", header)
		self.assertIn("AppClockCalibrationStop(", header)


if __name__ == "__main__":
	unittest.main()
