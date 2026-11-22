#!/usr/bin/env python3
"""Regression checks for the nRF52 Power & Clock 0.14 migration.

These source checks do not replace Eclipse managed builds or hardware tests.
"""
from pathlib import Path
import unittest

ROOT = Path(__file__).resolve().parents[2]
POWER = ROOT / "ARM/Nordic/nRF52/src/system_power_nrf52.cpp"
CLOCK = ROOT / "ARM/Nordic/src/system_clock_nrfx.cpp"
GPIO = ROOT / "ARM/Nordic/src/iopincfg_nrfx.c"
BUILD = ROOT / "tests/nordic/build_managed_power_clock.py"


def section(source, start, end):
    return source[source.index(start):source.index(end, source.index(start))]


class Nrf52PowerClockRegression(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.power = POWER.read_text(encoding="utf-8")
        cls.clock = CLOCK.read_text(encoding="utf-8")
        cls.gpio = GPIO.read_text(encoding="utf-8")
        cls.build = BUILD.read_text(encoding="utf-8")

    def test_resumable_sleep_preserves_irq_and_scr(self):
        sleep = section(self.power, "void SystemSleep(void)", "uint32_t SystemPowerCapabilitiesGet")
        for token in ("__get_IPSR()", "__get_PRIMASK()", "__disable_irq()",
                      "__WFI()", "SCB->SCR = scr;", "__set_PRIMASK(state)"):
            with self.subTest(token=token):
                self.assertIn(token, sleep)
        self.assertNotIn("IOPinDisableInterrupt", sleep)
        self.assertNotIn("SystemLowFreqClockDisable", sleep)

    def test_power_capabilities_are_explicit(self):
        caps = section(self.power, "uint32_t SystemPowerCapabilitiesGet(void)",
                       "void SystemPowerOff(void)")
        for token in ("SYSTEM_POWER_CAP_SLEEP", "SYSTEM_POWER_CAP_SYSTEM_OFF",
                      "SYSTEM_POWER_CAP_WAKE_PIN", "SYSTEM_POWER_CAP_TIMER_WAKE",
                      "SYSTEM_POWER_CAP_RAM_RETENTION", "SYSTEM_POWER_CAP_RET_REG"):
            self.assertIn(token, caps)
        self.assertNotIn("SYSTEM_POWER_CAP_LOW_POWER", caps)

    def test_system_off_rearms_wake_sense_after_irq_cleanup(self):
        off = section(self.power, "void SystemPowerOff(void)", "bool SystemWakePin(")
        wake = section(self.power, "bool SystemWakePin(", "int SystemRetRegCount(")
        cleanup = section(self.gpio, "void IOPinDisableInterrupt(", "void IOPinFreeInterrupt(") if "void IOPinFreeInterrupt(" in self.gpio else self.gpio[self.gpio.index("void IOPinDisableInterrupt("):]
        self.assertIn("SysPwrWakePinsRearm();", off)
        self.assertLess(off.index("SysPwrWakePinsRearm();"), off.index("sd_power_system_off()"))
        self.assertIn("s_SysPwrWakeMask[PortNo] |= bit", wake)
        self.assertIn("s_SysPwrWakeMask[PortNo] &= ~bit", wake)
        self.assertIn("s_SysPwrWakeHigh[PortNo]", wake)
        self.assertIn("GPIO_PIN_CNF_SENSE_Msk", cleanup)

    def test_deferred_lfclk_stop_is_nrf52_softdevice_only(self):
        declaration = section(self.clock, "static bool s_bNRFxLfclkStopPending",
                              "// Temporarily selected sources")
        self.assertIn("#if defined(NRF52_SERIES) && defined(SOFTDEVICE_PRESENT)", declaration)
        self.assertIn("s_bNRFxSdLfclkStopPending", declaration)
        handler = section(self.clock, "static void nRFxClockSdhStateHandler(",
                          "NRF_SDH_STATE_OBSERVER(")
        self.assertIn("NRF_SDH_EVT_STATE_DISABLED", handler)
        self.assertIn("s_bNRFxSdLfclkStopPending", handler)
        self.assertIn("s_nRFxLfclkReqCnt != 0U", handler)
        self.assertIn("s_nRFxMpslLfclkReqCnt == 0U", handler)
        self.assertIn("NRF_CLOCK->TASKS_LFCLKSTOP = 1U", handler)
        disable = section(self.clock, "void SystemLowFreqClockDisable(",
                          "bool SystemLowFreqClockSelect(")
        self.assertIn("s_bNRFxSdLfclkStopPending = true", disable)

    def test_mpsl_ownership_and_generic_drivers_remain_intact(self):
        for token in ("nRFxMpslHfclkRequest(void)", "nRFxMpslHfclkRelease(void)",
                      "nRFxMpslLfclkRequest(void)", "nRFxMpslLfclkRelease(void)",
                      "bool SystemHighFreqClockEnable(void)",
                      "void SystemHighFreqClockDisable(void)"):
            self.assertIn(token, self.clock)
        self.assertNotIn("nRFxLfclkAcquire", self.clock)
        self.assertNotIn("DeviceIntrfWaitTxComplete", self.power + self.clock)

    def test_managed_build_script_covers_primary_nrf52(self):
        self.assertIn('"nrf52832"', self.build)
        self.assertIn('"nrf52840"', self.build)
        self.assertIn('root / "exemples/HfxoTestBle/ioc"', self.build)


if __name__ == "__main__":
    unittest.main()
