#!/usr/bin/env python3
"""Review Nordic power, clock and GPIO ownership paths in production sources.

Source checks supplement, but never replace, target builds and board tests.
"""
from pathlib import Path
import unittest

ROOT = Path(__file__).resolve().parents[2]
POWER52 = ROOT / "ARM/Nordic/nRF52/src/system_power_nrf52.cpp"
POWER = ROOT / "ARM/Nordic/src/system_power_nrfx.cpp"
CLOCK = ROOT / "ARM/Nordic/src/system_clock_nrfx.cpp"
GPIO = ROOT / "ARM/Nordic/src/iopincfg_nrfx.c"


def function(text, name, next_name):
    start = text.index(name)
    stop = text.index(next_name, start)
    return text[start:stop]


class NordicPowerSource(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.power52 = POWER52.read_text()
        cls.power = POWER.read_text()
        cls.clock = CLOCK.read_text()
        cls.gpio = GPIO.read_text()

    def test_softdevice_sleep_uses_softdevice_wait(self):
        sleep = function(self.power52, "void SystemSleep(void)", "void SystemPowerOff(void)")
        self.assertIn("SysPwrSdEnabled()", sleep)
        self.assertIn("sd_app_evt_wait()", sleep)
        self.assertLess(sleep.index("sd_app_evt_wait()"), sleep.index("DisableInterrupt()"))
        self.assertIn("__WFI()", sleep)

    def test_resumable_sleep_does_not_enter_system_off(self):
        for power in (self.power52, self.power):
            sleep = function(power, "void SystemSleep(void)", "void SystemPowerOff(void)")
            self.assertIn("SCB_SCR_SLEEPDEEP_Msk", sleep)
            self.assertIn("SCB_SCR_SLEEPONEXIT_Msk", sleep)
            self.assertIn("SCB->SCR = scr;", sleep)
            self.assertIn("EnableInterrupt(state)", sleep)
            self.assertNotIn("SYSTEMOFF =", sleep)

    def test_gpio_domain_flag_is_decoded_before_switch(self):
        get = function(self.gpio, "NRF_GPIO_Type *nRFGpioGetReg(int PortNo)\n{",
                       "static NRF_GPIOTE_Type *nRFGpioteGetReg(")
        self.assertIn("(PortNo & 0x7F) >= GPIO_COUNT", get)
        self.assertIn("switch (PortNo & 0x7F)", get)
        self.assertIn("PortNo & 0x80", get)

    def test_wake_pin_validation_uses_physical_port_index(self):
        for power in (self.power52, self.power):
            wake = function(power, "bool SystemWakePin(", "int SystemRetRegCount(")
            for port in range(4):
                self.assertIn(
                    "if ((PortNo & 0x7F) == %d && PinNo >= IOPIN_P%d_MAXCOUNT)"
                    % (port, port), wake
                )

    def test_lf_clock_transition_refuses_isr(self):
        select = function(self.clock, "bool SystemLowFreqClockSelect(",
                          "uint32_t SystemPeriphClockGet(")
        self.assertIn("__get_IPSR() != 0U", select)
        self.assertIn("nRFxClockSdEnabled() || nRFxMpslActive()", select)

    def test_nrf52_demo_drains_console_before_system_off(self):
        demo = (ROOT / "exemples/misc/power_off_demo.cpp").read_text()
        self.assertIn("CFifoUsed(SysLogGet()->hFifo)", demo)
        self.assertIn("CFifoUsed(pUart->hTxFifo)", demo)
        self.assertIn("s_Uart.PowerOff();", demo)
        self.assertLess(demo.index("s_Uart.PowerOff();"),
                        demo.index("SystemPowerOff();", demo.index("s_Uart.PowerOff();")))
        for target in ("nRF52832", "nRF52840"):
            board = (ROOT / "ARM/Nordic/nRF52" / target /
                     "exemples/PowerOffDemo/src/board.h").read_text()
            self.assertIn("#define POWER_DEMO_UART_OFF\t1", board)

    def test_nrf52_resumable_timer_profile(self):
        demo = (ROOT / "exemples/misc/power_off_demo.cpp").read_text()
        sleep = demo.index("uint64_t period = nsTimerEnableTrigger(&s_WakeTimer")
        enable = demo.index("started = period != 0U && TimerEnable(&s_WakeTimer)", sleep)
        entry = demo.index("SystemSleep();", enable)
        self.assertLess(sleep, enable)
        self.assertLess(enable, entry)
        self.assertIn("started && s_StandbyTimerWoke", demo)
        for target in ("nRF52832", "nRF52840"):
            board = (ROOT / "ARM/Nordic/nRF52" / target /
                     "exemples/PowerOffDemo/src/board.h").read_text()
            self.assertIn("#define POWER_DEMO_NRF52_SLEEP_TEST\t0", board)
            self.assertIn("#define POWER_DEMO_TIMER_WAKE_MS\t\t1000", board)
            self.assertIn("#define POWER_DEMO_STANDBY_RAM_TEST\t1", board)
            self.assertIn("TimerGetLowFreqDevCount()", board)

    def test_nrf54l_off_checks_clock_owner(self):
        off = function(self.power, "void SystemPowerOff(void)", "bool SystemWakePin(")
        self.assertLess(off.index("nRFxClockPrepareSystemOff()"),
                        off.index("SYS_PWR_REGULATORS->SYSTEMOFF"))


if __name__ == "__main__":
    unittest.main()
