#!/usr/bin/env python3
"""Review Nordic power, clock and GPIO ownership paths in production sources.

Source checks supplement, but never replace, target builds and board tests.
"""
from pathlib import Path
import re
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
        self.assertLess(sleep.index("IOPinAllocateInterrupt(2,"), sleep.index("sd_app_evt_wait()"))
        self.assertIn("EnableInterrupt(state);", sleep)
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
            self.assertRegex(board, r"#define\s+POWER_DEMO_UART_OFF\s+1")

    def test_nrf52_uses_microchip_generic_resume_test(self):
        source = self.power52
        sleep = function(source, "void SystemSleep(void)", "void SystemPowerOff(void)")
        wake = function(source, "bool SystemWakePin(", "int SystemRetRegCount(")
        self.assertIn("IOPinAllocateInterrupt(2,", sleep)
        self.assertIn("IOPinDisableInterrupt(wakeIrq)", sleep)
        self.assertIn("s_SysPwrSleepSense", sleep)
        self.assertIn("s_SysPwrSleepSense = (IOPINSENSE)Sense;", wake)
        for mcu in ("nRF52832", "nRF52840"):
            board = (ROOT / "ARM/Nordic/nRF52" / mcu /
                     "exemples/PowerOffDemo/src/board.h").read_text()
            self.assertRegex(board, r"#define\s+POWER_DEMO_RESUME_WAKE\s+1")
            self.assertRegex(board, r"#define\s+POWER_DEMO_STANDBY_RAM_TEST\s+1")
            self.assertRegex(board, r"#define\s+POWER_DEMO_UART_WAKE\s+0")
            timer = re.search(r"#define\s+POWER_DEMO_TIMER_WAKE_MS\s+(\d+)\b", board)
            self.assertIsNotNone(timer)
            self.assertGreaterEqual(int(timer.group(1)), 0)
            self.assertRegex(board, r"#define\s+POWER_DEMO_TIMER_WAKE_DEVNO\s+\S+")
            self.assertRegex(board, r"#define\s+POWER_DEMO_UART_DMA\s+false")
            self.assertRegex(board, r"#define\s+WAKE_BUT_SENSE\s+IOPINSENSE_LOW_TRANSITION")
            self.assertNotIn("POWER_DEMO_NRF52_SLEEP_TEST", board)

    def test_generic_timer_wake_starts_timer_before_sleep(self):
        demo = (ROOT / "exemples/misc/power_off_demo.cpp").read_text()
        start = demo.index("timerStarted = period != 0U && TimerEnable(&s_WakeTimer);")
        sleep = demo.index("SystemSleep();", start)
        stop = demo.index("TimerDisable(&s_WakeTimer);", sleep)
        self.assertLess(start, sleep)
        self.assertLess(sleep, stop)
        self.assertIn("timerStarted && s_StandbyTimerWoke", demo)

    def test_nrf54l_off_checks_clock_owner(self):
        off = function(self.power, "void SystemPowerOff(void)", "bool SystemWakePin(")
        self.assertLess(off.index("nRFxClockPrepareSystemOff()"),
                        off.index("SYS_PWR_REGULATORS->SYSTEMOFF"))


if __name__ == "__main__":
    unittest.main()
