#!/usr/bin/env python3
"""nRF52 WDT, POWER/CLOCK and PPI-group source contracts.

This checks source-level invariants only, not MCU builds or hardware.
"""
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
WDT = (ROOT / "ARM/Nordic/src/wdt_nrfx.cpp").read_text()
WDT_H = (ROOT / "include/coredev/wdt.h").read_text()
PPI = (ROOT / "ARM/Nordic/nRF52/src/pei_nrf52.cpp").read_text()
PEI_H = (ROOT / "include/coredev/pei.h").read_text()
PEI_CPP = (ROOT / "include/coredev/pei_matrix.h").read_text()
SYSTEM_H = (ROOT / "ARM/Nordic/include/pei_nrf52_power_clock.h").read_text()
TEST = (ROOT / "tests/pei/ppi_nrf52_test.c").read_text()
DEMO = (ROOT / "exemples/pei/pei_group_demo.cpp").read_text()


def section(source, start, end):
    assert source.count(start) == 1, start
    assert source.count(end) == 1, end
    return source.split(start, 1)[1].split(end, 1)[0]


def main():
    wdt_routes = section(WDT, 'extern "C" bool WdtEnablePeiEvent(',
                         'extern "C" bool WdtDisablePeiEvent(')
    assert "&NRF_WDT->EVENTS_TIMEOUT" in wdt_routes
    assert "pDev != s_pWdtnRFDev" in wdt_routes
    assert "WdtStart(" in WDT
    assert "WdtReload(" in WDT
    assert "WdtConnectPeiTask" not in WDT
    assert "WdtConnectPeiTask" not in WDT_H
    assert "WDT_PEI_EVENT_TIMEOUT" in WDT_H

    power = section(PPI, "static PeiEndpoint_t PeiNrf52PowerEndpoint(",
                    "static PeiEndpoint_t PeiNrf52ClockEndpoint(")
    clock = section(PPI, "static PeiEndpoint_t PeiNrf52ClockEndpoint(",
                    "bool PeiNrf52BindPowerEvent(")
    for event in ("POFWARN", "SLEEPENTER", "SLEEPEXIT"):
        assert "&NRF_POWER->EVENTS_" + event in power
    for event in ("USBDETECTED", "USBREMOVED", "USBPWRRDY"):
        assert "&NRF_POWER->EVENTS_" + event in power
        assert "PEI_NRF52_POWER_EVENT_" + event in SYSTEM_H
    assert "#if defined(POWER_INTENSET_USBDETECTED_Msk)" in power
    for event in ("HFCLKSTARTED", "LFCLKSTARTED", "DONE", "CTTO"):
        assert "&NRF_CLOCK->EVENTS_" + event in clock
    assert "#if defined(CLOCK_INTENSET_CTSTARTED_Msk)" in clock
    assert "TASKS_HFCLKSTOP" not in clock
    assert "TASKS_LFCLKSTOP" not in clock
    assert "TASKS_CONSTLAT" not in power
    assert "PeiNrf52BindPowerEvent" in SYSTEM_H
    assert "PeiNrf52BindClockEvent" in SYSTEM_H

    groups = section(PPI, "bool PeiGroupInit(",
                     "// Software events on EGU channels.") if (
        PPI.index("bool PeiGroupInit(") < PPI.index(
            "// Software events on EGU channels.")) else PPI.split(
                "bool PeiGroupInit(", 1)[1]
    assert "NRF_PPI->TASKS_CHG[Group].EN" in PPI
    assert "NRF_PPI->TASKS_CHG[Group].DIS" in PPI
    assert "NRF_SOC_SD_PPI_GROUPS_SD_ENABLED_MSK" in PPI
    assert "NRFXLIB_SDC" in PPI
    assert "sd_ppi_group_assign" in PPI
    assert "PeiNrf52OwnedChannel(&pPei->Routes[i].Conn)" in PPI
    assert "NRF_PPI->CHEN & bit" in groups
    assert "PeiNrf52GroupAssign(i, NRF_PPI->CHG[i] & ~bit)" in PPI
    assert "PeiNrf52GroupTaskInUse(group)" in groups
    assert "PeiNrf52OwnedGroup(pGroup)" in groups
    assert "PeiGroup_t" in PEI_H
    assert "PeiGroupAddEvent(" in PEI_H
    assert "bool GroupAddEvent(PeiGroup_t &Group" in PEI_CPP
    assert "PeiGroupDeinit(" in PEI_H
    assert "static void TestGroups(void)" in TEST
    assert "PeiGroupDisableTask(&group[0])" in TEST
    assert "PeiGroupEnableTask(&copied) == 0U" in TEST

    assert "PeiGroup_t group = {};" in DEMO
    assert "pei.GroupAddEvent(group, fireEvt)" in DEMO
    assert "pei.GroupEnableTask(group)" in DEMO
    assert "pei.GroupDisableTask(group)" in DEMO
    assert "pei.Enable(armEvt)" in DEMO
    assert "pei.Enable(fireEvt)" not in DEMO.split("g_PeiGroupStatus = 1;")[0]
    assert "pei.BindEvent(" not in DEMO
    assert 'extern "C"' not in DEMO
    assert "NRF_PPI" not in DEMO
    print("nRF52 WDT/POWER/CLOCK/PPI group source contracts: PASS")


if __name__ == "__main__":
    main()
