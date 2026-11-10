#!/usr/bin/env python3
"""PPI group automation must keep ownership and group tests intact."""
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
PPI = (ROOT / "ARM/Nordic/nRF52/src/pei_nrf52.cpp").read_text()
PEI_H = (ROOT / "include/coredev/pei.h").read_text()
PEI_CPP = (ROOT / "include/coredev/pei_matrix.h").read_text()
TEST = (ROOT / "tests/pei/ppi_nrf52_test.c").read_text()
DEMO = (ROOT / "exemples/pei/pei_group_demo.cpp").read_text()

def main():
    assert "bool PeiGroupInit(" in PPI
    groups = PPI.split("bool PeiGroupInit(", 1)[1]
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
    assert "\tif (!pei.Enable(fireEvt))" not in DEMO
    assert "NRF_PPI" not in DEMO
    assert "PeiNrf52PowerEndpoint" not in PPI
    assert "PeiNrf52ClockEndpoint" not in PPI
    print("nRF52 PPI group automation source source checks: PASS")

if __name__ == "__main__":
    main()
