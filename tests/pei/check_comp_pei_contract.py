#!/usr/bin/env python3
"""Static contract for COMP/LPCOMP PEI endpoints and bare-metal test.

Hardware is not emulated by this check. Build/flash PeiCompDemo to validate
actual nRF52832 threshold crossings and event-to-task routing.
"""
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
SOURCE = (ROOT / "ARM/Nordic/src/analog_comp_nrf5x.cpp").read_text()
HEADER = (ROOT / "include/converters/analog_comp.h").read_text()
EXAMPLE = (ROOT / "exemples/pei/pei_comp_demo.cpp").read_text()


def section(first, last):
    assert SOURCE.count(first) == 1, first
    assert SOURCE.count(last) == 1, last
    return SOURCE.split(first, 1)[1].split(last, 1)[0]


def main():
    events = section("static PeiEndpoint_t AnalogCompPeiEventEndpoint(",
                     "static PeiEndpoint_t AnalogCompPeiTaskEndpoint(")
    tasks = section("static PeiEndpoint_t AnalogCompPeiTaskEndpoint(",
                    "static bool AnalogCompPeiReady(")
    connect = section('extern "C" bool AnalogCompPeiConnectTask(',
                      'extern "C" bool AnalogCompPeiDisconnectTask(')
    for event in ("READY", "DOWN", "UP", "CROSS"):
        assert "&NRF_ACOMP->EVENTS_" + event in events
    for task in ("START", "STOP", "SAMPLE"):
        assert "&NRF_ACOMP->TASKS_" + task in tasks
    for api in ("PeiBindEvent", "PeiGetEventEndpoint", "PeiUnbindEvent",
                "PeiConnectTask", "PeiDisconnectTask"):
        assert "#pragma weak " + api in SOURCE
    for api in ("BindEvent", "UnbindEvent", "ConnectTask",
                "DisconnectTask"):
        assert "bool AnalogCompPei" + api in HEADER
        assert 'extern "C" bool AnalogCompPei' + api in SOURCE
    assert "ANALOG_COMP_PEI_TASK_SAMPLE" in connect
    assert "ANALOG_COMP_MODE_CONTINUOUS" in connect
    assert "if (pCfg->EvtHandler != NULL)" in SOURCE
    assert "pDev->EvtHandler = pCfg->EvtHandler;" in SOURCE
    assert "pDev->Mode = pCfg->Mode;" in SOURCE
    assert "bool AnalogCompInit(AnalogCompDev_t *pDev, const AnalogCompCfg_t *pCfg);" in HEADER
    assert "AnalogCompPeiReady(pDev)" in SOURCE
    assert "ANALOG_COMP_PEI_EVENT_CROSS" in EXAMPLE
    assert "ANALOG_COMP_PEI_EVENT_UP" in EXAMPLE
    assert "ANALOG_COMP_PEI_EVENT_DOWN" in EXAMPLE
    assert "ANALOG_COMP_PEI_EVENT_READY" in EXAMPLE
    assert "ANALOG_COMP_PEI_TASK_START" in EXAMPLE
    assert 'extern "C"' not in EXAMPLE
    assert "NRF_ACOMP->" not in EXAMPLE
    assert "NRF_LPCOMP->" not in EXAMPLE
    assert "printf(" not in EXAMPLE
    print("nRF52 comparator PEI endpoint and ownership contracts: PASS")


if __name__ == "__main__":
    main()
