#!/usr/bin/env python3
"""QDEC PEI regression and explicit exclusion of one-shot nRF52 endpoints."""
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
QDEC = (ROOT / "ARM/Nordic/src/qdec_nrfx.cpp").read_text()
QDEC_H = (ROOT / "include/coredev/qdec.h").read_text()
WDT = (ROOT / "ARM/Nordic/src/wdt_nrfx.cpp").read_text()
WDT_H = (ROOT / "include/coredev/wdt.h").read_text()
TEMP = (ROOT / "ARM/Nordic/src/temp_nrf5x.cpp").read_text()
TEMP_H = (ROOT / "ARM/Nordic/include/temp_nrf5x.h").read_text()
RNG = (ROOT / "ARM/Nordic/src/rng_nrfx.cpp").read_text()
RNG_H = (ROOT / "ARM/Nordic/include/crypto_rng_nrf.h").read_text()
PPI = (ROOT / "ARM/Nordic/nRF52/src/pei_nrf52.cpp").read_text()

def main():
    events = QDEC.split("static PeiEndpoint_t QdecPeiEventEndpoint(", 1)[1]
    events = events.split("static PeiEndpoint_t QdecPeiTaskEndpoint(", 1)[0]
    tasks = QDEC.split("static PeiEndpoint_t QdecPeiTaskEndpoint(", 1)[1]
    tasks = tasks.split('extern "C" bool QdecPreparePeiStart(', 1)[0]
    for event in ("SAMPLERDY", "REPORTRDY", "ACCOF", "DBLRDY", "STOPPED"):
        assert "&NRF_QDEC->EVENTS_" + event in events
    for task in ("START", "STOP"):
        assert "&NRF_QDEC->TASKS_" + task in tasks
    assert "READCLRACC" not in tasks
    assert "NRF_QDEC->TASKS_READCLRACC = 1;" in QDEC
    assert "QdecPreparePeiStart(" in QDEC_H
    for name in ("PreparePeiStart", "EnablePeiEvent", "DisablePeiEvent",
                 "ConnectPeiTask", "DisconnectPeiTask"):
        assert "bool Qdec" + name + "(" in QDEC_H
        assert 'extern "C" bool Qdec' + name + "(" in QDEC
    for symbol in ("PeiBindEvent", "PeiGetEventEndpoint", "PeiUnbindEvent",
                   "PeiConnectTask", "PeiDisconnectTask"):
        assert "#pragma weak " + symbol in QDEC
    demo = (ROOT / "exemples/pei/pei_qdec_demo.cpp").read_text()
    assert "QDEC_PEI_EVENT_SAMPLERDY" in demo
    assert "QDEC_PEI_TASK_START" in demo
    assert "QDEC_PEI_TASK_STOP" in demo
    assert "g_PeiQdecStatus = 1;" in demo

    for source, marker in (
        (WDT, "WdtEnablePeiEvent"), (WDT_H, "WDT_PEI_EVENT"),
        (TEMP, "TempNrfPei"), (TEMP_H, "TEMP_NRF_PEI_"),
        (RNG, "RngNrfPei"), (RNG_H, "RNG_NRF_PEI_"),
        (PPI, "PeiNrf52BindPowerEvent"),
        (PPI, "PeiNrf52BindClockEvent")
    ):
        assert marker not in source, marker
    for path in (
        "ARM/Nordic/include/temp_nrf_pei.h",
        "ARM/Nordic/include/rng_nrf_pei.h",
        "ARM/Nordic/include/pei_nrf52_power_clock.h",
        "exemples/pei/pei_temp_demo.cpp",
        "exemples/pei/pei_rng_demo.cpp"
    ):
        assert not (ROOT / path).exists(), path
    # Preserve existing working TEMP interrupt behavior.
    assert "NRF_TEMP->INTENCLR = 1U;" in TEMP
    assert "NRF_TEMP->EVENTS_DATARDY = 0U;" in TEMP
    assert "NRF_TEMP->TASKS_STOP = 1;" in TEMP
    assert "WdtStart(" in WDT and "WdtReload(" in WDT
    assert "RngPeriphRxData(" in RNG and "reg->VALUE" in RNG

    print("nRF52 QDEC and one-shot PEI exclusion contracts: PASS")

if __name__ == "__main__":
    main()
