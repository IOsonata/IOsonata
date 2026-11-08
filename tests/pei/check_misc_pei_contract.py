#!/usr/bin/env python3
"""nRF52 QDEC/TEMP/RNG PEI endpoints and ownership source contracts.

These checks are not a substitute for Nordic CMSIS compilation or
physical tests. All three hardware demonstrations remain unvalidated.
"""
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
QDEC = (ROOT / "ARM/Nordic/src/qdec_nrfx.cpp").read_text()
TEMP = (ROOT / "ARM/Nordic/src/temp_nrf5x.cpp").read_text()
RNG = (ROOT / "ARM/Nordic/src/rng_nrfx.cpp").read_text()
QDEC_H = (ROOT / "include/coredev/qdec.h").read_text()
TEMP_H = (ROOT / "ARM/Nordic/include/temp_nrf_pei.h").read_text()
RNG_H = (ROOT / "ARM/Nordic/include/rng_nrf_pei.h").read_text()


def section(source, start, end):
    assert source.count(start) == 1, start
    assert source.count(end) == 1, end
    return source.split(start, 1)[1].split(end, 1)[0]


def main():
    q_events = section(QDEC, "static PeiEndpoint_t QdecPeiEventEndpoint(",
                       "static PeiEndpoint_t QdecPeiTaskEndpoint(")
    q_tasks = section(QDEC, "static PeiEndpoint_t QdecPeiTaskEndpoint(",
                      'extern "C" bool QdecPreparePeiStart(')
    for event in ("SAMPLERDY", "REPORTRDY", "ACCOF", "DBLRDY",
                  "STOPPED"):
        assert "&NRF_QDEC->EVENTS_" + event in q_events
    for task in ("START", "STOP"):
        assert "&NRF_QDEC->TASKS_" + task in q_tasks
    assert "READCLRACC" not in q_tasks
    assert "QdecnRFFold(" in QDEC
    assert "NRF_QDEC->TASKS_READCLRACC = 1;" in QDEC
    assert "QDEC_ENABLE_ENABLE_Enabled" in QDEC
    assert "s_pQdecnRFDev" in QDEC
    assert "QdecPreparePeiStart" in QDEC_H

    t_tasks = section(TEMP, "static PeiEndpoint_t TempNrfPeiTaskEndpoint(",
                      'extern "C" bool TempNrfPeiEnableEvent(')
    for task in ("START", "STOP"):
        assert "&NRF_TEMP->TASKS_" + task in t_tasks
    assert "&NRF_TEMP->EVENTS_DATARDY" in TEMP
    assert "NRF_TEMP->INTENCLR = 1U;" in TEMP
    assert "s_bTempPeiReady = true;" in TEMP
    assert "s_bTempPeiReady = false;" in TEMP
    assert "s_pTempnRF5x != NULL" in TEMP

    r_ready = section(RNG, "static bool RngNrfPeiReady(",
                      'extern "C" bool RngNrfPeiPrepare(')
    r_prep = section(RNG, 'extern "C" bool RngNrfPeiPrepare(',
                     'extern "C" bool RngNrfPeiRelease(')
    r_release = section(RNG, 'extern "C" bool RngNrfPeiRelease(',
                        'extern "C" bool RngNrfPeiEnableEvent(')
    assert "RngPeriphSdEnabled()" in r_ready
    assert "!defined(NRFXLIB_SDC)" in r_ready
    assert "atomic_flag_test_and_set(&pDev->bBusy)" in r_prep
    assert "atomic_flag_clear(&s_pPeriphIntrf->bBusy)" in r_release
    assert "reg->EVENTS_VALRDY = 0U;" in r_prep
    assert "s_bPeriphPeiPrepared = true;" in r_prep
    assert "s_bPeriphPeiPrepared = false;" in r_release
    assert "&RngPeriphReg()->EVENTS_VALRDY" in RNG
    assert "&RngPeriphReg()->TASKS_START" in RNG
    assert "&RngPeriphReg()->TASKS_STOP" in RNG
    assert "if (s_bPeriphPeiPrepared)" in RNG
    assert "s_bPeriphSdXfer = RngPeriphSdEnabled();" in RNG

    for source, prefix, header, funcs in (
        (QDEC, "Qdec", QDEC_H,
         ("PreparePeiStart", "EnablePeiEvent", "DisablePeiEvent",
          "ConnectPeiTask", "DisconnectPeiTask")),
        (TEMP, "TempNrfPei", TEMP_H,
         ("EnableEvent", "DisableEvent", "ConnectTask", "DisconnectTask")),
        (RNG, "RngNrfPei", RNG_H,
         ("Prepare", "Release", "EnableEvent", "DisableEvent",
          "ConnectTask", "DisconnectTask"))
    ):
        for name in funcs:
            assert "bool " + prefix + name + "(" in header
            assert 'extern "C" bool ' + prefix + name + "(" in source
        for api in ("PeiBindEvent", "PeiGetEventEndpoint",
                    "PeiUnbindEvent", "PeiConnectTask",
                    "PeiDisconnectTask"):
            assert "#pragma weak " + api in source
    for demo, status in (("pei_qdec_demo.cpp", "g_PeiQdecStatus = 1;"),
                         ("pei_temp_demo.cpp", "g_PeiTempStatus = 1;"),
                         ("pei_rng_demo.cpp", "g_PeiRngStatus = 1;")):
        code = (ROOT / "exemples/pei" / demo).read_text()
        assert status in code
        assert 'extern "C"' not in code
        assert "NRF_QDEC" not in code
        assert "NRF_TEMP" not in code
        assert "NRF_RNG" not in code
    print("nRF52 QDEC/TEMP/RNG PEI source contracts: PASS")


if __name__ == "__main__":
    main()
