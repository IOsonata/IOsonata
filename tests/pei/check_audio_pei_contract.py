#!/usr/bin/env python3
"""Source contracts for nRF52 PDM and I2S hardware PEI endpoints.

Validates the intended driver-owned DMA/IRQ preparation path without
pretending to simulate PDM microphone or I2S transceiver hardware.
"""
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
PDM = (ROOT / "ARM/Nordic/src/pdm_nrfx.cpp").read_text()
I2S = (ROOT / "ARM/Nordic/src/i2s_nrfx.cpp").read_text()
PDM_API = (ROOT / "include/coredev/pdm.h").read_text()
I2S_API = (ROOT / "include/coredev/i2s.h").read_text()


def body(source, beginning, ending):
    assert source.count(beginning) == 1, beginning
    assert source.count(ending) == 1, ending
    return source.split(beginning, 1)[1].split(ending, 1)[0]


def main():
    prepare = body(PDM, 'extern "C" bool PdmPreparePeiStart(',
                   'extern "C" bool PdmEnablePeiEvent(')
    ready = body(PDM, "static bool PdmPeiReady(",
                 'extern "C" bool PdmPreparePeiStart(')
    pdm_events = body(PDM, "static PeiEndpoint_t PdmPeiEventEndpoint(",
                      "static PeiEndpoint_t PdmPeiTaskEndpoint(")
    pdm_tasks = body(PDM, "static PeiEndpoint_t PdmPeiTaskEndpoint(",
                     "static bool PdmPeiReady(")
    for event in ("STARTED", "END", "STOPPED"):
        assert "&NRF_PDM->EVENTS_" + event in pdm_events
    for task in ("START", "STOP"):
        assert "&NRF_PDM->TASKS_" + task in pdm_tasks
    assert "CFifoPut(pDev->hFifo)" in prepare
    assert "NRF_PDM->SAMPLE.PTR =" in prepare
    assert "NRF_PDM->SAMPLE.MAXCNT =" in prepare
    assert "NRF_PDM->TASKS_START" not in prepare
    assert "!pDev->CfgData.bIntEn" in ready
    assert "!pDev->CfgData.bIntEn" in prepare
    assert "NRF_PDM->SAMPLE.PTR == 0U" in ready
    assert "NRF_PDM->SAMPLE.MAXCNT !=" in ready
    assert "NRF_PDM->SAMPLE.PTR != 0U" in prepare
    assert "PdmIrqHandler" in PDM
    assert "NRF_PDM->SAMPLE.PTR = (uint32_t)p;" in PDM
    for name in ("PreparePeiStart", "EnablePeiEvent", "DisablePeiEvent",
                 "ConnectPeiTask", "DisconnectPeiTask"):
        assert "bool Pdm" + name + "(" in PDM_API
        assert 'extern "C" bool Pdm' + name + "(" in PDM

    arm = body(I2S, "static bool nRFxI2SArm(",
               "static void nRFxI2SEnable(")
    prepare_i2s = body(I2S, 'extern "C" bool I2SPreparePeiStart(',
                       'extern "C" bool I2SEnablePeiEvent(')
    i2s_ready = body(I2S, "static bool I2SPeiReady(",
                     'extern "C" bool I2SPreparePeiStart(')
    i2s_events = body(I2S, "static PeiEndpoint_t I2SPeiEventEndpoint(",
                      "static PeiEndpoint_t I2SPeiTaskEndpoint(")
    i2s_tasks = body(I2S, "static PeiEndpoint_t I2SPeiTaskEndpoint(",
                     "static bool I2SPeiReady(")
    for event in ("RXPTRUPD", "TXPTRUPD", "STOPPED"):
        assert "&NRF_AUDIO->EVENTS_" + event in i2s_events
    for task in ("START", "STOP"):
        assert "&NRF_AUDIO->TASKS_" + task in i2s_tasks
    assert "if (StartNow)" in arm
    assert "NRF_AUDIO->TASKS_START = 1U;" in arm
    assert "nRFxI2SArm(pDev, true)" in I2S
    assert "nRFxI2SArm(&pDev->DevIntrf, false)" in prepare_i2s
    assert "atomic_load(&pDev->DevIntrf.EnCnt) != 0" in prepare_i2s
    assert "atomic_store(&pDev->DevIntrf.EnCnt, 1)" in prepare_i2s
    assert "!s_Running || !s_IntEnabled" in i2s_ready
    assert "s_I2S_RxBuffer[0]" in i2s_ready
    assert "s_I2S_RxBuffer[1]" in i2s_ready
    assert "s_I2S_TxBuffer[0]" in i2s_ready
    assert "s_I2S_TxBuffer[1]" in i2s_ready
    assert "NRF_AUDIO->RXTXD.MAXCNT != s_BlockWords" in i2s_ready
    assert "I2S_PEI_EVENT_RXPTRUPD" in I2S
    assert "NRF_AUDIO->CONFIG.RXEN == 0U" in I2S
    assert "NRF_AUDIO->CONFIG.TXEN == 0U" in I2S
    assert "nRFxI2SService(void)" in I2S
    for name in ("PreparePeiStart", "EnablePeiEvent", "DisablePeiEvent",
                 "ConnectPeiTask", "DisconnectPeiTask"):
        assert "bool I2S" + name + "(" in I2S_API
        assert 'extern "C" bool I2S' + name + "(" in I2S

    for source in (PDM, I2S):
        for name in ("PeiBindEvent", "PeiGetEventEndpoint",
                     "PeiUnbindEvent", "PeiConnectTask",
                     "PeiDisconnectTask"):
            assert "#pragma weak " + name in source
        assert "goto " not in source
    print("nRF52 PDM/I2S PEI DMA/IRQ ownership source contracts: PASS")


if __name__ == "__main__":
    main()
