#!/usr/bin/env python3
"""SAADC PEI safety invariant checks against the source and hardware example.

MCU compile and live measurements remain necessary.
"""
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
SOURCE = (ROOT / "ARM/Nordic/nRF52/src/adc_nrf52_saadc.cpp").read_text()
HEADER = (ROOT / "ARM/Nordic/include/adc_nrf52_saadc.h").read_text()
C_HEADER = (ROOT / "ARM/Nordic/include/adc_nrf52_pei.h").read_text()
EXAMPLE = (ROOT / "exemples/pei/pei_saadc_demo.cpp").read_text()


def section(start, end):
    assert SOURCE.count(start) == 1, start
    assert SOURCE.count(end) == 1, end
    return SOURCE.split(start, 1)[1].split(end, 1)[0]


def main():
    prepare = section("bool AdcnRF52::PreparePeiSampling()",
                      "static PeiEndpoint_t AdcNrf52PeiEventEndpoint(")
    ready = section("bool AdcnRF52::PeiSamplingReady() const",
                    "bool AdcnRF52::PreparePeiSampling()")
    event = section("static PeiEndpoint_t AdcNrf52PeiEventEndpoint(",
                    "static PeiEndpoint_t AdcNrf52PeiTaskEndpoint(")
    task = section("static PeiEndpoint_t AdcNrf52PeiTaskEndpoint(",
                   'bool AdcNrf52PreparePeiSampling(')
    assert "s_AdcnRF52DevData.ResData" in prepare
    assert "NRF_SAADC->RESULT.PTR =" in prepare
    assert "NRF_SAADC->RESULT.MAXCNT =" in prepare
    assert "NRF_SAADC->TASKS_START = 1U;" in prepare
    assert "NRF_SAADC->TASKS_SAMPLE = 1U;" not in prepare
    assert "SAADC_CH_CONFIG_TACQ_5us" in prepare
    assert "NRF52832_XXAA" in prepare
    assert "vMode != ADC_CONV_MODE_SINGLE || vbInterrupt" in prepare
    assert "vMode != ADC_CONV_MODE_SINGLE || vbInterrupt" in ready
    assert "NRF_SAADC->RESULT.PTR !=" in ready
    assert "NRF_SAADC->RESULT.MAXCNT !=" in ready
    for name in ("STARTED", "END", "DONE", "RESULTDONE", "STOPPED"):
        assert "&NRF_SAADC->EVENTS_" + name in event
    for name in ("START", "SAMPLE", "STOP"):
        assert "&NRF_SAADC->TASKS_" + name in task
    assert "PeiBindEvent(pPei, pEvt, endpoint)" in SOURCE
    assert "PeiConnectTask(pPei, Evt, endpoint)" in SOURCE
    assert "PeiUnbindEvent(pPei, Evt, endpoint)" in SOURCE
    assert "PeiDisconnectTask(pPei, Evt, endpoint)" in SOURCE
    assert "bool PreparePeiSampling();" in HEADER
    assert "AdcNrf52EnablePeiEvent" in C_HEADER
    assert "AdcNrf52ConnectTask" in C_HEADER
    assert "adc.ConnectTask(pei, compareEvt, ADC_NRF52_PEI_TASK_SAMPLE)" in EXAMPLE
    assert "adc.ConnectTask(pei, endEvt, ADC_NRF52_PEI_TASK_START)" in EXAMPLE
    assert "adc.EnablePeiEvent(pei, endEvt, ADC_NRF52_PEI_EVENT_END)" in EXAMPLE
    assert "g_PeiSaadcStatus = 1;" in EXAMPLE
    assert 'extern "C"' not in EXAMPLE
    assert "NRF_SAADC->" not in EXAMPLE
    assert "printf(" not in EXAMPLE
    print("nRF52 SAADC PEI preparation and endpoint source checks: PASS")


if __name__ == "__main__":
    main()
