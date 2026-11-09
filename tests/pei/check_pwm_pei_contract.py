#!/usr/bin/env python3
"""nRF52 PWM PEI: register endpoints, ownership and no-app-register invariants.

This is source/API validation only. MCU compilation and waveform observation
remain required to accept the new PeiPwmDemo on BLYST-NANO-DK.
"""
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
SOURCE = (ROOT / "ARM/Nordic/src/pwm_nrfx.cpp").read_text()
HEADER = (ROOT / "include/coredev/pwm.h").read_text()
DEMO = (ROOT / "exemples/pei/pei_pwm_demo.cpp").read_text()


def section(start, end):
    assert SOURCE.count(start) == 1, start
    assert SOURCE.count(end) == 1, end
    return SOURCE.split(start, 1)[1].split(end, 1)[0]


def main():
    events = section("static PeiEndpoint_t PwmPeiEventEndpoint(",
                     "static PeiEndpoint_t PwmPeiTaskEndpoint(")
    tasks = section("static PeiEndpoint_t PwmPeiTaskEndpoint(",
                    "static nRFPwmDev_t *PwmPeiOwnedDevice(")
    ready = section("static bool PwmPeiReady(",
                    'bool PWMEnablePeiEvent(')
    for event in ("SEQSTARTED[0]", "SEQSTARTED[1]", "SEQEND[0]",
                  "SEQEND[1]", "LOOPSDONE", "PWMPERIODEND", "STOPPED"):
        assert "pReg->EVENTS_" + event in events
    for task in ("SEQSTART[0]", "SEQSTART[1]", "STOP", "NEXTSTEP"):
        assert "pReg->TASKS_" + task in tasks
    connect = section('bool PWMConnectTask(',
                      'bool PWMDisconnectTask(')
    assert "PWM_DECODER_MODE_NextStep" in connect
    assert "Task == PWM_PEI_TASK_NEXTSTEP" in connect
    assert "dev->OpenMask == 0U" in ready
    assert "dev->TopCount == 0U" in ready
    assert "dev->pReg->ENABLE != PWM_ENABLE_ENABLE_Enabled" in ready
    assert "dev->pReg->SEQ[0].PTR !=" in ready
    assert "dev->pReg->SEQ[1].PTR !=" in ready
    for name in ("PeiBindEvent", "PeiGetEventEndpoint", "PeiUnbindEvent",
                 "PeiConnectTask", "PeiDisconnectTask"):
        assert "#pragma weak " + name in SOURCE
    for name in ("EnablePeiEvent", "DisablePeiEvent", "ConnectTask",
                 "DisconnectTask"):
        assert "bool PWM" + name + "(" in HEADER
        assert 'bool PWM' + name + "(" in SOURCE
    assert "PWM_PEI_TASK_SEQSTART0" in DEMO
    assert "PWM_PEI_TASK_STOP" in DEMO
    assert "PWM_PEI_EVENT_SEQSTARTED0" in DEMO
    assert "PWM_PEI_EVENT_STOPPED" in DEMO
    assert "g_PeiPwmStatus = 1;" in DEMO
    assert 'extern "C"' not in DEMO
    assert "NRF_PWM" not in DEMO
    assert "printf(" not in DEMO
    assert "goto " not in DEMO
    print("nRF52 PWM PEI register/sequence ownership contract: PASS")


if __name__ == "__main__":
    main()
