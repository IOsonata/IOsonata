#!/usr/bin/env python3
"""Source-level regression: PEI compare never stops or resets TIMER/RTC.

This complements (but does not replace) MCU compilation and on-board tests.
"""
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]


def section(source, begin, end):
    assert source.count(begin) == 1, begin
    assert source.count(end) >= 1, end
    return source.split(begin, 1)[1].split(end, 1)[0]


def check(path, function, ending):
    src = (ROOT / path).read_text(encoding="utf-8")
    code = section(src, "bool " + function + "(", ending)
    assert "PeiBindEvent(" in code, path
    assert "EVENTS_COMPARE[" in code, path
    assert "INTENSET" in code, path
    assert "TASKS_STOP =" not in code, path
    assert "TASKS_CLEAR =" not in code, path
    assert "PeiConnectTask(" not in code, path
    assert "SHORTS |=" not in code, path
    assert "SHORTS &=" not in code, path
    assert "TIMER_TRIG_PEI_CONTINUOUS" in code, path
    assert "TIMER_TRIG_PEI_SINGLE" in code, path
    assert "for (i = 0; i < pData->MaxNbTrigEvt;" not in code, path

    irq = section(src, "static void " +
                  ("TimerIRQHandler" if "timer_hf_" in path else "RtcIRQHandler") +
                  "(", "extern \"C\"")
    assert "TIMER_TRIG_PEI_CONTINUOUS" in irq, path
    assert "reg->CC[i] = " in irq, path
    assert "TASKS_STOP =" not in irq, path
    assert "TASKS_CLEAR =" not in irq, path

    if "timer_lf_" in path:
        assert "nRFxRtcIsPeiTrigger" not in src, path
        assert "nRFxRtcPeiBusy" not in src, path


def main():
    check("ARM/Nordic/src/timer_hf_nrfx.cpp", "nRFxTimerEnablePeiTrigger",
          "static uint64_t nRFxTimerEnableTrigger(")
    check("ARM/Nordic/src/timer_lf_nrfx.cpp", "nRFxRtcEnablePeiTrigger",
          "static uint64_t nRFxRtcEnableTrigger(")
    hf = (ROOT / "ARM/Nordic/src/timer_hf_nrfx.cpp").read_text(encoding="utf-8")
    hf_release = section(hf, "static void nRFxTimerDisableTrigger(",
                         "static int nRFxTimerFindAvailTrigger(")
    hf_unbind = hf_release.index("!PeiUnbindEvent(pei, evt, endpoint)")
    hf_disable = hf_release.index("reg->INTENCLR =")
    assert hf_unbind < hf_disable, "HF TIMER must unbind before disabling compare"

    rtc = (ROOT / "ARM/Nordic/src/timer_lf_nrfx.cpp").read_text(encoding="utf-8")
    release = section(rtc, "bool nRFxRtcDisablePeiEvent(",
                      "bool nRFxRtcEnablePeiTrigger(")
    compare = section(release, "endpoint = (PeiEndpoint_t)&reg->EVENTS_COMPARE[i];",
                      "reg->CC[i] = 0U;")
    assert compare.index("PeiUnbindEvent(pPei, Event, endpoint)") < compare.index(
        "reg->INTENCLR ="), "RTC unbind must precede disabling compare interrupt"
    assert compare.index("PeiUnbindEvent(pPei, Event, endpoint)") < compare.index(
        "reg->EVTENCLR ="), "RTC unbind must precede disabling compare event"

    timer_driver = (ROOT / "ARM/Nordic/src/timer_nrfx.cpp").read_text(
        encoding="utf-8")
    matrix = (ROOT / "src/coredev/pei.c").read_text(encoding="utf-8")
    timer_api = (ROOT / "include/coredev/timer.h").read_text(
        encoding="utf-8")
    assert "PeiTimerPrepare" not in timer_driver
    assert "PeiTimerPrepare" not in timer_api
    assert "PeiTimerPrepare" not in matrix
    assert "TimerPeiPrepareMcu" not in timer_driver
    assert "TimerPeiPrepareMcu" not in matrix
    assert "nRFxTimerInstallPei" not in hf
    assert "nRFxRtcInstallPei" not in rtc
    assert "bool nRFxTimerEnablePeiTrigger(" in hf
    assert "bool nRFxRtcEnablePeiTrigger(" in rtc
    assert "bool nRFxRtcEnablePeiEvent(" in rtc
    assert "bool nRFxRtcDisablePeiEvent(" in rtc
    for name in ("EnablePeiTrigger", "EnablePeiEvent", "DisablePeiEvent"):
        assert "(*" + name + ")" not in timer_api
    for name in ("EnablePeiTrigger", "EnablePeiEvent", "DisablePeiEvent"):
        assert "pTimer->" + name + " =" not in hf
        assert "pTimer->" + name + " =" not in rtc
    assert "int TimerEnablePeiTrigger(" in timer_driver
    assert "bool TimerEnablePeiEvent(" in timer_driver
    assert "bool TimerDisablePeiEvent(" in timer_driver
    assert "!PeiTimerPrepare(" not in timer_api

    print("TIMER/RTC free-running PEI compare source checks: PASS")


if __name__ == "__main__":
    main()
