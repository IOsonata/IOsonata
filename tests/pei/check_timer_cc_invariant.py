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
    code = section(src, "static bool " + function + "(", ending)
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
          "// Only installed by TimerPeiPrepareMcu")
    check("ARM/Nordic/src/timer_lf_nrfx.cpp", "nRFxRtcEnablePeiTrigger",
          "// Called only when an application explicitly uses PEI with this RTC.")
    print("TIMER/RTC free-running PEI compare source checks: PASS")


if __name__ == "__main__":
    main()
