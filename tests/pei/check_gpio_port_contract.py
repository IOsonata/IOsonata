#!/usr/bin/env python3
"""Check that optional GPIOTE PORT PEI setup is a read-only binding.

It must not claim an IN channel, touch GPIO SENSE, or alter PORT IRQ and
LATCH state. MCU compilation and hardware tests are still required.
"""
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
SOURCE = (ROOT / "ARM/Nordic/src/iopincfg_nrfx.c").read_text(encoding="utf-8")
HEADER = (ROOT / "include/coredev/iopincfg.h").read_text(encoding="utf-8")


def get_body(name, next_name):
    start = "bool " + name + "("
    end = "bool " + next_name + "("
    assert SOURCE.count(start) == 1, name
    assert SOURCE.count(end) == 1, next_name
    return SOURCE.split(start, 1)[1].split(end, 1)[0]


def main():
    enable = get_body("IOPinEnablePortEvent", "IOPinDisablePortEvent")
    disable = get_body("IOPinDisablePortEvent", "IOPinEnableTask")
    for code in (enable, disable):
        assert "&NRF_GPIOTE->EVENTS_PORT" in code
        for forbidden in (
            "PIN_CNF[", "LATCH =", "EVENTS_PORT =", "INTENSET =",
            "INTENCLR =", "CONFIG[", "s_GpIOEventUsed",
            "s_GpIOTaskUsed", "IOPinOpenEvent(", "IOPinCloseEvent("
        ):
            assert forbidden not in code, forbidden
    assert "PeiBindEvent(pPei, pEvt," in enable
    assert "PeiGetEventEndpoint(pPei, Evt)" in disable
    assert "PeiUnbindEvent(pPei, Evt, endpoint)" in disable
    assert "bool IOPinEnablePortEvent(PeiDev_t *pPei" in HEADER
    assert "bool IOPinDisablePortEvent(PeiDev_t *pPei" in HEADER
    assert "static inline bool IOPinEnablePortEvent(Pei &PeiDev" in HEADER
    print("GPIOTE PORT shared source non-invasive binding: PASS")


if __name__ == "__main__":
    main()
