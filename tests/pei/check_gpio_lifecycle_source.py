#!/usr/bin/env python3
"""Source regressions for shared nRF52 GPIOTE/PEI resource ownership.

This does not replace a real MCU build or live GPIO/IRQ checks.
"""
import pathlib

ROOT = pathlib.Path(__file__).resolve().parents[2]
GPIO = (ROOT / "ARM/Nordic/src/iopincfg_nrfx.c").read_text(encoding="utf-8")
DEMO = (ROOT / "exemples/pei/pei_gpiote_stress.cpp").read_text(encoding="utf-8")


def section(start, end):
    assert GPIO.count(start) == 1, start
    assert GPIO.count(end) == 1, end
    return GPIO.split(start, 1)[1].split(end, 1)[0]


def main():
    alloc = section("bool IOPinEnableInterrupt(", "// GPIO output task setup")
    choose = section("static int IOPinFindAvailInterrupt(",
                     "int IOPinAllocateInterrupt(")
    disable = section("void IOPinDisableInterrupt(",
                      "bool IOPinEnableInterrupt(")
    task = section("static int IOPinOpenTask(", "static bool IOPinCloseTask(")
    task_decls = task.split("if (pEndpoint == NULL", 1)[0]
    task_scan = task.split("for (i = 0; i < channelCount; i++)", 1)[1]
    assert "uint32_t oldMode;" in task_decls
    assert "uint32_t oldPin;" in task_decls
    assert "uint32_t oldPort;" in task_decls
    assert "uint32_t oldMode =" not in task_scan
    assert "uint32_t oldPin =" not in task_scan
    assert "uint32_t oldPort =" not in task_scan
    irq = GPIO.split("#else\nvoid __WEAK GPIOTE_IRQHandler(void)\n{")[-1]
    irq = irq.split("bool IOPinAssignCore(", 1)[0]

    checks = [
        "irqState = DisableInterrupt();" in alloc,
        "EnableInterrupt(irqState);" in alloc,
        "gpiotereg->CONFIG[IntNo] != 0U" in alloc,
        "otherPort == PortNo && otherPin == PinNo" in alloc,
        "Sense == IOPINSENSE_DISABLE" in alloc,
        "gpiotereg->EVENTS_IN[idx] = 0U;" in alloc,
        "NRF_GPIOTE->CONFIG[i] != 0U" in choose,
        "s_GpIOSenseEvt[i].Sense != IOPINSENSE_DISABLE" in choose,
        "NRF_GPIOTE->CONFIG[IntNo] == 0U" in disable,
        "gpiotereg->EVENTS_IN[IntNo] = 0U;" in disable,
        "s_pGpIOSenseEvt[i].Sense != IOPINSENSE_DISABLE" in irq,
        "s_pGpIOSenseEvt[IOPIN_MAX_INT].Sense != IOPINSENSE_DISABLE" in irq,
        "STRESS_EXHAUST_ROUNDS = 64" in DEMO,
        "STRESS_LIFECYCLE_ROUNDS = 128" in DEMO,
        "IOPinAllocateInterrupt(" in DEMO,
        "IOPinDisableInterrupt(irq);" in DEMO,
        "IOPinDisableTask(pei, evt," in DEMO,
        "IOPinDisableEvent(pei, evt)" in DEMO,
        "pei.ReleaseEvent(evt)" in DEMO,
        "IOPinEnableTask(pei, evt," in DEMO,
        "g_GpioteStress.Status = 2;" in DEMO,
        "printf(" not in DEMO,
        "goto " not in DEMO,
    ]
    assert all(checks), [i for i, ok in enumerate(checks) if not ok]
    print("GPIOTE/PEI resource ownership and stress source source check: PASS")


if __name__ == "__main__":
    main()
