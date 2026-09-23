#!/usr/bin/env python3
"""Guard nRF52 DMA retirement and host-consumed IN completion."""

from pathlib import Path


SOURCE = Path(__file__).parents[2] / "ARM/Nordic/nRF52/src/usb_ctrlr_nrf52.cpp"
HEADER = Path(__file__).parents[2] / "ARM/Nordic/include/usb_ctrlr.h"


def function_body(source: str, signature: str) -> str:
    start = source.index(signature)
    brace = source.index("{", start)
    depth = 0

    for pos in range(brace, len(source)):
        if source[pos] == "{":
            depth += 1
        elif source[pos] == "}":
            depth -= 1
            if depth == 0:
                return source[brace + 1 : pos]

    raise AssertionError("unterminated brace block")


source = SOURCE.read_text(encoding="utf-8")
dma_start = function_body(HEADER.read_text(encoding="utf-8"), "void nRFUsbdDmaStartLocked(")
dma_lock = function_body(source, "void nRFUsbdDmaLock(void)")
dma_finish = function_body(source, "void nRFUsbdDmaWait(void)")
retire = function_body(source, "bool nRFUsbdRetireDma(uint32_t StatusBit)")
interrupt = function_body(source, 'extern "C" void USBD_IRQHandler(void)')
queued = function_body(source, "void nRFUsbdStartQueuedDma(void)")
resume = function_body(source, "void nRFUsbdResumeQueuedDmaLocked(void)")

assert "nRFUsbdDmaReclaim" not in source
assert "nRFUsbdDmaEndIntEnable" not in source
assert "s_LazyInMask" not in source
assert "INTENSET" not in dma_start, "DMA start must not enable ENDEPIN"
assert "NRFX_USBD_EASYDMA_BUSY_REG_BUSY" in dma_lock
assert "NRFX_USBD_EASYDMA_BUSY_REG" not in dma_start
assert "nRFUsbdDmaLock" not in dma_start
assert "*pTask = 1" in dma_start
# Retirement is shared by the ISR default case and the foreground wait
# through nRFUsbdRetireDma; the ordering policy lives in that helper now.
assert "NRF_USBD->EPSTATUS & 0x00FF00FFUL" in dma_finish
assert "nRFUsbdRetireDma(" in dma_finish
assert "nRFUsbdStartQueuedDma" not in dma_finish + retire
assert "nRFUsbdResumeQueuedDma" not in dma_finish + retire
assert "EVENTS_ENDEPOUT[epNum]" in retire
assert "EVENTS_ENDEPIN[epNum]" in retire
assert "nRFUsbdDmaUnlock();" not in retire
assert "nRFUsbdDmaUnlock();" in dma_finish
assert retire.index("if (*pEnd == 0U)") < retire.index("*pEnd = 0U;")
assert retire.index("*pEnd = 0U;") < retire.index("__DSB();")
regular = interrupt[interrupt.index("if ((statusBit & 7U) != 0U) // EP1-7 IN/OUT") :]
regular = regular[:regular.index("if (NRF_USBD->EVENTS_USBEVENT")]
assert regular.index("nRFUsbdRetireDma(statusBit)") < regular.index(
    "nRFUsbEpRegisteredEvent("
)
assert regular.index("nRFUsbEpRegisteredEvent(") < regular.index(
    "nRFUsbdStartQueuedDma();"
)
assert interrupt.index("nRFUsbdStartQueuedDma();") < interrupt.index(
    "NRF_USBD->EVENTS_EPDATA = 0U;"
), "queued DMA must overlap EPDATA/SOF processing, not wait for the ISR tail"
assert "nRFUsbdQueueInComplete" not in regular
assert interrupt.index("const uint32_t inData = dataStatus & 0xFEU;") < interrupt.index(
    "nRFUsbdQueueInComplete("
)
assert "NRF_USBD->EPDATASTATUS" in interrupt
assert "nRFUsbdResumeQueuedDmaLocked();" not in interrupt, "completion must not wait for the ISR tail"
assert interrupt.count("nRFUsbdStartQueuedDma();") == 1
assert "hEp0Que" not in queued + resume
assert "nRFUsbdEp0" not in queued + resume
assert regular.index("nRFUsbdEp0InStart(pEp0)") < regular.index(
    "nRFUsbdStartQueuedDma();"
), "pending EP0 belongs to the ISR handoff, before non-control scheduling"

print("nrf_usb_dma_policy_test: PASS")
