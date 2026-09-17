#!/usr/bin/env python3
"""Guard nRF52 IN completion without a second per-packet interrupt."""

from pathlib import Path


SOURCE = Path(__file__).parents[2] / "ARM/Nordic/src/usb_ctrlr_nrf52.cpp"


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
dma_start = function_body(source, "void nRFUsbdDmaStartLocked(")
dma_finish = function_body(source, "uint8_t nRFUsbdDmaFinishLocked(")
interrupt = function_body(source, 'extern "C" void USBD_IRQHandler(void)')

assert "nRFUsbdDmaReclaim" not in source
assert "nRFUsbdDmaEndIntEnable" not in source
assert "s_LazyInMask" not in source
assert "INTENSET" not in dma_start, "DMA start must not enable ENDEPIN"
assert "NRFX_USBD_EASYDMA_BUSY_REG_BUSY" in dma_start
assert "*pTask = 1" in dma_start
assert "DmaStatus & 0x00FF00FFUL" in dma_finish
assert "EVENTS_ENDEPOUT[epNum]" in dma_finish
assert "EVENTS_ENDEPIN[epNum]" in dma_finish
assert "nRFUsbdDmaUnlock();" in dma_finish
assert interrupt.index("nRFUsbdDmaFinishLocked(") < interrupt.index(
    "nRFUsbdStartQueuedDma();"
)
assert "NRF_USBD->EPDATASTATUS" in interrupt
assert interrupt.rindex("nRFUsbdResumeQueuedDmaLocked();") > interrupt.index(
    "NRF_USBD->EPDATASTATUS"
)

print("nrf_usb_dma_policy_test: PASS")
