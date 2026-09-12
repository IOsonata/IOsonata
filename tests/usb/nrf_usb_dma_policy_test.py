#!/usr/bin/env python3
"""Guard nRF52 IN completion without a second per-packet interrupt."""

from pathlib import Path


SOURCE = Path(__file__).parents[2] / "ARM/Nordic/src/usb_ctrlr_nrfx.cpp"


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
dma_start = function_body(source, "static void nRFUsbdDmaStart(")
service = function_body(source, "static void nRFUsbdServicePending(void)")
interrupt = function_body(source, 'extern "C" void USBD_IRQHandler(void)')

assert "nRFUsbdDmaReclaim" not in source
assert "nRFUsbdDmaEndIntEnable" not in source
assert "s_LazyInMask" not in source
assert "INTENSET" not in dma_start, "DMA start must not enable ENDEPIN"
assert "atomic_store(&s_DmaEpAddr, EpAddr)" in dma_start
assert "NRF_USBD->EPSTATUS & NRF_USBD->EPDATASTATUS" in interrupt
assert "31U - (uint32_t)__CLZ(inEpStatus)" in interrupt
assert "31U - (uint32_t)__CLZ(outEpStatus)" in interrupt
assert "NRF_USBD->EVENTS_ENDEPIN[0]" in interrupt
assert "NRF_USBD->EVENTS_ENDEPOUT[0]" in interrupt
assert "NRF_USBD->EVENTS_ENDISOIN" in interrupt
assert "NRF_USBD->EVENTS_ENDISOOUT" in interrupt
assert interrupt.index("*pEndEvent = 0") < interrupt.index(
    "nRFUsbdDmaRelease();"
)
assert interrupt.index("nRFUsbdDmaRelease();") < interrupt.index(
    "nRFUsbdCollectEvents()"
)
assert "USBD_INTEN_EPDATA_Msk" in interrupt
assert interrupt.rindex("nRFUsbdServicePending();") > interrupt.index(
    "USBD_INTEN_EPDATA_Msk"
)
assert "nRFUsbdDmaReclaim" not in service

print("nrf_usb_dma_policy_test: PASS")
