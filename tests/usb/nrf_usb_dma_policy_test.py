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
interrupt = function_body(source, 'extern "C" void USBD_IRQHandler(void)')

assert "s_CtrlrBusy" in source
assert "INTENSET" not in dma_start, "DMA start must not enable ENDEPIN"
assert "const uint32_t epStatus = NRF_USBD->EPSTATUS" in dma_start
assert "NRF_USBD->EPSTATUS = epStatus" in dma_start
assert "epDataPending = NRF_USBD->EVENTS_EPDATA != 0U" in interrupt
assert interrupt.index("NRF_USBD->EVENTS_EPDATA") < interrupt.index(
    "NRF_USBD->EPDATASTATUS"
)
assert interrupt.index("NRF_USBD->EPDATASTATUS") < interrupt.index(
    "NRF_USBD->EPSTATUS"
)
assert "dataDmaStatus = epStatus & dataStatus" in interrupt
assert "__CLZ(dmaStatus)" in interrupt
assert "NRF_USBD->EVENTS_ENDEPIN[dmaEpNum]" in interrupt
assert "NRF_USBD->EVENTS_ENDEPOUT[dmaEpNum]" in interrupt
assert "NRF_USBD->EPSTATUS = epStatus" in interrupt
assert interrupt.index("*pEndEvent = 0") < interrupt.index(
    "nRFUsbdDmaRelease();"
)
assert interrupt.index("nRFUsbdDmaRelease();") < interrupt.index(
    "NRF_USBD->EPDATASTATUS = dataStatus"
)
assert interrupt.rindex("nRFUsbdServicePending();") > interrupt.index(
    "USBD_INTEN_EPDATA_Msk"
)

print("nrf_usb_dma_policy_test: PASS")
