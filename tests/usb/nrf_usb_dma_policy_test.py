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
assert "const uint32_t epStatus = NRF_USBD->EPSTATUS" in interrupt
assert "__CLZ(epStatus)" in interrupt
assert "dmaBit - (dmaBit >> 4U) * 6U" in interrupt
assert "dmaBit == NRFX_USBD_ISO_EP_NO" in interrupt
assert "const bool endPending = *pEndEvent != 0U" in interrupt
assert "dmaBit - 1U < NRFX_USBD_DATA_EP_COUNT - 1U" in interrupt
assert "&NRF_USBD->EVENTS_ENDEPIN[0]" in interrupt
assert "NRF_USBD->EPSTATUS = epStatus" in interrupt
assert interrupt.index("*pEndEvent = 0") < interrupt.index(
    "nRFUsbdDmaRelease();"
)
assert interrupt.index("nRFUsbdDmaRelease();") < interrupt.index(
    "nRFUsbdCollectEvents();"
)
assert interrupt.index("nRFUsbdCollectEvents();") < interrupt.index(
    "NRF_USBD->EPDATASTATUS"
)
assert interrupt.rindex("nRFUsbdServicePending();") > interrupt.index(
    "USBD_INTEN_EPDATA_Msk"
)

for dma_bit, expected in list(enumerate([*range(8), 9])) + list(
    zip(range(16, 25), range(10, 19))
):
    end_offset = dma_bit - (dma_bit >> 4) * 6 + (dma_bit == 8)
    assert end_offset == expected

print("nrf_usb_dma_policy_test: PASS")
