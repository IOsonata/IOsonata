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
abort_ep0 = function_body(source, "static void nRFUsbdAbortEp0(void)")
interrupt = function_body(source, 'extern "C" void USBD_IRQHandler(void)')

assert "nRFUsbdDmaReclaim" not in source
assert "nRFUsbdDmaEndIntEnable" not in source
assert "s_LazyInMask" not in source
assert "INTENSET" not in dma_start, "DMA start must not enable ENDEPIN"
assert "atomic_store(&s_DmaEpAddr, EpAddr)" in dma_start
assert "nRFUsbdDmaEndEvent(activeDma)" in interrupt
assert "s_DmaEpAddr" not in abort_ep0
assert "NRF_USBD->EPSTATUS" in abort_ep0
assert "nRFUsbdDmaWait" not in abort_ep0
reset = interrupt.index("NRF_USBD->EVENTS_USBRESET")
ep0 = interrupt.index("const uint32_t ep0Status")
data_dma = interrupt.index("const uint8_t activeDma")
collector = interrupt.index("nRFUsbdCollectEvents()")
assert reset < ep0 < data_dma < collector
assert interrupt.index("nRFUsbdBusReset();") < ep0
assert interrupt.index("nRFUsbdSetupEvent();") < data_dma
assert interrupt.index("*pEndEvent = 0", data_dma) < interrupt.index(
    "nRFUsbdDmaRelease();", data_dma
)
assert interrupt.index("nRFUsbdDmaRelease();") < collector
assert "USBD_INTEN_EPDATA_Msk" in interrupt
assert interrupt.rindex("nRFUsbdServicePending();") > interrupt.index(
    "USBD_INTEN_EPDATA_Msk"
)
assert "nRFUsbdDmaReclaim" not in service

print("nrf_usb_dma_policy_test: PASS")
