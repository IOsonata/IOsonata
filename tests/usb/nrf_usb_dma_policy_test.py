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
dma_wait = function_body(source, "static void nRFUsbdDmaWait(void)")
service = function_body(source, "static void nRFUsbdServicePending(void)")
abort_ep0 = function_body(source, "static void nRFUsbdAbortEp0(void)")
interrupt = function_body(source, 'extern "C" void USBD_IRQHandler(void)')

assert "nRFUsbdDmaReclaim" not in source
assert "nRFUsbdDmaEndIntEnable" not in source
assert "s_LazyInMask" not in source
assert "s_DmaEpAddr" not in source
assert "INTENSET" not in dma_start, "DMA start must not enable ENDEPIN"
assert "NRF_USBD->EPSTATUS" in dma_start
assert dma_start.index("NRF_USBD->EPSTATUS") < dma_start.index("*pTask = 1")
assert "return atomic_flag_test(&s_DmaRunning);" in source
assert "NRF_USBD->EPSTATUS" in dma_wait
assert "__CLZ(epBits)" in dma_wait
assert "NRF_USBD->EPSTATUS" in abort_ep0
assert "nRFUsbdDmaWait" not in abort_ep0
reset = interrupt.index("NRF_USBD->EVENTS_USBRESET")
bus_event = interrupt.index("NRF_USBD->EVENTS_USBEVENT")
dma_status = interrupt.index("const uint32_t dmaStatus")
ep0 = interrupt.index("const bool ep0Setup")
collector = interrupt.index("nRFUsbdCollectEvents()")
assert reset < bus_event < dma_status < ep0 < collector
assert interrupt.index("nRFUsbdBusReset();") < dma_status
assert "EVENTS_ENDEPIN[0]" in interrupt[dma_status:ep0]
assert "EVENTS_ENDEPOUT[0]" in interrupt[dma_status:ep0]
assert "EVENTS_ENDISOIN" in interrupt[dma_status:ep0]
assert "EVENTS_ENDISOOUT" in interrupt[dma_status:ep0]
assert "NRF_USBD->EPDATASTATUS & dmaStatus" in interrupt[dma_status:ep0]
assert "EVENTS_EPDATA" not in interrupt[dma_status:ep0]
assert "__CLZ(inStatus)" in interrupt[dma_status:ep0]
assert "__CLZ(outStatus)" in interrupt[dma_status:ep0]
dma_incomplete = interrupt.index("nRFUsbdDmaActive() && !xferComplete")
dma_dispatch = interrupt.index("if (xferComplete && dmaStatus != 0U)")
assert dma_status < dma_incomplete < dma_dispatch < ep0
assert "NRF_USBD->EPSTATUS = dmaStatus" in interrupt[dma_status:ep0]
assert "NRFX_USBD_EASYDMA_BUSY_REG = NRFX_USBD_EASYDMA_BUSY_REG_FREE" in interrupt[dma_status:ep0]
assert "atomic_flag_clear(&s_DmaRunning)" in interrupt[dma_status:ep0]
assert "nRFUsbdHandleInData(epNum)" in interrupt[dma_status:ep0]
assert "nRFUsbdHandleOutEnd(epNum)" in interrupt[dma_status:ep0]
assert "nRFUsbdHandleInData(0)" in interrupt[dma_status:ep0]
assert "nRFUsbdHandleOutEnd(0)" in interrupt[dma_status:ep0]
assert "nRFUsbdHandleIsoInEnd()" in interrupt[dma_status:ep0]
assert "nRFUsbdHandleIsoOutEnd()" in interrupt[dma_status:ep0]
assert "USBD_INTEN_EPDATA_Msk" in interrupt
assert interrupt.rindex("nRFUsbdServicePending();") > interrupt.index(
    "USBD_INTEN_EPDATA_Msk"
)
assert "nRFUsbdDmaReclaim" not in service

print("nrf_usb_dma_policy_test: PASS")
