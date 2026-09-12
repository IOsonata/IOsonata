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
handle_in = function_body(source, "static void nRFUsbdHandleInData(uint8_t EpNum)")
handle_out = function_body(source, "static void nRFUsbdHandleOutEnd(uint8_t EpNum)")
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
assert "NRF_USBD->ISOIN.AMOUNT" in handle_in
assert "NRF_USBD->EPIN[EpNum].AMOUNT" in handle_in
assert "EpNum != NRFX_USBD_ISO_EP_NO" in handle_in
assert "NRF_USBD->EVENTS_ENDISOIN = 0" in handle_in
assert "NRF_USBD->ISOOUT.AMOUNT" in handle_out
assert "NRF_USBD->EPOUT[EpNum].AMOUNT" in handle_out
assert "EpNum != NRFX_USBD_ISO_EP_NO" in handle_out
assert "NRF_USBD->EVENTS_ENDISOOUT = 0" in handle_out
assert "nRFUsbdHandleIsoInEnd" not in source
assert "nRFUsbdHandleIsoOutEnd" not in source
reset = interrupt.index("NRF_USBD->EVENTS_USBRESET")
bus_event = interrupt.index("NRF_USBD->EVENTS_USBEVENT")
dma_status = interrupt.index("const uint32_t dmaStatus")
ep0 = interrupt.index("const bool ep0Setup")
epdata = interrupt.index("NRF_USBD->EVENTS_EPDATA", ep0)
sof = interrupt.index("NRF_USBD->EVENTS_SOF", epdata)
assert reset < bus_event < dma_status < ep0 < epdata < sof
assert interrupt.index("nRFUsbdBusReset();") < dma_status
assert "nRFUsbdCollectEvents" not in source
assert "NRF_USBD->INTEN &" not in interrupt
assert "EVENTS_ENDEPIN[0]" in interrupt[dma_status:ep0]
assert "EVENTS_ENDEPOUT[0]" in interrupt[dma_status:ep0]
assert "EVENTS_ENDISOIN" in interrupt[dma_status:ep0]
assert "EVENTS_ENDISOOUT" in interrupt[dma_status:ep0]
assert "NRF_USBD->EPDATASTATUS & dmaStatus" in interrupt[dma_status:ep0]
assert "__CLZ(inStatus)" in interrupt[dma_status:ep0]
assert "__CLZ(outStatus)" in interrupt[dma_status:ep0]
assert "NRF_USBD->EPSTATUS = dmaStatus" in interrupt[dma_status:ep0]
assert "NRFX_USBD_EASYDMA_BUSY_REG = NRFX_USBD_EASYDMA_BUSY_REG_FREE" in interrupt[dma_status:ep0]
assert "atomic_flag_clear(&s_DmaRunning)" in interrupt[dma_status:ep0]
assert "const uint32_t epNum = 31U - (uint32_t)__CLZ(epin)" in interrupt[dma_status:ep0]
assert "if (epNum == 0U)" in interrupt[dma_status:ep0]
assert "NRF_USBD->EVENTS_ENDEPIN[0] = 0" in interrupt[dma_status:ep0]
assert "nRFUsbdHandleInData((uint8_t)epNum)" in interrupt[dma_status:ep0]
assert "nRFUsbdHandleOutEnd(31U - (uint32_t)__CLZ(epout))" in interrupt[dma_status:ep0]
assert "if (s_Ctrlr.SetupDirIn)" in interrupt[ep0:epdata]
assert "nRFUsbdHandleInData(0)" in interrupt[ep0:epdata]
assert "EVENTS_EP0DATADONE" not in interrupt[dma_status:interrupt.index("const uint32_t xferStatus")]
assert interrupt.rindex("nRFUsbdServicePending();") > sof
assert "nRFUsbdDmaReclaim" not in service

print("nrf_usb_dma_policy_test: PASS")
