#!/usr/bin/env python3
"""Guard the register-owned nRF52 USB DMA event flow."""

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
handle_in = function_body(source, "static void nRFUsbdHandleInData(uint8_t EpNum)")
handle_out = function_body(source, "static void nRFUsbdHandleOutEnd(uint8_t EpNum)")
open_ep = function_body(source, "static bool nRFUsbRegEpOpen(")
bus_reset = function_body(source, "static void nRFUsbdBusReset(void)")
interrupt = function_body(source, 'extern "C" void USBD_IRQHandler(void)')

assert "nRFUsbdDmaReclaim" not in source
assert "s_DmaEpAddr" not in source
assert "s_DmaRunning" not in source
assert "nRFUsbdDmaWait" not in source
assert "INTENSET" not in dma_start, "DMA start must not enable ENDEPIN"
assert "NRF_USBD->EPSTATUS" in dma_start
assert dma_start.index("NRF_USBD->EPSTATUS") < dma_start.index("*pTask = 1")
assert "NRFX_USBD_EASYDMA_BUSY_REG = NRFX_USBD_EASYDMA_BUSY_REG_BUSY" in dma_start
assert dma_start.index("NRFX_USBD_EASYDMA_BUSY_REG") < dma_start.index("*pTask = 1")
assert "return NRFX_USBD_EASYDMA_BUSY_REG ==" in source
assert "nRFUsbdDmaActive()" in service
assert "NVIC_SetPendingIRQ(USBD_IRQn)" in source
assert "NRF_USBD->EPSTATUS" in abort_ep0
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
assert "USBD_INTEN_ENDISOIN_Msk" in open_ep
assert "USBD_INTEN_ENDISOOUT_Msk" in open_ep
assert "USBD_INTEN_ENDEPOUT0_Pos + epNum" in open_ep
assert "USBD_INTEN_ENDEPIN0_Pos + epNum" not in open_ep
assert "USBD_INTEN_ENDEPIN0_Msk" not in bus_reset
assert "USBD_INTEN_ENDEPOUT0_Msk" in bus_reset
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
assert "NRF_USBD->EPDATASTATUS & dmaStatus" in interrupt[dma_status:ep0]
assert "__CLZ(inStatus)" in interrupt[dma_status:ep0]
assert "__CLZ(outStatus)" in interrupt[dma_status:ep0]
assert "NRF_USBD->EPSTATUS = dmaStatus" in interrupt[dma_status:ep0]
assert "nRFUsbdDmaRelease();" in interrupt[dma_status:ep0]
assert "__CLZ(epin)" in interrupt[dma_status:ep0]
assert "__CLZ(epout)" in interrupt[dma_status:ep0]
assert "else if (nRFUsbdDmaActive())" in interrupt[dma_status:ep0]
assert interrupt.rindex("nRFUsbdServicePending();") > sof

print("nrf_usb_dma_policy_test: PASS")
