#!/usr/bin/env python3
"""Guard nRF52 EasyDMA ownership, retirement and hand-off.

The USBD has one EasyDMA channel and no START task may be triggered before
the running transfer's END event. The controller models this with the
errata-199 busy word as a software lock:

- Lock is taken before a start, released only when nothing follows.
- A completed transfer is retired in the ISR endpoint switch from its END
  event (END cleared, then EPSTATUS), and the channel is handed straight to
  the next queued DMA before the EPDATA scan.
- EP0 IN chains its next staged packet directly from its END.
- Regular OUT completion is queued to AppEvt at END. Regular IN completion
  and OUT readiness (DRDY) are queued to AppEvt from EPDATA; the readiness
  latch is consumed only when the enqueue succeeds.
- EpReceive only queues the destination and resumes an idle channel.
- The close-path wait retires a running DMA and unlocks; it never starts
  another. Its callers exclude interrupts.
"""

from pathlib import Path


SOURCE = Path(__file__).parents[2] / "ARM/Nordic/nRF52/src/usb_ctrlr_nrf52.cpp"


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
dma_lock = function_body(source, "void nRFUsbdDmaLock(void)")
dma_unlock = function_body(source, "void nRFUsbdDmaUnlock(void)")
dma_wait = function_body(source, "static void nRFUsbdDmaWait(uint32_t mask)")
acquire = function_body(source, "bool nRFUsbdAcquireDma(void)")
interrupt = function_body(source, 'extern "C" void USBD_IRQHandler(void)')
completed = function_body(interrupt, "switch (epno)")
queued = function_body(source, "void nRFUsbdStartQueuedDma(bool ep0out)")
resume = function_body(source, "void nRFUsbdResumeQueuedDmaLocked(void)")
receive = function_body(source, "bool UsbCtrlrEpReceive(")
ep_close = function_body(source, "void UsbCtrlrEpClose(")
close_all = function_body(source, "void UsbCtrlrEpCloseAll(")
ep_enable = function_body(source, "void nRFUsbdEpHwEnable(")

# Removed mechanisms stay removed.
for name in ("nRFUsbdDmaReclaim", "nRFUsbdDmaEndIntEnable", "s_LazyInMask",
             "nRFUsbdGetCompletedXfer", "nRFUsbdProcessInComplete",
             "nRFUsbdProcessEpEvent", "newDmaWork"):
    assert name not in source, name

# Start: clear END, trigger. It neither takes the lock nor touches INTEN;
# END interrupts are enabled once, when the endpoint opens.
assert "*pEnd = 0U;" in dma_start and "*pTask = 1U;" in dma_start
assert dma_start.index("*pEnd = 0U;") < dma_start.index("*pTask = 1U;")
assert "INTENSET" not in dma_start
assert "NRFX_USBD_EASYDMA_BUSY_REG" not in dma_start
assert "USBD_INTEN_ENDEPIN0_Msk" in ep_enable
assert "USBD_INTEN_ENDEPOUT0_Msk" in ep_enable

# The lock is the busy word. Acquisition checks busy and suspend first.
assert "NRFX_USBD_EASYDMA_BUSY_REG_BUSY" in dma_lock
assert "NRFX_USBD_EASYDMA_BUSY_REG_CLEAR" in dma_unlock
assert "nRFUsbdDmaActive()" in acquire and "!nRFUsbdDmaAllowed()" in acquire
assert acquire.index("nRFUsbdDmaActive()") < acquire.index("nRFUsbdDmaLock();")
assert "nRFUsbdAcquireDma()" in resume
assert resume.index("nRFUsbdAcquireDma()") < resume.index("nRFUsbdStartQueuedDma(false);")

# The ISR decodes the active DMA once and retires it in the endpoint case.
# Each case clears its END event before EPSTATUS and returns nothing.
assert interrupt.count("switch (epno)") == 1
assert interrupt.index("if (nRFUsbdDmaActive() && dmastatus != 0U)") < \
    interrupt.index("switch (epno)")
cases = (
    ("case 0U:", "case 16U:", "NRF_USBD->EVENTS_ENDEPIN[0] = 0U;"),
    ("case 16U:", "case 8U:", "NRF_USBD->EVENTS_ENDEPOUT[0] = 0U;"),
    ("case 8U:", "case 24U:", "NRF_USBD->EVENTS_ENDISOIN = 0U;"),
    ("case 24U:", "case 1U:", "NRF_USBD->EVENTS_ENDISOOUT = 0U;"),
    ("case 1U:", "case 17U:", "NRF_USBD->EVENTS_ENDEPIN[epno] = 0U;"),
    ("case 17U:", None, "NRF_USBD->EVENTS_ENDEPOUT[epnum] = 0U;"),
)
for marker, following, end_clear in cases:
    part = completed[completed.index(marker):]
    if following:
        part = part[:part.index(following)]
    assert part.index(end_clear) < part.index("NRF_USBD->EPSTATUS = dmastatus;"), marker
    assert "reuseDma = true;" in part, marker
    assert "return" not in part, marker

# EP0 IN pops its packet and chains the next one while holding the channel.
ep0 = completed[completed.index("case 0U:"):completed.index("case 16U:")]
assert ep0.index("CFifoGet(s_Usbd.hEp0Que)") < ep0.index("nRFUsbdEp0InStart(pep0);")
assert ep0.index("nRFUsbdEp0InStart(pep0);") < ep0.index("reuseDma = false;")

# Regular IN pops its request; completion is reported later from EPDATA.
reg_in = completed[completed.index("case 1U:"):completed.index("case 17U:")]
assert "(void)CFifoGet(s_Usbd.hQue);" in reg_in
assert "UsbEvtQue" not in reg_in

# Regular OUT pops its request and queues its completion at END.
reg_out = completed[completed.index("case 17U:"):]
assert reg_out.index("(void)CFifoGet(s_Usbd.hQue);") < reg_out.index("UsbEvtQue(evt,")
assert "nRFUsbdProcessQueuedEvent" in reg_out
assert "USB_CTRLR_EVT_XFER_CMPL" not in completed
assert "nRFUsbdStartQueuedDma" not in completed
assert "nRFUsbdDmaUnlock" not in completed

# Hand-off: one scheduler call, after retirement and before the EPDATA scan.
assert interrupt.count("nRFUsbdStartQueuedDma(") == 1
hand_off = interrupt.index("nRFUsbdStartQueuedDma(ep0out);")
assert interrupt.index("switch (epno)") < hand_off
assert hand_off < interrupt.index("uint32_t pending = __ROR(datastatus & 0x00FE00FEUL, 16U);")
assert "if (reuseDma || ((ep0out || resumed) && nRFUsbdAcquireDma()))" in interrupt
assert interrupt.index("NRF_USBD->EPDATASTATUS = servicedstatus;") > hand_off

# Bus events are handled before retirement; a resume falls through to the
# scheduler so queued work restarts without another event.
bus = interrupt.index("if (NRF_USBD->EVENTS_USBEVENT != 0U)")
assert bus < interrupt.index("switch (epno)")
assert "resumed" in interrupt[hand_off - 200:hand_off]

# EPDATA: IN completion and OUT DRDY go through AppEvt. The OUT readiness
# latch is consumed only when the enqueue succeeds; IN stops on a full queue.
epdata = interrupt[interrupt.index("uint32_t pending = __ROR(datastatus & 0x00FE00FEUL, 16U);"):]
# One pass in the original order: IN highest endpoint first, then OUT.
assert "31U - (uint32_t)__CLZ(pending)" in epdata
assert "evt = (NRF_USBD->EPIN[epnum].AMOUNT << 8U) | epnum | (1UL << 16U);" in epdata
assert "uint32_t evt = epnum | (1UL << 17U);" in epdata
# OUT readiness waits while its DMA is captured, and is consumed only when
# the enqueue succeeds; a full queue stops the pass with the bit latched.
assert epdata.index("(NRF_USBD->EPSTATUS & bit) != 0U") < epdata.index("UsbEvtQue(evt,")
assert epdata.index("UsbEvtQue(evt,") < epdata.index("servicedstatus |= bit;")
assert "break;" in epdata
assert "CFifoPut" not in epdata

# EpReceive queues the destination and resumes an idle channel. The ISR
# already consumed readiness, so it does not touch EPDATASTATUS.
assert "EPDATASTATUS" not in receive
assert receive.index("CFifoPut(s_Usbd.hQue)") < receive.index("pQue->pBuffer = pBuffer;")
assert receive.index("pQue->pBuffer = pBuffer;") < receive.index("nRFUsbdResumeQueuedDmaLocked();")
assert "DisableInterrupt()" in receive

# Close-path wait: retires, pops a regular request, unlocks, starts nothing.
# Its callers hold the interrupt exclusion.
assert "__disable_irq" not in dma_wait and "DisableInterrupt" not in dma_wait
assert "nRFUsbdDmaUnlock();" in dma_wait
assert "CFifoGet(s_Usbd.hQue)" in dma_wait
assert "nRFUsbdStartQueuedDma" not in dma_wait
assert "nRFUsbdResumeQueuedDma" not in dma_wait
for body in (ep_close, close_all):
    assert body.index("DisableInterrupt()") < body.index("nRFUsbdDmaWait(")
    assert body.index("nRFUsbdDmaWait(") < body.index("EnableInterrupt(state);")

# Scheduler: one place decides the order; stale requests of a closed
# endpoint are skipped; it ends by releasing the lock when nothing starts.
assert queued.rstrip().endswith("nRFUsbdDmaUnlock();")
iso = queued.index("nRFUsbdIsoStart()")
ep0_out = queued.index("if (ep0out)")
ep0_in = queued.index("CFifoPeek(s_Usbd.hEp0Que)")
regular = queued.index("CFifoPeek(s_Usbd.hQue)")
assert iso < ep0_out < ep0_in < regular
assert "pQue->EpNum == 0U" in queued

print("nrf_usb_dma_policy_test: PASS")
