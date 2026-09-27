#!/usr/bin/env python3
"""Guard nRF52 EasyDMA ownership, retirement and hand-off.

The USBD has one EasyDMA channel and no START task may be triggered before
the running transfer's END event. The controller models this with the
errata-199 busy word as a software lock:

- Lock is taken before a start, released only when nothing follows.
- A completed transfer is retired from its END event (EPSTATUS cleared after
  the event), and the channel is handed straight to the next queued DMA
  inside the interrupt, before the EPDATA and bus event work.
- EP0 IN retires only once the host has taken the packet (EP0DATADONE).
- Regular IN completion to the class rides EPDATA (host consumed), queued
  to AppEvt; OUT completion is reported as soon as its DMA ends.
- The foreground wait retires a running DMA and unlocks; it never starts
  another.
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
dma_wait = function_body(source, "void nRFUsbdDmaWait(void)")
completed = function_body(source, "static int nRFUsbdGetCompletedXfer(void)")
interrupt = function_body(source, 'extern "C" void USBD_IRQHandler(void)')
queued = function_body(source, "void nRFUsbdStartQueuedDma(void)")
resume = function_body(source, "void nRFUsbdResumeQueuedDmaLocked(void)")
ep_enable = function_body(source, "void nRFUsbdEpHwEnable(")

# Removed mechanisms stay removed.
assert "nRFUsbdDmaReclaim" not in source
assert "nRFUsbdDmaEndIntEnable" not in source
assert "s_LazyInMask" not in source

# Start: clear END, trigger. It neither takes the lock nor touches INTEN;
# END interrupts are enabled once, when the endpoint opens.
assert "*pEnd = 0U;" in dma_start and "*pTask = 1U;" in dma_start
assert dma_start.index("*pEnd = 0U;") < dma_start.index("*pTask = 1U;")
assert "INTENSET" not in dma_start
assert "NRFX_USBD_EASYDMA_BUSY_REG" not in dma_start
assert "USBD_INTEN_ENDEPIN0_Msk" in ep_enable
assert "USBD_INTEN_ENDEPOUT0_Msk" in ep_enable

# The lock is the busy word.
assert "NRFX_USBD_EASYDMA_BUSY_REG_BUSY" in dma_lock
assert "NRFX_USBD_EASYDMA_BUSY_REG_CLEAR" in dma_unlock

# Retire: END event cleared before EPSTATUS, EPSTATUS before the barrier.
# Every transfer but EP0 IN takes the generic path after the switch.
generic = completed[completed.index("if (*pend != 0U)"):]
assert generic.index("*pend = 0U;") < generic.index("NRF_USBD->EPSTATUS = dmastatus;")
assert generic.index("NRF_USBD->EPSTATUS = dmastatus;") < generic.index("__DSB();")
# EP0 IN is retired only after the host took the packet, with the same
# event-before-status order.
ep0 = completed[completed.index("case 0U:"):completed.index("case 16U:")]
assert "EVENTS_ENDEPIN[0] == 0U" in ep0 and "EVENTS_EP0DATADONE == 0U" in ep0
assert "return -1;" in ep0
assert ep0.index("NRF_USBD->EVENTS_ENDEPIN[0] = 0U;") < ep0.index("NRF_USBD->EPSTATUS = dmastatus;")
assert ep0.index("NRF_USBD->EPSTATUS = dmastatus;") < ep0.index("__DSB();")
assert "CFifoGet(s_Usbd.hEp0Que)" in ep0
# Retirement never starts anything and never unlocks.
assert "nRFUsbdStartQueuedDma" not in completed
assert "nRFUsbdDmaUnlock" not in completed

# Foreground wait: retires under interrupt exclusion, unlocks, starts nothing.
assert "__disable_irq();" in dma_wait
assert "nRFUsbdDmaUnlock();" in dma_wait
assert "CFifoGet(s_Usbd.hQue)" in dma_wait
assert "nRFUsbdStartQueuedDma" not in dma_wait
assert "nRFUsbdResumeQueuedDma" not in dma_wait

# Interrupt: a completed regular transfer is popped and, for OUT, reported
# before the channel is handed on; the hand-off comes before EPDATA and
# before the bus event work, so the next DMA overlaps that processing.
hand_off = interrupt.index("nRFUsbdStartQueuedDma();")
assert interrupt.index("(void)CFifoGet(s_Usbd.hQue);") < hand_off
assert interrupt.index("nRFUsbEpRegisteredEvent(epNum, 0U,") < hand_off
assert hand_off < interrupt.index("NRF_USBD->EVENTS_EPDATA = 0U;")
assert hand_off < interrupt.index("if (NRF_USBD->EVENTS_USBEVENT != 0U)\n\t{\n\t\tNRF_USBD->EVENTS_USBEVENT = 0U;")
assert interrupt.count("nRFUsbdStartQueuedDma();") == 1
# The tail only picks up work discovered after the hand-off.
assert interrupt.count("nRFUsbdResumeQueuedDmaLocked();") == 1
assert interrupt.index("if (newDmaWork)") < interrupt.index("nRFUsbdResumeQueuedDmaLocked();")
assert interrupt.index("NRF_USBD->EPDATASTATUS = servicedStatus;") < interrupt.index(
    "nRFUsbdResumeQueuedDmaLocked();"
)
# A bus event that held back the handoff restarts the queue at the tail
# once handled; the resume itself is gated by the suspend state.
bus = interrupt[interrupt.index("if (NRF_USBD->EVENTS_USBEVENT != 0U)\n\t{\n\t\tNRF_USBD->EVENTS_USBEVENT = 0U;"):]
bus = bus[:bus.index("\n\t}\n") + 3]
assert bus.index("nRFUsbdHandleBusEvent(eventCause);") < bus.index("newDmaWork = true;")
assert "!nRFUsbdDmaAllowed()" in resume

# Regular IN completion goes through EPDATA to AppEvt.
assert interrupt.index("NRF_USBD->EVENTS_EPDATA = 0U;") < interrupt.index(
    "nRFUsbdQueueInComplete("
)

# Scheduler: one place decides the order; it ends by releasing the lock
# when there is nothing to start.
assert queued.rstrip().endswith("nRFUsbdDmaUnlock();")
iso = queued.index("nRFUsbdIsoStart()")
ep0_out = queued.index("EVENTS_EP0DATADONE")
ep0_in = queued.index("CFifoPeek(s_Usbd.hEp0Que)")
regular = queued.index("CFifoPeek(s_Usbd.hQue)")
assert iso < ep0_out < ep0_in < regular

# Resume from foreground: takes an idle channel only.
assert "nRFUsbdDmaActive()" in resume and "nRFUsbdDmaLock();" in resume
assert resume.index("nRFUsbdDmaActive()") < resume.index("nRFUsbdDmaLock();")

print("nrf_usb_dma_policy_test: PASS")
