#!/usr/bin/env python3
"""Guard nRF52 endpoint-8 ISO transport and core-owned SOF scheduling.

Source-level checks of the rules the ISO path depends on:

- The controller owns endpoint 8, its DMA scheduling and its OUT buffer,
  sized by the hardware (512 bytes with ISOSPLIT HalfIN).
- SOF is a bus event handed to the core; the controller does no ISO work on
  it. The core forwards it to the endpoint, and the ISO class decides what
  to offer through UsbCtrlrIsoSend.
- ISO is ahead of EP0 and of the regular queue in the DMA scheduler, and IN
  is staged before OUT (the IN data must be in place before the host's
  token; OUT only needs reading before the next SOF).
- UsbIntrf is generic: it has no ISO knowledge. The ISO class registers its
  own IN endpoint callback and leaves the event callback to the application.
"""

from pathlib import Path


ROOT = Path(__file__).parents[2]
HEADER = ROOT / "ARM/Nordic/include/usb_ctrlr.h"
BASE = ROOT / "ARM/Nordic/nRF52/src/usb_ctrlr_nrf52.cpp"
ISO = ROOT / "ARM/Nordic/nRF52/src/usb_ctrlr_nrf52_iso.cpp"
CORE = ROOT / "src/usb/usb.cpp"
INTRF = ROOT / "src/usb/usb_intrf.cpp"
ISO_INTRF = ROOT / "src/usb/usb_iso.cpp"


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


header = HEADER.read_text(encoding="utf-8")
base = BASE.read_text(encoding="utf-8")
iso = ISO.read_text(encoding="utf-8")
core = CORE.read_text(encoding="utf-8")
intrf = INTRF.read_text(encoding="utf-8")
iso_intrf = ISO_INTRF.read_text(encoding="utf-8")

start_iso = function_body(iso, "bool nRFUsbdIsoStart(void)")
iso_send = function_body(iso, "bool UsbCtrlrIsoSend(")
iso_complete = function_body(iso, "void nRFUsbdIsoComplete(")
iso_open = function_body(iso, "bool UsbCtrlrIsoOpen(")
iso_close = function_body(iso, "void nRFUsbdIsoEpClose(")
interrupt = function_body(base, 'extern "C" void USBD_IRQHandler(void)')
handle_sof = function_body(base, "static void nRFUsbdHandleSof(void)")
queued = function_body(base, "void nRFUsbdStartQueuedDma(void)")
completed = function_body(interrupt, "if (nRFUsbdDmaActive())")
process = function_body(core, "void UsbDevProcessEvent(")
update_sof = function_body(core, "static void UsbCoreUpdateSof(void)")

# Target description: endpoint 8 is the ISO pair, 512-byte packets.
assert "USB_EPIN_CNT_0 = 8" in header and "USB_EPOUT_CNT_0 = 8" in header
assert "USB_CTRLR0_ISO_PKT_LEN_MAX = 512" in header
assert "USB_ISO_EPIN_MASK_0 = (1U << 8)" in header
assert "USB_ISO_EPOUT_MASK_0 = (1U << 8)" in header
assert "USB_CTRLR_ISO_INIT(DevNo) UsbCtrlrIsoInit(DevNo)" in header
assert "NRF_USB_EP_COUNT = 9" in header
assert "NRFX_USBD_ISO_MAX_PACKET_SIZE = 512" in header

# Controller API: one send entry for the service interval. OUT lands in
# the destination submitted with EpReceive (the reserved RX FIFO block).
# The existing ISO scheduler owns the transfer deadline.
assert "UsbCtrlrIsoSend" in header
assert "UsbCtrlrIsoRxBuffer" not in header + base + iso + iso_intrf
assert "s_IsoRxBuffer" not in iso
assert "UsbCtrlrEpOutXfer" not in header + base + iso + iso_intrf
assert "UsbCtrlrIsoService" not in header + base + iso + iso_intrf

# ISO work is tracked as two ready flags; the scheduler starts it.
assert "NRFUSBD_ISO_IN_READY" in header and "NRFUSBD_ISO_OUT_READY" in header
assert "uint8_t IsoDataFlag;" in header
assert "IsoBusy" not in header + base + iso
assert "IsoBufState" not in header + base + iso

# OUT lands in the submitted destination; without one the owner is
# asked for one (DRDY) and the frame is dropped if it still has none.
assert "ISOOUT.PTR = (uint32_t)(uintptr_t)s_Usbd.pIsoBuffer[0]" in start_iso
assert start_iso.index("USB_CTRLR_EVT_DRDY") < start_iso.index("TASKS_STARTISOOUT")
assert "s_Usbd.pIsoBuffer[0] == NULL)" in start_iso
assert "MaxPacketSize > NRFX_USBD_ISO_MAX_PACKET_SIZE" in iso_open
assert "USBD_ISOSPLIT_SPLIT_HalfIN" in iso_open
assert "USBD_ISOINCONFIG_RESPONSE_ZeroData" in iso_open

# Start: IN first, OUT size read only once OUT is the selected transfer.
assert "TASKS_STARTISOIN" in start_iso and "TASKS_STARTISOOUT" in start_iso
assert start_iso.index("TASKS_STARTISOIN") < start_iso.index("NRF_USBD->SIZE.ISOOUT")
assert start_iso.index("NRF_USBD->SIZE.ISOOUT") < start_iso.index("TASKS_STARTISOOUT")

# Send: records the offer and asks the scheduler; it never touches the OUT
# hardware and never starts a DMA by itself.
assert "NRF_USBD->SIZE.ISOOUT" not in iso_send
assert "TASKS_STARTISO" not in iso_send
assert "NRFUSBD_ISO_IN_READY" in iso_send
assert "NRFUSBD_ISO_OUT_READY" in iso_send
assert "nRFUsbdResumeQueuedDmaLocked();" in iso_send

# Completion clears the direction's flag and reports through the endpoint
# registration, like every other endpoint.
assert "IsoDataFlag &=" in iso_complete
assert "nRFUsbEpRegisteredEvent(NRFX_USBD_ISO_EP_NO" in iso_complete
assert "USB_CTRLR_EVT_XFER_CMPL" in iso_complete
assert "nRFUsbdDmaUnlock" not in iso_complete

# The ISO member does no SOF handling of its own.
assert "nRFUsbdSofAcquire" not in iso and "nRFUsbdSofRelease" not in iso
assert "nRFUsbdIsoService" not in base + iso
assert "nRFUsbdSofRelease" not in iso_close

# SOF goes to the core, gated only by whether the core asked for it.
assert "USB_CTRLR_EVT_SOF" in handle_sof
assert "UsbDevProcessEvent(0, &evt)" in handle_sof
assert "if (s_Usbd.SofEnabled)" in handle_sof
assert "nRFUsbdIsoStart" not in handle_sof
assert "UsbCtrlrIsoSend" not in handle_sof

# Interrupt order: bus reset, then SETUP, then SOF, then transfer
# completion and the DMA hand-off.
reset = interrupt.index("if (NRF_USBD->EVENTS_USBRESET != 0U)")
setup = interrupt.index("if (NRF_USBD->EVENTS_EP0SETUP != 0U)")
sof = interrupt.index("if (NRF_USBD->EVENTS_SOF != 0U)")
done = interrupt.index("if (nRFUsbdDmaActive())")
assert reset < setup < sof < done
assert interrupt.count("nRFUsbdHandleSof();") == 1
assert "nRFUsbdIsoComplete(epno == 8U);" in interrupt

# Completion recognizes both ISO directions by their EPSTATUS bit.
assert "case 8U:" in completed and "NRF_USBD->EVENTS_ENDISOIN" in completed
assert "case 24U:" in completed and "NRF_USBD->EVENTS_ENDISOOUT" in completed

# Scheduler: ISO first, then EP0 OUT data, EP0 IN, the regular queue.
iso_pos = queued.index("nRFUsbdIsoStart()")
assert iso_pos < queued.index("EVENTS_EP0DATADONE")
assert iso_pos < queued.index("CFifoPeek(s_Usbd.hEp0Que)")
assert iso_pos < queued.index("CFifoPeek(s_Usbd.hQue)")

# Core: forwards SOF to the ISO endpoint, and enables SOF only when needed.
assert "case USB_CTRLR_EVT_SOF:" in process
assert "UsbCtrlrEpProcessEvent" in process
assert "UsbCoreServiceIso" not in core
assert "SofCount" not in core
assert "UsbCtrlrSofEnable" in update_sof

# UsbIntrf is generic: no ISO names, no SOF handling.
assert "Iso" not in intrf
assert "USB_CTRLR_EVT_SOF" not in intrf

# ISO class: its own IN endpoint callback handles SOF and the IN completion;
# the event callback is the application's; the RX FIFO drops the oldest.
in_event = function_body(iso_intrf, "static void UsbIsoIntrfCtrlrInEvent(")
iso_event = function_body(iso_intrf, "static void UsbIsoIntrfProcessEvent(")
init = function_body(iso_intrf, "bool UsbIsoIntrfInit(")
assert "case USB_CTRLR_EVT_SOF:" in in_event
assert "UsbIsoIntrfProcessEvent(pIntrf, Length);" in in_event
assert "case USB_CTRLR_EVT_XFER_CMPL:" in in_event
assert "pIntrf->Opened" in iso_event
assert "pIntrf->Suspended" in iso_event
assert "pIntrf->Interval" in iso_event
assert "pIntrf->EpNo" in iso_event
assert "UsbCtrlrIsoSend" in iso_event
assert "CFifoPeek(pIntrf->pData->hTxFifo)" in iso_event
assert "CFifoGet" not in iso_event
assert "UsbCtrlrEpBind(pCfg->DevNo, pCfg->EpNo, true" in init
assert "UsbIsoIntrfCtrlrInEvent" in init
assert "cfg.EvtCB = pCfg->EvtCB;" in init
assert "RxHandler" not in iso_intrf and "TxHandler" not in iso_intrf

print("nrf_usb_iso_policy_test: PASS")
