#!/usr/bin/env python3
"""Guard nRF52 endpoint-8 ISO transport and core-owned SOF scheduling."""

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

open_ep = function_body(iso, "bool UsbCtrlrEpOpen(")
start_iso = function_body(iso, "bool nRFUsbdIsoStart(void)")
in_xfer = function_body(iso, "bool UsbCtrlrEpInXfer(")
finish_iso = function_body(iso, "static bool nRFUsbdFinishIsoDma(bool In, bool Notify)")
interrupt = function_body(base, 'extern "C" void USBD_IRQHandler(void)')
handle_sof = function_body(base, "static void nRFUsbdHandleSof(void)")
queued = function_body(base, "void nRFUsbdStartQueuedDma(void)")
process = function_body(core, "void UsbDevProcessEvent(")
update_sof = function_body(core, "static void UsbCoreUpdateSof(void)")

assert "USB_EPIN_CNT_0 = 8" in header and "USB_EPOUT_CNT_0 = 8" in header
assert "USB_CTRLR0_ISO_PKT_LEN_MAX = 512" in header
assert "USB_ISO_EPIN_MASK_0 = (1U << 8)" in header
assert "USB_ISO_EPOUT_MASK_0 = (1U << 8)" in header
assert "USB_CTRLR_ISO_INIT(DevNo) UsbCtrlrIsoInit(DevNo)" in header
assert "NRF_USB_EP_COUNT = 9" in header
assert "UsbCtrlrEpOutXfer" not in header + base + iso + iso_intrf
assert "UsbCtrlrEpInXfer" in header

for source in (header, base, iso):
    assert "NRFUSBD_ISO_OUT_READY" not in source
    assert "NRFUSBD_ISO_IN_READY" not in source
    assert "IsoBufState" not in source
assert "uint8_t IsoBusy;" in header
assert "NRFUSBD_ISO_OUT_BUSY" in start_iso
assert "NRFUSBD_ISO_IN_BUSY" in in_xfer
assert "NRFUSBD_ISO_OUT_BUSY" in in_xfer
assert "NRF_USBD->SIZE.ISOOUT" in start_iso
assert "NRF_USBD->SIZE.ISOOUT" not in in_xfer

assert "nRFUsbdIsoSof" not in base + iso
assert "nRFUsbdIsoService" not in base + iso
assert "nRFUsbdSofAcquire" not in open_ep
assert "nRFUsbdSofRelease" not in open_ep
assert "nRFUsbdSofRelease" not in function_body(iso, "void nRFUsbdIsoEpClose(")
assert "TASKS_STARTISOIN" in start_iso
assert "TASKS_STARTISOOUT" in start_iso

assert "USB_CTRLR_EVT_SOF" in handle_sof
assert "UsbDevProcessEvent(0, &evt)" in handle_sof
assert "nRFUsbdIso" not in handle_sof
assert "if (s_Usbd.SofEnabled)" in handle_sof
assert "nRFUsbdSofAcquire" not in handle_sof
assert "nRFUsbdSofRelease" not in handle_sof

assert "case USB_CTRLR_EVT_SOF:" in process
assert "UsbCtrlrEpProcessEvent" in process
assert "UsbCoreServiceIso" not in core
assert "SofCount" not in core
assert "UsbCtrlrSofEnable" in update_sof

in_event = function_body(intrf, "static void UsbIntrfCtrlrInEvent(")
iso_event = function_body(iso_intrf, "void UsbIsoIntrfProcessEvent(")
assert "USB_CTRLR_EVT_SOF" in in_event
assert "UsbIsoIntrfProcessEvent" in in_event
assert "pIntrf->Opened" in iso_event
assert "pIntrf->Suspended" in iso_event
assert "pIntrf->Interval" in iso_event
assert "pIntrf->Mps" in iso_event
assert "pIntrf->EpNo" in iso_event
assert "UsbCtrlrEpInXfer" in iso_event
assert "UsbCtrlrEpOutXfer" not in iso_event

assert "NRF_USBD->EVENTS_ENDISOIN" in finish_iso
assert "NRF_USBD->EVENTS_ENDISOOUT" in finish_iso
assert "nRFUsbdDmaUnlock" not in finish_iso
assert "IsoBusy" in finish_iso
assert "IsoDmaLen" in finish_iso
assert finish_iso.index("NRF_USBD->EPSTATUS =") < finish_iso.index("__DSB();")

dma = interrupt.index("const uint32_t dmastatus")
reset = interrupt.index("if (NRF_USBD->EVENTS_USBRESET != 0U)")
iso_end = interrupt.index("NRF_USBD->EVENTS_ENDISOOUT")
sof = interrupt.index("if (NRF_USBD->EVENTS_SOF != 0U)")
assert dma < reset < iso_end < sof
assert interrupt.count("nRFUsbdHandleSof();") == 1
assert queued.index("nRFUsbdIsoStart()") < queued.index("CFifoPeek(s_Usbd.hQue)")

print("nrf_usb_iso_policy_test: PASS")
