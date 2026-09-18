#!/usr/bin/env python3
"""Guard nRF52 endpoint-8 ISO support that host tests cannot compile."""

from pathlib import Path


ROOT = Path(__file__).parents[2]
HEADER = ROOT / "ARM/Nordic/include/usb_ctrlr.h"
SOURCE = ROOT / "ARM/Nordic/nRF52/src/usb_ctrlr_nrf52.cpp"


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
source = SOURCE.read_text(encoding="utf-8")
open_ep = function_body(source, "static bool nRFUsbRegEpOpen(")
start_iso = function_body(source, "static bool nRFUsbdStartIsoNow(void)")
service_iso = function_body(source, "static void nRFUsbdServiceIso(void)")
handle_sof = function_body(source, "static void nRFUsbdHandleSof(void)")
interrupt = function_body(source, 'extern "C" void USBD_IRQHandler(void)')

assert "USB_EPIN_CNT_0 = 8" in header and "USB_EPOUT_CNT_0 = 8" in header
assert "USB_PKT_MAXLEN_0_ISO = 512" in header
assert "USB_ISO_EPIN_MASK_0 = (1U << 8)" in header
assert "USB_ISO_EPOUT_MASK_0 = (1U << 8)" in header
assert "NRF_USB_EP_COUNT = 9" in source
assert "USBD_ISOSPLIT_SPLIT_HalfIN" in open_ep
assert "USBD_ISOINCONFIG_RESPONSE_ZeroData" in open_ep
assert "TASKS_STARTISOIN" in start_iso
assert "TASKS_STARTISOOUT" in start_iso
assert "nRFUsbdStartIsoNow()" in service_iso
assert "NRF_USBD->SIZE.ISOOUT" in handle_sof
finish_iso = function_body(source, "static bool nRFUsbdFinishIsoDma(bool In)\n{")
assert "NRF_USBD->EVENTS_ENDISOIN" in finish_iso
assert "NRF_USBD->EVENTS_ENDISOOUT" in finish_iso
assert finish_iso.index("if (*pEnd == 0U)") < finish_iso.index("nRFUsbdDmaUnlock();")
assert finish_iso.index("NRF_USBD->EPSTATUS =") < finish_iso.index("nRFUsbdDmaUnlock();")
assert "nRFUsbdFinishIsoDma(true);" in interrupt
assert "nRFUsbdFinishIsoDma(false);" in interrupt
assert "dmastatus & 0x01000100UL" in interrupt
# SOF services ISO, and the shared scheduler gives it priority before
# selecting regular queue work. These calls no longer sit directly in the ISR.
queued = function_body(source, "void nRFUsbdStartQueuedDma(void)")
assert "nRFUsbdServiceIso();" in handle_sof
assert queued.index("nRFUsbdStartIsoNow()") < queued.index("CFifoGet(s_hQue)")
assert interrupt.index("nRFUsbdHandleSof();") < interrupt.index(
    "nRFUsbdResumeQueuedDmaLocked();"
)

print("nrf_usb_iso_policy_test: PASS")
