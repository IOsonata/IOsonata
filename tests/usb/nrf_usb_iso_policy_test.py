#!/usr/bin/env python3
"""Guard nRF52 endpoint-8 ISO support and link-time separation."""

from pathlib import Path


ROOT = Path(__file__).parents[2]
HEADER = ROOT / "ARM/Nordic/include/usb_ctrlr.h"
BASE = ROOT / "ARM/Nordic/nRF52/src/usb_ctrlr_nrf52.cpp"
ISO = ROOT / "ARM/Nordic/nRF52/src/usb_ctrlr_nrf52_iso.cpp"
PRIV = ROOT / "ARM/Nordic/nRF52/src/usb_ctrlr_nrf52_priv.h"


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
priv = PRIV.read_text(encoding="utf-8")

open_ep = function_body(iso, "bool nRFUsbdIsoEpOpen(")
start_iso = function_body(iso, "static bool nRFUsbdStartIsoNow(void)")
service_iso = function_body(iso, "static void nRFUsbdServiceIso(void)")
iso_sof = function_body(iso, "void nRFUsbdIsoSof(void)")
finish_iso = function_body(iso, "static bool nRFUsbdFinishIsoDma(bool In)")
interrupt = function_body(base, 'extern "C" void USBD_IRQHandler(void)')
handle_sof = function_body(base, "static void nRFUsbdHandleSof(void)")
queued = function_body(base, "void nRFUsbdStartQueuedDma(void)")

assert "USB_EPIN_CNT_0 = 8" in header and "USB_EPOUT_CNT_0 = 8" in header
assert "USB_PKT_MAXLEN_0_ISO = 512" in header
assert "USB_ISO_EPIN_MASK_0 = (1U << 8)" in header
assert "USB_ISO_EPOUT_MASK_0 = (1U << 8)" in header
assert "USB_CTRLR_ISO_INIT(DevNo) UsbCtrlrIsoInit(DevNo)" in header
assert "NRF_USB_EP_COUNT = 9" in priv

assert "USBD_ISOSPLIT_SPLIT_HalfIN" in open_ep
assert "USBD_ISOINCONFIG_RESPONSE_ZeroData" in open_ep
assert "TASKS_STARTISOIN" in start_iso
assert "TASKS_STARTISOOUT" in start_iso
assert "nRFUsbdStartIsoNow()" in service_iso
assert "NRF_USBD->SIZE.ISOOUT" in iso_sof
assert "NRF_USBD->EVENTS_ENDISOIN" in finish_iso
assert "NRF_USBD->EVENTS_ENDISOOUT" in finish_iso
assert finish_iso.index("if (*pEnd == 0U)") < finish_iso.index(
    "NRFX_USBD_EASYDMA_BUSY_REG = NRFX_USBD_EASYDMA_BUSY_REG_CLEAR"
)
assert finish_iso.index("NRF_USBD->EPSTATUS =") < finish_iso.index(
    "NRFX_USBD_EASYDMA_BUSY_REG = NRFX_USBD_EASYDMA_BUSY_REG_CLEAR"
)

assert "extern bool nRFUsbdIsoStart(void) __attribute__((weak));" in base
assert (
    "extern bool nRFUsbdIsoFinishDma(uint32_t DmaStatus) "
    "__attribute__((weak));"
) in base
assert "bool UsbCtrlrIsoInit(int DevNo)" in iso
assert "nRFUsbdIsoFinishDma(dmastatus);" in interrupt
assert "nRFUsbdIsoSof();" in handle_sof
assert "nRFUsbdIsoService();" in handle_sof
assert queued.index("nRFUsbdIsoStart()") < queued.index("CFifoGet(s_Usbd.hQue)")

print("nrf_usb_iso_policy_test: PASS")
