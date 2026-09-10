#!/usr/bin/env python3
"""Guard nRF52 endpoint-8 ISO support that host tests cannot compile."""

from pathlib import Path


ROOT = Path(__file__).parents[2]
HEADER = ROOT / "ARM/Nordic/include/usb_ctrlr.h"
SOURCE = ROOT / "ARM/Nordic/src/usb_ctrlr_nrfx.cpp"


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
service = function_body(source, "static void nRFUsbdServicePending(void)")
interrupt = function_body(source, 'extern "C" void USBD_IRQHandler(void)')

assert "USB_EPIN_CNT_0 = 8" in header and "USB_EPOUT_CNT_0 = 8" in header
assert "USB_PKT_MAXLEN_0_ISO = 512" in header
assert "USB_ISO_EPIN_MASK_0 = (1U << 8)" in header
assert "USB_ISO_EPOUT_MASK_0 = (1U << 8)" in header
assert "NRF_USB_EP_COUNT = 9" in source
assert "USBD_ISOSPLIT_SPLIT_HalfIN" in open_ep
assert "USBD_ISOINCONFIG_RESPONSE_ZeroData" in open_ep
assert service.index("nRFUsbdStartIsoNow()") < service.index("CFifoGet(s_hQue)")
assert "NRF_USBD->SIZE.ISOOUT" in interrupt
assert "nRFUsbdHandleIsoInEnd();" in interrupt
assert "nRFUsbdHandleIsoOutEnd();" in interrupt
assert interrupt.index("atomic_load(&s_IsoInOpen)") < interrupt.index(
    "if (s_Ctrlr.SofEnabled)"
)
assert interrupt.index("atomic_load(&s_IsoOutOpen)") < interrupt.index(
    "if (s_Ctrlr.SofEnabled)"
)

print("nrf_usb_iso_policy_test: PASS")
