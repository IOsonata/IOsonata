#!/usr/bin/env python3
"""Guard the nRF52 USB DMA architecture that host tests cannot compile."""

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
service = function_body(source, "static void nRFUsbdServicePending(void)")
out_data = function_body(source, "static void nRFUsbdHandleOutData(uint8_t EpNum)")

# Silicon capability remains advertised separately from the ordinary endpoint
# count. Endpoint 8 is the nRF52 dedicated isochronous endpoint.
assert "USB_EPIN_CNT_0 = 8" in header and "USB_EPOUT_CNT_0 = 8" in header
assert "USB_PKT_MAXLEN_0_ISO = 512" in header
assert "USB_ISO_EPIN_MASK_0 = (1U << 8)" in header
assert "USB_ISO_EPOUT_MASK_0 = (1U << 8)" in header

# EasyDMA arbitration stays generic: ordinary endpoint work is serialized by
# the CFifo in submission order. ISO must not insert a private scheduler ahead
# of that queue.
assert "CFifoGet(s_hQue)" in service
assert "nRFUsbdStartIsoNow" not in source
assert "USBD_ISOSPLIT_SPLIT_HalfIN" not in source
assert "USBD_ISOINCONFIG_RESPONSE_NoResp" not in source

# Blocking is an endpoint policy. A blocking OUT endpoint reports DRDY; a
# nonblocking OUT endpoint immediately submits its registered DMA buffer.
assert "bool bBlocking;" in source
assert "pReg->bBlocking" in out_data
assert "USB_CTRLR_EVT_DRDY" in out_data
assert "nRFUsbRegDataEpXfer" in out_data

print("nrf_usb_iso_policy_test: PASS")
