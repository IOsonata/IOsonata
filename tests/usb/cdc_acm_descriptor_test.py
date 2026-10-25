#!/usr/bin/env python3
"""CDC ACM class and interface protocol checks."""
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
source = (ROOT / "src/usb/usbd_cdc_desc.cpp").read_text()
assert "desc.Association.bFunctionProtocol = USB_CDC_PROT_ITUTV250;" in source
assert "desc.Control.bInterfaceProtocol = USB_CDC_PROT_ITUTV250;" in source
assert "desc.Control.bInterfaceSubClass = USB_CDC_SUBCLASS_ACM;" in source
assert "desc.Union.bSubordinateInterf[0] = data;" in source
print("CDC ACM descriptor regression: PASS")
