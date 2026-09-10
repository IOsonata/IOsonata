#!/usr/bin/env python3
"""Guard Nordic interrupt OUT start policy not covered by host builds."""

from pathlib import Path


ROOT = Path(__file__).parents[2]
HEADER = ROOT / "ARM/Nordic/include/usb_ctrlr.h"
INT_SOURCE = ROOT / "src/usb/usb_int.cpp"

header = HEADER.read_text(encoding="utf-8")
int_source = INT_SOURCE.read_text(encoding="utf-8")

usbd = header[header.index("#if defined(USBD_PRESENT)") :
              header.index("#elif defined(USBHS_PRESENT)")]
usbhs = header[header.index("#elif defined(USBHS_PRESENT)") :
               header.index("#else\n#error")]

assert "USB_OUT_PREARM_0 = 0" in usbd
assert "USB_OUT_PREARM_0 = 1" in usbhs
assert "cfg.bBlocking = !USB_OUT_PREARM(pCfg->DevNo);" in int_source
assert "cfg.bRxPrearm = USB_OUT_PREARM(pCfg->DevNo);" in int_source
assert "UsbIntrfArmRx(&pIntrf->IntrfData)" in int_source

print("nrf_usb_int_policy_test: PASS")
