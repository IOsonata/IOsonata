from pathlib import Path
import re


def rep(path, old, new, count=1):
    p = Path(path)
    s = p.read_text()
    n = s.count(old)
    if n != count:
        raise SystemExit(f"{path}: expected {count} copies, found {n}: {old[:100]!r}")
    p.write_text(s.replace(old, new))


def sub(path, pattern, repl, count=1):
    p = Path(path)
    s = p.read_text()
    out, n = re.subn(pattern, repl, s, count=count, flags=re.S)
    if n != count:
        raise SystemExit(f"{path}: expected {count} regex replacements, got {n}: {pattern[:100]!r}")
    p.write_text(out)


# Function-level USB completion dispatch is already known to be XFER_CMPL.
rep('src/usb/usbd_cdc.cpp',
    'UsbdCdcNotifXfer(EpAddr, Length, Result, pCdc);',
    'UsbdCdcNotifXfer(EpAddr, USB_CTRLR_EVT_XFER_CMPL,\n\t\t\tLength, Result, pCdc);')

# Discarding an HCI ACL packet must release RX storage through UsbIntrf so it
# alone decides whether a previously deferred DRDY can now be serviced.
sub('src/bluetooth/bt_hci_usb.cpp',
    r'''static void BtHciUsbDropPhysicalAcl\(BtHciUsbDev_t \*pHci\)\n\{.*?\n\}\n\nstatic bool BtHciUsbBulkRxTypeValid''',
    '''static void BtHciUsbDropPhysicalAcl(BtHciUsbDev_t *pHci)\n{\n\t(void)pHci->AclRxData(&pHci->pAcl->DevIntrf,\n\t\tBtHciUsbAclRxBuffer(pHci), pHci->pAcl->Mps);\n}\n\nstatic bool BtHciUsbBulkRxTypeValid''')

# Reconfiguration follows the same ordering as initial configuration: MPS is
# active before OUT opens, and OUT is opened last so an immediate DRDY is safe.
sub('src/bluetooth/bt_hci_usb.cpp',
    r'''static bool BtHciUsbResetBulkTransport\(BtHciUsbDev_t \*pHci\)\n\{.*?\n\treturn true;\n\}''',
    '''static bool BtHciUsbResetBulkTransport(BtHciUsbDev_t *pHci)\n{\n\tconst uint16_t mps = BtHciUsbAclMps(pHci);\n\tUsbCtrlrEpClose(pHci->DevNo, USB_ENDPADDR_DIROUT(pHci->AclEpNo));\n\tUsbCtrlrEpClose(pHci->DevNo, USB_ENDPADDR_DIRIN(pHci->AclEpNo));\n\tUsbIntrfUnconfigure(pHci->pAcl);\n\tBtHciUsbClearBulkTransport(pHci);\n\n\tif (!UsbIntrfConfigure(pHci->pAcl, mps))\n\t{\n\t\treturn false;\n\t}\n\n\tif (!BtHciUsbOpenEndpoint(pHci, USB_ENDPADDR_DIRIN(pHci->AclEpNo),\n\t\t\tUSB_ENDPATT_TRANS_BULK, mps, 0U) ||\n\t\t!BtHciUsbOpenEndpoint(pHci, USB_ENDPADDR_DIROUT(pHci->AclEpNo),\n\t\t\tUSB_ENDPATT_TRANS_BULK, mps, 0U))\n\t{\n\t\tUsbCtrlrEpClose(pHci->DevNo,\n\t\t\tUSB_ENDPADDR_DIROUT(pHci->AclEpNo));\n\t\tUsbCtrlrEpClose(pHci->DevNo,\n\t\t\tUSB_ENDPADDR_DIRIN(pHci->AclEpNo));\n\t\tUsbIntrfUnconfigure(pHci->pAcl);\n\t\treturn false;\n\t}\n\treturn true;\n}''')

print('remaining USB architecture call sites fixed')
