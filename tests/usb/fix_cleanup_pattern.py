from pathlib import Path
p = Path('tests/usb/cleanup_usb_hotpath.py')
s = p.read_text()
old = r"    r'''bool UsbCtrlrEpXfer\\(int DevNo, uint8_t EpAddr, uint16_t Length\\)\\n\\{.*?\\n\\}\\n\\nbool UsbCtrlrEp0Xfer''',"
new = r"    r'''bool UsbCtrlrEpXfer\\(int DevNo, uint8_t EpAddr, uint16_t Length\\)\\n\\{.*?\\n\\}\\n+bool UsbCtrlrEp0Xfer''',"
if s.count(old) != 1:
    raise SystemExit('cleanup pattern not found exactly once')
p.write_text(s.replace(old, new))
