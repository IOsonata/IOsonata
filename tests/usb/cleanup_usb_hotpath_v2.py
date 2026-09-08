from pathlib import Path

# Reuse the guarded cleanup. Its only known mismatch is the final public
# UsbCtrlrEpXfer text pattern; all earlier transformations must still pass.
try:
    exec(compile(Path('tests/usb/cleanup_usb_hotpath.py').read_text(),
                 'tests/usb/cleanup_usb_hotpath.py', 'exec'), {})
except SystemExit as exc:
    if 'UsbCtrlrEpXfer' not in str(exc):
        raise

path = Path('ARM/Nordic/src/usb_ctrlr_nrfx.cpp')
s = path.read_text()
old = '''bool UsbCtrlrEpXfer(int DevNo, uint8_t EpAddr, uint16_t Length)
{
	const uint8_t epNum = USB_ENDPADDR_NUM(EpAddr);
	if (!nRFUsbValidDevNo(DevNo) || epNum == 0U ||
		epNum >= NRF_USB_EP_COUNT ||
		(EpAddr & ~(USB_ENDPADDR_DIR_MASK | USB_ENDPADDR_NUM_MASK)) != 0U)
	{
		return false;
	}

	nRFUsbEpReg_t *pReg = nRFUsbGetEpReg(EpAddr);
	if (pReg->pBuffer == NULL || pReg->Handler == NULL)
	{
		return false;
	}

	return nRFUsbRegEpXfer(EpAddr, NULL, Length);
}
'''
new = '''bool UsbCtrlrEpXfer(int DevNo, uint8_t EpAddr, uint16_t Length)
{
	(void)DevNo;
	return nRFUsbRegEpXfer(EpAddr, NULL, Length);
}
'''
if s.count(old) != 1:
    raise SystemExit(f'expected UsbCtrlrEpXfer body exactly once, got {s.count(old)}')
path.write_text(s.replace(old, new))
print('USB hot-path cleanup applied')
