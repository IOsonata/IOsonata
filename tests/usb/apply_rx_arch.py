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


# One endpoint callback, with an explicit controller event code.
for path in ('ARM/Nordic/include/usb_ctrlr.h', 'tests/usb/hostport/usb_ctrlr.h'):
    rep(path,
        '\tUSB_CTRLR_EVT_SETUP,\t\t//!< New EP0 SETUP request\n\tUSB_CTRLR_EVT_XFER_CMPL,\t//!< Endpoint transfer completed\n',
        '\tUSB_CTRLR_EVT_SETUP,\t\t//!< New EP0 SETUP request\n\tUSB_CTRLR_EVT_DRDY,\t\t//!< Data is ready in the device to be retrieved\n\tUSB_CTRLR_EVT_XFER_CMPL,\t//!< Endpoint transfer completed\n')
    rep(path,
        'typedef void (*UsbCtrlrEpHandler_t)(uint8_t EpAddr, uint16_t Length,\n\t\t\t\t\t\t\t\t\tUsbCtrlrXferResult_t Result, void *pContext);',
        'typedef void (*UsbCtrlrEpHandler_t)(uint8_t EpAddr, UsbCtrlrEvtType_t Event,\n\t\t\t\t\t\t\t\t\tuint16_t Length, UsbCtrlrXferResult_t Result,\n\t\t\t\t\t\t\t\t\tvoid *pContext);')
    rep(path,
        'bool UsbCtrlrEpRxArm(int DevNo, uint8_t EpNo);\nbool UsbCtrlrEpSend(int DevNo, uint8_t EpNo, uint16_t Length);',
        'bool UsbCtrlrEpXfer(int DevNo, uint8_t EpAddr, uint16_t Length);')

path = 'ARM/Nordic/src/usb_ctrlr_nrfx.cpp'
rep(path,
    '''static bool nRFUsbEpRegisteredXfer(uint8_t EpAddr, uint16_t Length,\n\t\t\t\t\t\t\t\t   UsbCtrlrXferResult_t Result)\n{\n\tconst uint8_t epNum = USB_ENDPADDR_NUM(EpAddr);\n\tif (epNum == 0U || epNum >= NRF_USB_EP_COUNT)\n\t{\n\t\treturn false;\n\t}\n\n\tnRFUsbEpReg_t *pReg = nRFUsbGetEpReg(EpAddr);\n\tif (pReg->Handler == NULL)\n\t{\n\t\treturn false;\n\t}\n\n\tpReg->Handler(EpAddr, Length, Result, pReg->pContext);\n\treturn true;\n}\n''',
    '''static bool nRFUsbEpRegisteredEvent(uint8_t EpAddr, UsbCtrlrEvtType_t Event,\n\t\t\t\t\t\t\t\t\tuint16_t Length,\n\t\t\t\t\t\t\t\t\tUsbCtrlrXferResult_t Result)\n{\n\tconst uint8_t epNum = USB_ENDPADDR_NUM(EpAddr);\n\tif (epNum == 0U || epNum >= NRF_USB_EP_COUNT)\n\t{\n\t\treturn false;\n\t}\n\n\tnRFUsbEpReg_t *pReg = nRFUsbGetEpReg(EpAddr);\n\tif (pReg->Handler == NULL)\n\t{\n\t\treturn false;\n\t}\n\n\tpReg->Handler(EpAddr, Event, Length, Result, pReg->pContext);\n\treturn true;\n}\n''')
rep(path,
    'nRFUsbEpRegisteredXfer(EpAddr, Length, Result)',
    'nRFUsbEpRegisteredEvent(EpAddr, USB_CTRLR_EVT_XFER_CMPL, Length, Result)',
    count=2)

# nRF52 shared EasyDMA queue carries only endpoint and length. Fixed DMA
# buffers remain in s_EpReg and are looked up when the request starts.
rep(path,
    '''typedef struct __nRF_Usbd_Que {\n\tuint8_t EpAddr;\t\t\t\t//!< Endpoint address, direction bit included\n\tuint16_t Len;\t\t\t\t//!< Bytes this transfer moves\n\tuint8_t *pBuffer;\t\t\t//!< DMA buffer for this transfer\n} nRFUsbdQue_t;''',
    '''typedef struct __nRF_Usbd_Que {\n\tuint8_t EpAddr;\t\t\t\t//!< Endpoint address, direction bit included\n\tuint16_t Len;\t\t\t\t//!< Bytes this transfer moves\n} nRFUsbdQue_t;''')

rep(path,
    'NRF_USBD->ISOIN.PTR = (uint32_t)(uintptr_t)pIn->pBuffer;',
    'NRF_USBD->ISOIN.PTR = (uint32_t)(uintptr_t)nRFUsbGetEpReg(inAddr)->pBuffer;')
rep(path,
    'NRF_USBD->ISOOUT.PTR = (uint32_t)(uintptr_t)pOut->pBuffer;',
    'NRF_USBD->ISOOUT.PTR = (uint32_t)(uintptr_t)\n\t\t\tnRFUsbGetEpReg(NRFX_USBD_ISO_EP_NO)->pBuffer;')

rep(path,
    '''\tconst bool isIn = USB_ENDPADDR_IS_IN(pQue->EpAddr);\n\tnRFUsbdXfer_t *pXfer = &s_Ctrlr.Xfer[epNum][isIn ? 1 : 0];\n\n\tif (!pXfer->Started || pXfer->ActualLen > pXfer->TotalLen)''',
    '''\tconst bool isIn = USB_ENDPADDR_IS_IN(pQue->EpAddr);\n\tnRFUsbdXfer_t *pXfer = &s_Ctrlr.Xfer[epNum][isIn ? 1 : 0];\n\tuint8_t *pBuffer = epNum == 0U ? s_Ep0Bounce :\n\t\tnRFUsbGetEpReg(pQue->EpAddr)->pBuffer;\n\n\tif (pBuffer == NULL || !pXfer->Started ||\n\t\tpXfer->ActualLen > pXfer->TotalLen)''')
rep(path,
    'NRF_USBD->EPIN[epNum].PTR = (uint32_t)(uintptr_t)pQue->pBuffer;',
    'NRF_USBD->EPIN[epNum].PTR = (uint32_t)(uintptr_t)pBuffer;')
rep(path,
    'NRF_USBD->EPOUT[epNum].PTR = (uint32_t)(uintptr_t)pQue->pBuffer;',
    'NRF_USBD->EPOUT[epNum].PTR = (uint32_t)(uintptr_t)pBuffer;')

sub(path,
    r'''\t\t\tif \(USB_ENDPADDR_IS_IN\(que\.EpAddr\)\)\n\t\t\t\{.*?\n\t\t\t\}\n\t\t\telse\n\t\t\t\{\n\t\t\t\t// The endpoint still holds the packet after a refused OUT\n\t\t\t\t// start\. Preserve that so the next receive submission takes it\.\n\t\t\t\ts_Ctrlr\.Xfer\[epNum\]\[0\]\.DataReceived = true;\n\t\t\t\}''',
    '''\t\t\tnRFUsbdXfer_t *pXfer = &s_Ctrlr.Xfer[epNum]\n\t\t\t\t[USB_ENDPADDR_IS_IN(que.EpAddr) ? 1 : 0];\n\t\t\tif (pXfer->Started)\n\t\t\t{\n\t\t\t\tpXfer->Started = false;\n\t\t\t\tatomic_flag_clear(&s_DmaRunning);\n\t\t\t\tnRFUsbdEmitXfer(que.EpAddr, pXfer->ActualLen,\n\t\t\t\t\t\t\t USB_CTRLR_XFER_FAILED);\n\t\t\t\tcontinue;\n\t\t\t}''')

sub(path,
    r'''static void nRFUsbdQueXfer\(uint8_t EpAddr, uint8_t \*pBuffer, uint16_t Len\)\n\{.*?\n\}\n\n/\*\* Remove one endpoint number''',
    '''static void nRFUsbdQueXfer(uint8_t EpAddr, uint16_t Len)\n{\n\tconst uint32_t state = DisableInterrupt();\n\tnRFUsbdQue_t *pQue = (nRFUsbdQue_t *)CFifoPut(s_hQue);\n\n\tif (pQue != NULL)\n\t{\n\t\tpQue->EpAddr = EpAddr;\n\t\tpQue->Len = Len;\n\t}\n\n\tEnableInterrupt(state);\n}\n\n/** Remove one endpoint number''')

sub(path,
    r'''static void nRFUsbdQueueOut\(uint8_t EpNum\)\n\{.*?\n\}\n\nstatic void nRFUsbdQueueIn''',
    '''static void nRFUsbdQueueOut(uint8_t EpNum)\n{\n\tnRFUsbdXfer_t *pXfer = &s_Ctrlr.Xfer[EpNum][0];\n\n\tnRFUsbdQueXfer(EpNum,\n\t\t\t\t (uint16_t)(pXfer->TotalLen - pXfer->ActualLen));\n\tif (!nRFUsbdDeferFromInterrupt())\n\t{\n\t\tnRFUsbdServicePending();\n\t}\n}\n\nstatic void nRFUsbdQueueIn''')

sub(path,
    r'''static void nRFUsbdQueueIn\(uint8_t EpNum\)\n\{.*?\n\}\n\nstatic void nRFUsbdQueueEp0Status''',
    '''static void nRFUsbdQueueIn(uint8_t EpNum)\n{\n\tnRFUsbdXfer_t *pXfer = &s_Ctrlr.Xfer[EpNum][1];\n\tconst uint16_t remaining =\n\t\t(uint16_t)(pXfer->TotalLen - pXfer->ActualLen);\n\tconst uint16_t length = remaining < pXfer->Mps ? remaining : pXfer->Mps;\n\n\tif (EpNum == 0U && length > 0U)\n\t{\n\t\tmemcpy(s_Ep0Bounce, pXfer->pBuffer, length);\n\t}\n\n\tnRFUsbdQueXfer((uint8_t)(EpNum | USB_ENDPADDR_DIR_IN), length);\n\tif (!nRFUsbdDeferFromInterrupt())\n\t{\n\t\tnRFUsbdServicePending();\n\t}\n}\n\nstatic void nRFUsbdQueueEp0Status''')

# Replace both target-specific endpoint transfer engines. EP0 retains dynamic
# buffer ownership; non-control endpoints use their registered fixed buffer.
p = Path(path)
s = p.read_text()
pat = re.compile(r'''static bool nRFUsbRegEpXfer\(uint8_t EpAddr, uint8_t \*pBuffer, uint16_t TotalBytes\)\n\{.*?\n\}\n\nstatic uint16_t nRFUsbRegEpMps''', re.S)
ms = list(pat.finditer(s))
if len(ms) != 2:
    raise SystemExit(f"{path}: expected 2 transfer engines, found {len(ms)}")

nrf52 = '''static bool nRFUsbRegEpXfer(uint8_t EpAddr, uint8_t *pBuffer, uint16_t TotalBytes)\n{\n\tconst uint8_t epNum = USB_ENDPADDR_NUM(EpAddr);\n\tuint8_t *pDmaBuffer = epNum == 0U ? pBuffer :\n\t\tnRFUsbGetEpReg(EpAddr)->pBuffer;\n\n\tif (epNum >= NRFX_USBD_EP_COUNT ||\n\t\t(TotalBytes > 0U && pDmaBuffer == NULL) ||\n\t\tatomic_load(&s_BusSuspended))\n\t{\n\t\treturn false;\n\t}\n\n\tconst uint32_t state = DisableInterrupt();\n\tnRFUsbdXfer_t *pXfer = nRFUsbdGetXfer(EpAddr);\n\tif (pXfer->Started || pXfer->Mps == 0U)\n\t{\n\t\tEnableInterrupt(state);\n\t\treturn false;\n\t}\n\n\tpXfer->pBuffer = epNum == 0U ? pBuffer : NULL;\n\tpXfer->TotalLen = TotalBytes;\n\tpXfer->ActualLen = 0U;\n\tpXfer->Started = true;\n\tif (epNum == NRFX_USBD_ISO_EP_NO)\n\t{\n\t\tif (TotalBytes > pXfer->Mps)\n\t\t{\n\t\t\tpXfer->Started = false;\n\t\t\tEnableInterrupt(state);\n\t\t\treturn false;\n\t\t}\n\t\tEnableInterrupt(state);\n\t\tif (!nRFUsbdDeferFromInterrupt())\n\t\t{\n\t\t\tnRFUsbdServicePending();\n\t\t}\n\t\treturn true;\n\t}\n\n\tconst bool controlStatus =\n\t\tepNum == 0U && TotalBytes == 0U &&\n\t\tUSB_ENDPADDR_IS_IN(EpAddr) != s_Ctrlr.SetupDirIn;\n\n\tif (controlStatus)\n\t{\n\t\tnRFUsbdQueueEp0Status();\n\t}\n\telse if (USB_ENDPADDR_IS_IN(EpAddr))\n\t{\n\t\tnRFUsbdQueueIn(epNum);\n\t}\n\telse if (epNum == 0U)\n\t{\n\t\tnRFUsbdQueueEp0RcvOut();\n\t}\n\telse\n\t{\n\t\tnRFUsbdQueueOut(epNum);\n\t}\n\n\tEnableInterrupt(state);\n\treturn true;\n}\n\nstatic uint16_t nRFUsbRegEpMps'''

nrf54 = '''static bool nRFUsbRegEpXfer(uint8_t EpAddr, uint8_t *pBuffer, uint16_t TotalBytes)\n{\n\tconst uint8_t epNum = USB_ENDPADDR_NUM(EpAddr);\n\tuint8_t *pDmaBuffer = epNum == 0U ? pBuffer :\n\t\tnRFUsbGetEpReg(EpAddr)->pBuffer;\n\tif (!s_Ctrlr.Started || epNum >= NRF54_USBD_EP_COUNT ||\n\t\t(TotalBytes > 0U && pDmaBuffer == NULL))\n\t{\n\t\treturn false;\n\t}\n\n\tnRF54UsbdXfer_t *pXfer = nRF54UsbdGetXfer(EpAddr);\n\tif (pXfer->Started || pXfer->Mps == 0U)\n\t{\n\t\treturn false;\n\t}\n\n\tif (epNum != 0U && TotalBytes > 0U &&\n\t\t(((uintptr_t)pDmaBuffer & 0x3U) != 0U))\n\t{\n\t\treturn false;\n\t}\n\n\tpXfer->pBuffer = epNum == 0U ? pBuffer : NULL;\n\tpXfer->TotalLen = TotalBytes;\n\tpXfer->ActualLen = 0U;\n\tpXfer->ChunkLen = 0U;\n\tpXfer->Started = true;\n\n\tif (epNum == 0U)\n\t{\n\t\treturn nRF54UsbdStartEp0Chunk(EpAddr);\n\t}\n\n\tconst uint32_t packetCnt = TotalBytes == 0U ? 1U :\n\t\t((uint32_t)TotalBytes + pXfer->Mps - 1U) / pXfer->Mps;\n\n\tif (packetCnt > 0x3FFUL)\n\t{\n\t\tpXfer->Started = false;\n\t\treturn false;\n\t}\n\n\tconst uint32_t size =\n\t\t((uint32_t)TotalBytes & NRF54_USBD_DEPTSIZ_XFERSIZE_Msk) |\n\t\t(packetCnt << NRF54_USBD_DEPTSIZ_PKTCNT_Pos);\n\n\tif (USB_ENDPADDR_IS_IN(EpAddr))\n\t{\n\t\tNRF54_USBD_DIEPDMA(epNum) = (uint32_t)(uintptr_t)pDmaBuffer;\n\t\tNRF54_USBD_DIEPTSIZ(epNum) = size;\n\t\tNRF54_USBD_DIEPCTL(epNum) |=\n\t\t\tNRF54_USBD_DEPCTL_CNAK | NRF54_USBD_DEPCTL_EPENA;\n\t}\n\telse\n\t{\n\t\tNRF54_USBD_DOEPDMA(epNum) = (uint32_t)(uintptr_t)pDmaBuffer;\n\t\tNRF54_USBD_DOEPTSIZ(epNum) = size;\n\t\tNRF54_USBD_DOEPCTL(epNum) |=\n\t\t\tNRF54_USBD_DEPCTL_CNAK | NRF54_USBD_DEPCTL_EPENA;\n\t}\n\n\treturn true;\n}\n\nstatic uint16_t nRFUsbRegEpMps'''

s = s[:ms[1].start()] + nrf54 + s[ms[1].end():]
s = s[:ms[0].start()] + nrf52 + s[ms[0].end():]
p.write_text(s)

# nRF52 ordinary OUT: EPDATA is DRDY. EP0 keeps its control-transfer flow.
sub(path,
    r'''static void nRFUsbdHandleOutData\(uint8_t EpNum\)\n\{.*?\n\}\n\nstatic void nRFUsbdHandleInData''',
    '''static void nRFUsbdHandleOutData(uint8_t EpNum)\n{\n\tif (EpNum != 0U)\n\t{\n\t\t(void)nRFUsbEpRegisteredEvent(EpNum, USB_CTRLR_EVT_DRDY, 0U,\n\t\t\t\t\t\t\t\t USB_CTRLR_XFER_SUCCESS);\n\t\treturn;\n\t}\n\n\tnRFUsbdXfer_t *pXfer = &s_Ctrlr.Xfer[0][0];\n\tif (pXfer->Started &&\n\t\t(pXfer->ActualLen < pXfer->TotalLen || pXfer->TotalLen == 0U))\n\t{\n\t\tpXfer->DataReceived = false;\n\t\tnRFUsbdQueueOut(0U);\n\t}\n\telse\n\t{\n\t\tpXfer->DataReceived = true;\n\t}\n}\n\nstatic void nRFUsbdHandleInData''')

# ISO OUT has its data-ready indication at SOF/SIZE.ISOOUT rather than EPDATA.
rep(path,
    '''\t\t\t\tatomic_store(&s_IsoOutReady, true);\n''',
    '''\t\t\t\tatomic_store(&s_IsoOutReady, true);\n\t\t\t\t(void)nRFUsbEpRegisteredEvent(NRFX_USBD_ISO_EP_NO,\n\t\t\t\t\tUSB_CTRLR_EVT_DRDY, 0U, USB_CTRLR_XFER_SUCCESS);\n''')

sub(path,
    r'''bool UsbCtrlrEpRxArm\(int DevNo, uint8_t EpNo\)\n\{.*?\n\}\n\nbool UsbCtrlrEpSend\(int DevNo, uint8_t EpNo, uint16_t Length\)\n\{.*?\n\}\n''',
    '''bool UsbCtrlrEpXfer(int DevNo, uint8_t EpAddr, uint16_t Length)\n{\n\tconst uint8_t epNum = USB_ENDPADDR_NUM(EpAddr);\n\tif (!nRFUsbValidDevNo(DevNo) || epNum == 0U ||\n\t\tepNum >= NRF_USB_EP_COUNT ||\n\t\t(EpAddr & ~(USB_ENDPADDR_DIR_MASK | USB_ENDPADDR_NUM_MASK)) != 0U)\n\t{\n\t\treturn false;\n\t}\n\n\tnRFUsbEpReg_t *pReg = nRFUsbGetEpReg(EpAddr);\n\tif (pReg->pBuffer == NULL || pReg->Handler == NULL)\n\t{\n\t\treturn false;\n\t}\n\n\treturn nRFUsbRegEpXfer(EpAddr, NULL, Length);\n}\n\n''')

# CDC notification endpoint uses the same registered event callback.
path = 'src/usb/usbd_cdc.cpp'
rep(path,
    'UsbCtrlrEpSend(pCdc->DevNo, pCdc->NotifyEpNo, USBD_CDC_NOTIFY_LEN)',
    'UsbCtrlrEpXfer(pCdc->DevNo, USB_ENDPADDR_DIRIN(pCdc->NotifyEpNo),\n\t\t\t\t\t\t USBD_CDC_NOTIFY_LEN)')
rep(path,
    '''static void UsbdCdcNotifXfer(uint8_t, uint16_t,\n\t\t\t\t\t\t\t UsbCtrlrXferResult_t Result, void *pContext)\n{\n\tUsbdCdcDev_t *pCdc = static_cast<UsbdCdcDev_t *>(pContext);\n\n\tif (pCdc == nullptr)''',
    '''static void UsbdCdcNotifXfer(uint8_t, UsbCtrlrEvtType_t Event,\n\t\t\t\t\t\t\t uint16_t, UsbCtrlrXferResult_t Result,\n\t\t\t\t\t\t\t void *pContext)\n{\n\tUsbdCdcDev_t *pCdc = static_cast<UsbdCdcDev_t *>(pContext);\n\n\tif (pCdc == nullptr || Event != USB_CTRLR_EVT_XFER_CMPL)''')

# UsbIntrf must know MPS before OUT can generate DRDY. Open OUT last.
sub(path,
    r'''\tif \(!UsbdCdcOpenEndpoint\(pCdc, USB_ENDPADDR_DIRIN\(pCdc->NotifyEpNo\),.*?\n\tif \(!UsbIntrfConfigure\(pCdc->pIntrfData, dataMps\)\)\n\t\{\n\t\tUsbdCdcCloseEndpoints\(pCdc\);\n\t\treturn false;\n\t\}\n''',
    '''\tif (!UsbIntrfConfigure(pCdc->pIntrfData, dataMps))\n\t{\n\t\treturn false;\n\t}\n\n\tif (!UsbdCdcOpenEndpoint(pCdc, USB_ENDPADDR_DIRIN(pCdc->NotifyEpNo),\n\t\t\t\t\t\t\t USB_ENDPATT_TRANS_INT,\n\t\t\t\t\t\t\t USBD_CDC_NOTIF_MPS,\n\t\t\t\t\t\t\t UsbdCdcNotifInterval(pCdc)) ||\n\t\t!UsbdCdcOpenEndpoint(pCdc, USB_ENDPADDR_DIRIN(pCdc->DataEpNo),\n\t\t\t\t\t\t\t USB_ENDPATT_TRANS_BULK, dataMps, 0U) ||\n\t\t!UsbdCdcOpenEndpoint(pCdc, USB_ENDPADDR_DIROUT(pCdc->DataEpNo),\n\t\t\t\t\t\t\t USB_ENDPATT_TRANS_BULK, dataMps, 0U))\n\t{\n\t\tUsbdCdcCloseEndpoints(pCdc);\n\t\tUsbIntrfUnconfigure(pCdc->pIntrfData);\n\t\treturn false;\n\t}\n''')

# Vendor bulk uses the same configure-before-OUT ordering.
path = 'src/usb/usbd_bulk.cpp'
sub(path,
    r'''\tconst uint16_t mps = UsbdBulkMps\(pBulk\);\n\tif \(!UsbdBulkOpenEndpoint\(pBulk, USB_ENDPADDR_DIROUT\(pBulk->EpNo\), mps\) \|\|\n\t\t!UsbdBulkOpenEndpoint\(pBulk, USB_ENDPADDR_DIRIN\(pBulk->EpNo\), mps\)\)\n\t\{\n\t\tUsbdBulkCloseEndpoints\(pBulk\);\n\t\treturn false;\n\t\}\n\n\tif \(!UsbIntrfConfigure\(pBulk->pIntrfData, mps\)\)\n\t\{\n\t\tUsbdBulkCloseEndpoints\(pBulk\);\n\t\treturn false;\n\t\}\n''',
    '''\tconst uint16_t mps = UsbdBulkMps(pBulk);\n\tif (!UsbIntrfConfigure(pBulk->pIntrfData, mps))\n\t{\n\t\treturn false;\n\t}\n\n\tif (!UsbdBulkOpenEndpoint(pBulk, USB_ENDPADDR_DIRIN(pBulk->EpNo), mps) ||\n\t\t!UsbdBulkOpenEndpoint(pBulk, USB_ENDPADDR_DIROUT(pBulk->EpNo), mps))\n\t{\n\t\tUsbdBulkCloseEndpoints(pBulk);\n\t\tUsbIntrfUnconfigure(pBulk->pIntrfData);\n\t\treturn false;\n\t}\n''')

# Temporary standalone ISO wrapper: DRDY submits the RX DMA; completion does
# not rearm. This will later be replaced by UsbdIso inheriting UsbIntrf.
path = 'src/usb/usb_iso.cpp'
sub(path,
    r'''static bool UsbIsoIntrfArmRx\(UsbIsoIntrf_t \*pIntrf\)\n\{.*?\n\}\n\nstatic void UsbIsoIntrfComplete''',
    '''static void UsbIsoIntrfComplete''')
sub(path,
    r'''static void UsbIsoIntrfComplete\(uint8_t EpAddr, uint16_t Length,\n\t\t\t\t\t\t\t\tUsbCtrlrXferResult_t Result, void \*pContext\)\n\{.*?\n\}\n\nbool UsbIsoIntrfInit''',
    '''static void UsbIsoIntrfComplete(uint8_t EpAddr, UsbCtrlrEvtType_t Event,\n\t\t\t\t\t\t\t\tuint16_t Length,\n\t\t\t\t\t\t\t\tUsbCtrlrXferResult_t Result, void *pContext)\n{\n\tUsbIsoIntrf_t *pIntrf = static_cast<UsbIsoIntrf_t *>(pContext);\n\tif (pIntrf == nullptr || USB_ENDPADDR_NUM(EpAddr) != pIntrf->EpNo)\n\t{\n\t\treturn;\n\t}\n\n\tif (Event == USB_CTRLR_EVT_DRDY)\n\t{\n\t\tif (USB_ENDPADDR_IS_IN(EpAddr) || !pIntrf->Opened ||\n\t\t\tpIntrf->Suspended || pIntrf->RxArmed)\n\t\t{\n\t\t\treturn;\n\t\t}\n\t\tif (UsbCtrlrEpXfer(pIntrf->DevNo, EpAddr, pIntrf->Mps))\n\t\t{\n\t\t\tpIntrf->RxArmed = true;\n\t\t}\n\t\telse\n\t\t{\n\t\t\tpIntrf->RxMissCnt++;\n\t\t}\n\t\treturn;\n\t}\n\n\tif (Event != USB_CTRLR_EVT_XFER_CMPL)\n\t{\n\t\treturn;\n\t}\n\n\tif (USB_ENDPADDR_IS_IN(EpAddr))\n\t{\n\t\tif (!pIntrf->TxActive)\n\t\t{\n\t\t\treturn;\n\t\t}\n\n\t\tconst uint16_t expected = pIntrf->TxLength;\n\t\tpIntrf->TxActive = false;\n\t\tpIntrf->TxLength = 0U;\n\n\t\tUsbCtrlrXferResult_t result = Result;\n\t\tif (result == USB_CTRLR_XFER_SUCCESS && Length != expected)\n\t\t{\n\t\t\tresult = USB_CTRLR_XFER_FAILED;\n\t\t}\n\t\tif (result != USB_CTRLR_XFER_SUCCESS)\n\t\t{\n\t\t\tpIntrf->TxMissCnt++;\n\t\t}\n\t\telse if (Length == 0U)\n\t\t{\n\t\t\tpIntrf->TxEmptyCnt++;\n\t\t}\n\n\t\tif (pIntrf->TxHandler != nullptr)\n\t\t{\n\t\t\tpIntrf->TxHandler(pIntrf, Length, result, pIntrf->pContext);\n\t\t}\n\t\treturn;\n\t}\n\n\tpIntrf->RxArmed = false;\n\tUsbCtrlrXferResult_t result = Result;\n\tif (result == USB_CTRLR_XFER_SUCCESS && Length > pIntrf->Mps)\n\t{\n\t\tresult = USB_CTRLR_XFER_FAILED;\n\t}\n\tif (result != USB_CTRLR_XFER_SUCCESS)\n\t{\n\t\tpIntrf->RxMissCnt++;\n\t}\n\telse if (Length == 0U)\n\t{\n\t\tpIntrf->RxEmptyCnt++;\n\t}\n\n\tif (pIntrf->RxHandler != nullptr)\n\t{\n\t\tpIntrf->RxHandler(pIntrf, UsbIsoIntrfRxBuffer(pIntrf), Length,\n\t\t\tresult, pIntrf->pContext);\n\t}\n}\n\nbool UsbIsoIntrfInit''')
sub(path,
    r'''\n\tif \(!UsbIsoIntrfArmRx\(pIntrf\)\)\n\t\{\n\t\tUsbIsoIntrfClose\(pIntrf\);\n\t\treturn false;\n\t\}\n\n\treturn true;''',
    '''\n\treturn true;''')
rep(path,
    '''\tpIntrf->Suspended = false;\n\treturn pIntrf->RxArmed || UsbIsoIntrfArmRx(pIntrf);''',
    '''\tpIntrf->Suspended = false;\n\treturn true;''')
rep(path,
    'UsbCtrlrEpSend(pIntrf->DevNo, pIntrf->EpNo, Length)',
    'UsbCtrlrEpXfer(pIntrf->DevNo, USB_ENDPADDR_DIRIN(pIntrf->EpNo), Length)')

# HCI event-IN is directly registered; ACL data remains through UsbIntrf.
path = 'src/bluetooth/bt_hci_usb.cpp'
rep(path,
    'UsbCtrlrEpSend(pHci->DevNo, pHci->EventEpNo,\n\t\tpHci->EventTxChunkLength)',
    'UsbCtrlrEpXfer(pHci->DevNo, USB_ENDPADDR_DIRIN(pHci->EventEpNo),\n\t\tpHci->EventTxChunkLength)')
rep(path,
    'UsbCtrlrEpSend(pHci->DevNo, pHci->EventEpNo, 0U)',
    'UsbCtrlrEpXfer(pHci->DevNo, USB_ENDPADDR_DIRIN(pHci->EventEpNo), 0U)')
rep(path,
    '''static void BtHciUsbEventComplete(uint8_t, uint16_t Length,\n\t\t\t\t\t\t\t\t UsbCtrlrXferResult_t Result, void *pContext)\n{\n\tBtHciUsbDev_t *pHci = static_cast<BtHciUsbDev_t *>(pContext);\n\tif (pHci == nullptr || !pHci->EventTxActive)''',
    '''static void BtHciUsbEventComplete(uint8_t, UsbCtrlrEvtType_t Event,\n\t\t\t\t\t\t\t\t uint16_t Length,\n\t\t\t\t\t\t\t\t UsbCtrlrXferResult_t Result, void *pContext)\n{\n\tBtHciUsbDev_t *pHci = static_cast<BtHciUsbDev_t *>(pContext);\n\tif (pHci == nullptr || Event != USB_CTRLR_EVT_XFER_CMPL ||\n\t\t!pHci->EventTxActive)''')
rep(path,
    'BtHciUsbEventComplete(EpAddr, Length, Result, pHci);',
    'BtHciUsbEventComplete(EpAddr, USB_CTRLR_EVT_XFER_CMPL,\n\t\t\tLength, Result, pHci);')

# ACL MPS must also be known before its OUT endpoint can emit DRDY.
sub(path,
    r'''\tif \(!BtHciUsbOpenEndpoint\(pHci, USB_ENDPADDR_DIRIN\(pHci->EventEpNo\),.*?\n\t\t!UsbIntrfConfigure\(pHci->pAcl, aclMps\)\)\n\t\{\n\t\tBtHciUsbUnconfigure\(pHci\);\n\t\tBtHciUsbCloseEndpoints\(pHci\);\n\t\treturn false;\n\t\}\n''',
    '''\tif (!UsbIntrfConfigure(pHci->pAcl, aclMps))\n\t{\n\t\treturn false;\n\t}\n\n\tif (!BtHciUsbOpenEndpoint(pHci, USB_ENDPADDR_DIRIN(pHci->EventEpNo),\n\t\t\t\t\t\t\t USB_ENDPATT_TRANS_INT, eventMps, eventInterval) ||\n\t\t!BtHciUsbOpenEndpoint(pHci, USB_ENDPADDR_DIRIN(pHci->AclEpNo),\n\t\t\t\t\t\t\t USB_ENDPATT_TRANS_BULK, aclMps, 0U) ||\n\t\t!BtHciUsbOpenEndpoint(pHci, USB_ENDPADDR_DIROUT(pHci->AclEpNo),\n\t\t\t\t\t\t\t USB_ENDPATT_TRANS_BULK, aclMps, 0U))\n\t{\n\t\tBtHciUsbUnconfigure(pHci);\n\t\tBtHciUsbCloseEndpoints(pHci);\n\t\treturn false;\n\t}\n''')

print('USB architecture transform applied')
