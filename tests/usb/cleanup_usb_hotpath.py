from pathlib import Path
import re


def sub(path, pattern, repl, count=1):
    p = Path(path)
    s = p.read_text()
    out, n = re.subn(pattern, repl, s, count=count, flags=re.S)
    if n != count:
        raise SystemExit(f'{path}: expected {count} replacements, got {n}: {pattern[:100]!r}')
    p.write_text(out)


def rep(path, old, new, count=1):
    p = Path(path)
    s = p.read_text()
    n = s.count(old)
    if n != count:
        raise SystemExit(f'{path}: expected {count} copies, got {n}: {old[:100]!r}')
    p.write_text(s.replace(old, new))


# UsbIntrf registered endpoint callbacks are entered only after Init/Configure/
# EpOpen succeeded. Keep validation at those boundaries, not in the per-packet
# ISR path.
path = 'src/usb/usb_intrf.cpp'

sub(path,
    r'''static inline __attribute__\(\(always_inline\)\)\nbool UsbIntrfEnabled\(const UsbDevIntrf_t \*pIntrf\)\n\{.*?\n\}\n\n''',
    '')

sub(path,
    r'''static bool UsbIntrfRxSubmit\(UsbDevIntrf_t \*pIntrf\)\n\{.*?\n\}\n\n/\*\* Service only a DRDY event that was previously deferred\. \*/''',
    '''static inline __attribute__((always_inline))\nvoid UsbIntrfRxSubmit(UsbDevIntrf_t *pIntrf)\n{\n\tif (pIntrf->bBlocking && CFifoAvail(pIntrf->hRxFifo) <= 0)\n\t{\n\t\tpIntrf->RxPending = true;\n\t\treturn;\n\t}\n\n\t(void)UsbCtrlrEpXfer(pIntrf->DevNo,\n\t\t\t\t\t\tUSB_ENDPADDR_DIROUT(pIntrf->EpNo), pIntrf->Mps);\n\tpIntrf->RxPending = false;\n}\n\n/** Service only a DRDY event that was previously deferred. */''')

sub(path,
    r'''static void UsbIntrfRxResumePending\(UsbDevIntrf_t \*pIntrf\)\n\{.*?\n\}\n\nstatic inline __attribute__\(\(always_inline\)\)\nvoid UsbIntrfSetTxIdle''',
    '''static void UsbIntrfRxResumePending(UsbDevIntrf_t *pIntrf)\n{\n\tif (!pIntrf->RxPending)\n\t{\n\t\treturn;\n\t}\n\n\tconst uint32_t state = DisableInterrupt();\n\tif (pIntrf->RxPending && CFifoAvail(pIntrf->hRxFifo) > 0)\n\t{\n\t\t(void)UsbCtrlrEpXfer(pIntrf->DevNo,\n\t\t\t\t\t\t\tUSB_ENDPADDR_DIROUT(pIntrf->EpNo), pIntrf->Mps);\n\t\tpIntrf->RxPending = false;\n\t}\n\tEnableInterrupt(state);\n}\n\nstatic inline __attribute__((always_inline))\nvoid UsbIntrfSetTxIdle''')

rep(path,
    '''\tif (!UsbCtrlrEpXfer(pIntrf->DevNo,\n\t\t\t\t\t\tUSB_ENDPADDR_DIRIN(pIntrf->EpNo), (uint16_t)cnt))\n\t{\n\t\tUsbIntrfTxFailure(pIntrf, (uint16_t)cnt);\n\t}\n''',
    '''\t(void)UsbCtrlrEpXfer(pIntrf->DevNo,\n\t\t\t\t\t\t USB_ENDPADDR_DIRIN(pIntrf->EpNo), (uint16_t)cnt);\n''',
    count=2)

sub(path,
    r'''static void UsbIntrfEnable\(DevIntrf_t \* const pDevIntrf\)\n\{.*?\n\}\n\nstatic uint32_t UsbIntrfGetRate''',
    '''static void UsbIntrfEnable(DevIntrf_t * const pDevIntrf)\n{\n\t(void)pDevIntrf;\n}\n\nstatic uint32_t UsbIntrfGetRate''')

sub(path,
    r'''static void UsbIntrfCtrlrEvent\(uint8_t EpAddr, UsbCtrlrEvtType_t Event,\n.*?\n\}\n\nbool UsbIntrfInit''',
    '''static void UsbIntrfCtrlrEvent(uint8_t EpAddr, UsbCtrlrEvtType_t Event,\n\t\t\t\t\t\t\t   uint16_t Length,\n\t\t\t\t\t\t\t   UsbCtrlrXferResult_t Result,\n\t\t\t\t\t\t\t   void *pContext)\n{\n\tUsbDevIntrf_t *pIntrf = static_cast<UsbDevIntrf_t *>(pContext);\n\n\tif (Event == USB_CTRLR_EVT_DRDY)\n\t{\n\t\tUsbIntrfRxSubmit(pIntrf);\n\t\treturn;\n\t}\n\n\tif (USB_ENDPADDR_IS_IN(EpAddr))\n\t{\n\t\tUsbIntrfTxXferComplete(pIntrf, Length, Result);\n\t}\n\telse\n\t{\n\t\tUsbIntrfRxXferComplete(pIntrf, Length, Result);\n\t}\n}\n\nbool UsbIntrfInit''')

sub(path,
    r'''static void UsbIntrfRxXferComplete\(UsbDevIntrf_t \*pIntrf, uint16_t Length,\n\t\t\t\t\t\t\t\tUsbCtrlrXferResult_t Result\)\n\{.*?\n\}\n\nstatic void UsbIntrfTxXferComplete''',
    '''static void UsbIntrfRxXferComplete(UsbDevIntrf_t *pIntrf, uint16_t Length,\n\t\t\t\t\t\t\t\tUsbCtrlrXferResult_t Result)\n{\n\tif (Result == USB_CTRLR_XFER_SUCCESS)\n\t{\n\t\tUsbPkt_t *pPacket = reinterpret_cast<UsbPkt_t *>(\n\t\t\tCFifoPut(pIntrf->hRxFifo));\n\t\tpPacket->Hdr.Length = Length;\n\t\tpPacket->Hdr.Reserved = 0U;\n\t\tif (Length > 0U)\n\t\t{\n\t\t\tmemcpy(pPacket->Data, pIntrf->pRxBuffer, Length);\n\t\t}\n\n\t\tif (pIntrf->DevIntrf.EvtCB != nullptr)\n\t\t{\n\t\t\tconst int used = CFifoUsed(pIntrf->hRxFifo);\n\t\t\tif (pIntrf->bBlocking && used >= pIntrf->hRxFifo->MaxIdxCnt)\n\t\t\t{\n\t\t\t\tpIntrf->DevIntrf.EvtCB(&pIntrf->DevIntrf,\n\t\t\t\t\t\t\t\t   DEVINTRF_EVT_RX_FIFO_FULL, nullptr, used);\n\t\t\t}\n\t\t\tpIntrf->DevIntrf.EvtCB(&pIntrf->DevIntrf,\n\t\t\t\t\t\t\t   DEVINTRF_EVT_RX_DATA, nullptr, used);\n\t\t}\n\t\treturn;\n\t}\n\n\tif (Result == USB_CTRLR_XFER_FAILED)\n\t{\n\t\tpIntrf->RxDropCnt++;\n\t\tif (pIntrf->DevIntrf.EvtCB != nullptr)\n\t\t{\n\t\t\tpIntrf->DevIntrf.EvtCB(&pIntrf->DevIntrf,\n\t\t\t\t\t\t\t   DEVINTRF_EVT_RX_TIMEOUT, nullptr, Length);\n\t\t}\n\t}\n}\n\nstatic void UsbIntrfTxXferComplete''')

rep(path,
    '''\tif (pIntrf == nullptr || pIntrf->hTxFifo == nullptr)\n\t{\n\t\treturn;\n\t}\n\n''',
    '',
    count=1)

# Nordic registered endpoint fast path. Registration/opening establishes the
# endpoint index, callback, buffer alignment and MPS before any event can use it.
path = 'ARM/Nordic/src/usb_ctrlr_nrfx.cpp'

sub(path,
    r'''static bool nRFUsbEpRegisteredEvent\(uint8_t EpAddr, UsbCtrlrEvtType_t Event,\n.*?\n\}\n\n//\n// Bus power''',
    '''static inline __attribute__((always_inline))\nvoid nRFUsbEpRegisteredEvent(uint8_t EpAddr, UsbCtrlrEvtType_t Event,\n\t\t\t\t\t\t\t\t uint16_t Length, UsbCtrlrXferResult_t Result)\n{\n\tnRFUsbEpReg_t *pReg = nRFUsbGetEpReg(EpAddr);\n\tpReg->Handler(EpAddr, Event, Length, Result, pReg->pContext);\n}\n\n//\n// Bus power''')

# Registered completion is direct; only EP0 uses the generic controller event.
sub(path,
    r'''static void nRFUsbdEmitXfer\(uint8_t EpAddr, uint16_t Length,\n\t\t\t\t\t\t UsbCtrlrXferResult_t Result\)\n\{.*?\n\}\n\nstatic void nRFUsbdDmaEndIntEnable''',
    '''static void nRFUsbdEmitXfer(uint8_t EpAddr, uint16_t Length,\n\t\t\t\t\t\t UsbCtrlrXferResult_t Result)\n{\n\tif (USB_ENDPADDR_NUM(EpAddr) != 0U)\n\t{\n\t\tnRFUsbEpRegisteredEvent(EpAddr, USB_CTRLR_EVT_XFER_CMPL, Length, Result);\n\t\treturn;\n\t}\n\n\tUsbCtrlrEvt_t evt = {};\n\tevt.Type = USB_CTRLR_EVT_XFER_CMPL;\n\tevt.Xfer.EpAddr = EpAddr;\n\tevt.Xfer.Length = Length;\n\tevt.Xfer.Result = Result;\n\tnRFUsbdEmit(&evt);\n}\n\nstatic void nRFUsbdDmaEndIntEnable''')

rep(path,
    '''\tif (pBuffer == NULL || !pXfer->Started ||\n\t\tpXfer->ActualLen > pXfer->TotalLen)\n\t{\n\t\treturn false;\n\t}\n\n''',
    '',
    count=1)

rep(path,
    '''\tif (pQue != NULL)\n\t{\n\t\tpQue->EpAddr = EpAddr;\n\t\tpQue->Len = Len;\n\t}\n''',
    '''\tpQue->EpAddr = EpAddr;\n\tpQue->Len = Len;\n''',
    count=1)

# Remove invariant validation from the nRF52 and nRF54 registered data-transfer
# engines while leaving EP0/configuration validation intact.
rep(path,
    '''\tif (epNum >= NRFX_USBD_EP_COUNT ||\n\t\t(TotalBytes > 0U && pDmaBuffer == NULL) ||\n\t\tatomic_load(&s_BusSuspended))\n\t{\n\t\treturn false;\n\t}\n\n\tconst uint32_t state = DisableInterrupt();\n\tnRFUsbdXfer_t *pXfer = nRFUsbdGetXfer(EpAddr);\n\tif (pXfer->Started || pXfer->Mps == 0U)\n\t{\n\t\tEnableInterrupt(state);\n\t\treturn false;\n\t}\n\n''',
    '''\tconst uint32_t state = DisableInterrupt();\n\tnRFUsbdXfer_t *pXfer = nRFUsbdGetXfer(EpAddr);\n\n''',
    count=1)

rep(path,
    '''\tif (!s_Ctrlr.Started || epNum >= NRF54_USBD_EP_COUNT ||\n\t\t(TotalBytes > 0U && pDmaBuffer == NULL))\n\t{\n\t\treturn false;\n\t}\n\n\tnRF54UsbdXfer_t *pXfer = nRF54UsbdGetXfer(EpAddr);\n\tif (pXfer->Started || pXfer->Mps == 0U)\n\t{\n\t\treturn false;\n\t}\n\n\tif (epNum != 0U && TotalBytes > 0U &&\n\t\t(((uintptr_t)pDmaBuffer & 0x3U) != 0U))\n\t{\n\t\treturn false;\n\t}\n\n''',
    '''\tnRF54UsbdXfer_t *pXfer = nRF54UsbdGetXfer(EpAddr);\n\n''',
    count=1)

# Public registered transfer submission is itself a hot-path primitive. DevNo,
# endpoint number, registration and buffer validity were established at Init.
sub(path,
    r'''bool UsbCtrlrEpXfer\(int DevNo, uint8_t EpAddr, uint16_t Length\)\n\{.*?\n\}\n\nbool UsbCtrlrEp0Xfer''',
    '''bool UsbCtrlrEpXfer(int DevNo, uint8_t EpAddr, uint16_t Length)\n{\n\t(void)DevNo;\n\treturn nRFUsbRegEpXfer(EpAddr, NULL, Length);\n}\n\nbool UsbCtrlrEp0Xfer''')

print('USB hot-path cleanup applied')
