#!/usr/bin/env python3
"""Check the nRF54 send API and controller-owned RX with simulated registers."""
from pathlib import Path
import os
import re
import subprocess
import tempfile

ROOT = Path(__file__).resolve().parents[2]
source = (ROOT / 'ARM/Nordic/nRF54/src/usb_ctrlr_nrf54.cpp').read_text()


def function(name):
    match = re.search(r'(?:void|bool|uint8_t|uint32_t|nRFUsbEpReg_t\s*\*|nRF54UsbdXfer_t\s*\*)\s*' +
                      name + r'\([^;{}]*\)\s*\{', source)
    assert match, name
    end = source.index('{', match.start()) + 1
    depth = 1
    while depth:
        depth += (source[end] == '{') - (source[end] == '}')
        end += 1
    return source[match.start():end]


code = r'''
#include <cassert>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <initializer_list>
#include "usb/usb.h"
uint32_t regs[1024];
bool powered;
uint32_t &reg(unsigned offset){assert(powered);return regs[offset/4];}
#define NRF54_USBD_REG(ofs) reg(ofs)
#define NRF54_USBD_EP_COUNT 16U
constexpr unsigned NRF_USB_EP_COUNT=16;
'''
code += source[source.index('#define NRF54_USBD_GAHBCFG'):source.index('\nenum\n{')]
code += source[source.index('typedef struct __nRF_Usb_Ep_Registration'):
               source.index('// Fixed data-endpoint ownership')]
code += r'''
nRFUsbEpReg_t s_EpReg[16][2];
nRF54UsbdCtrlr_t s_Ctrlr;
uint8_t s_Ep0Bounce[64],buffer[1024],other[1024];
unsigned irqMask,completions,ready;
bool hold,releaseBuffer,closeOnComplete,expectedIn;
uint8_t expectedEp;
uint16_t expectedLength;
uint32_t DisableInterrupt(){auto old=irqMask;irqMask=1;return old;}
void EnableInterrupt(uint32_t old){irqMask=old;}
void AppEvtHandlerExec(){}
void nRFUsbPowerProcess(){}
void UsbDevProcessEvent(int,const UsbCtrlrEvt_t*){assert(false);}
bool nRF54UsbdStartEp0Chunk(uint8_t){assert(false);return false;}
bool nRF54UsbdGrowRxFifo(uint16_t){return true;}
bool nRF54UsbdAllocateTxFifo(uint8_t,uint16_t){return true;}
bool nRF54UsbdDisableEndpoint(uint8_t,bool){return true;}
'''
for name in ['nRFUsbValidDevNo', 'nRFUsbEpDir', 'nRFUsbGetEpReg',
             'nRF54UsbdDir', 'nRF54UsbdGetXfer', 'nRFUsbEpRegisteredEvent',
             'nRF54UsbdEmit', 'nRF54UsbdEmitXfer', 'nRF54UsbdEpType',
             'nRFUsbRegEpXfer', 'nRFUsbRegEpOpen', 'nRFUsbRegEpClose',
             'nRF54UsbdCompleteData', 'UsbCtrlrEpAlloc', 'UsbCtrlrEpSend',
             'UsbCtrlrProcess']:
    code += function(name) + '\n'
code += r'''
void callback(UsbCtrlrEvtType_t event,uint16_t length,void *context){
 assert(context==buffer);
 const uint8_t ep=expectedEp;
 const uint8_t epAddr=ep|(expectedIn?0x80:0);
 if(event==USB_CTRLR_EVT_DRDY){
  assert(!expectedIn && ep<16 && irqMask && !length);++ready;
  if(releaseBuffer)s_EpReg[ep][0].pBuffer=buffer;
  return;
 }
 assert(event==USB_CTRLR_EVT_XFER_CMPL && length==expectedLength);
 assert(!nRF54UsbdGetXfer(epAddr)->Started);++completions;
 if(hold)s_EpReg[ep][0].pBuffer=nullptr;
 if(closeOnComplete)nRFUsbRegEpClose(epAddr);
}
void init(){
 powered=true;memset(regs,0,sizeof(regs));memset(s_EpReg,0,sizeof(s_EpReg));
 s_Ctrlr={};s_Ctrlr.Started=true;
 irqMask=completions=ready=0;hold=releaseBuffer=closeOnComplete=false;
 expectedEp=0;expectedIn=false;
}
int main(){
 for(unsigned ep=1;ep<16;++ep)for(unsigned amount:{0U,1U,63U,64U}){
  init();expectedEp=ep;expectedIn=false;expectedLength=amount;
  UsbCtrlrEpAlloc(0,ep,false,buffer,true,callback,buffer);
  assert(nRFUsbRegEpOpen(ep,USB_ENDPATT_TRANS_BULK,64));
  assert(s_Ctrlr.Xfer[ep][0].Started && s_Ctrlr.Xfer[ep][0].TotalLen==64);
  assert(NRF54_USBD_DOEPDMA(ep)==uint32_t(uintptr_t(buffer)));
  NRF54_USBD_DOEPTSIZ(ep)=64-amount;
  nRF54UsbdCompleteData(ep);
  assert(completions==1 && !ready && s_Ctrlr.Xfer[ep][0].Started);
  assert((NRF54_USBD_DOEPTSIZ(ep)&NRF54_USBD_DEPTSIZ_XFERSIZE_Msk)==64);
  hold=true;NRF54_USBD_DOEPTSIZ(ep)=64-amount;nRF54UsbdCompleteData(ep);
  assert(completions==2 && !s_Ctrlr.Xfer[ep][0].Started);
  for(unsigned mask:{0U,1U}){
   irqMask=mask;UsbCtrlrProcess(0);assert(irqMask==mask);
   assert(!s_Ctrlr.Xfer[ep][0].Started);
  }
  hold=false;releaseBuffer=true;UsbCtrlrProcess(0);
  assert(ready==3 && s_Ctrlr.Xfer[ep][0].Started);
  // An active receive must not be restarted by the foreground poll.
  NRF54_USBD_DOEPTSIZ(ep)=64-amount;UsbCtrlrProcess(0);
  assert(ready==3 && NRF54_USBD_DOEPTSIZ(ep)==64-amount);
  closeOnComplete=true;nRF54UsbdCompleteData(ep);
  assert(completions==3 && !s_Ctrlr.Xfer[ep][0].Started);
  UsbCtrlrProcess(0);assert(!s_Ctrlr.Xfer[ep][0].Started && ready==3);
 }
 for(unsigned ep=1;ep<16;++ep)for(unsigned amount:{0U,1U,64U}){
  init();expectedEp=ep;expectedIn=true;expectedLength=amount;
  UsbCtrlrEpAlloc(0,ep,true,other,false,callback,buffer);
  assert(nRFUsbRegEpOpen(ep|0x80,USB_ENDPATT_TRANS_BULK,64));
  assert(UsbCtrlrEpSend(0,ep,buffer,amount));
  assert(NRF54_USBD_DIEPDMA(ep)==uint32_t(uintptr_t(buffer)));
  assert((NRF54_USBD_DIEPTSIZ(ep)&NRF54_USBD_DEPTSIZ_XFERSIZE_Msk)==amount);
  NRF54_USBD_DIEPTSIZ(ep)=0;nRF54UsbdCompleteData(ep|0x80);
  assert(completions==1 && !s_Ctrlr.Xfer[ep][1].Started);
 }
 init();s_Ctrlr.Started=false;powered=false;UsbCtrlrProcess(0);
 puts("PASS: nRF54 sends explicit buffers/ZLPs; RX opens, rearms, withholds, retries and closes inside the controller");
}
'''
with tempfile.TemporaryDirectory(prefix='iosonata-nrf54-data-') as tmp:
    path = Path(tmp) / 'test.cpp'
    binary = Path(tmp) / 'test'
    path.write_text(code)
    subprocess.run([os.environ.get('CXX', 'g++'), '-std=gnu++17', '-O1',
                    '-fsanitize=undefined', '-fno-sanitize-recover=all',
                    '-I' + str(ROOT / 'include'), '-I' + str(ROOT / 'tests/usb/hostport'),
                    str(path), '-o', str(binary)], check=True)
    subprocess.run([str(binary)], check=True)
