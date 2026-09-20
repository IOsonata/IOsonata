#!/usr/bin/env python3
"""Exercise production queued DMA selection with real CFifo storage.

EP0 can wait behind regular DMA; its compact packet header is not a regular
queue header. This register simulation checks that handoff and DMA ownership.
Packet IN also runs the production UsbIntrf producer/completion and AppEvt
dispatch, so ENDEP and host-consumption ownership are checked separately.

Use --full-appevt to reproduce the regular IN completion lost when AppEvt is
full. This currently fails the assertion that host completion releases a packet.
"""
from pathlib import Path
import os
import re
import subprocess
import sys
import tempfile

ROOT = Path(__file__).resolve().parents[2]
source = Path(os.environ.get('USB_CTRLR_SOURCE',
    ROOT / 'ARM/Nordic/nRF52/src/usb_ctrlr_nrf52.cpp')).read_text()
intrf_source = (ROOT / 'src/usb/usb_intrf.cpp').read_text()
header = (ROOT / 'ARM/Nordic/include/usb_ctrlr.h').read_text()


def function(name, source=source):
    match = re.search(r'(?:void|bool|int|uint8_t|nRFUsbEpReg_t\s*\*)\s*' +
        name + r'\([^;{}]*\)\s*\{', source)
    assert match, name
    end = source.index('{', match.start()) + 1
    depth = 1
    while depth:
        depth += (source[end] == '{') - (source[end] == '}')
        end += 1
    return source[match.start():end]


code = r'''
#include <algorithm>
#include <cassert>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include "cfifo.h"
#include "app_evt_handler.h"
#include "coredev/interrupt.h"
// Bypass the legacy host adapter; exercise the production directional API.
#define UsbCtrlrEpInXfer HostUsbCtrlrEpInXfer
#include "usb/usb_intrf.h"
#undef UsbCtrlrEpInXfer
bool UsbCtrlrEpXfer(int,uint8_t,uint16_t){assert(false);return false;}
using std::min;
constexpr int NRFX_USBD_MAX_PACKET_SIZE=64;
constexpr int NRFX_USBD_ISO_EP_NO=8;
constexpr uint32_t NRFX_USBD_EASYDMA_BUSY_REG_BUSY=0x82;
constexpr uint32_t NRFX_USBD_EASYDMA_BUSY_REG_CLEAR=0;
constexpr uint32_t USBD_SHORTS_EP0DATADONE_EP0STATUS_Msk=1;
uint32_t dmaBusy;
#define NRFX_USBD_EASYDMA_BUSY_REG dmaBusy
struct Endpoint {uint32_t PTR,MAXCNT,AMOUNT;};
using USBD_EPIN_Type=Endpoint;
using USBD_EPOUT_Type=Endpoint;
struct W1C {uint32_t bits;void operator=(uint32_t value){bits&=~value;}};
struct {
 Endpoint EPIN[8],EPOUT[8];
 uint32_t TASKS_STARTEPIN[8],TASKS_STARTEPOUT[8];
 uint32_t EVENTS_ENDEPIN[8],EVENTS_ENDEPOUT[8],EVENTS_EP0DATADONE,SHORTS;
 struct {uint32_t EPOUT[8];} SIZE;
 W1C EPSTATUS;
} regs;
auto *NRF_USBD=&regs;
void __DSB(){}
bool isoReady;
unsigned isoChecks;
bool nRFUsbdIsoStart(){++isoChecks;return isoReady;}
'''
queue_enum = re.search(r'enum\s*\{[^}]*NRFX_USBD_QUE_IN_SCRATCH[^}]*\};', source)
assert queue_enum
code += queue_enum.group(0) + '\n#pragma pack(push,4)\n'
for tag, name in [('__nRF_Usbd_Que', 'nRFUsbdQue_t'), ('__nRF_Ep_Packet', 'nRFEPPkt_t')]:
    match = re.search(r'typedef struct ' + tag + r' \{.*?\} ' + name + ';', source, re.S)
    assert match
    code += match.group(0) + '\n'
code += re.search(r'typedef struct __nRF_Usb_Ep_Registration\s*\{.*?\} nRFUsbEpReg_t;',
    header, re.S).group(0) + '\n'
code += re.search(r'enum\s*\{[^}]*USBD_FLAG_SUSPENDED[^}]*\};', header).group(0) + '\n'
code += r'''
#pragma pack(pop)
struct {
 uint32_t Flags;
 nRFUsbEpReg_t EpReg[8][2];
 hCFifo_t hQue,hEp0Que;
 struct {struct {uint16_t TotalLen;} Ep0[2];} Ctrlr;
 alignas(4) uint8_t Ep0Bounce[64];
} s_Usbd;
alignas(8) uint8_t queueMem[CFIFO_TOTAL_MEMSIZE(16,sizeof(nRFUsbdQue_t))];
alignas(8) uint8_t ep0Mem[CFIFO_TOTAL_MEMSIZE(4,sizeof(nRFEPPkt_t))];
'''
code += '\n'.join(function(name) for name in [
    'nRFUsbdDmaActive', 'nRFUsbdDmaUnlock', 'nRFUsbdDmaStartLocked', 'nRFUsbdRetireDma',
    'nRFUsbdEp0InProgram', 'nRFUsbdStartDmaNow', 'nRFUsbdStartQueuedDma',
    'nRFUsbdResumeQueuedDmaLocked', 'UsbCtrlrEpInXfer',
    'nRFUsbEpDir', 'nRFUsbGetEpReg', 'nRFUsbEpRegisteredEvent',
    'nRFUsbdProcessInComplete', 'nRFUsbdQueueInComplete', 'UsbCtrlrEp0Send'])
code += '\n'.join(function(name, intrf_source) for name in [
    'UsbIntrfSetTxIdle', 'UsbIntrfTakeTx', 'UsbIntrfDirectClear',
    'UsbIntrfTxFailure', 'UsbIntrfEpSendPktMode', 'UsbIntrfTxPackets',
    'UsbIntrfCtrlrInEvent'])
code += r'''
void init(){
 regs={};dmaBusy=0;isoReady=false;isoChecks=0;
 s_Usbd.Flags=0;
 assert(AppEvtHandlerInit(nullptr,0));
 memset(ep0Mem,0xA5,sizeof(ep0Mem));
 s_Usbd.hQue=CFifoInit(queueMem,sizeof(queueMem),sizeof(nRFUsbdQue_t),true);
 s_Usbd.hEp0Que=CFifoInit(ep0Mem,sizeof(ep0Mem),sizeof(nRFEPPkt_t),true);
 assert(s_Usbd.hQue && s_Usbd.hEp0Que);
}
void retire(unsigned ep,bool in){
 const unsigned bit=ep+(in?0:16);
 auto &event=in?regs.EVENTS_ENDEPIN[ep]:regs.EVENTS_ENDEPOUT[ep];
 event=0;regs.EPSTATUS.bits=1U<<bit;
 assert(!nRFUsbdRetireDma(bit) && dmaBusy==0x82);
 event=1;
 assert(nRFUsbdRetireDma(bit) && !dmaBusy && !regs.EPSTATUS.bits && !event);
}
int main(int argc,char **argv){
 const bool fullAppEvt=argc==2 && !strcmp(argv[1],"--full-appevt");
 alignas(8) uint8_t data[192];
 for(unsigned i=0;i<sizeof(data);++i)data[i]=uint8_t(i);
 const int lengths[]={0,1,9,63,64,65,129};
 for(int length:lengths){
  init();dmaBusy=0x82;
  assert(UsbCtrlrEp0Send(0,data,length)==length);
  assert(regs.TASKS_STARTEPIN[0]==0);
  int offset=0,count=std::max(1,(length+63)/64);
  assert(CFifoUsed(s_Usbd.hEp0Que)==count);
  // A ready ISO request must wait until all staged EP0 packets have started.
  isoReady=true;dmaBusy=0;
  while(count){
   auto *packet=(nRFEPPkt_t*)CFifoPeek(s_Usbd.hEp0Que);
   const unsigned bytes=min(length-offset,64);
   nRFUsbdStartQueuedDma();
   assert(dmaBusy==0x82 && isoChecks==0);
   assert(regs.TASKS_STARTEPIN[0]==1 && regs.TASKS_STARTEPOUT[0]==0);
   assert(regs.EPIN[0].PTR==uint32_t(uintptr_t(packet->Payload)));
   assert(regs.EPIN[0].MAXCNT==bytes && !memcmp(packet->Payload,data+offset,bytes));
   assert(regs.SHORTS==(bytes<64?USBD_SHORTS_EP0DATADONE_EP0STATUS_Msk:0U));
   assert(CFifoUsed(s_Usbd.hEp0Que)==count);
   retire(0,true);offset+=bytes;
   assert(CFifoUsed(s_Usbd.hEp0Que)==--count);
  }
  nRFUsbdStartQueuedDma();assert(isoChecks==1);
 }
 puts("PASS: queued EP0 uses its compact header, packet lengths, payload and status SHORTS");

 alignas(8) uint8_t txMem[CFIFO_TOTAL_MEMSIZE(128,1)];
 for(unsigned ep=1;ep<8;++ep)for(unsigned kind=0;kind<4;++kind){
  init();auto fifo=CFifoInit(txMem,sizeof(txMem),1,true);
  int count=64;auto *head=CFifoPutMultiple(fifo,&count);assert(head && count==64);
  memcpy(head,data,count);
  auto *entry=(nRFUsbdQue_t*)CFifoPut(s_Usbd.hQue);
  entry->EpNum=ep;entry->Dir=kind;entry->Len=kind==3?3:64;
  const uint8_t *expected=data;
  if(kind==2){entry->hFifo=fifo;expected=head;}
  else if(kind==3){entry->Scratch=0xA5030201;expected=(uint8_t*)&entry->Scratch;}
  else entry->pBuffer=data;
  regs.SIZE.EPOUT[ep]=9;
  isoReady=true;nRFUsbdStartQueuedDma();
  assert(!dmaBusy && CFifoUsed(s_Usbd.hQue)==1);
  isoReady=false;nRFUsbdStartQueuedDma();
  assert(dmaBusy==0x82 && CFifoPeek(s_Usbd.hQue)==(uint8_t*)entry);
  const auto &dma=kind?regs.EPIN[ep]:regs.EPOUT[ep];
  assert(dma.PTR==uint32_t(uintptr_t(expected)));
  assert(dma.MAXCNT==(kind?entry->Len:9));
  assert((kind?regs.TASKS_STARTEPIN[ep]:regs.TASKS_STARTEPOUT[ep])==1);
  retire(ep,kind!=0);
  assert(CFifoUsed(s_Usbd.hQue)==0 && CFifoUsed(fifo)==64 && CFifoPeek(fifo)==head);
 }
 puts("PASS: ISO priority, regular DMA sources, OUT length and ENDEP queue ownership");

 // Three TX slots exercise physical wrap; repeated batches wrap hQue too.
 // Packet data stays owned even when ENDEP has freed the DMA queue slot.
 for(bool blocking:{false,true})for(unsigned mps:{9U,63U,64U}){
  init();
  const unsigned block=USB_INTRF_PKT_BLKSIZE(mps);
  alignas(8) uint8_t packetMem[USB_INTRF_RXMEM_SIZE(3,64)];
  alignas(8) uint8_t input[USB_INTRF_PKT_BLKSIZE(64)];
  UsbDevIntrf_t intrf={};
  intrf.DevIntrf.pDevData=&intrf;
  intrf.hTxFifo=CFifoInit(packetMem,CFIFO_TOTAL_MEMSIZE(3,block),block,blocking);
  intrf.Mps=mps;intrf.EpNo=1;intrf.Mode=USB_INTRF_MODE_PACKET;
  intrf.EpSend=UsbIntrfEpSendPktMode;
  UsbIntrfSetTxIdle(&intrf);
  auto &reg=s_Usbd.EpReg[0][1];
  reg={};reg.Handler=UsbIntrfCtrlrInEvent;reg.pContext=&intrf;
  const unsigned lengths[]={mps,mps-1,1,0};
  for(unsigned batch=0;batch<24;++batch){
   // Another endpoint owns DMA while packet submissions fill the TX FIFO.
   assert(UsbCtrlrEpInXfer(0,2,data,7));
   for(unsigned p=0;p<3;++p){
    memset(input,0x30+batch+p,block);
    reinterpret_cast<UsbPkt_t*>(input)->Hdr.Length=lengths[(batch+p)%4];
    assert(UsbIntrfTxPackets(&intrf.DevIntrf,input,block)==int(block));
   }
   auto *first=(UsbPkt_t*)CFifoPeek(intrf.hTxFifo);
   assert(CFifoUsed(intrf.hTxFifo)==3 && CFifoUsed(s_Usbd.hQue)==2);
   assert(!atomic_load(&intrf.DevIntrf.bTxReady));
   // Both blocking and nonblocking producers must preserve the in-flight head.
   assert(UsbIntrfTxPackets(&intrf.DevIntrf,input,block)==0);
   assert(CFifoPeek(intrf.hTxFifo)==(uint8_t*)first);
   retire(2,true);
   nRFUsbdResumeQueuedDmaLocked();
   for(unsigned p=0;p<3;++p){
    auto *packet=(UsbPkt_t*)CFifoPeek(intrf.hTxFifo);
    const unsigned len=lengths[(batch+p)%4];
    auto *entry=(nRFUsbdQue_t*)CFifoPeek(s_Usbd.hQue);
    assert(entry && entry->EpNum==1 && entry->Dir==NRFX_USBD_QUE_IN_BUFFER);
    assert(entry->pBuffer==packet->Data && entry->Len==len);
    assert((uintptr_t(packet->Data)&3U)==0 && packet->Hdr.Length==len);
    assert(regs.EPIN[1].PTR==uint32_t(uintptr_t(packet->Data)));
    assert(regs.EPIN[1].MAXCNT==len && dmaBusy==0x82);
    for(unsigned i=0;i<len;++i)assert(packet->Data[i]==uint8_t(0x30+batch+p));
    retire(1,true);
    assert(CFifoUsed(s_Usbd.hQue)==0);
    assert(CFifoUsed(intrf.hTxFifo)==int(3-p));
    assert(CFifoPeek(intrf.hTxFifo)==(uint8_t*)packet);
    // Reuse the released queue slot and DMA channel before host completion.
    assert(UsbCtrlrEpInXfer(0,2,data,7));
    if(fullAppEvt){
     while(AppEvtHandlerQue(0,nullptr,[](uint32_t,void*){})){}
    }
    nRFUsbdQueueInComplete(1,len);
    assert(CFifoPeek(intrf.hTxFifo)==(uint8_t*)packet);
    assert(CFifoUsed(intrf.hTxFifo)==int(3-p));
    AppEvtHandlerExec();
    assert(CFifoUsed(intrf.hTxFifo)==int(2-p));
    assert(CFifoUsed(s_Usbd.hQue)==(p<2?2:1));
    retire(2,true);
    nRFUsbdResumeQueuedDmaLocked();
   }
   assert(atomic_load(&intrf.DevIntrf.bTxReady));
   assert(CFifoUsed(s_Usbd.hQue)==0 && !dmaBusy);
  }
 }
 puts("PASS: packet IN full/short/ZLP, aligned ring wrap, queued DMA contention,");
 puts("      ENDEP preserves TX data, host completion releases exactly one packet");
}
'''
with tempfile.TemporaryDirectory(prefix='iosonata-queue-') as temp:
    path = Path(temp) / 'queue_test.cpp'
    path.write_text(code)
    binary = Path(temp) / 'queue_test'
    subprocess.run([os.environ.get('CXX', 'g++'), '-std=c++17', '-O1',
        '-fsanitize=undefined', '-fno-sanitize-recover=all',
        '-I'+str(ROOT/'include'), '-I'+str(ROOT/'tests/usb/hostport'),
        '-x', 'c++', str(path), str(ROOT/'src/cfifo.c'),
        str(ROOT/'src/app_evt_handler.cpp'), '-o', str(binary)], check=True)
    subprocess.run([str(binary), *sys.argv[1:]], check=True)
