#!/usr/bin/env python3
"""Exercise production queued DMA selection with real CFifo storage.

EP0 can wait behind regular DMA; its compact packet header is not a regular
queue header. This register simulation checks that handoff and DMA ownership.
Packet IN also runs the production UsbIntrf producer/completion and AppEvt
dispatch, so ENDEP and host-consumption ownership are checked separately.
OUT runs the production ISR and UsbIntrf RX completion against recycled DMA
entries with both blocking settings, including FIFO overflow and direct slots.

Use --full-appevt to fill AppEvt before each packet completion and verify
foreground retry without another USB interrupt.
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
intrf_source = Path(os.environ.get('USB_INTRF_SOURCE',
    ROOT / 'src/usb/usb_intrf.cpp')).read_text()
header = (ROOT / 'ARM/Nordic/include/usb_ctrlr.h').read_text()
queue_blocking = re.search(r's_Usbd.hQue = CFifoInit\(s_QueMem,.*?\b(true|false)\);',
    source, re.S).group(1)


def function(name, source=source):
    match = re.search(r'(?:void|bool|int|uint8_t|uint32_t|nRFUsbEpReg_t\s*\*)\s*' +
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
// Bypass the legacy host adapter; exercise the production directional API.
#define UsbCtrlrEpInXfer HostUsbCtrlrEpInXfer
#define UsbCtrlrEpOutXfer HostUsbCtrlrEpOutXfer
#include "usb/usb_intrf.h"
#undef UsbCtrlrEpInXfer
#undef UsbCtrlrEpOutXfer
bool nRFUsbdIsoXfer(uint8_t,uint16_t){assert(false);return false;}
uint32_t irqMask;
uint32_t DisableInterrupt(){auto old=irqMask;irqMask=1;return old;}
void EnableInterrupt(uint32_t old){irqMask=old;}
unsigned __CLZ(uint32_t value){assert(value);return __builtin_clz(value);}
using std::min;
constexpr int NRFX_USBD_MAX_PACKET_SIZE=64;
constexpr int NRFX_USBD_ISO_EP_NO=8;
constexpr int NRFX_USBD_DATA_EP_COUNT=8;
constexpr uint32_t NRFX_USBD_EASYDMA_BUSY_REG_BUSY=0x82;
constexpr uint32_t NRFX_USBD_EASYDMA_BUSY_REG_CLEAR=0;
constexpr uint32_t USBD_SHORTS_EP0DATADONE_EP0STATUS_Msk=1;
constexpr uint32_t USBD_INTEN_USBRESET_Msk=1U,USBD_INTEN_USBEVENT_Msk=1U<<22;
constexpr uint32_t USBD_INTEN_EPDATA_Msk=1U<<24,USBD_INTEN_EP0SETUP_Msk=1U<<23;
constexpr uint32_t USBD_INTEN_EP0DATADONE_Msk=1U<<10,USBD_INTEN_ENDEPOUT0_Msk=1U<<12;
uint32_t dmaBusy;
#define NRFX_USBD_EASYDMA_BUSY_REG dmaBusy
struct Endpoint {uint32_t PTR,MAXCNT,AMOUNT;};
using USBD_EPIN_Type=Endpoint;
using USBD_EPOUT_Type=Endpoint;
struct W1C {
 uint32_t bits;
 operator uint32_t() const{return bits;}
 void operator=(uint32_t value){bits&=~value;}
};
struct NRF_USBD_Type {
 Endpoint EPIN[8],EPOUT[8];
 uint32_t beforeTasks,TASKS_STARTEPIN[8],TASKS_STARTISOIN;
 uint32_t TASKS_STARTEPOUT[8],TASKS_STARTISOOUT,afterTasks;
 uint32_t EVENTS_ENDEPIN[8],EVENTS_ENDEPOUT[8],EVENTS_EP0DATADONE,SHORTS,EVENTS_EPDATA;
 uint32_t EVENTS_EP0SETUP,EVENTS_USBEVENT,EVENTS_SOF,EVENTS_USBRESET;
 uint32_t EPOUTEN,EPINEN,INTEN,INTENCLR,INTENSET;
 struct {uint32_t EPOUT[8];} SIZE;
 W1C EPSTATUS,EPDATASTATUS,EVENTCAUSE;
} regs;
auto *NRF_USBD=&regs;
void __DSB(){}
void UsbdSync(){}
bool isoReady;
unsigned isoChecks;
bool nRFUsbdIsoStart(){++isoChecks;return isoReady;}
bool nRFUsbdIsoFinishDma(uint32_t){assert(false);return false;}
void nRFUsbdQueueEp0Complete(bool,uint16_t){assert(false);}
void nRFUsbdHandleBusEvent(uint32_t){assert(false);}
void nRFUsbdProcessEP0Setup(uint32_t,void*){assert(false);}
void nRFUsbdHandleSof(){assert(false);}
void nRFUsbdTryRemoteWake(){}
void nRFUsbdTryEnterLowPower(){}
unsigned resets;
void nRFUsbdEmitSimple(UsbCtrlrEvtType_t event){assert(event==USB_CTRLR_EVT_RESET);++resets;}
'''
queue_enum = re.search(r'enum\s*\{[^}]*NRFX_USBD_QUE_IN_SCRATCH[^}]*\};', source)
assert queue_enum
code += queue_enum.group(0) + '\n#pragma pack(push,4)\n'
code += 'constexpr bool dmaQueueBlocking=' + queue_blocking + ';\n'
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
 uint32_t IsoGeneration[2];
 uint16_t IsoOutSize;
 nRFUsbEpReg_t EpReg[8][2];
 hCFifo_t hQue,hEp0Que;
 struct {struct {uint16_t TotalLen;} Ep0[2];} Ctrlr;
 alignas(4) uint8_t Ep0Bounce[64];
} s_Usbd;
alignas(8) uint8_t queueMem[CFIFO_TOTAL_MEMSIZE(16,sizeof(nRFUsbdQue_t))];
alignas(8) uint8_t ep0Mem[CFIFO_TOTAL_MEMSIZE(4,sizeof(nRFEPPkt_t))];
uint8_t *checkedDmaQueuePut(hCFifo_t fifo){
 assert(irqMask==1); // Queue publication and initialization exclude the ISR.
 return CFifoPut(fifo);
}
'''
code += '\n'.join(function(name).replace('CFifoPut(', 'checkedDmaQueuePut(')
    if name in ('nRFUsbdQueXferDir', 'UsbCtrlrEpInXfer') else function(name)
    for name in [
    'nRFUsbdDmaActive', 'nRFUsbdDmaUnlock', 'nRFUsbdDmaStartLocked', 'nRFUsbdRetireDma',
    'nRFUsbdEp0InProgram', 'nRFUsbdStartDmaNow', 'nRFUsbdStartQueuedDma',
    'nRFUsbdResumeQueuedDmaLocked', 'nRFUsbdResumeQueuedDma', 'nRFUsbdQueXferDir', 'UsbCtrlrEpXfer',
    'UsbCtrlrEpInXfer', 'UsbCtrlrEpOutXfer',
    'nRFUsbEpDir', 'nRFUsbGetEpReg', 'nRFUsbEpRegisteredEvent',
    'nRFUsbdProcessInComplete', 'nRFUsbdQueueInComplete', 'UsbCtrlrEp0Send',
    'UsbCtrlrEpAlloc', 'nRFUsbdProcessOutData', 'UsbCtrlrProcess',
    'nRFUsbdResetState', 'nRFUsbdBusReset', 'USBD_IRQHandler'])
code += re.search(r'static constexpr uint16_t USB_INTRF_RX_DRDY[^;]+;', intrf_source).group(0)
code += '\nvoid UsbIntrfCtrlrOutEvent(uint8_t,UsbCtrlrEvtType_t,uint16_t,UsbCtrlrXferResult_t,void*);\n'
code += '\n'.join(function(name, intrf_source) for name in [
    'UsbIntrfSetTxIdle', 'UsbIntrfTakeTx', 'UsbIntrfDirectClear',
    'UsbIntrfTxFailure', 'UsbIntrfEpSendPktMode', 'UsbIntrfTxPackets',
    'UsbIntrfCtrlrInEvent', 'UsbIntrfDirectReady', 'UsbIntrfDirectRxComplete',
    'UsbIntrfRegisterRx', 'UsbIntrfReleaseRx', 'UsbIntrfCompleteRx', 'UsbIntrfRetryRx',
    'UsbIntrfCtrlrOutEvent', 'UsbIntrfRxData', 'UsbIntrfRxDirect', 'UsbIntrfUnconfigure'])
# Exercise the actual ISR acknowledgement block, not a hand-coded queue call.
start = source.index('if (NRF_USBD->EVENTS_EPDATA != 0U ||')
end = source.index('{', start) + 1
depth = 1
while depth:
    depth += (source[end] == '{') - (source[end] == '}')
    end += 1
code += '\nvoid dataEvent(){const auto state=DisableInterrupt();uint8_t outEp=0;\n'
code += source[start:end] + '\nEnableInterrupt(state);}\n'
code += r'''
void init(){
 regs={};dmaBusy=0;isoReady=false;isoChecks=0;irqMask=0;resets=0;
 s_Usbd.Flags=0;memset(s_Usbd.EpReg,0,sizeof(s_Usbd.EpReg));
 assert(AppEvtHandlerInit(nullptr,0));
 memset(ep0Mem,0xA5,sizeof(ep0Mem));
 s_Usbd.hQue=CFifoInit(queueMem,sizeof(queueMem),sizeof(nRFUsbdQue_t),dmaQueueBlocking);
 s_Usbd.hEp0Que=CFifoInit(ep0Mem,sizeof(ep0Mem),sizeof(nRFEPPkt_t),true);
 assert(s_Usbd.hQue && s_Usbd.hEp0Que);
}
void interrupt(){const auto state=DisableInterrupt();USBD_IRQHandler();EnableInterrupt(state);}
void outComplete(uint8_t ep,UsbCtrlrEvtType_t event,uint16_t len,
 UsbCtrlrXferResult_t result,void *context){
 assert(event==USB_CTRLR_EVT_XFER_CMPL && result==USB_CTRLR_XFER_SUCCESS);
 UsbIntrfCtrlrOutEvent(ep,event,len,result,context);
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

 // Public submissions own exclusion through queue initialization and DMA
 // restart, and restore the caller's mask on both success and refusal.
 for(unsigned masked:{0U,1U}){
  init();irqMask=masked;
  s_Usbd.EpReg[1][1].pBuffer=data;
  assert(UsbCtrlrEpXfer(0,0x82,9) && irqMask==masked && dmaBusy);
  auto *entry=(nRFUsbdQue_t*)CFifoPeek(s_Usbd.hQue);
  assert(entry->EpNum==2 && entry->Dir==NRFX_USBD_QUE_IN_BUFFER);
  assert(entry->Len==9 && entry->pBuffer==data);
  s_Usbd.EpReg[0][0].pBuffer=data+64;
  assert(UsbCtrlrEpOutXfer(0,1,64) && irqMask==masked);
  assert(CFifoUsed(s_Usbd.hQue)==2);
  assert(!UsbCtrlrEpOutXfer(0,3,64) && irqMask==masked);
  assert(CFifoUsed(s_Usbd.hQue)==2); // A withheld buffer publishes no entry.
  retire(2,true);
  entry=(nRFUsbdQue_t*)CFifoPeek(s_Usbd.hQue);
  assert(entry->EpNum==1 && entry->Dir==NRFX_USBD_QUE_OUT);
  assert(entry->Len==64 && entry->pBuffer==data+64);
 }
 puts("PASS: public DMA submission holds exclusion and restores the caller's IRQ mask");

 // Poison every recycled DMA slot with a different destination. The actual
 // OUT ISR must replace it before DMA starts, including when IN owns DMA.
 for(bool blocking:{false,true})
 for(auto mode:{USB_INTRF_MODE_BYTE,USB_INTRF_MODE_PACKET,USB_INTRF_MODE_DIRECT})
 for(unsigned ep=1;ep<8;++ep){
  if(blocking && mode!=USB_INTRF_MODE_DIRECT)continue; // Covered by lossless overflow below.
  init();
  alignas(8) uint8_t decoy[64];memset(decoy,0xA5,sizeof(decoy));
  for(unsigned slot=0;slot<16;++slot){
   auto *q=(nRFUsbdQue_t*)CFifoPut(s_Usbd.hQue);
   q->EpNum=7;q->Dir=NRFX_USBD_QUE_IN_BUFFER;q->Len=64;q->pBuffer=decoy;
   assert(CFifoGet(s_Usbd.hQue)==(uint8_t*)q);
  }
  alignas(8) struct {uint32_t before;uint8_t slot[USB_INTRF_PKT_BLKSIZE(64)];uint32_t after;} rx={};
  rx.before=rx.after=0xAC1357DE;
  auto *direct=(UsbPkt_t*)rx.slot;
  alignas(8) uint8_t rxMem[USB_INTRF_RXMEM_SIZE(3,64)];
  UsbDevIntrf_t intrf={};
  intrf.DevIntrf.pDevData=&intrf;intrf.Mode=mode;intrf.EpNo=ep;intrf.Mps=64;
  intrf.bBlocking=blocking;
  intrf.hRxFifo=CFifoInit(rxMem,sizeof(rxMem),USB_INTRF_PKT_BLKSIZE(64),blocking);
  intrf.pRxBuffer=direct->Data;
  if(mode==USB_INTRF_MODE_DIRECT)intrf.pRxDirectBuffer=direct;
  UsbCtrlrEpAlloc(0,ep,intrf.pRxBuffer,blocking,outComplete,&intrf);
  s_Usbd.EpReg[ep-1][0].MaxPacketSize=64;
  const unsigned received[]={0,1,9,63,64};
  auto receive=[&](unsigned len,unsigned value){
   assert(UsbCtrlrEpInXfer(0,7,data,7));
   regs.SIZE.EPOUT[ep]=len;regs.EPDATASTATUS.bits=1U<<(ep+16);
   regs.EVENTS_EPDATA=1;interrupt();
   assert(CFifoUsed(s_Usbd.hQue)==2 && !regs.EPDATASTATUS.bits);
   assert(!intrf.RxPending);
   regs.EPSTATUS.bits=1U<<7;regs.EVENTS_ENDEPIN[7]=1;interrupt();
   auto *q=(nRFUsbdQue_t*)CFifoPeek(s_Usbd.hQue);
   assert(q && q->EpNum==ep && q->Dir==NRFX_USBD_QUE_OUT && q->Len==64);
   assert(q->pBuffer==intrf.pRxBuffer);
   assert(regs.EPOUT[ep].PTR==uint32_t(uintptr_t(intrf.pRxBuffer)));
   assert(regs.EPOUT[ep].MAXCNT==len && dmaBusy==0x82);
   memset(q->pBuffer,value,len); // Simulated host payload through EasyDMA.
   regs.EPOUT[ep].AMOUNT=len;regs.EPSTATUS.bits=1U<<(ep+16);
   regs.EVENTS_ENDEPOUT[ep]=1;interrupt();
   assert(!dmaBusy && CFifoUsed(s_Usbd.hQue)==0);
   assert(!regs.EVENTS_ENDEPOUT[ep] && !regs.EPSTATUS.bits);
   assert(rx.before==0xAC1357DE && rx.after==0xAC1357DE);
   for(auto byte:decoy)assert(byte==0xA5);
  };
  for(unsigned packet=0;packet<20;++packet){
   const unsigned len=received[packet%5];
   receive(len,0x80+packet);
   if(mode==USB_INTRF_MODE_DIRECT){
    assert(UsbIntrfDirectReady(direct) && direct->Hdr.Length==len);
    assert(intrf.RxDropCnt==packet);
   }else{
    assert(CFifoUsed(intrf.hRxFifo)==int(std::min(packet+1,3U)));
    auto *oldest=(UsbPkt_t*)CFifoPeek(intrf.hRxFifo);
    assert(intrf.RxDropCnt==(blocking && packet>=3?packet-2:0));
    const unsigned first=blocking || packet<3?0:packet-2;
    assert(oldest->Hdr.Length==received[first%5]);
    for(unsigned i=0;i<oldest->Hdr.Length;++i)assert(oldest->Data[i]==0x80+first);
   }
  }
  uint8_t output[192]={};
  if(mode==USB_INTRF_MODE_DIRECT){
   assert(UsbIntrfRxDirect(&intrf.DevIntrf,output,sizeof(output))==64);
   for(unsigned i=0;i<64;++i)assert(output[i]==0x80+19);
   assert(!UsbIntrfDirectReady(direct));
  }else{
   assert(intrf.hRxFifo->DropCnt==(blocking?0:17));
   assert(UsbIntrfRxData(&intrf.DevIntrf,output,sizeof(output))==(blocking?0+1+9:9+63+64));
   unsigned offset=0;
   const unsigned first=blocking?0:17;
   for(unsigned packet=first;packet<first+3;++packet)
    for(unsigned i=0;i<received[packet%5];++i)assert(output[offset++]==0x80+packet);
   assert(CFifoUsed(intrf.hRxFifo)==0);
  }
  // Reading after overflow needs no DRDY restart. The next arrival is DMAed
  // normally and accepted now that the FIFO/direct slot is empty.
  assert(!intrf.RxPending && !dmaBusy && CFifoUsed(s_Usbd.hQue)==0);
  receive(64,0xE1);
  assert((mode==USB_INTRF_MODE_DIRECT?
   UsbIntrfRxDirect(&intrf.DevIntrf,output,sizeof(output)):
   UsbIntrfRxData(&intrf.DevIntrf,output,sizeof(output)))==64);
  for(unsigned i=0;i<64;++i)assert(output[i]==0xE1);
 }
 puts("PASS: OUT EP1-7 queues DMA without DRDY in byte, packet and direct modes");
 puts("      full/short/ZLP, nonblocking FIFO/direct replacement and recovery");

 // Reproduce the hardware failure: four unread packets fill RX; the fifth
 // completed packet must wait in its DMA buffer rather than disappear.
 for(auto mode:{USB_INTRF_MODE_BYTE,USB_INTRF_MODE_PACKET})
 for(unsigned ep:{1U,7U})
 for(unsigned length:{0U,1U,9U,63U,64U})for(bool fullEvents:{false,true}){
  init();
  alignas(8) uint8_t rxMem[USB_INTRF_RXMEM_SIZE(4,64)];
  alignas(4) uint8_t rx[64]={},other[64]={};
  UsbDevIntrf_t intrf={};
  intrf.DevIntrf.pDevData=&intrf;intrf.EpNo=ep;intrf.Mps=64;
  intrf.Mode=mode;intrf.bBlocking=true;intrf.pRxBuffer=rx;
  intrf.hRxFifo=CFifoInit(rxMem,sizeof(rxMem),USB_INTRF_PKT_BLKSIZE(64),true);
  UsbCtrlrEpAlloc(0,ep,rx,true,outComplete,&intrf);
  s_Usbd.EpReg[ep-1][0].MaxPacketSize=64;
  const uint32_t outBit=1U<<(ep+16);
  auto finish=[&](unsigned len,unsigned value){
   assert(dmaBusy && regs.EPOUT[ep].PTR==uint32_t(uintptr_t(rx)));
   memset(rx,value,len);
   regs.EPOUT[ep].AMOUNT=len;regs.EPSTATUS.bits=outBit;
   regs.EVENTS_ENDEPOUT[ep]=1;interrupt();
  };
  auto receive=[&](unsigned len,unsigned value){
   regs.SIZE.EPOUT[ep]=len;regs.EPDATASTATUS.bits|=outBit;
   regs.EVENTS_EPDATA=1;interrupt();finish(len,value);
  };
  for(unsigned p=0;p<4;++p)receive(64,0xA0+p);
  if(fullEvents)while(AppEvtHandlerQue(0,nullptr,[](uint32_t,void*){})){}
  receive(length,0xE5);
  assert(intrf.RxPending==length+2 && !intrf.RxDropCnt);
  assert(s_Usbd.EpReg[ep-1][0].pBuffer==nullptr);
  assert(!dmaBusy && CFifoUsed(s_Usbd.hQue)==0 && CFifoUsed(intrf.hRxFifo)==4);
  UsbCtrlrProcess(0); // A retry while RX is still full must remain pending.
  if(fullEvents)while(AppEvtHandlerQue(0,nullptr,[](uint32_t,void*){})){}
  // The next host packet remains in the endpoint. It cannot overwrite E5,
  // even when both the RX FIFO and AppEvt are full.
  regs.TASKS_STARTEPOUT[ep]=0;
  regs.SIZE.EPOUT[ep]=64;regs.EPDATASTATUS.bits|=outBit;
  regs.EVENTS_EPDATA=1;interrupt();
  UsbCtrlrProcess(0);interrupt();
  assert((regs.EPDATASTATUS.bits&outBit) && !regs.TASKS_STARTEPOUT[ep]);
  assert(!dmaBusy && CFifoUsed(s_Usbd.hQue)==0);
  for(unsigned i=0;i<length;++i)assert(rx[i]==0xE5);
  // Unrelated OUT and IN endpoints continue while either EP1 or EP7 is held.
  unsigned otherComplete=0;
  UsbCtrlrEpAlloc(0,2,other,true,
   [](uint8_t ep,UsbCtrlrEvtType_t evt,uint16_t len,UsbCtrlrXferResult_t,void *ctx){
    assert(ep==2 && evt==USB_CTRLR_EVT_XFER_CMPL && len==1);++*(unsigned*)ctx;
   },&otherComplete);
  s_Usbd.EpReg[1][0].MaxPacketSize=64;
  regs.SIZE.EPOUT[2]=1;regs.EPDATASTATUS.bits|=1U<<18;regs.EVENTS_EPDATA=1;
  interrupt();UsbCtrlrProcess(0);
  assert(regs.EPOUT[2].PTR==uint32_t(uintptr_t(other)));
  regs.EPOUT[2].AMOUNT=1;regs.EPSTATUS.bits=1U<<18;regs.EVENTS_ENDEPOUT[2]=1;
  interrupt();assert(otherComplete==1);
  assert(UsbCtrlrEpInXfer(0,3,data,7));retire(3,true);
  uint8_t output[256]={};
  assert(UsbIntrfRxData(&intrf.DevIntrf,output,64)==64);
  for(unsigned i=0;i<64;++i)assert(output[i]==0xA0);
  assert(!intrf.RxPending && !intrf.RxDropCnt && CFifoUsed(intrf.hRxFifo)==4);
  assert(s_Usbd.EpReg[ep-1][0].pBuffer==rx);
  UsbCtrlrProcess(0);
  assert(!regs.EPDATASTATUS.bits && CFifoUsed(s_Usbd.hQue)==1);
  assert(dmaBusy && regs.TASKS_STARTEPOUT[ep]);
  assert(UsbIntrfRxData(&intrf.DevIntrf,output,sizeof(output))==int(192+length));
  for(unsigned p=0;p<3;++p)for(unsigned i=0;i<64;++i)assert(output[p*64+i]==0xA1+p);
  for(unsigned i=0;i<length;++i)assert(output[192+i]==0xE5);
  assert(CFifoUsed(intrf.hRxFifo)==0);
  finish(64,0xF6);
  assert(UsbIntrfRxData(&intrf.DevIntrf,output,64)==64);
  for(unsigned i=0;i<64;++i)assert(output[i]==0xF6);
  UsbCtrlrProcess(0);UsbCtrlrProcess(0);
  assert(!intrf.RxDropCnt && !intrf.RxPending && CFifoUsed(intrf.hRxFifo)==0);
  assert(!dmaBusy && CFifoUsed(s_Usbd.hQue)==0); // No duplicate AppEvt enqueue/copy.

  // Cancellation releases the held buffer; queued retry callbacks become no-ops.
  for(unsigned p=0;p<4;++p)receive(64,0xB0+p);
  receive(length,0xE7);assert(intrf.RxPending==length+2);
  UsbIntrfUnconfigure(&intrf);AppEvtHandlerExec();
  assert(!intrf.RxPending && CFifoUsed(intrf.hRxFifo)==0);
  assert(s_Usbd.EpReg[ep-1][0].pBuffer==rx);
  // Exercise a failed put under the interface's nonblocking policy.
  intrf.Mps=64;intrf.bBlocking=false;
  for(unsigned p=0;p<4;++p)receive(64,0xC0+p);
  receive(length,0xE8);AppEvtHandlerExec();
  assert(intrf.RxDropCnt==1 && !intrf.RxPending);
  assert(s_Usbd.EpReg[ep-1][0].pBuffer==rx && CFifoUsed(intrf.hRxFifo)==4);
 }
 puts("PASS: blocking RX overflow retries full/short/ZLP without loss or buffer overwrite");
 puts("      other endpoints progress; full AppEvt recovers; cancellation and nonblocking rejection");

 // Force saturation even though sixteen entries exceed the fourteen regular
 // endpoint directions. Retry must preserve the active inline scratch and
 // enqueue exactly one OUT once ENDEP releases a slot.
 for(bool blocking:{false,true})for(unsigned len:{0U,1U,63U,64U}){
  init();dmaBusy=0x82;
  for(unsigned slot=0;slot<16;++slot){
   auto *q=(nRFUsbdQue_t*)CFifoPut(s_Usbd.hQue);
   q->EpNum=7;q->Dir=NRFX_USBD_QUE_IN_SCRATCH;q->Len=3;q->Scratch=0xA5010203+slot;
  }
  auto *head=(nRFUsbdQue_t*)CFifoPeek(s_Usbd.hQue);
  const auto get=s_Usbd.hQue->GetIdx,put=s_Usbd.hQue->PutIdx;
  alignas(4) uint8_t out[64]={};
  unsigned completions=0;
  UsbCtrlrEpAlloc(0,1,out,blocking,
   [](uint8_t ep,UsbCtrlrEvtType_t event,uint16_t,
      UsbCtrlrXferResult_t result,void *context){
    assert(ep==1 && event==USB_CTRLR_EVT_XFER_CMPL && result==USB_CTRLR_XFER_SUCCESS);
    ++*(unsigned*)context;
   },&completions);
  s_Usbd.EpReg[0][0].MaxPacketSize=64;
  regs.SIZE.EPOUT[1]=len;regs.EPDATASTATUS.bits=1U<<17;regs.EVENTS_EPDATA=1;
  interrupt();
  assert(regs.EPDATASTATUS.bits==(1U<<17) && !regs.TASKS_STARTEPOUT[1]);
  AppEvtHandlerDispatch(); // Still full: retry must put itself back in AppEvt.
  AppEvtHandlerExec(); // Remains bounded while DMA has not finished.
  assert(!UsbCtrlrEpInXfer(0,2,data,7) && !irqMask);
  assert(!UsbCtrlrEpOutXfer(0,2,64) && !irqMask);
  assert(s_Usbd.hQue->GetIdx==get && s_Usbd.hQue->PutIdx==put);
  assert(CFifoPeek(s_Usbd.hQue)==(uint8_t*)head && head->Scratch==0xA5010203);
  assert(!s_Usbd.hQue->DropCnt && completions==0);
  retire(7,true);
  AppEvtHandlerDispatch(); // One free slot accepts the deferred OUT.
  assert(CFifoUsed(s_Usbd.hQue)==16 && s_Usbd.hQue->PutIdx==put+1);
  assert(!regs.EPDATASTATUS.bits);
  for(unsigned slot=1;slot<16;++slot){
   auto *q=(nRFUsbdQue_t*)CFifoPeek(s_Usbd.hQue);
   assert(q && q->EpNum==7 && q->Scratch==0xA5010203+slot);
   retire(7,true);nRFUsbdResumeQueuedDmaLocked();
  }
  auto *q=(nRFUsbdQue_t*)CFifoPeek(s_Usbd.hQue);
  assert(q && q->EpNum==1 && q->Dir==NRFX_USBD_QUE_OUT && q->pBuffer==out);
  assert(regs.EPOUT[1].PTR==uint32_t(uintptr_t(out)) && regs.EPOUT[1].MAXCNT==len);
  regs.EPOUT[1].AMOUNT=len;regs.EPSTATUS.bits=1U<<17;regs.EVENTS_ENDEPOUT[1]=1;
  interrupt();AppEvtHandlerExec();
  assert(completions==1 && !dmaBusy && CFifoUsed(s_Usbd.hQue)==0);
 }
 puts("PASS: full DMA queue preserves active scratch; AppEvt retries OUT once space opens");

 // The reset loop must cover all eighteen start tasks and no adjacent task.
 init();dmaBusy=0x82;
 for(unsigned ep=0;ep<8;++ep)regs.TASKS_STARTEPIN[ep]=regs.TASKS_STARTEPOUT[ep]=0xFFFFFFFF;
 regs.TASKS_STARTISOIN=regs.TASKS_STARTISOOUT=0xFFFFFFFF;
 regs.beforeTasks=0x12345678;regs.afterTasks=0x87654321;
 regs.EPSTATUS.bits=0x01010101;regs.EPDATASTATUS.bits=0x01FF01FF;
 regs.EVENTCAUSE.bits=0xFFFF;regs.EVENTS_USBEVENT=1;regs.INTEN=0xFFFFFFFF;
 s_Usbd.IsoGeneration[0]=4;s_Usbd.IsoGeneration[1]=8;s_Usbd.IsoOutSize=33;
 assert(CFifoPut(s_Usbd.hQue) && CFifoPut(s_Usbd.hEp0Que));
 regs.EVENTS_USBRESET=1;interrupt();
 assert(resets==1 && !dmaBusy && !regs.EVENTS_USBRESET);
 for(unsigned ep=0;ep<8;++ep)assert(!regs.TASKS_STARTEPIN[ep] && !regs.TASKS_STARTEPOUT[ep]);
 assert(!regs.TASKS_STARTISOIN && !regs.TASKS_STARTISOOUT);
 assert(regs.beforeTasks==0x12345678 && regs.afterTasks==0x87654321);
 assert(!regs.EPSTATUS.bits && !regs.EPDATASTATUS.bits && !regs.EVENTCAUSE.bits);
 assert(!regs.EVENTS_USBEVENT && regs.EPOUTEN==1 && regs.EPINEN==1);
 assert(regs.INTENCLR==0xFFFFFFFF && regs.INTENSET==(USBD_INTEN_USBRESET_Msk |
  USBD_INTEN_USBEVENT_Msk | USBD_INTEN_EPDATA_Msk | USBD_INTEN_EP0SETUP_Msk |
  USBD_INTEN_EP0DATADONE_Msk | USBD_INTEN_ENDEPOUT0_Msk));
 assert(CFifoUsed(s_Usbd.hQue)==0 && CFifoUsed(s_Usbd.hEp0Que)==0);
 assert(s_Usbd.Flags==USBD_FLAG_MAC_AWAKE && !s_Usbd.IsoOutSize);
 assert(s_Usbd.IsoGeneration[0]==5 && s_Usbd.IsoGeneration[1]==9);
 puts("PASS: bus reset clears all regular/ISO start tasks and preserves adjacent registers");

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
    regs.EPIN[1].AMOUNT=len;
    regs.EPDATASTATUS.bits=1U<<1;
    regs.EVENTS_EPDATA=1;
    dataEvent();
    assert(!regs.EVENTS_EPDATA);
    assert(regs.EPDATASTATUS.bits==(fullAppEvt?1U<<1:0U));
    dataEvent(); // A repeated interrupt must not duplicate accepted completion.
    assert(CFifoPeek(intrf.hTxFifo)==(uint8_t*)packet);
    assert(CFifoUsed(intrf.hTxFifo)==int(3-p));
    UsbCtrlrProcess(0);
    assert(!regs.EPDATASTATUS.bits && !irqMask);
    if(fullAppEvt){
     // First pass drained AppEvt and published the retained completion.
     assert(CFifoUsed(intrf.hTxFifo)==int(3-p));
    }
    UsbCtrlrProcess(0);
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

 // Seven pending IN endpoints exceed the default AppEvt capacity of four.
 // A full AppEvt queue must leave all seven latched; OUT retry shares the status register.
 init();
 unsigned calls[8]={},amounts[8]={};
 struct Completion {unsigned *calls,*amounts;} completion={calls,amounts};
 for(unsigned ep=1;ep<8;++ep){
  auto &reg=s_Usbd.EpReg[ep-1][1];reg.pContext=&completion;
  reg.Handler=[](uint8_t ep,UsbCtrlrEvtType_t event,uint16_t length,
                UsbCtrlrXferResult_t result,void *context){
   assert(!irqMask && event==USB_CTRLR_EVT_XFER_CMPL && result==USB_CTRLR_XFER_SUCCESS);
   assert(USB_ENDPADDR_IS_IN(ep));ep=USB_ENDPADDR_NUM(ep);
   auto *c=(Completion*)context;++c->calls[ep];c->amounts[ep]=length;
  };
  regs.EPIN[ep].AMOUNT=ep==7?0:ep*9;
 }
 while(AppEvtHandlerQue(0,nullptr,[](uint32_t,void*){})){}
 s_Usbd.EpReg[3][0].pBuffer=data;s_Usbd.EpReg[1][0].pBuffer=data+64;
 regs.EPDATASTATUS.bits=0xFEU | (1U<<20) | (1U<<18) | 0x10001U;
 regs.EVENTS_EPDATA=1;dataEvent();
 assert(regs.EPDATASTATUS.bits==(0xFEU | (1U<<20) | (1U<<18)));
 // No further USB interrupt is needed; IN acknowledgements must not erase OUT requests.
 for(unsigned pass=0;pass<10;++pass)UsbCtrlrProcess(0);
 assert(!regs.EPDATASTATUS.bits && CFifoUsed(s_Usbd.hQue)==2);
 for(unsigned ep=1;ep<8;++ep){
  assert(calls[ep]==1 && amounts[ep]==(ep==7?0:ep*9));
 }
 // Clear-on-close/reset discards a retained hardware completion.
 while(AppEvtHandlerQue(0,nullptr,[](uint32_t,void*){})){}
 regs.EPDATASTATUS.bits=1U<<3;dataEvent();
 assert(regs.EPDATASTATUS.bits==(1U<<3));
 regs.EPDATASTATUS=1U<<3;
 UsbCtrlrProcess(0);UsbCtrlrProcess(0);
 assert(calls[3]==1 && !regs.EPDATASTATUS.bits);
 puts("PASS: full AppEvt retry drains all seven IN completions once, also queues retained OUT requests once,");
 puts("      and honors cancellation by clearing the retained hardware status");
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
