#!/usr/bin/env python3
"""Run the SAM4L controller against a single-bank USBC register/DMA model.

MMIO lvalues and host-to-DMA address encoding are replaced. Production endpoint functions and the real
SAM4L register definitions are compiled. This checks ownership, W1C ordering,
endpoint redirection and control stages; it cannot validate USB wire timing.
"""
from pathlib import Path
import os
import re
import subprocess
import tempfile

ROOT = Path(__file__).resolve().parents[2]
PORT = ROOT / 'ARM/Microchip/SAM4L'
source = (PORT / 'src/usb_ctrlr_sam4l.cpp').read_text()
source, count = re.subn(
    r'static inline volatile uint32_t &Sam4lUsbEpReg.*?\n\}',
    'static Register &Sam4lUsbEpReg(uint32_t Offset, uint8_t Ep)\n'
    '{ return reg(Offset + Ep * 4U); }', source, count=1, flags=re.S)
assert count == 1
source = source.replace('reinterpret_cast<uintptr_t>(', 'encodeDmaAddress(')
fields = sorted(set(re.findall(r'SAM4L_USBC->(USBC_\w+)', source)))
HEADER = r'''
#pragma once
#include <cstdint>
#include <cstddef>
typedef uint8_t RoReg8;
typedef uint32_t RoReg;
typedef uint32_t WoReg;
typedef uint32_t RwReg;
#include "component/component_usbc.h"
#include "component/component_pm.h"
#include "component/component_bpm.h"
extern Pm pm;
extern Bpm bpm;
#define SAM4L_PM (&pm)
#define SAM4L_BPM (&bpm)
struct Register {
 uint32_t offset, value;
 operator uint32_t() const;
 Register &operator=(uint32_t value);
 Register &operator|=(uint32_t value) { return *this = uint32_t(*this) | value; }
 Register &operator&=(uint32_t value) { return *this = uint32_t(*this) & value; }
};
Register &reg(unsigned);
uint32_t encodeDmaAddress(const volatile void *);
const volatile void *dmaPointer(uint32_t);
struct UsbRegisters {
''' + ''.join(' Register &%s = reg(%s_OFFSET);\n' % (f, f) for f in fields) + r'''
};
extern UsbRegisters usbRegisters;
#define SAM4L_USBC (&usbRegisters)
struct ClockRegisters { struct { uint32_t SCIF_GCCTRL; } SCIF_GCCTRL[8]; };
extern ClockRegisters clocks;
#define SAM4L_SCIF (&clocks)
#define SCIF_GCCTRL_CEN 1U
#define USBC_IRQn 18
#define __NVIC_PRIO_BITS 4
extern uint32_t SystemCoreClock;
inline void NVIC_DisableIRQ(int) {}
inline void NVIC_EnableIRQ(int) {}
inline void NVIC_SetPriority(int, int) {}
inline void NVIC_ClearPendingIRQ(int) {}
inline void NVIC_SetPendingIRQ(int) {}
inline void __DMB() {}
inline void __DSB() {}
'''
MODEL = r'''
#include <cassert>
#include <cstdio>
#include <cstring>
#include <vector>
#include "sam4lxxx.h"
static Register registers[1024];
static std::vector<const volatile void *> dmaPointers;
uint32_t encodeDmaAddress(const volatile void *pointer) {
 for (unsigned i = 0; i < dmaPointers.size(); ++i)
  if (dmaPointers[i] == pointer) return ((i + 1) << 2) | (uintptr_t(pointer) & 3U);
 dmaPointers.push_back(pointer); return (dmaPointers.size() << 2) | (uintptr_t(pointer) & 3U);
}
const volatile void *dmaPointer(uint32_t address) {
 address >>= 2;
 assert(address > 0 && address <= dmaPointers.size());
 return dmaPointers[address - 1];
}
Register &reg(unsigned offset) {
 assert(offset / 4 < 1024); auto &r = registers[offset / 4];
 r.offset = offset; return r;
}
static uint32_t get(unsigned offset) { return reg(offset).value; }
static void set(unsigned offset, uint32_t value) { reg(offset).value = value; }
static void add(unsigned offset, uint32_t value) { reg(offset).value |= value; }
UsbRegisters usbRegisters;
ClockRegisters clocks;
Pm pm;
Bpm bpm;
uint32_t SystemCoreClock = 48000000;
bool boardVbus = true;
static unsigned releases[8];
static bool bankOwned[8];
static bool injectFirstOut;
Register::operator uint32_t() const {
 if (offset != USBC_UDINT_OFFSET) return value;
 uint32_t pending = value & 0x7FU;
 for (unsigned i = 0; i < 8; ++i)
  if (get(0x130 + i * 4) & get(0x1C0 + i * 4) & 0x85FU)
   pending |= 1U << (12 + i);
 return pending;
}
Register &Register::operator=(uint32_t v) {
 if (offset == USBC_UDINTCLR_OFFSET) set(USBC_UDINT_OFFSET, get(USBC_UDINT_OFFSET) & ~v);
 else if (offset == USBC_UDINTESET_OFFSET) add(USBC_UDINTE_OFFSET, v);
 else if (offset == USBC_UDINTECLR_OFFSET) set(USBC_UDINTE_OFFSET, get(USBC_UDINTE_OFFSET) & ~v);
 else if (offset >= 0x160 && offset < 0x180) {
  const unsigned ep = (offset - 0x160) / 4;
  // EP0 RXOUTIC releases a bank, not merely a latched interrupt flag.
  // A SETUP without an old OUT packet must not acknowledge an OUT bank.
  if (ep == 0 && (v & USBC_UESTA0CLR_RXOUTIC))
   assert(get(USBC_UESTA0_OFFSET) & USBC_UESTA0_RXOUTI);
  set(0x130 + ep * 4, get(0x130 + ep * 4) & ~v);
 }
 else if (offset >= 0x190 && offset < 0x1B0) add(0x130 + offset - 0x190, v);
 else if (offset >= 0x1F0 && offset < 0x210) {
  const unsigned ep = (offset - 0x1F0) / 4;
  if (v & USBC_UECON0SET_KILLBKS) {
   bankOwned[ep] = false;
   set(0x130 + ep * 4, get(0x130 + ep * 4) & ~USBC_UESTA0_NBUSYBK_Msk);
   add(0x1C0 + ep * 4, USBC_UECON0_FIFOCON);
   v &= ~USBC_UECON0SET_KILLBKS;
  }
  add(0x1C0 + ep * 4, v);
 }
 else if (offset >= 0x220 && offset < 0x240) {
  const unsigned ep = (offset - 0x220) / 4;
  assert(ep != 0 || !(v & USBC_UECON0CLR_FIFOCONC));
  if (v & USBC_UECON0CLR_FIFOCONC) {
   // Acknowledge TXINI/RXOUTI before handing the bank to DMA.
   assert(!(get(0x130 + ep * 4) & 3U));
   ++releases[ep]; bankOwned[ep] = true;
  }
  set(0x1C0 + ep * 4, get(0x1C0 + ep * 4) & ~v);
  if (ep && injectFirstOut && (v & USBC_UECON0CLR_BUSY0C) &&
      !(get(0x100 + ep * 4) & USBC_UECFG0_EPDIR)) {
   injectFirstOut = false;
   auto *bank = const_cast<uint32_t *>(static_cast<const volatile uint32_t *>(
       dmaPointer(get(USBC_UDESC_OFFSET)))) + ep * 8;
   bank[1] = 0;
   add(0x130 + ep * 4, USBC_UESTA0_RXOUTI);
   add(0x1C0 + ep * 4, USBC_UECON0_FIFOCON);
  }
 }
 else if (offset == USBC_UERST_OFFSET) {
  for (unsigned ep = 0; ep < 8; ++ep) {
   if (!(v & (1U << ep))) {
    set(0x130 + ep * 4, 0); set(0x1C0 + ep * 4, 0); bankOwned[ep] = false;
   } else if (!(value & (1U << ep))) {
    const volatile uint32_t *bank = static_cast<const volatile uint32_t *>(dmaPointer(get(USBC_UDESC_OFFSET))) + ep * 8;
    assert(bank[0] != 0U); // Descriptor address must be valid before endpoint activation.
    if (ep == 0 || (get(0x100 + ep * 4) & USBC_UECFG0_EPDIR))
     add(0x130 + ep * 4, USBC_UESTA0_TXINI);
    if (ep && !(get(0x100 + ep * 4) & USBC_UECFG0_EPDIR)) {
     bankOwned[ep] = true;
     if (injectFirstOut) {
      injectFirstOut = false;
      auto *data = const_cast<uint32_t *>(bank); data[1] = 0;
      add(0x130 + ep * 4, USBC_UESTA0_RXOUTI);
      add(0x1C0 + ep * 4, USBC_UECON0_FIFOCON);
     }
    } else if (ep) add(0x1C0 + ep * 4, USBC_UECON0_FIFOCON);
   }
  }
  value = v;
 } else value = v;
 return *this;
}
'''
TEST = r'''
McuOsc_t g_McuOsc = {{OSC_TYPE_XTAL, 12000000, 20, 180}, {}, true};
extern "C" void IOPinConfig(int, int, int, IOPINDIR, IOPINRES, IOPINTYPE) {}
extern "C" void IOPinDisableInterrupt(int) {}
extern "C" bool IOPinEnableInterrupt(int, int, uint32_t, uint32_t, IOPINSENSE, IOPinEvtHandler_t, void *) { return true; }
extern "C" void UsbProcessQue(int) {}
extern "C" uint32_t SystemCoreClockGet() { return SystemCoreClock; }
static std::vector<UsbCtrlrEvt_t> events;
static std::vector<UsbCtrlrEvtType_t> order;
alignas(4) static uint8_t rxBuffer[64], txBuffer[80], outCopy[64];
static bool acceptRx = true;
static unsigned completes, cancels, failures;
static uint16_t lastLength;
static int setupAction;
static unsigned physical(uint8_t ep, bool in) { return s_Usb.Map[ep][in]; }
static void endpoint(UsbCtrlrEvtType_t type, uint16_t length, void *) {
 if (type == USB_CTRLR_EVT_DRDY && acceptRx) assert(UsbCtrlrEpReceive(0, 2, rxBuffer, sizeof(rxBuffer)));
 if (type == USB_CTRLR_EVT_XFER_CMPL) { ++completes; lastLength = length; }
 if (type == USB_CTRLR_EVT_CANCEL) ++cancels;
 if (type == USB_CTRLR_EVT_XFER_FAILED) ++failures;
}
extern "C" void UsbDevProcessEvent(int, const UsbCtrlrEvt_t *event) {
 events.push_back(*event); order.push_back(event->Type);
 if (event->Type == USB_CTRLR_EVT_SETUP) {
  if (setupAction == 1) { UsbCtrlrSetAddress(0, event->Setup.wValue); assert(UsbCtrlrEp0Status(0, 0x80)); }
  if (setupAction == 2) assert(UsbCtrlrEp0Send(0, txBuffer, 80) == 64);
 }
 if (event->Type == USB_CTRLR_EVT_XFER_CMPL && event->Xfer.EpAddr == 0 && event->Xfer.Length)
  memcpy(outCopy, event->Xfer.pBuffer, event->Xfer.Length);
}
static void setup(uint8_t request, uint16_t value, int action) {
 setupAction = action; UsbSetupData_t packet = {};
 packet.bRequest = request; packet.wValue = value;
 memcpy(s_Ep0Buffer, &packet, sizeof(packet));
 s_Bank[0][0].PacketSize = sizeof(packet);
 add(0x130, USBC_UESTA0_RXSTPI); USBC_Handler();
}
static void completeIn(unsigned ep) {
 bankOwned[ep] = false; add(0x130 + 4 * ep, USBC_UESTA0_TXINI);
 add(0x1C0 + 4 * ep, USBC_UECON0_FIFOCON); USBC_Handler();
}
int main() {
 IOPinCfg_t pins[USB_VBUS_PIN_IDX + 1] = {};
 pins[USB_VBUS_PIN_IDX].PortNo = 2; pins[USB_VBUS_PIN_IDX].PinNo = 11;
 UsbCtrlrCfg_t cfg = {}; cfg.IntPrio = 6; cfg.bLowPowerSuspend = true;
 cfg.pIOPinMap = pins; cfg.NbIOPins = USB_VBUS_PIN_IDX + 1;
 assert(!UsbCtrlrInit(1, &cfg)); assert(UsbCtrlrInit(0, &cfg));
 assert(!UsbCtrlrStart(0)); // USB clock disabled.
 clocks.SCIF_GCCTRL[7].SCIF_GCCTRL = SCIF_GCCTRL_CEN;
 pm.PM_HSBMASK = PM_HSBMASK_USBC | PM_HSBMASK_HTOP1;
 pm.PM_PBBMASK = PM_PBBMASK_USBC;
 boardVbus = false; assert(!UsbCtrlrStart(0)); boardVbus = true;
 assert(!UsbCtrlrStart(0)); // Generic clock has not become usable.
 set(USBC_USBSTA_OFFSET, USBC_USBSTA_CLKUSABLE);
 assert(UsbCtrlrStart(0));
 assert((uintptr_t(s_Bank) & 7U) == 0U);
 UsbCtrlrEpBind(0, 2, false, true, endpoint, nullptr);
 UsbCtrlrEpBind(0, 2, true, true, endpoint, nullptr);
 assert(UsbCtrlrEpOpenData(0, 2, true, BULK, 64));
 assert(UsbCtrlrEpOpenData(0, 2, false, BULK, 64));
 const unsigned in = physical(2, true), out = physical(2, false);
 assert(in && out && in != out);
 assert((get(USBC_UERST_OFFSET) & (1U << in)) != 0U);
 assert((get(USBC_UERST_OFFSET) & (1U << out)) != 0U);
 assert(get(0x1C0 + out * 4) & USBC_UECON0_BUSY0);
 assert(get(0x130 + in * 4) & USBC_UESTA0_TXINI);
 assert(get(0x1C0 + in * 4) & USBC_UECON0_FIFOCON);
 assert((get(0x100 + in * 4) & USBC_UECFG0_REPNB_Msk) == USBC_UECFG0_REPNB(2));
 assert(!UsbCtrlrEpReceive(0, 2, rxBuffer, 63));
 injectFirstOut = true;
 USBC_Handler(); assert(s_Usb.Ep[out].Busy);
 assert(get(0x130 + out * 4) & USBC_UESTA0_RXOUTI);
 acceptRx = false; USBC_Handler(); assert(lastLength == 0 && !s_Usb.Ep[out].Busy);
 acceptRx = true; assert(UsbCtrlrEpReceive(0, 2, rxBuffer, 64));
 assert(dmaPointer(s_Bank[out][0].Address) == rxBuffer);
 assert(!UsbCtrlrEpReceive(0, 2, rxBuffer, 64));
 assert(UsbCtrlrEpSend(0, 2, txBuffer + 1, 63)); // Caller-owned unaligned SRAM.
 assert(!UsbCtrlrEpSend(0, 2, txBuffer, 1));
 const auto done = completes; completeIn(in); assert(completes == done + 1);
 assert(lastLength == 3 && !s_Usb.Ep[in].Busy);
 assert(UsbCtrlrEpSend(0, 2, txBuffer + 4, 60));
 assert(dmaPointer(s_Bank[in][0].Address) == txBuffer + 4);
 completeIn(in); assert(lastLength == 60);
 assert(UsbCtrlrEpSend(0, 2, nullptr, 0)); completeIn(in); assert(lastLength == 0);
 // Receive backpressure: completion publishes once and retains the bank.
 acceptRx = false; const auto released = releases[out];
 memset(rxBuffer, 0xAB, sizeof(rxBuffer)); s_Bank[out][0].PacketSize = 64;
 add(0x130 + out * 4, USBC_UESTA0_RXOUTI);
 add(0x1C0 + out * 4, USBC_UECON0_FIFOCON); USBC_Handler();
 assert(lastLength == 64 && !s_Usb.Ep[out].Busy && releases[out] == released);
 USBC_Handler(); assert(releases[out] == released);
 assert(UsbCtrlrEpReceive(0, 2, rxBuffer, 64)); assert(releases[out] == released + 1);
 // DMA failure retires the bank before relinquishing the buffer.
 assert(UsbCtrlrEpSend(0, 2, txBuffer, 16));
 add(0x130 + in * 4, USBC_UESTA0_RAMACERI | USBC_UESTA0_NBUSYBK(1)); USBC_Handler();
 assert(failures == 1 && !bankOwned[in] && !s_Usb.Ep[in].Busy);
 assert(UsbCtrlrEpSend(0, 2, txBuffer, 16)); completeIn(in);
 // Physical endpoint exhaustion fails without disturbing existing owners.
 for (unsigned n = 1; n <= 6; ++n) if (n != 2) assert(UsbCtrlrEpOpenData(0, n, true, INT, 10));
 assert(!UsbCtrlrEpOpenData(0, 7, true, BULK, 64));
 assert(!UsbCtrlrEpOpenData(0, 2, true, BULK, 64));
 assert(!UsbCtrlrIsoOpen(0, 1, true, 64));
 UsbCtrlrEpStall(0, 2, true); assert(get(0x1C0 + in * 4) & USBC_UECON0_STALLRQ);
 UsbCtrlrEpClearStall(0, 2, true); assert(!(get(0x1C0 + in * 4) & USBC_UECON0_STALLRQ));
 assert(get(0x1C0 + in * 4) & USBC_UECON0_RSTDT);
 // SET_ADDRESS takes effect only after the status ACK, not at SETUP.
 setup(5, 19, 1); assert(!(get(USBC_UDCON_OFFSET) & USBC_UDCON_ADDEN));
 assert(s_Usb.AddressPending); completeIn(0);
 assert(get(USBC_UDCON_OFFSET) & USBC_UDCON_ADDEN); assert(!s_Usb.AddressPending);
 setup(5, 21, 1); setup(6, 0x100, 0); completeIn(0);
 // An aborted SET_ADDRESS retains the address from the last completed request.
 assert((get(USBC_UDCON_OFFSET) & USBC_UDCON_UADD_Msk) == USBC_UDCON_UADD(19));
 assert(!s_Usb.AddressPending);
 // EP0 owns a copy; a new SETUP wins over stale IN/OUT flags.
 memset(txBuffer, 0x5A, sizeof(txBuffer)); setup(6, 0x100, 2);
 memset(txBuffer, 0, sizeof(txBuffer)); assert(s_Ep0Buffer[63] == 0x5A);
 const auto prior = events.size();
 add(0x130, USBC_UESTA0_TXINI | USBC_UESTA0_RXOUTI);
 setup(6, 0x200, 0); assert(events.size() == prior + 1 && events.back().Type == USB_CTRLR_EVT_SETUP);
 // IN ACK then OUT status can be pending together; report in that order.
 setup(6, 0x100, 2); s_Bank[0][0].PacketSize = 0;
 add(0x130, USBC_UESTA0_TXINI | USBC_UESTA0_RXOUTI);
 const auto beforeStatus = events.size();
 USBC_Handler(); assert(events.size() == beforeStatus + 2);
 assert(events[beforeStatus].Type == USB_CTRLR_EVT_XFER_CMPL &&
        events[beforeStatus].Xfer.EpAddr == 0x80);
 assert(events.back().Type == USB_CTRLR_EVT_XFER_CMPL && events.back().Xfer.EpAddr == 0);
 memset(s_Ep0Buffer, 0xA5, 7); s_Bank[0][0].PacketSize = 7;
 add(0x130, USBC_UESTA0_RXOUTI); USBC_Handler(); assert(outCopy[6] == 0xA5);
 // Reset cancels DMA and discards completions from the preceding bus state.
 const auto beforeReset = completes;
 assert(UsbCtrlrEpSend(0, 2, txBuffer, 8)); add(0x130 + in * 4, USBC_UESTA0_TXINI);
 add(USBC_UDINT_OFFSET, USBC_UDINT_EORST); USBC_Handler();
 assert(completes == beforeReset && cancels >= 2 && physical(2, true) == 0);
 assert(events.back().Type == USB_CTRLR_EVT_RESET && get(USBC_UERST_OFFSET) == 1U);
 add(USBC_UDINT_OFFSET, USBC_UDINT_SUSP); USBC_Handler();
 assert(s_Usb.Suspended && (get(USBC_USBCON_OFFSET) & USBC_USBCON_FRZCLK));
 add(USBC_UDINT_OFFSET, USBC_UDINT_WAKEUP); USBC_Handler();
 assert(!s_Usb.Suspended && events.back().Type == USB_CTRLR_EVT_RESUME);
 UsbCtrlrStop(0); assert(!s_Usb.Started);
 assert(!UsbCtrlrEpSend(0, 2, txBuffer, 1));
 assert(UsbCtrlrStart(0)); // Callback registrations survive stop/start.
 assert(s_Usb.Reg[1][0].Handler == endpoint);
 puts("SAM4L USBC ownership, EP0, redirection, reset and suspend: PASS");
}
'''
with tempfile.TemporaryDirectory(prefix='sam4l-usbc-') as tmp:
    tmp = Path(tmp)
    (tmp / 'sam4lxxx.h').write_text(HEADER)
    (tmp / 'iopinctrl.h').write_text(
        '#pragma once\n#include "coredev/iopincfg.h"\n'
        'extern bool boardVbus;\n'
        'inline int IOPinRead(int, int) { return boardVbus; }\n')
    (tmp / 'test.cpp').write_text(MODEL + source + TEST)

    command = [os.environ.get('CXX', 'g++'), '-std=gnu++17', '-O1', '-g',
               '-fsanitize=undefined', '-fno-sanitize-recover=all',
               '-I' + str(tmp), '-I' + str(ROOT / 'include'), '-I' + str(PORT / 'include'),
               str(tmp / 'test.cpp'), '-o', str(tmp / 'test')]
    subprocess.run(command, check=True)
    subprocess.run([str(tmp / 'test')], check=True)
