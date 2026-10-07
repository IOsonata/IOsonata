/**-------------------------------------------------------------------------
@file	usb_ctrlr_sam4l.cpp

@brief	Native SAM4L USBC full-speed device controller.

One SRAM bank per physical endpoint. Non-control payload storage belongs to
UsbIntrf; the controller calls only the registered endpoint callbacks. There
is no additional payload queue. EP0 copies its source before returning.

Register sequences follow ATSAM4L8/L4/L2, 42023H, chapter 17. The application
supplies its USB pin map; MCUOSC/SystemInit supplies the USB clock.

@author	Hoang Nguyen Hoan
@date	Oct. 5, 2026

@license

MIT License

Copyright (c) 2026, I-SYST inc., all rights reserved

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.

----------------------------------------------------------------------------*/
#include <string.h>

#include "sam4lxxx.h"
#include "coredev/interrupt.h"
#include "coredev/iopincfg.h"
#include "iopinctrl.h"
#include "coredev/system_core_clock.h"
#include "usb/usb.h"
#include "usb_ctrlr.h"

#if defined(USB_DEBUG_TRACE) && USB_DEBUG_TRACE > 0
#include "syslog.h"
#define SAM4L_USB_TRACE(...) SysLogPrintf(SysLogGet(), "[USBC] " __VA_ARGS__)
#else
#define SAM4L_USB_TRACE(...) ((void)0)
#endif

#if defined(USB_DEBUG_TRACE) && USB_DEBUG_TRACE > 1
// Values are copied into the event, not references to mutable IRQ snapshots.
static void Sam4lUsbTraceEvent(uint32_t Value, void *pContext)
{
	SysLogPrintf(SysLogGet(), "[USBC] %s=%08lx\r\n",
		static_cast<const char *>(pContext), static_cast<unsigned long>(Value));
}
#define SAM4L_USB_TRACE_QUE(Label, Value) \
	(void)UsbEvtQue((Value), const_cast<char *>(Label), Sam4lUsbTraceEvent)
#else
#define SAM4L_USB_TRACE_QUE(Label, Value) ((void)0)
#endif

enum {
	SAM4L_USB_EP_COUNT = 8,
	SAM4L_USB_EP0_MPS = 64,
	SAM4L_USB_PACKET_COUNT_MASK = 0x7FFFU,
	SAM4L_USB_WAIT_COUNT = 1024U,
	// The vendor header names bits 2/6 for control/bulk semantics only.
	// On ISO endpoints the same documented bit positions are ERRORFI/CRCERRI.
	SAM4L_USB_ISO_ERRORFI = 1U << 2,
	SAM4L_USB_ISO_CRCERRI = 1U << 6,
	SAM4L_USB_ISO_BANK_CRCERR = 1U << 16,
};

// All these interrupt bits have the same positions in UDINT, UDINTE,
// UDINTCLR and UDINTESET/CLR. Only documented SAM4L device bits are used.
static const uint32_t SAM4L_USB_RESUME_EVENTS =
	USBC_UDINT_WAKEUP | USBC_UDINT_EORSM | USBC_UDINT_UPRSM;
static const uint32_t SAM4L_USB_GLOBAL_EVENTS =
	USBC_UDINT_SUSP | USBC_UDINT_SOF | USBC_UDINT_EORST |
	SAM4L_USB_RESUME_EVENTS;

// Figure 17-6: TWO 16-byte descriptors per endpoint, even with one bank.
typedef struct __Sam4l_Usb_Bank {
	uint32_t Address;
	uint32_t PacketSize;
	uint32_t Status;
	uint32_t Reserved;
} Sam4lUsbBank_t;

typedef struct __Sam4l_Usb_Registration {
	UsbCtrlrEpHandler_t Handler;
	void *pContext;
} Sam4lUsbReg_t;

typedef struct __Sam4l_Usb_Endpoint {
	Sam4lUsbReg_t *pReg;
	uint32_t Generation;
	uint16_t Mps;
	uint16_t Length;
	uint8_t Address;
	uint8_t Type;
	bool Busy;
	bool Halted;
} Sam4lUsbEp_t;

typedef struct __Sam4l_Usb_State {
	Sam4lUsbEp_t Ep[SAM4L_USB_EP_COUNT];
	Sam4lUsbReg_t Reg[SAM4L_USB_EP_COUNT - 1][2];
	uint8_t Map[SAM4L_USB_EP_COUNT][2];
	uint8_t RxReady;
	uint8_t PendingAddress;
	const IOPinCfg_t *pIOPinMap;
	int NbIOPins;
	int VbusInt;
	bool Initialized;
	bool Started;
	bool Closing;
	bool Suspended;
	bool LowPowerSuspend;
	bool SofEnabled;
	bool AddressPending;
	bool Ep0In;
	bool Ep0StatusOut;
} Sam4lUsbState_t;

static Sam4lUsbState_t s_Usb;
alignas(32) static volatile Sam4lUsbBank_t s_Bank[SAM4L_USB_EP_COUNT][2];
alignas(4) static uint8_t s_Ep0Buffer[SAM4L_USB_EP0_MPS];
static_assert(sizeof(Sam4lUsbBank_t) == 16, "USBC bank descriptor stride");
static_assert(sizeof(s_Bank[0]) == 32, "USBC endpoint descriptor stride");

// Retained for a debugger. 0: none; 1: USB clock; 2: IN-bank abort; 3: EP activation.
static volatile uint32_t s_UsbLastError;

static inline volatile uint32_t &Sam4lUsbEpReg(uint32_t Offset, uint8_t Ep)
{
	return *reinterpret_cast<volatile uint32_t *>(
		reinterpret_cast<uintptr_t>(SAM4L_USBC) + Offset + Ep * sizeof(uint32_t));
}

static uint8_t Sam4lUsbPhysical(uint8_t EpNo, bool In)
{
	return EpNo < SAM4L_USB_EP_COUNT ? s_Usb.Map[EpNo][In] : 0U;
}

static void Sam4lUsbNotify(uint8_t Physical, UsbCtrlrEvtType_t Event, uint16_t Length)
{
	const uint32_t state = DisableInterrupt();
	const Sam4lUsbReg_t reg = s_Usb.Ep[Physical].pReg != nullptr ?
		*s_Usb.Ep[Physical].pReg : Sam4lUsbReg_t{};
	EnableInterrupt(state);
	if (reg.Handler != nullptr)
		reg.Handler(Event, Length, reg.pContext);
}

static void Sam4lUsbBusEvent(UsbCtrlrEvtType_t Type)
{
	UsbCtrlrEvt_t event = {};
	event.Type = Type;
	UsbDevProcessEvent(0, &event);
}

static void Sam4lUsbVbusEvent(int IntNo, void *pContext)
{
	(void)IntNo;
	(void)pContext;
	UsbProcessQue(0);
}

static void Sam4lUsbBankInit(uint8_t Physical)
{
	for (unsigned bank = 0U; bank < 2U; ++bank)
	{
		// An initialized SRAM address, not a second data buffer. An OUT bank
		// with no caller buffer stays BUSY (or stalled) and cannot DMA here.
		s_Bank[Physical][bank].Address = reinterpret_cast<uintptr_t>(s_Ep0Buffer);
		s_Bank[Physical][bank].PacketSize = 0U;
		s_Bank[Physical][bank].Status = 0U;
		s_Bank[Physical][bank].Reserved = 0U;
	}
}

// Caller has stopped the relevant hardware before returning buffer ownership.
// Closing blocks reentrant submissions from cancellation callbacks.
static void Sam4lUsbForgetEndpoint(uint8_t Physical)
{
	Sam4lUsbEp_t *ep = &s_Usb.Ep[Physical];
	const bool busy = ep->Busy;
	const Sam4lUsbReg_t reg = ep->pReg != nullptr ? *ep->pReg : Sam4lUsbReg_t{};
	if (ep->Mps != 0U)
		s_Usb.Map[ep->Address & 0x0FU][(ep->Address & USB_ENDPADDR_DIR_IN) != 0U] = 0U;
	s_Usb.RxReady &= ~(1U << Physical);
	ep->Mps = 0U;
	ep->Length = 0U;
	ep->Type = CONTROL;
	ep->Busy = false;
	ep->Halted = false;
	ep->pReg = nullptr;
	++ep->Generation;
	if (busy && reg.Handler != nullptr)
		reg.Handler(USB_CTRLR_EVT_CANCEL, 0U, reg.pContext);
}

static void Sam4lUsbHardwareFault(uint32_t Error)
{
	const uint32_t state = DisableInterrupt();
	s_UsbLastError = Error;
	NVIC_DisableIRQ(USBC_IRQn);
	// USBE is accessible even while frozen. Reset the DMA before cancelling
	// buffers; do not touch clock-dependent registers on this failure path.
	SAM4L_USBC->USBC_USBCON = USBC_USBCON_UIMOD | USBC_USBCON_FRZCLK;
	__DSB();
	s_Usb.Started = false;
	s_Usb.Suspended = false;
	s_Usb.SofEnabled = false;
	s_Usb.AddressPending = false;
	s_Usb.Ep[0].Busy = false;
	s_Usb.Closing = true;
	for (uint8_t ep = 1U; ep < SAM4L_USB_EP_COUNT; ++ep)
		Sam4lUsbForgetEndpoint(ep);
	s_Usb.Closing = false;
	NVIC_ClearPendingIRQ(USBC_IRQn);
	EnableInterrupt(state);
	SAM4L_USB_TRACE_QUE("controller stopped, error", Error);
}

// Sections 17.6.1.3 and 17.6.2.8: unfreeze BEFORE device-register access.
// CLKUSABLE qualifies subsequent access, not merely the FRZCLK readback.
static bool Sam4lUsbClockReady(void)
{
	SAM4L_USBC->USBC_USBCON &= ~USBC_USBCON_FRZCLK;
	uint32_t timeout = SAM4L_USB_WAIT_COUNT;
	do {
		if ((SAM4L_USBC->USBC_USBSTA & USBC_USBSTA_CLKUSABLE) != 0U)
			return true;
	} while (--timeout != 0U);
	return false;
}

static bool Sam4lUsbRequireClock(void)
{
	if (Sam4lUsbClockReady())
		return true;
	Sam4lUsbHardwareFault(1U);
	return false;
}

static void Sam4lUsbMaybeFreeze(void)
{
	if (s_Usb.Started && s_Usb.Suspended && s_Usb.LowPowerSuspend &&
		(SAM4L_USBC->USBC_USBCON & USBC_USBCON_FRZCLK) == 0U &&
		(SAM4L_USBC->USBC_UDCON & USBC_UDCON_RMWKUP) == 0U &&
		(SAM4L_USBC->USBC_UDINT &
		 (SAM4L_USB_RESUME_EVENTS | USBC_UDINT_EORST)) == 0U)
	{
		__DSB();
		SAM4L_USBC->USBC_USBCON |= USBC_USBCON_FRZCLK;
	}
}

static void Sam4lUsbEp0Init(void)
{
	// Bus reset clears UECFG/UECON, but does not promise to clear EPEN.
	// Disable EP0 explicitly before changing its descriptor and configuration.
	SAM4L_USBC->USBC_UERST &= ~1U;
	__DSB();
	s_Usb.Ep[0].Busy = false;
	s_Usb.Ep[0].Halted = false;
	s_Usb.Ep[0].Length = 0U;
	s_Usb.AddressPending = false;
	s_Usb.Ep0In = false;
	s_Usb.Ep0StatusOut = false;
	Sam4lUsbBankInit(0U);
	SAM4L_USBC->USBC_UECFG0 = USBC_UECFG0_EPSIZE(3U);
	__DMB();
	SAM4L_USBC->USBC_UERST |= 1U;
	SAM4L_USBC->USBC_UECON0CLR = USBC_UECON0CLR_BUSY0C | USBC_UECON0CLR_TXINEC;
	SAM4L_USBC->USBC_UECON0SET = USBC_UECON0SET_RXSTPES |
		USBC_UECON0SET_RXOUTES | USBC_UECON0SET_RAMACERES;
	SAM4L_USBC->USBC_UDINTESET = USBC_UDINTESET_EP0INTES;
}

bool UsbCtrlrInit(int DevNo, const UsbCtrlrCfg_t *pCfg)
{
	if (DevNo != 0 || pCfg == nullptr || pCfg->IntPrio < 0 ||
		pCfg->IntPrio >= (1 << __NVIC_PRIO_BITS) ||
		pCfg->pIOPinMap == nullptr || pCfg->NbIOPins <= USB_VBUS_PIN_IDX)
		return false;

	const IOPinCfg_t *vbus = &pCfg->pIOPinMap[USB_VBUS_PIN_IDX];
	if (vbus->PortNo < 0 || vbus->PortNo >= 3 ||
		vbus->PinNo < 0 || vbus->PinNo >= 32)
		return false;

	const uint32_t state = DisableInterrupt();
	// Reinitialization requires the port to be stopped. Release the previous
	// VBUS interrupt before installing a different pin map.
	if (s_Usb.Started || s_Usb.Closing)
	{
		EnableInterrupt(state);
		return false;
	}
	NVIC_DisableIRQ(USBC_IRQn);
	if (s_Usb.Initialized && s_Usb.VbusInt >= 0)
		IOPinDisableInterrupt(s_Usb.VbusInt);
	memset(&s_Usb, 0, sizeof(s_Usb));
	s_Usb.VbusInt = -1;
	s_Usb.LowPowerSuspend = pCfg->bLowPowerSuspend;
	NVIC_SetPriority(USBC_IRQn, pCfg->IntPrio);
	IOPinCfg(pCfg->pIOPinMap, pCfg->NbIOPins);
	const int vbusInt = (vbus->PortNo << 2) | (vbus->PinNo >> 3);
	if (!IOPinEnableInterrupt(vbusInt, pCfg->IntPrio,
			vbus->PortNo, vbus->PinNo, IOPINSENSE_TOGGLE,
			Sam4lUsbVbusEvent, nullptr))
	{
		EnableInterrupt(state);
		return false;
	}
	s_Usb.pIOPinMap = pCfg->pIOPinMap;
	s_Usb.NbIOPins = pCfg->NbIOPins;
	s_Usb.VbusInt = vbusInt;
	// PA25/PA26 function A are MCU USB signals, not board wiring (table 3-1).
	IOPinConfig(0, 25, IOPINOP_FUNC0, IOPINDIR_INPUT, IOPINRES_NONE, IOPINTYPE_NORMAL);
	IOPinConfig(0, 26, IOPINOP_FUNC0, IOPINDIR_INPUT, IOPINRES_NONE, IOPINTYPE_NORMAL);
	s_Usb.Initialized = true;
	EnableInterrupt(state);
	return true;
}

bool UsbCtrlrVbusDetected(int DevNo)
{
	if (DevNo != 0 || !s_Usb.Initialized || s_Usb.pIOPinMap == nullptr ||
		s_Usb.NbIOPins <= USB_VBUS_PIN_IDX)
		return false;
	const IOPinCfg_t *vbus = &s_Usb.pIOPinMap[USB_VBUS_PIN_IDX];
	return vbus->PortNo >= 0 && vbus->PortNo < 3 &&
		vbus->PinNo >= 0 && vbus->PinNo < 32 &&
		IOPinRead(vbus->PortNo, vbus->PinNo) != 0;
}

bool UsbCtrlrStart(int DevNo)
{
	if (DevNo != 0 || !s_Usb.Initialized || s_Usb.Closing || !UsbCtrlrVbusDetected(0))
		return false;
	if (s_Usb.Started)
		return true;

	// Check before the FIRST USBC register access. Do not repair SystemInit
	// or take ownership of GCLK7 here. The configured 48 MHz reference and
	// its accuracy remain the clock layer's responsibility (17.5.3).
	const uint32_t hsbMask = PM_HSBMASK_USBC | PM_HSBMASK_HTOP1;
	if (!g_McuOsc.bUSBClk || SystemCoreClockGet() < 12000000U ||
		(SAM4L_SCIF->SCIF_GCCTRL[7].SCIF_GCCTRL & SCIF_GCCTRL_CEN) == 0U ||
		(SAM4L_PM->PM_HSBMASK & hsbMask) != hsbMask ||
		(SAM4L_PM->PM_PBBMASK & PM_PBBMASK_USBC) == 0U ||
		(SAM4L_PM->PM_CPUSEL & PM_CPUSEL_MASK) !=
		 (SAM4L_PM->PM_PBBSEL & PM_PBBSEL_MASK) ||
		(SAM4L_BPM->BPM_PMCON & BPM_PMCON_PS_Msk) == BPM_PMCON_PS(1U))
		return false;

	const uint32_t state = DisableInterrupt();
	NVIC_DisableIRQ(USBC_IRQn);
	SAM4L_USBC->USBC_USBCON = USBC_USBCON_UIMOD;
	SAM4L_USBC->USBC_USBCON = USBC_USBCON_UIMOD | USBC_USBCON_USBE;
	if (!Sam4lUsbClockReady())
	{
		SAM4L_USBC->USBC_USBCON = USBC_USBCON_UIMOD | USBC_USBCON_FRZCLK;
		s_UsbLastError = 1U;
		EnableInterrupt(state);
		return false;
	}
	// Section 17.6.1.4 defines LS=0 for full speed. Do not write the
	// undocumented high-speed/test fields present in the shared CMSIS header.
	SAM4L_USBC->USBC_UDCON = USBC_UDCON_DETACH;
	for (uint8_t ep = 0U; ep < SAM4L_USB_EP_COUNT; ++ep)
		Sam4lUsbBankInit(ep);
	__DMB();
	SAM4L_USBC->USBC_UDESC = reinterpret_cast<uintptr_t>(s_Bank);
	SAM4L_USBC->USBC_UDINTECLR = SAM4L_USB_GLOBAL_EVENTS | (0xFFU << 12U);
	SAM4L_USBC->USBC_UDINTCLR = SAM4L_USB_GLOBAL_EVENTS;
	SAM4L_USBC->USBC_UDINTESET = USBC_UDINTESET_EORSTES | USBC_UDINTESET_SUSPES;
	s_Usb.Suspended = false;
	s_Usb.SofEnabled = false;
	s_Usb.RxReady = 0U;
	s_UsbLastError = 0U;
	s_Usb.Started = true;
	Sam4lUsbEp0Init();
	NVIC_ClearPendingIRQ(USBC_IRQn);
	EnableInterrupt(state);
	SAM4L_USB_TRACE("ready CPU=%lu GCLK7=%08lx USBSTA=%08lx\r\n",
		static_cast<unsigned long>(SystemCoreClockGet()),
		static_cast<unsigned long>(SAM4L_SCIF->SCIF_GCCTRL[7].SCIF_GCCTRL),
		static_cast<unsigned long>(SAM4L_USBC->USBC_USBSTA));
	return true;
}

void UsbCtrlrStop(int DevNo)
{
	if (DevNo != 0 || !s_Usb.Started || s_Usb.Closing)
		return;
	const uint32_t state = DisableInterrupt();
	NVIC_DisableIRQ(USBC_IRQn);
	// Hard disable is documented to reset the device/pads and works frozen.
	// DMA is stopped BEFORE the cancellation callbacks can recycle payloads.
	SAM4L_USBC->USBC_USBCON = USBC_USBCON_UIMOD | USBC_USBCON_FRZCLK;
	__DSB();
	s_Usb.Started = false;
	s_Usb.Closing = true;
	for (uint8_t ep = 1U; ep < SAM4L_USB_EP_COUNT; ++ep)
		Sam4lUsbForgetEndpoint(ep);
	s_Usb.Closing = false;
	s_Usb.Ep[0].Busy = false;
	s_Usb.AddressPending = false;
	s_Usb.Suspended = false;
	s_Usb.SofEnabled = false;
	NVIC_ClearPendingIRQ(USBC_IRQn);
	EnableInterrupt(state);
}

void UsbCtrlrProcess(int DevNo)
{
	if (DevNo != 0)
		return;
	const uint32_t state = DisableInterrupt();
	Sam4lUsbMaybeFreeze();
	EnableInterrupt(state);
}

bool UsbCtrlrHighSpeed(int DevNo)
{
	(void)DevNo;
	return false;
}

void UsbCtrlrIntEnable(int DevNo)
{
	if (DevNo == 0 && s_Usb.Started)
		NVIC_EnableIRQ(USBC_IRQn);
}

void UsbCtrlrIntDisable(int DevNo)
{
	if (DevNo == 0)
		NVIC_DisableIRQ(USBC_IRQn);
}

void UsbCtrlrConnect(int DevNo)
{
	if (DevNo != 0 || !s_Usb.Started || s_Usb.Closing || !UsbCtrlrVbusDetected(0))
		return;
	const uint32_t state = DisableInterrupt();
	if (Sam4lUsbRequireClock())
	{
		SAM4L_USBC->USBC_UDCON &= ~USBC_UDCON_DETACH;
		__DSB();
	}
	EnableInterrupt(state);
}

void UsbCtrlrDisconnect(int DevNo)
{
	if (DevNo != 0 || !s_Usb.Started)
		return;
	const uint32_t state = DisableInterrupt();
	if (Sam4lUsbRequireClock())
	{
		SAM4L_USBC->USBC_UDCON |= USBC_UDCON_DETACH;
		s_Usb.Suspended = false;
		SAM4L_USBC->USBC_UDINTECLR = SAM4L_USB_RESUME_EVENTS;
		SAM4L_USBC->USBC_UDINTESET = USBC_UDINTESET_SUSPES;
		SAM4L_USBC->USBC_UDINTCLR = SAM4L_USB_RESUME_EVENTS | USBC_UDINT_SUSP;
		__DSB();
	}
	EnableInterrupt(state);
}

void UsbCtrlrRemoteWakeup(int DevNo)
{
	// The generic USB core checks host permission. The hardware enforces
	// the five milliseconds of inactivity (section 17.6.2.10).
	if (DevNo != 0 || !s_Usb.Started || !s_Usb.Suspended || !UsbCtrlrVbusDetected(0))
		return;
	const uint32_t state = DisableInterrupt();
	if (Sam4lUsbRequireClock())
	{
		SAM4L_USBC->USBC_UDINTESET = SAM4L_USB_RESUME_EVENTS;
		SAM4L_USBC->USBC_UDCON |= USBC_UDCON_RMWKUP;
		__DSB();
		// Do not freeze while the hardware is generating upstream resume.
	}
	EnableInterrupt(state);
}

void UsbCtrlrSofEnable(int DevNo, bool Enable)
{
	if (DevNo != 0 || !s_Usb.Started)
		return;
	const uint32_t state = DisableInterrupt();
	if (Sam4lUsbRequireClock())
	{
		s_Usb.SofEnabled = Enable;
		SAM4L_USBC->USBC_UDINTCLR = USBC_UDINTCLR_SOFC;
		if (Enable)
			SAM4L_USBC->USBC_UDINTESET = USBC_UDINTESET_SOFES;
		else
			SAM4L_USBC->USBC_UDINTECLR = USBC_UDINTECLR_SOFEC;
	}
	EnableInterrupt(state);
}

void UsbCtrlrSetAddress(int DevNo, uint8_t Address)
{
	if (DevNo != 0 || !s_Usb.Started || Address > 127U)
		return;
	const uint32_t state = DisableInterrupt();
	// Keep the CURRENT address through the status ACK. In particular, do
	// not change a live UADD when ADDEN was already set. A new SETUP can
	// cancel this software value without changing the hardware address.
	s_Usb.PendingAddress = Address;
	s_Usb.AddressPending = true;
	EnableInterrupt(state);
}

void UsbCtrlrEpBind(int DevNo, uint8_t EpNo, bool bIn, bool bBlocking,
					 UsbCtrlrEpHandler_t Handler, void *pContext)
{
	(void)bBlocking;
	if (DevNo != 0 || EpNo == 0U || EpNo >= SAM4L_USB_EP_COUNT || s_Usb.Closing)
		return;
	const uint32_t state = DisableInterrupt();
	s_Usb.Reg[EpNo - 1U][bIn].Handler = Handler;
	s_Usb.Reg[EpNo - 1U][bIn].pContext = pContext;
	EnableInterrupt(state);
}

bool UsbCtrlrEpOpenData(int DevNo, uint8_t EpNo, bool bIn, uint8_t Type,
						 uint16_t MaxPacketSize)
{
	if (DevNo != 0 || !s_Usb.Started || s_Usb.Closing || EpNo == 0U ||
		EpNo >= SAM4L_USB_EP_COUNT ||
		(Type != ISO && Type != BULK && Type != INT) ||
		MaxPacketSize == 0U ||
		MaxPacketSize > (Type == ISO ? USB_CTRLR0_ISO_PKT_LEN_MAX : 64U) ||
		(Type == BULK && MaxPacketSize != 8U && MaxPacketSize != 16U &&
		 MaxPacketSize != 32U && MaxPacketSize != 64U))
		return false;

	const uint32_t state = DisableInterrupt();
	if (Sam4lUsbPhysical(EpNo, bIn) != 0U || !Sam4lUsbRequireClock())
	{
		EnableInterrupt(state);
		return false;
	}
	uint8_t physical = 1U;
	while (physical < SAM4L_USB_EP_COUNT && s_Usb.Ep[physical].Mps != 0U)
		++physical;
	if (physical == SAM4L_USB_EP_COUNT)
	{
		EnableInterrupt(state);
		return false;
	}
	uint32_t size = 0U;
	while ((8U << size) < MaxPacketSize)
		++size;
	Sam4lUsbEp_t *ep = &s_Usb.Ep[physical];
	ep->Mps = MaxPacketSize;
	ep->Length = 0U;
	ep->Address = EpNo | (bIn ? USB_ENDPADDR_DIR_IN : 0U);
	ep->Type = Type;
	ep->pReg = &s_Usb.Reg[EpNo - 1U][bIn];
	ep->Busy = false;
	ep->Halted = false;
	++ep->Generation;
	s_Usb.Map[EpNo][bIn] = physical;

	// Open must make HALT/CLEAR_FEATURE effective even before the first DMA.
	// EPEN=0 resets UECON, so setting BUSY before EPEN does NOT protect OUT.
	// Brief GNAK (17.7.2.1) covers activation until BUSY0E is installed.
	const bool gnak = (SAM4L_USBC->USBC_UDCON & USBC_UDCON_GNAK) != 0U;
	SAM4L_USBC->USBC_UDCON |= USBC_UDCON_GNAK;
	__DSB();
	Sam4lUsbBankInit(physical);
	Sam4lUsbEpReg(USBC_UECFG0_OFFSET, physical) = USBC_UECFG0_REPNB(EpNo) |
		USBC_UECFG0_EPSIZE(size) | USBC_UECFG0_EPTYPE(Type) |
		(bIn ? USBC_UECFG0_EPDIR : 0U);
	__DMB();
	SAM4L_USBC->USBC_UERST |= 1U << physical;
	// Bulk/interrupt IN is kept explicitly NAKed while software owns the
	// free bank. ISO IN is different: an empty bank must remain visible so
	// hardware can produce the specified automatic ZLP on an IN token.
	Sam4lUsbEpReg(USBC_UECON0SET_OFFSET, physical) = USBC_UECON0SET_RAMACERES |
		(bIn ? (Type == ISO ? 0U : USBC_UECON0SET_BUSY0S) :
		 USBC_UECON0SET_BUSY0S | USBC_UECON0SET_RXOUTES);
	__DSB();
	// Do not return a successfully opened IN endpoint until its first bank
	// is writable. The caller may submit immediately and owns its TX queue.
	uint32_t timeout = SAM4L_USB_WAIT_COUNT;
	bool ready;
	do {
		const uint32_t control = Sam4lUsbEpReg(USBC_UECON0_OFFSET, physical);
		ready = bIn ? (control & USBC_UECON0_FIFOCON) != 0U &&
			(Sam4lUsbEpReg(USBC_UESTA0_OFFSET, physical) & USBC_UESTA0_TXINI) != 0U :
			(control & USBC_UECON0_BUSY0) != 0U;
	} while (!ready && --timeout != 0U);
	if (!ready)
	{
		SAM4L_USBC->USBC_UERST &= ~(1U << physical);
		__DSB();
		Sam4lUsbForgetEndpoint(physical);
		s_UsbLastError = 3U;
	}
	if (!gnak)
		SAM4L_USBC->USBC_UDCON &= ~USBC_UDCON_GNAK;
	if (!ready)
	{
		EnableInterrupt(state);
		return false;
	}
	SAM4L_USBC->USBC_UDINTESET = USBC_UDINTESET_EP0INTES << physical;
	if (!bIn)
	{
		s_Usb.RxReady |= 1U << physical;
		NVIC_SetPendingIRQ(USBC_IRQn);
	}
	EnableInterrupt(state);
	return true;
}

bool UsbCtrlrEpOpen(int DevNo, const UsbEndPointDesc_t *pDesc)
{
	if (pDesc == nullptr || (pDesc->bEndpointAddress & 0x70U) != 0U)
		return false;
	return UsbCtrlrEpOpenData(DevNo, pDesc->bEndpointAddress & 0x0FU,
		(pDesc->bEndpointAddress & USB_ENDPADDR_DIR_IN) != 0U,
		pDesc->bmAttributes & 3U, pDesc->wMaxPacketSize);
}

void UsbCtrlrEpClose(int DevNo, uint8_t EpNo, bool bIn)
{
	if (DevNo != 0 || EpNo == 0U || s_Usb.Closing)
		return;
	const uint32_t state = DisableInterrupt();
	const uint8_t physical = Sam4lUsbPhysical(EpNo, bIn);
	if (physical != 0U && s_Usb.Started && Sam4lUsbRequireClock())
	{
		s_Usb.Closing = true;
		SAM4L_USBC->USBC_UDINTECLR = USBC_UDINTECLR_EP0INTEC << physical;
		SAM4L_USBC->USBC_UERST &= ~(1U << physical);
		__DSB();
		Sam4lUsbForgetEndpoint(physical);
		s_Usb.Closing = false;
	}
	EnableInterrupt(state);
}

void UsbCtrlrEpCloseAll(int DevNo)
{
	if (DevNo != 0 || !s_Usb.Started || s_Usb.Closing)
		return;
	const uint32_t state = DisableInterrupt();
	if (Sam4lUsbRequireClock())
	{
		s_Usb.Closing = true;
		SAM4L_USBC->USBC_UDINTECLR = 0xFEU << 12U;
		SAM4L_USBC->USBC_UERST &= ~0xFEU;
		__DSB();
		for (uint8_t ep = 1U; ep < SAM4L_USB_EP_COUNT; ++ep)
			Sam4lUsbForgetEndpoint(ep);
		s_Usb.Closing = false;
	}
	EnableInterrupt(state);
}

bool UsbCtrlrEpReceive(int DevNo, uint8_t EpNo, uint8_t *pBuffer, uint16_t Capacity)
{
	if (DevNo != 0 || EpNo == 0U || pBuffer == nullptr)
		return false;
	const uint32_t state = DisableInterrupt();
	const uint8_t physical = Sam4lUsbPhysical(EpNo, false);
	Sam4lUsbEp_t *ep = &s_Usb.Ep[physical];
	if (!s_Usb.Started || s_Usb.Closing || physical == 0U || ep->Busy ||
		Capacity < ep->Mps || !Sam4lUsbRequireClock())
	{
		EnableInterrupt(state);
		return false;
	}
	// Either BUSY0E/STALL holds the initial empty bank, or FIFOCON retains
	// a completed bank. No payload is accepted without a caller destination.
	s_Bank[physical][0].Address = reinterpret_cast<uintptr_t>(pBuffer);
	// MPS==EPSIZE completes after one full packet. MPS<EPSIZE is the
	// bounded short-packet buffer case in 17.6.2.13 (interrupt MPS 1..64).
	s_Bank[physical][0].PacketSize = static_cast<uint32_t>(ep->Mps) << 16U;
	s_Bank[physical][0].Status = 0U;
	ep->Busy = true;
	s_Usb.RxReady &= ~(1U << physical);
	__DMB();
	if (ep->Type == ISO)
		Sam4lUsbEpReg(USBC_UESTA0CLR_OFFSET, physical) =
			SAM4L_USB_ISO_ERRORFI | SAM4L_USB_ISO_CRCERRI;
	Sam4lUsbEpReg(USBC_UESTA0CLR_OFFSET, physical) = USBC_UESTA0CLR_RXOUTIC;
	if ((Sam4lUsbEpReg(USBC_UECON0_OFFSET, physical) & USBC_UECON0_FIFOCON) != 0U)
		Sam4lUsbEpReg(USBC_UECON0CLR_OFFSET, physical) = USBC_UECON0CLR_FIFOCONC;
	Sam4lUsbEpReg(USBC_UECON0CLR_OFFSET, physical) = USBC_UECON0CLR_BUSY0C;
	EnableInterrupt(state);
	return true;
}

bool UsbCtrlrEpSend(int DevNo, uint8_t EpNum, uint8_t *pBuffer, uint16_t Length)
{
	if (DevNo != 0 || EpNum == 0U || (Length != 0U && pBuffer == nullptr))
		return false;
	const uint32_t state = DisableInterrupt();
	const uint8_t physical = Sam4lUsbPhysical(EpNum, true);
	Sam4lUsbEp_t *ep = &s_Usb.Ep[physical];
	if (!s_Usb.Started || s_Usb.Closing || physical == 0U || ep->Busy ||
		Length > ep->Mps || !Sam4lUsbRequireClock())
	{
		EnableInterrupt(state);
		return false;
	}
	// Suspension or HALT does not discard a newly queued IN packet. The
	// bank remains owned until the host resumes/clears HALT, or cancellation.
	//
	// Bulk/interrupt idle banks are held with BUSY0E after open/completion.
	// TXINI is acknowledged at completion (ASF's ownership sequence), so for
	// those endpoints FIFOCON + BUSY0E is the software ownership handshake.
	// ISO keeps the native TXINI/FIFOCON handshake because an empty ISO bank
	// must remain available for automatic ZLP generation.
	const uint32_t epStatus = Sam4lUsbEpReg(USBC_UESTA0_OFFSET, physical);
	const uint32_t epControl = Sam4lUsbEpReg(USBC_UECON0_OFFSET, physical);
	const bool iso = ep->Type == ISO;
	if ((epControl & USBC_UECON0_FIFOCON) == 0U ||
		(iso ? (epStatus & USBC_UESTA0_TXINI) == 0U :
		       (epControl & USBC_UECON0_BUSY0) == 0U))
	{
		EnableInterrupt(state);
		return false;
	}
	uintptr_t dmaAddress = reinterpret_cast<uintptr_t>(
		pBuffer != nullptr ? pBuffer : s_Ep0Buffer);
	const uint32_t misalign = static_cast<uint32_t>(dmaAddress & 3U);
	if (Length != 0U && misalign != 0U)
	{
		// USBC DMA requires a word-aligned source. Transfer only the leading
		// bytes required to reach the next word boundary. Completion advances
		// the owner's FIFO by this shortened length; the following submission
		// is naturally aligned and remains zero-copy.
		const uint16_t repair = static_cast<uint16_t>(4U - misalign);
		if (Length > repair)
			Length = repair;

		// Every SAM4L endpoint descriptor has two 16-byte bank records, but
		// this driver configures EPBK=single. Bank 1 is therefore never owned
		// by USBC. Its aligned Address word is a per-physical-endpoint 4-byte
		// scratch slot, preserving independent endpoint DMA with no extra BSS.
		uint32_t scratch = 0U;
		memcpy(&scratch, pBuffer, Length);
		s_Bank[physical][1].Address = scratch;
		dmaAddress = reinterpret_cast<uintptr_t>(
			&s_Bank[physical][1].Address);
	}
	s_Bank[physical][0].Address = dmaAddress;
	s_Bank[physical][0].PacketSize = Length;
	s_Bank[physical][0].Status = 0U;
	ep->Length = Length;
	ep->Busy = true;
	__DMB();
	if (iso)
		Sam4lUsbEpReg(USBC_UESTA0CLR_OFFSET, physical) = SAM4L_USB_ISO_ERRORFI;
	// TXINI must be acknowledged BEFORE handing FIFOCON to hardware. For
	// bulk/interrupt the bank is still forced busy here, so descriptor writes
	// cannot race an IN token. Release BUSY0 only after FIFOCON is handed off.
	Sam4lUsbEpReg(USBC_UESTA0CLR_OFFSET, physical) = USBC_UESTA0CLR_TXINIC;
	Sam4lUsbEpReg(USBC_UECON0CLR_OFFSET, physical) = USBC_UECON0CLR_FIFOCONC;
	if (!iso)
		Sam4lUsbEpReg(USBC_UECON0CLR_OFFSET, physical) = USBC_UECON0CLR_BUSY0C;
	Sam4lUsbEpReg(USBC_UECON0SET_OFFSET, physical) = USBC_UECON0SET_TXINES;
	EnableInterrupt(state);
	return true;
}

void UsbCtrlrEpProcessEvent(int DevNo, uint8_t EpNo, bool bIn,
						 UsbCtrlrEvtType_t Event, uint16_t Value)
{
	if (DevNo != 0 || !s_Usb.Started)
		return;
	const uint8_t physical = Sam4lUsbPhysical(EpNo, bIn);
	if (physical != 0U)
		Sam4lUsbNotify(physical, Event, Value);
}

int UsbCtrlrEp0Send(int DevNo, uint8_t *pBuffer, int Length)
{
	if (DevNo != 0 || Length < 0 || (Length != 0 && pBuffer == nullptr))
		return -1;
	const uint32_t state = DisableInterrupt();
	if (!s_Usb.Started || s_Usb.Closing || s_Usb.Ep[0].Busy ||
		s_Usb.Ep[0].Halted || !Sam4lUsbRequireClock())
	{
		EnableInterrupt(state);
		return -1;
	}
	const uint32_t status = SAM4L_USBC->USBC_UESTA0;
	// Do not overwrite a received SETUP/OUT bank from a completion callback.
	// TXINI is the control bank's CPU-write ownership handshake (17.6.2.14).
	if ((status & (USBC_UESTA0_RXSTPI | USBC_UESTA0_RXOUTI)) != 0U ||
		(status & USBC_UESTA0_TXINI) == 0U)
	{
		EnableInterrupt(state);
		return -1;
	}
	const uint16_t length = Length < SAM4L_USB_EP0_MPS ? Length : SAM4L_USB_EP0_MPS;
	if (length != 0U)
		memcpy(s_Ep0Buffer, pBuffer, length);
	// SETUP has hardware priority even with CPU interrupts masked. Leave it
	// pending rather than submitting a stale packet if it superseded this IN.
	if ((SAM4L_USBC->USBC_UESTA0 & USBC_UESTA0_RXSTPI) != 0U)
	{
		EnableInterrupt(state);
		return -1;
	}
	s_Bank[0][0].PacketSize = length;
	s_Bank[0][0].Status = 0U;
	s_Usb.Ep[0].Length = length;
	s_Usb.Ep[0].Busy = true;
	s_Usb.Ep0StatusOut = false;
	__DMB();
	// Control endpoints submit through TXINI, NEVER through FIFOCON.
	SAM4L_USBC->USBC_UESTA0CLR = USBC_UESTA0CLR_TXINIC;
	SAM4L_USBC->USBC_UECON0SET = USBC_UECON0SET_TXINES;
	EnableInterrupt(state);
	return length;
}

bool UsbCtrlrEp0Status(int DevNo, uint8_t EpAddr)
{
	if (EpAddr == USB_ENDPADDR_DIR_IN)
		return UsbCtrlrEp0Send(DevNo, nullptr, 0) == 0;
	if (DevNo != 0 || EpAddr != 0U || !s_Usb.Started || s_Usb.Closing)
		return false;
	const uint32_t state = DisableInterrupt();
	const bool accepted = !s_Usb.Ep[0].Busy && !s_Usb.Ep[0].Halted;
	if (accepted)
		s_Usb.Ep0StatusOut = true;
	EnableInterrupt(state);
	return accepted;
}

void UsbCtrlrEpStall(int DevNo, uint8_t EpNo, bool bIn)
{
	if (DevNo != 0 || !s_Usb.Started || s_Usb.Closing)
		return;
	const uint32_t state = DisableInterrupt();
	const uint8_t physical = Sam4lUsbPhysical(EpNo, bIn);
	if ((EpNo == 0U || physical != 0U) && Sam4lUsbRequireClock())
	{
		// Never stall a newly received SETUP on behalf of an older request.
		if (EpNo == 0U && (SAM4L_USBC->USBC_UESTA0 & USBC_UESTA0_RXSTPI) != 0U)
		{
			EnableInterrupt(state);
			return;
		}
		s_Usb.Ep[physical].Halted = true;
		Sam4lUsbEpReg(USBC_UECON0SET_OFFSET, physical) = USBC_UECON0SET_STALLRQS;
		if (EpNo == 0U)
		{
			SAM4L_USBC->USBC_UECON0CLR = USBC_UECON0CLR_TXINEC;
			s_Usb.Ep[0].Busy = false;
			s_Usb.AddressPending = false;
			s_Usb.Ep0StatusOut = false;
		}
		else if (!bIn && !s_Usb.Ep[physical].Busy)
		{
			// STALL now guards the buffer-less endpoint. Remove forced BUSY
			// so it cannot mask the requested STALL with a NAK.
			Sam4lUsbEpReg(USBC_UECON0CLR_OFFSET, physical) = USBC_UECON0CLR_BUSY0C;
		}
	}
	EnableInterrupt(state);
}

void UsbCtrlrEpClearStall(int DevNo, uint8_t EpNo, bool bIn)
{
	if (DevNo != 0 || !s_Usb.Started || s_Usb.Closing)
		return;
	const uint32_t state = DisableInterrupt();
	const uint8_t physical = Sam4lUsbPhysical(EpNo, bIn);
	if ((EpNo == 0U || physical != 0U) && Sam4lUsbRequireClock())
	{
		if (EpNo == 0U && (SAM4L_USBC->USBC_UESTA0 & USBC_UESTA0_RXSTPI) != 0U)
		{
			EnableInterrupt(state);
			return;
		}
		if (EpNo != 0U && !bIn && !s_Usb.Ep[physical].Busy)
			Sam4lUsbEpReg(USBC_UECON0SET_OFFSET, physical) = USBC_UECON0SET_BUSY0S;
		// Reset DATA toggle while STALL still prevents another transaction.
		Sam4lUsbEpReg(USBC_UECON0SET_OFFSET, physical) = USBC_UECON0SET_RSTDTS;
		Sam4lUsbEpReg(USBC_UESTA0CLR_OFFSET, physical) = USBC_UESTA0CLR_STALLEDIC;
		__DMB();
		Sam4lUsbEpReg(USBC_UECON0CLR_OFFSET, physical) = USBC_UECON0CLR_STALLRQC;
		s_Usb.Ep[physical].Halted = false;
	}
	EnableInterrupt(state);
}

static void Sam4lUsbEp0Result(bool In, uint16_t Length,
		const uint8_t *pBuffer, UsbCtrlrXferResult_t Result)
{
	UsbCtrlrEvt_t event = {};
	event.Type = USB_CTRLR_EVT_XFER_CMPL;
	event.Xfer.EpAddr = In ? USB_ENDPADDR_DIR_IN : 0U;
	event.Xfer.Length = Length;
	event.Xfer.pBuffer = pBuffer;
	event.Xfer.Result = Result;
	UsbDevProcessEvent(0, &event);
}

static void Sam4lUsbEp0Interrupt(void)
{
	const uint32_t status = SAM4L_USBC->USBC_UESTA0;
	const uint32_t enabled = SAM4L_USBC->USBC_UECON0;
	__DMB();
	if ((status & USBC_UESTA0_RXSTPI) != 0U)
	{
		SAM4L_USBC->USBC_UECON0CLR = USBC_UECON0CLR_TXINEC;
		s_Usb.Ep[0].Busy = false;
		s_Usb.Ep[0].Halted = false;
		s_Usb.AddressPending = false;
		s_Usb.Ep0StatusOut = false;
		UsbCtrlrEvt_t event = {};
		event.Type = USB_CTRLR_EVT_SETUP;
		memcpy(&event.Setup, s_Ep0Buffer, sizeof(event.Setup));
		s_Usb.Ep0In = (event.Setup.bmRequestType & 0x80U) != 0U;
		s_Bank[0][0].PacketSize = 0U;
		s_Bank[0][0].Status = 0U;
		__DMB();
		SAM4L_USBC->USBC_UESTA0CLR = status &
			(USBC_UESTA0CLR_RXOUTIC | USBC_UESTA0CLR_RAMACERIC | USBC_UESTA0CLR_STALLEDIC);
		SAM4L_USBC->USBC_UESTA0CLR = USBC_UESTA0CLR_RXSTPIC;
		UsbDevProcessEvent(0, &event);
		return;
	}

	if ((status & enabled & USBC_UESTA0_RAMACERI) != 0U)
	{
		SAM4L_USBC->USBC_UECON0CLR = USBC_UECON0CLR_TXINEC;
		SAM4L_USBC->USBC_UESTA0CLR = USBC_UESTA0CLR_RAMACERIC;
		s_Usb.Ep[0].Busy = false;
		s_Usb.AddressPending = false;
		Sam4lUsbEp0Result(true, 0U, nullptr, USB_CTRLR_XFER_FAILED);
		if (s_Usb.Started)
			UsbCtrlrEpStall(0, 0U, false);
		return;
	}

	// The host may have already sent status OUT when the CPU sees IN done.
	// Snapshot its bytes/count BEFORE the IN callback can change the bank.
	const bool out = (status & enabled & USBC_UESTA0_RXOUTI) != 0U;
	const uint16_t rxLength = out ?
		(s_Bank[0][0].PacketSize & SAM4L_USB_PACKET_COUNT_MASK) : 0U;
	uint8_t data[SAM4L_USB_EP0_MPS];
	if (out && rxLength != 0U && rxLength <= sizeof(data))
		memcpy(data, s_Ep0Buffer, rxLength);

	if ((status & enabled & USBC_UESTA0_TXINI) != 0U)
	{
		SAM4L_USBC->USBC_UECON0CLR = USBC_UECON0CLR_TXINEC;
		if (s_Usb.Ep[0].Busy)
		{
			const uint16_t txLength = s_Usb.Ep[0].Length;
			s_Usb.Ep[0].Busy = false;
			if (s_Usb.AddressPending && txLength == 0U)
			{
				// Section 17.6.2.7: UADD first, ADDEN in a separate write.
				// This is after the status packet ACK, not merely submission.
				// Write ADDEN=0 in the first value even when it is already set:
				// do not write a one to ADDEN alongside the changed UADD.
				SAM4L_USBC->USBC_UDCON =
					(SAM4L_USBC->USBC_UDCON &
					 ~(USBC_UDCON_UADD_Msk | USBC_UDCON_ADDEN)) |
					USBC_UDCON_UADD(s_Usb.PendingAddress);
				SAM4L_USBC->USBC_UDCON |= USBC_UDCON_ADDEN;
				s_Usb.AddressPending = false;
			}
			// Do not reset PacketSize here: hardware may now own it for OUT
			// (or a new SETUP), even if RXOUTI was not in our first snapshot.
			Sam4lUsbEp0Result(true, txLength, nullptr, USB_CTRLR_XFER_SUCCESS);
			if (!s_Usb.Started ||
				(SAM4L_USBC->USBC_UESTA0 & USBC_UESTA0_RXSTPI) != 0U)
				return;
		}
	}

	if (out)
	{
		if (!s_Usb.Started ||
			(SAM4L_USBC->USBC_UESTA0 & USBC_UESTA0_RXSTPI) != 0U)
			return;
		// Stop a superseded IN before acknowledging the OUT status bank.
		SAM4L_USBC->USBC_UECON0CLR = USBC_UECON0CLR_TXINEC;
		s_Usb.Ep[0].Busy = false;
		s_Usb.AddressPending = false;
		const bool halted = s_Usb.Ep[0].Halted;
		const bool valid = rxLength <= sizeof(data) &&
			(!s_Usb.Ep0In || (s_Usb.Ep0StatusOut && rxLength == 0U));
		s_Bank[0][0].PacketSize = 0U;
		s_Bank[0][0].Status = 0U;
		__DMB();
		SAM4L_USBC->USBC_UESTA0CLR = USBC_UESTA0CLR_RXOUTIC;
		// A stale IN continuation refused by Ep0Send may already have made
		// the generic core abort/stall. Do not send it another completion.
		if (!halted)
			Sam4lUsbEp0Result(false, valid ? rxLength : 0U,
				valid ? data : nullptr, valid ? USB_CTRLR_XFER_SUCCESS : USB_CTRLR_XFER_CANCELLED);
	}
}

// RAMACERI is an IN underflow, not an OUT-overflow indication (17.7.2.11).
// Retire a still-owned IN bank before reporting failure/releasing its source.
static bool Sam4lUsbRetireIn(uint8_t Physical)
{
	Sam4lUsbEpReg(USBC_UECON0CLR_OFFSET, Physical) = USBC_UECON0CLR_TXINEC;
	if ((Sam4lUsbEpReg(USBC_UESTA0_OFFSET, Physical) & USBC_UESTA0_NBUSYBK_Msk) != 0U)
	{
		Sam4lUsbEpReg(USBC_UECON0SET_OFFSET, Physical) = USBC_UECON0SET_KILLBKS;
		uint32_t timeout = SAM4L_USB_WAIT_COUNT;
		while ((Sam4lUsbEpReg(USBC_UECON0_OFFSET, Physical) & USBC_UECON0_KILLBK) != 0U)
		{
			if (--timeout == 0U)
			{
				Sam4lUsbHardwareFault(2U);
				return false;
			}
		}
	}
	__DSB();
	if ((Sam4lUsbEpReg(USBC_UESTA0_OFFSET, Physical) & USBC_UESTA0_NBUSYBK_Msk) != 0U ||
		(Sam4lUsbEpReg(USBC_UECON0_OFFSET, Physical) & USBC_UECON0_FIFOCON) == 0U)
	{
		Sam4lUsbHardwareFault(2U);
		return false;
	}
	// An actually killed bank need not produce TXINI (17.7.2.14). It is
	// now demonstrably free, so restore the software-ready indication.
	Sam4lUsbEpReg(USBC_UESTA0SET_OFFSET, Physical) = USBC_UESTA0SET_TXINIS;
	return true;
}

static void Sam4lUsbDataInterrupt(uint8_t Physical)
{
	Sam4lUsbEp_t *ep = &s_Usb.Ep[Physical];
	const uint32_t status = Sam4lUsbEpReg(USBC_UESTA0_OFFSET, Physical);
	const uint32_t enabled = Sam4lUsbEpReg(USBC_UECON0_OFFSET, Physical);
	const uint32_t active = status & enabled;
	const bool in = (ep->Address & USB_ENDPADDR_DIR_IN) != 0U;
	const uint32_t done = in ? USBC_UESTA0_TXINI : USBC_UESTA0_RXOUTI;
	if ((active & (done | USBC_UESTA0_RAMACERI)) == 0U)
		return;

	// Acknowledge/mask even when no software transfer is outstanding; a
	// latched enabled error must never trap the CPU in an idle-endpoint IRQ.
	if ((active & USBC_UESTA0_RAMACERI) != 0U)
		Sam4lUsbEpReg(USBC_UESTA0CLR_OFFSET, Physical) = USBC_UESTA0CLR_RAMACERIC;
	if (in)
	{
		Sam4lUsbEpReg(USBC_UECON0CLR_OFFSET, Physical) = USBC_UECON0CLR_TXINEC;
		// Match the SAM4L reference-driver ownership sequence for normal IN:
		// force NAK before acknowledging TXINI, then keep the bank locked while
		// the completion callback advances the queue and programs the next DMA.
		// ISO must not be locked: an empty ISO IN bank intentionally auto-ZLPs.
		if (ep->Type != ISO)
		{
			Sam4lUsbEpReg(USBC_UECON0SET_OFFSET, Physical) = USBC_UECON0SET_BUSY0S;
			Sam4lUsbEpReg(USBC_UESTA0CLR_OFFSET, Physical) = USBC_UESTA0CLR_TXINIC;
		}
	}
	if (!ep->Busy)
	{
		if (!in)
			Sam4lUsbEpReg(USBC_UESTA0CLR_OFFSET, Physical) = USBC_UESTA0CLR_RXOUTIC;
		return;
	}
	__DMB();
	const uint16_t length = in ? ep->Length :
		(s_Bank[Physical][0].PacketSize & SAM4L_USB_PACKET_COUNT_MASK);
	const bool iso = ep->Type == ISO;
	const uint32_t bankStatus = iso ? s_Bank[Physical][0].Status : 0U;
	const bool crcFailed = iso && !in &&
		(((status & SAM4L_USB_ISO_CRCERRI) != 0U) ||
		 ((bankStatus & SAM4L_USB_ISO_BANK_CRCERR) != 0U));
	const bool failed = (active & USBC_UESTA0_RAMACERI) != 0U ||
		length > ep->Mps || crcFailed;
	if (iso)
	{
		Sam4lUsbEpReg(USBC_UESTA0CLR_OFFSET, Physical) =
			SAM4L_USB_ISO_ERRORFI | SAM4L_USB_ISO_CRCERRI;
		s_Bank[Physical][0].Status = 0U;
	}
	if (in && failed && !Sam4lUsbRetireIn(Physical))
		return;
	if (!in)
		Sam4lUsbEpReg(USBC_UESTA0CLR_OFFSET, Physical) = USBC_UESTA0CLR_RXOUTIC;
	// Non-ISO IN remains BUSY/NAKed until the completion callback queues the
	// next packet. ISO keeps its native empty-bank state. OUT retains FIFOCON
	// until a replacement destination has been installed.
	const uint32_t generation = ep->Generation;
	ep->Busy = false;
	Sam4lUsbNotify(Physical, failed ? USB_CTRLR_EVT_XFER_FAILED : USB_CTRLR_EVT_XFER_CMPL,
		failed && !in ? 0U : length);
	// Completion may close/reopen this same physical slot or stop the device.
	if (s_Usb.Started && !in && ep->Generation == generation && ep->Mps != 0U && !ep->Busy)
		Sam4lUsbNotify(Physical, USB_CTRLR_EVT_DRDY, 0U);
}

static void Sam4lUsbInterrupt(void)
{
	if (!s_Usb.Started || !Sam4lUsbRequireClock())
		return;
	uint32_t status = SAM4L_USBC->USBC_UDINT & SAM4L_USBC->USBC_UDINTE;
	if ((status & USBC_UDINT_EORST) != 0U)
	{
		SAM4L_USB_TRACE_QUE("RESET", status);
		SAM4L_USBC->USBC_UDINTCLR = SAM4L_USB_GLOBAL_EVENTS;
		SAM4L_USBC->USBC_UDINTECLR = SAM4L_USB_RESUME_EVENTS;
		SAM4L_USBC->USBC_UDINTESET = USBC_UDINTESET_SUSPES;
		s_Usb.Suspended = false;
		UsbCtrlrEpCloseAll(0);
		if (!s_Usb.Started)
			return;
		Sam4lUsbEp0Init();
		Sam4lUsbBusEvent(USB_CTRLR_EVT_RESET);
		return; // Old endpoint completions do not belong to the new bus state.
	}

	if (s_Usb.Suspended && (status & SAM4L_USB_RESUME_EVENTS) != 0U)
	{
		// WAKEUP explicitly excludes our upstream resume in 17.7.2.2.
		// Keep UPRSM/EORSM armed too, or a remote wake can remain suspended.
		SAM4L_USBC->USBC_UDINTCLR = SAM4L_USB_RESUME_EVENTS | USBC_UDINT_SUSP;
		SAM4L_USBC->USBC_UDINTECLR = SAM4L_USB_RESUME_EVENTS;
		SAM4L_USBC->USBC_UDINTESET = USBC_UDINTESET_SUSPES;
		s_Usb.Suspended = false;
		Sam4lUsbBusEvent(USB_CTRLR_EVT_RESUME);
		if (!s_Usb.Started)
			return;
	}

	if ((status & USBC_UDINT_EP0INT) != 0U)
	{
		Sam4lUsbEp0Interrupt();
		if (!s_Usb.Started)
			return;
	}
	for (uint8_t ep = 1U; ep < SAM4L_USB_EP_COUNT; ++ep)
	{
		// Reread after callbacks, not a stale endpoint-summary snapshot.
		if ((SAM4L_USBC->USBC_UDINT & SAM4L_USBC->USBC_UDINTE &
			 (USBC_UDINT_EP0INT << ep)) != 0U)
			Sam4lUsbDataInterrupt(ep);
		if (!s_Usb.Started)
			return;
		if ((s_Usb.RxReady & (1U << ep)) != 0U)
		{
			s_Usb.RxReady &= ~(1U << ep);
			if (s_Usb.Ep[ep].Mps != 0U && !s_Usb.Ep[ep].Busy &&
				(s_Usb.Ep[ep].Address & USB_ENDPADDR_DIR_IN) == 0U)
				Sam4lUsbNotify(ep, USB_CTRLR_EVT_DRDY, 0U);
			if (!s_Usb.Started)
				return;
		}
	}

	status = SAM4L_USBC->USBC_UDINT & SAM4L_USBC->USBC_UDINTE;
	if ((status & USBC_UDINT_SOF) != 0U)
	{
		const uint32_t frame = SAM4L_USBC->USBC_UDFNUM;
		SAM4L_USBC->USBC_UDINTCLR = USBC_UDINTCLR_SOFC;
		if (s_Usb.SofEnabled && !s_Usb.Suspended && (frame & USBC_UDFNUM_FNCERR) == 0U)
		{
			UsbCtrlrEvt_t event = {};
			event.Type = USB_CTRLR_EVT_SOF;
			event.FrameNo = (frame & USBC_UDFNUM_FNUM_Msk) >> USBC_UDFNUM_FNUM_Pos;
			UsbDevProcessEvent(0, &event);
			if (!s_Usb.Started)
				return;
		}
	}

	if (!s_Usb.Suspended &&
		(SAM4L_USBC->USBC_UDINT & SAM4L_USBC->USBC_UDINTE & USBC_UDINT_SUSP) != 0U)
	{
		// WAKEUP latches on ANY non-idle traffic, including normal operation.
		// A stale, masked WAKEUP must not veto SUSP and leave its IRQ asserted.
		SAM4L_USBC->USBC_UDINTCLR = SAM4L_USB_RESUME_EVENTS;
		__DMB();
		// Hardware clears SUSP on wake. Check it before acknowledging SUSP:
		// traffic arriving during the handover must not put us back to sleep.
		if ((SAM4L_USBC->USBC_UDINT & USBC_UDINT_SUSP) != 0U)
		{
			SAM4L_USBC->USBC_UDINTECLR = USBC_UDINTECLR_SUSPEC;
			SAM4L_USBC->USBC_UDINTCLR = USBC_UDINTCLR_SUSPC;
			SAM4L_USBC->USBC_UDINTESET = SAM4L_USB_RESUME_EVENTS;
			s_Usb.Suspended = true;
			Sam4lUsbBusEvent(USB_CTRLR_EVT_SUSPEND);
		}
	}
	Sam4lUsbMaybeFreeze();
}

extern "C" void USBC_Handler(void)
{
	const uint32_t state = DisableInterrupt();
	Sam4lUsbInterrupt();
	__DSB();
	EnableInterrupt(state);
}

size_t UsbCtrlrGetSerial(int DevNo, char *pBuff, size_t BuffLen)
{
	if (DevNo != 0 || pBuff == nullptr || BuffLen == 0U)
		return 0U;
	// FLASHCALW factory serial: 120 bits, 0x0080020C..0x0080021A.
	const volatile uint8_t *uid = reinterpret_cast<const volatile uint8_t *>(0x0080020CU);
	const size_t count = BuffLen > 30U ? 30U : BuffLen - 1U;
	for (size_t i = 0U; i < count; ++i)
	{
		const uint8_t nibble = (uid[i >> 1] >> ((i & 1U) != 0U ? 0 : 4)) & 15U;
		pBuff[i] = nibble < 10U ? '0' + nibble : 'A' + nibble - 10U;
	}
	pBuff[count] = '\0';
	return count;
}

__attribute__((weak))
bool UsbCtrlrIsoOpen(int DevNo, uint8_t EpNo, bool bIn, uint16_t MaxPacketSize)
{
	(void)DevNo;
	(void)EpNo;
	(void)bIn;
	(void)MaxPacketSize;
	return false;
}

__attribute__((weak))
bool UsbCtrlrIsoSend(int DevNo, uint8_t EpNum, uint8_t *pBuffer, uint16_t Length)
{
	(void)DevNo;
	(void)EpNum;
	(void)pBuffer;
	(void)Length;
	return false;
}

__attribute__((weak))
uint16_t UsbCtrlrIsoTraceSnapshot(int DevNo, uint8_t **ppData)
{
	(void)DevNo;
	if (ppData != nullptr)
		*ppData = nullptr;
	return 0U;
}
