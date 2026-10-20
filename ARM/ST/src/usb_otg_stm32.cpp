/**-------------------------------------------------------------------------
@file	usb_otg_stm32.cpp

@brief	STM32 OTG USB device controller, shared FS and embedded HS FIFO engine.

The protocol and class implementations remain in src/usb. Payloads belong to
the endpoint owner. FIFO access is interrupt driven; no DMA alignment or
cache maintenance is imposed on callers. This is not the STM32 PMA engine.

@author	Hoang Nguyen Hoan
@date	Oct. 18, 2026

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

#include "stm32.h"
#include "istddef.h"
#include "idelay.h"
#include "coredev/interrupt.h"
#include "coredev/system_core_clock.h"
#include "usb/usb.h"
#if defined(STM32H753xx)
#include "app_evt_handler.h"
#include "syslog.h"
#endif

// Private target hooks implement clock, power and PHY setup only.
extern bool Stm32UsbOtgPowerInit(const UsbCtrlrCfg_t *pCfg);
extern void Stm32UsbOtgReset(void);
extern USB_OTG_GlobalTypeDef *Stm32UsbOtgRegs(void);
extern IRQn_Type Stm32UsbOtgIrq(void);
extern uint16_t Stm32UsbOtgFifoWords(void);

enum {
	USB_EP_COUNT = USB_EPIN_CNT_0,
	USB_EP0_MPS = 64,
	USB_WAIT_LIMIT = 100000,
};

typedef struct __Stm32_Usb_Endpoint {
	UsbCtrlrEpHandler_t Handler;
	void *pContext;
	uint8_t *pBuffer;
	uint32_t Generation;
	uint16_t Mps;
	uint16_t Length;
	uint16_t Actual;
	uint8_t Type;
	bool Busy;
	bool Halted;
	bool Overflow;
} Stm32UsbEp_t;

static struct {
	USB_OTG_GlobalTypeDef *pReg;
	Stm32UsbEp_t Ep[USB_EP_COUNT][2];
	uint16_t TxBase[USB_EP_COUNT];
	uint16_t TxWords[USB_EP_COUNT];
	uint16_t RxWords;
	uint16_t RxReady;
	uint16_t ControlRemaining;
	UsbSetupData_t Setup;
	uint8_t Ep0Out[USB_EP0_MPS];
	bool Initialized;
	bool Started;
	bool Closing;
	bool HighSpeed;
	bool Suspended;
	bool SofEnabled;
	bool Recover;
	bool SetupPending;
} s_Usb;

static USB_OTG_DeviceTypeDef *Device(void)
{
	return reinterpret_cast<USB_OTG_DeviceTypeDef *>(
		reinterpret_cast<uintptr_t>(s_Usb.pReg) + USB_OTG_DEVICE_BASE);
}

// H753 bring-up trace. Only bus events are queued; packet interrupts are silent.
// Formatting and UART writes run in AppRun, never in the USB ISR. Register
// values below are live at dispatch time; Status is the original IRQ snapshot.
#if defined(STM32H753xx)
static unsigned s_TraceDropped;

static void TraceDispatch(uint32_t Status, void *pContext)
{
	const char *name = static_cast<const char *>(pContext);
	SysLogPrintf(SysLogGet(), "USB %s irq=%08x dropped=%u\r\n",
		name, static_cast<unsigned>(Status), __atomic_load_n(&s_TraceDropped, __ATOMIC_RELAXED));
	SysLogPrintf(SysLogGet(), "USB now GOTGCTL=%08x GINTSTS=%08x GINTMSK=%08x\r\n",
		static_cast<unsigned>(s_Usb.pReg->GOTGCTL),
		static_cast<unsigned>(s_Usb.pReg->GINTSTS),
		static_cast<unsigned>(s_Usb.pReg->GINTMSK));
	SysLogPrintf(SysLogGet(), "USB now DCFG=%08x DCTL=%08x DSTS=%08x\r\n",
		static_cast<unsigned>(Device()->DCFG), static_cast<unsigned>(Device()->DCTL),
		static_cast<unsigned>(Device()->DSTS));
	if (Status == 0)
	{
		SysLogPrintf(SysLogGet(), "USB now GAHBCFG=%08x GUSBCFG=%08x GCCFG=%08x\r\n",
			static_cast<unsigned>(s_Usb.pReg->GAHBCFG),
			static_cast<unsigned>(s_Usb.pReg->GUSBCFG),
			static_cast<unsigned>(s_Usb.pReg->GCCFG));
		SysLogPrintf(SysLogGet(), "USB RCC CR=%08x USBSEL=%08x AHB1ENR=%08x PWR CR3=%08x\r\n",
			static_cast<unsigned>(RCC->CR), static_cast<unsigned>(RCC->D2CCIP2R),
			static_cast<unsigned>(RCC->AHB1ENR), static_cast<unsigned>(PWR->CR3));
		SysLogPrintf(SysLogGet(), "USB RCC AHB1LPENR=%08x\r\n",
			static_cast<unsigned>(RCC->AHB1LPENR));
		SysLogPrintf(SysLogGet(), "USB GPIOA MODER=%08x AFRH=%08x CRS ISR=%08x\r\n",
			static_cast<unsigned>(GPIOA->MODER), static_cast<unsigned>(GPIOA->AFR[1]),
			static_cast<unsigned>(CRS->ISR));
	}
}

static void Trace(uint32_t Status, const char *pName)
{
	if (SysLogGet()->Marker != SYSLOG_INIT_MARKER)
	{
		return;
	}
	if (!AppEvtHandlerQue(Status, const_cast<char *>(pName), TraceDispatch))
	{
		__atomic_fetch_add(&s_TraceDropped, 1U, __ATOMIC_RELAXED);
	}
}
#else
static inline void Trace(uint32_t, const char *) {}
#endif

static USB_OTG_INEndpointTypeDef *In(unsigned Ep)
{
	return reinterpret_cast<USB_OTG_INEndpointTypeDef *>(
		reinterpret_cast<uintptr_t>(s_Usb.pReg) + USB_OTG_IN_ENDPOINT_BASE +
		Ep * USB_OTG_EP_REG_SIZE);
}

static USB_OTG_OUTEndpointTypeDef *Out(unsigned Ep)
{
	return reinterpret_cast<USB_OTG_OUTEndpointTypeDef *>(
		reinterpret_cast<uintptr_t>(s_Usb.pReg) + USB_OTG_OUT_ENDPOINT_BASE +
		Ep * USB_OTG_EP_REG_SIZE);
}

static volatile uint32_t *Fifo(unsigned Ep)
{
	return reinterpret_cast<volatile uint32_t *>(
		reinterpret_cast<uintptr_t>(s_Usb.pReg) + USB_OTG_FIFO_BASE +
		Ep * USB_OTG_FIFO_SIZE);
}

static bool Wait(volatile uint32_t *pReg, uint32_t Mask, bool Set)
{
	for (unsigned i = 0; i < USB_WAIT_LIMIT; i++)
	{
		if (((*pReg & Mask) != 0U) == Set)
		{
			return true;
		}
	}
	return false;
}

static void BusEvent(UsbCtrlrEvtType_t Type)
{
	UsbCtrlrEvt_t evt = {};
	evt.Type = Type;
	UsbDevProcessEvent(0, &evt);
}

static void Notify(unsigned Ep, bool bIn, UsbCtrlrEvtType_t Type, uint16_t Length)
{
	Stm32UsbEp_t *ep = &s_Usb.Ep[Ep][bIn];
	if (ep->Handler != nullptr)
	{
		ep->Handler(Type, Length, ep->pContext);
	}
}

static void Forget(unsigned Ep, bool bIn)
{
	Stm32UsbEp_t *ep = &s_Usb.Ep[Ep][bIn];
	const bool busy = ep->Busy;
	ep->Busy = false;
	ep->pBuffer = nullptr;
	ep->Mps = 0;
	ep->Halted = false;
	ep->Generation++;
	if (busy && Ep != 0)
	{
		Notify(Ep, bIn, USB_CTRLR_EVT_CANCEL, 0);
	}
}

static void Fault(void)
{
	Trace(0, "controller fault");
	NVIC_DisableIRQ(Stm32UsbOtgIrq());
	Stm32UsbOtgReset();
	s_Usb.Started = false;
	s_Usb.Closing = true;
	for (unsigned ep = 0; ep < USB_EP_COUNT; ep++)
	{
		Forget(ep, false);
		Forget(ep, true);
	}
	s_Usb.Closing = false;
	s_Usb.Recover = true;
	UsbProcessQue(0);
}

static bool FlushTx(unsigned Ep)
{
	s_Usb.pReg->GRSTCTL = USB_OTG_GRSTCTL_TXFFLSH |
		(Ep << USB_OTG_GRSTCTL_TXFNUM_Pos);
	return Wait(&s_Usb.pReg->GRSTCTL, USB_OTG_GRSTCTL_TXFFLSH, false);
}

// Read every word, including rejected bytes, so the next RX status stays in sync.
static void ReadFifo(uint8_t *pBuffer, unsigned Capacity, unsigned Length)
{
	for (unsigned offset = 0; offset < Length; offset += 4)
	{
		uint32_t word = *Fifo(0);
		for (unsigned i = 0; i < 4 && offset + i < Length; i++)
		{
			if (pBuffer != nullptr && offset + i < Capacity)
			{
				pBuffer[offset + i] = static_cast<uint8_t>(word);
			}
			word >>= 8;
		}
	}
}

static void ReceiveFifo(void)
{
	const uint32_t status = s_Usb.pReg->GRXSTSP;
	const unsigned epno = status & USB_OTG_GRXSTSP_EPNUM;
	const unsigned length = (status & USB_OTG_GRXSTSP_BCNT) >> 4;
	const unsigned packet = (status & USB_OTG_GRXSTSP_PKTSTS) >> 17;
	if (packet == 6 && epno == 0 && length == sizeof(UsbSetupData_t))
	{
		ReadFifo(reinterpret_cast<uint8_t *>(&s_Usb.Setup), sizeof(s_Usb.Setup), length);
		s_Usb.SetupPending = true;
		return;
	}
	if (packet == 2 && epno < USB_EP_COUNT)
	{
		Stm32UsbEp_t *ep = &s_Usb.Ep[epno][0];
		const unsigned capacity = ep->Busy ? ep->Length - ep->Actual : 0;
		ReadFifo(ep->pBuffer != nullptr ? ep->pBuffer + ep->Actual : nullptr,
			capacity, length);
		if (length > capacity)
		{
			ep->Overflow = true;
		}
		else
		{
			ep->Actual += length;
		}
		return;
	}
	ReadFifo(nullptr, 0, length);
}

static bool Retire(unsigned Ep, bool bIn)
{
	if (bIn)
	{
		if ((In(Ep)->DIEPCTL & USB_OTG_DIEPCTL_EPENA) != 0)
		{
			In(Ep)->DIEPCTL |= USB_OTG_DIEPCTL_SNAK;
			In(Ep)->DIEPCTL |= USB_OTG_DIEPCTL_EPDIS;
			if (!Wait(&In(Ep)->DIEPCTL, USB_OTG_DIEPCTL_EPENA, false))
			{
				return false;
			}
		}
		In(Ep)->DIEPINT = 0xFFFFFFFFU;
		return FlushTx(Ep);
	}
	if (Ep == 0 || (Out(Ep)->DOEPCTL & USB_OTG_DOEPCTL_EPENA) == 0)
	{
		return true;
	}
	Device()->DCTL |= USB_OTG_DCTL_SGONAK;
	// Slave/FIFO mode acknowledges the global OUT NAK through the RX queue.
	for (unsigned i = 0; i < USB_WAIT_LIMIT; i++)
	{
		if ((s_Usb.pReg->GINTSTS & USB_OTG_GINTSTS_RXFLVL) != 0)
		{
			ReceiveFifo();
		}
		if ((s_Usb.pReg->GINTSTS & USB_OTG_GINTSTS_BOUTNAKEFF) != 0)
		{
			Out(Ep)->DOEPCTL |= USB_OTG_DOEPCTL_EPDIS | USB_OTG_DOEPCTL_SNAK;
			const bool result = Wait(&Out(Ep)->DOEPCTL, USB_OTG_DOEPCTL_EPENA, false);
			Out(Ep)->DOEPINT = 0xFFFFFFFFU;
			Device()->DCTL |= USB_OTG_DCTL_CGONAK;
			return result;
		}
	}
	Device()->DCTL |= USB_OTG_DCTL_CGONAK;
	return false;
}

static bool AllocateTx(unsigned Ep, unsigned Mps)
{
	const unsigned words = Mps > 64 ? (Mps + 3) / 4 : 16;
	unsigned top = Stm32UsbOtgFifoWords();
	for (unsigned pass = 0; pass < USB_EP_COUNT; pass++)
	{
		if (top < s_Usb.RxWords + words)
		{
			return false;
		}
		const unsigned start = top - words;
		unsigned next = top;
		for (unsigned ep = 0; ep < USB_EP_COUNT; ep++)
		{
			if (ep != Ep && s_Usb.TxWords[ep] != 0 && s_Usb.TxBase[ep] < top &&
				s_Usb.TxBase[ep] + s_Usb.TxWords[ep] > start && s_Usb.TxBase[ep] < next)
			{
				next = s_Usb.TxBase[ep];
			}
		}
		if (next != top)
		{
			top = next;
			continue;
		}
		s_Usb.TxBase[Ep] = start;
		s_Usb.TxWords[Ep] = words;
		if (Ep == 0)
		{
			s_Usb.pReg->DIEPTXF0_HNPTXFSIZ = (words << 16) | start;
		}
		else
		{
			s_Usb.pReg->DIEPTXF[Ep - 1] = (words << 16) | start;
		}
		return true;
	}
	return false;
}

static bool GrowRx(unsigned Mps)
{
	// SETUP/control overhead, two maximum OUT packets and endpoint status words.
	const unsigned words = 14 + 2 * ((Mps + 3) / 4 + 1) + 2 * USB_EP_COUNT;
	if (words <= s_Usb.RxWords)
	{
		return true;
	}
	for (unsigned ep = 0; ep < USB_EP_COUNT; ep++)
	{
		if (s_Usb.TxWords[ep] != 0 && words > s_Usb.TxBase[ep])
		{
			return false;
		}
	}
	if (words > Stm32UsbOtgFifoWords())
	{
		return false;
	}
	s_Usb.RxWords = words;
	s_Usb.pReg->GRXFSIZ = words;
	return true;
}

static void PrimeSetup(void)
{
	Out(0)->DOEPTSIZ = (3U << USB_OTG_DOEPTSIZ_STUPCNT_Pos) |
		(1U << USB_OTG_DOEPTSIZ_PKTCNT_Pos) | 24U;
	Out(0)->DOEPCTL |= USB_OTG_DOEPCTL_EPENA | USB_OTG_DOEPCTL_USBAEP;
}

static void ArmOut(unsigned Ep)
{
	Stm32UsbEp_t *ep = &s_Usb.Ep[Ep][0];
	ep->Actual = 0;
	ep->Overflow = false;
	Out(Ep)->DOEPINT = USB_OTG_DOEPINT_XFRC;
	Out(Ep)->DOEPTSIZ = ep->Mps | (1U << USB_OTG_DOEPTSIZ_PKTCNT_Pos) |
		(Ep == 0 ? 3U << USB_OTG_DOEPTSIZ_STUPCNT_Pos : 0);
	const uint32_t ctl = Out(Ep)->DOEPCTL;
	Out(Ep)->DOEPCTL = ctl | USB_OTG_DOEPCTL_EPENA | USB_OTG_DOEPCTL_CNAK;
}

static void ArmIn(unsigned Ep)
{
	Stm32UsbEp_t *ep = &s_Usb.Ep[Ep][1];
	In(Ep)->DIEPINT = USB_OTG_DIEPINT_XFRC;
	In(Ep)->DIEPTSIZ = ep->Length | (1U << USB_OTG_DIEPTSIZ_PKTCNT_Pos);
	const uint32_t ctl = In(Ep)->DIEPCTL;
	In(Ep)->DIEPCTL = ctl | USB_OTG_DIEPCTL_EPENA | USB_OTG_DIEPCTL_CNAK;
	for (unsigned offset = 0; offset < ep->Length; offset += 4)
	{
		uint32_t word = 0;
		const unsigned remain = ep->Length - offset;
		memcpy(&word, ep->pBuffer + offset, remain < 4 ? remain : 4);
		*Fifo(Ep) = word;
	}
}

static void ResetEndpoints(void)
{
	s_Usb.Closing = true;
	s_Usb.RxReady = 0;
	s_Usb.ControlRemaining = 0;
	s_Usb.SetupPending = false;
	s_Usb.Suspended = false;
	s_Usb.SofEnabled = false;
	Device()->DAINTMSK = 0;
	Device()->DIEPEMPMSK = 0;
	Device()->DCFG &= ~USB_OTG_DCFG_DAD;
	Device()->DCTL &= ~USB_OTG_DCTL_RWUSIG;
	s_Usb.pReg->GINTMSK &= ~USB_OTG_GINTSTS_SOF;
	for (unsigned ep = 0; ep < USB_EP_COUNT; ep++)
	{
		In(ep)->DIEPCTL = USB_OTG_DIEPCTL_SNAK;
		Out(ep)->DOEPCTL = USB_OTG_DOEPCTL_SNAK;
		In(ep)->DIEPINT = 0xFFFFFFFFU;
		Out(ep)->DOEPINT = 0xFFFFFFFFU;
		Forget(ep, false);
		Forget(ep, true);
		s_Usb.TxWords[ep] = 0;
	}
	s_Usb.Closing = false;
	if (!FlushTx(0x10))
	{
		Fault();
		return;
	}
	s_Usb.pReg->GRSTCTL = USB_OTG_GRSTCTL_RXFFLSH;
	if (!Wait(&s_Usb.pReg->GRSTCTL, USB_OTG_GRSTCTL_RXFFLSH, false))
	{
		Fault();
		return;
	}
	s_Usb.RxWords = 0;
	GrowRx(USB_CTRLR0_INT_PKT_LEN_MAX);
	AllocateTx(0, USB_EP0_MPS);
	s_Usb.Ep[0][0].Mps = USB_EP0_MPS;
	s_Usb.Ep[0][1].Mps = USB_EP0_MPS;
	In(0)->DIEPCTL = USB_OTG_DIEPCTL_USBAEP;
	Out(0)->DOEPCTL = USB_OTG_DOEPCTL_USBAEP;
	Device()->DIEPMSK = USB_OTG_DIEPMSK_XFRCM;
	Device()->DOEPMSK = USB_OTG_DOEPMSK_XFRCM | USB_OTG_DOEPMSK_STUPM;
	Device()->DAINTMSK = 0x10001U;
	PrimeSetup();
}

bool UsbCtrlrInit(int DevNo, const UsbCtrlrCfg_t *pCfg)
{
	if (DevNo != 0 || pCfg == nullptr || s_Usb.Started || s_Usb.Closing ||
		pCfg->IntPrio < 0 || pCfg->IntPrio >= (1 << __NVIC_PRIO_BITS))
	{
		return false;
	}
	if (!Stm32UsbOtgPowerInit(pCfg))
	{
		return false;
	}
	memset(&s_Usb, 0, sizeof(s_Usb));
	s_Usb.pReg = Stm32UsbOtgRegs();
	s_Usb.Initialized = true;
	NVIC_DisableIRQ(Stm32UsbOtgIrq());
	NVIC_SetPriority(Stm32UsbOtgIrq(), pCfg->IntPrio);
	return true;
}

bool UsbCtrlrStart(int DevNo)
{
	if (DevNo != 0 || !s_Usb.Initialized || s_Usb.Closing)
	{
		return false;
	}
	if (s_Usb.Started)
	{
		return true;
	}
	Stm32UsbOtgReset();
	USB_OTG_GlobalTypeDef *reg = s_Usb.pReg;
#if defined(STM32WBA65xx)
	reg->GUSBCFG &= ~USB_OTG_GUSBCFG_TSDPS;
#else
	reg->GUSBCFG |= USB_OTG_GUSBCFG_PHYSEL;
#endif
	if (!Wait(&reg->GRSTCTL, USB_OTG_GRSTCTL_AHBIDL, true))
	{
		return false;
	}
	reg->GRSTCTL |= USB_OTG_GRSTCTL_CSRST;
#if defined(USB_OTG_GRSTCTL_CSRSTDONE)
	if (!Wait(&reg->GRSTCTL, USB_OTG_GRSTCTL_CSRSTDONE, true))
	{
		return false;
	}
	reg->GRSTCTL &= ~USB_OTG_GRSTCTL_CSRST;
#else
	if (!Wait(&reg->GRSTCTL, USB_OTG_GRSTCTL_CSRST, false))
	{
		return false;
	}
#endif
	reg->GUSBCFG = (reg->GUSBCFG & ~USB_OTG_GUSBCFG_FHMOD) | USB_OTG_GUSBCFG_FDMOD;
	usDelay(50000);
	Device()->DCTL = USB_OTG_DCTL_SDIS;
#if defined(STM32WBA65xx)
	reg->GCCFG &= ~(USB_OTG_GCCFG_PULLDOWNEN | USB_OTG_GCCFG_VBVALEXTOEN);
	reg->GCCFG |= USB_OTG_GCCFG_VBDEN;
	Device()->DCFG = 0;
#else
	reg->GCCFG |= USB_OTG_GCCFG_PWRDWN;
#if defined(USB_OTG_GCCFG_VBUSBSEN)
	reg->GCCFG = (reg->GCCFG & ~USB_OTG_GCCFG_NOVBUSSENS) | USB_OTG_GCCFG_VBUSBSEN;
#else
	reg->GCCFG |= USB_OTG_GCCFG_VBDEN;
#endif
	Device()->DCFG = USB_OTG_DCFG_DSPD;
#endif
	reg->GAHBCFG = 0;
	reg->GINTMSK = 0;
	reg->GINTSTS = 0xFFFFFFFFU;
	reg->GOTGINT = 0xFFFFFFFFU;
	s_Usb.Started = true;
	ResetEndpoints();
	if (!s_Usb.Started)
	{
		return false;
	}
	reg->GINTMSK = USB_OTG_GINTSTS_USBRST | USB_OTG_GINTSTS_ENUMDNE |
		USB_OTG_GINTSTS_RXFLVL | USB_OTG_GINTSTS_IEPINT | USB_OTG_GINTSTS_OEPINT |
		USB_OTG_GINTSTS_USBSUSP | USB_OTG_GINTSTS_WKUINT |
		USB_OTG_GINTSTS_SRQINT | USB_OTG_GINTSTS_OTGINT;
	NVIC_ClearPendingIRQ(Stm32UsbOtgIrq());
	return true;
}

void UsbCtrlrStop(int DevNo)
{
	if (DevNo != 0 || !s_Usb.Started)
	{
		return;
	}
	const uint32_t state = DisableInterrupt();
	Fault();
	s_Usb.Recover = false;
	EnableInterrupt(state);
}

void UsbCtrlrProcess(int DevNo)
{
	if (DevNo == 0 && s_Usb.Recover)
	{
		s_Usb.Recover = false;
		BusEvent(USB_CTRLR_EVT_RESET);
		if (UsbCtrlrStart(0))
		{
			UsbCtrlrIntEnable(0);
			UsbCtrlrConnect(0);
		}
	}
}

bool UsbCtrlrVbusDetected(int DevNo)
{
	return DevNo == 0 && s_Usb.Initialized &&
		#if defined(USB_OTG_GOTGCTL_BSESVLD)
		(s_Usb.pReg->GOTGCTL & USB_OTG_GOTGCTL_BSESVLD) != 0;
#else
		(s_Usb.pReg->GOTGCTL & USB_OTG_GOTGCTL_BSVLD) != 0;
#endif
}

bool UsbCtrlrHighSpeed(int DevNo)
{
	return DevNo == 0 && s_Usb.HighSpeed;
}

void UsbCtrlrIntEnable(int DevNo)
{
	if (DevNo == 0 && s_Usb.Started)
	{
		s_Usb.pReg->GAHBCFG |= USB_OTG_GAHBCFG_GINT;
		NVIC_EnableIRQ(Stm32UsbOtgIrq());
	}
}

void UsbCtrlrIntDisable(int DevNo)
{
	if (DevNo == 0)
	{
		NVIC_DisableIRQ(Stm32UsbOtgIrq());
	}
}

void UsbCtrlrConnect(int DevNo)
{
	if (DevNo == 0 && s_Usb.Started)
	{
		Device()->DCTL &= ~USB_OTG_DCTL_SDIS;
		Trace(0, "connected");
	}
}

void UsbCtrlrDisconnect(int DevNo)
{
	if (DevNo == 0 && s_Usb.Started)
	{
		Device()->DCTL |= USB_OTG_DCTL_SDIS;
	}
}

void UsbCtrlrRemoteWakeup(int DevNo)
{
	if (DevNo == 0 && s_Usb.Started && s_Usb.Suspended && UsbCtrlrVbusDetected(0))
	{
		// Meet the minimum suspend quiet time, then signal for 5 ms (USB 7.1.7.7).
		usDelay(5000);
		if ((Device()->DSTS & USB_OTG_DSTS_SUSPSTS) != 0)
		{
			Device()->DCTL |= USB_OTG_DCTL_RWUSIG;
			usDelay(5000);
			Device()->DCTL &= ~USB_OTG_DCTL_RWUSIG;
		}
	}
}

void UsbCtrlrSofEnable(int DevNo, bool Enable)
{
	if (DevNo == 0 && s_Usb.Started)
	{
		s_Usb.SofEnabled = Enable;
		if (Enable)
		{
			s_Usb.pReg->GINTMSK |= USB_OTG_GINTSTS_SOF;
		}
		else
		{
			s_Usb.pReg->GINTMSK &= ~USB_OTG_GINTSTS_SOF;
		}
	}
}

void UsbCtrlrSetAddress(int DevNo, uint8_t Address)
{
	if (DevNo == 0 && s_Usb.Started && Address < 128)
	{
		Device()->DCFG = (Device()->DCFG & ~USB_OTG_DCFG_DAD) |
			(static_cast<uint32_t>(Address) << USB_OTG_DCFG_DAD_Pos);
	}
}

void UsbCtrlrEpBind(int DevNo, uint8_t EpNo, bool bIn, bool bBlocking,
	UsbCtrlrEpHandler_t Handler, void *pContext)
{
	(void)bBlocking;
	if (DevNo != 0 || EpNo == 0 || EpNo >= USB_EP_COUNT)
	{
		return;
	}
	const uint32_t state = DisableInterrupt();
	Stm32UsbEp_t *ep = &s_Usb.Ep[EpNo][bIn];
	if (!s_Usb.Closing && !ep->Busy)
	{
		ep->Handler = Handler;
		ep->pContext = pContext;
		ep->Generation++;
	}
	EnableInterrupt(state);
}

bool UsbCtrlrEpOpenData(int DevNo, uint8_t EpNo, bool bIn, uint8_t Type, uint16_t MaxPacketSize)
{
	const unsigned limit = Type == BULK ? (s_Usb.HighSpeed ? 512 : 64) :
		(s_Usb.HighSpeed ? 1024 : 64);
	if (DevNo != 0 || EpNo == 0 || EpNo >= USB_EP_COUNT || !s_Usb.Started ||
		s_Usb.Closing || (Type != BULK && Type != INT) ||
		MaxPacketSize == 0 || MaxPacketSize > limit ||
		(Type == BULK && (s_Usb.HighSpeed ? MaxPacketSize != 512 :
		(MaxPacketSize != 8 && MaxPacketSize != 16 && MaxPacketSize != 32 && MaxPacketSize != 64))))
	{
		return false;
	}
	const uint32_t state = DisableInterrupt();
	Stm32UsbEp_t *ep = &s_Usb.Ep[EpNo][bIn];
	if (ep->Mps != 0 || (bIn && !AllocateTx(EpNo, MaxPacketSize)))
	{
		EnableInterrupt(state);
		return false;
	}
	ep->Mps = MaxPacketSize;
	ep->Type = Type;
	ep->Busy = false;
	ep->Halted = false;
	ep->Generation++;
	const uint32_t ctl = MaxPacketSize | (static_cast<uint32_t>(Type) << 18) |
		USB_OTG_DIEPCTL_USBAEP | USB_OTG_DIEPCTL_SD0PID_SEVNFRM | USB_OTG_DIEPCTL_SNAK;
	if (bIn)
	{
		In(EpNo)->DIEPCTL = ctl | (static_cast<uint32_t>(EpNo) << USB_OTG_DIEPCTL_TXFNUM_Pos);
		In(EpNo)->DIEPINT = 0xFFFFFFFFU;
		Device()->DAINTMSK |= 1U << EpNo;
	}
	else
	{
		Out(EpNo)->DOEPCTL = ctl;
		Out(EpNo)->DOEPINT = 0xFFFFFFFFU;
		Device()->DAINTMSK |= 1U << (EpNo + 16);
		s_Usb.RxReady |= 1U << EpNo;
		NVIC_SetPendingIRQ(Stm32UsbOtgIrq());
	}
	EnableInterrupt(state);
	return true;
}

bool UsbCtrlrEpOpen(int DevNo, const UsbEndPointDesc_t *pDesc)
{
	return pDesc != nullptr && (pDesc->bEndpointAddress & 0x70U) == 0 &&
		UsbCtrlrEpOpenData(DevNo, pDesc->bEndpointAddress & 15U,
			USB_ENDPADDR_IS_IN(pDesc->bEndpointAddress), pDesc->bmAttributes & 3U,
			pDesc->wMaxPacketSize);
}

void UsbCtrlrEpClose(int DevNo, uint8_t EpNo, bool bIn)
{
	if (DevNo != 0 || EpNo == 0 || EpNo >= USB_EP_COUNT || !s_Usb.Started || s_Usb.Closing)
	{
		return;
	}
	const uint32_t state = DisableInterrupt();
	s_Usb.Closing = true;
	if (!Retire(EpNo, bIn))
	{
		Fault();
		EnableInterrupt(state);
		return;
	}
	Device()->DAINTMSK &= ~(1U << (EpNo + (bIn ? 0 : 16)));
	if (bIn)
	{
		In(EpNo)->DIEPCTL = USB_OTG_DIEPCTL_SNAK;
		s_Usb.TxWords[EpNo] = 0;
	}
	else
	{
		Out(EpNo)->DOEPCTL = USB_OTG_DOEPCTL_SNAK;
		s_Usb.RxReady &= ~(1U << EpNo);
	}
	Forget(EpNo, bIn);
	s_Usb.Closing = false;
	EnableInterrupt(state);
}

void UsbCtrlrEpCloseAll(int DevNo)
{
	for (unsigned ep = 1; ep < USB_EP_COUNT; ep++)
	{
		UsbCtrlrEpClose(DevNo, ep, false);
		UsbCtrlrEpClose(DevNo, ep, true);
	}
}

bool UsbCtrlrEpReceive(int DevNo, uint8_t EpNo, uint8_t *pBuffer, uint16_t Capacity)
{
	if (DevNo != 0 || EpNo == 0 || EpNo >= USB_EP_COUNT || pBuffer == nullptr)
	{
		return false;
	}
	const uint32_t state = DisableInterrupt();
	Stm32UsbEp_t *ep = &s_Usb.Ep[EpNo][0];
	const bool accepted = s_Usb.Started && !s_Usb.Closing && ep->Mps != 0 &&
		!ep->Busy && ep->pBuffer == nullptr && Capacity >= ep->Mps;
	if (accepted)
	{
		ep->pBuffer = pBuffer;
		ep->Length = ep->Mps;
		s_Usb.RxReady &= ~(1U << EpNo);
		ep->Busy = true;
		if (!ep->Halted)
		{
			ArmOut(EpNo);
		}
	}
	EnableInterrupt(state);
	return accepted;
}

bool UsbCtrlrEpSend(int DevNo, uint8_t EpNum, uint8_t *pBuffer, uint16_t Length)
{
	if (DevNo != 0 || EpNum == 0 || EpNum >= USB_EP_COUNT || (Length != 0 && pBuffer == nullptr))
	{
		return false;
	}
	const uint32_t state = DisableInterrupt();
	Stm32UsbEp_t *ep = &s_Usb.Ep[EpNum][1];
	const bool accepted = s_Usb.Started && !s_Usb.Closing && ep->Mps != 0 && !ep->Busy;
	if (accepted)
	{
		ep->pBuffer = pBuffer;
		ep->Length = Length < ep->Mps ? Length : ep->Mps;
		ep->Busy = true;
		if (!ep->Halted)
		{
			ArmIn(EpNum);
		}
	}
	EnableInterrupt(state);
	return accepted;
}

int UsbCtrlrEp0Send(int DevNo, uint8_t *pBuffer, int Length)
{
	if (DevNo != 0 || Length < 0 || (Length != 0 && pBuffer == nullptr))
	{
		return -1;
	}
	const uint32_t state = DisableInterrupt();
	Stm32UsbEp_t *ep = &s_Usb.Ep[0][1];
	if (!s_Usb.Started || s_Usb.Closing || ep->Busy)
	{
		EnableInterrupt(state);
		return -1;
	}
	ep->Length = Length < USB_EP0_MPS ? Length : USB_EP0_MPS;
	ep->pBuffer = pBuffer;
	ep->Busy = true;
	ArmIn(0);
	// FIFO now owns the EP0 source bytes, including stack-local replies.
	ep->pBuffer = nullptr;
	const int length = ep->Length;
	EnableInterrupt(state);
	return length;
}

bool UsbCtrlrEp0Status(int DevNo, uint8_t EpAddr)
{
	if (EpAddr == USB_ENDPADDR_DIR_IN)
	{
		return UsbCtrlrEp0Send(DevNo, nullptr, 0) == 0;
	}
	if (DevNo != 0 || EpAddr != 0)
	{
		return false;
	}
	const uint32_t state = DisableInterrupt();
	if (!s_Usb.Started || s_Usb.Closing || s_Usb.Ep[0][0].Busy)
	{
		EnableInterrupt(state);
		return false;
	}
	s_Usb.Ep[0][0].Busy = true;
	s_Usb.Ep[0][0].pBuffer = s_Usb.Ep0Out;
	s_Usb.Ep[0][0].Length = USB_EP0_MPS;
	s_Usb.ControlRemaining = 0;
	ArmOut(0);
	EnableInterrupt(state);
	return true;
}

void UsbCtrlrEpStall(int DevNo, uint8_t EpNo, bool bIn)
{
	if (DevNo != 0 || EpNo >= USB_EP_COUNT || !s_Usb.Started)
	{
		return;
	}
	const uint32_t state = DisableInterrupt();
	if (!Retire(EpNo, EpNo == 0 ? true : bIn))
	{
		Fault();
	}
	else if (EpNo == 0)
	{
		s_Usb.ControlRemaining = 0;
		s_Usb.Ep[0][0].Busy = false;
		s_Usb.Ep[0][1].Busy = false;
		In(0)->DIEPCTL |= USB_OTG_DIEPCTL_STALL;
		Out(0)->DOEPCTL |= USB_OTG_DOEPCTL_STALL;
		PrimeSetup();
	}
	else if (s_Usb.Ep[EpNo][bIn].Type != ISO)
	{
		s_Usb.Ep[EpNo][bIn].Halted = true;
		if (bIn)
		{
			In(EpNo)->DIEPCTL |= USB_OTG_DIEPCTL_STALL;
		}
		else
		{
			Out(EpNo)->DOEPCTL |= USB_OTG_DOEPCTL_STALL;
		}
	}
	EnableInterrupt(state);
}

void UsbCtrlrEpClearStall(int DevNo, uint8_t EpNo, bool bIn)
{
	if (DevNo != 0 || EpNo == 0 || EpNo >= USB_EP_COUNT || !s_Usb.Started)
	{
		return;
	}
	const uint32_t state = DisableInterrupt();
	Stm32UsbEp_t *ep = &s_Usb.Ep[EpNo][bIn];
	if (ep->Mps != 0 && ep->Type != ISO)
	{
		ep->Halted = false;
		if (bIn)
		{
			In(EpNo)->DIEPCTL = (In(EpNo)->DIEPCTL & ~USB_OTG_DIEPCTL_STALL) |
				USB_OTG_DIEPCTL_SD0PID_SEVNFRM;
			if (ep->Busy)
			{
				ArmIn(EpNo);
			}
		}
		else
		{
			Out(EpNo)->DOEPCTL = (Out(EpNo)->DOEPCTL & ~USB_OTG_DOEPCTL_STALL) |
				USB_OTG_DOEPCTL_SD0PID_SEVNFRM;
			if (ep->Busy)
			{
				ArmOut(EpNo);
			}
			else
			{
				s_Usb.RxReady |= 1U << EpNo;
				NVIC_SetPendingIRQ(Stm32UsbOtgIrq());
			}
		}
	}
	EnableInterrupt(state);
}

static void Complete(unsigned Ep, bool bIn, bool Failed)
{
	Stm32UsbEp_t *ep = &s_Usb.Ep[Ep][bIn];
	if (!ep->Busy)
	{
		return;
	}
	const uint16_t length = bIn ? ep->Length : ep->Actual;
	const uint32_t generation = ep->Generation;
	ep->Busy = false;
	ep->pBuffer = nullptr;
	if (Ep == 0)
	{
		if (!bIn)
		{
			Failed = Failed || length > s_Usb.ControlRemaining;
			s_Usb.ControlRemaining = length <= s_Usb.ControlRemaining ?
				s_Usb.ControlRemaining - length : 0;
			if (length < USB_EP0_MPS)
			{
				s_Usb.ControlRemaining = 0;
			}
		}
		UsbCtrlrEvt_t evt = {};
		evt.Type = USB_CTRLR_EVT_XFER_CMPL;
		evt.Xfer.EpAddr = bIn ? USB_ENDPADDR_DIR_IN : 0;
		evt.Xfer.Length = length;
		evt.Xfer.Result = Failed ? USB_CTRLR_XFER_FAILED : USB_CTRLR_XFER_SUCCESS;
		evt.Xfer.pBuffer = s_Usb.Ep0Out;
		UsbDevProcessEvent(0, &evt);
		if (!bIn && s_Usb.ControlRemaining != 0 &&
			(Out(0)->DOEPCTL & USB_OTG_DOEPCTL_STALL) == 0)
		{
			ep->Busy = true;
			ep->pBuffer = s_Usb.Ep0Out;
			ep->Length = USB_EP0_MPS;
			ArmOut(0);
		}
		else if (!s_Usb.Ep[0][0].Busy)
		{
			PrimeSetup();
		}
		return;
	}
	Notify(Ep, bIn, Failed ? USB_CTRLR_EVT_XFER_FAILED : USB_CTRLR_EVT_XFER_CMPL,
		Failed ? 0 : length);
	if (!bIn && ep->Generation == generation && ep->Mps != 0 &&
		!ep->Busy && ep->Type != ISO && !s_Usb.Closing)
	{
		Notify(Ep, false, USB_CTRLR_EVT_DRDY, 0);
	}
}

static void Setup(void)
{
	if (!s_Usb.SetupPending)
	{
		return;
	}
	s_Usb.SetupPending = false;
	if (!Retire(0, true))
	{
		Fault();
		return;
	}
	s_Usb.ControlRemaining = 0;
	s_Usb.Ep[0][0].Busy = false;
	s_Usb.Ep[0][1].Busy = false;
	In(0)->DIEPCTL &= ~USB_OTG_DIEPCTL_STALL;
	Out(0)->DOEPCTL &= ~USB_OTG_DOEPCTL_STALL;
	UsbCtrlrEvt_t evt = {};
	evt.Type = USB_CTRLR_EVT_SETUP;
	evt.Setup = s_Usb.Setup;
	UsbDevProcessEvent(0, &evt);
	if ((evt.Setup.bmRequestType & USB_REQTYPE_MASK_DIR) == 0 &&
		evt.Setup.wLength != 0 && (Out(0)->DOEPCTL & USB_OTG_DOEPCTL_STALL) == 0)
	{
		s_Usb.ControlRemaining = evt.Setup.wLength;
		s_Usb.Ep[0][0].Busy = true;
		s_Usb.Ep[0][0].pBuffer = s_Usb.Ep0Out;
		s_Usb.Ep[0][0].Length = USB_EP0_MPS;
		ArmOut(0);
	}
	else
	{
		PrimeSetup();
	}
}

// Called by the target's vector. Endpoint callbacks run here and may immediately
// replace the retired buffer. Bus events alone queue application work.
void Stm32UsbOtgInterrupt(void)
{
	if (!s_Usb.Started)
	{
		return;
	}
	USB_OTG_GlobalTypeDef *reg = s_Usb.pReg;
	uint32_t status = reg->GINTSTS & reg->GINTMSK;
	const uint32_t bus = status & (USB_OTG_GINTSTS_USBRST | USB_OTG_GINTSTS_ENUMDNE |
		USB_OTG_GINTSTS_USBSUSP | USB_OTG_GINTSTS_WKUINT |
		USB_OTG_GINTSTS_SRQINT | USB_OTG_GINTSTS_OTGINT);
	if (bus != 0)
	{
		Trace(status, "bus");
	}
	if ((status & USB_OTG_GINTSTS_USBRST) != 0)
	{
		reg->GINTSTS = USB_OTG_GINTSTS_USBRST;
		ResetEndpoints();
		BusEvent(USB_CTRLR_EVT_RESET);
		return;
	}
	if ((status & USB_OTG_GINTSTS_ENUMDNE) != 0)
	{
		reg->GINTSTS = USB_OTG_GINTSTS_ENUMDNE;
		s_Usb.HighSpeed = (Device()->DSTS & USB_OTG_DSTS_ENUMSPD) == 0;
		Device()->DCTL |= USB_OTG_DCTL_CGINAK;
		unsigned trdt = 6;
		const uint32_t clock = SystemCoreClockGet();
		const uint32_t thresholds[] = {32000000, 27700000, 24000000, 21800000,
			20000000, 18500000, 17200000, 16000000, 15000000};
		for (unsigned i = 0; i < sizeof(thresholds) / sizeof(thresholds[0]); i++)
		{
			if (clock < thresholds[i])
			{
				trdt++;
			}
		}
		if (s_Usb.HighSpeed)
		{
			trdt = 9;
		}
		reg->GUSBCFG = (reg->GUSBCFG & ~USB_OTG_GUSBCFG_TRDT) |
			(trdt << USB_OTG_GUSBCFG_TRDT_Pos);
	}
	// Drain receive statuses before endpoint completion, including a SETUP
	// that supersedes an old control IN completion in the same IRQ snapshot.
	for (unsigned n = 0; n < 32 && (reg->GINTSTS & USB_OTG_GINTSTS_RXFLVL) != 0; n++)
	{
		ReceiveFifo();
	}
	if ((reg->GINTSTS & USB_OTG_GINTSTS_RXFLVL) != 0)
	{
		return;
	}
	for (unsigned ep = 0; ep < USB_EP_COUNT; ep++)
	{
		if ((Device()->DAINT & Device()->DAINTMSK & (1U << (ep + 16))) != 0)
		{
			const uint32_t bits = Out(ep)->DOEPINT & Device()->DOEPMSK;
			Out(ep)->DOEPINT = bits;
			if (ep == 0 && (bits & USB_OTG_DOEPINT_STUP) != 0)
			{
				Setup();
			}
			else if ((bits & USB_OTG_DOEPINT_XFRC) != 0 && !(ep == 0 && s_Usb.SetupPending))
			{
				Complete(ep, false, s_Usb.Ep[ep][0].Overflow);
			}
		}
		if (!s_Usb.Started)
		{
			return;
		}
		if ((Device()->DAINT & Device()->DAINTMSK & (1U << ep)) != 0)
		{
			const uint32_t bits = In(ep)->DIEPINT & Device()->DIEPMSK;
			In(ep)->DIEPINT = bits;
			if ((bits & USB_OTG_DIEPINT_XFRC) != 0 && !(ep == 0 && s_Usb.SetupPending))
			{
				Complete(ep, true, false);
			}
		}
		if (!s_Usb.Started)
		{
			return;
		}
		if (ep != 0 && (s_Usb.RxReady & (1U << ep)) != 0)
		{
			s_Usb.RxReady &= ~(1U << ep);
			Notify(ep, false, USB_CTRLR_EVT_DRDY, 0);
		}
	}
	status = reg->GINTSTS & reg->GINTMSK;
	if ((status & USB_OTG_GINTSTS_USBSUSP) != 0)
	{
		reg->GINTSTS = USB_OTG_GINTSTS_USBSUSP;
		if ((Device()->DSTS & USB_OTG_DSTS_SUSPSTS) != 0)
		{
			s_Usb.Suspended = true;
			BusEvent(USB_CTRLR_EVT_SUSPEND);
		}
	}
	if ((status & USB_OTG_GINTSTS_WKUINT) != 0)
	{
		reg->GINTSTS = USB_OTG_GINTSTS_WKUINT;
		s_Usb.Suspended = false;
		BusEvent(USB_CTRLR_EVT_RESUME);
	}
	if ((status & (USB_OTG_GINTSTS_SRQINT | USB_OTG_GINTSTS_OTGINT)) != 0)
	{
		reg->GOTGINT = reg->GOTGINT;
		reg->GINTSTS = status & USB_OTG_GINTSTS_SRQINT;
		if (!UsbCtrlrVbusDetected(0))
		{
			ResetEndpoints();
			BusEvent(USB_CTRLR_EVT_RESET);
		}
		UsbProcessQue(0);
	}
	if ((status & USB_OTG_GINTSTS_SOF) != 0)
	{
		reg->GINTSTS = USB_OTG_GINTSTS_SOF;
		if (s_Usb.SofEnabled && !s_Usb.Suspended)
		{
			UsbCtrlrEvt_t evt = {};
			evt.Type = USB_CTRLR_EVT_SOF;
			evt.FrameNo = (((Device()->DSTS & USB_OTG_DSTS_FNSOF) >> USB_OTG_DSTS_FNSOF_Pos) + 1) & 0x3FFF;
			UsbDevProcessEvent(0, &evt);
		}
	}
}

void UsbCtrlrEpProcessEvent(int DevNo, uint8_t EpNo, bool bIn, UsbCtrlrEvtType_t Event, uint16_t Value)
{
	if (DevNo == 0 && EpNo != 0 && EpNo < USB_EP_COUNT && s_Usb.Ep[EpNo][bIn].Mps != 0)
	{
		Notify(EpNo, bIn, Event, Value);
	}
}

// ISO is deliberately unavailable until frame scheduling is validated.
bool UsbCtrlrIsoInit(int DevNo)
{
	(void)DevNo;
	return false;
}

bool UsbCtrlrIsoOpen(int DevNo, uint8_t EpNo, bool bIn, uint16_t MaxPacketSize)
{
	(void)DevNo;
	(void)EpNo;
	(void)bIn;
	(void)MaxPacketSize;
	return false;
}

bool UsbCtrlrIsoSend(int DevNo, uint8_t EpNum, uint8_t *pBuffer, uint16_t Length)
{
	(void)DevNo;
	(void)EpNum;
	(void)pBuffer;
	(void)Length;
	return false;
}

uint16_t UsbCtrlrIsoTraceSnapshot(int DevNo, uint8_t **ppData)
{
	(void)DevNo;
	if (ppData != nullptr)
	{
		*ppData = nullptr;
	}
	return 0;
}

size_t UsbCtrlrGetSerial(int DevNo, char *pBuff, size_t BuffLen)
{
	if (DevNo != 0 || pBuff == nullptr || BuffLen == 0)
	{
		return 0;
	}
	const volatile uint8_t *uid = reinterpret_cast<const volatile uint8_t *>(UID_BASE);
	const size_t length = BuffLen > 24 ? 24 : BuffLen - 1;
	const char hex[] = "0123456789ABCDEF";
	for (size_t i = 0; i < length; i++)
	{
		pBuff[i] = hex[(uid[i / 2] >> ((i & 1) ? 0 : 4)) & 15];
	}
	pBuff[length] = 0;
	return length;
}
