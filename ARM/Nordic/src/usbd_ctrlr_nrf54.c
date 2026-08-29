/**-------------------------------------------------------------------------
@file	usbd_ctrlr_nrf54.c

@brief	Native IOsonata USB device controller for Nordic nRF54 USBHS.

Implements usbd_ctrlr.h directly on the nRF54LM20 USBHS DesignWare device
core. Nordic's wrapper, regulator, PHY and clock sequencing remain in
usbd_nrfx.cpp. This controller owns endpoint state, internal DMA and
USBHS_IRQHandler. No TinyUSB code or header is required.

Only control, bulk and interrupt endpoints are implemented. Isochronous
support is intentionally left for a later milestone.

@author	Hoang Nguyen Hoan
@date	Aug. 29, 2026

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

#include "nrf.h"
#include "nrf_peripherals.h"
#include "soc/nrfx_coredep.h"

#include "usb/usbd_ctrlr.h"

#ifndef USBHS_PRESENT
#error "usbd_ctrlr_nrf54: this part has no USBHS peripheral"
#endif

#ifndef USBHSCORE_PRESENT
#error "usbd_ctrlr_nrf54: this part has no USBHS core"
#endif

#define NRF54_USBD_EP_COUNT				16U
#define NRF54_USBD_EP0_MPS				64U
#define NRF54_USBD_SETUP_COUNT			3U
#define NRF54_USBD_SETUP_SIZE			8U
#define NRF54_USBD_WAIT_LOOPS			1000000UL

#if defined(NRF_TRUSTZONE_NONSECURE)
#define NRF54_USBD_CORE					NRF_USBHSCORE_NS
#else
#define NRF54_USBD_CORE					NRF_USBHSCORE_S
#endif

#define NRF54_USBD_REG(ofs) \
	(*(volatile uint32_t *)((uintptr_t)NRF54_USBD_CORE + (ofs)))

#define NRF54_USBD_GAHBCFG				NRF54_USBD_REG(0x008U)
#define NRF54_USBD_GUSBCFG				NRF54_USBD_REG(0x00CU)
#define NRF54_USBD_GRSTCTL				NRF54_USBD_REG(0x010U)
#define NRF54_USBD_GINTSTS				NRF54_USBD_REG(0x014U)
#define NRF54_USBD_GINTMSK				NRF54_USBD_REG(0x018U)
#define NRF54_USBD_GRXFSIZ				NRF54_USBD_REG(0x024U)
#define NRF54_USBD_GNPTXFSIZ			NRF54_USBD_REG(0x028U)
#define NRF54_USBD_GHWCFG2				NRF54_USBD_REG(0x048U)
#define NRF54_USBD_GHWCFG3				NRF54_USBD_REG(0x04CU)
#define NRF54_USBD_GDFIFOCFG			NRF54_USBD_REG(0x05CU)

#define NRF54_USBD_DCFG					NRF54_USBD_REG(0x800U)
#define NRF54_USBD_DCTL					NRF54_USBD_REG(0x804U)
#define NRF54_USBD_DSTS					NRF54_USBD_REG(0x808U)
#define NRF54_USBD_DIEPMSK				NRF54_USBD_REG(0x810U)
#define NRF54_USBD_DOEPMSK				NRF54_USBD_REG(0x814U)
#define NRF54_USBD_DAINT				NRF54_USBD_REG(0x818U)
#define NRF54_USBD_DAINTMSK				NRF54_USBD_REG(0x81CU)

#define NRF54_USBD_DIEPCTL(ep)			NRF54_USBD_REG(0x900U + ((ep) * 0x20U))
#define NRF54_USBD_DIEPINT(ep)			NRF54_USBD_REG(0x908U + ((ep) * 0x20U))
#define NRF54_USBD_DIEPTSIZ(ep)			NRF54_USBD_REG(0x910U + ((ep) * 0x20U))
#define NRF54_USBD_DIEPDMA(ep)			NRF54_USBD_REG(0x914U + ((ep) * 0x20U))

#define NRF54_USBD_DOEPCTL(ep)			NRF54_USBD_REG(0xB00U + ((ep) * 0x20U))
#define NRF54_USBD_DOEPINT(ep)			NRF54_USBD_REG(0xB08U + ((ep) * 0x20U))
#define NRF54_USBD_DOEPTSIZ(ep)			NRF54_USBD_REG(0xB10U + ((ep) * 0x20U))
#define NRF54_USBD_DOEPDMA(ep)			NRF54_USBD_REG(0xB14U + ((ep) * 0x20U))

#define NRF54_USBD_DIEPTXF(ep)			NRF54_USBD_REG(0x104U + (((ep) - 1U) * 4U))

#define NRF54_USBD_GAHBCFG_GINT			(1UL << 0)
#define NRF54_USBD_GAHBCFG_HBSTLEN_INCR4	(1UL << 1)
#define NRF54_USBD_GAHBCFG_DMAEN			(1UL << 5)

#define NRF54_USBD_GUSBCFG_FORCEDEVMODE	(1UL << 30)
#define NRF54_USBD_GUSBCFG_FORCEHSTMODE	(1UL << 29)

#define NRF54_USBD_GRSTCTL_CSFTRST		(1UL << 0)
#define NRF54_USBD_GRSTCTL_RXFFLSH		(1UL << 4)
#define NRF54_USBD_GRSTCTL_TXFFLSH		(1UL << 5)
#define NRF54_USBD_GRSTCTL_TXFNUM_Pos		6U
#define NRF54_USBD_GRSTCTL_AHBIDLE		(1UL << 31)
#define NRF54_USBD_GRSTCTL_TXFIFO_ALL		0x10UL

#define NRF54_USBD_GINTSTS_SOF			(1UL << 3)
#define NRF54_USBD_GINTSTS_GOUTNAKEFF		(1UL << 7)
#define NRF54_USBD_GINTSTS_USBSUSP		(1UL << 11)
#define NRF54_USBD_GINTSTS_USBRST		(1UL << 12)
#define NRF54_USBD_GINTSTS_ENUMDONE		(1UL << 13)
#define NRF54_USBD_GINTSTS_IEPINT		(1UL << 18)
#define NRF54_USBD_GINTSTS_OEPINT		(1UL << 19)
#define NRF54_USBD_GINTSTS_WKUPINT		(1UL << 31)

#define NRF54_USBD_GHWCFG2_ARCH_Pos		3U
#define NRF54_USBD_GHWCFG2_ARCH_Msk		(3UL << NRF54_USBD_GHWCFG2_ARCH_Pos)
#define NRF54_USBD_GHWCFG2_ARCH_INTDMA	(2UL << NRF54_USBD_GHWCFG2_ARCH_Pos)
#define NRF54_USBD_GHWCFG3_DFIFO_Pos		16U

#define NRF54_USBD_DCFG_DEVSPD_Msk		3UL
#define NRF54_USBD_DCFG_DEVSPD_HS		0UL
#define NRF54_USBD_DCFG_DEVADDR_Pos		4U
#define NRF54_USBD_DCFG_DEVADDR_Msk		(0x7FUL << NRF54_USBD_DCFG_DEVADDR_Pos)

#define NRF54_USBD_DCTL_RMTWKUPSIG		(1UL << 0)
#define NRF54_USBD_DCTL_SFTDISCON		(1UL << 1)
#define NRF54_USBD_DCTL_SGOUTNAK			(1UL << 9)
#define NRF54_USBD_DCTL_CGOUTNAK			(1UL << 10)

#define NRF54_USBD_DSTS_ENUMSPD_Pos		1U
#define NRF54_USBD_DSTS_ENUMSPD_Msk		(3UL << NRF54_USBD_DSTS_ENUMSPD_Pos)
#define NRF54_USBD_DSTS_ENUMSPD_HS		(0UL << NRF54_USBD_DSTS_ENUMSPD_Pos)
#define NRF54_USBD_DSTS_FNSOF_Pos		8U
#define NRF54_USBD_DSTS_FNSOF_Msk		(0x3FFFUL << NRF54_USBD_DSTS_FNSOF_Pos)

#define NRF54_USBD_DAINT_IN(ep)			(1UL << (ep))
#define NRF54_USBD_DAINT_OUT(ep)			(1UL << (16U + (ep)))

#define NRF54_USBD_DEPCTL_MPS_Msk		0x7FFUL
#define NRF54_USBD_DEPCTL_USBACTEP		(1UL << 15)
#define NRF54_USBD_DEPCTL_EPTYPE_Pos		18U
#define NRF54_USBD_DEPCTL_EPTYPE_Msk		(3UL << NRF54_USBD_DEPCTL_EPTYPE_Pos)
#define NRF54_USBD_DEPCTL_STALL			(1UL << 21)
#define NRF54_USBD_DIEPCTL_TXFNUM_Pos		22U
#define NRF54_USBD_DIEPCTL_TXFNUM_Msk		(0xFUL << NRF54_USBD_DIEPCTL_TXFNUM_Pos)
#define NRF54_USBD_DEPCTL_CNAK			(1UL << 26)
#define NRF54_USBD_DEPCTL_SNAK			(1UL << 27)
#define NRF54_USBD_DEPCTL_SETD0PID		(1UL << 28)
#define NRF54_USBD_DEPCTL_EPDIS			(1UL << 30)
#define NRF54_USBD_DEPCTL_EPENA			(1UL << 31)

#define NRF54_USBD_EP0_MPS_Msk			3UL
#define NRF54_USBD_EP0_MPS_64			0UL

#define NRF54_USBD_DIEPINT_XFRC			(1UL << 0)
#define NRF54_USBD_DIEPINT_EPDISBLD		(1UL << 1)
#define NRF54_USBD_DIEPINT_INEPNAKEFF		(1UL << 6)
#define NRF54_USBD_DOEPINT_XFRC			(1UL << 0)
#define NRF54_USBD_DOEPINT_EPDISBLD		(1UL << 1)
#define NRF54_USBD_DOEPINT_SETUP			(1UL << 3)

#define NRF54_USBD_DIEPTSIZ0_XFERSIZE_Msk	0x7FUL
#define NRF54_USBD_DIEPTSIZ0_PKTCNT_Pos	19U
#define NRF54_USBD_DOEPTSIZ0_XFERSIZE_Msk	0x7FUL
#define NRF54_USBD_DOEPTSIZ0_PKTCNT_Pos	19U
#define NRF54_USBD_DOEPTSIZ0_SUPCNT_Pos	29U

#define NRF54_USBD_DEPTSIZ_XFERSIZE_Msk	0x7FFFFUL
#define NRF54_USBD_DEPTSIZ_PKTCNT_Pos		19U

#define NRF54_USBD_DOEPMSK_XFRC			(1UL << 0)
#define NRF54_USBD_DOEPMSK_SETUP			(1UL << 3)
#define NRF54_USBD_DIEPMSK_XFRC			(1UL << 0)

typedef struct __nRF54_Usbd_Xfer
{
	uint8_t *pBuffer;
	uint16_t TotalLen;
	uint16_t ActualLen;
	uint16_t Mps;
	uint16_t ChunkLen;
	bool Started;
} nRF54UsbdXfer_t;

typedef struct __nRF54_Usbd_Ctrlr
{
	nRF54UsbdXfer_t Xfer[NRF54_USBD_EP_COUNT][2];
	UsbdCtrlrEvtHandler_t EvtHandler;
	void *pContext;
	uint16_t TxFifoWords[NRF54_USBD_EP_COUNT];
	uint16_t FifoTop;
	uint16_t RxWords;
	bool Started;
	bool HighSpeed;
	bool SofEnabled;
	bool AddressPending;
	uint8_t PendingAddress;
} nRF54UsbdCtrlr_t;

static nRF54UsbdCtrlr_t s_Ctrlr;

static uint32_t s_SetupBuffer[(NRF54_USBD_SETUP_COUNT * NRF54_USBD_SETUP_SIZE) /
								 sizeof(uint32_t)] __attribute__((aligned(4)));
static uint32_t s_Ep0Bounce[NRF54_USBD_EP0_MPS / sizeof(uint32_t)]
	__attribute__((aligned(4)));

static inline uint8_t nRF54UsbdDir(uint8_t EpAddr)
{
	return USBD_EP_IS_IN(EpAddr) ? 1U : 0U;
}

static inline nRF54UsbdXfer_t *nRF54UsbdGetXfer(uint8_t EpAddr)
{
	return &s_Ctrlr.Xfer[USBD_EP_NUM(EpAddr)][nRF54UsbdDir(EpAddr)];
}

static bool nRF54UsbdWaitSet(volatile uint32_t *pReg, uint32_t Mask)
{
	uint32_t count = NRF54_USBD_WAIT_LOOPS;

	while ((*pReg & Mask) == 0U)
	{
		if (--count == 0U)
		{
			return false;
		}
	}

	return true;
}

static bool nRF54UsbdWaitClear(volatile uint32_t *pReg, uint32_t Mask)
{
	uint32_t count = NRF54_USBD_WAIT_LOOPS;

	while ((*pReg & Mask) != 0U)
	{
		if (--count == 0U)
		{
			return false;
		}
	}

	return true;
}

static bool nRF54UsbdFlushTx(uint8_t FifoNo)
{
	NRF54_USBD_GRSTCTL = NRF54_USBD_GRSTCTL_TXFFLSH |
		((uint32_t)FifoNo << NRF54_USBD_GRSTCTL_TXFNUM_Pos);
	return nRF54UsbdWaitClear(&NRF54_USBD_GRSTCTL,
						  NRF54_USBD_GRSTCTL_TXFFLSH);
}

static bool nRF54UsbdFlushRx(void)
{
	NRF54_USBD_GRSTCTL = NRF54_USBD_GRSTCTL_RXFFLSH;
	return nRF54UsbdWaitClear(&NRF54_USBD_GRSTCTL,
						  NRF54_USBD_GRSTCTL_RXFFLSH);
}

static void nRF54UsbdEmit(const UsbdCtrlrEvt_t *pEvt)
{
	if (s_Ctrlr.EvtHandler != NULL)
	{
		s_Ctrlr.EvtHandler(pEvt, s_Ctrlr.pContext);
	}
}

static void nRF54UsbdEmitSimple(UsbdCtrlrEvtType_t Type)
{
	UsbdCtrlrEvt_t evt = { .Type = Type };
	nRF54UsbdEmit(&evt);
}

static void nRF54UsbdEmitXfer(uint8_t EpAddr, uint16_t Length,
							  UsbdCtrlrXferResult_t Result)
{
	UsbdCtrlrEvt_t evt = {
		.Type = USBD_CTRLR_EVT_XFER_CMPL,
		.Xfer = {
			.EpAddr = EpAddr,
			.Length = Length,
			.Result = Result,
		},
	};

	nRF54UsbdEmit(&evt);
}

static uint16_t nRF54UsbdRxWords(uint16_t Mps)
{
	uint16_t packetWords = (uint16_t)((Mps + 3U) / 4U);

	// DWC2 device FIFO recommendation for buffer DMA:
	// 13 setup/control words + global NAK + two largest OUT packets +
	// two status words for each OUT endpoint.
	return (uint16_t)(14U + (2U * (packetWords + 1U)) +
					  (2U * NRF54_USBD_EP_COUNT));
}

static bool nRF54UsbdCoreReset(void)
{
	if (!nRF54UsbdWaitSet(&NRF54_USBD_GRSTCTL,
						 NRF54_USBD_GRSTCTL_AHBIDLE))
	{
		return false;
	}

	NRF54_USBD_GRSTCTL |= NRF54_USBD_GRSTCTL_CSFTRST;

	return nRF54UsbdWaitClear(&NRF54_USBD_GRSTCTL,
						   NRF54_USBD_GRSTCTL_CSFTRST);
}

static void nRF54UsbdResetSoftware(void)
{
	UsbdCtrlrEvtHandler_t handler = s_Ctrlr.EvtHandler;
	void *context = s_Ctrlr.pContext;

	memset(&s_Ctrlr, 0, sizeof(s_Ctrlr));
	s_Ctrlr.EvtHandler = handler;
	s_Ctrlr.pContext = context;
	s_Ctrlr.Xfer[0][0].Mps = NRF54_USBD_EP0_MPS;
	s_Ctrlr.Xfer[0][1].Mps = NRF54_USBD_EP0_MPS;
}

static void nRF54UsbdPrimeSetup(void)
{
	// A DMA setup queue that is still enabled can already receive the next
	// SETUP packet. Do not rewrite DOEPTSIZ/DOEPDMA under an active endpoint.
	if ((NRF54_USBD_DOEPCTL(0) & NRF54_USBD_DEPCTL_EPENA) != 0U)
	{
		return;
	}

	memset(s_SetupBuffer, 0, sizeof(s_SetupBuffer));

	NRF54_USBD_DOEPTSIZ(0) =
		(NRF54_USBD_SETUP_COUNT << NRF54_USBD_DOEPTSIZ0_SUPCNT_Pos);
	NRF54_USBD_DOEPDMA(0) = (uint32_t)(uintptr_t)s_SetupBuffer;

	// Do not clear STALL here. A new SETUP automatically recovers EP0 from a
	// stalled control request, and the stall must remain asserted long enough
	// for the host to observe it.
	uint32_t ctl = NRF54_USBD_DOEPCTL(0);
	ctl &= ~NRF54_USBD_EP0_MPS_Msk;
	ctl |= NRF54_USBD_EP0_MPS_64 |
		   NRF54_USBD_DEPCTL_USBACTEP |
		   NRF54_USBD_DEPCTL_EPENA;
	NRF54_USBD_DOEPCTL(0) = ctl;
}

static void nRF54UsbdPrepareEp0(void)
{
	uint32_t inCtl = NRF54_USBD_DIEPCTL(0);
	inCtl &= ~(NRF54_USBD_EP0_MPS_Msk | NRF54_USBD_DEPCTL_STALL);
	inCtl |= NRF54_USBD_EP0_MPS_64 | NRF54_USBD_DEPCTL_USBACTEP;
	NRF54_USBD_DIEPCTL(0) = inCtl;

	uint32_t outCtl = NRF54_USBD_DOEPCTL(0);
	outCtl &= ~(NRF54_USBD_EP0_MPS_Msk | NRF54_USBD_DEPCTL_STALL);
	outCtl |= NRF54_USBD_EP0_MPS_64 | NRF54_USBD_DEPCTL_USBACTEP;
	NRF54_USBD_DOEPCTL(0) = outCtl;

	NRF54_USBD_DAINTMSK =
		NRF54_USBD_DAINT_IN(0) | NRF54_USBD_DAINT_OUT(0);
	NRF54_USBD_DIEPMSK = NRF54_USBD_DIEPMSK_XFRC;
	NRF54_USBD_DOEPMSK =
		NRF54_USBD_DOEPMSK_XFRC | NRF54_USBD_DOEPMSK_SETUP;

	nRF54UsbdPrimeSetup();
}

static bool nRF54UsbdAllocateTxFifo(uint8_t EpNum, uint16_t Mps)
{
	if (EpNum >= NRF54_USBD_EP_COUNT)
	{
		return false;
	}

	uint16_t words = (uint16_t)((Mps + 3U) / 4U);
	if (words == 0U)
	{
		words = 1U;
	}

	if (s_Ctrlr.TxFifoWords[EpNum] != 0U)
	{
		return s_Ctrlr.TxFifoWords[EpNum] >= words;
	}

	if (s_Ctrlr.FifoTop < (uint16_t)(s_Ctrlr.RxWords + words))
	{
		return false;
	}

	s_Ctrlr.FifoTop = (uint16_t)(s_Ctrlr.FifoTop - words);
	s_Ctrlr.TxFifoWords[EpNum] = words;

	uint32_t value = ((uint32_t)words << 16) | s_Ctrlr.FifoTop;
	if (EpNum == 0U)
	{
		NRF54_USBD_GNPTXFSIZ = value;
	}
	else
	{
		NRF54_USBD_DIEPTXF(EpNum) = value;
	}

	return true;
}

static bool nRF54UsbdGrowRxFifo(uint16_t Mps)
{
	uint16_t words = nRF54UsbdRxWords(Mps);

	if (words <= s_Ctrlr.RxWords)
	{
		return true;
	}

	if (words > s_Ctrlr.FifoTop)
	{
		return false;
	}

	s_Ctrlr.RxWords = words;
	NRF54_USBD_GRXFSIZ = words;
	return true;
}

static uint32_t nRF54UsbdEpType(uint8_t Type)
{
	switch (Type)
	{
		case USB_ENDPATT_TRANS_BULK:
			return 2UL;

		case USB_ENDPATT_TRANS_INT:
			return 3UL;

		default:
			return 0UL;
	}
}

static bool nRF54UsbdDisableEndpoint(uint8_t EpAddr, bool Stall)
{
	const uint8_t epNum = USBD_EP_NUM(EpAddr);
	if (epNum >= NRF54_USBD_EP_COUNT)
	{
		return false;
	}

	const uint32_t stall = Stall ? NRF54_USBD_DEPCTL_STALL : 0U;

	if (USBD_EP_IS_IN(EpAddr))
	{
		uint32_t ctl = NRF54_USBD_DIEPCTL(epNum);

		if ((ctl & NRF54_USBD_DEPCTL_EPENA) != 0U)
		{
			NRF54_USBD_DIEPCTL(epNum) = ctl | NRF54_USBD_DEPCTL_SNAK;
			if (!nRF54UsbdWaitSet(&NRF54_USBD_DIEPINT(epNum),
								 NRF54_USBD_DIEPINT_INEPNAKEFF))
			{
				return false;
			}

			NRF54_USBD_DIEPCTL(epNum) |=
				NRF54_USBD_DEPCTL_EPDIS | stall;
			if (!nRF54UsbdWaitSet(&NRF54_USBD_DIEPINT(epNum),
								 NRF54_USBD_DIEPINT_EPDISBLD))
			{
				return false;
			}

			NRF54_USBD_DIEPINT(epNum) =
				NRF54_USBD_DIEPINT_EPDISBLD |
				NRF54_USBD_DIEPINT_INEPNAKEFF;
		}
		else
		{
			NRF54_USBD_DIEPCTL(epNum) =
				ctl | NRF54_USBD_DEPCTL_SNAK | stall;
		}

		if (epNum != 0U && !nRF54UsbdFlushTx(epNum))
		{
			return false;
		}

		if (!Stall && epNum != 0U)
		{
			NRF54_USBD_DIEPCTL(epNum) &= ~NRF54_USBD_DEPCTL_USBACTEP;
		}

		return true;
	}

	uint32_t ctl = NRF54_USBD_DOEPCTL(epNum);
	if (epNum == 0U)
	{
		if (Stall)
		{
			NRF54_USBD_DOEPCTL(0) = ctl | NRF54_USBD_DEPCTL_STALL;
		}
		return true;
	}

	if ((ctl & NRF54_USBD_DEPCTL_EPENA) != 0U)
	{
		NRF54_USBD_DCTL |= NRF54_USBD_DCTL_SGOUTNAK;
		if (!nRF54UsbdWaitSet(&NRF54_USBD_GINTSTS,
							 NRF54_USBD_GINTSTS_GOUTNAKEFF))
		{
			NRF54_USBD_DCTL |= NRF54_USBD_DCTL_CGOUTNAK;
			return false;
		}

		NRF54_USBD_DOEPCTL(epNum) |= NRF54_USBD_DEPCTL_EPDIS | stall;
		if (!nRF54UsbdWaitSet(&NRF54_USBD_DOEPINT(epNum),
							 NRF54_USBD_DOEPINT_EPDISBLD))
		{
			NRF54_USBD_DCTL |= NRF54_USBD_DCTL_CGOUTNAK;
			return false;
		}

		NRF54_USBD_DOEPINT(epNum) = NRF54_USBD_DOEPINT_EPDISBLD;
		NRF54_USBD_DCTL |= NRF54_USBD_DCTL_CGOUTNAK;
	}
	else
	{
		NRF54_USBD_DOEPCTL(epNum) =
			ctl | NRF54_USBD_DEPCTL_SNAK | stall;
	}

	if (!Stall)
	{
		NRF54_USBD_DOEPCTL(epNum) &= ~NRF54_USBD_DEPCTL_USBACTEP;
	}

	return true;
}

static bool nRF54UsbdStartEp0Chunk(uint8_t EpAddr)
{
	nRF54UsbdXfer_t *pXfer = nRF54UsbdGetXfer(EpAddr);
	const uint16_t remaining =
		(uint16_t)(pXfer->TotalLen - pXfer->ActualLen);
	const uint16_t chunk = remaining < NRF54_USBD_EP0_MPS ?
		remaining : NRF54_USBD_EP0_MPS;

	pXfer->ChunkLen = chunk;

	if (USBD_EP_IS_IN(EpAddr))
	{
		if (chunk > 0U)
		{
			memcpy(s_Ep0Bounce, pXfer->pBuffer + pXfer->ActualLen, chunk);
		}

		NRF54_USBD_DIEPDMA(0) = (uint32_t)(uintptr_t)s_Ep0Bounce;
		NRF54_USBD_DIEPTSIZ(0) =
			chunk | (1UL << NRF54_USBD_DIEPTSIZ0_PKTCNT_Pos);
		NRF54_USBD_DIEPCTL(0) |=
			NRF54_USBD_DEPCTL_CNAK | NRF54_USBD_DEPCTL_EPENA;
	}
	else
	{
		NRF54_USBD_DOEPDMA(0) = (uint32_t)(uintptr_t)s_Ep0Bounce;
		NRF54_USBD_DOEPTSIZ(0) =
			chunk | (1UL << NRF54_USBD_DOEPTSIZ0_PKTCNT_Pos);
		NRF54_USBD_DOEPCTL(0) |=
			NRF54_USBD_DEPCTL_CNAK | NRF54_USBD_DEPCTL_EPENA;
	}

	return true;
}

static void nRF54UsbdCompleteEp0(uint8_t EpAddr)
{
	nRF54UsbdXfer_t *pXfer = nRF54UsbdGetXfer(EpAddr);
	if (!pXfer->Started)
	{
		return;
	}

	const uint16_t chunk = pXfer->ChunkLen;

	if (!USBD_EP_IS_IN(EpAddr) && chunk > 0U)
	{
		const uint16_t left =
			(uint16_t)(NRF54_USBD_DOEPTSIZ(0) &
						   NRF54_USBD_DOEPTSIZ0_XFERSIZE_Msk);
		const uint16_t received = left < chunk ?
			(uint16_t)(chunk - left) : 0U;

		if (received > 0U)
		{
			memcpy(pXfer->pBuffer + pXfer->ActualLen,
				   s_Ep0Bounce, received);
		}
		pXfer->ActualLen = (uint16_t)(pXfer->ActualLen + received);

		if (received < chunk)
		{
			pXfer->Started = false;
			nRF54UsbdEmitXfer(EpAddr, pXfer->ActualLen,
							 USBD_CTRLR_XFER_SUCCESS);
			return;
		}
	}
	else
	{
		pXfer->ActualLen = (uint16_t)(pXfer->ActualLen + chunk);
	}

	if (pXfer->ActualLen < pXfer->TotalLen)
	{
		(void)nRF54UsbdStartEp0Chunk(EpAddr);
		return;
	}

	const uint16_t total = pXfer->ActualLen;
	const bool zeroLength = pXfer->TotalLen == 0U;
	pXfer->Started = false;

	if (USBD_EP_IS_IN(EpAddr) && zeroLength && s_Ctrlr.AddressPending)
	{
		uint32_t dcfg = NRF54_USBD_DCFG;
		dcfg &= ~NRF54_USBD_DCFG_DEVADDR_Msk;
		dcfg |= (uint32_t)s_Ctrlr.PendingAddress <<
				NRF54_USBD_DCFG_DEVADDR_Pos;
		NRF54_USBD_DCFG = dcfg;
		s_Ctrlr.AddressPending = false;
	}

	if (zeroLength)
	{
		nRF54UsbdPrimeSetup();
	}

	nRF54UsbdEmitXfer(EpAddr, total, USBD_CTRLR_XFER_SUCCESS);
}

static void nRF54UsbdCompleteData(uint8_t EpAddr)
{
	nRF54UsbdXfer_t *pXfer = nRF54UsbdGetXfer(EpAddr);
	const uint8_t epNum = USBD_EP_NUM(EpAddr);

	if (!pXfer->Started)
	{
		return;
	}

	uint16_t remaining;
	if (USBD_EP_IS_IN(EpAddr))
	{
		remaining = (uint16_t)(NRF54_USBD_DIEPTSIZ(epNum) &
							  NRF54_USBD_DEPTSIZ_XFERSIZE_Msk);
	}
	else
	{
		remaining = (uint16_t)(NRF54_USBD_DOEPTSIZ(epNum) &
							  NRF54_USBD_DEPTSIZ_XFERSIZE_Msk);
	}

	const uint16_t actual = remaining <= pXfer->TotalLen ?
		(uint16_t)(pXfer->TotalLen - remaining) : 0U;

	pXfer->ActualLen = actual;
	pXfer->Started = false;
	nRF54UsbdEmitXfer(EpAddr, actual, USBD_CTRLR_XFER_SUCCESS);
}

static void nRF54UsbdBusReset(void)
{
	s_Ctrlr.HighSpeed = false;
	s_Ctrlr.AddressPending = false;
	s_Ctrlr.PendingAddress = 0U;

	uint32_t dcfg = NRF54_USBD_DCFG;
	dcfg &= ~NRF54_USBD_DCFG_DEVADDR_Msk;
	NRF54_USBD_DCFG = dcfg;

	for (uint8_t epNum = 0U; epNum < NRF54_USBD_EP_COUNT; epNum++)
	{
		s_Ctrlr.Xfer[epNum][0].Started = false;
		s_Ctrlr.Xfer[epNum][1].Started = false;
		s_Ctrlr.Xfer[epNum][0].ActualLen = 0U;
		s_Ctrlr.Xfer[epNum][1].ActualLen = 0U;

		NRF54_USBD_DIEPINT(epNum) = 0xFFFFFFFFUL;
		NRF54_USBD_DOEPINT(epNum) = 0xFFFFFFFFUL;

		NRF54_USBD_DOEPCTL(epNum) |= NRF54_USBD_DEPCTL_SNAK;

		if ((NRF54_USBD_DIEPCTL(epNum) & NRF54_USBD_DEPCTL_EPENA) != 0U)
		{
			NRF54_USBD_DIEPCTL(epNum) |=
				NRF54_USBD_DEPCTL_SNAK | NRF54_USBD_DEPCTL_EPDIS;
		}
	}

	(void)nRF54UsbdFlushTx((uint8_t)NRF54_USBD_GRSTCTL_TXFIFO_ALL);
	(void)nRF54UsbdFlushRx();

	nRF54UsbdPrepareEp0();
	nRF54UsbdEmitSimple(USBD_CTRLR_EVT_RESET);
}

static void nRF54UsbdSetupEvent(void)
{
	// A new SETUP cancels the previous EP0 transaction.
	s_Ctrlr.Xfer[0][0].Started = false;
	s_Ctrlr.Xfer[0][1].Started = false;
	s_Ctrlr.AddressPending = false;

	uintptr_t setupAddr =
		(uintptr_t)NRF54_USBD_DOEPDMA(0) - NRF54_USBD_SETUP_SIZE;
	const uintptr_t first = (uintptr_t)s_SetupBuffer;
	const uintptr_t last = first + sizeof(s_SetupBuffer) -
						   NRF54_USBD_SETUP_SIZE;

	if (setupAddr < first || setupAddr > last)
	{
		setupAddr = first;
	}

	UsbdCtrlrEvt_t evt = { .Type = USBD_CTRLR_EVT_SETUP };
	memcpy(&evt.Setup, (const void *)setupAddr, sizeof(evt.Setup));
	nRF54UsbdEmit(&evt);
}

static void nRF54UsbdInInterrupt(void)
{
	const uint32_t pending = NRF54_USBD_DAINT &
							 NRF54_USBD_DAINTMSK & 0xFFFFUL;

	for (uint8_t epNum = 0U; epNum < NRF54_USBD_EP_COUNT; epNum++)
	{
		if ((pending & NRF54_USBD_DAINT_IN(epNum)) == 0U)
		{
			continue;
		}

		const uint32_t epInt = NRF54_USBD_DIEPINT(epNum) &
							   NRF54_USBD_DIEPMSK;
		NRF54_USBD_DIEPINT(epNum) = epInt;

		if ((epInt & NRF54_USBD_DIEPINT_XFRC) != 0U)
		{
			if (epNum == 0U)
			{
				nRF54UsbdCompleteEp0(USBD_EP_DIR_IN);
			}
			else
			{
				nRF54UsbdCompleteData((uint8_t)(epNum | USBD_EP_DIR_IN));
			}
		}
	}
}

static void nRF54UsbdOutInterrupt(void)
{
	const uint32_t pending = (NRF54_USBD_DAINT &
							  NRF54_USBD_DAINTMSK) >> 16;

	for (uint8_t epNum = 0U; epNum < NRF54_USBD_EP_COUNT; epNum++)
	{
		if ((pending & (1UL << epNum)) == 0U)
		{
			continue;
		}

		const uint32_t epInt = NRF54_USBD_DOEPINT(epNum) &
							   NRF54_USBD_DOEPMSK;
		NRF54_USBD_DOEPINT(epNum) = epInt;

		if (epNum == 0U && (epInt & NRF54_USBD_DOEPINT_SETUP) != 0U)
		{
			nRF54UsbdSetupEvent();
			continue;
		}

		if ((epInt & NRF54_USBD_DOEPINT_XFRC) != 0U)
		{
			if (epNum == 0U)
			{
				if (s_Ctrlr.Xfer[0][0].Started)
				{
					nRF54UsbdCompleteEp0(USBD_EP_DIR_OUT);
				}
			}
			else
			{
				nRF54UsbdCompleteData(epNum);
			}
		}
	}
}

bool UsbdCtrlrInit(UsbdCtrlrEvtHandler_t EvtHandler, void *pContext)
{
	memset(&s_Ctrlr, 0, sizeof(s_Ctrlr));
	s_Ctrlr.EvtHandler = EvtHandler;
	s_Ctrlr.pContext = pContext;
	s_Ctrlr.Xfer[0][0].Mps = NRF54_USBD_EP0_MPS;
	s_Ctrlr.Xfer[0][1].Mps = NRF54_USBD_EP0_MPS;
	return true;
}

bool UsbdCtrlrStart(void)
{
	if ((NRF54_USBD_GHWCFG2 & NRF54_USBD_GHWCFG2_ARCH_Msk) !=
		NRF54_USBD_GHWCFG2_ARCH_INTDMA)
	{
		return false;
	}

	NRF54_USBD_DCTL |= NRF54_USBD_DCTL_SFTDISCON;

	if (!nRF54UsbdCoreReset())
	{
		return false;
	}

	nRF54UsbdResetSoftware();

	uint32_t gusbcfg = NRF54_USBD_GUSBCFG;
	gusbcfg &= ~NRF54_USBD_GUSBCFG_FORCEHSTMODE;
	gusbcfg |= NRF54_USBD_GUSBCFG_FORCEDEVMODE;
	NRF54_USBD_GUSBCFG = gusbcfg;

	const uint16_t dfifoDepth =
		(uint16_t)(NRF54_USBD_GHWCFG3 >> NRF54_USBD_GHWCFG3_DFIFO_Pos);
	const uint16_t epInfoWords = 2U * NRF54_USBD_EP_COUNT;
	const uint32_t totalFifo = (uint32_t)dfifoDepth + epInfoWords;

	if (totalFifo > 0xFFFFUL || dfifoDepth <= epInfoWords)
	{
		return false;
	}

	// Buffer DMA reserves one endpoint-info word per endpoint direction. The
	// nRF54 GHWCFG3 depth is the 3040-word data FIFO; GDFIFOCFG then places the
	// endpoint-info controller immediately above it in the 3072-word RAM.
	s_Ctrlr.FifoTop = dfifoDepth;
	NRF54_USBD_GDFIFOCFG =
		((uint32_t)dfifoDepth << 16) | dfifoDepth;

	s_Ctrlr.RxWords = nRF54UsbdRxWords(NRF54_USBD_EP0_MPS);
	NRF54_USBD_GRXFSIZ = s_Ctrlr.RxWords;

	if (!nRF54UsbdAllocateTxFifo(0U, NRF54_USBD_EP0_MPS))
	{
		return false;
	}

	uint32_t gahbcfg = NRF54_USBD_GAHBCFG;
	gahbcfg &= ~(0xFUL << 1);
	gahbcfg |= NRF54_USBD_GAHBCFG_HBSTLEN_INCR4 |
			   NRF54_USBD_GAHBCFG_DMAEN;
	gahbcfg &= ~NRF54_USBD_GAHBCFG_GINT;
	NRF54_USBD_GAHBCFG = gahbcfg;

	uint32_t dcfg = NRF54_USBD_DCFG;
	dcfg &= ~(NRF54_USBD_DCFG_DEVSPD_Msk |
			  NRF54_USBD_DCFG_DEVADDR_Msk);
	dcfg |= NRF54_USBD_DCFG_DEVSPD_HS;
	NRF54_USBD_DCFG = dcfg;

	NRF54_USBD_GINTMSK = 0U;
	NRF54_USBD_GINTSTS = 0xFFFFFFFFUL;
	NRF54_USBD_DAINTMSK = 0U;
	NRF54_USBD_DIEPMSK = 0U;
	NRF54_USBD_DOEPMSK = 0U;

	for (uint8_t epNum = 0U; epNum < NRF54_USBD_EP_COUNT; epNum++)
	{
		NRF54_USBD_DIEPINT(epNum) = 0xFFFFFFFFUL;
		NRF54_USBD_DOEPINT(epNum) = 0xFFFFFFFFUL;
	}

	nRF54UsbdPrepareEp0();

	NRF54_USBD_GINTMSK =
		NRF54_USBD_GINTSTS_USBRST |
		NRF54_USBD_GINTSTS_ENUMDONE |
		NRF54_USBD_GINTSTS_IEPINT |
		NRF54_USBD_GINTSTS_OEPINT |
		NRF54_USBD_GINTSTS_USBSUSP |
		NRF54_USBD_GINTSTS_WKUPINT;

	s_Ctrlr.Started = true;
	return true;
}

void UsbdCtrlrStop(void)
{
	if (!s_Ctrlr.Started)
	{
		return;
	}

	NRF54_USBD_GAHBCFG &= ~NRF54_USBD_GAHBCFG_GINT;
	NRF54_USBD_GINTMSK = 0U;
	NRF54_USBD_DCTL |= NRF54_USBD_DCTL_SFTDISCON;

	for (uint8_t epNum = 0U; epNum < NRF54_USBD_EP_COUNT; epNum++)
	{
		s_Ctrlr.Xfer[epNum][0].Started = false;
		s_Ctrlr.Xfer[epNum][1].Started = false;
	}

	s_Ctrlr.Started = false;
	s_Ctrlr.HighSpeed = false;
	s_Ctrlr.AddressPending = false;
}

bool UsbdCtrlrHighSpeed(void)
{
	return s_Ctrlr.Started && s_Ctrlr.HighSpeed;
}

void UsbdCtrlrIntEnable(void)
{
	NRF54_USBD_GAHBCFG |= NRF54_USBD_GAHBCFG_GINT;
	NVIC_ClearPendingIRQ(USBHS_IRQn);
	NVIC_EnableIRQ(USBHS_IRQn);
}

void UsbdCtrlrIntDisable(void)
{
	NVIC_DisableIRQ(USBHS_IRQn);
	NRF54_USBD_GAHBCFG &= ~NRF54_USBD_GAHBCFG_GINT;
}

void UsbdCtrlrConnect(void)
{
	NRF54_USBD_DCTL &= ~NRF54_USBD_DCTL_SFTDISCON;
}

void UsbdCtrlrDisconnect(void)
{
	NRF54_USBD_DCTL |= NRF54_USBD_DCTL_SFTDISCON;
}

void UsbdCtrlrRemoteWakeup(void)
{
	NRF54_USBD_DCTL |= NRF54_USBD_DCTL_RMTWKUPSIG;
	nrfx_coredep_delay_us(2000U);
	NRF54_USBD_DCTL &= ~NRF54_USBD_DCTL_RMTWKUPSIG;
}

void UsbdCtrlrSofEnable(bool Enable)
{
	s_Ctrlr.SofEnabled = Enable;

	if (Enable)
	{
		NRF54_USBD_GINTSTS = NRF54_USBD_GINTSTS_SOF;
		NRF54_USBD_GINTMSK |= NRF54_USBD_GINTSTS_SOF;
	}
	else
	{
		NRF54_USBD_GINTMSK &= ~NRF54_USBD_GINTSTS_SOF;
	}
}

void UsbdCtrlrSetAddress(uint8_t Address)
{
	s_Ctrlr.PendingAddress = (uint8_t)(Address & 0x7FU);
	s_Ctrlr.AddressPending = true;
}

bool UsbdCtrlrEpOpen(const UsbEndPointDesc_t *pDesc)
{
	if (!s_Ctrlr.Started || pDesc == NULL)
	{
		return false;
	}

	const uint8_t epAddr = pDesc->bEndpointAddress;
	const uint8_t epNum = USBD_EP_NUM(epAddr);
	const uint8_t type = pDesc->bmAttributes & 0x03U;
	const uint16_t mps = pDesc->wMaxPacketSize;

	if (epNum == 0U || epNum >= NRF54_USBD_EP_COUNT ||
		(type != USB_ENDPATT_TRANS_BULK &&
		 type != USB_ENDPATT_TRANS_INT) ||
		mps == 0U)
	{
		return false;
	}

	const uint16_t maxMps = type == USB_ENDPATT_TRANS_BULK ?
		(s_Ctrlr.HighSpeed ? 512U : 64U) :
		(s_Ctrlr.HighSpeed ? 1024U : 64U);
	if (mps > maxMps)
	{
		return false;
	}

	nRF54UsbdXfer_t *pXfer = nRF54UsbdGetXfer(epAddr);
	pXfer->Mps = mps;
	pXfer->Started = false;
	pXfer->ActualLen = 0U;

	const uint32_t epType =
		nRF54UsbdEpType(type) << NRF54_USBD_DEPCTL_EPTYPE_Pos;

	if (USBD_EP_IS_IN(epAddr))
	{
		if (!nRF54UsbdAllocateTxFifo(epNum, mps))
		{
			return false;
		}

		uint32_t ctl = NRF54_USBD_DIEPCTL(epNum);
		ctl &= ~(NRF54_USBD_DEPCTL_MPS_Msk |
				 NRF54_USBD_DEPCTL_EPTYPE_Msk |
				 NRF54_USBD_DIEPCTL_TXFNUM_Msk |
				 NRF54_USBD_DEPCTL_STALL);
		ctl |= mps |
			   epType |
			   NRF54_USBD_DEPCTL_USBACTEP |
			   NRF54_USBD_DEPCTL_SETD0PID |
			   ((uint32_t)epNum << NRF54_USBD_DIEPCTL_TXFNUM_Pos);
		NRF54_USBD_DIEPCTL(epNum) = ctl;
		NRF54_USBD_DAINTMSK |= NRF54_USBD_DAINT_IN(epNum);
	}
	else
	{
		if (!nRF54UsbdGrowRxFifo(mps))
		{
			return false;
		}

		uint32_t ctl = NRF54_USBD_DOEPCTL(epNum);
		ctl &= ~(NRF54_USBD_DEPCTL_MPS_Msk |
				 NRF54_USBD_DEPCTL_EPTYPE_Msk |
				 NRF54_USBD_DEPCTL_STALL);
		ctl |= mps |
			   epType |
			   NRF54_USBD_DEPCTL_USBACTEP |
			   NRF54_USBD_DEPCTL_SETD0PID;
		NRF54_USBD_DOEPCTL(epNum) = ctl;
		NRF54_USBD_DAINTMSK |= NRF54_USBD_DAINT_OUT(epNum);
	}

	return true;
}

void UsbdCtrlrEpClose(uint8_t EpAddr)
{
	const uint8_t epNum = USBD_EP_NUM(EpAddr);
	if (epNum == 0U || epNum >= NRF54_USBD_EP_COUNT)
	{
		return;
	}

	(void)nRF54UsbdDisableEndpoint(EpAddr, false);

	nRF54UsbdXfer_t *pXfer = nRF54UsbdGetXfer(EpAddr);
	pXfer->Started = false;
	pXfer->ActualLen = 0U;
	pXfer->TotalLen = 0U;
	pXfer->ChunkLen = 0U;
	pXfer->pBuffer = NULL;

	if (USBD_EP_IS_IN(EpAddr))
	{
		NRF54_USBD_DAINTMSK &= ~NRF54_USBD_DAINT_IN(epNum);
	}
	else
	{
		NRF54_USBD_DAINTMSK &= ~NRF54_USBD_DAINT_OUT(epNum);
	}
}

void UsbdCtrlrEpCloseAll(void)
{
	for (uint8_t epNum = 1U; epNum < NRF54_USBD_EP_COUNT; epNum++)
	{
		UsbdCtrlrEpClose(epNum);
		UsbdCtrlrEpClose((uint8_t)(epNum | USBD_EP_DIR_IN));
	}

	(void)nRF54UsbdFlushRx();
}

bool UsbdCtrlrEpXfer(uint8_t EpAddr, uint8_t *pBuffer, uint16_t TotalBytes)
{
	const uint8_t epNum = USBD_EP_NUM(EpAddr);
	if (!s_Ctrlr.Started || epNum >= NRF54_USBD_EP_COUNT ||
		(TotalBytes > 0U && pBuffer == NULL))
	{
		return false;
	}

	nRF54UsbdXfer_t *pXfer = nRF54UsbdGetXfer(EpAddr);
	if (pXfer->Started || pXfer->Mps == 0U)
	{
		return false;
	}

	if (epNum != 0U && TotalBytes > 0U &&
		(((uintptr_t)pBuffer & 0x3U) != 0U))
	{
		// nRF54LM20 has instruction cache only. Buffer DMA therefore needs no
		// data-cache maintenance, but the DMA address must remain word aligned.
		return false;
	}

	pXfer->pBuffer = pBuffer;
	pXfer->TotalLen = TotalBytes;
	pXfer->ActualLen = 0U;
	pXfer->ChunkLen = 0U;
	pXfer->Started = true;

	if (epNum == 0U)
	{
		return nRF54UsbdStartEp0Chunk(EpAddr);
	}

	const uint32_t packetCnt = TotalBytes == 0U ? 1U :
		((uint32_t)TotalBytes + pXfer->Mps - 1U) / pXfer->Mps;

	if (packetCnt > 0x3FFUL)
	{
		pXfer->Started = false;
		return false;
	}

	const uint32_t size =
		((uint32_t)TotalBytes & NRF54_USBD_DEPTSIZ_XFERSIZE_Msk) |
		(packetCnt << NRF54_USBD_DEPTSIZ_PKTCNT_Pos);

	if (USBD_EP_IS_IN(EpAddr))
	{
		NRF54_USBD_DIEPDMA(epNum) = (uint32_t)(uintptr_t)pBuffer;
		NRF54_USBD_DIEPTSIZ(epNum) = size;
		NRF54_USBD_DIEPCTL(epNum) |=
			NRF54_USBD_DEPCTL_CNAK | NRF54_USBD_DEPCTL_EPENA;
	}
	else
	{
		NRF54_USBD_DOEPDMA(epNum) = (uint32_t)(uintptr_t)pBuffer;
		NRF54_USBD_DOEPTSIZ(epNum) = size;
		NRF54_USBD_DOEPCTL(epNum) |=
			NRF54_USBD_DEPCTL_CNAK | NRF54_USBD_DEPCTL_EPENA;
	}

	return true;
}

bool UsbdCtrlrEpBusy(uint8_t EpAddr)
{
	const uint8_t epNum = USBD_EP_NUM(EpAddr);
	return epNum < NRF54_USBD_EP_COUNT &&
		   nRF54UsbdGetXfer(EpAddr)->Started;
}

void UsbdCtrlrEpStall(uint8_t EpAddr)
{
	const uint8_t epNum = USBD_EP_NUM(EpAddr);
	if (epNum >= NRF54_USBD_EP_COUNT)
	{
		return;
	}

	(void)nRF54UsbdDisableEndpoint(EpAddr, true);

	// EP0 OUT remains active for SETUP reception. If the setup queue was
	// exhausted by the failed request, re-arm it without clearing STALL.
	if (epNum == 0U && !USBD_EP_IS_IN(EpAddr))
	{
		nRF54UsbdPrimeSetup();
	}
}

void UsbdCtrlrEpClearStall(uint8_t EpAddr)
{
	const uint8_t epNum = USBD_EP_NUM(EpAddr);
	if (epNum >= NRF54_USBD_EP_COUNT)
	{
		return;
	}

	if (USBD_EP_IS_IN(EpAddr))
	{
		uint32_t ctl = NRF54_USBD_DIEPCTL(epNum);
		ctl &= ~NRF54_USBD_DEPCTL_STALL;
		if (epNum != 0U)
		{
			ctl |= NRF54_USBD_DEPCTL_SETD0PID;
		}
		NRF54_USBD_DIEPCTL(epNum) = ctl;
	}
	else
	{
		uint32_t ctl = NRF54_USBD_DOEPCTL(epNum);
		ctl &= ~NRF54_USBD_DEPCTL_STALL;
		if (epNum != 0U)
		{
			ctl |= NRF54_USBD_DEPCTL_SETD0PID;
		}
		NRF54_USBD_DOEPCTL(epNum) = ctl;
	}
}

void USBHS_IRQHandler(void)
{
	const uint32_t status = NRF54_USBD_GINTSTS & NRF54_USBD_GINTMSK;
	if (status == 0U)
	{
		return;
	}

	if ((status & NRF54_USBD_GINTSTS_USBRST) != 0U)
	{
		NRF54_USBD_GINTSTS = NRF54_USBD_GINTSTS_USBRST;
		nRF54UsbdBusReset();
	}

	if ((status & NRF54_USBD_GINTSTS_ENUMDONE) != 0U)
	{
		NRF54_USBD_GINTSTS = NRF54_USBD_GINTSTS_ENUMDONE;
		s_Ctrlr.HighSpeed =
			(NRF54_USBD_DSTS & NRF54_USBD_DSTS_ENUMSPD_Msk) ==
			NRF54_USBD_DSTS_ENUMSPD_HS;
	}

	if ((status & NRF54_USBD_GINTSTS_OEPINT) != 0U)
	{
		nRF54UsbdOutInterrupt();
	}

	if ((status & NRF54_USBD_GINTSTS_IEPINT) != 0U)
	{
		nRF54UsbdInInterrupt();
	}

	if ((status & NRF54_USBD_GINTSTS_USBSUSP) != 0U)
	{
		NRF54_USBD_GINTSTS = NRF54_USBD_GINTSTS_USBSUSP;
		nRF54UsbdEmitSimple(USBD_CTRLR_EVT_SUSPEND);
	}

	if ((status & NRF54_USBD_GINTSTS_WKUPINT) != 0U)
	{
		NRF54_USBD_GINTSTS = NRF54_USBD_GINTSTS_WKUPINT;
		nRF54UsbdEmitSimple(USBD_CTRLR_EVT_RESUME);
	}

	if ((status & NRF54_USBD_GINTSTS_SOF) != 0U)
	{
		NRF54_USBD_GINTSTS = NRF54_USBD_GINTSTS_SOF;

		if (s_Ctrlr.SofEnabled)
		{
			UsbdCtrlrEvt_t evt = {
				.Type = USBD_CTRLR_EVT_SOF,
				.FrameNo = (uint16_t)((NRF54_USBD_DSTS &
									  NRF54_USBD_DSTS_FNSOF_Msk) >>
									  NRF54_USBD_DSTS_FNSOF_Pos),
			};
			nRF54UsbdEmit(&evt);
		}
	}
}
