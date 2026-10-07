/**-------------------------------------------------------------------------
@file	usb_ctrlr_nrf54.cpp

@brief	USB device controller for Nordic nRF54LM20x USBHS.

Direct register implementation for the high-speed USBHS peripheral. The port
owns VREGUSB, HFCLK24M, the USBHS core and its endpoint DMA/FIFO configuration.

DevNo selects the controller. Every nRF part has exactly one, USB_CTRLR_CNT is
1, so the entry points validate DevNo and the state stays a singleton. Arraying
it is work for the first part that has two.

@author	Hoang Nguyen Hoan
@date	Sep. 3, 2026

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
#include <stdint.h>
#include <string.h>

#include "nrf.h"
#include "nrf_peripherals.h"
#include "hal/nrf_ficr.h"
// Bus power, clock and VBUS.
// The USBHS clock and VBUS regulator are separate peripherals on this part.
#include "hal/nrf_clock.h"
#include "hal/nrf_vregusb.h"
#include "lib/nrfx_coredep.h"

#include "istddef.h"
#include "app_evt_handler.h"
#include "coredev/interrupt.h"
#include "usb/usb.h"

#ifndef NRF54_USB_TRACE
#ifdef NDEBUG
#define NRF54_USB_TRACE	0
#else
#define NRF54_USB_TRACE	1
#endif
#endif

#if NRF54_USB_TRACE
#include <stdio.h>

// Capture in the ISR; formatting and rdimon output run in UsbCtrlrProcess.
// Keep the oldest records and report overflow instead of blocking USB.
#define NRF54_USB_TRACE_COUNT	64U
typedef struct __nRF54_Usb_Trace {
	const char *pFormat;
	uint32_t Arg[4];
} nRF54UsbTrace_t;

static nRF54UsbTrace_t s_UsbTrace[NRF54_USB_TRACE_COUNT];
static volatile uint32_t s_UsbTraceWrite;
static volatile uint32_t s_UsbTraceRead;
static volatile uint32_t s_UsbTraceLost;

static void nRF54UsbTrace(const char *pFormat, uint32_t A = 0U,
	uint32_t B = 0U, uint32_t C = 0U, uint32_t D = 0U)
{
	const uint32_t state = DisableInterrupt();
	if (s_UsbTraceWrite - s_UsbTraceRead < NRF54_USB_TRACE_COUNT)
	{
		s_UsbTrace[s_UsbTraceWrite % NRF54_USB_TRACE_COUNT] = {pFormat, {A, B, C, D}};
		s_UsbTraceWrite = s_UsbTraceWrite + 1U;
	}
	else
		s_UsbTraceLost = s_UsbTraceLost + 1U;
	EnableInterrupt(state);
	UsbProcessQue(0);
}

static void nRF54UsbTraceFlush(void)
{
	for (unsigned i = 0U; i < NRF54_USB_TRACE_COUNT; ++i)
	{
		const uint32_t state = DisableInterrupt();
		if (s_UsbTraceRead == s_UsbTraceWrite)
		{
			const uint32_t lost = s_UsbTraceLost;
			s_UsbTraceLost = 0U;
			EnableInterrupt(state);
			if (lost != 0U)
				printf("USB TRACE lost=%lu\n", (unsigned long)lost);
			fflush(stdout);
			return;
		}
		const nRF54UsbTrace_t record = s_UsbTrace[s_UsbTraceRead % NRF54_USB_TRACE_COUNT];
		s_UsbTraceRead = s_UsbTraceRead + 1U;
		EnableInterrupt(state);
		printf(record.pFormat, (unsigned long)record.Arg[0], (unsigned long)record.Arg[1],
			(unsigned long)record.Arg[2], (unsigned long)record.Arg[3]);
	}
	// A busy trace cannot monopolize one application event indefinitely.
	fflush(stdout);
	UsbProcessQue(0);
}
#define USB_TRACE(...)	nRF54UsbTrace(__VA_ARGS__)
#else
#define USB_TRACE(...)	((void)0)
#endif


#ifndef NRFX_USBD_XTAL_WAIT_LOOPS
#define NRFX_USBD_XTAL_WAIT_LOOPS			2000000UL
#endif

#ifndef NRFX_USBD_READY_WAIT_LOOPS
#define NRFX_USBD_READY_WAIT_LOOPS			2000000UL
#endif


// Undocumented VREGUSB status register. The events below report the edges and
// are in the MDK, but the level is not, and something has to answer what the
// state is at start up before any edge has happened. Offset and bit are as
// used by Zephyr's regulator_nrf_vregusb driver, which says in its own source
// that the register is not part of NRF_VREGUSB_Type.
#define NRFX_USBD_VREGUSB_STATUS_OFS		0x400UL
#define NRFX_USBD_VREGUSB_STATUS_VBUSDET	(1UL << 2)


#ifndef USBHSCORE_PRESENT
#error "usb_ctrlr_nrf54: this part has no USBHS core"
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
#define NRF54_USBD_GHWCFG4				NRF54_USBD_REG(0x050U)
#define NRF54_USBD_GDFIFOCFG			NRF54_USBD_REG(0x05CU)
#define NRF54_USBD_PCGCCTL				NRF54_USBD_REG(0xE00U)

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
#define NRF54_USBD_GAHBCFG_HBSTLEN_INCR4	(3UL << 1)
#define NRF54_USBD_GAHBCFG_DMAEN			(1UL << 5)

#define NRF54_USBD_GUSBCFG_FORCEDEVMODE	(1UL << 30)
#define NRF54_USBD_GUSBCFG_FORCEHSTMODE	(1UL << 29)

#define NRF54_USBD_GRSTCTL_CSFTRST		(1UL << 0)
#define NRF54_USBD_GRSTCTL_CSFTRSTDONE	(1UL << 29)
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
#define NRF54_USBD_GINTSTS_ISOINCOMP		(1UL << 20)
#define NRF54_USBD_GINTSTS_ISOOUTCOMP	(1UL << 21)
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
#define NRF54_USBD_DEPCTL_SETODDFR		(1UL << 29)
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
#define NRF54_USBD_DOEPINT_OUTPKTERR		(1UL << 8)
#define NRF54_USBD_DEPINT_AHBERR			(1UL << 2)
#define NRF54_USBD_DOEPINT_STSPHSERCVD	(1UL << 5)
#define NRF54_USBD_DOEPINT_STUPPKTRCVD	(1UL << 15)
#define NRF54_USBD_PCGCCTL_STOPPCLK		(1UL << 0)

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

enum
{
	NRF_USB_EP_COUNT = USB_EPIN_CNT(0) > USB_EPOUT_CNT(0) ?
		USB_EPIN_CNT(0) : USB_EPOUT_CNT(0),
};

typedef struct __nRF_Usb_Ep_Registration
{
	UsbCtrlrEpHandler_t Handler;
	void *pContext;
	uint32_t Generation;
} nRFUsbEpReg_t;

enum nRF54UsbdXferState : uint8_t
{
	NRF54_XFER_IDLE,
	NRF54_XFER_ACTIVE,
	NRF54_XFER_DISABLING,
};

typedef struct __nRF54_Usbd_Xfer
{
	uint8_t *pBuffer;
	uint32_t Scratch;
	uint16_t TotalLen;
	uint16_t ActualLen;
	uint16_t Mps;
	uint16_t ChunkLen;
	nRF54UsbdXferState State;
	UsbCtrlrEvtType_t Result;
	uint8_t Type;
	bool Open;
} nRF54UsbdXfer_t;

typedef struct __nRF54_Usbd_Ctrlr
{
	nRF54UsbdXfer_t Xfer[NRF54_USBD_EP_COUNT][2];
	uint16_t TxFifoWords[NRF54_USBD_EP_COUNT];
	uint16_t FifoTop;
	uint16_t FifoDepth;
	uint16_t RxWords;
	bool Started;
	bool Suspended;
	bool HighSpeed;
	bool SofEnabled;
} nRF54UsbdCtrlr_t;

// Fixed data-endpoint ownership lives outside the active-transfer state so a
// bus reset can cancel transfers without losing registrations.
static nRFUsbEpReg_t s_EpReg[NRF_USB_EP_COUNT][2];

// One USB controller per part, so the common power/clock state is file scope.
// USBHS_IRQHandler in this file owns the controller interrupt.
static uint8_t s_UsbdIntPrio;
static bool s_UsbdLowPowerSuspend;
static bool s_UsbdInitialized = false;
static bool s_UsbdStarted = false;
static bool s_UsbdXtalHeld = false;
static bool s_UsbdRestart;
static bool s_UsbdDetached;


// The nRF54 reports VBUS edges, so retain the resulting level here.
static bool s_UsbdVbusLevel = false;

static nRF54UsbdCtrlr_t s_Ctrlr;

// OUT also receives up to three SETUP packets. Leave room after a full
// data packet while software retires that DMA and primes the next stage.
static uint32_t s_Ep0Bounce[(NRF54_USBD_EP0_MPS +
	NRF54_USBD_SETUP_COUNT * NRF54_USBD_SETUP_SIZE) / sizeof(uint32_t)]
	__attribute__((aligned(4)));
static uint32_t s_Ep0In[NRF54_USBD_EP0_MPS / sizeof(uint32_t)]
	__attribute__((aligned(4)));
static UsbSetupData_t s_Setup;


static bool nRFUsbRegEpXfer(uint8_t EpAddr, uint8_t *pBuffer, uint16_t TotalBytes);
static void nRF54UsbdResetSoftware(void);
static void nRF54UsbdEmitSimple(UsbCtrlrEvtType_t Type);

/// Only DevNo 0 exists on every nRF part shipped so far.
static inline __attribute__((always_inline))
bool nRFUsbValidDevNo(int DevNo)
{
	return DevNo == 0;
}

static inline __attribute__((always_inline))
uint8_t nRFUsbEpDir(uint8_t EpAddr)
{
	return USB_ENDPADDR_IS_IN(EpAddr) ? 1U : 0U;
}

static inline __attribute__((always_inline))
nRFUsbEpReg_t *nRFUsbGetEpReg(uint8_t EpAddr)
{
	return &s_EpReg[USB_ENDPADDR_NUM(EpAddr)][nRFUsbEpDir(EpAddr)];
}

static inline __attribute__((always_inline))
void nRFUsbEpRegisteredEvent(uint8_t EpAddr, UsbCtrlrEvtType_t Event,
								 uint16_t Length)
{
	nRFUsbEpReg_t *pReg = nRFUsbGetEpReg(EpAddr);
	if (pReg->Handler != nullptr)
		pReg->Handler(Event, Length, pReg->pContext);
}

__attribute__((weak)) bool UsbdXtalRequest(void)
{
	// The USBHS PHY runs from the 24 MHz clock, not from the radio crystal.
	// This is a separate domain with its own task and its own started event.
	if (nrf_clock_is_running(NRF_CLOCK, NRF_CLOCK_DOMAIN_HFCLK24M, NULL))
	{
		return true;
	}

	nrf_clock_event_clear(NRF_CLOCK, NRF_CLOCK_EVENT_HFCLK24MSTARTED);
	nrf_clock_task_trigger(NRF_CLOCK, NRF_CLOCK_TASK_HFCLK24MSTART);

	for (uint32_t i = 0; i < NRFX_USBD_XTAL_WAIT_LOOPS; i++)
	{
		if (nrf_clock_event_check(NRF_CLOCK, NRF_CLOCK_EVENT_HFCLK24MSTARTED))
		{
			nrf_clock_event_clear(NRF_CLOCK, NRF_CLOCK_EVENT_HFCLK24MSTARTED);
			return true;
		}
	}

	return false;
}

__attribute__((weak)) void UsbdXtalRelease(void)
{
	// Left running. Other peripherals take this clock without
	// counting, so stopping one here would stop it under them.
}


// A lost PHY clock or failed DMA shutdown must not leave a caller-owned
// buffer accessible to hardware. Reset the wrapper without reading the core;
// the existing USB process event restores the bus session afterwards.
static void nRFUsbAbortCore(void)
{
	USB_TRACE("USB ABORT started=%lu\n", s_Ctrlr.Started);
	s_UsbdRestart = true;
	s_UsbdDetached = true;
	NVIC_DisableIRQ(USBHS_IRQn);
	NRF_USBHS->ENABLE = 0U;
	nRF54UsbdResetSoftware();
	s_UsbdStarted = false;
	UsbProcessQue(0);
}

/**
 * Consume the VBUS edges the regulator has recorded and answer the level they
 * leave behind. Called from the VREGUSB interrupt and from the process event.
 * The events hold until they are cleared, so nothing is lost between calls.
 */
static bool UsbdVbusPoll(void)
{
	const bool detected = nrf_vregusb_event_check(NRF_VREGUSB, NRF_VREGUSB_EVENT_VBUS_DETECTED);
	const bool removed = nrf_vregusb_event_check(NRF_VREGUSB, NRF_VREGUSB_EVENT_VBUS_REMOVED);
	if (detected)
		nrf_vregusb_event_clear(NRF_VREGUSB, NRF_VREGUSB_EVENT_VBUS_DETECTED);
	if (removed)
		nrf_vregusb_event_clear(NRF_VREGUSB, NRF_VREGUSB_EVENT_VBUS_REMOVED);
	if (detected || removed)
	{
		// Both edge bits can accumulate during a quick unplug/replug. Their
		// order is unknown; read the level after acknowledging them.
		s_UsbdVbusLevel = (*(volatile uint32_t *)((uintptr_t)NRF_VREGUSB +
			NRFX_USBD_VREGUSB_STATUS_OFS) & NRFX_USBD_VREGUSB_STATUS_VBUSDET) != 0U;
		USB_TRACE("USB VBUS detected=%lu removed=%lu level=%lu\n",
			detected, removed, s_UsbdVbusLevel);
		if (removed && s_Ctrlr.Started)
			nRFUsbAbortCore();
	}
	return s_UsbdVbusLevel;
}

/**
 * Power the USBHS wrapper, its PHY and the core, in the order the part
 * requires. The two delays are not optional: a core register read before the
 * PHY clock is up can hang the bus. This sequence completes before
 * nRFUsbRegStart() touches NRF_USBHSCORE.
 *
 * The sequence follows Nordic's USBHS integration requirements, matching the
 * ordering used by Zephyr's Nordic USBHS vendor quirk implementation.
 */
static bool UsbdStartCtrlr(void)
{
	// Core first, PHY still held in reset
	NRF_USBHS->ENABLE = USBHS_ENABLE_CORE_Msk;

	// Device role, and hold VBUSVALID low until the core can enumerate
	NRF_USBHS->PHY.OVERRIDEVALUES =
		USBHS_PHY_OVERRIDEVALUES_ID_Device << USBHS_PHY_OVERRIDEVALUES_ID_Pos;
	NRF_USBHS->PHY.INPUTOVERRIDE =
		USBHS_PHY_INPUTOVERRIDE_ID_Msk | USBHS_PHY_INPUTOVERRIDE_VBUSVALID_Msk;

	// Keep the PHY tuning installed by the Nordic SystemInit/FICR trims.

	// Release the PHY power on reset
	NRF_USBHS->ENABLE = USBHS_ENABLE_PHY_Msk | USBHS_ENABLE_CORE_Msk;

	// PHY clock start
	nrfx_coredep_delay_us(45);

	NRF_USBHS->TASKS_START = USBHS_TASKS_START_TASKS_START_Trigger;

	// Settle before anything reads a core register
	nrfx_coredep_delay_us(2);

	// Keep the pull-up suppressed until UsbCtrlrConnect.

	// The wrapper and the core are separate peripheral blocks at separate
	// addresses. Without this the writes above can still be sitting in the
	// write buffer when the core is first read.
	__DSB();

	return true;
}

static void UsbdStopCtrlr(void)
{
	NVIC_DisableIRQ(USBHS_IRQn);

	NRF_USBHS->PHY.INPUTOVERRIDE = USBHS_PHY_INPUTOVERRIDE_ID_Msk |
		USBHS_PHY_INPUTOVERRIDE_VBUSVALID_Msk | USBHS_PHY_INPUTOVERRIDE_SUSPENDM0_Msk;
	NRF_USBHS->ENABLE = 0;
	nrfx_coredep_delay_us(3U);
	__ISB();
	__DSB();
}


static bool nRFUsbVbusDetected(void)
{
	return UsbdVbusPoll();
}



static size_t nRFUsbSerial(char *pBuff, size_t BuffLen)
{
	static const char hex[] = "0123456789ABCDEF";
	size_t cnt = 0;

	if (pBuff == nullptr || BuffLen == 0)
	{
		return 0;
	}

	// nrf_ficr_deviceid_get and not FICR->DEVICEID, because the nRF52 keeps
	// the id flat and the nRF54 keeps it under INFO, and the HAL already knows
	// which.
	for (int i = 0; i < 2; i++)
	{
		uint32_t id = nrf_ficr_deviceid_get(NRF_FICR, (uint32_t)i);

		for (int n = 7; n >= 0; n--)
		{
			if (cnt + 1 >= BuffLen)
			{
				pBuff[cnt] = '\0';
				return cnt;
			}

			pBuff[cnt++] = hex[(id >> (n * 4)) & 0x0F];
		}
	}

	pBuff[cnt] = '\0';

	return cnt;
}

static bool nRFUsbPowerInit(const UsbCtrlrCfg_t *pCfg)
{
	if (pCfg == nullptr)
	{
		return false;
	}

	s_UsbdIntPrio = pCfg->IntPrio;
	s_UsbdLowPowerSuspend = pCfg->bLowPowerSuspend;

	// Start the regulator here so VBUS state is available before the native
	// controller is enabled. The wrapper/core power sequence happens later in
	// UsbdStart(), before nRFUsbRegStart() reads the DesignWare registers.
	nrf_vregusb_event_clear(NRF_VREGUSB, NRF_VREGUSB_EVENT_VBUS_DETECTED);
	nrf_vregusb_event_clear(NRF_VREGUSB, NRF_VREGUSB_EVENT_VBUS_REMOVED);
	nrf_vregusb_task_trigger(NRF_VREGUSB, NRF_VREGUSB_TASK_START);

	s_UsbdVbusLevel =
		(*(volatile uint32_t *)((uintptr_t)NRF_VREGUSB +
								NRFX_USBD_VREGUSB_STATUS_OFS) &
		 NRFX_USBD_VREGUSB_STATUS_VBUSDET) != 0;

	// The cable edges are an interrupt of the regulator, owned by this port.
	// The handler queues the process event, which reports the edge and
	// connects or disconnects.
	nrf_vregusb_int_enable(NRF_VREGUSB, NRF_VREGUSB_INT_VBUS_DETECTED_MASK |
										NRF_VREGUSB_INT_VBUS_REMOVED_MASK);
	NVIC_ClearPendingIRQ(VREGUSB_IRQn);
	NVIC_SetPriority(VREGUSB_IRQn, s_UsbdIntPrio);
	NVIC_EnableIRQ(VREGUSB_IRQn);

	s_UsbdRestart = s_UsbdDetached = false;
	s_UsbdInitialized = true;
	s_UsbdStarted = false;
	(void)nRFUsbVbusDetected();

	return true;
}

// Cable attach and removal. Records the level and queues the process event.
extern "C" void VREGUSB_IRQHandler(void)
{
	(void)UsbdVbusPoll();
	UsbProcessQue(0);
}

static bool nRFUsbPowerStart(void)
{
	if (s_UsbdInitialized == false)
	{
		return false;
	}

	if (s_UsbdStarted)
	{
		return true;
	}

	if (nRFUsbVbusDetected() == false)
	{
		// No cable. Not a failure: the cable interrupt queues the attach
		// and the caller comes back.
		return false;
	}

	if (UsbdXtalRequest() == false)
	{
		return false;
	}

	s_UsbdXtalHeld = true;

	NVIC_SetPriority(USBHS_IRQn, s_UsbdIntPrio);

	if (UsbdStartCtrlr() == false)
	{
		UsbdXtalRelease();
		s_UsbdXtalHeld = false;
		return false;
	}

	if (!nRFUsbVbusDetected())
	{
		UsbdStopCtrlr();
		UsbdXtalRelease();
		s_UsbdXtalHeld = false;
		return false;
	}
	s_UsbdStarted = true;

	return true;
}

static void nRFUsbPowerStop(void)
{
	if (s_UsbdStarted == false)
	{
		if (s_UsbdXtalHeld)
		{
			UsbdXtalRelease();
			s_UsbdXtalHeld = false;
		}
		return;
	}

	// Stop the controller interrupt before powering down the wrapper.
	NVIC_DisableIRQ(USBHS_IRQn);

	UsbdStopCtrlr();

	if (s_UsbdXtalHeld)
	{
		UsbdXtalRelease();
		s_UsbdXtalHeld = false;
	}

	s_UsbdStarted = false;
}

/**
 * Called from the process event through UsbCtrlrProcess. It must cost
 * nothing when there is nothing to do: a pending low power exit to retire, or
 * a VBUS edge. Neither waits.
 */
static void nRFUsbPowerProcess(void)
{
	if (!s_UsbdInitialized)
		return;
	const uint32_t state = DisableInterrupt();
	const bool vbus = nRFUsbVbusDetected();
	if (s_UsbdDetached)
	{
		s_UsbdDetached = false;
		if (s_UsbdXtalHeld)
		{
			UsbdXtalRelease();
			s_UsbdXtalHeld = false;
		}
		nRF54UsbdEmitSimple(USB_CTRLR_EVT_RESET);
	}
	EnableInterrupt(state);
	if (s_UsbdRestart && vbus && UsbCtrlrStart(0))
	{
		s_UsbdRestart = false;
		UsbCtrlrIntEnable(0);
		UsbCtrlrConnect(0);
	}
}

//
// Endpoint and DMA registers. Exactly one of these compiles.
//


static inline __attribute__((always_inline))
uint8_t nRF54UsbdDir(uint8_t EpAddr)
{
	return USB_ENDPADDR_IS_IN(EpAddr) ? 1U : 0U;
}

static inline __attribute__((always_inline))
nRF54UsbdXfer_t *nRF54UsbdGetXfer(uint8_t EpAddr)
{
	return &s_Ctrlr.Xfer[USB_ENDPADDR_NUM(EpAddr)][nRF54UsbdDir(EpAddr)];
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

static inline __attribute__((always_inline))
void nRF54UsbdEmit(const UsbCtrlrEvt_t *pEvt)
{
	UsbDevProcessEvent(0, pEvt);
}

static void nRF54UsbdEmitSimple(UsbCtrlrEvtType_t Type)
{
	UsbCtrlrEvt_t evt = { .Type = Type };
	nRF54UsbdEmit(&evt);
}

static void nRF54UsbdEmitXfer(uint8_t EpAddr, uint16_t Length)
{
	if (USB_ENDPADDR_NUM(EpAddr) != 0U)
	{
		nRFUsbEpRegisteredEvent(EpAddr, USB_CTRLR_EVT_XFER_CMPL, Length);
		return;
	}

	UsbCtrlrEvt_t evt = {
		.Type = USB_CTRLR_EVT_XFER_CMPL,
		.Xfer = {
			.EpAddr = EpAddr,
			.Length = Length,
			.Result = USB_CTRLR_XFER_SUCCESS,
			.pBuffer = (const uint8_t *)s_Ep0Bounce,
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
	if (!nRF54UsbdWaitSet(&NRF54_USBD_GRSTCTL, NRF54_USBD_GRSTCTL_AHBIDLE))
		return false;
	NRF54_USBD_GRSTCTL = NRF54_USBD_GRSTCTL_CSFTRST;
	for (uint32_t i = 0; i < NRF54_USBD_WAIT_LOOPS; ++i)
	{
		const uint32_t ctl = NRF54_USBD_GRSTCTL;
		// Newer DWC2 revisions signal DONE instead of clearing CSFTRST.
		if ((ctl & NRF54_USBD_GRSTCTL_CSFTRST) == 0U ||
			(ctl & NRF54_USBD_GRSTCTL_CSFTRSTDONE) != 0U)
		{
			NRF54_USBD_GRSTCTL = ctl & ~NRF54_USBD_GRSTCTL_CSFTRST;
			return true;
		}
	}
	return false;
}

static void nRF54UsbdResetSoftware(void)
{
	for (auto &ep : s_EpReg)
		for (auto &reg : ep)
			++reg.Generation;
	memset(&s_Ctrlr, 0, sizeof(s_Ctrlr));
	s_Ctrlr.Xfer[0][0].Mps = NRF54_USBD_EP0_MPS;
	s_Ctrlr.Xfer[0][1].Mps = NRF54_USBD_EP0_MPS;
}

static void nRF54UsbdPrimeSetup(void)
{
	if ((NRF54_USBD_DOEPCTL(0) & NRF54_USBD_DEPCTL_EPENA) != 0U)
		return;
	NRF54_USBD_DOEPTSIZ(0) =
		NRF54_USBD_SETUP_COUNT << NRF54_USBD_DOEPTSIZ0_SUPCNT_Pos;
	NRF54_USBD_DOEPDMA(0) = (uint32_t)(uintptr_t)s_Ep0Bounce;
	// SETUP-only reception must not clear NAK in buffer DMA mode.
	NRF54_USBD_DOEPCTL(0) |= NRF54_USBD_DEPCTL_USBACTEP | NRF54_USBD_DEPCTL_EPENA;
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
	NRF54_USBD_DIEPMSK = NRF54_USBD_DIEPMSK_XFRC |
		NRF54_USBD_DIEPINT_EPDISBLD | NRF54_USBD_DIEPINT_INEPNAKEFF |
		NRF54_USBD_DEPINT_AHBERR;
	NRF54_USBD_DOEPMSK =
		NRF54_USBD_DOEPMSK_XFRC | NRF54_USBD_DOEPMSK_SETUP |
		NRF54_USBD_DOEPINT_STSPHSERCVD | NRF54_USBD_DOEPINT_EPDISBLD |
		NRF54_USBD_DOEPINT_OUTPKTERR |
		NRF54_USBD_DEPINT_AHBERR;

	nRF54UsbdPrimeSetup();
}

static bool nRF54UsbdAllocateTxFifo(uint8_t EpNum, uint16_t Mps)
{
	const uint16_t words = Mps > 64U ? (uint16_t)((Mps + 3U) / 4U) : 16U;
	if (s_Ctrlr.TxFifoWords[EpNum] >= words)
		return true;

	// Allocate downwards into a free extent. Closing an alternate setting
	// frees its extent without relocating another endpoint's active FIFO.
	uint16_t top = s_Ctrlr.FifoDepth;
	for (unsigned pass = 0; pass < NRF54_USBD_EP_COUNT; ++pass)
	{
		if (top < s_Ctrlr.RxWords + words)
			return false;
		const uint16_t start = top - words;
		uint16_t next = top;
		for (unsigned ep = 0; ep < NRF54_USBD_EP_COUNT; ++ep)
		{
			if (ep == EpNum || s_Ctrlr.TxFifoWords[ep] == 0U)
				continue;
			const uint16_t base = (uint16_t)(ep == 0U ?
				NRF54_USBD_GNPTXFSIZ : NRF54_USBD_DIEPTXF(ep));
			if (base < top && base + s_Ctrlr.TxFifoWords[ep] > start && base < next)
				next = base;
		}
		if (next != top)
		{
			top = next;
			continue;
		}
		s_Ctrlr.TxFifoWords[EpNum] = words;
		const uint32_t value = ((uint32_t)words << 16) | start;
		if (EpNum == 0U)
			NRF54_USBD_GNPTXFSIZ = value;
		else
			NRF54_USBD_DIEPTXF(EpNum) = value;
		if (start < s_Ctrlr.FifoTop)
			s_Ctrlr.FifoTop = start;
		return true;
	}
	return false;
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
		case USB_ENDPATT_TRANS_ISO:
			return 1UL;

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
	const uint8_t epNum = USB_ENDPADDR_NUM(EpAddr);
	if (epNum >= NRF54_USBD_EP_COUNT)
	{
		return false;
	}

	NRF54_USBD_PCGCCTL &= ~NRF54_USBD_PCGCCTL_STOPPCLK;
	const uint32_t stall = Stall ? NRF54_USBD_DEPCTL_STALL : 0U;

	if (USB_ENDPADDR_IS_IN(EpAddr))
	{
		uint32_t ctl = NRF54_USBD_DIEPCTL(epNum);

		if ((ctl & NRF54_USBD_DEPCTL_EPENA) != 0U)
		{
			NRF54_USBD_DIEPINT(epNum) = NRF54_USBD_DIEPINT_EPDISBLD |
				NRF54_USBD_DIEPINT_INEPNAKEFF;
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

		NRF54_USBD_DOEPINT(epNum) = NRF54_USBD_DOEPINT_EPDISBLD;
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
	const uint16_t remaining = pXfer->TotalLen - pXfer->ActualLen;
	const uint16_t chunk = remaining < NRF54_USBD_EP0_MPS ? remaining : NRF54_USBD_EP0_MPS;
	pXfer->ChunkLen = chunk;
	if (USB_ENDPADDR_IS_IN(EpAddr))
	{
		if (chunk != 0U)
			memcpy(s_Ep0In, pXfer->pBuffer, chunk);
		pXfer->pBuffer = nullptr;
		nRF54UsbdPrimeSetup();
		NRF54_USBD_DIEPDMA(0) = (uint32_t)(uintptr_t)s_Ep0In;
		NRF54_USBD_DIEPTSIZ(0) = chunk | (1UL << NRF54_USBD_DIEPTSIZ0_PKTCNT_Pos);
		__DSB();
		NRF54_USBD_DIEPCTL(0) |= NRF54_USBD_DEPCTL_CNAK | NRF54_USBD_DEPCTL_EPENA;
	}
	else
	{
		// Even a short requested OUT stage must have room for one full packet.
		// The core validates the actual length before copying to class storage.
		pXfer->ChunkLen = chunk == 0U ? 0U : NRF54_USBD_EP0_MPS;
		NRF54_USBD_DOEPDMA(0) = (uint32_t)(uintptr_t)s_Ep0Bounce;
		NRF54_USBD_DOEPTSIZ(0) = pXfer->ChunkLen |
			(1UL << NRF54_USBD_DOEPTSIZ0_PKTCNT_Pos) |
			(NRF54_USBD_SETUP_COUNT << NRF54_USBD_DOEPTSIZ0_SUPCNT_Pos);
		__DSB();
		NRF54_USBD_DOEPCTL(0) |= NRF54_USBD_DEPCTL_CNAK | NRF54_USBD_DEPCTL_EPENA;
	}
	USB_TRACE("USB EP0 ARM ep=%02lx len=%lu ctl=%08lx size=%08lx\n",
		EpAddr, pXfer->ChunkLen,
		USB_ENDPADDR_IS_IN(EpAddr) ? NRF54_USBD_DIEPCTL(0) : NRF54_USBD_DOEPCTL(0),
		USB_ENDPADDR_IS_IN(EpAddr) ? NRF54_USBD_DIEPTSIZ(0) : NRF54_USBD_DOEPTSIZ(0));
	return true;
}

static void nRF54UsbdCompleteEp0(uint8_t EpAddr)
{
	nRF54UsbdXfer_t *pXfer = nRF54UsbdGetXfer(EpAddr);
	USB_TRACE("USB EP0 DONE ep=%02lx state=%lu total=%lu dcfg=%08lx\n",
		EpAddr, pXfer->State, pXfer->TotalLen, NRF54_USBD_DCFG);
	if (pXfer->State != NRF54_XFER_ACTIVE)
		return;
	uint16_t amount = pXfer->ChunkLen;
	if (!USB_ENDPADDR_IS_IN(EpAddr))
	{
		const uint16_t left = NRF54_USBD_DOEPTSIZ(0) & NRF54_USBD_DOEPTSIZ0_XFERSIZE_Msk;
		amount = left <= amount ? amount - left : 0U;
	}
	pXfer->ActualLen += amount;
	const bool more = !USB_ENDPADDR_IS_IN(EpAddr) && amount == NRF54_USBD_EP0_MPS &&
		pXfer->ActualLen < pXfer->TotalLen;
	pXfer->State = more ? NRF54_XFER_ACTIVE : NRF54_XFER_IDLE;
	// The callback may stall, submit the next IN packet, or start STATUS.
	nRF54UsbdEmitXfer(EpAddr, amount);
	if (more && pXfer->State == NRF54_XFER_ACTIVE &&
		(NRF54_USBD_DOEPCTL(0) & NRF54_USBD_DEPCTL_STALL) == 0U)
		(void)nRF54UsbdStartEp0Chunk(EpAddr);
	else if (pXfer->State == NRF54_XFER_IDLE)
		nRF54UsbdPrimeSetup();
	USB_TRACE("USB EP0 NEXT out_ctl=%08lx out_size=%08lx out_dma=%08lx out_irq=%08lx\n",
		NRF54_USBD_DOEPCTL(0), NRF54_USBD_DOEPTSIZ(0),
		NRF54_USBD_DOEPDMA(0), NRF54_USBD_DOEPINT(0));
}

// Endpoint completions run in the controller interrupt, as on nRF52. The
// endpoint is idle before the owner is called, so the owner can start its
// next transfer from the callback. A regular OUT owner then gets DRDY to
// supply its next buffer, unless the completion callback already did, or
// closed or rebound the endpoint. ISO OUT is armed by the SOF callback.
static void nRF54UsbdCompleteData(uint8_t EpAddr,
	UsbCtrlrEvtType_t Result = USB_CTRLR_EVT_XFER_CMPL)
{
	nRF54UsbdXfer_t *pXfer = nRF54UsbdGetXfer(EpAddr);
	const uint8_t epNum = USB_ENDPADDR_NUM(EpAddr);
	if (pXfer->State != NRF54_XFER_ACTIVE && pXfer->State != NRF54_XFER_DISABLING)
		return;
	const uint32_t remaining = (USB_ENDPADDR_IS_IN(EpAddr) ?
		NRF54_USBD_DIEPTSIZ(epNum) : NRF54_USBD_DOEPTSIZ(epNum)) &
		NRF54_USBD_DEPTSIZ_XFERSIZE_Msk;
	pXfer->ActualLen = Result == USB_CTRLR_EVT_XFER_CMPL && remaining <= pXfer->TotalLen ?
		pXfer->TotalLen - remaining : 0U;
	pXfer->pBuffer = nullptr;
	pXfer->Result = Result;
	pXfer->State = NRF54_XFER_IDLE;
	__DSB();
	nRFUsbEpReg_t *pReg = nRFUsbGetEpReg(EpAddr);
	const uint32_t generation = pReg->Generation;
	nRFUsbEpRegisteredEvent(EpAddr, Result, pXfer->ActualLen);
	if (!USB_ENDPADDR_IS_IN(EpAddr) && pXfer->Type != USB_ENDPATT_TRANS_ISO &&
		pReg->Generation == generation && pXfer->Open &&
		pXfer->State == NRF54_XFER_IDLE)
		nRFUsbEpRegisteredEvent(EpAddr, USB_CTRLR_EVT_DRDY, 0U);
}

static void nRF54UsbdBusReset(void)
{
	USB_TRACE("USB RESET dcfg=%08lx dsts=%08lx\n", NRF54_USBD_DCFG, NRF54_USBD_DSTS);
	// USB reset has stopped DMA. Invalidate software before the core closes
	// interfaces and releases their CFifo storage.
	const uint16_t depth = s_Ctrlr.FifoDepth;
	nRF54UsbdResetSoftware();
	s_Ctrlr.Started = true;
	s_Ctrlr.FifoDepth = s_Ctrlr.FifoTop = depth;
	NRF54_USBD_PCGCCTL &= ~NRF54_USBD_PCGCCTL_STOPPCLK;
	NRF54_USBD_DCFG &= ~NRF54_USBD_DCFG_DEVADDR_Msk;
	NRF54_USBD_GINTMSK &= ~NRF54_USBD_GINTSTS_SOF;
	for (unsigned ep = 0; ep < NRF54_USBD_EP_COUNT; ++ep)
	{
		NRF54_USBD_DIEPCTL(ep) = NRF54_USBD_DEPCTL_SNAK;
		NRF54_USBD_DOEPCTL(ep) = NRF54_USBD_DEPCTL_SNAK;
		NRF54_USBD_DIEPINT(ep) = 0xFFFFFFFFUL;
		NRF54_USBD_DOEPINT(ep) = 0xFFFFFFFFUL;
	}
	(void)nRF54UsbdFlushTx(NRF54_USBD_GRSTCTL_TXFIFO_ALL);
	(void)nRF54UsbdFlushRx();
	s_Ctrlr.RxWords = nRF54UsbdRxWords(NRF54_USBD_EP0_MPS);
	NRF54_USBD_GRXFSIZ = s_Ctrlr.RxWords;
	(void)nRF54UsbdAllocateTxFifo(0U, NRF54_USBD_EP0_MPS);
	nRF54UsbdPrepareEp0();
	nRF54UsbdEmitSimple(USB_CTRLR_EVT_RESET);
}

static void nRF54UsbdSetupEvent(void)
{
	USB_TRACE("USB SETUP type_req=%04lx value=%04lx index=%04lx len=%lu\n",
		((uint32_t)s_Setup.bmRequestType << 8) | s_Setup.bRequest,
		s_Setup.wValue, s_Setup.wIndex, s_Setup.wLength);
	USB_TRACE("USB EP0 PRESETUP in=%08lx state=%lu out_ctl=%08lx dcfg=%08lx\n",
		NRF54_USBD_DIEPINT(0), s_Ctrlr.Xfer[0][1].State,
		NRF54_USBD_DOEPCTL(0), NRF54_USBD_DCFG);
	// The OUT ISR snapshots SETUP at DMA completion, before SETUP-phase-done.
	// Quiesce the previous IN transfer before its bounce buffer is reused.
	if (!nRF54UsbdDisableEndpoint(USB_ENDPADDR_DIR_IN, false) || !nRF54UsbdFlushTx(0U))
	{
		nRFUsbAbortCore();
		return;
	}
	NRF54_USBD_DIEPINT(0) = 0xFFFFFFFFUL;
	s_Ctrlr.Xfer[0][0].State = NRF54_XFER_IDLE;
	s_Ctrlr.Xfer[0][1].State = NRF54_XFER_IDLE;
	NRF54_USBD_DIEPCTL(0) &= ~NRF54_USBD_DEPCTL_STALL;
	NRF54_USBD_DOEPCTL(0) &= ~NRF54_USBD_DEPCTL_STALL;
	UsbCtrlrEvt_t evt = { .Type = USB_CTRLR_EVT_SETUP, .Setup = s_Setup };
	nRF54UsbdEmit(&evt);
	if ((evt.Setup.bmRequestType & USB_REQTYPE_MASK_DIR) == 0U &&
		evt.Setup.wLength != 0U &&
		(NRF54_USBD_DOEPCTL(0) & NRF54_USBD_DEPCTL_STALL) == 0U)
		(void)nRFUsbRegEpXfer(0U, nullptr, evt.Setup.wLength);
	else
		nRF54UsbdPrimeSetup();
}

static void nRF54UsbdInInterrupt(void)
{
	const uint32_t pending = NRF54_USBD_DAINT & NRF54_USBD_DAINTMSK & 0xFFFFUL;
	for (uint8_t ep = 0U; ep < NRF54_USBD_EP_COUNT; ++ep)
	{
		if ((pending & NRF54_USBD_DAINT_IN(ep)) == 0U)
			continue;
		const uint32_t status = NRF54_USBD_DIEPINT(ep) & NRF54_USBD_DIEPMSK;
		NRF54_USBD_DIEPINT(ep) = status;
		if (ep == 0U)
			USB_TRACE("USB EP0 IN status=%08lx ctl=%08lx size=%08lx\n",
				status, NRF54_USBD_DIEPCTL(0), NRF54_USBD_DIEPTSIZ(0));
		const uint8_t addr = USB_ENDPADDR_DIRIN(ep);
		nRF54UsbdXfer_t *pXfer = &s_Ctrlr.Xfer[ep][1];
		if ((status & NRF54_USBD_DIEPINT_EPDISBLD) != 0U &&
			pXfer->State == NRF54_XFER_DISABLING)
		{
			(void)nRF54UsbdFlushTx(ep);
			nRF54UsbdCompleteData(addr, USB_CTRLR_EVT_XFER_FAILED);
		}
		else if ((status & NRF54_USBD_DIEPINT_INEPNAKEFF) != 0U &&
			pXfer->State == NRF54_XFER_DISABLING)
			NRF54_USBD_DIEPCTL(ep) |= NRF54_USBD_DEPCTL_EPDIS;
		else if ((status & NRF54_USBD_DEPINT_AHBERR) != 0U)
		{
			if (ep == 0U)
				UsbCtrlrEpStall(0, 0U, false);
			else
			{
				if (!nRF54UsbdDisableEndpoint(addr, true))
				{
					nRFUsbAbortCore();
					return;
				}
				nRF54UsbdCompleteData(addr, USB_CTRLR_EVT_XFER_FAILED);
			}
		}
		else if ((status & NRF54_USBD_DIEPINT_XFRC) != 0U)
		{
			if (ep == 0U)
				nRF54UsbdCompleteEp0(addr);
			else
				nRF54UsbdCompleteData(addr);
		}
		if (!s_Ctrlr.Started)
			return;
	}
}

static void nRF54UsbdOutInterrupt(void)
{
	const uint32_t pending = (NRF54_USBD_DAINT & NRF54_USBD_DAINTMSK) >> 16;
	for (uint8_t ep = 0U; ep < NRF54_USBD_EP_COUNT; ++ep)
	{
		if ((pending & (1UL << ep)) == 0U)
			continue;
		const uint32_t raw = NRF54_USBD_DOEPINT(ep);
		uint32_t status = raw & NRF54_USBD_DOEPMSK;
		NRF54_USBD_DOEPINT(ep) = status;
		if (ep == 0U)
		{
			USB_TRACE("USB EP0 OUT raw=%08lx status=%08lx dma=%08lx\n",
				raw, status, NRF54_USBD_DOEPDMA(0));
			if ((raw & NRF54_USBD_DOEPINT_STUPPKTRCVD) != 0U &&
				(status & NRF54_USBD_DOEPINT_XFRC) != 0U)
			{
				NRF54_USBD_DOEPINT(0) = NRF54_USBD_DOEPINT_STUPPKTRCVD;
				const uintptr_t addr = (uintptr_t)NRF54_USBD_DOEPDMA(0) - sizeof(s_Setup);
				const uintptr_t base = (uintptr_t)s_Ep0Bounce;
				if (addr < base || addr > base + sizeof(s_Ep0Bounce) - sizeof(s_Setup))
				{
					USB_TRACE("USB SETUP bad DMA addr=%08lx base=%08lx\n",
						(uint32_t)addr, (uint32_t)base);
					UsbCtrlrEpStall(0, 0U, false);
					continue;
				}
				__DSB();
				memcpy(&s_Setup, (const void *)addr, sizeof(s_Setup));
				status &= ~NRF54_USBD_DOEPINT_XFRC;
			}
			if ((status & NRF54_USBD_DOEPINT_STSPHSERCVD) != 0U)
				NRF54_USBD_DIEPCTL(0) |= NRF54_USBD_DEPCTL_CNAK;
			if ((status & NRF54_USBD_DOEPINT_SETUP) != 0U)
			{
				nRF54UsbdSetupEvent();
				if (!s_Ctrlr.Started)
					return;
				continue;
			}
		}
		if ((raw & NRF54_USBD_DOEPINT_OUTPKTERR) != 0U)
			s_Ctrlr.Xfer[ep][0].Result = USB_CTRLR_EVT_XFER_FAILED;
		if ((status & NRF54_USBD_DOEPINT_EPDISBLD) != 0U &&
			s_Ctrlr.Xfer[ep][0].State == NRF54_XFER_DISABLING)
			nRF54UsbdCompleteData(ep, USB_CTRLR_EVT_XFER_FAILED);
		else if ((status & NRF54_USBD_DEPINT_AHBERR) != 0U)
		{
			if (ep == 0U)
				UsbCtrlrEpStall(0, 0U, false);
			else
			{
				if (!nRF54UsbdDisableEndpoint(ep, true))
				{
					nRFUsbAbortCore();
					return;
				}
				nRF54UsbdCompleteData(ep, USB_CTRLR_EVT_XFER_FAILED);
			}
		}
		else if ((status & NRF54_USBD_DOEPINT_XFRC) != 0U)
		{
			if (ep == 0U)
				nRF54UsbdCompleteEp0(0U);
			else
				nRF54UsbdCompleteData(ep, s_Ctrlr.Xfer[ep][0].Result);
		}
		if (!s_Ctrlr.Started)
			return;
	}
}

// ISO misses retire asynchronously; no wait for a host token in the ISR.
static void nRF54UsbdIsoIncomplete(bool In)
{
	const uint32_t parity = (NRF54_USBD_DSTS >> NRF54_USBD_DSTS_FNSOF_Pos) & 1U;
	for (uint8_t ep = 1; ep < NRF54_USBD_EP_COUNT; ++ep)
	{
		nRF54UsbdXfer_t *pXfer = &s_Ctrlr.Xfer[ep][In];
		if (!pXfer->Open || pXfer->Type != USB_ENDPATT_TRANS_ISO ||
			pXfer->State != NRF54_XFER_ACTIVE)
			continue;
		volatile uint32_t *pCtl = In ? &NRF54_USBD_DIEPCTL(ep) : &NRF54_USBD_DOEPCTL(ep);
		if ((*pCtl & NRF54_USBD_DEPCTL_EPENA) == 0U ||
			(!In && ((*pCtl >> 16U) & 1U) != parity))
			continue;
		pXfer->State = NRF54_XFER_DISABLING;
		if (In)
			*pCtl |= NRF54_USBD_DEPCTL_SNAK;
		else
		{
			NRF54_USBD_GINTMSK |= NRF54_USBD_GINTSTS_GOUTNAKEFF;
			NRF54_USBD_DCTL |= NRF54_USBD_DCTL_SGOUTNAK;
		}
	}
}

static void nRF54UsbdOutNak(void)
{
	for (uint8_t ep = 1; ep < NRF54_USBD_EP_COUNT; ++ep)
		if (s_Ctrlr.Xfer[ep][0].State == NRF54_XFER_DISABLING)
			NRF54_USBD_DOEPCTL(ep) |= NRF54_USBD_DEPCTL_EPDIS | NRF54_USBD_DEPCTL_SNAK;
	NRF54_USBD_GINTMSK &= ~NRF54_USBD_GINTSTS_GOUTNAKEFF;
	NRF54_USBD_DCTL |= NRF54_USBD_DCTL_CGOUTNAK;
}

static bool nRFUsbRegInit(void)
{
	memset(&s_Ctrlr, 0, sizeof(s_Ctrlr));
	s_Ctrlr.Xfer[0][0].Mps = NRF54_USBD_EP0_MPS;
	s_Ctrlr.Xfer[0][1].Mps = NRF54_USBD_EP0_MPS;
	return true;
}

static bool nRFUsbRegStart(void)
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
	gusbcfg &= ~(NRF54_USBD_GUSBCFG_FORCEHSTMODE | (1UL << 6) | (1UL << 4));
	if ((NRF54_USBD_GHWCFG4 & (3UL << 14)) != 0U)
		gusbcfg |= 1UL << 3; // 16-bit UTMI interface.
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
	s_Ctrlr.FifoDepth = s_Ctrlr.FifoTop = dfifoDepth;
	NRF54_USBD_GDFIFOCFG =
		((uint32_t)dfifoDepth << 16) | totalFifo;

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
	dcfg &= ~((1UL << 23) | NRF54_USBD_DCFG_DEVSPD_Msk |
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
		NRF54_USBD_GINTSTS_ISOINCOMP | NRF54_USBD_GINTSTS_ISOOUTCOMP |
		NRF54_USBD_GINTSTS_USBSUSP |
		NRF54_USBD_GINTSTS_WKUPINT;

	s_Ctrlr.Started = true;
	USB_TRACE("USB START cfg=%08lx ahb=%08lx dcfg=%08lx mask=%08lx\n",
		NRF54_USBD_GUSBCFG, NRF54_USBD_GAHBCFG, NRF54_USBD_DCFG, NRF54_USBD_GINTMSK);
	return true;
}

static void nRFUsbRegStop(void)
{
	if (!s_Ctrlr.Started)
		return;
	NRF54_USBD_PCGCCTL &= ~NRF54_USBD_PCGCCTL_STOPPCLK;
	NRF54_USBD_GAHBCFG &= ~NRF54_USBD_GAHBCFG_GINT;
	NRF54_USBD_GINTMSK = 0U;
	NRF54_USBD_DCTL |= NRF54_USBD_DCTL_SFTDISCON;
	nRF54UsbdResetSoftware();
}

static bool nRFUsbRegHighSpeed(void)
{
	return s_Ctrlr.Started && s_Ctrlr.HighSpeed;
}

static void nRFUsbRegIntEnable(void)
{
	// LM20x routes the DWC2 interrupt directly to USBHS_IRQn. Its wrapper
	// has no CORE event or INTEN registers (USBHS_HAS_CORE_EVENT is zero).
	NRF54_USBD_GAHBCFG |= NRF54_USBD_GAHBCFG_GINT;
	NVIC_ClearPendingIRQ(USBHS_IRQn);
	NVIC_EnableIRQ(USBHS_IRQn);
}

static void nRFUsbRegIntDisable(void)
{
	NVIC_DisableIRQ(USBHS_IRQn);
	NRF54_USBD_GAHBCFG &= ~NRF54_USBD_GAHBCFG_GINT;
}

static void nRFUsbRegConnect(void)
{
	NRF54_USBD_DCTL &= ~NRF54_USBD_DCTL_SFTDISCON;
	NRF_USBHS->PHY.INPUTOVERRIDE = USBHS_PHY_INPUTOVERRIDE_ID_Msk;
	USB_TRACE("USB CONNECT dctl=%08lx ahb=%08lx\n", NRF54_USBD_DCTL, NRF54_USBD_GAHBCFG);
}

static void nRFUsbRegDisconnect(void)
{
	NRF54_USBD_PCGCCTL &= ~NRF54_USBD_PCGCCTL_STOPPCLK;
	NRF54_USBD_DCTL |= NRF54_USBD_DCTL_SFTDISCON;
}

static void nRFUsbRegRemoteWakeup(void)
{
	if (!s_Ctrlr.Suspended)
		return;
	NRF54_USBD_PCGCCTL &= ~NRF54_USBD_PCGCCTL_STOPPCLK;
	// USB suspend is detected after 3 ms idle; remote wake requires 5 ms.
	nrfx_coredep_delay_us(2000U);
	if (!s_Ctrlr.Started || !s_UsbdVbusLevel || !s_Ctrlr.Suspended)
		return;
	NRF54_USBD_DCTL |= NRF54_USBD_DCTL_RMTWKUPSIG;
	nrfx_coredep_delay_us(2000U);
	if (s_Ctrlr.Started && s_UsbdVbusLevel)
		NRF54_USBD_DCTL &= ~NRF54_USBD_DCTL_RMTWKUPSIG;
}

static void nRFUsbRegSofEnable(bool Enable)
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

static void nRFUsbRegSetAddress(uint8_t Address)
{
	// Follow the DWC2 sequence: program DCFG while handling SET_ADDRESS,
	// before the generic core arms the status IN packet.
	NRF54_USBD_DCFG = (NRF54_USBD_DCFG & ~NRF54_USBD_DCFG_DEVADDR_Msk) |
		((uint32_t)(Address & 0x7FU) << NRF54_USBD_DCFG_DEVADDR_Pos);
	USB_TRACE("USB ADDRESS programmed=%lu dcfg=%08lx\n", Address, NRF54_USBD_DCFG);
}

static bool nRFUsbRegEpOpen(uint8_t epAddr, uint8_t type, uint16_t mps)
{
	const uint8_t epNum = USB_ENDPADDR_NUM(epAddr);
	if (!s_Ctrlr.Started || epNum == 0U || (epAddr & 0x70U) != 0U || mps == 0U ||
		(type != USB_ENDPATT_TRANS_BULK && type != USB_ENDPATT_TRANS_INT &&
		 type != USB_ENDPATT_TRANS_ISO))
		return false;
	const uint16_t maxMps = type == USB_ENDPATT_TRANS_BULK ?
		(s_Ctrlr.HighSpeed ? 512U : 64U) : type == USB_ENDPATT_TRANS_INT ?
		(s_Ctrlr.HighSpeed ? 1024U : 64U) : (s_Ctrlr.HighSpeed ? 1024U : 1023U);
	if (mps > maxMps)
		return false; // Also rejects high-bandwidth transaction bits.
	nRF54UsbdXfer_t *pXfer = nRF54UsbdGetXfer(epAddr);
	if (pXfer->Open || nRFUsbGetEpReg(epAddr)->Handler == nullptr)
		return false;
	const bool in = USB_ENDPADDR_IS_IN(epAddr);
	if (!(in ? nRF54UsbdAllocateTxFifo(epNum, mps) : nRF54UsbdGrowRxFifo(mps)))
		return false;
	++nRFUsbGetEpReg(epAddr)->Generation;
	*pXfer = {};
	pXfer->Mps = mps;
	pXfer->Type = type;
	pXfer->Open = true;
	const uint32_t ctl = mps | (nRF54UsbdEpType(type) << NRF54_USBD_DEPCTL_EPTYPE_Pos) |
		NRF54_USBD_DEPCTL_USBACTEP | NRF54_USBD_DEPCTL_SETD0PID | NRF54_USBD_DEPCTL_SNAK;
	if (in)
	{
		NRF54_USBD_DIEPINT(epNum) = 0xFFFFFFFFUL;
		NRF54_USBD_DIEPCTL(epNum) = ctl | ((uint32_t)epNum << NRF54_USBD_DIEPCTL_TXFNUM_Pos);
		NRF54_USBD_DAINTMSK |= NRF54_USBD_DAINT_IN(epNum);
	}
	else
	{
		NRF54_USBD_DOEPINT(epNum) = 0xFFFFFFFFUL;
		NRF54_USBD_DOEPCTL(epNum) = ctl;
		NRF54_USBD_DAINTMSK |= NRF54_USBD_DAINT_OUT(epNum);
		if (type != USB_ENDPATT_TRANS_ISO)
			nRFUsbEpRegisteredEvent(epNum, USB_CTRLR_EVT_DRDY, 0U);
	}
	return true;
}

static void nRFUsbRegEpClose(uint8_t EpAddr)
{
	const uint8_t ep = USB_ENDPADDR_NUM(EpAddr);
	if (ep == 0U || (EpAddr & 0x70U) != 0U)
		return;
	++nRFUsbGetEpReg(EpAddr)->Generation;
	nRF54UsbdXfer_t *pXfer = nRF54UsbdGetXfer(EpAddr);
	if (s_Ctrlr.Started && pXfer->Open)
	{
		if (!nRF54UsbdDisableEndpoint(EpAddr, false))
		{
			nRFUsbAbortCore();
			return;
		}
		if (USB_ENDPADDR_IS_IN(EpAddr))
		{
			NRF54_USBD_DAINTMSK &= ~NRF54_USBD_DAINT_IN(ep);
			NRF54_USBD_DIEPINT(ep) = 0xFFFFFFFFUL;
		}
		else
		{
			NRF54_USBD_DAINTMSK &= ~NRF54_USBD_DAINT_OUT(ep);
			NRF54_USBD_DOEPINT(ep) = 0xFFFFFFFFUL;
		}
	}
	*pXfer = {};
	if (USB_ENDPADDR_IS_IN(EpAddr))
	{
		s_Ctrlr.TxFifoWords[ep] = 0U;
		s_Ctrlr.FifoTop = s_Ctrlr.FifoDepth;
		if (s_Ctrlr.Started)
			for (unsigned i = 0; i < NRF54_USBD_EP_COUNT; ++i)
				if (s_Ctrlr.TxFifoWords[i] != 0U)
				{
					const uint16_t base = (uint16_t)(i == 0U ?
						NRF54_USBD_GNPTXFSIZ : NRF54_USBD_DIEPTXF(i));
					if (base < s_Ctrlr.FifoTop)
						s_Ctrlr.FifoTop = base;
				}
	}
}

static void nRFUsbRegEpCloseAll(void)
{
	for (uint8_t epNum = 1U; epNum < NRF54_USBD_EP_COUNT; epNum++)
	{
		nRFUsbRegEpClose(epNum);
		nRFUsbRegEpClose((uint8_t)(epNum | USB_ENDPADDR_DIR_IN));
	}

	if (s_Ctrlr.Started)
		(void)nRF54UsbdFlushRx();
}

static bool nRFUsbRegEpXfer(uint8_t EpAddr, uint8_t *pBuffer, uint16_t TotalBytes)
{
	if (!s_Ctrlr.Started || (EpAddr & 0x70U) != 0U)
		return false;
	const uint8_t epNum = USB_ENDPADDR_NUM(EpAddr);
	const bool in = USB_ENDPADDR_IS_IN(EpAddr);
	nRF54UsbdXfer_t *pXfer = nRF54UsbdGetXfer(EpAddr);
	if (pXfer->State != NRF54_XFER_IDLE ||
		(epNum != 0U && (!pXfer->Open || (TotalBytes != 0U && pBuffer == nullptr))))
		return false;
	if (epNum == 0U)
	{
		if (in && TotalBytes != 0U && pBuffer == nullptr)
			return false;
		pXfer->pBuffer = pBuffer;
		pXfer->TotalLen = TotalBytes;
		pXfer->ActualLen = 0U;
		pXfer->State = NRF54_XFER_ACTIVE;
		return nRF54UsbdStartEp0Chunk(EpAddr);
	}

	volatile uint32_t *pCtl = in ? &NRF54_USBD_DIEPCTL(epNum) : &NRF54_USBD_DOEPCTL(epNum);
	if ((*pCtl & NRF54_USBD_DEPCTL_STALL) != 0U)
		return false;
	const uint32_t misalign = (uintptr_t)pBuffer & 3U;
	if (misalign != 0U)
	{
		if (!in || pXfer->Type == USB_ENDPATT_TRANS_ISO)
			return false;
		// Byte CFifo heads can be unaligned. Like nRF52, send the short
		// alignment prefix and report only those bytes to the owner.
		if (TotalBytes > 4U - misalign)
			TotalBytes = 4U - misalign;
		memcpy(&pXfer->Scratch, pBuffer, TotalBytes);
		pBuffer = (uint8_t *)&pXfer->Scratch;
	}
	if (pBuffer == nullptr)
		pBuffer = (uint8_t *)&pXfer->Scratch;
	const uint32_t packets = TotalBytes == 0U ? 1U :
		((uint32_t)TotalBytes + pXfer->Mps - 1U) / pXfer->Mps;
	if (packets > 0x3FFU || (packets > 1U && (pXfer->Mps & 3U) != 0U) ||
		(pXfer->Type == USB_ENDPATT_TRANS_ISO && packets > 1U))
		return false;
	pXfer->pBuffer = pBuffer;
	pXfer->TotalLen = TotalBytes;
	pXfer->ActualLen = 0U;
	pXfer->State = NRF54_XFER_ACTIVE;
	pXfer->Result = USB_CTRLR_EVT_XFER_CMPL;
	uint32_t size = TotalBytes | (packets << NRF54_USBD_DEPTSIZ_PKTCNT_Pos);
	uint32_t ctl = *pCtl;
	if (pXfer->Type == USB_ENDPATT_TRANS_ISO)
	{
		// The SOF callback supplies the next (micro)frame's packet.
		const bool odd = ((NRF54_USBD_DSTS >> NRF54_USBD_DSTS_FNSOF_Pos) & 1U) != 0U;
		ctl &= ~(NRF54_USBD_DEPCTL_SETD0PID | NRF54_USBD_DEPCTL_SETODDFR);
		ctl |= odd ? NRF54_USBD_DEPCTL_SETD0PID : NRF54_USBD_DEPCTL_SETODDFR;
	}
	if (in)
	{
		if (pXfer->Type == USB_ENDPATT_TRANS_ISO || pXfer->Type == USB_ENDPATT_TRANS_INT)
			size |= 1UL << 29; // One periodic transaction per service opportunity.
		NRF54_USBD_DIEPDMA(epNum) = (uint32_t)(uintptr_t)pBuffer;
		NRF54_USBD_DIEPTSIZ(epNum) = size;
	}
	else
	{
		NRF54_USBD_DOEPDMA(epNum) = (uint32_t)(uintptr_t)pBuffer;
		NRF54_USBD_DOEPTSIZ(epNum) = size;
	}
	__DSB();
	*pCtl = ctl | NRF54_USBD_DEPCTL_CNAK | NRF54_USBD_DEPCTL_EPENA;
	return true;
}



static void nRFUsbRegEpStall(uint8_t EpAddr)
{
	USB_TRACE("USB STALL ep=%02lx\n", EpAddr);
	if (!s_Ctrlr.Started || (EpAddr & 0x70U) != 0U)
		return;
	const uint8_t ep = USB_ENDPADDR_NUM(EpAddr);
	if (ep == 0U)
	{
		s_Ctrlr.Xfer[0][0].State = NRF54_XFER_IDLE;
		s_Ctrlr.Xfer[0][1].State = NRF54_XFER_IDLE;
		NRF54_USBD_DIEPCTL(0) |= NRF54_USBD_DEPCTL_STALL;
		NRF54_USBD_DOEPCTL(0) |= NRF54_USBD_DEPCTL_STALL;
		nRF54UsbdPrimeSetup();
	}
	else if (nRF54UsbdGetXfer(EpAddr)->Type != USB_ENDPATT_TRANS_ISO)
	{
		if (!nRF54UsbdDisableEndpoint(EpAddr, true))
		{
			nRFUsbAbortCore();
			return;
		}
		if (nRF54UsbdGetXfer(EpAddr)->State == NRF54_XFER_ACTIVE)
			nRF54UsbdGetXfer(EpAddr)->State = NRF54_XFER_IDLE;
	}
}

static void nRFUsbRegEpClearStall(uint8_t EpAddr)
{
	if (!s_Ctrlr.Started || (EpAddr & 0x70U) != 0U)
		return;
	const uint8_t ep = USB_ENDPADDR_NUM(EpAddr);
	nRF54UsbdXfer_t *pXfer = nRF54UsbdGetXfer(EpAddr);
	if (pXfer->Type == USB_ENDPATT_TRANS_ISO)
		return;
	volatile uint32_t *pCtl = USB_ENDPADDR_IS_IN(EpAddr) ?
		&NRF54_USBD_DIEPCTL(ep) : &NRF54_USBD_DOEPCTL(ep);
	*pCtl = (*pCtl & ~NRF54_USBD_DEPCTL_STALL) | (ep ? NRF54_USBD_DEPCTL_SETD0PID : 0U);
	if (ep != 0U && pXfer->Open && pXfer->State == NRF54_XFER_IDLE)
	{
		if (pXfer->pBuffer != nullptr)
			(void)nRFUsbRegEpXfer(EpAddr, pXfer->pBuffer, pXfer->TotalLen);
		else if (!USB_ENDPADDR_IS_IN(EpAddr))
			nRFUsbEpRegisteredEvent(ep, USB_CTRLR_EVT_DRDY, 0U);
	}
}

extern "C" void USBHS_IRQHandler(void)
{
	if (!s_Ctrlr.Started || !s_UsbdVbusLevel)
		return;
	const uint32_t status = NRF54_USBD_GINTSTS & NRF54_USBD_GINTMSK;
	if ((status & (NRF54_USBD_GINTSTS_USBRST | NRF54_USBD_GINTSTS_WKUPINT)) != 0U)
		NRF54_USBD_PCGCCTL &= ~NRF54_USBD_PCGCCTL_STOPPCLK;
	if ((status & NRF54_USBD_GINTSTS_USBRST) != 0U)
	{
		NRF54_USBD_GINTSTS = NRF54_USBD_GINTSTS_USBRST;
		nRF54UsbdBusReset();
		return; // Endpoint bits in this snapshot belong to the old bus session.
	}
	if ((status & NRF54_USBD_GINTSTS_ENUMDONE) != 0U)
	{
		NRF54_USBD_GINTSTS = NRF54_USBD_GINTSTS_ENUMDONE;
		s_Ctrlr.HighSpeed = (NRF54_USBD_DSTS & NRF54_USBD_DSTS_ENUMSPD_Msk) == NRF54_USBD_DSTS_ENUMSPD_HS;
		USB_TRACE("USB SPEED hs=%lu dsts=%08lx\n", s_Ctrlr.HighSpeed, NRF54_USBD_DSTS);
	}
	if ((status & NRF54_USBD_GINTSTS_OEPINT) != 0U)
		nRF54UsbdOutInterrupt(); // SETUP cancels stale EP0 IN before IN is read.
	if (!s_Ctrlr.Started)
		return;
	if ((status & NRF54_USBD_GINTSTS_IEPINT) != 0U)
		nRF54UsbdInInterrupt();
	if (!s_Ctrlr.Started)
		return;
	if ((status & NRF54_USBD_GINTSTS_ISOINCOMP) != 0U)
	{
		nRF54UsbdIsoIncomplete(true);
		NRF54_USBD_GINTSTS = NRF54_USBD_GINTSTS_ISOINCOMP;
	}
	if ((status & NRF54_USBD_GINTSTS_ISOOUTCOMP) != 0U)
	{
		nRF54UsbdIsoIncomplete(false);
		NRF54_USBD_GINTSTS = NRF54_USBD_GINTSTS_ISOOUTCOMP;
	}
	if ((status & NRF54_USBD_GINTSTS_GOUTNAKEFF) != 0U)
		nRF54UsbdOutNak();
	if ((status & NRF54_USBD_GINTSTS_USBSUSP) != 0U &&
		(status & NRF54_USBD_GINTSTS_WKUPINT) == 0U)
	{
		NRF54_USBD_GINTSTS = NRF54_USBD_GINTSTS_USBSUSP;
		s_Ctrlr.Suspended = true;
		USB_TRACE("USB SUSPEND status=%08lx\n", status);
		nRF54UsbdEmitSimple(USB_CTRLR_EVT_SUSPEND);
		if (s_UsbdLowPowerSuspend)
			NRF54_USBD_PCGCCTL |= NRF54_USBD_PCGCCTL_STOPPCLK;
	}
	if ((status & NRF54_USBD_GINTSTS_WKUPINT) != 0U)
	{
		NRF54_USBD_GINTSTS = NRF54_USBD_GINTSTS_WKUPINT | NRF54_USBD_GINTSTS_USBSUSP;
		s_Ctrlr.Suspended = false;
		USB_TRACE("USB RESUME status=%08lx\n", status);
		nRF54UsbdEmitSimple(USB_CTRLR_EVT_RESUME);
	}
	if ((status & NRF54_USBD_GINTSTS_SOF) != 0U)
	{
		NRF54_USBD_GINTSTS = NRF54_USBD_GINTSTS_SOF;
		if (s_Ctrlr.SofEnabled && !s_Ctrlr.Suspended)
		{
			// DWC2 queues ISO for the next microframe, including interval
			// selection. In full-speed mode FNSOF counts 1 ms frames.
			UsbCtrlrEvt_t evt = { .Type = USB_CTRLR_EVT_SOF,
				.FrameNo = (uint16_t)((((NRF54_USBD_DSTS & NRF54_USBD_DSTS_FNSOF_Msk) >>
					NRF54_USBD_DSTS_FNSOF_Pos) + 1U) & 0x3FFFU) };
			nRF54UsbdEmit(&evt);
		}
	}
}


//
// Entry points declared in usb.h. Each validates DevNo and then runs the
// power stage and the register stage in the order the hardware needs.
//

bool UsbCtrlrInit(int DevNo, const UsbCtrlrCfg_t *pCfg)
{
	if (!nRFUsbValidDevNo(DevNo) || pCfg == NULL)
	{
		return false;
	}

#if NRF54_USB_TRACE
	// Verify stdout before USB starts, without waiting for the event queue.
	printf("USB TRACE enabled dev=%d\n", DevNo);
	fflush(stdout);
#endif

	for (auto &ep : s_EpReg)
		for (auto &reg : ep)
		{
			++reg.Generation;
			reg.Handler = nullptr;
			reg.pContext = nullptr;
		}

	// Power first. The register stage must not touch a peripheral that has no
	// clock, which is why nRFUsbRegInit only sets software state.
	if (!nRFUsbPowerInit(pCfg))
	{
		return false;
	}

	return nRFUsbRegInit();
}

bool UsbCtrlrStart(int DevNo)
{
	if (!nRFUsbValidDevNo(DevNo))
	{
		return false;
	}
	if (s_Ctrlr.Started)
		return true;

	// One call where usbd.h and usbd_ctrlr.h used to need two. Power, clock
	// and PHY come up, then endpoint zero is prepared.
	if (!nRFUsbPowerStart())
	{
		USB_TRACE("USB START power failed vbus=%lu\n", s_UsbdVbusLevel);
		return false;
	}

	const uint32_t state = DisableInterrupt();
	if (!nRFUsbVbusDetected() || !nRFUsbRegStart())
	{
		USB_TRACE("USB START core failed vbus=%lu\n", s_UsbdVbusLevel);
		nRFUsbPowerStop();
		EnableInterrupt(state);
		return false;
	}
	EnableInterrupt(state);

	return true;
}

void UsbCtrlrStop(int DevNo)
{
	if (!nRFUsbValidDevNo(DevNo))
	{
		return;
	}

	const uint32_t state = DisableInterrupt();
	s_UsbdRestart = s_UsbdDetached = false;
	nRFUsbRegStop();
	nRFUsbPowerStop();
	EnableInterrupt(state);
}

void UsbCtrlrProcess(int DevNo)
{
	if (!nRFUsbValidDevNo(DevNo))
		return;
	nRFUsbPowerProcess();
#if NRF54_USB_TRACE
	nRF54UsbTraceFlush();
#endif
}

bool UsbCtrlrVbusDetected(int DevNo)
{
	if (!nRFUsbValidDevNo(DevNo))
		return false;
	const uint32_t state = DisableInterrupt();
	const bool detected = nRFUsbVbusDetected();
	EnableInterrupt(state);
	return detected;
}

bool UsbCtrlrHighSpeed(int DevNo)
{
	return nRFUsbValidDevNo(DevNo) && nRFUsbRegHighSpeed();
}

size_t UsbCtrlrGetSerial(int DevNo, char *pBuff, size_t BuffLen)
{
	return nRFUsbValidDevNo(DevNo) ? nRFUsbSerial(pBuff, BuffLen) : 0;
}

void UsbCtrlrIntEnable(int DevNo)
{
	if (nRFUsbValidDevNo(DevNo) && s_Ctrlr.Started)
	{
		nRFUsbRegIntEnable();
	}
}

void UsbCtrlrIntDisable(int DevNo)
{
	if (nRFUsbValidDevNo(DevNo) && s_Ctrlr.Started)
	{
		nRFUsbRegIntDisable();
	}
}

void UsbCtrlrConnect(int DevNo)
{
	if (nRFUsbValidDevNo(DevNo) && s_Ctrlr.Started)
	{
		nRFUsbRegConnect();
	}
}

void UsbCtrlrDisconnect(int DevNo)
{
	if (nRFUsbValidDevNo(DevNo) && s_Ctrlr.Started)
	{
		nRFUsbRegDisconnect();
	}
}

void UsbCtrlrRemoteWakeup(int DevNo)
{
	if (nRFUsbValidDevNo(DevNo) && s_Ctrlr.Started)
	{
		nRFUsbRegRemoteWakeup();
	}
}

void UsbCtrlrSofEnable(int DevNo, bool Enable)
{
	if (nRFUsbValidDevNo(DevNo) && s_Ctrlr.Started)
	{
		nRFUsbRegSofEnable(Enable);
	}
}

void UsbCtrlrSetAddress(int DevNo, uint8_t Address)
{
	if (nRFUsbValidDevNo(DevNo) && s_Ctrlr.Started)
	{
		nRFUsbRegSetAddress(Address);
	}
}

bool UsbCtrlrEpOpen(int DevNo, const UsbEndPointDesc_t *pDesc)
{
	return nRFUsbValidDevNo(DevNo) && pDesc != NULL &&
		nRFUsbRegEpOpen(pDesc->bEndpointAddress,
			pDesc->bmAttributes & 0x03U, pDesc->wMaxPacketSize);
}

bool UsbCtrlrIsoOpen(int DevNo, uint8_t EpNo, bool bIn,
	uint16_t MaxPacketSize)
{
	const uint8_t epAddr = (uint8_t)(EpNo |
		(bIn ? USB_ENDPADDR_DIR_IN : 0U));
	return nRFUsbValidDevNo(DevNo) && EpNo < NRF54_USBD_EP_COUNT &&
		nRFUsbRegEpOpen(epAddr, USB_ENDPATT_TRANS_ISO, MaxPacketSize);
}

bool UsbCtrlrEpOpenData(int DevNo, uint8_t EpNo, bool bIn, uint8_t Type,
						 uint16_t MaxPacketSize)
{
	const uint8_t epAddr = (uint8_t)(EpNo |
		(bIn ? USB_ENDPADDR_DIR_IN : 0U));
	return nRFUsbValidDevNo(DevNo) && EpNo < NRF54_USBD_EP_COUNT &&
		nRFUsbRegEpOpen(epAddr, Type, MaxPacketSize);
}

void UsbCtrlrEpClose(int DevNo, uint8_t EpNo, bool bIn)
{
	if (nRFUsbValidDevNo(DevNo) && EpNo < NRF54_USBD_EP_COUNT)
	{
		const uint32_t state = DisableInterrupt();
		nRFUsbRegEpClose((uint8_t)(EpNo |
			(bIn ? USB_ENDPADDR_DIR_IN : 0U)));
		EnableInterrupt(state);
	}
}

void UsbCtrlrEpCloseAll(int DevNo)
{
	if (nRFUsbValidDevNo(DevNo))
	{
		const uint32_t state = DisableInterrupt();
		nRFUsbRegEpCloseAll();
		EnableInterrupt(state);
	}
}

void UsbCtrlrEpBind(int DevNo, uint8_t EpNo, bool bIn, bool bBlocking,
	UsbCtrlrEpHandler_t Handler, void *pContext)
{
	(void)bBlocking;
	if (!nRFUsbValidDevNo(DevNo) || EpNo == 0U || EpNo >= NRF54_USBD_EP_COUNT)
		return;
	const uint32_t state = DisableInterrupt();
	nRFUsbEpReg_t *pReg = &s_EpReg[EpNo][bIn];
	++pReg->Generation;
	pReg->Handler = Handler;
	pReg->pContext = pContext;
	EnableInterrupt(state);
}

bool UsbCtrlrEpReceive(int DevNo, uint8_t EpNo, uint8_t *pBuffer, uint16_t Capacity)
{
	if (!nRFUsbValidDevNo(DevNo) || EpNo == 0U || EpNo >= NRF54_USBD_EP_COUNT ||
		pBuffer == nullptr || ((uintptr_t)pBuffer & 3U) != 0U)
		return false;
	const uint32_t state = DisableInterrupt();
	nRF54UsbdXfer_t *pXfer = &s_Ctrlr.Xfer[EpNo][0];
	bool accepted = false;
	if (s_Ctrlr.Started && pXfer->Open && pXfer->State == NRF54_XFER_IDLE &&
		pXfer->pBuffer == nullptr && Capacity >= pXfer->Mps)
	{
		if (pXfer->Type == USB_ENDPATT_TRANS_ISO)
		{
			pXfer->pBuffer = pBuffer;
			accepted = true; // Armed by the next service-interval callback.
		}
		else
			accepted = nRFUsbRegEpXfer(EpNo, pBuffer, pXfer->Mps);
	}
	EnableInterrupt(state);
	return accepted;
}

void UsbCtrlrEpProcessEvent(int DevNo, uint8_t EpNo, bool bIn,
						 UsbCtrlrEvtType_t Event, uint16_t Value)
{
	if (!nRFUsbValidDevNo(DevNo) || EpNo == 0U || EpNo >= NRF_USB_EP_COUNT)
	{
		return;
	}

	nRFUsbEpReg_t *pReg = nRFUsbGetEpReg((uint8_t)(EpNo |
		(bIn ? USB_ENDPADDR_DIR_IN : 0U)));
	if (pReg->Handler != nullptr && (Event != USB_CTRLR_EVT_SOF ||
		(s_Ctrlr.Xfer[EpNo][bIn].Open && s_Ctrlr.Xfer[EpNo][bIn].Type == USB_ENDPATT_TRANS_ISO)))
	{
		pReg->Handler(Event, Value, pReg->pContext);
	}
}

bool UsbCtrlrEpSend(int DevNo, uint8_t EpNum, uint8_t *pBuffer, uint16_t Length)
{
	if (!nRFUsbValidDevNo(DevNo) || EpNum == 0U || EpNum >= NRF54_USBD_EP_COUNT)
		return false;
	const uint32_t state = DisableInterrupt();
	const bool accepted = nRFUsbRegEpXfer(USB_ENDPADDR_DIRIN(EpNum), pBuffer, Length);
	EnableInterrupt(state);
	return accepted;
}

bool UsbCtrlrIsoInit(int DevNo)
{
	return nRFUsbValidDevNo(DevNo);
}

uint16_t UsbCtrlrIsoTraceSnapshot(int DevNo, uint8_t **ppData)
{
	(void)DevNo;
	if (ppData != nullptr)
		*ppData = nullptr;
	return 0U;
}

bool UsbCtrlrIsoSend(int DevNo, uint8_t EpNum, uint8_t *pBuffer, uint16_t Length)
{
	if (!nRFUsbValidDevNo(DevNo) || EpNum == 0U || EpNum >= NRF54_USBD_EP_COUNT)
		return false;
	const uint32_t state = DisableInterrupt();
	nRF54UsbdXfer_t *pIn = &s_Ctrlr.Xfer[EpNum][1];
	nRF54UsbdXfer_t *pOut = &s_Ctrlr.Xfer[EpNum][0];
	bool accepted = false;
	if (s_Ctrlr.Started && !s_Ctrlr.Suspended && pIn->Open && pOut->Open &&
		pIn->Type == USB_ENDPATT_TRANS_ISO && pOut->Type == USB_ENDPATT_TRANS_ISO)
	{
		if (pOut->State == NRF54_XFER_IDLE)
		{
			if (pOut->pBuffer == nullptr)
				nRFUsbEpRegisteredEvent(EpNum, USB_CTRLR_EVT_DRDY, 0U);
			if (pOut->pBuffer != nullptr)
				(void)nRFUsbRegEpXfer(EpNum, pOut->pBuffer, pOut->Mps);
		}
		accepted = pBuffer == nullptr || (Length <= pIn->Mps &&
			nRFUsbRegEpXfer(USB_ENDPADDR_DIRIN(EpNum), pBuffer, Length));
	}
	EnableInterrupt(state);
	return accepted;
}


bool UsbCtrlrEp0Status(int DevNo, uint8_t EpAddr)
{
	if (!nRFUsbValidDevNo(DevNo) || (EpAddr != 0U && EpAddr != USB_ENDPADDR_DIR_IN))
		return false;
	const uint32_t state = DisableInterrupt();
	const bool accepted = nRFUsbRegEpXfer(EpAddr, nullptr, 0U);
	EnableInterrupt(state);
	return accepted;
}

int UsbCtrlrEp0Send(int DevNo, uint8_t *pBuffer, int Length)
{
	if (!nRFUsbValidDevNo(DevNo) || Length < 0 || Length > UINT16_MAX)
		return -1;

	// The upper layer submits the remainder on completion. Copy this packet
	// into the existing bounce buffer before returning to its source owner.
	const uint16_t copied = Length < (int)NRF54_USBD_EP0_MPS ?
		(uint16_t)Length : NRF54_USBD_EP0_MPS;
	const uint32_t state = DisableInterrupt();
	const bool accepted = nRFUsbRegEpXfer(USB_ENDPADDR_DIR_IN, pBuffer, copied);
	EnableInterrupt(state);
	return accepted ? copied : -1;
}

void UsbCtrlrEpStall(int DevNo, uint8_t EpNo, bool bIn)
{
	if (nRFUsbValidDevNo(DevNo) && EpNo < NRF54_USBD_EP_COUNT)
	{
		const uint32_t state = DisableInterrupt();
		nRFUsbRegEpStall((uint8_t)(EpNo |
			(bIn ? USB_ENDPADDR_DIR_IN : 0U)));
		EnableInterrupt(state);
	}
}

void UsbCtrlrEpClearStall(int DevNo, uint8_t EpNo, bool bIn)
{
	if (nRFUsbValidDevNo(DevNo) && EpNo < NRF54_USBD_EP_COUNT)
	{
		const uint32_t state = DisableInterrupt();
		nRFUsbRegEpClearStall((uint8_t)(EpNo |
			(bIn ? USB_ENDPADDR_DIR_IN : 0U)));
		EnableInterrupt(state);
	}
}
