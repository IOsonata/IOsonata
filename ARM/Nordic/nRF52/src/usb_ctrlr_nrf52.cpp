/**-------------------------------------------------------------------------
@file	usb_ctrlr_nrf52.cpp

@brief	USB device controller for Nordic nRF52 parts.

Direct register implementation for the full-speed USBD peripheral. The port
owns bus power, clock and VBUS handling, endpoint events and the single shared
EasyDMA channel.

DevNo selects the controller. Every nRF part has exactly one, USB_CTRLR_CNT is
1, so the entry points validate DevNo and the state stays a singleton. Arraying
it is work for the first part that carries two.

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
#include <stddef.h>
#include <stdatomic.h>
#include <string.h>

#include "nrf.h"
#include "nrf_peripherals.h"
#include "nrf_erratas.h"
#include "hal/nrf_ficr.h"

#include "istddef.h"
#include "app_evt_handler.h"
#include "cfifo.h"
#include "coredev/interrupt.h"
#include "usb/usb.h"


// Bus power, clock and VBUS.


// A legacy nRF52 SoftDevice owns HFCLK while it is enabled. Only clock
// request/release needs the SoftDevice API; USB controller operation does not.
#if defined(NRF52_SERIES) && \
	(defined(SOFTDEVICE_PRESENT) || defined(S140))
#define SOFTDEVICE_PRESENT			1
#endif

#ifdef SOFTDEVICE_PRESENT
#include "nrf_soc.h"
#include "nrf_sdm.h"
#include "nrf_error.h"
#endif

#ifndef NRFX_USBD_XTAL_WAIT_LOOPS
#define NRFX_USBD_XTAL_WAIT_LOOPS			2000000UL
#endif

#ifndef NRFX_USBD_READY_WAIT_LOOPS
#define NRFX_USBD_READY_WAIT_LOOPS			2000000UL
#endif


//
// Errata registers. These addresses are not in the MDK because the registers
// they name are not documented; they come from Nordic's own driver and from
// the errata sheet. nrf52_errata_nnn() decides whether the part in hand needs
// each one, so the values below are only ever written where they apply.
//
#define NRFX_USBD_ERRATA_UNLOCK_REG		0x4006EC00UL
#define NRFX_USBD_ERRATA_UNLOCK_KEY		0x00009375UL
#define NRFX_USBD_ERRATA_171_REG			0x4006EC14UL
#define NRFX_USBD_ERRATA_187_REG			0x4006ED14UL
#define NRFX_USBD_ERRATA_166_REG_A			(NRF_USBD_BASE + 0x800UL)
#define NRFX_USBD_ERRATA_166_REG_B			(NRF_USBD_BASE + 0x804UL)

#define NRFX_USBD_REG32(a)					(*(volatile uint32_t *)(a))


enum
{
	// The ordinary endpoint count excludes the dedicated ISO endpoint 8.
	NRF_USB_EP_COUNT = 9,
};

typedef struct __nRF_Usb_Ep_Registration
{
	uint8_t *pBuffer;
	UsbCtrlrEpHandler_t Handler;
	void *pContext;
	uint16_t Mps;
	bool bBlocking;
} nRFUsbEpReg_t;

// Fixed data-endpoint ownership lives outside the active-transfer state so a
// bus reset can cancel transfers without losing registrations.
static nRFUsbEpReg_t s_EpReg[NRF_USB_EP_COUNT][2];

// One USB controller per part, so the common power/clock state is file scope.
// Controller interrupts are owned by the corresponding UsbdCtrlr backend.
static uint8_t s_UsbdIntPrio;
static bool s_UsbdLowPowerSuspend;
static bool s_UsbdInitialized = false;
static bool s_UsbdStarted = false;

static inline __attribute__((always_inline)) bool nRFUsbdDmaActive(void);


//
// nRF52 USBD constants and types.
//

// Errata 199's hardware-visible EasyDMA busy register is also the shared DMA
// ownership flag: 0x82 before STARTEP and zero after ENDEP.
#define NRFX_USBD_EASYDMA_BUSY_REG			(*((volatile uint32_t *)0x40027C1CUL))
#define NRFX_USBD_EASYDMA_BUSY_REG_BUSY		0x82UL
#define NRFX_USBD_EASYDMA_BUSY_REG_CLEAR	0UL

// EP1-7 have fourteen regular directions. Sixteen slots let CFifo use its
// mask indexing path; EP0 and ISO do not occupy this queue.
#define NRFUSBD_QUE_DEPTH			16U
#define NRFUSBD_EP0_QUE_DEPTH		4U

enum
{
	NRFX_USBD_DATA_EP_COUNT = 8,
	NRFX_USBD_EP_COUNT = 9,
	NRFX_USBD_ISO_EP_NO = 8,
	NRFX_USBD_MAX_PACKET_SIZE = 64,
	NRFX_USBD_ISO_MAX_PACKET_SIZE = 512,
	NRFX_USBD_XFER_EVT_OUT = 0x80U,
	NRFX_USBD_ISO_OUT_OPEN = 1U,
	NRFX_USBD_ISO_IN_OPEN = 2U,
};

#pragma pack(push, 4)

typedef struct __nRF_Usbd_Xfer
{
	uint8_t *pBuffer;
	uint16_t TotalLen;
	volatile uint16_t ActualLen;
} nRFUsbdXfer_t;

typedef struct __nRF_Usbd_Ctrlr
{
	// Regular endpoints use registered buffers and queued packet lengths.
	nRFUsbdXfer_t Ep0[2];
	nRFUsbdXfer_t Iso[2];
	bool SofEnabled;
} nRFUsbdCtrlr_t;

// One EasyDMA engine serves every endpoint in both directions, so a transfer
// request waits in this descriptor queue and starts in submission order.
//
// An endpoint cannot ask for a second transfer in the same direction until the
// first completes, so one slot per endpoint per direction is always enough and
// the queue cannot overflow.
typedef struct __nRF_Usbd_Que {
	uint8_t EpNum;				//!< Hardware endpoint number
	uint8_t Dir;					//!< 0 for OUT, 1 for IN
	uint16_t Len;				//!< Bytes this transfer moves
} nRFUsbdQue_t;

typedef struct __nRF_Ep_Packet {
	nRFUsbdQue_t Hdr;
	uint8_t Payload[NRFX_USBD_MAX_PACKET_SIZE];
} nRFEPPkt_t;

#pragma pack(pop)


//
// nRF52 USBD state.
//

static nRFUsbdCtrlr_t s_Ctrlr;
alignas(4) static uint8_t s_QueMem[
	CFIFO_TOTAL_MEMSIZE(NRFUSBD_QUE_DEPTH, sizeof(nRFUsbdQue_t))];

alignas(4) static uint8_t s_Ep0QueMem[
	CFIFO_TOTAL_MEMSIZE(NRFUSBD_EP0_QUE_DEPTH, sizeof(nRFEPPkt_t))];
static hCFifo_t s_hQue;
static hCFifo_t s_hEp0Que;

// EP0 accepts descriptor and class buffers from the generic USB layer. Those
// buffers may be const flash or have arbitrary alignment, while nRF52 USBD
// EasyDMA requires controller-visible, word-aligned RAM. Stage one control
// packet here in either direction; control transfers are serialized by EP0.
alignas(4) static uint8_t s_Ep0Bounce[NRFX_USBD_MAX_PACKET_SIZE];

// Keep event state together so the controller can address it from one base.
// Every field retains its existing atomic type and ordering.
static struct
{
	atomic_bool BusSuspended;
	atomic_bool SuspendPending;
	atomic_bool RemoteWakePending;
	atomic_bool HostResumePending;
	atomic_bool MacAwake;
	atomic_bool IsoInReady;
	atomic_bool IsoOutReady;
	uint16_t IsoOutSize;
	atomic_uint_fast8_t IsoOpen;
	// ISO buffers remain owned through deferred completion. A non-NULL
	// transfer buffer identifies a request waiting for its frame/DMA turn.
	atomic_uint_fast8_t IsoBusy;
	atomic_uint_fast8_t IsoComplete;
	uint32_t IsoGeneration[2];
} s_EventState;

static bool nRFUsbdFinishIsoDma(bool In);
static void nRFUsbdRetryIsoComplete(void);

static void nRFUsbdHostResumeDetected(void);


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

// Share callback dispatch across regular and ISO event paths.
static __attribute__((noinline))
void nRFUsbEpRegisteredEvent(uint8_t EpAddr, UsbCtrlrEvtType_t Event,
								 uint16_t Length, UsbCtrlrXferResult_t Result)
{
	nRFUsbEpReg_t *pReg = nRFUsbGetEpReg(EpAddr);
	pReg->Handler(EpAddr, Event, Length, Result, pReg->pContext);
}

#ifdef SOFTDEVICE_PRESENT
/**
 * Whether a SoftDevice is programmed at all.
 *
 * sd_softdevice_is_enabled is an SVC. On a part with no SoftDevice in flash
 * nothing implements that vector, so the call lands in the default handler
 * and stops there. The image has to be found before it may be asked
 * anything. Same test as SdPresent in nvm_nrfx.cpp.
 */
static bool UsbdSdPresent(void)
{
#if defined(SD_MAGIC_NUMBER) && defined(MBR_SIZE)
	return SD_MAGIC_NUMBER_GET(MBR_SIZE) == SD_MAGIC_NUMBER;
#else
	return false;
#endif
}

static bool UsbdSdRunning(void)
{
	uint8_t en = 0;

	if (UsbdSdPresent() == false)
	{
		return false;
	}

	if (sd_softdevice_is_enabled(&en) != NRF_SUCCESS)
	{
		return false;
	}

	return en != 0;
}
#endif

__attribute__((weak)) bool UsbdXtalRequest(void)
{
#ifdef SOFTDEVICE_PRESENT
	if (UsbdSdRunning())
	{
		uint32_t running = 0;

		if (sd_clock_hfclk_request() != NRF_SUCCESS)
		{
			return false;
		}

		for (uint32_t i = 0; i < NRFX_USBD_XTAL_WAIT_LOOPS; i++)
		{
			if (sd_clock_hfclk_is_running(&running) != NRF_SUCCESS)
			{
				return false;
			}

			if (running != 0)
			{
				return true;
			}
		}

		(void)sd_clock_hfclk_release();

		return false;
	}
#endif

	if ((NRF_CLOCK->HFCLKSTAT &
		 (CLOCK_HFCLKSTAT_STATE_Msk | CLOCK_HFCLKSTAT_SRC_Msk)) ==
		(CLOCK_HFCLKSTAT_STATE_Msk |
		 (CLOCK_HFCLKSTAT_SRC_Xtal << CLOCK_HFCLKSTAT_SRC_Pos)))
	{
		return true;
	}

	NRF_CLOCK->EVENTS_HFCLKSTARTED = 0;
	NRF_CLOCK->TASKS_HFCLKSTART = 1;

	for (uint32_t i = 0; i < NRFX_USBD_XTAL_WAIT_LOOPS; i++)
	{
		if (NRF_CLOCK->EVENTS_HFCLKSTARTED != 0)
		{
			NRF_CLOCK->EVENTS_HFCLKSTARTED = 0;
			return true;
		}
	}

	return false;
}

__attribute__((weak)) void UsbdXtalRelease(void)
{
#ifdef SOFTDEVICE_PRESENT
	if (UsbdSdRunning())
	{
		(void)sd_clock_hfclk_release();
		return;
	}
#endif

	// Left running on both parts. Other peripherals take these clocks without
	// counting, so stopping one here would stop it under them.
}


/**
 * Write one errata register.
 *
 * The registers these workarounds use sit behind an unlock word. When that
 * word reads zero the block is locked and a write to the errata register is
 * discarded, so the key has to be presented around it. Errata 187 is titled
 * "USBD cannot be enabled", and losing its write is not visible until ENABLE
 * is set and EVENTCAUSE never reports READY.
 *
 * Same sequence as Nordic's own driver.
 */
static void UsbdErrataWrite(uint32_t Reg, uint32_t Value)
{
	if (NRFX_USBD_REG32(NRFX_USBD_ERRATA_UNLOCK_REG) == 0)
	{
		NRFX_USBD_REG32(NRFX_USBD_ERRATA_UNLOCK_REG) =
			NRFX_USBD_ERRATA_UNLOCK_KEY;
		NRFX_USBD_REG32(Reg) = Value;
		NRFX_USBD_REG32(NRFX_USBD_ERRATA_UNLOCK_REG) =
			NRFX_USBD_ERRATA_UNLOCK_KEY;
	}
	else
	{
		NRFX_USBD_REG32(Reg) = Value;
	}
}

/**
 * Errata 187 and 171 are applied around the enable and taken away again once
 * the controller reports itself ready. Errata 166 is undone at the same time.
 * Nordic's own driver does exactly this and the order matters : the writes
 * before the enable are what make the enable work.
 */
static void UsbdErrataApply(void)
{
	if (nrf52_errata_187())
	{
		UsbdErrataWrite(NRFX_USBD_ERRATA_187_REG, 0x00000003UL);
	}

	if (nrf52_errata_171())
	{
		UsbdErrataWrite(NRFX_USBD_ERRATA_171_REG, 0x000000C0UL);
	}
}

static void UsbdErrataRevert(void)
{
	if (nrf52_errata_171())
	{
		UsbdErrataWrite(NRFX_USBD_ERRATA_171_REG, 0x00000000UL);
	}

	if (nrf52_errata_187())
	{
		UsbdErrataWrite(NRFX_USBD_ERRATA_187_REG, 0x00000000UL);
	}

	if (nrf52_errata_166())
	{
		NRFX_USBD_REG32(NRFX_USBD_ERRATA_166_REG_A) = 0x7E3UL;
		NRFX_USBD_REG32(NRFX_USBD_ERRATA_166_REG_B) = 0x40UL;
		__ISB();
		__DSB();
	}
}

static bool UsbdStartCtrlr(void)
{
	NRF_USBD->EVENTCAUSE = USBD_EVENTCAUSE_READY_Msk;
	__ISB();
	__DSB();

	UsbdErrataApply();

	NRF_USBD->ENABLE = 1;
	__ISB();
	__DSB();

	uint32_t i;

	for (i = 0; i < NRFX_USBD_READY_WAIT_LOOPS; i++)
	{
		if (NRF_USBD->EVENTCAUSE & USBD_EVENTCAUSE_READY_Msk)
		{
			break;
		}
	}

	if ((NRF_USBD->EVENTCAUSE & USBD_EVENTCAUSE_READY_Msk) == 0)
	{
		UsbdErrataRevert();
		NRF_USBD->ENABLE = 0;
		return false;
	}

	NRF_USBD->EVENTCAUSE = USBD_EVENTCAUSE_READY_Msk;
	__ISB();
	__DSB();

	UsbdErrataRevert();

	// The regulator reports itself usable separately from the controller, and
	// pulling up before it does gives the host a device that cannot answer.
	for (i = 0; i < NRFX_USBD_READY_WAIT_LOOPS; i++)
	{
		if (NRF_POWER->USBREGSTATUS & POWER_USBREGSTATUS_OUTPUTRDY_Msk)
		{
			break;
		}
	}

	if ((NRF_POWER->USBREGSTATUS & POWER_USBREGSTATUS_OUTPUTRDY_Msk) == 0)
	{
		NRF_USBD->ENABLE = 0;
		return false;
	}

	// The stock Nordic power-ready path arms USBRESET before the pull-up is
	// enabled. This port replaces that power handler, so do the same here. The
	// native nRF52 controller expands the mask after the first bus reset.
	NRF_USBD->EVENTS_USBRESET = 0;
	NRF_USBD->INTENCLR = NRF_USBD->INTEN;
	NRF_USBD->INTENSET = USBD_INTEN_USBRESET_Msk;
	NVIC_ClearPendingIRQ(USBD_IRQn);

	return true;
}

//
// Endpoint and DMA registers. Exactly one of these compiles.
//


static inline __attribute__((always_inline)) bool nRFUsbdDmaActive(void)
{
	return NRFX_USBD_EASYDMA_BUSY_REG ==
		NRFX_USBD_EASYDMA_BUSY_REG_BUSY;
}


static inline __attribute__((always_inline))
uint8_t nRFUsbdDir(uint8_t EpAddr)
{
	return USB_ENDPADDR_IS_IN(EpAddr) ? 1U : 0U;
}

// USBD interrupt bits index the event registers from EVENTS_USBRESET.
// Decode END once for endpoint open/close; EP0DATADONE separates regular
// IN from ISO IN in the event register bank.
static inline __attribute__((always_inline))
uint8_t nRFUsbdDmaEndBit(uint8_t EpNum, bool In)
{
	return In ? (EpNum == NRFX_USBD_ISO_EP_NO ?
		USBD_INTEN_ENDISOIN_Pos : USBD_INTEN_ENDEPIN0_Pos + EpNum) :
		USBD_INTEN_ENDEPOUT0_Pos + EpNum;
}

static inline __attribute__((always_inline))
volatile uint32_t *nRFUsbdDmaEndEvent(uint8_t EndBit)
{
	return (volatile uint32_t *)((uintptr_t)&NRF_USBD->EVENTS_USBRESET +
		EndBit * sizeof(uint32_t));
}

static_assert(offsetof(NRF_USBD_Type, EVENTS_ENDEPIN) -
	offsetof(NRF_USBD_Type, EVENTS_USBRESET) ==
	USBD_INTEN_ENDEPIN0_Pos * sizeof(uint32_t), "USBD IN event layout");
static_assert(offsetof(NRF_USBD_Type, EVENTS_ENDISOIN) -
	offsetof(NRF_USBD_Type, EVENTS_USBRESET) ==
	USBD_INTEN_ENDISOIN_Pos * sizeof(uint32_t), "USBD ISO IN event layout");
static_assert(offsetof(NRF_USBD_Type, EVENTS_ENDEPOUT) -
	offsetof(NRF_USBD_Type, EVENTS_USBRESET) ==
	USBD_INTEN_ENDEPOUT0_Pos * sizeof(uint32_t), "USBD OUT event layout");
static_assert(offsetof(NRF_USBD_Type, EVENTS_ENDISOOUT) -
	offsetof(NRF_USBD_Type, EVENTS_USBRESET) ==
	(USBD_INTEN_ENDEPOUT0_Pos + NRFX_USBD_ISO_EP_NO) * sizeof(uint32_t),
	"USBD ISO OUT event layout");

static inline __attribute__((always_inline))
void nRFUsbdEmit(const UsbCtrlrEvt_t *pEvt)
{
	UsbDevProcessEvent(0, pEvt);
}

static void nRFUsbdEmitSimple(UsbCtrlrEvtType_t Type)
{
	UsbCtrlrEvt_t evt = {};
	evt.Type = Type;
	nRFUsbdEmit(&evt);
}

static void nRFUsbdEmitXfer(uint8_t EpAddr, uint16_t Length,
						 UsbCtrlrXferResult_t Result)
{
	if (USB_ENDPADDR_NUM(EpAddr) != 0U)
	{
		nRFUsbEpRegisteredEvent(EpAddr, USB_CTRLR_EVT_XFER_CMPL, Length, Result);
		return;
	}

	UsbCtrlrEvt_t evt = {};
	evt.Type = USB_CTRLR_EVT_XFER_CMPL;
	evt.Xfer.EpAddr = EpAddr;
	evt.Xfer.Length = Length;
	evt.Xfer.Result = Result;
	nRFUsbdEmit(&evt);
}

static inline __attribute__((always_inline))
void nRFUsbdDmaUnlock(void)
{
	NRFX_USBD_EASYDMA_BUSY_REG = NRFX_USBD_EASYDMA_BUSY_REG_CLEAR;
	__DSB();
}

/** Start EasyDMA while the caller already excludes the USBD interrupt. */
static inline __attribute__((always_inline))
void nRFUsbdDmaStartLocked(volatile uint32_t *pTask,
	volatile uint32_t *pEnd)
{
	*pEnd = 0;
	__DSB();

	NRFX_USBD_EASYDMA_BUSY_REG = NRFX_USBD_EASYDMA_BUSY_REG_BUSY;
	*pTask = 1;
	__DSB();
}


/** Retire active DMA before a foreground stop or endpoint close. */
static void nRFUsbdDmaWait(void)
{
	while (nRFUsbdDmaActive())
	{
		if (NRF_USBD->EVENTS_USBRESET != 0U)
		{
			nRFUsbdDmaUnlock();
			return;
		}

		const uint32_t dmaStatus = NRF_USBD->EPSTATUS & 0x00FF00FFUL;
		const uint32_t primask = __get_PRIMASK();
		__disable_irq();
		if (dmaStatus != 0U)
		{
			const uint32_t statusBit = 31U - (uint32_t)__CLZ(dmaStatus);
			const uint8_t epNum = (uint8_t)(statusBit & 7U);
			volatile uint32_t *pEnd = statusBit >= 16U ?
				&NRF_USBD->EVENTS_ENDEPOUT[epNum] :
				&NRF_USBD->EVENTS_ENDEPIN[epNum];
			if (*pEnd != 0U)
			{
				*pEnd = 0U;
				NRF_USBD->EPSTATUS = 1UL << statusBit;
				__DSB();
				// Only EP0 retains its queue entry until DMA completion.
				if (epNum == 0U)
					(void)CFifoGet(s_hEp0Que);
				nRFUsbdDmaUnlock();
			}
		}
		else if (NRF_USBD->EVENTS_ENDISOIN != 0U)
		{
			(void)nRFUsbdFinishIsoDma(true);
		}
		else if (NRF_USBD->EVENTS_ENDISOOUT != 0U)
		{
			(void)nRFUsbdFinishIsoDma(false);
		}
		__set_PRIMASK(primask);
	}
}

static void nRFUsbdNoDmaTask(volatile uint32_t *pTask)
{
	*pTask = 1;
	__ISB();
	__DSB();
}

static bool nRFUsbdStartIsoNow(void)
{
	const uint_fast8_t isoOpen = atomic_load(&s_EventState.IsoOpen);
	nRFUsbdXfer_t *pXfer = &s_Ctrlr.Iso[1];
	volatile uint32_t *pTask;
	volatile uint32_t *pEnd;
	if ((isoOpen & NRFX_USBD_ISO_IN_OPEN) != 0U &&
		atomic_load(&s_EventState.IsoInReady) && pXfer->pBuffer != NULL)
	{
		atomic_store(&s_EventState.IsoInReady, false);
		NRF_USBD->ISOIN.PTR = (uint32_t)(uintptr_t)s_EpReg[NRFX_USBD_ISO_EP_NO][1].pBuffer;
		NRF_USBD->ISOIN.MAXCNT = pXfer->TotalLen;
		pTask = &NRF_USBD->TASKS_STARTISOIN;
		pEnd = &NRF_USBD->EVENTS_ENDISOIN;
	}
	else
	{
		pXfer = &s_Ctrlr.Iso[0];
		if ((isoOpen & NRFX_USBD_ISO_OUT_OPEN) == 0U ||
			!atomic_load(&s_EventState.IsoOutReady) || pXfer->pBuffer == NULL)
			return false;

		atomic_store(&s_EventState.IsoOutReady, false);
		const uint16_t len = s_EventState.IsoOutSize < pXfer->TotalLen ?
			s_EventState.IsoOutSize : pXfer->TotalLen;
		NRF_USBD->ISOOUT.PTR = (uint32_t)(uintptr_t)
			s_EpReg[NRFX_USBD_ISO_EP_NO][0].pBuffer;
		NRF_USBD->ISOOUT.MAXCNT = len;
		pTask = &NRF_USBD->TASKS_STARTISOOUT;
		pEnd = &NRF_USBD->EVENTS_ENDISOOUT;
	}

	pXfer->pBuffer = NULL;
	nRFUsbdDmaStartLocked(pTask, pEnd);
	return true;
}

static inline __attribute__((always_inline))
bool nRFUsbdIsoPending(void)
{
	return atomic_load(&s_EventState.IsoInReady) || atomic_load(&s_EventState.IsoOutReady);
}

static void nRFUsbdServiceIso(void)
{
	if (!nRFUsbdIsoPending())
	{
		return;
	}

	const uint32_t state = DisableInterrupt();
	if (!atomic_load(&s_EventState.HostResumePending) &&
		(!atomic_load(&s_EventState.BusSuspended) || atomic_load(&s_EventState.SuspendPending)) &&
		!nRFUsbdDmaActive())
	{
		(void)nRFUsbdStartIsoNow();
	}
	EnableInterrupt(state);
}

/**
 * Start EasyDMA for one queued directional request. Endpoint number and
 * direction stay separate in the scheduler; what an OUT endpoint actually
 * holds is only known now, so that is read here.
 */
static inline __attribute__((always_inline))
void nRFUsbdStartDmaNow(const nRFUsbdQue_t *pQue)
{
	const uint8_t epNum = pQue->EpNum;
	const bool isIn = pQue->Dir != 0U;
	const uint8_t *pBuffer = epNum == 0U ?
		(isIn ? ((const nRFEPPkt_t *)pQue)->Payload : s_Ep0Bounce) :
		s_EpReg[epNum][isIn ? 1 : 0].pBuffer;

	volatile uint32_t *pTask;
	volatile uint32_t *pEnd;
	if (isIn)
	{
		NRF_USBD->EPIN[epNum].PTR = (uint32_t)(uintptr_t)pBuffer;
		NRF_USBD->EPIN[epNum].MAXCNT = pQue->Len;
		pTask = &NRF_USBD->TASKS_STARTEPIN[epNum];
		pEnd = &NRF_USBD->EVENTS_ENDEPIN[epNum];
	}
	else
	{
		const uint16_t received = (uint16_t)NRF_USBD->SIZE.EPOUT[epNum];
		const uint16_t len = received < pQue->Len ? received : pQue->Len;

		NRF_USBD->EPOUT[epNum].PTR = (uint32_t)(uintptr_t)pBuffer;
		NRF_USBD->EPOUT[epNum].MAXCNT = len;
		pTask = &NRF_USBD->TASKS_STARTEPOUT[epNum];
		pEnd = &NRF_USBD->EVENTS_ENDEPOUT[epNum];
	}
	nRFUsbdDmaStartLocked(pTask, pEnd);
}

// Share the scheduler across ISR and foreground callers instead of expanding
// the DMA register setup at each call site.
static __attribute__((noinline)) void nRFUsbdStartQueuedDma(void)
{
	nRFUsbdQue_t *pQue = (nRFUsbdQue_t *)CFifoPeek(s_hEp0Que);
	if (pQue == NULL)
	{
		if (nRFUsbdStartIsoNow())
			return;
		pQue = (nRFUsbdQue_t *)CFifoGet(s_hQue);
	}

	if (pQue != NULL)
		nRFUsbdStartDmaNow(pQue);
}


// Keep the DMA/suspend gate shared by submission and completion paths.
static __attribute__((noinline)) void nRFUsbdResumeQueuedDmaLocked(void)
{
	if (nRFUsbdDmaActive() ||
		atomic_load(&s_EventState.HostResumePending) ||
		(atomic_load(&s_EventState.BusSuspended) &&
		 !atomic_load(&s_EventState.SuspendPending)))
	{
		return;
	}

	nRFUsbdStartQueuedDma();
}
static void nRFUsbdResumeQueuedDma(void)
{
	const uint32_t state = DisableInterrupt();
	nRFUsbdResumeQueuedDmaLocked();
	EnableInterrupt(state);
}

/**
 * Put one DMA request on the queue. Filling the block runs with interrupts
 * off because CFifoPut publishes the slot before the caller writes it, and
 * the interrupt is the other producer.
 */
static void nRFUsbdQueXferDir(uint8_t EpNum, bool In, uint16_t Len)
{
	const uint32_t state = DisableInterrupt();
	hCFifo_t hQue = EpNum == 0U ? s_hEp0Que : s_hQue;
	nRFUsbdQue_t *pQue = (nRFUsbdQue_t *)CFifoPut(hQue);

	pQue->EpNum = EpNum;
	pQue->Dir = In ? 1U : 0U;
	pQue->Len = Len;

	nRFUsbdResumeQueuedDmaLocked();

	EnableInterrupt(state);
}

/** Remove one endpoint number without disturbing the order of other work. */
static void nRFUsbdQueRemoveEp(uint8_t EpNum)
{
	const uint32_t state = DisableInterrupt();
	if (EpNum == 0U)
	{
		CFifoFlush(s_hEp0Que);
		EnableInterrupt(state);
		return;
	}

	const int count = CFifoUsed(s_hQue);

	// Rotate exactly the entries that were present on entry. Kept entries go
	// back at the tail in the same order. The queue has one slot per endpoint
	// direction, so each get guarantees space for its matching put.
	for (int i = 0; i < count; i++)
	{
		const nRFUsbdQue_t que =
			*(nRFUsbdQue_t *)CFifoGet(s_hQue);
		if (que.EpNum == EpNum)
		{
			continue;
		}

		*(nRFUsbdQue_t *)CFifoPut(s_hQue) = que;
	}

	EnableInterrupt(state);
}

static void nRFUsbdQueueEp0Out(void)
{
	nRFUsbdXfer_t *pXfer = &s_Ctrlr.Ep0[0];

	nRFUsbdQueXferDir(0U, false,
				 (uint16_t)(pXfer->TotalLen - pXfer->ActualLen));
}

static void nRFUsbdQueueEp0In(void)
{
	nRFUsbdXfer_t *pXfer = &s_Ctrlr.Ep0[1];
	const uint16_t remaining =
		(uint16_t)(pXfer->TotalLen - pXfer->ActualLen);
	const uint16_t mps = NRFX_USBD_MAX_PACKET_SIZE;
	const uint16_t length = remaining < mps ? remaining : mps;

	nRFUsbdQueXferDir(0U, true, length);
}

static void nRFUsbdResetState(void)
{
	memset(&s_Ctrlr, 0, sizeof(s_Ctrlr));

	CFifoFlush(s_hQue);
	CFifoFlush(s_hEp0Que);
	atomic_store(&s_EventState.BusSuspended, false);
	atomic_store(&s_EventState.SuspendPending, false);
	atomic_store(&s_EventState.RemoteWakePending, false);
	atomic_store(&s_EventState.HostResumePending, false);
	atomic_store(&s_EventState.MacAwake, true);
	atomic_store(&s_EventState.IsoOpen, 0U);
	atomic_store(&s_EventState.IsoInReady, false);
	atomic_store(&s_EventState.IsoOutReady, false);
	s_EventState.IsoOutSize = 0U;
	atomic_store(&s_EventState.IsoBusy, 0U);
	atomic_store(&s_EventState.IsoComplete, 0U);
	++s_EventState.IsoGeneration[0];
	++s_EventState.IsoGeneration[1];
	NRFX_USBD_EASYDMA_BUSY_REG = NRFX_USBD_EASYDMA_BUSY_REG_CLEAR;
}

static void nRFUsbdAbortEp0(void)
{
	nRFUsbdQueRemoveEp(0U);

	for (uint8_t dir = 0; dir < 2U; dir++)
	{
		nRFUsbdXfer_t *pXfer = &s_Ctrlr.Ep0[dir];
		pXfer->pBuffer = NULL;
		pXfer->TotalLen = 0;
		pXfer->ActualLen = 0;
	}

	NRF_USBD->EVENTS_ENDEPIN[0] = 0;
	NRF_USBD->EVENTS_ENDEPOUT[0] = 0;
	NRF_USBD->EPDATASTATUS = (1UL << 0) | (1UL << 16);
	__ISB();
	__DSB();
}

static void nRFUsbdTryEnterLowPower(void)
{
	if (!s_UsbdLowPowerSuspend ||
		!atomic_load(&s_EventState.BusSuspended) ||
		!atomic_load(&s_EventState.SuspendPending))
	{
		return;
	}

	// Check ownership and wake state once, with interrupts excluded.

	const uint32_t irqState = DisableInterrupt();
	if (!atomic_load(&s_EventState.BusSuspended) ||
		!atomic_load(&s_EventState.SuspendPending) ||
		atomic_load(&s_EventState.RemoteWakePending) ||
		atomic_load(&s_EventState.HostResumePending) ||
		nRFUsbdDmaActive() ||
		CFifoUsed(s_hQue) > 0)
	{
		EnableInterrupt(irqState);
		return;
	}

	if ((NRF_USBD->EVENTCAUSE & USBD_EVENTCAUSE_RESUME_Msk) != 0U ||
		NRF_USBD->EVENTS_SOF != 0U)
	{
		EnableInterrupt(irqState);
		nRFUsbdHostResumeDetected();
		return;
	}

	atomic_store(&s_EventState.MacAwake, false);
	NRF_USBD->LOWPOWER =
		USBD_LOWPOWER_LOWPOWER_LowPower << USBD_LOWPOWER_LOWPOWER_Pos;
	(void)NRF_USBD->LOWPOWER;
	__ISB();
	__DSB();

	if ((NRF_USBD->EVENTCAUSE & USBD_EVENTCAUSE_RESUME_Msk) != 0U ||
		NRF_USBD->EVENTS_SOF != 0U)
	{
		EnableInterrupt(irqState);
		nRFUsbdHostResumeDetected();
		return;
	}

	// No software wake state changes while interrupts remain excluded.
	// Hardware resume is handled by the RESUME/SOF check above.
	atomic_store(&s_EventState.SuspendPending, false);
	EnableInterrupt(irqState);
}

static void nRFUsbdTryRemoteWake(void)
{
	if (!atomic_load(&s_EventState.RemoteWakePending))
	{
		return;
	}

	// Validate the wake request once while interrupts are excluded.
	const uint32_t irqState = DisableInterrupt();
	if (!atomic_load(&s_EventState.RemoteWakePending) ||
		!atomic_load(&s_EventState.BusSuspended) ||
		atomic_load(&s_EventState.HostResumePending) ||
		!atomic_load(&s_EventState.MacAwake) ||
		nRFUsbdDmaActive() ||
		NRF_USBD->LOWPOWER !=
			(USBD_LOWPOWER_LOWPOWER_ForceNormal << USBD_LOWPOWER_LOWPOWER_Pos))
	{
		EnableInterrupt(irqState);
		return;
	}

	atomic_store(&s_EventState.RemoteWakePending, false);
	NRF_USBD->DPDMVALUE = USBD_DPDMVALUE_STATE_Resume;
	NRF_USBD->TASKS_DPDMDRIVE = 1;
	__ISB();
	__DSB();
	EnableInterrupt(irqState);

	if ((NRF_USBD->INTEN & USBD_INTEN_SOF_Msk) == 0U)
	{
		NRF_USBD->EVENTS_SOF = 0;
	}
	NRF_USBD->INTENSET = USBD_INTENSET_SOF_Msk;
}

static void nRFUsbdHostResumeDetected(void)
{
	if (!atomic_load(&s_EventState.BusSuspended))
	{
		return;
	}

	atomic_store(&s_EventState.BusSuspended, false);
	atomic_store(&s_EventState.SuspendPending, false);
	atomic_store(&s_EventState.RemoteWakePending, false);

	if (!atomic_load(&s_EventState.MacAwake) ||
		NRF_USBD->LOWPOWER !=
			(USBD_LOWPOWER_LOWPOWER_ForceNormal << USBD_LOWPOWER_LOWPOWER_Pos))
	{
		atomic_store(&s_EventState.HostResumePending, true);
		if (NRF_USBD->LOWPOWER !=
			(USBD_LOWPOWER_LOWPOWER_ForceNormal << USBD_LOWPOWER_LOWPOWER_Pos))
		{
			NRF_USBD->LOWPOWER =
				USBD_LOWPOWER_LOWPOWER_ForceNormal << USBD_LOWPOWER_LOWPOWER_Pos;
			__ISB();
			__DSB();
		}
		return;
	}

	atomic_store(&s_EventState.HostResumePending, false);
	nRFUsbdEmitSimple(USB_CTRLR_EVT_RESUME);
}

static void nRFUsbdWakeAllowed(void)
{
	atomic_store(&s_EventState.MacAwake, true);

	if (atomic_load(&s_EventState.HostResumePending))
	{
		atomic_store(&s_EventState.HostResumePending, false);
		nRFUsbdEmitSimple(USB_CTRLR_EVT_RESUME);
		return;
	}

	nRFUsbdTryRemoteWake();
}

static bool nRFUsbRegIsoXfer(uint8_t EpAddr, uint16_t Length)
{
	const uint8_t dir = nRFUsbdDir(EpAddr);
	const uint_fast8_t bit = 1U << dir;
	nRFUsbEpReg_t *pReg = nRFUsbGetEpReg(EpAddr);
	const uint32_t state = DisableInterrupt();
	if ((atomic_load(&s_EventState.IsoOpen) & bit) == 0U ||
		(atomic_load(&s_EventState.IsoBusy) & bit) != 0U ||
		pReg->pBuffer == NULL || pReg->Handler == NULL ||
		Length > pReg->Mps)
	{
		EnableInterrupt(state);
		return false;
	}

	nRFUsbdXfer_t *pXfer = &s_Ctrlr.Iso[dir];
	pXfer->pBuffer = pReg->pBuffer;
	pXfer->TotalLen = Length;
	pXfer->ActualLen = 0U;
	atomic_fetch_or(&s_EventState.IsoBusy, bit);
	nRFUsbdServiceIso();
	EnableInterrupt(state);
	return true;
}

static void nRFUsbdBusReset(void)
{
	if (nRFUsbdDmaActive())
	{
		nRFUsbdDmaUnlock();
	}

	NRF_USBD->EPOUTEN = 1UL;
	NRF_USBD->EPINEN = 1UL;

	for (uint8_t epNum = 0; epNum < NRFX_USBD_DATA_EP_COUNT; epNum++)
	{
		NRF_USBD->TASKS_STARTEPIN[epNum] = 0;
		NRF_USBD->TASKS_STARTEPOUT[epNum] = 0;
	}
	NRF_USBD->TASKS_STARTISOIN = 0;
	NRF_USBD->TASKS_STARTISOOUT = 0;

	const uint32_t epStatus = NRF_USBD->EPSTATUS;
	NRF_USBD->EPSTATUS = epStatus;
	const uint32_t dataStatus = NRF_USBD->EPDATASTATUS;
	NRF_USBD->EPDATASTATUS = dataStatus;

	NRF_USBD->EVENTS_USBEVENT = 0;
	const uint32_t cause = NRF_USBD->EVENTCAUSE;
	NRF_USBD->EVENTCAUSE = cause;

	NRF_USBD->INTENCLR = NRF_USBD->INTEN;
	NRF_USBD->INTENSET =
		USBD_INTEN_USBRESET_Msk |
		USBD_INTEN_USBEVENT_Msk |
		USBD_INTEN_EPDATA_Msk |
		USBD_INTEN_EP0SETUP_Msk |
		USBD_INTEN_EP0DATADONE_Msk |
		USBD_INTEN_ENDEPOUT0_Msk;

	nRFUsbdResetState();
}

static void nRFUsbdHandleEp0OutEnd(uint16_t TransferLen)
{
	nRFUsbdXfer_t *pXfer = &s_Ctrlr.Ep0[0];

	if (pXfer->pBuffer != NULL)
	{
		if (TransferLen > 0U)
		{
			memcpy(pXfer->pBuffer, s_Ep0Bounce, TransferLen);
		}
		pXfer->pBuffer += TransferLen;
	}
	pXfer->ActualLen += TransferLen;

	if (TransferLen == NRFX_USBD_MAX_PACKET_SIZE &&
		pXfer->ActualLen < pXfer->TotalLen)
	{
		nRFUsbdNoDmaTask(&NRF_USBD->TASKS_EP0RCVOUT);
	}
	else
	{
		nRFUsbdEmitXfer(0U, pXfer->ActualLen, USB_CTRLR_XFER_SUCCESS);
	}
}

static void nRFUsbdHandleEp0OutData(void)
{
	nRFUsbdXfer_t *pXfer = &s_Ctrlr.Ep0[0];
	if (pXfer->ActualLen < pXfer->TotalLen || pXfer->TotalLen == 0U)
	{
		nRFUsbdQueueEp0Out();
	}
}

static void nRFUsbdProcessIsoComplete(uint32_t Evt, void *pContext)
{
	(void)pContext;
	const uint8_t dir = (uint8_t)(Evt & 1U);
	const uint_fast8_t bit = 1U << dir;
	const uint32_t generation = Evt >> 1U;
	uint32_t state = DisableInterrupt();
	if (generation != (s_EventState.IsoGeneration[dir] & 0x7FFFFFFFUL) ||
		(atomic_load(&s_EventState.IsoOpen) & atomic_load(&s_EventState.IsoBusy) & bit) == 0U)
	{
		EnableInterrupt(state);
		return;
	}

	const uint8_t epAddr = dir != 0U ?
		USB_ENDPADDR_DIRIN(NRFX_USBD_ISO_EP_NO) : NRFX_USBD_ISO_EP_NO;
	const uint16_t amount = s_Ctrlr.Iso[dir].ActualLen;
	// IN DMA has copied the TX payload into controller memory. Release it
	// before calling the handler so that handler may submit the next frame.
	if (dir != 0U)
		atomic_fetch_and(&s_EventState.IsoBusy, (uint_fast8_t)~bit);
	EnableInterrupt(state);

	nRFUsbEpRegisteredEvent(epAddr, USB_CTRLR_EVT_XFER_CMPL,
		amount, USB_CTRLR_XFER_SUCCESS);

	// OUT DMA memory is still the callback's source until it returns.
	state = DisableInterrupt();
	if (dir == 0U && generation == (s_EventState.IsoGeneration[dir] & 0x7FFFFFFFUL))
		atomic_fetch_and(&s_EventState.IsoBusy, (uint_fast8_t)~bit);
	nRFUsbdResumeQueuedDmaLocked();
	EnableInterrupt(state);
}

// A full AppEvt queue retains the completion and buffer ownership. The idle
// hook retries publication after foreground dispatch makes room.
static void nRFUsbdRetryIsoComplete(void)
{
	if (atomic_load(&s_EventState.IsoComplete) == 0U)
		return;
	const uint32_t state = DisableInterrupt();
	for (uint8_t dir = 0U; dir < 2U; ++dir)
	{
		const uint_fast8_t bit = 1U << dir;
		if ((atomic_load(&s_EventState.IsoComplete) & bit) != 0U &&
			AppEvtHandlerQue((s_EventState.IsoGeneration[dir] << 1U) | dir,
				NULL, nRFUsbdProcessIsoComplete))
			atomic_fetch_and(&s_EventState.IsoComplete, (uint_fast8_t)~bit);
	}
	EnableInterrupt(state);
}

static bool nRFUsbdFinishIsoDma(bool In)
{
	volatile uint32_t *pEnd = In ?
		&NRF_USBD->EVENTS_ENDISOIN : &NRF_USBD->EVENTS_ENDISOOUT;
	if (*pEnd == 0U)
		return false;

	const uint8_t dir = In ? 1U : 0U;
	s_Ctrlr.Iso[dir].ActualLen = (uint16_t)(In ?
		NRF_USBD->ISOIN.AMOUNT : NRF_USBD->ISOOUT.AMOUNT);
	*pEnd = 0U;
	NRF_USBD->EPSTATUS = In ? (1UL << 8U) : (1UL << 24U);
	__DSB();
	nRFUsbdDmaUnlock();
	atomic_fetch_or(&s_EventState.IsoComplete, (uint_fast8_t)(1U << dir));
	nRFUsbdRetryIsoComplete();
	return true;
}

static void nRFUsbdProcessEp0Complete(uint32_t Evt, void *pContext)
{
	const uint16_t amount = (uint16_t)(Evt >> 8U);

	(void)pContext;

	if ((Evt & NRFX_USBD_XFER_EVT_OUT) != 0U)
	{
		nRFUsbdHandleEp0OutEnd(amount);
	}
	else
	{
		nRFUsbdEmitXfer(USB_ENDPADDR_DIR_IN, amount,
			USB_CTRLR_XFER_SUCCESS);
	}

	nRFUsbdResumeQueuedDma();
}

static void nRFUsbdProcessOutComplete(uint32_t Evt, void *pContext)
{
	const uint8_t epNum = (uint8_t)Evt;
	const uint16_t amount = (uint16_t)(Evt >> 8U);

	(void)pContext;

	// UsbIntrf packetizes regular endpoints. One queued request is one DMA
	// transaction, so only EP0 needs multi-packet completion processing.
	nRFUsbEpRegisteredEvent(epNum, USB_CTRLR_EVT_XFER_CMPL, amount,
		USB_CTRLR_XFER_SUCCESS);
}

static void nRFUsbdProcessInComplete(uint32_t Evt, void *pContext)
{
	const uint8_t epNum = (uint8_t)Evt;
	const uint16_t amount = (uint16_t)(Evt >> 8U);

	(void)pContext;

	nRFUsbEpRegisteredEvent((uint8_t)(epNum | USB_ENDPADDR_DIR_IN),
		USB_CTRLR_EVT_XFER_CMPL, amount, USB_CTRLR_XFER_SUCCESS);
}

static void nRFUsbdProcessEp0OutData(uint32_t Evt, void *pContext)
{
	(void)Evt;
	(void)pContext;

	nRFUsbdHandleEp0OutData();
}

static void nRFUsbdProcessOutData(uint32_t Evt, void *pContext)
{
	const uint8_t epNum = (uint8_t)Evt;

	(void)pContext;

	// Only blocking endpoints reach AppEvt. Nonblocking OUT is queued for
	// EasyDMA directly from EPDATASTATUS in the ISR.
	nRFUsbEpRegisteredEvent(epNum, USB_CTRLR_EVT_DRDY, 0U,
		USB_CTRLR_XFER_SUCCESS);
}

static void nRFUsbdQueueEp0Complete(bool Out, uint16_t Amount)
{
	const uint32_t evt = ((uint32_t)Amount << 8U) |
		(Out ? NRFX_USBD_XFER_EVT_OUT : 0U);
	(void)AppEvtHandlerQue(evt, NULL, nRFUsbdProcessEp0Complete);
}

static inline __attribute__((always_inline))
void nRFUsbdQueueOutComplete(uint8_t EpNum, uint16_t Amount)
{
	const uint32_t evt = ((uint32_t)Amount << 8U) | EpNum;
	(void)AppEvtHandlerQue(evt, NULL, nRFUsbdProcessOutComplete);
}

static inline __attribute__((always_inline))
void nRFUsbdQueueInComplete(uint8_t EpNum, uint16_t Amount)
{
	const uint32_t evt = ((uint32_t)Amount << 8U) | EpNum;
	(void)AppEvtHandlerQue(evt, NULL, nRFUsbdProcessInComplete);
}

static void nRFUsbdHandleBusEvent(uint32_t EventCause)
{
	if ((EventCause & USBD_EVENTCAUSE_SUSPEND_Msk) != 0U &&
		!atomic_load(&s_EventState.BusSuspended))
	{
		atomic_store(&s_EventState.BusSuspended, true);
		// A bus suspend and a peripheral low-power transition are separate.
		// When low-power suspend is disabled, retain all endpoint state and
		// wait for RESUME or SOF without touching USBD LOWPOWER.
		atomic_store(&s_EventState.SuspendPending, s_UsbdLowPowerSuspend);
		atomic_store(&s_EventState.RemoteWakePending, false);
		atomic_store(&s_EventState.HostResumePending, false);
		atomic_store(&s_EventState.IsoInReady, false);
		atomic_store(&s_EventState.IsoOutReady, false);
		NRF_USBD->EVENTS_SOF = 0U;
		NRF_USBD->INTENSET = USBD_INTENSET_SOF_Msk;
		nRFUsbdEmitSimple(USB_CTRLR_EVT_SUSPEND);
	}

	if ((EventCause & USBD_EVENTCAUSE_RESUME_Msk) != 0)
	{
		nRFUsbdHostResumeDetected();
	}

	if ((EventCause & USBD_EVENTCAUSE_USBWUALLOWED_Msk) != 0)
	{
		nRFUsbdWakeAllowed();
	}
}

static void nRFUsbdHandleSof(void)
{
	nRFUsbdHostResumeDetected();

	const uint_fast8_t isoOpen = atomic_load(&s_EventState.IsoOpen);
	if ((isoOpen & NRFX_USBD_ISO_IN_OPEN) != 0U)
	{
		atomic_store(&s_EventState.IsoInReady, true);
	}

	if ((isoOpen & NRFX_USBD_ISO_OUT_OPEN) != 0U)
	{
		const uint32_t size = NRF_USBD->SIZE.ISOOUT;
		// ISO OUT is a current-frame slot, not a backlog. Keep the DMA
		// buffer stable until its deferred completion callback returns.
		nRFUsbdXfer_t *pOut = &s_Ctrlr.Iso[0];
		const bool waiting = pOut->pBuffer != NULL;
		if ((atomic_load(&s_EventState.IsoBusy) & NRFX_USBD_ISO_OUT_OPEN) == 0U || waiting)
		{
			atomic_store(&s_EventState.IsoOutReady, size != 0U);
			if (size != 0U)
			{
				s_EventState.IsoOutSize = (size & USBD_SIZE_ISOOUT_ZERO_Msk) != 0U ?
					0U : (uint16_t)size;
				if (!waiting)
				{
					nRFUsbEpReg_t *pReg = nRFUsbGetEpReg(NRFX_USBD_ISO_EP_NO);
					if (pReg->bBlocking)
					{
						nRFUsbEpRegisteredEvent(NRFX_USBD_ISO_EP_NO,
							USB_CTRLR_EVT_DRDY, 0U, USB_CTRLR_XFER_SUCCESS);
					}
					else
					{
						(void)nRFUsbRegIsoXfer(NRFX_USBD_ISO_EP_NO, pReg->Mps);
					}
				}
			}
		}
	}

	if (s_Ctrlr.SofEnabled)
	{
		UsbCtrlrEvt_t evt = {};
		evt.Type = USB_CTRLR_EVT_SOF;
		evt.FrameNo = (uint16_t)NRF_USBD->FRAMECNTR;
		nRFUsbdEmit(&evt);
	}

	if (!s_Ctrlr.SofEnabled && isoOpen == 0U &&
		!atomic_load(&s_EventState.BusSuspended))
	{
		NRF_USBD->INTENCLR = USBD_INTENCLR_SOF_Msk;
	}

	// ISO is bound to this service interval. Start it here when EasyDMA is
	// free; otherwise the current DMA completion retries it before bulk work.
	nRFUsbdServiceIso();
}

static void nRFUsbdProcessEP0Setup(uint32_t Evt, void *pContext)
{
	(void)Evt;
	(void)pContext;

	UsbCtrlrEvt_t evt = {};
	evt.Type = USB_CTRLR_EVT_SETUP;
	evt.Setup.bmRequestType = (uint8_t)NRF_USBD->BMREQUESTTYPE;
	evt.Setup.bRequest = (uint8_t)NRF_USBD->BREQUEST;
	evt.Setup.wValue = (uint16_t)NRF_USBD->WVALUEL |
		((uint16_t)NRF_USBD->WVALUEH << 8);
	evt.Setup.wIndex = (uint16_t)NRF_USBD->WINDEXL |
		((uint16_t)NRF_USBD->WINDEXH << 8);
	evt.Setup.wLength = (uint16_t)NRF_USBD->WLENGTHL |
		((uint16_t)NRF_USBD->WLENGTHH << 8);

	while (nRFUsbdDmaActive())
	{
	}

	nRFUsbdHostResumeDetected();
	nRFUsbdAbortEp0();

	const bool setAddress =
		(evt.Setup.bmRequestType &
		 (USB_REQTYPE_MASK_RECEIPT | USB_REQTYPE_MASK_TYPE)) == 0U &&
		evt.Setup.bRequest == USB_REQ_SET_ADDRESS;

	if (setAddress)
	{
		UsbCtrlrEvt_t addrEvt = {};
		addrEvt.Type = USB_CTRLR_EVT_ADDRESS;
		addrEvt.Address = (uint8_t)(evt.Setup.wValue & 0x7FU);
		nRFUsbdEmit(&addrEvt);
	}
	else
	{
		nRFUsbdEmit(&evt);
	}
}


extern "C" void USBD_IRQHandler(void)
{
	uint32_t dmastatus = NRF_USBD->EPSTATUS;
	uint8_t outEp = 0U;

	// Reset cancels any active DMA and must not wait for ENDEP.
	if (NRF_USBD->EVENTS_USBRESET != 0U)
	{
		NRF_USBD->EVENTS_USBRESET = 0U;
		__ISB();
		__DSB();
		nRFUsbdBusReset();
		nRFUsbdEmitSimple(USB_CTRLR_EVT_RESET);
		return;
	}

	// Exactly one endpoint can own EasyDMA.
	// Handle DMA transfer complete
	switch (dmastatus)
	{
		case 0x00000001U: // EP0 IN
			if (NRF_USBD->EVENTS_EP0DATADONE != 0U)
			{
				NRF_USBD->EVENTS_EP0DATADONE = 0U;
				NRF_USBD->EVENTS_ENDEPIN[0] = 0U;
				NRF_USBD->EPSTATUS = dmastatus;
				__DSB();

				if (NRF_USBD->EVENTS_EP0SETUP != 0U)
				{
					nRFUsbdDmaUnlock();
					break;
				}

				(void)CFifoGet(s_hEp0Que);
				nRFEPPkt_t *p =
					(nRFEPPkt_t *)CFifoPeek(s_hEp0Que);
				if (p != NULL)
				{
					NRF_USBD->EPIN[0].PTR =
						(uint32_t)(uintptr_t)p->Payload;
					NRF_USBD->EPIN[0].MAXCNT = p->Hdr.Len;
					NRF_USBD->SHORTS =
						p->Hdr.Len < NRFX_USBD_MAX_PACKET_SIZE ?
						USBD_SHORTS_EP0DATADONE_EP0STATUS_Msk : 0U;
					nRFUsbdDmaStartLocked(
						&NRF_USBD->TASKS_STARTEPIN[0],
						&NRF_USBD->EVENTS_ENDEPIN[0]);
				}
				else
				{
					nRFUsbdDmaUnlock();
					nRFUsbdQueueEp0Complete(false,
						s_Ctrlr.Ep0[1].TotalLen);
				}
			}
			break;

		case 0U:
			break;

		case 0x00010000U: // EP0 OUT
			if (NRF_USBD->EVENTS_ENDEPOUT[0] != 0U)
			{
				NRF_USBD->EVENTS_ENDEPOUT[0] = 0U;
				NRF_USBD->EPSTATUS = dmastatus;
				__DSB();

				nRFUsbdDmaUnlock();
				nRFUsbdQueueEp0Complete(true,
					(uint16_t)NRF_USBD->EPOUT[0].AMOUNT);
			}
			break;
		case 0x00000100U: // ISO IN
			(void)nRFUsbdFinishIsoDma(true);
			break;
		case 0x01000000U: // ISO OUT
			(void)nRFUsbdFinishIsoDma(false);
			break;
		default:          // EP1-7 IN/OUT
		{
			// EP0 and ISO were separated above. Retire the one regular DMA
			// directly; IN application completion still waits for EPDATA.
			const uint32_t statusBit = 31U - (uint32_t)__CLZ(dmastatus);
			const uint8_t epNum = (uint8_t)(statusBit & 7U);
			const bool out = statusBit >= 16U;
			volatile uint32_t *pEnd = out ?
				&NRF_USBD->EVENTS_ENDEPOUT[epNum] :
				&NRF_USBD->EVENTS_ENDEPIN[epNum];
			if (*pEnd == 0U)
				return;

			*pEnd = 0U;
			NRF_USBD->EPSTATUS = 1UL << statusBit;
			__DSB();
			nRFUsbdDmaUnlock();

			if (out)
			{
				//nRFUsbdQueueOutComplete(epNum, (uint16_t)NRF_USBD->EPOUT[epNum].AMOUNT);
				uint32_t evt = epNum | (NRF_USBD->EPOUT[epNum].AMOUNT << 8U);
				nRFUsbdProcessOutComplete(evt, nullptr);
			}

			if (NRF_USBD->EVENTS_EP0SETUP == 0U &&
				NRF_USBD->EVENTS_USBEVENT == 0U)
				nRFUsbdStartQueuedDma();
			break;
		}
	}

	if (NRF_USBD->EVENTS_STARTED != 0U)
	{
		NRF_USBD->EVENTS_STARTED = 0U;
	}

	if (NRF_USBD->EVENTS_USBEVENT != 0U)
	{
		NRF_USBD->EVENTS_USBEVENT = 0U;
		const uint32_t eventCause = NRF_USBD->EVENTCAUSE;
		NRF_USBD->EVENTCAUSE = eventCause;
		__ISB();
		__DSB();

		nRFUsbdHandleBusEvent(eventCause);
	}

	if (NRF_USBD->EVENTS_EP0SETUP != 0U)
	{
		NRF_USBD->EVENTS_EP0SETUP = 0U;

		// A new SETUP aborts the previous control transfer. Discard any
		// simultaneously latched completion from that old transfer.
		NRF_USBD->EVENTS_EP0DATADONE = 0U;
		__ISB();
		__DSB();

		(void)AppEvtHandlerQue(0U, NULL, nRFUsbdProcessEP0Setup);

		return;
	}
#if 0
	if (NRF_USBD->EVENTS_EP0DATADONE != 0U)
	{
		NRF_USBD->EVENTS_EP0DATADONE = 0U;
		__ISB();
		__DSB();
		(void)AppEvtHandlerQue(0U, NULL, nRFUsbdProcessEp0OutData);
	}
	else
#endif
		if (NRF_USBD->EVENTS_EPDATA != 0U ||
		(NRF_USBD->EPDATASTATUS & 0x00FE00FEUL) != 0U)
	{
		// Clear the event first so a new endpoint event remains observable.
		// Service at most one endpoint per direction in this interrupt. Any
		// remaining status bits are retained and serviced by a pending IRQ.
		NRF_USBD->EVENTS_EPDATA = 0U;
		const uint32_t dataStatus = NRF_USBD->EPDATASTATUS;
		uint32_t servicedStatus = dataStatus & 0x00010001UL;

		const uint32_t outData = (dataStatus >> 16U) & 0xFEU;
		if (outData != 0U)
		{
			outEp = (uint8_t)(31U - (uint32_t)__CLZ(outData));
			servicedStatus |= 1UL << (outEp + 16U);

		}

		const uint32_t inData = dataStatus & 0xFEU;
		if (inData != 0U)
		{
			const uint32_t epNum = 31U - (uint32_t)__CLZ(inData);
			servicedStatus |= 1UL << epNum;
			nRFUsbdQueueInComplete((uint8_t)epNum,
				(uint16_t)NRF_USBD->EPIN[epNum].AMOUNT);
		}

		NRF_USBD->EPDATASTATUS = servicedStatus;
		__ISB();
		__DSB();

		if ((dataStatus & 0x00FE00FEUL & ~servicedStatus) != 0U)
		{
//			NVIC_SetPendingIRQ(USBD_IRQn);
		}
	}


	if (NRF_USBD->EVENTS_SOF != 0U)
	{
		NRF_USBD->EVENTS_SOF = 0U;
		__ISB();
		__DSB();

		nRFUsbdHandleSof();
	}

	nRFUsbdTryRemoteWake();

	// ENDEP released the shared EasyDMA channel above. ISO has priority when
	// it is open; ordinary CDC traffic avoids the ISO service path entirely.
	if (outEp != 0U)
	{
		const uint8_t epNum = outEp;
		nRFUsbEpReg_t *pReg = &s_EpReg[epNum][0];
		if (pReg->bBlocking)
		{
			// EPDATASTATUS is cleared before DRDY may start another DMA.
			// UsbIntrf checks RX space and sets RxPending when it is full.
			nRFUsbEpRegisteredEvent(epNum, USB_CTRLR_EVT_DRDY, 0U,
				USB_CTRLR_XFER_SUCCESS);
		}
		else
		{
			nRFUsbdQue_t *pQue = (nRFUsbdQue_t *)CFifoPut(s_hQue);
			if (pQue != NULL)
			{
				pQue->EpNum = epNum;
				pQue->Dir = 0U;
				pQue->Len = pReg->Mps;
			}
			else
			{
				(void)AppEvtHandlerQue(epNum, NULL, nRFUsbdProcessOutData);
			}
		}
		nRFUsbdResumeQueuedDmaLocked();
	}

	if ((dmastatus & 0x01000100UL) != 0U && atomic_load(&s_EventState.IsoOpen) != 0U)
		nRFUsbdResumeQueuedDmaLocked();

	nRFUsbdTryEnterLowPower();

}

// Public controller API.

bool UsbCtrlrInit(int DevNo, const UsbCtrlrCfg_t *pCfg)
{
	if (DevNo != 0 || pCfg == NULL)
	{
		return false;
	}

	memset(s_EpReg, 0, sizeof(s_EpReg));

	s_UsbdIntPrio = pCfg->IntPrio;
	s_UsbdLowPowerSuspend = pCfg->bLowPowerSuspend;


	s_UsbdInitialized = true;
	s_UsbdStarted = false;

	s_hQue = CFifoInit(s_QueMem, sizeof(s_QueMem), sizeof(nRFUsbdQue_t),
					   false);
	s_hEp0Que = CFifoInit(s_Ep0QueMem, sizeof(s_Ep0QueMem),
						  sizeof(nRFEPPkt_t), true);
	if (s_hQue == NULL || s_hEp0Que == NULL)
	{
		return false;
	}

	nRFUsbdResetState();
	return true;
}

bool UsbCtrlrStart(int DevNo)
{
	if (DevNo != 0)
	{
		return false;
	}

	if (s_UsbdInitialized == false)
	{
		return false;
	}

	if (s_UsbdStarted)
	{
		return true;
	}

	if (UsbCtrlrVbusDetected(DevNo) == false)
	{
		// No cable. Not a failure: the poll in UsbdProcess reports the attach
		// and the caller comes back.
		return false;
	}

	if (UsbdXtalRequest() == false)
	{
		return false;
	}


	NVIC_SetPriority(USBD_IRQn, s_UsbdIntPrio);

	if (UsbdStartCtrlr() == false)
	{
		UsbdXtalRelease();
		return false;
	}

	s_UsbdStarted = true;

	return true;
}

void UsbCtrlrStop(int DevNo)
{
	if (DevNo != 0)
	{
		return;
	}

	nRFUsbdDmaWait();
	nRFUsbdResetState();

	if (s_UsbdStarted == false)
	{
		return;
	}

	// Stop the controller interrupt before powering down the wrapper.
	NVIC_DisableIRQ(USBD_IRQn);

	NRF_USBD->INTEN = 0;
	NRF_USBD->USBPULLUP = 0;
	NRF_USBD->ENABLE = 0;
	__ISB();
	__DSB();

	// A successful start owns one clock request; a failed start releases it.
	UsbdXtalRelease();

	s_UsbdStarted = false;
}

// Suspend and wake are owned by the USBEVENT/SOF handlers. With low-power
// suspend disabled, this driver never enters peripheral low-power mode.
void UsbCtrlrProcess(int DevNo)
{
	if (DevNo == 0)
		AppEvtHandlerExec();
}

bool UsbCtrlrVbusDetected(int DevNo)
{
	return DevNo == 0 &&
		(NRF_POWER->USBREGSTATUS & POWER_USBREGSTATUS_VBUSDETECT_Msk) != 0;
}

bool UsbCtrlrHighSpeed(int DevNo)
{
	(void)DevNo;
	return false;
}

size_t UsbCtrlrGetSerial(int DevNo, char *pBuff, size_t BuffLen)
{
	if (DevNo != 0)
	{
		return 0;
	}

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

void UsbCtrlrIntEnable(int DevNo)
{
	if (DevNo != 0)
	{
		return;
	}

	NVIC_EnableIRQ(USBD_IRQn);
}

void UsbCtrlrIntDisable(int DevNo)
{
	if (DevNo != 0)
	{
		return;
	}

	NVIC_DisableIRQ(USBD_IRQn);
}

void UsbCtrlrConnect(int DevNo)
{
	if (DevNo != 0)
	{
		return;
	}

	NRF_USBD->USBPULLUP = 1;
}

void UsbCtrlrDisconnect(int DevNo)
{
	if (DevNo != 0)
	{
		return;
	}

	NRF_USBD->USBPULLUP = 0;
}

void UsbCtrlrRemoteWakeup(int DevNo)
{
	if (DevNo != 0)
	{
		return;
	}

	if (!atomic_load(&s_EventState.BusSuspended) || atomic_load(&s_EventState.HostResumePending))
	{
		return;
	}

	atomic_store(&s_EventState.SuspendPending, false);
	atomic_store(&s_EventState.RemoteWakePending, true);

	if (NRF_USBD->LOWPOWER !=
		(USBD_LOWPOWER_LOWPOWER_ForceNormal << USBD_LOWPOWER_LOWPOWER_Pos))
	{
		NRF_USBD->LOWPOWER =
			USBD_LOWPOWER_LOWPOWER_ForceNormal << USBD_LOWPOWER_LOWPOWER_Pos;
		__ISB();
		__DSB();
	}

	nRFUsbdTryRemoteWake();
}

void UsbCtrlrSofEnable(int DevNo, bool Enable)
{
	if (DevNo != 0)
	{
		return;
	}

	s_Ctrlr.SofEnabled = Enable;

	if (Enable)
	{
		NRF_USBD->EVENTS_SOF = 0;
		NRF_USBD->INTENSET = USBD_INTENSET_SOF_Msk;
	}
	else
	{
		if (atomic_load(&s_EventState.IsoOpen) == 0U &&
			!atomic_load(&s_EventState.BusSuspended))
		{
			NRF_USBD->INTENCLR = USBD_INTENCLR_SOF_Msk;
		}
	}
}

void UsbCtrlrSetAddress(int DevNo, uint8_t Address)
{
	// USBD applies SET_ADDRESS in hardware.
	(void)DevNo;
	(void)Address;
}

bool UsbCtrlrEpOpen(int DevNo, const UsbEndPointDesc_t *pDesc)
{
	if (DevNo != 0)
	{
		return false;
	}

	if (pDesc == NULL)
	{
		return false;
	}

	const uint8_t epAddr = pDesc->bEndpointAddress;
	const uint8_t epNum = USB_ENDPADDR_NUM(epAddr);
	const uint8_t type = pDesc->bmAttributes & 0x03U;

	const bool iso = epNum == NRFX_USBD_ISO_EP_NO;
	const bool in = USB_ENDPADDR_IS_IN(epAddr);
	if (epNum == 0 || epNum >= NRFX_USBD_EP_COUNT ||
		(iso ? type != USB_ENDPATT_TRANS_ISO :
		 (type != USB_ENDPATT_TRANS_BULK && type != USB_ENDPATT_TRANS_INT)) ||
		pDesc->wMaxPacketSize == 0 ||
		pDesc->wMaxPacketSize > (iso ? NRFX_USBD_ISO_MAX_PACKET_SIZE :
			NRFX_USBD_MAX_PACKET_SIZE))
	{
		return false;
	}

	nRFUsbGetEpReg(epAddr)->Mps = pDesc->wMaxPacketSize;

	if (iso)
	{
		if (!AppEvtHandlerIdleRegister(nRFUsbdRetryIsoComplete))
			return false;

		// Both directions share the 1024-byte ISO buffer. HalfIN gives each
		// direction 512 bytes, which is more than the Bluetooth SCO maximum.
		NRF_USBD->ISOSPLIT =
			USBD_ISOSPLIT_SPLIT_HalfIN << USBD_ISOSPLIT_SPLIT_Pos;
		NRF_USBD->ISOINCONFIG =
			USBD_ISOINCONFIG_RESPONSE_ZeroData <<
			USBD_ISOINCONFIG_RESPONSE_Pos;
	}

	const uint8_t endBit = nRFUsbdDmaEndBit(epNum, in);
	*nRFUsbdDmaEndEvent(endBit) = 0U;
	if (iso || !in)
	{
		NRF_USBD->INTENSET = 1UL << endBit;
	}
	volatile uint32_t *pEnable = in ? &NRF_USBD->EPINEN : &NRF_USBD->EPOUTEN;
	*pEnable |= 1UL << epNum;

	if (iso)
	{
		atomic_fetch_or(&s_EventState.IsoOpen, (uint_fast8_t)(in ?
			NRFX_USBD_ISO_IN_OPEN : NRFX_USBD_ISO_OUT_OPEN));
		atomic_store(in ? &s_EventState.IsoInReady : &s_EventState.IsoOutReady, false);
		NRF_USBD->EVENTS_SOF = 0U;
		NRF_USBD->INTENSET = USBD_INTEN_SOF_Msk;
	}
	else
	{
		// Regular IN completion remains host-consumed EPDATA; no ENDEPIN IRQ.
		if (!in)
			NRF_USBD->SIZE.EPOUT[epNum] = 0U;
		NRF_USBD->EPSTALL =
			(USBD_EPSTALL_STALL_UnStall << USBD_EPSTALL_STALL_Pos) | epAddr;
		NRF_USBD->DTOGGLE =
			(USBD_DTOGGLE_VALUE_Data0 << USBD_DTOGGLE_VALUE_Pos) | epAddr;
	}
	__ISB();
	__DSB();
	return true;
}

void UsbCtrlrEpClose(int DevNo, uint8_t EpAddr)
{
	if (DevNo != 0)
	{
		return;
	}

	const uint8_t epNum = USB_ENDPADDR_NUM(EpAddr);
	if (epNum == 0 || epNum >= NRFX_USBD_EP_COUNT)
	{
		return;
	}

	const bool iso = epNum == NRFX_USBD_ISO_EP_NO;
	const bool in = USB_ENDPADDR_IS_IN(EpAddr);
	// Exclude SOF submission while retiring and closing an ISO direction.
	const uint32_t isoState = iso ? DisableInterrupt() : 0U;
	if (nRFUsbdDmaActive())
	{
		nRFUsbdDmaWait();
	}

	if (iso)
	{
		const uint8_t dir = in ? 1U : 0U;
		nRFUsbdXfer_t *pXfer = &s_Ctrlr.Iso[dir];
		const uint_fast8_t bit = 1U << dir;
		++s_EventState.IsoGeneration[dir];
		atomic_fetch_and(&s_EventState.IsoBusy, (uint_fast8_t)~bit);
		atomic_fetch_and(&s_EventState.IsoComplete, (uint_fast8_t)~bit);
		pXfer->pBuffer = NULL;
		pXfer->TotalLen = 0U;
		pXfer->ActualLen = 0U;
		atomic_fetch_and(&s_EventState.IsoOpen, (uint_fast8_t)~bit);
		atomic_store(in ? &s_EventState.IsoInReady : &s_EventState.IsoOutReady, false);
	}

	const uint8_t endBit = nRFUsbdDmaEndBit(epNum, in);
	volatile uint32_t *pEnable = in ? &NRF_USBD->EPINEN : &NRF_USBD->EPOUTEN;
	NRF_USBD->INTENCLR = 1UL << endBit;
	*pEnable &= ~(1UL << epNum);
	*nRFUsbdDmaEndEvent(endBit) = 0U;

	if (iso)
	{
		if (!s_Ctrlr.SofEnabled && atomic_load(&s_EventState.IsoOpen) == 0U &&
			!atomic_load(&s_EventState.BusSuspended))
		{
			NRF_USBD->INTENCLR = USBD_INTEN_SOF_Msk;
		}
		nRFUsbGetEpReg(EpAddr)->Mps = 0U;
		__DSB();
		EnableInterrupt(isoState);
		return;
	}

	NRF_USBD->EPDATASTATUS = 1UL << (epNum + (in ? 0U : 16U));
	if (!in)
	{
		NRF_USBD->SIZE.EPOUT[epNum] = 0;
	}
	nRFUsbGetEpReg(EpAddr)->Mps = 0U;
	__ISB();
	__DSB();
}

void UsbCtrlrEpCloseAll(int DevNo)
{
	if (DevNo != 0)
	{
		return;
	}

	nRFUsbdDmaWait();

	for (uint8_t epNum = 1; epNum < NRFX_USBD_EP_COUNT; epNum++)
	{
		UsbCtrlrEpClose(DevNo, epNum);
		UsbCtrlrEpClose(DevNo, (uint8_t)(epNum | USB_ENDPADDR_DIR_IN));
	}

	NRF_USBD->EPOUTEN = 1UL;
	NRF_USBD->EPINEN = 1UL;
}

bool UsbCtrlrEpRegister(int DevNo, uint8_t EpAddr, uint8_t *pBuffer,
						bool bBlocking, UsbCtrlrEpHandler_t Handler, void *pContext)
{
	const uint8_t epNum = USB_ENDPADDR_NUM(EpAddr);
	if (DevNo != 0 || epNum == 0U ||
		epNum >= NRF_USB_EP_COUNT ||
		(EpAddr & ~(USB_ENDPADDR_DIR_MASK | USB_ENDPADDR_NUM_MASK)) != 0U ||
		pBuffer == NULL || Handler == NULL)
	{
		return false;
	}

	nRFUsbEpReg_t *pReg = nRFUsbGetEpReg(EpAddr);
	pReg->pBuffer = pBuffer;
	pReg->Handler = Handler;
	pReg->pContext = pContext;
	pReg->bBlocking = bBlocking;
	return true;
}

bool UsbCtrlrEpXfer(int DevNo, uint8_t EpAddr, uint16_t Length)
{
	(void)DevNo;
	const uint8_t epNum = USB_ENDPADDR_NUM(EpAddr);
	const bool in = USB_ENDPADDR_IS_IN(EpAddr);
	if (epNum == NRFX_USBD_ISO_EP_NO)
	{
		return nRFUsbRegIsoXfer(in ? USB_ENDPADDR_DIRIN(epNum) : epNum, Length);
	}

	nRFUsbdQueXferDir(epNum, in, Length);
	return true;
}

bool UsbCtrlrEpOutXfer(int DevNo, uint8_t EpNum, uint16_t Length)
{
	(void)DevNo;
	if (EpNum == NRFX_USBD_ISO_EP_NO)
	{
		return nRFUsbRegIsoXfer(EpNum, Length);
	}

	nRFUsbdQueXferDir(EpNum, false, Length);
	return true;
}

bool UsbCtrlrEpInXfer(int DevNo, uint8_t EpNum, uint16_t Length)
{
	(void)DevNo;
	if (EpNum == NRFX_USBD_ISO_EP_NO)
	{
		return nRFUsbRegIsoXfer(USB_ENDPADDR_DIRIN(EpNum), Length);
	}

	nRFUsbdQueXferDir(EpNum, true, Length);
	return true;
}


bool UsbCtrlrEp0Xfer(int DevNo, uint8_t EpAddr, uint8_t *pBuffer,
						 uint16_t Length)
{
	if (DevNo != 0 || USB_ENDPADDR_NUM(EpAddr) != 0U)
	{
		return false;
	}

	if (USB_ENDPADDR_IS_IN(EpAddr) &&
		(NRF_USBD->BMREQUESTTYPE & USB_REQTYPE_MASK_DIR) != 0U)
	{
		return UsbCtrlrEp0Send(DevNo, pBuffer, Length) == Length;
	}

	const uint32_t state = DisableInterrupt();
	nRFUsbdXfer_t *pXfer = &s_Ctrlr.Ep0[nRFUsbdDir(EpAddr)];

	pXfer->pBuffer = pBuffer;
	pXfer->TotalLen = Length;
	pXfer->ActualLen = 0U;

	const bool controlStatus =
		Length == 0U &&
		USB_ENDPADDR_IS_IN(EpAddr) !=
		((NRF_USBD->BMREQUESTTYPE & USB_REQTYPE_MASK_DIR) != 0U);

	if (controlStatus)
	{
		if ((NRF_USBD->BMREQUESTTYPE & USB_REQTYPE_MASK_DIR) == 0U ||
			(NRF_USBD->SHORTS &
			 USBD_SHORTS_EP0DATADONE_EP0STATUS_Msk) == 0U)
		{
			nRFUsbdNoDmaTask(&NRF_USBD->TASKS_EP0STATUS);
		}
		EnableInterrupt(state);
		nRFUsbdEmitXfer(EpAddr, 0U, USB_CTRLR_XFER_SUCCESS);
		return true;
	}
	else if (USB_ENDPADDR_IS_IN(EpAddr))
	{
		nRFUsbdQueueEp0In();
	}
	else
	{
		nRFUsbdNoDmaTask(&NRF_USBD->TASKS_EP0RCVOUT);
	}

	EnableInterrupt(state);
	return true;
}

int UsbCtrlrEp0Send(int DevNo, uint8_t *pBuffer, int Length)
{
	int cnt = 0;

	(void)DevNo;
	s_Ctrlr.Ep0[1].TotalLen = (uint16_t)Length;

	do
	{
		const int l = min(Length, NRFX_USBD_MAX_PACKET_SIZE);

		nRFEPPkt_t *p = (nRFEPPkt_t*)CFifoPut(s_hEp0Que);

		if (l > 0)
		{
			memcpy(p->Payload, pBuffer, l);
			pBuffer += l;
		}
		p->Hdr.EpNum = 0U;
		p->Hdr.Dir = 1U;
		p->Hdr.Len = l;
		cnt += l;
		Length -= l;
	} while (Length > 0);

	if (NRFX_USBD_EASYDMA_BUSY_REG == NRFX_USBD_EASYDMA_BUSY_REG_CLEAR)
	{
		NRFX_USBD_EASYDMA_BUSY_REG = NRFX_USBD_EASYDMA_BUSY_REG_BUSY;
		NRF_USBD->EVENTS_EP0DATADONE = 0U;
		NRF_USBD->EVENTS_ENDEPIN[0] = 0U;

		nRFEPPkt_t *p = (nRFEPPkt_t *)CFifoPeek(s_hEp0Que);
		NRF_USBD->EPIN[0].PTR = (uint32_t)(uintptr_t)p->Payload;
		NRF_USBD->EPIN[0].MAXCNT = p->Hdr.Len;

		NRF_USBD->SHORTS = 0;

		if (p->Hdr.Len < NRFX_USBD_MAX_PACKET_SIZE)
		{
			NRF_USBD->SHORTS = USBD_SHORTS_EP0DATADONE_EP0STATUS_Msk;
		}
		NRF_USBD->TASKS_STARTEPIN[0] = 1U;

	}

	return cnt;
}

void UsbCtrlrEpStall(int DevNo, uint8_t EpAddr)
{
	if (DevNo != 0)
	{
		return;
	}

	const uint8_t epNum = USB_ENDPADDR_NUM(EpAddr);
	if (epNum >= NRFX_USBD_EP_COUNT || epNum == NRFX_USBD_ISO_EP_NO)
	{
		return;
	}

	if (epNum == 0)
	{
		NRF_USBD->TASKS_EP0STALL = 1;
	}
	else
	{
		NRF_USBD->EPSTALL =
			(USBD_EPSTALL_STALL_Stall << USBD_EPSTALL_STALL_Pos) | EpAddr;
	}
	__ISB();
	__DSB();
}

void UsbCtrlrEpClearStall(int DevNo, uint8_t EpAddr)
{
	if (DevNo != 0)
	{
		return;
	}

	const uint8_t epNum = USB_ENDPADDR_NUM(EpAddr);
	if (epNum == 0 || epNum >= NRFX_USBD_EP_COUNT ||
		epNum == NRFX_USBD_ISO_EP_NO)
	{
		return;
	}

	NRF_USBD->DTOGGLE = EpAddr;
	NRF_USBD->DTOGGLE =
		(USBD_DTOGGLE_VALUE_Data0 << USBD_DTOGGLE_VALUE_Pos) | EpAddr;
	NRF_USBD->EPSTALL =
		(USBD_EPSTALL_STALL_UnStall << USBD_EPSTALL_STALL_Pos) | EpAddr;

	if (!USB_ENDPADDR_IS_IN(EpAddr))
	{
		NRF_USBD->SIZE.EPOUT[epNum] = 0;
	}
	__ISB();
	__DSB();
}
