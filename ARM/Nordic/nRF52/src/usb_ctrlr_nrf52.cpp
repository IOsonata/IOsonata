/**-------------------------------------------------------------------------
@file	usb_ctrlr_nrf52.cpp

@brief	USB device controller for Nordic nRF52 parts.

Direct register implementation for the full-speed USBD peripheral. The port
owns bus power, clock and VBUS handling, endpoint events and the single shared
EasyDMA channel.

DevNo selects the controller. Every nRF part has exactly one, USB_CTRLR_CNT is
1. UsbCtrlrInit validates DevNo; later entry points receive that stored,
validated controller number and the state stays a singleton. Arraying it is
work for the first part that carries two.

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

// Barriers repeat at many sites. One out-of-line copy: a barrier keeps its
// effect when reached through a call, and the call is half the size of the
// pair.
static __attribute__((noinline)) void UsbdSync(void)
{
	__ISB();
	__DSB();
}


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
	NRFX_USBD_XFER_EVT_OUT = 0x80U,
	NRFX_USBD_QUE_OUT = 0U,
	NRFX_USBD_QUE_IN_BUFFER = 1U,
	NRFX_USBD_QUE_IN_FIFO = 2U,
	NRFX_USBD_QUE_IN_SCRATCH = 3U,
};

#pragma pack(push, 4)

typedef struct __nRF_Usbd_Que {
	uint8_t EpNum;				//!< Hardware endpoint number
	uint8_t Dir;					//!< Queue source/direction
	uint16_t Len;				//!< Bytes this transfer moves
	union {
		uint8_t *pBuffer;		//!< OUT or direct IN DMA buffer
		hCFifo_t hFifo;			//!< Byte-mode IN source FIFO
		uint32_t Scratch;		//!< Aligned byte-mode IN repair
	};
} nRFUsbdQue_t;

typedef struct __nRF_Ep_Packet {
	uint16_t Resv;
	uint16_t Len;
	uint8_t Payload[NRFX_USBD_MAX_PACKET_SIZE];
} nRFEPPkt_t;

#pragma pack(pop)


//
// nRF52 USBD state.
//

alignas(4) static uint8_t s_QueMem[
	CFIFO_TOTAL_MEMSIZE(NRFUSBD_QUE_DEPTH, sizeof(nRFUsbdQue_t))];

alignas(4) static uint8_t s_Ep0QueMem[
	CFIFO_TOTAL_MEMSIZE(NRFUSBD_EP0_QUE_DEPTH, sizeof(nRFEPPkt_t))];

nRFUsbdState_t s_Usbd;

extern bool nRFUsbdIsoStart(void) __attribute__((weak));
extern void nRFUsbdIsoService(void) __attribute__((weak));
extern bool nRFUsbdIsoFinishDma(uint32_t DmaStatus) __attribute__((weak));
extern void nRFUsbdIsoSof(void) __attribute__((weak));
extern bool nRFUsbdIsoEpOpen(const UsbEndPointDesc_t *pDesc) __attribute__((weak));
extern void nRFUsbdIsoEpClose(uint8_t EpAddr) __attribute__((weak));
extern bool nRFUsbdIsoXfer(uint8_t EpAddr, uint16_t Length) __attribute__((weak));

static void nRFUsbdHostResumeDetected(void);


static __attribute__((noinline))
nRFUsbEpReg_t *nRFUsbGetEpReg(uint8_t EpNum, uint8_t Dir)
{
	return &s_Usbd.EpReg[EpNum - 1U][Dir];
}

// Internal callers already know endpoint number and direction. Form the USB
// address only for the registered callback; these events always report success.
__attribute__((noinline))
void nRFUsbEpRegisteredEvent(uint8_t EpNum, uint8_t Dir,
	UsbCtrlrEvtType_t Event, uint16_t Length)
{
	nRFUsbEpReg_t *pReg = nRFUsbGetEpReg(EpNum, Dir);
	pReg->Handler(EpNum | (Dir * USB_ENDPADDR_DIR_IN), Event, Length,
		USB_CTRLR_XFER_SUCCESS, pReg->pContext);
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

// Bounded spin for a status bit shared by the clock, controller and
// regulator ready waits.
static __attribute__((noinline)) bool UsbdWaitReady(const volatile uint32_t *pReg, uint32_t Msk,
						  uint32_t Loops)
{
	// Include the final readiness read after the requested wait iterations.
	do
	{
		if ((*pReg & Msk) != 0U)
		{
			return true;
		}
	} while (Loops-- != 0U);

	return false;
}

static bool UsbdXtalRequest(void)
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
				break;
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

	if (UsbdWaitReady(&NRF_CLOCK->EVENTS_HFCLKSTARTED, 0xFFFFFFFFUL,
					  NRFX_USBD_XTAL_WAIT_LOOPS))
	{
		NRF_CLOCK->EVENTS_HFCLKSTARTED = 0;
		return true;
	}

	return false;
}

static inline void UsbdXtalRelease(void)
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
static void Usbd171Write(uint32_t Value)
{
	if (nrf52_errata_171())
	{
		UsbdErrataWrite(NRFX_USBD_ERRATA_171_REG, Value);
	}
}

static void UsbdErrataApply(void)
{
	if (nrf52_errata_187())
	{
		UsbdErrataWrite(NRFX_USBD_ERRATA_187_REG, 0x00000003UL);
	}

	Usbd171Write(0x000000C0UL);
}

static void UsbdErrataRevert(void)
{
	Usbd171Write(0x00000000UL);

	if (nrf52_errata_187())
	{
		UsbdErrataWrite(NRFX_USBD_ERRATA_187_REG, 0x00000000UL);
	}

	if (nrf52_errata_166())
	{
		NRFX_USBD_REG32(NRFX_USBD_ERRATA_166_REG_A) = 0x7E3UL;
		NRFX_USBD_REG32(NRFX_USBD_ERRATA_166_REG_B) = 0x40UL;
		UsbdSync();
	}
}


static inline bool UsbdIsForceNormal(void)
{
	return NRF_USBD->LOWPOWER ==
		(USBD_LOWPOWER_LOWPOWER_ForceNormal << USBD_LOWPOWER_LOWPOWER_Pos);
}

static __attribute__((noinline)) void UsbdForceNormal(void)
{
	if (!UsbdIsForceNormal())
	{
		NRF_USBD->LOWPOWER =
			USBD_LOWPOWER_LOWPOWER_ForceNormal << USBD_LOWPOWER_LOWPOWER_Pos;
		UsbdSync();
	}
}

static bool UsbdStartCtrlr(void)
{
	NRF_USBD->EVENTCAUSE = USBD_EVENTCAUSE_READY_Msk;
	UsbdSync();

	UsbdErrataApply();

	NRF_USBD->ENABLE = 1;
	UsbdSync();

	const bool ready = UsbdWaitReady(&NRF_USBD->EVENTCAUSE,
		USBD_EVENTCAUSE_READY_Msk, NRFX_USBD_READY_WAIT_LOOPS);
	if (ready)
	{
		NRF_USBD->EVENTCAUSE = USBD_EVENTCAUSE_READY_Msk;
		UsbdSync();
	}

	UsbdErrataRevert();

	// The regulator reports itself usable separately from the controller, and
	// pulling up before it does gives the host a device that cannot answer.
	if (!ready || !UsbdWaitReady(&NRF_POWER->USBREGSTATUS,
					   POWER_USBREGSTATUS_OUTPUTRDY_Msk,
					   NRFX_USBD_READY_WAIT_LOOPS))
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


static_assert(offsetof(NRF_USBD_Type, EVENTS_ENDEPIN) -
	offsetof(NRF_USBD_Type, EVENTS_USBRESET) ==
	USBD_INTEN_ENDEPIN0_Pos * sizeof(uint32_t), "USBD IN event layout");
static_assert(offsetof(NRF_USBD_Type, EVENTS_ENDEPOUT) -
	offsetof(NRF_USBD_Type, EVENTS_USBRESET) ==
	USBD_INTEN_ENDEPOUT0_Pos * sizeof(uint32_t), "USBD OUT event layout");

// Endpoint interrupt, END event and enable-mask writes shared by open and
// close.
static __attribute__((noinline))
void nRFUsbdEpHwEnable(uint8_t EpNum, bool In, bool Enable)
{
	// Interrupt bits index the event registers from EVENTS_USBRESET.
	const uint8_t endBit = In ? USBD_INTEN_ENDEPIN0_Pos + EpNum :
		USBD_INTEN_ENDEPOUT0_Pos + EpNum;
	volatile uint32_t *pEnd = (volatile uint32_t *)(
		(uintptr_t)&NRF_USBD->EVENTS_USBRESET + endBit * sizeof(uint32_t));
	volatile uint32_t *pEnable = In ? &NRF_USBD->EPINEN : &NRF_USBD->EPOUTEN;
	const uint32_t msk = 1UL << EpNum;

	if (Enable)
	{
		*pEnd = 0U;
		// Regular IN completion is host-consumed EPDATA, so only OUT needs
		// an END interrupt.
		if (!In)
		{
			NRF_USBD->INTENSET = 1UL << endBit;
		}
		*pEnable |= msk;
	}
	else
	{
		NRF_USBD->INTENCLR = 1UL << endBit;
		*pEnable &= ~msk;
		*pEnd = 0U;
	}
}

// Initialize the active event fields; UsbDevProcessEvent reads only that
// variant.
static __attribute__((noinline)) void nRFUsbdEmitSimple(UsbCtrlrEvtType_t Type)
{
	UsbCtrlrEvt_t evt;
	evt.Type = Type;
	UsbDevProcessEvent(0, &evt);
}

// EP0 only: registered endpoints complete through their handler, never
// through the core event path.
static __attribute__((noinline)) void nRFUsbdEmitXfer(uint8_t EpAddr, uint16_t Length)
{
	UsbCtrlrEvt_t evt;
	evt.Type = USB_CTRLR_EVT_XFER_CMPL;
	evt.Xfer.EpAddr = EpAddr;
	evt.Xfer.Length = Length;
	evt.Xfer.Result = USB_CTRLR_XFER_SUCCESS;
	UsbDevProcessEvent(0, &evt);
}

static inline __attribute__((always_inline)) void nRFUsbdDmaLock(void)
{
	NRFX_USBD_EASYDMA_BUSY_REG = NRFX_USBD_EASYDMA_BUSY_REG_BUSY;
}

__attribute__((noinline))
void nRFUsbdDmaUnlock(void)
{
	NRFX_USBD_EASYDMA_BUSY_REG = NRFX_USBD_EASYDMA_BUSY_REG_CLEAR;
	__DSB();
}

/** Start EasyDMA with the channel already locked by the caller. */
__attribute__((noinline))
void nRFUsbdDmaStartLocked(volatile uint32_t *pTask,
	volatile uint32_t *pEnd)
{
	*pEnd = 0;
	__DSB();

	*pTask = 1;
	__DSB();
}


// Retire one regular-endpoint DMA identified by its EPSTATUS bit index.
// Returns false while its END event has not fired. Release the queue entry
// only after DMA has finished reading it, including inline alignment scratch.
// Keep the channel locked for the caller's next DMA or explicit release.
static __attribute__((noinline)) bool nRFUsbdRetireDma(uint32_t StatusBit)
{
	const uint8_t epNum = (uint8_t)(StatusBit & 7U);
	volatile uint32_t *pEnd = StatusBit >= 16U ?
		&NRF_USBD->EVENTS_ENDEPOUT[epNum] :
		&NRF_USBD->EVENTS_ENDEPIN[epNum];

	if (*pEnd == 0U)
	{
		return false;
	}

	*pEnd = 0U;
	NRF_USBD->EPSTATUS = 1UL << StatusBit;
	(void)CFifoGet(epNum == 0U ? s_Usbd.hEp0Que : s_Usbd.hQue);
	// Complete END and EPSTATUS writes before the OUT callback or next DMA.
	__DSB();
	return true;
}

/** Retire active DMA before a foreground stop or endpoint close. */
void nRFUsbdDmaWait(void)
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
		bool complete = false;
		if (dmaStatus != 0U)
		{
			complete = nRFUsbdRetireDma(31U - (uint32_t)__CLZ(dmaStatus));
		}
		else if (nRFUsbdIsoFinishDma != nullptr)
		{
			complete = nRFUsbdIsoFinishDma(0U);
		}
		if (complete)
			nRFUsbdDmaUnlock();
		__set_PRIMASK(primask);
	}
}

// Program and start one staged EP0 IN packet, arming the status-stage short
// when the packet is short. The caller already owns EasyDMA.
static __attribute__((noinline)) void nRFUsbdEp0InStart(const nRFEPPkt_t *p)
{
	NRF_USBD->EPIN[0].PTR = (uint32_t)(uintptr_t)p->Payload;
	NRF_USBD->EPIN[0].MAXCNT = p->Len;
	NRF_USBD->SHORTS = p->Len < NRFX_USBD_MAX_PACKET_SIZE ?
		USBD_SHORTS_EP0DATADONE_EP0STATUS_Msk : 0U;
	nRFUsbdDmaStartLocked(&NRF_USBD->TASKS_STARTEPIN[0],
		&NRF_USBD->EVENTS_ENDEPIN[0]);
}

static __attribute__((noinline)) void nRFUsbdNoDmaTask(volatile uint32_t *pTask)
{
	*pTask = 1;
	UsbdSync();
}

// The SOF interrupt is shared: application SOF events, open ISO endpoints
// and suspend-time resume detection all need it. Acquire clears a stale
// event first; release drops it only when nothing needs it anymore.
__attribute__((noinline)) void nRFUsbdSofAcquire(void)
{
	NRF_USBD->EVENTS_SOF = 0U;
	NRF_USBD->INTENSET = USBD_INTENSET_SOF_Msk;
}

__attribute__((noinline)) void nRFUsbdSofRelease(void)
{
	if (!s_Usbd.Ctrlr.SofEnabled &&
		(s_Usbd.Flags & (USBD_FLAG_ISO_IN_OPEN | USBD_FLAG_ISO_OUT_OPEN |
			USBD_FLAG_SUSPENDED)) == 0U)
	{
		NRF_USBD->INTENCLR = USBD_INTENCLR_SOF_Msk;
	}
}

/**
 * Start EasyDMA for one regular queued request. Endpoint number and
 * direction stay separate in the scheduler; what an OUT endpoint actually
 * holds is only known now, so that is read here.
 */
static inline __attribute__((always_inline))
void nRFUsbdStartDmaNow(const nRFUsbdQue_t *pQue)
{
	const uint8_t epNum = pQue->EpNum;
	const bool isIn = pQue->Dir != NRFX_USBD_QUE_OUT;
	const uint8_t *pBuffer;

	if (pQue->Dir == NRFX_USBD_QUE_IN_FIFO)
	{
		pBuffer = CFifoPeek(pQue->hFifo);
	}
	else if (pQue->Dir == NRFX_USBD_QUE_IN_SCRATCH)
	{
		pBuffer = (const uint8_t *)&pQue->Scratch;
	}
	else
	{
		pBuffer = pQue->pBuffer;
	}

	uint16_t len = pQue->Len;

	volatile USBD_EPIN_Type *pEp;
	volatile uint32_t *pTask;
	if (isIn)
	{
		pEp = &NRF_USBD->EPIN[epNum];
		pTask = &NRF_USBD->TASKS_STARTEPIN[epNum];
	}
	else
	{
		const uint16_t received = (uint16_t)NRF_USBD->SIZE.EPOUT[epNum];
		if (received < len)
		{
			len = received;
		}

		static_assert(offsetof(USBD_EPOUT_Type, PTR) ==
			offsetof(USBD_EPIN_Type, PTR) &&
			offsetof(USBD_EPOUT_Type, MAXCNT) ==
			offsetof(USBD_EPIN_Type, MAXCNT), "EPIN/EPOUT layout");
		pEp = (volatile USBD_EPIN_Type *)&NRF_USBD->EPOUT[epNum];
		pTask = &NRF_USBD->TASKS_STARTEPOUT[epNum];
	}

	pEp->PTR = (uint32_t)(uintptr_t)pBuffer;
	pEp->MAXCNT = len;
	*pTask = 1U;
	__DSB();
}

static inline __attribute__((always_inline)) bool nRFUsbdDmaAllowed(void)
{
	const uint32_t gate = s_Usbd.Flags &
		(USBD_FLAG_HOST_RESUME | USBD_FLAG_SUSPENDED | USBD_FLAG_SUSPEND_PEND);
	return (gate & USBD_FLAG_HOST_RESUME) == 0U &&
		gate != USBD_FLAG_SUSPENDED;
}

// Callers check the power gate and own the channel lock. EP0 starts
// separately in its submission path or the ISR completion handoff.
static __attribute__((noinline)) void nRFUsbdStartQueuedDma(void)
{
	if (nRFUsbdIsoStart != nullptr && nRFUsbdIsoStart())
		return;
	nRFUsbdQue_t *pQue = (nRFUsbdQue_t *)CFifoPeek(s_Usbd.hQue);
	if (pQue != NULL)
	{
		nRFUsbdStartDmaNow(pQue);
		return;
	}
	nRFUsbdDmaUnlock();
}


// Submission acquires an idle channel; completion retains the existing lock.
__attribute__((noinline)) void nRFUsbdResumeQueuedDmaLocked(void)
{
	if (nRFUsbdDmaActive() || !nRFUsbdDmaAllowed())
		return;

	nRFUsbdDmaLock();
	nRFUsbdStartQueuedDma();
}
static void nRFUsbdResumeQueuedDma(void)
{
	const uint32_t state = DisableInterrupt();
	nRFUsbdResumeQueuedDmaLocked();
	EnableInterrupt(state);
}

/**
 * Put one regular DMA request on the queue with interrupts already excluded by the
 * caller: CFifoPut publishes the slot before the caller writes it. The caller
 * resumes DMA after any required EPDATASTATUS acknowledgement.
 */
static __attribute__((noinline)) bool nRFUsbdQueXferDir(uint8_t EpNum, bool In, uint16_t Len)
{
	uint8_t *pBuffer = s_Usbd.EpReg[EpNum - 1U][In ? 1 : 0].pBuffer;
	if (!In && pBuffer == NULL)
	{
		return false;
	}
	nRFUsbdQue_t *pQue = (nRFUsbdQue_t *)CFifoPut(s_Usbd.hQue);
	if (pQue == NULL)
	{
		return false;
	}

	pQue->EpNum = EpNum;
	pQue->Dir = In ? NRFX_USBD_QUE_IN_BUFFER : NRFX_USBD_QUE_OUT;
	pQue->Len = Len;
	pQue->pBuffer = pBuffer;

	return true;
}
static void nRFUsbdResetState(void)
{
	memset(&s_Usbd.Ctrlr, 0, sizeof(s_Usbd.Ctrlr));

	CFifoFlush(s_Usbd.hQue);
	CFifoFlush(s_Usbd.hEp0Que);
	s_Usbd.Flags = USBD_FLAG_MAC_AWAKE;
	s_Usbd.IsoOutSize = 0U;
	++s_Usbd.IsoGeneration[0];
	++s_Usbd.IsoGeneration[1];
	nRFUsbdDmaUnlock();
}

static void nRFUsbdAbortEp0(void)
{
	const uint32_t state = DisableInterrupt();
	CFifoFlush(s_Usbd.hEp0Que);
	EnableInterrupt(state);

	memset(&s_Usbd.Ctrlr.Ep0, 0, sizeof(s_Usbd.Ctrlr.Ep0));

	NRF_USBD->EVENTS_ENDEPIN[0] = 0;
	NRF_USBD->EVENTS_ENDEPOUT[0] = 0;
	NRF_USBD->EPDATASTATUS = (1UL << 0) | (1UL << 16);
	UsbdSync();
}

// ISR context only: this interrupt is the sole mutator of the wake state,
// so no interrupt exclusion is needed here.
static void nRFUsbdTryEnterLowPower(void)
{
	const uint32_t entryMask = USBD_FLAG_SUSPENDED | USBD_FLAG_SUSPEND_PEND |
		USBD_FLAG_REMOTE_WAKE | USBD_FLAG_HOST_RESUME;
	const uint32_t entryWant = USBD_FLAG_SUSPENDED | USBD_FLAG_SUSPEND_PEND;

	if (!s_Usbd.LowPowerSuspend ||
		(s_Usbd.Flags & entryMask) != entryWant ||
		nRFUsbdDmaActive() ||
		CFifoUsed(s_Usbd.hQue) > 0)
	{
		return;
	}

	if ((NRF_USBD->EVENTCAUSE & USBD_EVENTCAUSE_RESUME_Msk) != 0U ||
		NRF_USBD->EVENTS_SOF != 0U)
	{
		nRFUsbdHostResumeDetected();
		return;
	}

	s_Usbd.Flags &= ~(uint32_t)USBD_FLAG_MAC_AWAKE;
	NRF_USBD->LOWPOWER =
		USBD_LOWPOWER_LOWPOWER_LowPower << USBD_LOWPOWER_LOWPOWER_Pos;
	(void)NRF_USBD->LOWPOWER;
	UsbdSync();

	if ((NRF_USBD->EVENTCAUSE & USBD_EVENTCAUSE_RESUME_Msk) != 0U ||
		NRF_USBD->EVENTS_SOF != 0U)
	{
		nRFUsbdHostResumeDetected();
		return;
	}

	// Hardware resume is handled by the RESUME/SOF check above.
	s_Usbd.Flags &= ~(uint32_t)USBD_FLAG_SUSPEND_PEND;
}

static void nRFUsbdTryRemoteWake(void)
{
	if ((s_Usbd.Flags & USBD_FLAG_REMOTE_WAKE) == 0U)
	{
		return;
	}

	// Validate the wake request once while interrupts are excluded.
	const uint32_t wakeMask = USBD_FLAG_REMOTE_WAKE | USBD_FLAG_SUSPENDED |
		USBD_FLAG_HOST_RESUME | USBD_FLAG_MAC_AWAKE;
	const uint32_t wakeWant = USBD_FLAG_REMOTE_WAKE | USBD_FLAG_SUSPENDED |
		USBD_FLAG_MAC_AWAKE;
	const uint32_t irqState = DisableInterrupt();
	if ((s_Usbd.Flags & wakeMask) != wakeWant ||
		nRFUsbdDmaActive() ||
		!UsbdIsForceNormal())
	{
		EnableInterrupt(irqState);
		return;
	}

	s_Usbd.Flags &= ~(uint32_t)USBD_FLAG_REMOTE_WAKE;
	NRF_USBD->DPDMVALUE = USBD_DPDMVALUE_STATE_Resume;
	NRF_USBD->TASKS_DPDMDRIVE = 1;
	UsbdSync();
	EnableInterrupt(irqState);

	if ((NRF_USBD->INTEN & USBD_INTEN_SOF_Msk) == 0U)
	{
		NRF_USBD->EVENTS_SOF = 0;
	}
	NRF_USBD->INTENSET = USBD_INTENSET_SOF_Msk;
}

static void nRFUsbdHostResumeDetected(void)
{
	const uint32_t irqState = DisableInterrupt();
	uint32_t flags = s_Usbd.Flags;

	if ((flags & USBD_FLAG_SUSPENDED) == 0U)
	{
		EnableInterrupt(irqState);
		return;
	}

	flags &= ~(uint32_t)(USBD_FLAG_SUSPENDED | USBD_FLAG_SUSPEND_PEND |
		USBD_FLAG_REMOTE_WAKE);

	if ((flags & USBD_FLAG_MAC_AWAKE) == 0U || !UsbdIsForceNormal())
	{
		s_Usbd.Flags = flags | USBD_FLAG_HOST_RESUME;
		EnableInterrupt(irqState);
		UsbdForceNormal();
		return;
	}

	s_Usbd.Flags = flags & ~(uint32_t)USBD_FLAG_HOST_RESUME;
	EnableInterrupt(irqState);
	nRFUsbdEmitSimple(USB_CTRLR_EVT_RESUME);
}

// ISR context only.
static void nRFUsbdWakeAllowed(void)
{
	const uint32_t flags = s_Usbd.Flags | USBD_FLAG_MAC_AWAKE;

	if ((flags & USBD_FLAG_HOST_RESUME) != 0U)
	{
		s_Usbd.Flags = flags & ~(uint32_t)USBD_FLAG_HOST_RESUME;
		nRFUsbdEmitSimple(USB_CTRLR_EVT_RESUME);
		return;
	}

	s_Usbd.Flags = flags;
}

static void nRFUsbdBusReset(void)
{
	NRF_USBD->EPOUTEN = 1UL;
	NRF_USBD->EPINEN = 1UL;

	// STARTEPIN[8], STARTISOIN, STARTEPOUT[8] and STARTISOOUT are eighteen
	// consecutive task registers; clear them in one pass.
	static_assert(offsetof(NRF_USBD_Type, TASKS_STARTISOOUT) -
		offsetof(NRF_USBD_Type, TASKS_STARTEPIN) == 17U * sizeof(uint32_t),
		"USBD start task layout");
	volatile uint32_t *pTask = &NRF_USBD->TASKS_STARTEPIN[0];
	for (uint8_t i = 0; i < 18U; i++)
	{
		pTask[i] = 0;
	}

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
	nRFUsbdXfer_t *pXfer = &s_Usbd.Ctrlr.Ep0[0];

	if (pXfer->pBuffer != NULL)
	{
		if (TransferLen > 0U)
		{
			memcpy(pXfer->pBuffer, s_Usbd.Ep0Bounce, TransferLen);
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
		nRFUsbdEmitXfer(0U, pXfer->ActualLen);
	}
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
		nRFUsbdEmitXfer(USB_ENDPADDR_DIR_IN, amount);
	}

	nRFUsbdResumeQueuedDma();
}

static void nRFUsbdProcessInComplete(uint32_t Evt, void *pContext)
{
	(void)pContext;

	// This callback handles IN; the event carries only the endpoint number.
	nRFUsbEpRegisteredEvent((uint8_t)Evt, 1U,
		USB_CTRLR_EVT_XFER_CMPL, (uint16_t)(Evt >> 8U));
}


static void nRFUsbdProcessOutData(uint32_t Evt, void *pContext)
{
	const uint8_t epNum = (uint8_t)Evt;
	(void)pContext;

	const uint32_t state = DisableInterrupt();
	const uint32_t bit = 1UL << (epNum + 16U);
	// A latched OUT bit owns the request until DMA can be queued. AppEvt
	// retries may overlap; only the first accepted enqueue consumes it.
	if ((NRF_USBD->EPDATASTATUS & bit) != 0U)
	{
		const uint16_t len = s_Usbd.EpReg[epNum - 1U][0].MaxPacketSize;
		if (nRFUsbdQueXferDir(epNum, false, len))
		{
			// Acknowledge before starting DMA, which can admit the next packet.
			NRF_USBD->EPDATASTATUS = bit;
			__DSB();
			nRFUsbdResumeQueuedDmaLocked();
		}
		else
		{
			(void)AppEvtHandlerQue(epNum, NULL, nRFUsbdProcessOutData);
		}
	}
	EnableInterrupt(state);
}

static __attribute__((noinline)) void nRFUsbdQueueEp0Complete(bool Out, uint16_t Amount)
{
	const uint32_t evt = ((uint32_t)Amount << 8U) |
		(Out ? (uint32_t)NRFX_USBD_XFER_EVT_OUT : 0U);
	(void)AppEvtHandlerQue(evt, NULL, nRFUsbdProcessEp0Complete);
}

// InData is nonzero. Return only the status bit accepted by AppEvt; a full
// queue leaves it in EPDATASTATUS for UsbCtrlrProcess to retry.
static __attribute__((noinline))
uint32_t nRFUsbdQueueInComplete(uint32_t InData)
{
	const uint32_t epNum = 31U - (uint32_t)__CLZ(InData);
	const uint32_t evt = (NRF_USBD->EPIN[epNum].AMOUNT << 8U) | epNum;
	return (uint32_t)AppEvtHandlerQue(evt, NULL, nRFUsbdProcessInComplete) << epNum;
}

static void nRFUsbdHandleBusEvent(uint32_t EventCause)
{
	if ((EventCause & USBD_EVENTCAUSE_SUSPEND_Msk) != 0U &&
		(s_Usbd.Flags & USBD_FLAG_SUSPENDED) == 0U)
	{
		// A bus suspend and a peripheral low-power transition are separate.
		// When low-power suspend is disabled, retain all endpoint state and
		// wait for RESUME or SOF without touching USBD LOWPOWER.
		s_Usbd.Flags = (s_Usbd.Flags & ~(uint32_t)(USBD_FLAG_REMOTE_WAKE |
			USBD_FLAG_HOST_RESUME | USBD_FLAG_SUSPEND_PEND |
			USBD_FLAG_ISO_IN_READY | USBD_FLAG_ISO_OUT_READY)) |
			USBD_FLAG_SUSPENDED |
			(s_Usbd.LowPowerSuspend ?
			 (uint32_t)USBD_FLAG_SUSPEND_PEND : 0U);
		nRFUsbdSofAcquire();
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

	if (nRFUsbdIsoSof != nullptr)
	{
		nRFUsbdIsoSof();
	}

	if (s_Usbd.Ctrlr.SofEnabled)
	{
		UsbCtrlrEvt_t evt;
		evt.Type = USB_CTRLR_EVT_SOF;
		evt.FrameNo = (uint16_t)NRF_USBD->FRAMECNTR;
		UsbDevProcessEvent(0, &evt);
	}

	nRFUsbdSofRelease();

	if (nRFUsbdIsoService != nullptr)
	{
		nRFUsbdIsoService();
	}
}

static void nRFUsbdProcessEP0Setup(uint32_t Evt, void *pContext)
{
	(void)Evt;
	(void)pContext;

	UsbCtrlrEvt_t evt;
	evt.Type = USB_CTRLR_EVT_SETUP;

	// BMREQUESTTYPE through WLENGTHH are eight consecutive byte-wide
	// registers whose byte order is exactly the little endian layout of
	// UsbSetupData_t. Reading them in a loop beats five field combines.
	static_assert(offsetof(NRF_USBD_Type, WLENGTHH) -
		offsetof(NRF_USBD_Type, BMREQUESTTYPE) == 7U * sizeof(uint32_t),
		"USBD setup register layout");
	static_assert(sizeof(UsbSetupData_t) == 8, "UsbSetupData_t layout");
	const volatile uint32_t *pReg = &NRF_USBD->BMREQUESTTYPE;
	uint8_t *pDst = (uint8_t *)&evt.Setup;
	for (int i = 0; i < 8; i++)
	{
		pDst[i] = (uint8_t)pReg[i];
	}

	while (nRFUsbdDmaActive())
	{
	}

	nRFUsbdHostResumeDetected();
	nRFUsbdAbortEp0();

	if ((evt.Setup.bmRequestType &
		 (USB_REQTYPE_MASK_RECEIPT | USB_REQTYPE_MASK_TYPE)) == 0U &&
		evt.Setup.bRequest == USB_REQ_SET_ADDRESS)
	{
		// Address and Setup share the event union; wValue is read before
		// the union is repurposed.
		const uint8_t addr = (uint8_t)(evt.Setup.wValue & 0x7FU);
		evt.Type = USB_CTRLR_EVT_ADDRESS;
		evt.Address = addr;
	}

	UsbDevProcessEvent(0, &evt);
}


extern "C" void USBD_IRQHandler(void)
{
	uint32_t dmastatus = NRF_USBD->EPSTATUS;
	uint8_t outEp = 0U;

	// Reset cancels any active DMA and must not wait for ENDEP.
	if (NRF_USBD->EVENTS_USBRESET != 0U)
	{
		NRF_USBD->EVENTS_USBRESET = 0U;
		UsbdSync();
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

				(void)CFifoGet(s_Usbd.hEp0Que);
				nRFEPPkt_t *p =
					(nRFEPPkt_t *)CFifoPeek(s_Usbd.hEp0Que);
				if (p != NULL)
				{
					nRFUsbdEp0InStart(p);
				}
				else
				{
					nRFUsbdDmaUnlock();
					nRFUsbdQueueEp0Complete(false,
						s_Usbd.Ctrlr.Ep0[1].TotalLen);
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
		case 0x01000000U: // ISO OUT
			if (nRFUsbdIsoFinishDma != nullptr &&
				nRFUsbdIsoFinishDma(dmastatus))
				goto dmaComplete;
			break;
		default:          // EP1-7 IN/OUT
		{
			// EP0 and ISO were separated above. Retire the one regular DMA
			// directly; IN application completion still waits for EPDATA.
			const uint32_t statusBit = 31U - (uint32_t)__CLZ(dmastatus);
			const uint8_t epNum = (uint8_t)(statusBit & 7U);
			if (!nRFUsbdRetireDma(statusBit))
				return;

			if (statusBit >= 16U)
			{
				nRFUsbEpRegisteredEvent(epNum, 0U, USB_CTRLR_EVT_XFER_CMPL,
					(uint16_t)NRF_USBD->EPOUT[epNum].AMOUNT);
			}
		}
		dmaComplete:
			// Completion retains the lock through the regular OUT callback.
			// Restart before EPDATA/SOF work; SETUP and bus events take precedence.
			if (NRF_USBD->EVENTS_EP0SETUP == 0U &&
				NRF_USBD->EVENTS_USBEVENT == 0U && nRFUsbdDmaAllowed())
			{
				// SETUP's idle wait can be interrupted by a data submission.
				// Give its queued control response the retained channel first.
				const nRFEPPkt_t *pEp0 =
					(const nRFEPPkt_t *)CFifoPeek(s_Usbd.hEp0Que);
				if (pEp0 != NULL)
					nRFUsbdEp0InStart(pEp0);
				else
					nRFUsbdStartQueuedDma();
			}
			else
				nRFUsbdDmaUnlock();
			break;
	}

	if (NRF_USBD->EVENTS_USBEVENT != 0U)
	{
		NRF_USBD->EVENTS_USBEVENT = 0U;
		const uint32_t eventCause = NRF_USBD->EVENTCAUSE;
		NRF_USBD->EVENTCAUSE = eventCause;
		UsbdSync();

		nRFUsbdHandleBusEvent(eventCause);
	}

	if (NRF_USBD->EVENTS_EP0SETUP != 0U)
	{
		NRF_USBD->EVENTS_EP0SETUP = 0U;

		// A new SETUP aborts the previous control transfer. Discard any
		// simultaneously latched completion from that old transfer.
		NRF_USBD->EVENTS_EP0DATADONE = 0U;
		UsbdSync();

		(void)AppEvtHandlerQue(0U, NULL, nRFUsbdProcessEP0Setup);

		return;
	}
	if (NRF_USBD->EVENTS_EPDATA != 0U ||
		(NRF_USBD->EPDATASTATUS & 0x00FE00FEUL) != 0U)
	{
		// Clear the event first so a new endpoint event remains observable.
		// Service at most one endpoint per direction in this interrupt.
		// Unaccepted IN completions remain available to the foreground retry.
		NRF_USBD->EVENTS_EPDATA = 0U;
		const uint32_t dataStatus = NRF_USBD->EPDATASTATUS;
		uint32_t servicedStatus = dataStatus & 0x00010001UL;

		const uint32_t outData = (dataStatus >> 16U) & 0xFEU;
		if (outData != 0U)
		{
			outEp = (uint8_t)(31U - (uint32_t)__CLZ(outData));
		}

		const uint32_t inData = dataStatus & 0xFEU;
		if (inData != 0U)
		{
			servicedStatus |= nRFUsbdQueueInComplete(inData);
		}

		// Clear only serviced endpoints; keep every other status bit latched.
		NRF_USBD->EPDATASTATUS = servicedStatus;
		UsbdSync();
	}


	if (NRF_USBD->EVENTS_SOF != 0U)
	{
		NRF_USBD->EVENTS_SOF = 0U;
		UsbdSync();

		nRFUsbdHandleSof();
	}

	nRFUsbdTryRemoteWake();

	// Queue newly received OUT data; completion already restarted pending DMA.
	if (outEp != 0U)
	{
		nRFUsbdProcessOutData(outEp, NULL);
	}

	nRFUsbdTryEnterLowPower();

}

// Public controller API.

bool UsbCtrlrInit(int DevNo, const UsbCtrlrCfg_t *pCfg)
{
	if (DevNo != 0 || pCfg == NULL)
	{
		return false;
	}

	// Registrations, transfer state and flags all restart from zero;
	// ResetState below rebuilds what must not be zero.
	memset(&s_Usbd, 0, sizeof(s_Usbd));

	s_Usbd.IntPrio = pCfg->IntPrio;
	s_Usbd.LowPowerSuspend = pCfg->bLowPowerSuspend;

	s_Usbd.hQue = CFifoInit(s_QueMem, sizeof(s_QueMem), sizeof(nRFUsbdQue_t),
					   true);
	s_Usbd.hEp0Que = CFifoInit(s_Ep0QueMem, sizeof(s_Ep0QueMem),
						  sizeof(nRFEPPkt_t), true);

	nRFUsbdResetState();
	return true;
}

bool UsbCtrlrStart(int DevNo)
{
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

	NVIC_SetPriority(USBD_IRQn, s_Usbd.IntPrio);

	if (UsbdStartCtrlr() == false)
	{
		UsbdXtalRelease();
		return false;
	}

	return true;
}

void UsbCtrlrStop(int DevNo)
{
	(void)DevNo;
	nRFUsbdDmaWait();
	nRFUsbdResetState();

	// Stop the controller interrupt before powering down the wrapper.
	NVIC_DisableIRQ(USBD_IRQn);

	NRF_USBD->INTEN = 0;
	NRF_USBD->USBPULLUP = 0;
	NRF_USBD->ENABLE = 0;
	UsbdSync();

	// A successful start owns one clock request; a failed start releases it.
	UsbdXtalRelease();
}

// Suspend and wake are owned by the USBEVENT/SOF handlers. With low-power
// suspend disabled, this driver never enters peripheral low-power mode.
void UsbCtrlrProcess(int DevNo)
{
	(void)DevNo;
	AppEvtHandlerExec();

	// Share EPDATASTATUS with the ISR without publishing a completion twice.
	const uint32_t state = DisableInterrupt();
	const uint32_t inData = NRF_USBD->EPDATASTATUS & 0xFEU;
	if (inData != 0U)
	{
		NRF_USBD->EPDATASTATUS = nRFUsbdQueueInComplete(inData);
		__DSB();
	}
	uint32_t outData = (NRF_USBD->EPDATASTATUS >> 16U) & 0xFEU;
	while (outData != 0U)
	{
		const uint8_t epNum = (uint8_t)(31U - (uint32_t)__CLZ(outData));
		outData &= ~(1UL << epNum);
		// A held RX buffer must not starve another pending endpoint.
		nRFUsbdProcessOutData(epNum, NULL);
	}
	EnableInterrupt(state);
}

bool UsbCtrlrVbusDetected(int DevNo)
{
	(void)DevNo;
	return (NRF_POWER->USBREGSTATUS &
		POWER_USBREGSTATUS_VBUSDETECT_Msk) != 0;
}

bool UsbCtrlrHighSpeed(int DevNo)
{
	(void)DevNo;
	return false;
}

size_t UsbCtrlrGetSerial(int DevNo, char *pBuff, size_t BuffLen)
{
	(void)DevNo;
	(void)BuffLen;
	char *p = pBuff;

	for (uint8_t word = 0U; word < 2U; ++word)
	{
		uint32_t id = nrf_ficr_deviceid_get(NRF_FICR, word);
		for (uint8_t digitNo = 0; digitNo < 8U; ++digitNo)
		{
			const unsigned digit = id >> 28U;
			id <<= 4U;
			*p++ = (char)(digit + (digit < 10U ? '0' : 'A' - 10));
		}
	}
	*p = '\0';
	return 16U;
}

void UsbCtrlrIntEnable(int DevNo)
{
	(void)DevNo;
	NVIC_EnableIRQ(USBD_IRQn);
}

void UsbCtrlrIntDisable(int DevNo)
{
	(void)DevNo;
	NVIC_DisableIRQ(USBD_IRQn);
}

void UsbCtrlrConnect(int DevNo)
{
	(void)DevNo;
	NRF_USBD->USBPULLUP = 1;
}

void UsbCtrlrDisconnect(int DevNo)
{
	(void)DevNo;
	NRF_USBD->USBPULLUP = 0;
}

void UsbCtrlrRemoteWakeup(int DevNo)
{
	(void)DevNo;
	const uint32_t state = DisableInterrupt();
	const uint32_t flags = s_Usbd.Flags;
	if ((flags & (USBD_FLAG_SUSPENDED | USBD_FLAG_HOST_RESUME)) !=
		USBD_FLAG_SUSPENDED)
	{
		EnableInterrupt(state);
		return;
	}

	s_Usbd.Flags = (flags & ~(uint32_t)USBD_FLAG_SUSPEND_PEND) |
		USBD_FLAG_REMOTE_WAKE;
	EnableInterrupt(state);

	UsbdForceNormal();
	nRFUsbdTryRemoteWake();
}

void UsbCtrlrSofEnable(int DevNo, bool Enable)
{
	(void)DevNo;
	s_Usbd.Ctrlr.SofEnabled = Enable;

	if (Enable)
	{
		nRFUsbdSofAcquire();
	}
	else
	{
		nRFUsbdSofRelease();
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
	(void)DevNo;
	return nRFUsbdIsoEpOpen != nullptr && nRFUsbdIsoEpOpen(pDesc);
}

bool UsbCtrlrEpOpenData(int DevNo, uint8_t EpAddr, uint8_t Type,
						 uint16_t MaxPacketSize)
{
	(void)DevNo;
	(void)Type;
	const uint8_t epNum = USB_ENDPADDR_NUM(EpAddr);
	const bool in = USB_ENDPADDR_IS_IN(EpAddr);
	if (epNum == 0U || epNum >= NRFX_USBD_DATA_EP_COUNT ||
		MaxPacketSize == 0U || MaxPacketSize > NRFX_USBD_MAX_PACKET_SIZE)
	{
		return false;
	}

	nRFUsbGetEpReg(epNum, in)->MaxPacketSize = MaxPacketSize;
	nRFUsbdEpHwEnable(epNum, in, true);

	if (!in)
		NRF_USBD->SIZE.EPOUT[epNum] = 0U;
	NRF_USBD->EPSTALL =
		(USBD_EPSTALL_STALL_UnStall << USBD_EPSTALL_STALL_Pos) | EpAddr;
	NRF_USBD->DTOGGLE =
		(USBD_DTOGGLE_VALUE_Data0 << USBD_DTOGGLE_VALUE_Pos) | EpAddr;
	UsbdSync();
	return true;
}

void UsbCtrlrEpClose(int DevNo, uint8_t EpAddr)
{
	(void)DevNo;
	const uint8_t epNum = USB_ENDPADDR_NUM(EpAddr);
	if (epNum == 0U || epNum >= NRFX_USBD_EP_COUNT)
	{
		return;
	}

	if (epNum == NRFX_USBD_ISO_EP_NO)
	{
		if (nRFUsbdIsoEpClose != nullptr)
		{
			nRFUsbdIsoEpClose(EpAddr);
		}
		return;
	}

	const bool in = USB_ENDPADDR_IS_IN(EpAddr);
	nRFUsbdDmaWait();

	nRFUsbdEpHwEnable(epNum, in, false);
	NRF_USBD->EPDATASTATUS = 1UL << (epNum + (in ? 0U : 16U));
	if (!in)
	{
		NRF_USBD->SIZE.EPOUT[epNum] = 0;
	}
	nRFUsbGetEpReg(epNum, in)->MaxPacketSize = 0U;
	UsbdSync();
}

void UsbCtrlrEpCloseAll(int DevNo)
{
	for (uint8_t epNum = 1; epNum < NRFX_USBD_EP_COUNT; epNum++)
	{
		UsbCtrlrEpClose(DevNo, epNum);
		UsbCtrlrEpClose(DevNo, (uint8_t)(epNum | USB_ENDPADDR_DIR_IN));
	}

	NRF_USBD->EPOUTEN = 1UL;
	NRF_USBD->EPINEN = 1UL;
}

void UsbCtrlrEpAlloc(int DevNo, uint8_t EpAddr, uint8_t *pBuffer,
					 bool bBlocking,
					 UsbCtrlrEpHandler_t Handler, void *pContext)
{
	(void)DevNo;
	nRFUsbEpReg_t *pReg = nRFUsbGetEpReg(USB_ENDPADDR_NUM(EpAddr),
		USB_ENDPADDR_IS_IN(EpAddr));
	pReg->pBuffer = pBuffer;
	pReg->Handler = Handler;
	pReg->pContext = pContext;
	pReg->bBlocking = bBlocking;
}

// Keep the ISO dispatch in one copy for the directional wrappers.
__attribute__((noinline))
bool UsbCtrlrEpXfer(int DevNo, uint8_t EpAddr, uint16_t Length)
{
	(void)DevNo;
	const uint8_t epNum = USB_ENDPADDR_NUM(EpAddr);
	if (epNum == NRFX_USBD_ISO_EP_NO)
	{
		return nRFUsbdIsoXfer != nullptr &&
			nRFUsbdIsoXfer(EpAddr, Length);
	}

	const uint32_t state = DisableInterrupt();
	const bool queued = nRFUsbdQueXferDir(epNum, USB_ENDPADDR_IS_IN(EpAddr), Length);
	nRFUsbdResumeQueuedDmaLocked();
	EnableInterrupt(state);
	return queued;
}

bool UsbCtrlrEpOutXfer(int DevNo, uint8_t EpNum, uint16_t Length)
{
	return UsbCtrlrEpXfer(DevNo, EpNum, Length);
}

bool UsbCtrlrEpInXfer(int DevNo, uint8_t EpNum, uint8_t *pBuffer,
						 uint16_t Length)
{
	if (EpNum == NRFX_USBD_ISO_EP_NO)
	{
		s_Usbd.EpReg[EpNum - 1U][1].pBuffer = pBuffer;
		return UsbCtrlrEpXfer(DevNo, USB_ENDPADDR_DIRIN(EpNum), Length);
	}

	const uint32_t state = DisableInterrupt();
	nRFUsbdQue_t *pQue = (nRFUsbdQue_t *)CFifoPut(s_Usbd.hQue);
	if (pQue == NULL)
	{
		EnableInterrupt(state);
		return false;
	}
	pQue->EpNum = EpNum;
	pQue->Len = Length;

	if (pBuffer == NULL)
	{
		hCFifo_t hFifo = (hCFifo_t)s_Usbd.EpReg[EpNum - 1U][1].pBuffer;
		uint8_t *pData = CFifoPeek(hFifo);
		const uint32_t misalign = (uint32_t)(uintptr_t)pData & 3U;
		if (misalign != 0U)
		{
			const uint16_t repair = (uint16_t)(4U - misalign);
			if (pQue->Len > repair)
			{
				pQue->Len = repair;
			}
			memcpy(&pQue->Scratch, pData, pQue->Len);
			pQue->Dir = NRFX_USBD_QUE_IN_SCRATCH;
		}
		else
		{
			pQue->hFifo = hFifo;
			pQue->Dir = NRFX_USBD_QUE_IN_FIFO;
		}
	}
	else
	{
		pQue->Dir = NRFX_USBD_QUE_IN_BUFFER;
		pQue->pBuffer = pBuffer;
	}
	nRFUsbdResumeQueuedDmaLocked();
	EnableInterrupt(state);
	return true;
}


bool UsbCtrlrEp0Xfer(int DevNo, uint8_t EpAddr, uint8_t *pBuffer,
						 uint16_t Length)
{
	if (USB_ENDPADDR_NUM(EpAddr) != 0U)
	{
		return false;
	}

	const bool in = USB_ENDPADDR_IS_IN(EpAddr);
	const bool reqIn =
		(NRF_USBD->BMREQUESTTYPE & USB_REQTYPE_MASK_DIR) != 0U;

	if (Length == 0U && in != reqIn)
	{
		// Control status stage; a short IN packet may already have armed it.
		const uint32_t state = DisableInterrupt();
		if (!reqIn ||
			(NRF_USBD->SHORTS & USBD_SHORTS_EP0DATADONE_EP0STATUS_Msk) == 0U)
		{
			nRFUsbdNoDmaTask(&NRF_USBD->TASKS_EP0STATUS);
		}
		EnableInterrupt(state);
		nRFUsbdEmitXfer(EpAddr, 0U);
		return true;
	}

	if (in)
	{
		return UsbCtrlrEp0Send(DevNo, pBuffer, Length) == Length;
	}

	const uint32_t state = DisableInterrupt();
	nRFUsbdXfer_t *pXfer = &s_Usbd.Ctrlr.Ep0[0];
	pXfer->pBuffer = pBuffer;
	pXfer->TotalLen = Length;
	pXfer->ActualLen = 0U;
	nRFUsbdNoDmaTask(&NRF_USBD->TASKS_EP0RCVOUT);
	EnableInterrupt(state);
	return true;
}

int UsbCtrlrEp0Send(int DevNo, uint8_t *pBuffer, int Length)
{
	const int cnt = Length;

	(void)DevNo;
	s_Usbd.Ctrlr.Ep0[1].TotalLen = (uint16_t)Length;

	do
	{
		const int l = min(Length, NRFX_USBD_MAX_PACKET_SIZE);

		nRFEPPkt_t *p = (nRFEPPkt_t*)CFifoPut(s_Usbd.hEp0Que);

		if (l > 0)
		{
			memcpy(p->Payload, pBuffer, l);
			pBuffer += l;
		}

		p->Len = l;
		Length -= l;
	} while (Length != 0);

	if (NRFX_USBD_EASYDMA_BUSY_REG == NRFX_USBD_EASYDMA_BUSY_REG_CLEAR)
	{
		nRFUsbdDmaLock();
		NRF_USBD->EVENTS_EP0DATADONE = 0U;

		nRFUsbdEp0InStart((nRFEPPkt_t *)CFifoPeek(s_Usbd.hEp0Que));
	}

	return cnt;
}

void UsbCtrlrEpStall(int DevNo, uint8_t EpAddr)
{
	(void)DevNo;
	const uint8_t epNum = USB_ENDPADDR_NUM(EpAddr);
	if (epNum >= NRFX_USBD_DATA_EP_COUNT)
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
	UsbdSync();
}

void UsbCtrlrEpClearStall(int DevNo, uint8_t EpAddr)
{
	(void)DevNo;
	const uint8_t epNum = USB_ENDPADDR_NUM(EpAddr);
	if (epNum == 0 || epNum >= NRFX_USBD_DATA_EP_COUNT)
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
	UsbdSync();
}
