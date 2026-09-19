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

// EP0 OUT data staging through the DMA queue. Its interrupt-side producer
// is disabled; the whole chain sits behind this switch until it is wired
// back, so the disabled half does not cost object bytes or warnings.
#ifndef NRFX_USBD_EP0_OUT_QUE
#define NRFX_USBD_EP0_OUT_QUE				0
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

// EP1-7 can queue seven OUT entries and fourteen IN entries (scratch plus
// FIFO). Keep a power-of-two depth; EP0 and ISO do not occupy this queue.
#define NRFUSBD_QUE_DEPTH			32U
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
	nRFUsbdQue_t Hdr;
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


static inline __attribute__((always_inline))
uint8_t nRFUsbEpDir(uint8_t EpAddr)
{
	return USB_ENDPADDR_IS_IN(EpAddr) ? 1U : 0U;
}

static __attribute__((noinline))
nRFUsbEpReg_t *nRFUsbGetEpReg(uint8_t EpAddr)
{
	return &s_Usbd.EpReg[USB_ENDPADDR_NUM(EpAddr)][nRFUsbEpDir(EpAddr)];
}

// Share callback dispatch across regular and ISO event paths.
__attribute__((noinline))
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

// Bounded spin for a status bit shared by the clock, controller and
// regulator ready waits.
static __attribute__((noinline)) bool UsbdWaitReady(const volatile uint32_t *pReg, uint32_t Msk,
						  uint32_t Loops)
{
	for (uint32_t i = 0; i < Loops; i++)
	{
		if ((*pReg & Msk) != 0U)
		{
			return true;
		}
	}

	// One re-read after exhaustion, as the original loops had: a bit that
	// lands between the last iteration and this check still counts.
	return (*pReg & Msk) != 0U;
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

	if (!UsbdWaitReady(&NRF_USBD->EVENTCAUSE, USBD_EVENTCAUSE_READY_Msk,
					   NRFX_USBD_READY_WAIT_LOOPS))
	{
		UsbdErrataRevert();
		NRF_USBD->ENABLE = 0;
		return false;
	}

	NRF_USBD->EVENTCAUSE = USBD_EVENTCAUSE_READY_Msk;
	UsbdSync();

	UsbdErrataRevert();

	// The regulator reports itself usable separately from the controller, and
	// pulling up before it does gives the host a device that cannot answer.
	if (!UsbdWaitReady(&NRF_POWER->USBREGSTATUS,
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
	return In ? USBD_INTEN_ENDEPIN0_Pos + EpNum :
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
static_assert(offsetof(NRF_USBD_Type, EVENTS_ENDEPOUT) -
	offsetof(NRF_USBD_Type, EVENTS_USBRESET) ==
	USBD_INTEN_ENDEPOUT0_Pos * sizeof(uint32_t), "USBD OUT event layout");

static inline __attribute__((always_inline))
void nRFUsbdEmit(const UsbCtrlrEvt_t *pEvt)
{
	UsbDevProcessEvent(0, pEvt);
}

// Endpoint interrupt, END event and enable-mask writes shared by open and
// close.
static __attribute__((noinline))
void nRFUsbdEpHwEnable(uint8_t EpNum, bool In, bool Enable)
{
	const uint8_t endBit = nRFUsbdDmaEndBit(EpNum, In);
	volatile uint32_t *pEnable = In ? &NRF_USBD->EPINEN : &NRF_USBD->EPOUTEN;
	const uint32_t msk = 1UL << EpNum;

	if (Enable)
	{
		*nRFUsbdDmaEndEvent(endBit) = 0U;
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
		*nRFUsbdDmaEndEvent(endBit) = 0U;
	}
}

// Initialize the active event fields; UsbDevProcessEvent reads only that
// variant.
static __attribute__((noinline)) void nRFUsbdEmitSimple(UsbCtrlrEvtType_t Type)
{
	UsbCtrlrEvt_t evt;
	evt.Type = Type;
	nRFUsbdEmit(&evt);
}

// EP0 only: registered endpoints complete through their handler, never
// through the core event path.
static __attribute__((noinline)) void nRFUsbdEmitXfer(uint8_t EpAddr, uint16_t Length,
						 UsbCtrlrXferResult_t Result)
{
	UsbCtrlrEvt_t evt;
	evt.Type = USB_CTRLR_EVT_XFER_CMPL;
	evt.Xfer.EpAddr = EpAddr;
	evt.Xfer.Length = Length;
	evt.Xfer.Result = Result;
	nRFUsbdEmit(&evt);
}

__attribute__((noinline))
void nRFUsbdDmaUnlock(void)
{
	NRFX_USBD_EASYDMA_BUSY_REG = NRFX_USBD_EASYDMA_BUSY_REG_CLEAR;
	__DSB();
}

/** Start EasyDMA while the caller already excludes the USBD interrupt. */
__attribute__((noinline))
void nRFUsbdDmaStartLocked(volatile uint32_t *pTask,
	volatile uint32_t *pEnd)
{
	*pEnd = 0;
	__DSB();

	NRFX_USBD_EASYDMA_BUSY_REG = NRFX_USBD_EASYDMA_BUSY_REG_BUSY;
	*pTask = 1;
	__DSB();
}


// Retire one regular-endpoint DMA identified by its EPSTATUS bit index.
// Returns false while its END event has not fired. The queue owns its source
// through END. Regular IN keeps END latched until the host consumes the packet.
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

	if (StatusBit >= 16U || epNum == 0U)
		*pEnd = 0U;
	NRF_USBD->EPSTATUS = 1UL << StatusBit;
	if (epNum == 0U)
	{
		(void)CFifoGet(s_Usbd.hEp0Que);
	}
	else
	{
		(void)CFifoGet(s_Usbd.hQue);
	}
	// Unlock's DSB completes END, EPSTATUS and busy-register writes before
	// the OUT callback or another DMA can use this buffer.
	nRFUsbdDmaUnlock();
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
		if (dmaStatus != 0U)
		{
			(void)nRFUsbdRetireDma(31U - (uint32_t)__CLZ(dmaStatus));
		}
		else if (nRFUsbdIsoFinishDma != nullptr)
		{
			(void)nRFUsbdIsoFinishDma(0U);
		}
		__set_PRIMASK(primask);
	}
}

// Program EP0 IN for one staged packet and arm the status-stage short when
// the packet is short. The caller owns EasyDMA.
static __attribute__((noinline)) void nRFUsbdEp0InProgram(const nRFEPPkt_t *p)
{
	NRF_USBD->EPIN[0].PTR = (uint32_t)(uintptr_t)p->Payload;
	NRF_USBD->EPIN[0].MAXCNT = p->Hdr.Len;
	NRF_USBD->SHORTS = p->Hdr.Len < NRFX_USBD_MAX_PACKET_SIZE ?
		USBD_SHORTS_EP0DATADONE_EP0STATUS_Msk : 0U;
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



// Callers exclude the interrupt: RegIsoXfer holds DisableInterrupt and
// HandleSof runs in the ISR itself.


/**
 * Start EasyDMA for one queued directional request. Endpoint number and
 * direction stay separate in the scheduler; what an OUT endpoint actually
 * holds is only known now, so that is read here.
 */
static inline __attribute__((always_inline))
void nRFUsbdStartDmaNow(const nRFUsbdQue_t *pQue)
{
	const uint8_t epNum = pQue->EpNum;
	const bool isIn = pQue->Dir != NRFX_USBD_QUE_OUT;
	const uint8_t *pBuffer;

	if (epNum != 0U && isIn && NRF_USBD->EVENTS_ENDEPIN[epNum] != 0U)
		return;

	if (epNum == 0U)
	{
		pBuffer = isIn ? ((const nRFEPPkt_t *)pQue)->Payload :
			s_Usbd.Ep0Bounce;
	}
	else if (pQue->Dir == NRFX_USBD_QUE_IN_FIFO)
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
	volatile uint32_t *pEnd;
	if (isIn)
	{
		pEp = &NRF_USBD->EPIN[epNum];
		pTask = &NRF_USBD->TASKS_STARTEPIN[epNum];
		pEnd = &NRF_USBD->EVENTS_ENDEPIN[epNum];
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
		pEnd = &NRF_USBD->EVENTS_ENDEPOUT[epNum];
	}

	pEp->PTR = (uint32_t)(uintptr_t)pBuffer;
	pEp->MAXCNT = len;
	if (epNum == 0U)
	{
		*pEnd = 0U;
		__DSB();
	}
	NRFX_USBD_EASYDMA_BUSY_REG = NRFX_USBD_EASYDMA_BUSY_REG_BUSY;
	*pTask = 1U;
	__DSB();
}

// Share the scheduler across ISR and foreground callers instead of expanding
// the DMA register setup at each call site.
static __attribute__((noinline)) void nRFUsbdStartQueuedDma(void)
{
	nRFUsbdQue_t *pQue = (nRFUsbdQue_t *)CFifoPeek(s_Usbd.hEp0Que);
	if (pQue == NULL)
	{
		if (nRFUsbdIsoStart != nullptr && nRFUsbdIsoStart())
			return;
		pQue = (nRFUsbdQue_t *)CFifoPeek(s_Usbd.hQue);
	}

	if (pQue != NULL)
		nRFUsbdStartDmaNow(pQue);
}


// Keep the DMA/suspend gate shared by submission and completion paths.
__attribute__((noinline)) void nRFUsbdResumeQueuedDmaLocked(void)
{
	const uint32_t gate = s_Usbd.Flags &
		(USBD_FLAG_HOST_RESUME | USBD_FLAG_SUSPENDED | USBD_FLAG_SUSPEND_PEND);
	if (nRFUsbdDmaActive() ||
		(gate & USBD_FLAG_HOST_RESUME) != 0U ||
		gate == USBD_FLAG_SUSPENDED)
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
static __attribute__((noinline)) void nRFUsbdQueXferDir(uint8_t EpNum, bool In, uint16_t Len)
{
	const uint32_t state = DisableInterrupt();
	hCFifo_t hQue = EpNum == 0U ? s_Usbd.hEp0Que : s_Usbd.hQue;
	nRFUsbdQue_t *pQue = (nRFUsbdQue_t *)CFifoPut(hQue);

	pQue->EpNum = EpNum;
	pQue->Dir = In ? NRFX_USBD_QUE_IN_BUFFER : NRFX_USBD_QUE_OUT;
	pQue->Len = Len;
	pQue->pBuffer = EpNum == 0U ? NULL :
		s_Usbd.EpReg[EpNum][In ? 1 : 0].pBuffer;

	nRFUsbdResumeQueuedDmaLocked();

	EnableInterrupt(state);
}

static void nRFUsbdQueInFifo(uint8_t EpNum, hCFifo_t hFifo, uint16_t Len)
{
	const uint32_t state = DisableInterrupt();
	nRFUsbdQue_t *pQue = (nRFUsbdQue_t *)CFifoPut(s_Usbd.hQue);
	int count = Len;
	uint8_t *pData = CFifoPeekMultiple(hFifo, &count);
	const uint32_t misalign = (uint32_t)(uintptr_t)pData & 3U;

	pQue->EpNum = EpNum;
	pQue->Len = (uint16_t)count;

	if (misalign != 0U)
	{
		const int repair = (int)(4U - misalign);
		if (count > repair)
			count = repair;
		pData = CFifoGetMultiple(hFifo, &count);
		pQue->Len = (uint16_t)count;
		pQue->Scratch = 0U;
		memcpy(&pQue->Scratch, pData, (size_t)count);
		pQue->Dir = NRFX_USBD_QUE_IN_SCRATCH;

		// Only the copied prefix is consumed now. The aligned run remains in
		// the TX FIFO until its host completion. A full queue defers that run.
		pQue = NULL;
		if (count == repair && CFifoAvail(s_Usbd.hQue) > 0)
		{
			count = s_Usbd.EpReg[EpNum][1].MaxPacketSize;
			if (CFifoPeekMultiple(hFifo, &count) != NULL)
			{
				pQue = (nRFUsbdQue_t *)CFifoPut(s_Usbd.hQue);
				pQue->EpNum = EpNum;
				pQue->Len = (uint16_t)count;
			}
		}
	}
	if (pQue != NULL)
	{
		pQue->hFifo = hFifo;
		pQue->Dir = NRFX_USBD_QUE_IN_FIFO;
	}

	nRFUsbdResumeQueuedDmaLocked();
	EnableInterrupt(state);
}

/** Remove one endpoint number without disturbing the order of other work. */
static void nRFUsbdQueRemoveEp(uint8_t EpNum)
{
	const uint32_t state = DisableInterrupt();
	if (EpNum == 0U)
	{
		CFifoFlush(s_Usbd.hEp0Que);
		EnableInterrupt(state);
		return;
	}

	const int count = CFifoUsed(s_Usbd.hQue);

	// Rotate exactly the entries that were present on entry. Kept entries go
	// back at the tail in the same order. Each get guarantees space for its
	// matching put.
	for (int i = 0; i < count; i++)
	{
		const nRFUsbdQue_t que =
			*(nRFUsbdQue_t *)CFifoGet(s_Usbd.hQue);
		if (que.EpNum == EpNum)
		{
			continue;
		}

		*(nRFUsbdQue_t *)CFifoPut(s_Usbd.hQue) = que;
	}

	EnableInterrupt(state);
}

#if NRFX_USBD_EP0_OUT_QUE
static void nRFUsbdQueueEp0Out(void)
{
	nRFUsbdXfer_t *pXfer = &s_Usbd.Ctrlr.Ep0[0];

	nRFUsbdQueXferDir(0U, false,
				 (uint16_t)(pXfer->TotalLen - pXfer->ActualLen));
}
#endif

static void nRFUsbdQueueEp0In(void)
{
	nRFUsbdXfer_t *pXfer = &s_Usbd.Ctrlr.Ep0[1];
	const uint16_t remaining =
		(uint16_t)(pXfer->TotalLen - pXfer->ActualLen);
	const uint16_t mps = NRFX_USBD_MAX_PACKET_SIZE;
	const uint16_t length = remaining < mps ? remaining : mps;

	nRFUsbdQueXferDir(0U, true, length);
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
	NRFX_USBD_EASYDMA_BUSY_REG = NRFX_USBD_EASYDMA_BUSY_REG_CLEAR;
}

static void nRFUsbdAbortEp0(void)
{
	nRFUsbdQueRemoveEp(0U);

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
	nRFUsbdTryRemoteWake();
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
		nRFUsbdEmitXfer(0U, pXfer->ActualLen, USB_CTRLR_XFER_SUCCESS);
	}
}

#if NRFX_USBD_EP0_OUT_QUE
static void nRFUsbdHandleEp0OutData(void)
{
	nRFUsbdXfer_t *pXfer = &s_Usbd.Ctrlr.Ep0[0];
	if (pXfer->ActualLen < pXfer->TotalLen || pXfer->TotalLen == 0U)
	{
		nRFUsbdQueueEp0Out();
	}
}
#endif



// A full AppEvt queue retains the completion and buffer ownership. The idle
// hook retries publication after foreground dispatch makes room.




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

#if NRFX_USBD_EP0_OUT_QUE
static void nRFUsbdProcessEp0OutData(uint32_t Evt, void *pContext)
{
	(void)Evt;
	(void)pContext;

	nRFUsbdHandleEp0OutData();
}
#endif

static void nRFUsbdProcessOutData(uint32_t Evt, void *pContext)
{
	const uint8_t epNum = (uint8_t)Evt;

	(void)pContext;

	// Only blocking endpoints reach AppEvt. Nonblocking OUT is queued for
	// EasyDMA directly from EPDATASTATUS in the ISR.
	nRFUsbEpRegisteredEvent(epNum, USB_CTRLR_EVT_DRDY, 0U,
		USB_CTRLR_XFER_SUCCESS);
}

static __attribute__((noinline)) void nRFUsbdQueueEp0Complete(bool Out, uint16_t Amount)
{
	const uint32_t evt = ((uint32_t)Amount << 8U) |
		(Out ? (uint32_t)NRFX_USBD_XFER_EVT_OUT : 0U);
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
	NRF_USBD->EVENTS_ENDEPIN[EpNum] = 0U;
	const uintptr_t source = NRF_USBD->EPIN[EpNum].PTR;
	if (source - (uintptr_t)s_QueMem < sizeof(s_QueMem))
	{
		// Scratch bytes were removed when queued. An already queued FIFO run
		// continues directly; otherwise a zero-length completion requests more.
		nRFUsbdQue_t *pQue = (nRFUsbdQue_t *)CFifoPeek(s_Usbd.hQue);
		if (pQue != NULL && pQue->EpNum == EpNum &&
			pQue->Dir == NRFX_USBD_QUE_IN_FIFO)
			return;
		Amount = 0U;
	}
	const uint32_t evt = ((uint32_t)Amount << 8U) | EpNum;
	(void)AppEvtHandlerQue(evt, NULL, nRFUsbdProcessInComplete);
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
		nRFUsbdEmit(&evt);
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

	UsbCtrlrEvt_t evt = {};
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

	nRFUsbdEmit(&evt);
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
					nRFUsbdEp0InProgram(p);
					nRFUsbdDmaStartLocked(
						&NRF_USBD->TASKS_STARTEPIN[0],
						&NRF_USBD->EVENTS_ENDEPIN[0]);
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
			if (nRFUsbdIsoFinishDma != nullptr)
			{
				(void)nRFUsbdIsoFinishDma(dmastatus);
			}
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
				nRFUsbEpRegisteredEvent(epNum, USB_CTRLR_EVT_XFER_CMPL,
					(uint16_t)NRF_USBD->EPOUT[epNum].AMOUNT,
					USB_CTRLR_XFER_SUCCESS);
			}

			if (NRF_USBD->EVENTS_EP0SETUP == 0U &&
				NRF_USBD->EVENTS_USBEVENT == 0U)
				nRFUsbdStartQueuedDma();
			break;
		}
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
#if NRFX_USBD_EP0_OUT_QUE
	if (NRF_USBD->EVENTS_EP0DATADONE != 0U)
	{
		NRF_USBD->EVENTS_EP0DATADONE = 0U;
		UsbdSync();
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

		// Remaining status bits are retained in EPDATASTATUS and serviced
		// by the pending interrupt they keep raised.
		NRF_USBD->EPDATASTATUS = servicedStatus;
		UsbdSync();
		nRFUsbdResumeQueuedDmaLocked();
	}


	if (NRF_USBD->EVENTS_SOF != 0U)
	{
		NRF_USBD->EVENTS_SOF = 0U;
		UsbdSync();

		nRFUsbdHandleSof();
	}

	nRFUsbdTryRemoteWake();

	// ENDEP released the shared EasyDMA channel above. ISO has priority when
	// it is open; ordinary CDC traffic avoids the ISO service path entirely.
	if (outEp != 0U)
	{
		const uint8_t epNum = outEp;
		nRFUsbEpReg_t *pReg = &s_Usbd.EpReg[epNum][0];
		if (pReg->bBlocking)
		{
			// EPDATASTATUS is cleared before DRDY may start another DMA.
			// UsbIntrf checks RX space and sets RxPending when it is full.
			nRFUsbEpRegisteredEvent(epNum, USB_CTRLR_EVT_DRDY, 0U,
				USB_CTRLR_XFER_SUCCESS);
		}
		else
		{
			nRFUsbdQue_t *pQue = (nRFUsbdQue_t *)CFifoPut(s_Usbd.hQue);
			if (pQue != NULL)
			{
				pQue->EpNum = epNum;
				pQue->Dir = 0U;
				pQue->Len = pReg->MaxPacketSize;
			}
			else
			{
				(void)AppEvtHandlerQue(epNum, NULL, nRFUsbdProcessOutData);
			}
		}
		nRFUsbdResumeQueuedDmaLocked();
	}

	if ((dmastatus & 0x01000100UL) != 0U &&
		(s_Usbd.Flags & (USBD_FLAG_ISO_IN_OPEN | USBD_FLAG_ISO_OUT_OPEN)) != 0U)
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

	// Registrations, transfer state and flags all restart from zero;
	// ResetState below rebuilds what must not be zero.
	memset(&s_Usbd, 0, sizeof(s_Usbd));

	s_Usbd.IntPrio = pCfg->IntPrio;
	s_Usbd.LowPowerSuspend = pCfg->bLowPowerSuspend;

	s_Usbd.hQue = CFifoInit(s_QueMem, sizeof(s_QueMem), sizeof(nRFUsbdQue_t),
					   false);
	s_Usbd.hEp0Que = CFifoInit(s_Ep0QueMem, sizeof(s_Ep0QueMem),
						  sizeof(nRFEPPkt_t), true);
	if (s_Usbd.hQue == NULL || s_Usbd.hEp0Que == NULL)
	{
		return false;
	}

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
		const uint32_t id = nrf_ficr_deviceid_get(NRF_FICR, word);
		for (int8_t shift = 28; shift >= 0; shift -= 4)
		{
			const unsigned digit = (id >> shift) & 15U;
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

	nRFUsbGetEpReg(EpAddr)->MaxPacketSize = MaxPacketSize;
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
	if (nRFUsbdDmaActive())
	{
		nRFUsbdDmaWait();
	}

	nRFUsbdEpHwEnable(epNum, in, false);
	NRF_USBD->EPDATASTATUS = 1UL << (epNum + (in ? 0U : 16U));
	if (!in)
	{
		NRF_USBD->SIZE.EPOUT[epNum] = 0;
	}
	nRFUsbGetEpReg(EpAddr)->MaxPacketSize = 0U;
	UsbdSync();
}

void UsbCtrlrEpCloseAll(int DevNo)
{
	nRFUsbdDmaWait();

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
	nRFUsbEpReg_t *pReg = nRFUsbGetEpReg(EpAddr);
	pReg->pBuffer = pBuffer;
	pReg->Handler = Handler;
	pReg->pContext = pContext;
	pReg->bBlocking = bBlocking;
}

bool UsbCtrlrEpXfer(int DevNo, uint8_t EpAddr, uint16_t Length)
{
	(void)DevNo;
	const uint8_t epNum = USB_ENDPADDR_NUM(EpAddr);
	if (epNum == NRFX_USBD_ISO_EP_NO)
	{
		return nRFUsbdIsoXfer != nullptr &&
			nRFUsbdIsoXfer(EpAddr, Length);
	}

	nRFUsbdQueXferDir(epNum, USB_ENDPADDR_IS_IN(EpAddr), Length);
	return true;
}

bool UsbCtrlrEpOutXfer(int DevNo, uint8_t EpNum, uint16_t Length)
{
	return UsbCtrlrEpXfer(DevNo, EpNum, Length);
}

bool UsbCtrlrEpInXfer(int DevNo, uint8_t EpNum, uint8_t *pBuffer,
						 uint16_t Length)
{
	(void)DevNo;
	if (pBuffer == NULL)
	{
		nRFUsbdQueInFifo(EpNum,
			(hCFifo_t)s_Usbd.EpReg[EpNum][1].pBuffer, Length);
	}
	else
	{
		const uint32_t state = DisableInterrupt();
		nRFUsbdQue_t *pQue = (nRFUsbdQue_t *)CFifoPut(s_Usbd.hQue);
		pQue->EpNum = EpNum;
		pQue->Dir = NRFX_USBD_QUE_IN_BUFFER;
		pQue->Len = Length;
		pQue->pBuffer = pBuffer;
		nRFUsbdResumeQueuedDmaLocked();
		EnableInterrupt(state);
	}
	return true;
}


bool UsbCtrlrEp0Xfer(int DevNo, uint8_t EpAddr, uint8_t *pBuffer,
						 uint16_t Length)
{
	(void)DevNo;
	if (USB_ENDPADDR_NUM(EpAddr) != 0U)
	{
		return false;
	}

	const bool in = USB_ENDPADDR_IS_IN(EpAddr);
	const bool reqIn =
		(NRF_USBD->BMREQUESTTYPE & USB_REQTYPE_MASK_DIR) != 0U;

	if (in && reqIn)
	{
		return UsbCtrlrEp0Send(DevNo, pBuffer, Length) == Length;
	}

	const uint32_t state = DisableInterrupt();
	nRFUsbdXfer_t *pXfer = &s_Usbd.Ctrlr.Ep0[in ? 1 : 0];

	pXfer->pBuffer = pBuffer;
	pXfer->TotalLen = Length;
	pXfer->ActualLen = 0U;

	if (Length == 0U && in != reqIn)
	{
		// Control status stage.
		if (!reqIn ||
			(NRF_USBD->SHORTS &
			 USBD_SHORTS_EP0DATADONE_EP0STATUS_Msk) == 0U)
		{
			nRFUsbdNoDmaTask(&NRF_USBD->TASKS_EP0STATUS);
		}
		EnableInterrupt(state);
		nRFUsbdEmitXfer(EpAddr, 0U, USB_CTRLR_XFER_SUCCESS);
		return true;
	}
	else if (in)
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

		nRFUsbdEp0InProgram((nRFEPPkt_t *)CFifoPeek(s_Usbd.hEp0Que));
		NRF_USBD->TASKS_STARTEPIN[0] = 1U;
	}

	return cnt;
}

void UsbCtrlrEpStall(int DevNo, uint8_t EpAddr)
{
	(void)DevNo;
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
	UsbdSync();
}

void UsbCtrlrEpClearStall(int DevNo, uint8_t EpAddr)
{
	(void)DevNo;
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
	UsbdSync();
}
