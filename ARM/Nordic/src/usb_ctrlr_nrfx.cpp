#if defined(USBHS_PRESENT)
#include "usb_ctrlr_nrfx_usbhs.inc"
#elif defined(USBD_PRESENT)

/**-------------------------------------------------------------------------
@file	usb_ctrlr_nrfx.cpp

@brief	USB controller for Nordic parts.

nRF52 uses the proven pre-ISO CFifo EasyDMA scheduler. The endpoint callback
API is the current event-driven API; blocking OUT endpoints receive DRDY and
nonblocking OUT endpoints start the registered DMA buffer immediately.

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
#include "nrf_erratas.h"
#include "hal/nrf_ficr.h"

#include "cfifo.h"
#include "coredev/interrupt.h"
#include "usb/usb.h"

/// Only DevNo 0 exists on every nRF part shipped so far.
static inline __attribute__((always_inline))
bool nRFUsbValidDevNo(int DevNo)
{
	return DevNo >= 0 && DevNo < USB_CTRLR_CNT;
}

enum
{
	NRF_USB_EP_COUNT = USB_EPIN_CNT(0) > USB_EPOUT_CNT(0) ?
		USB_EPIN_CNT(0) : USB_EPOUT_CNT(0),
};

typedef struct __nRF_Usb_Ep_Registration
{
	uint8_t *pBuffer;
	UsbCtrlrEpHandler_t Handler;
	void *pContext;
	bool bBlocking;
} nRFUsbEpReg_t;

static nRFUsbEpReg_t s_EpReg[NRF_USB_EP_COUNT][2];

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

static bool nRFUsbEpRegisteredEvent(uint8_t EpAddr, UsbCtrlrEvtType_t Event,
								 uint16_t Length, UsbCtrlrXferResult_t Result)
{
	const uint8_t epNum = USB_ENDPADDR_NUM(EpAddr);
	if (epNum == 0U || epNum >= NRF_USB_EP_COUNT)
	{
		return false;
	}

	nRFUsbEpReg_t *pReg = nRFUsbGetEpReg(EpAddr);
	if (pReg->Handler == NULL)
	{
		return false;
	}

	pReg->Handler(EpAddr, Event, Length, Result, pReg->pContext);
	return true;
}

#define NRFX_USBD_HAS_USBD					1

#if defined(SOFTDEVICE_PRESENT) || defined(S112) || defined(S113) || \
	defined(S132) || defined(S140) || defined(S145)
#define NRFX_USBD_SOFTDEVICE				1
#endif

#ifdef NRFX_USBD_SOFTDEVICE
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

#define NRFX_USBD_ERRATA_UNLOCK_REG		0x4006EC00UL
#define NRFX_USBD_ERRATA_UNLOCK_KEY		0x00009375UL
#define NRFX_USBD_ERRATA_171_REG			0x4006EC14UL
#define NRFX_USBD_ERRATA_187_REG			0x4006ED14UL
#define NRFX_USBD_ERRATA_166_REG_A			(NRF_USBD_BASE + 0x800UL)
#define NRFX_USBD_ERRATA_166_REG_B			(NRF_USBD_BASE + 0x804UL)
#define NRFX_USBD_REG32(a)					(*(volatile uint32_t *)(a))

static UsbCtrlrCfg_t s_UsbdCfg;
static bool s_UsbdInitialized = false;
static bool s_UsbdStarted = false;
static bool s_UsbdXtalHeld = false;
static bool s_UsbdVbusLast = false;

static inline __attribute__((always_inline)) bool nRFUsbdDmaActive(void);

#ifdef NRFX_USBD_SOFTDEVICE
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
#ifdef NRFX_USBD_SOFTDEVICE
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
#ifdef NRFX_USBD_SOFTDEVICE
	if (UsbdSdRunning())
	{
		(void)sd_clock_hfclk_release();
		return;
	}
#endif
}

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

static bool s_LowPowerExitPending = false;

static void UsbdLowPowerExitFinish(void)
{
	if (!s_LowPowerExitPending ||
		(NRF_USBD->EVENTCAUSE & USBD_EVENTCAUSE_READY_Msk) == 0U)
	{
		return;
	}

	NRF_USBD->EVENTCAUSE = USBD_EVENTCAUSE_READY_Msk;

	if (nrf52_errata_171())
	{
		UsbdErrataWrite(NRFX_USBD_ERRATA_171_REG, 0x00000000UL);
	}

	s_LowPowerExitPending = false;
}

static void UsbdLowPowerExit(void)
{
	UsbdLowPowerExitFinish();

	if (s_LowPowerExitPending ||
		NRF_USBD->LOWPOWER == USBD_LOWPOWER_LOWPOWER_ForceNormal)
	{
		return;
	}

	NRF_USBD->EVENTCAUSE = USBD_EVENTCAUSE_READY_Msk;
	NRF_USBD->LOWPOWER = USBD_LOWPOWER_LOWPOWER_ForceNormal;

	if (nrf52_errata_171())
	{
		UsbdErrataWrite(NRFX_USBD_ERRATA_171_REG, 0x000000C0UL);
	}

	s_LowPowerExitPending = true;
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

	NRF_USBD->EVENTS_USBRESET = 0;
	NRF_USBD->INTENCLR = NRF_USBD->INTEN;
	NRF_USBD->INTENSET = USBD_INTEN_USBRESET_Msk;
	NVIC_ClearPendingIRQ(USBD_IRQn);
	return true;
}

static void UsbdStopCtrlr(void)
{
	NRF_USBD->INTEN = 0;
	NRF_USBD->USBPULLUP = 0;
	NRF_USBD->ENABLE = 0;
	__ISB();
	__DSB();
}

static bool nRFUsbVbusDetected(void)
{
	return (NRF_POWER->USBREGSTATUS & POWER_USBREGSTATUS_VBUSDETECT_Msk) != 0;
}

static size_t nRFUsbSerial(char *pBuff, size_t BuffLen)
{
	static const char hex[] = "0123456789ABCDEF";
	size_t cnt = 0;

	if (pBuff == nullptr || BuffLen == 0)
	{
		return 0;
	}

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

	memcpy(&s_UsbdCfg, pCfg, sizeof(UsbCtrlrCfg_t));
	s_UsbdInitialized = true;
	s_UsbdStarted = false;
	s_UsbdVbusLast = nRFUsbVbusDetected();
	return true;
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

	if (nRFUsbVbusDetected() == false || UsbdXtalRequest() == false)
	{
		return false;
	}

	s_UsbdXtalHeld = true;
	NVIC_SetPriority(USBD_IRQn, s_UsbdCfg.IntPrio);

	if (UsbdStartCtrlr() == false)
	{
		UsbdXtalRelease();
		s_UsbdXtalHeld = false;
		return false;
	}

	s_UsbdStarted = true;
	s_UsbdVbusLast = true;
	return true;
}

static void nRFUsbPowerStop(void)
{
	if (s_UsbdStarted == false)
	{
		return;
	}

	NVIC_DisableIRQ(USBD_IRQn);
	UsbdStopCtrlr();

	if (s_UsbdXtalHeld)
	{
		UsbdXtalRelease();
		s_UsbdXtalHeld = false;
	}

	s_UsbdStarted = false;
}

static void nRFUsbPowerProcess(void)
{
	if (s_UsbdInitialized == false)
	{
		return;
	}

	const bool dmaActive = nRFUsbdDmaActive();
	if (!dmaActive && s_LowPowerExitPending)
	{
		UsbdLowPowerExitFinish();
	}
	else if (!dmaActive && s_UsbdStarted &&
			 s_UsbdCfg.bLowPowerSuspend == false &&
			 NRF_USBD->LOWPOWER != USBD_LOWPOWER_LOWPOWER_ForceNormal)
	{
		UsbdLowPowerExit();
	}

	const bool vbus = nRFUsbVbusDetected();
	if (vbus != s_UsbdVbusLast)
	{
		s_UsbdVbusLast = vbus;
	}
}

typedef bool atomic_flag;
typedef bool atomic_bool;
typedef uint_fast8_t atomic_uint_fast8_t;

#define ATOMIC_FLAG_INIT false
#define atomic_load(p) __atomic_load_n((p), __ATOMIC_SEQ_CST)
#define atomic_store(p, v) __atomic_store_n((p), (v), __ATOMIC_SEQ_CST)
#define atomic_exchange(p, v) __atomic_exchange_n((p), (v), __ATOMIC_SEQ_CST)
#define atomic_compare_exchange_strong(p, expected, desired) \
	__atomic_compare_exchange_n((p), (expected), (desired), false, \
		__ATOMIC_SEQ_CST, __ATOMIC_SEQ_CST)
#define atomic_flag_test_and_set(p) __atomic_exchange_n((p), true, __ATOMIC_SEQ_CST)
#define atomic_flag_clear(p) __atomic_store_n((p), false, __ATOMIC_SEQ_CST)

enum
{
	NRFX_USBD_EP_COUNT = 8,
	NRFX_USBD_MAX_PACKET_SIZE = 64,
	NRFX_USBD_DMA_EP_NONE = 0xFFU,
};

#define NRFX_USBD_IRQ_EVENT_COUNT	(USBD_INTEN_EPDATA_Pos + 1)
#define NRFUSBD_IRQ_MASK \
	((uint32_t)((1ULL << NRFX_USBD_IRQ_EVENT_COUNT) - 1ULL))
#define NRFX_USBD_ERRATA_199_REG \
	(*((volatile uint32_t *)0x40027C1CUL))

typedef struct __nRF_Usbd_Xfer
{
	uint8_t *pBuffer;
	uint16_t TotalLen;
	volatile uint16_t ActualLen;
	uint16_t Mps;
	volatile bool DataReceived;
	volatile bool Started;
} nRFUsbdXfer_t;

typedef struct __nRF_Usbd_Ctrlr
{
	nRFUsbdXfer_t Xfer[NRFX_USBD_EP_COUNT][2];
	UsbCtrlrEvtHandler_t EvtHandler;
	void *pContext;
	bool SofEnabled;
	bool SetupDirIn;
} nRFUsbdCtrlr_t;

static nRFUsbdCtrlr_t s_Ctrlr;
static atomic_flag s_DmaRunning = ATOMIC_FLAG_INIT;
static atomic_uint_fast8_t s_DmaEpAddr;

#define NRFUSBD_QUE_DEPTH			(NRFX_USBD_EP_COUNT * 2)

typedef struct __nRF_Usbd_Que
{
	uint8_t EpAddr;
	uint16_t Len;
	uint8_t *pBuffer;
} nRFUsbdQue_t;

alignas(4) static uint8_t s_QueMem[
	CFIFO_TOTAL_MEMSIZE(NRFUSBD_QUE_DEPTH, sizeof(nRFUsbdQue_t))];
static hCFifo_t s_hQue;
alignas(4) static uint8_t s_Ep0Bounce[NRFX_USBD_MAX_PACKET_SIZE];

static atomic_bool s_PendingEp0Status;
static atomic_bool s_PendingEp0RcvOut;
static atomic_bool s_BusSuspended;
static atomic_bool s_SuspendPending;
static atomic_bool s_RemoteWakePending;
static atomic_bool s_HostResumePending;
static atomic_bool s_MacAwake;

static inline __attribute__((always_inline)) bool nRFUsbdDmaActive(void)
{
	return (uint8_t)atomic_load(&s_DmaEpAddr) != NRFX_USBD_DMA_EP_NONE;
}

static inline __attribute__((always_inline))
uint8_t nRFUsbdDir(uint8_t EpAddr)
{
	return USB_ENDPADDR_IS_IN(EpAddr) ? 1U : 0U;
}

static inline __attribute__((always_inline))
bool nRFUsbdDataIn(uint8_t EpAddr)
{
	return USB_ENDPADDR_IS_IN(EpAddr) && USB_ENDPADDR_NUM(EpAddr) != 0U;
}

static inline __attribute__((always_inline))
nRFUsbdXfer_t *nRFUsbdGetXfer(uint8_t EpAddr)
{
	return &s_Ctrlr.Xfer[USB_ENDPADDR_NUM(EpAddr)][nRFUsbdDir(EpAddr)];
}

static inline __attribute__((always_inline))
uint32_t nRFUsbdLowestBit(uint32_t Mask)
{
	return 31U - (uint32_t)__CLZ(Mask & (uint32_t)(0U - Mask));
}

static uint32_t nRFUsbdDmaEndMask(uint8_t EpAddr)
{
	const uint8_t epNum = USB_ENDPADDR_NUM(EpAddr);
	return USB_ENDPADDR_IS_IN(EpAddr) ?
		(1UL << (USBD_INTEN_ENDEPIN0_Pos + epNum)) :
		(1UL << (USBD_INTEN_ENDEPOUT0_Pos + epNum));
}

static volatile uint32_t *nRFUsbdDmaEndEvent(uint8_t EpAddr)
{
	const uint8_t epNum = USB_ENDPADDR_NUM(EpAddr);
	return USB_ENDPADDR_IS_IN(EpAddr) ?
		&NRF_USBD->EVENTS_ENDEPIN[epNum] :
		&NRF_USBD->EVENTS_ENDEPOUT[epNum];
}

static void nRFUsbdEmit(const UsbCtrlrEvt_t *pEvt)
{
	if (s_Ctrlr.EvtHandler != NULL)
	{
		s_Ctrlr.EvtHandler(0, pEvt, s_Ctrlr.pContext);
	}
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
	if (nRFUsbEpRegisteredEvent(EpAddr, USB_CTRLR_EVT_XFER_CMPL,
		Length, Result))
	{
		return;
	}

	UsbCtrlrEvt_t evt = {};
	evt.Type = USB_CTRLR_EVT_XFER_CMPL;
	evt.Xfer.EpAddr = EpAddr;
	evt.Xfer.Length = Length;
	evt.Xfer.Result = Result;
	nRFUsbdEmit(&evt);
}

static void nRFUsbdDmaEndIntEnable(uint8_t EpAddr)
{
	const uint32_t primask = __get_PRIMASK();
	__disable_irq();
	if ((uint8_t)atomic_load(&s_DmaEpAddr) == EpAddr)
	{
		NRF_USBD->INTENSET = nRFUsbdDmaEndMask(EpAddr);
	}
	__set_PRIMASK(primask);
}

static void nRFUsbdDmaReclaim(void)
{
	uint_fast8_t epAddr = atomic_load(&s_DmaEpAddr);
	if ((uint8_t)epAddr == NRFX_USBD_DMA_EP_NONE ||
		!nRFUsbdDataIn((uint8_t)epAddr))
	{
		return;
	}

	volatile uint32_t *pEvent = nRFUsbdDmaEndEvent((uint8_t)epAddr);
	if (*pEvent == 0U)
	{
		return;
	}

	if (!atomic_compare_exchange_strong(&s_DmaEpAddr, &epAddr,
		(uint_fast8_t)NRFX_USBD_DMA_EP_NONE))
	{
		return;
	}

	NRF_USBD->INTENCLR = nRFUsbdDmaEndMask((uint8_t)epAddr);
	*pEvent = 0;
	__ISB();
	__DSB();

	if (nrf52_errata_199())
	{
		NRFX_USBD_ERRATA_199_REG = 0x00000000UL;
	}
	atomic_flag_clear(&s_DmaRunning);
}

static void nRFUsbdDmaRelease(void)
{
	const uint32_t primask = __get_PRIMASK();
	__disable_irq();
	const uint8_t epAddr = (uint8_t)atomic_load(&s_DmaEpAddr);
	if (epAddr != NRFX_USBD_DMA_EP_NONE && nRFUsbdDataIn(epAddr))
	{
		NRF_USBD->INTENCLR = nRFUsbdDmaEndMask(epAddr);
	}

	if (nrf52_errata_199())
	{
		NRFX_USBD_ERRATA_199_REG = 0x00000000UL;
	}

	atomic_store(&s_DmaEpAddr, NRFX_USBD_DMA_EP_NONE);
	atomic_flag_clear(&s_DmaRunning);
	__ISB();
	__DSB();
	__set_PRIMASK(primask);
}

static void nRFUsbdDmaStart(volatile uint32_t *pTask, uint8_t EpAddr)
{
	const uint32_t primask = __get_PRIMASK();
	__disable_irq();
	*nRFUsbdDmaEndEvent(EpAddr) = 0;
	__ISB();
	__DSB();

	if (nrf52_errata_199())
	{
		NRFX_USBD_ERRATA_199_REG = 0x00000082UL;
	}

	atomic_store(&s_DmaEpAddr, EpAddr);
	*pTask = 1;
	__ISB();
	__DSB();
	__set_PRIMASK(primask);
}

static void nRFUsbdDmaWait(void)
{
	for (;;)
	{
		const uint8_t epAddr = (uint8_t)atomic_load(&s_DmaEpAddr);
		if (epAddr == NRFX_USBD_DMA_EP_NONE)
		{
			return;
		}
		if (NRF_USBD->EVENTS_USBRESET != 0U)
		{
			nRFUsbdDmaRelease();
			return;
		}
		volatile uint32_t *pEvent = nRFUsbdDmaEndEvent(epAddr);
		if (*pEvent == 0U)
		{
			continue;
		}
		*pEvent = 0;
		__ISB();
		__DSB();
		nRFUsbdDmaRelease();
	}
}

static void nRFUsbdNoDmaTask(volatile uint32_t *pTask)
{
	*pTask = 1;
	__ISB();
	__DSB();
	atomic_flag_clear(&s_DmaRunning);
}

static void nRFUsbdEp0StatusNow(void)
{
	const uint8_t epAddr = s_Ctrlr.SetupDirIn ?
		USB_ENDPADDR_DIR_OUT : USB_ENDPADDR_DIR_IN;
	nRFUsbdXfer_t *pXfer = nRFUsbdGetXfer(epAddr);
	NRF_USBD->TASKS_EP0STATUS = 1;
	__ISB();
	__DSB();
	if (pXfer->Started && pXfer->TotalLen == 0U)
	{
		pXfer->Started = false;
		nRFUsbdEmitXfer(epAddr, 0, USB_CTRLR_XFER_SUCCESS);
	}
	atomic_flag_clear(&s_DmaRunning);
}

static bool nRFUsbdStartDmaNow(const nRFUsbdQue_t *pQue)
{
	const uint8_t epNum = USB_ENDPADDR_NUM(pQue->EpAddr);
	const bool isIn = USB_ENDPADDR_IS_IN(pQue->EpAddr);
	nRFUsbdXfer_t *pXfer = &s_Ctrlr.Xfer[epNum][isIn ? 1 : 0];

	if (!pXfer->Started || pXfer->ActualLen > pXfer->TotalLen)
	{
		return false;
	}

	if (isIn)
	{
		NRF_USBD->EPIN[epNum].PTR = (uint32_t)(uintptr_t)pQue->pBuffer;
		NRF_USBD->EPIN[epNum].MAXCNT = pQue->Len;
		nRFUsbdDmaStart(&NRF_USBD->TASKS_STARTEPIN[epNum], pQue->EpAddr);
	}
	else
	{
		const uint16_t received = (uint16_t)NRF_USBD->SIZE.EPOUT[epNum];
		const uint16_t len = received < pQue->Len ? received : pQue->Len;
		NRF_USBD->EPOUT[epNum].PTR = (uint32_t)(uintptr_t)pQue->pBuffer;
		NRF_USBD->EPOUT[epNum].MAXCNT = len;
		nRFUsbdDmaStart(&NRF_USBD->TASKS_STARTEPOUT[epNum], pQue->EpAddr);
	}
	return true;
}

static void nRFUsbdServicePending(void)
{
	if (atomic_load(&s_HostResumePending) ||
		(atomic_load(&s_BusSuspended) && !atomic_load(&s_SuspendPending)))
	{
		return;
	}

	for (;;)
	{
		if (atomic_flag_test_and_set(&s_DmaRunning))
		{
			nRFUsbdDmaReclaim();
			if (atomic_flag_test_and_set(&s_DmaRunning))
			{
				const uint8_t epAddr = (uint8_t)atomic_load(&s_DmaEpAddr);
				if (epAddr != NRFX_USBD_DMA_EP_NONE && nRFUsbdDataIn(epAddr))
				{
					nRFUsbdDmaEndIntEnable(epAddr);
				}
				return;
			}
		}

		if (atomic_exchange(&s_PendingEp0Status, false))
		{
			nRFUsbdEp0StatusNow();
			continue;
		}

		if (atomic_exchange(&s_PendingEp0RcvOut, false))
		{
			nRFUsbdNoDmaTask(&NRF_USBD->TASKS_EP0RCVOUT);
			continue;
		}

		nRFUsbdQue_t que;
		const uint32_t state = DisableInterrupt();
		nRFUsbdQue_t *pHead = (nRFUsbdQue_t *)CFifoGet(s_hQue);
		const bool haveQue = pHead != NULL;
		if (haveQue)
		{
			que = *pHead;
		}
		EnableInterrupt(state);

		if (haveQue)
		{
			const uint8_t epNum = USB_ENDPADDR_NUM(que.EpAddr);
			if (nRFUsbdStartDmaNow(&que))
			{
				return;
			}

			if (USB_ENDPADDR_IS_IN(que.EpAddr))
			{
				nRFUsbdXfer_t *pXfer = &s_Ctrlr.Xfer[epNum][1];
				if (pXfer->Started)
				{
					pXfer->Started = false;
					atomic_flag_clear(&s_DmaRunning);
					nRFUsbdEmitXfer(que.EpAddr, pXfer->ActualLen,
						USB_CTRLR_XFER_FAILED);
					continue;
				}
			}
			else
			{
				s_Ctrlr.Xfer[epNum][0].DataReceived = true;
			}

			atomic_flag_clear(&s_DmaRunning);
			continue;
		}

		atomic_flag_clear(&s_DmaRunning);
		if (CFifoUsed(s_hQue) > 0 ||
			atomic_load(&s_PendingEp0Status) ||
			atomic_load(&s_PendingEp0RcvOut))
		{
			continue;
		}
		return;
	}
}

static inline __attribute__((always_inline))
bool nRFUsbdDeferFromInterrupt(void)
{
	const uint32_t exception = __get_IPSR();
	if (exception == 0U)
	{
		return false;
	}
	if (exception != (uint32_t)USBD_IRQn + 16U)
	{
		NVIC_SetPendingIRQ(USBD_IRQn);
	}
	return true;
}

static void nRFUsbdQueXfer(uint8_t EpAddr, uint8_t *pBuffer, uint16_t Len)
{
	const uint32_t state = DisableInterrupt();
	nRFUsbdQue_t *pQue = (nRFUsbdQue_t *)CFifoPut(s_hQue);
	pQue->EpAddr = EpAddr;
	pQue->Len = Len;
	pQue->pBuffer = pBuffer;
	EnableInterrupt(state);
}

static void nRFUsbdQueRemoveEp(uint8_t EpNum)
{
	const uint32_t state = DisableInterrupt();
	const int count = CFifoUsed(s_hQue);
	for (int i = 0; i < count; i++)
	{
		const nRFUsbdQue_t que = *(nRFUsbdQue_t *)CFifoGet(s_hQue);
		if (USB_ENDPADDR_NUM(que.EpAddr) != EpNum)
		{
			*(nRFUsbdQue_t *)CFifoPut(s_hQue) = que;
		}
	}
	EnableInterrupt(state);
}

static void nRFUsbdQueueOut(uint8_t EpNum)
{
	nRFUsbdXfer_t *pXfer = &s_Ctrlr.Xfer[EpNum][0];
	uint8_t *pBuffer = EpNum == 0U ? s_Ep0Bounce : pXfer->pBuffer;
	nRFUsbdQueXfer(EpNum, pBuffer,
		(uint16_t)(pXfer->TotalLen - pXfer->ActualLen));
	if (!nRFUsbdDeferFromInterrupt())
	{
		nRFUsbdServicePending();
	}
}

static void nRFUsbdQueueIn(uint8_t EpNum)
{
	nRFUsbdXfer_t *pXfer = &s_Ctrlr.Xfer[EpNum][1];
	const uint16_t remaining =
		(uint16_t)(pXfer->TotalLen - pXfer->ActualLen);
	const uint16_t length = remaining < pXfer->Mps ? remaining : pXfer->Mps;
	uint8_t *pBuffer = pXfer->pBuffer;
	if (EpNum == 0U && length > 0U)
	{
		memcpy(s_Ep0Bounce, pBuffer, length);
		pBuffer = s_Ep0Bounce;
	}
	nRFUsbdQueXfer((uint8_t)(EpNum | USB_ENDPADDR_DIR_IN), pBuffer, length);
	if (!nRFUsbdDeferFromInterrupt())
	{
		nRFUsbdServicePending();
	}
}

static void nRFUsbdQueueEp0Status(void)
{
	atomic_store(&s_PendingEp0Status, true);
	if (!nRFUsbdDeferFromInterrupt())
	{
		nRFUsbdServicePending();
	}
}

static void nRFUsbdQueueEp0RcvOut(void)
{
	atomic_store(&s_PendingEp0RcvOut, true);
	if (!nRFUsbdDeferFromInterrupt())
	{
		nRFUsbdServicePending();
	}
}

static void nRFUsbdResetState(void)
{
	memset(s_Ctrlr.Xfer, 0, sizeof(s_Ctrlr.Xfer));
	s_Ctrlr.Xfer[0][0].Mps = NRFX_USBD_MAX_PACKET_SIZE;
	s_Ctrlr.Xfer[0][1].Mps = NRFX_USBD_MAX_PACKET_SIZE;
	s_Ctrlr.SofEnabled = false;
	s_Ctrlr.SetupDirIn = false;
	CFifoFlush(s_hQue);
	atomic_store(&s_PendingEp0Status, false);
	atomic_store(&s_PendingEp0RcvOut, false);
	atomic_store(&s_BusSuspended, false);
	atomic_store(&s_SuspendPending, false);
	atomic_store(&s_RemoteWakePending, false);
	atomic_store(&s_HostResumePending, false);
	atomic_store(&s_MacAwake, true);
	atomic_store(&s_DmaEpAddr, NRFX_USBD_DMA_EP_NONE);
	atomic_flag_clear(&s_DmaRunning);
	if (nrf52_errata_199())
	{
		NRFX_USBD_ERRATA_199_REG = 0x00000000UL;
	}
}

static void nRFUsbdAbortEp0(void)
{
	const uint8_t dmaEpAddr = (uint8_t)atomic_load(&s_DmaEpAddr);
	if (dmaEpAddr != NRFX_USBD_DMA_EP_NONE && USB_ENDPADDR_NUM(dmaEpAddr) == 0U)
	{
		nRFUsbdDmaWait();
	}
	nRFUsbdQueRemoveEp(0U);
	atomic_store(&s_PendingEp0Status, false);
	atomic_store(&s_PendingEp0RcvOut, false);
	for (uint8_t dir = 0; dir < 2U; dir++)
	{
		nRFUsbdXfer_t *pXfer = &s_Ctrlr.Xfer[0][dir];
		pXfer->pBuffer = NULL;
		pXfer->TotalLen = 0;
		pXfer->ActualLen = 0;
		pXfer->Mps = NRFX_USBD_MAX_PACKET_SIZE;
		pXfer->DataReceived = false;
		pXfer->Started = false;
	}
	NRF_USBD->EVENTS_ENDEPIN[0] = 0;
	NRF_USBD->EVENTS_ENDEPOUT[0] = 0;
	NRF_USBD->EPDATASTATUS = (1UL << 0) | (1UL << 16);
	__ISB();
	__DSB();
}

static void nRFUsbdHostResumeDetected(void);

static void nRFUsbdTryEnterLowPower(void)
{
	if (!s_UsbdCfg.bLowPowerSuspend ||
		!atomic_load(&s_BusSuspended) ||
		!atomic_load(&s_SuspendPending) ||
		atomic_load(&s_RemoteWakePending) ||
		atomic_load(&s_HostResumePending) ||
		(uint8_t)atomic_load(&s_DmaEpAddr) != NRFX_USBD_DMA_EP_NONE ||
		CFifoUsed(s_hQue) > 0 ||
		atomic_load(&s_PendingEp0Status) ||
		atomic_load(&s_PendingEp0RcvOut))
	{
		return;
	}

	if ((NRF_USBD->EVENTCAUSE & USBD_EVENTCAUSE_RESUME_Msk) != 0U ||
		NRF_USBD->EVENTS_SOF != 0U)
	{
		nRFUsbdHostResumeDetected();
		return;
	}

	if (atomic_flag_test_and_set(&s_DmaRunning))
	{
		return;
	}

	if (!atomic_load(&s_BusSuspended) ||
		!atomic_load(&s_SuspendPending) ||
		atomic_load(&s_RemoteWakePending) ||
		atomic_load(&s_HostResumePending) ||
		(uint8_t)atomic_load(&s_DmaEpAddr) != NRFX_USBD_DMA_EP_NONE ||
		CFifoUsed(s_hQue) > 0 ||
		atomic_load(&s_PendingEp0Status) ||
		atomic_load(&s_PendingEp0RcvOut))
	{
		atomic_flag_clear(&s_DmaRunning);
		return;
	}

	if ((NRF_USBD->EVENTCAUSE & USBD_EVENTCAUSE_RESUME_Msk) != 0U ||
		NRF_USBD->EVENTS_SOF != 0U)
	{
		atomic_flag_clear(&s_DmaRunning);
		nRFUsbdHostResumeDetected();
		return;
	}

	atomic_store(&s_MacAwake, false);
	NRF_USBD->LOWPOWER =
		USBD_LOWPOWER_LOWPOWER_LowPower << USBD_LOWPOWER_LOWPOWER_Pos;
	(void)NRF_USBD->LOWPOWER;
	__ISB();
	__DSB();

	if ((NRF_USBD->EVENTCAUSE & USBD_EVENTCAUSE_RESUME_Msk) != 0U ||
		NRF_USBD->EVENTS_SOF != 0U)
	{
		atomic_flag_clear(&s_DmaRunning);
		nRFUsbdHostResumeDetected();
		return;
	}

	if (atomic_load(&s_RemoteWakePending) || !atomic_load(&s_SuspendPending))
	{
		atomic_store(&s_SuspendPending, false);
		NRF_USBD->LOWPOWER =
			USBD_LOWPOWER_LOWPOWER_ForceNormal << USBD_LOWPOWER_LOWPOWER_Pos;
		__ISB();
		__DSB();
		atomic_flag_clear(&s_DmaRunning);
		return;
	}

	atomic_store(&s_SuspendPending, false);
	atomic_flag_clear(&s_DmaRunning);
}

static void nRFUsbdTryRemoteWake(void)
{
	if (!atomic_load(&s_RemoteWakePending) ||
		!atomic_load(&s_BusSuspended) ||
		atomic_load(&s_HostResumePending) ||
		!atomic_load(&s_MacAwake) ||
		NRF_USBD->LOWPOWER !=
			(USBD_LOWPOWER_LOWPOWER_ForceNormal << USBD_LOWPOWER_LOWPOWER_Pos))
	{
		return;
	}

	if (atomic_flag_test_and_set(&s_DmaRunning))
	{
		return;
	}

	if (!atomic_load(&s_RemoteWakePending) ||
		!atomic_load(&s_BusSuspended) ||
		atomic_load(&s_HostResumePending) ||
		!atomic_load(&s_MacAwake) ||
		(uint8_t)atomic_load(&s_DmaEpAddr) != NRFX_USBD_DMA_EP_NONE)
	{
		atomic_flag_clear(&s_DmaRunning);
		return;
	}

	atomic_store(&s_RemoteWakePending, false);
	NRF_USBD->DPDMVALUE = USBD_DPDMVALUE_STATE_Resume;
	NRF_USBD->TASKS_DPDMDRIVE = 1;
	__ISB();
	__DSB();
	atomic_flag_clear(&s_DmaRunning);

	if ((NRF_USBD->INTEN & USBD_INTEN_SOF_Msk) == 0U)
	{
		NRF_USBD->EVENTS_SOF = 0;
	}
	NRF_USBD->INTENSET = USBD_INTENSET_SOF_Msk;
}

static void nRFUsbdHostResumeDetected(void)
{
	if (!atomic_load(&s_BusSuspended))
	{
		return;
	}

	atomic_store(&s_BusSuspended, false);
	atomic_store(&s_SuspendPending, false);
	atomic_store(&s_RemoteWakePending, false);

	if (!atomic_load(&s_MacAwake) ||
		NRF_USBD->LOWPOWER !=
			(USBD_LOWPOWER_LOWPOWER_ForceNormal << USBD_LOWPOWER_LOWPOWER_Pos))
	{
		atomic_store(&s_HostResumePending, true);
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

	atomic_store(&s_HostResumePending, false);
	nRFUsbdEmitSimple(USB_CTRLR_EVT_RESUME);
}

static void nRFUsbdWakeAllowed(void)
{
	atomic_store(&s_MacAwake, true);
	if (atomic_exchange(&s_HostResumePending, false))
	{
		nRFUsbdEmitSimple(USB_CTRLR_EVT_RESUME);
		return;
	}
	nRFUsbdTryRemoteWake();
}

static bool nRFUsbRegInit(UsbCtrlrEvtHandler_t EvtHandler, void *pContext)
{
	s_hQue = CFifoInit(s_QueMem, sizeof(s_QueMem), sizeof(nRFUsbdQue_t), false);
	if (s_hQue == NULL)
	{
		return false;
	}
	s_Ctrlr.EvtHandler = EvtHandler;
	s_Ctrlr.pContext = pContext;
	nRFUsbdResetState();
	return true;
}

static void nRFUsbRegStop(void)
{
	nRFUsbdDmaWait();
	nRFUsbdResetState();
}

static void nRFUsbRegIntEnable(void) { NVIC_EnableIRQ(USBD_IRQn); }
static void nRFUsbRegIntDisable(void) { NVIC_DisableIRQ(USBD_IRQn); }
static void nRFUsbRegConnect(void) { NRF_USBD->USBPULLUP = 1; }
static void nRFUsbRegDisconnect(void) { NRF_USBD->USBPULLUP = 0; }

static void nRFUsbRegRemoteWakeup(void)
{
	if (!atomic_load(&s_BusSuspended) || atomic_load(&s_HostResumePending))
	{
		return;
	}
	atomic_store(&s_SuspendPending, false);
	atomic_store(&s_RemoteWakePending, true);
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

static void nRFUsbRegSofEnable(bool Enable)
{
	s_Ctrlr.SofEnabled = Enable;
	if (Enable)
	{
		NRF_USBD->EVENTS_SOF = 0;
		NRF_USBD->INTENSET = USBD_INTENSET_SOF_Msk;
	}
	else if (!atomic_load(&s_BusSuspended))
	{
		NRF_USBD->INTENCLR = USBD_INTENCLR_SOF_Msk;
	}
}

static void nRFUsbRegSetAddress(uint8_t Address) { (void)Address; }

static bool nRFUsbRegEpOpen(const UsbEndPointDesc_t *pDesc)
{
	if (pDesc == NULL)
	{
		return false;
	}
	const uint8_t epAddr = pDesc->bEndpointAddress;
	const uint8_t epNum = USB_ENDPADDR_NUM(epAddr);
	const uint8_t type = pDesc->bmAttributes & 0x03U;
	if (epNum == 0 || epNum >= NRFX_USBD_EP_COUNT ||
		(type != USB_ENDPATT_TRANS_BULK && type != USB_ENDPATT_TRANS_INT) ||
		pDesc->wMaxPacketSize == 0 ||
		pDesc->wMaxPacketSize > NRFX_USBD_MAX_PACKET_SIZE)
	{
		return false;
	}

	nRFUsbdXfer_t *pXfer = nRFUsbdGetXfer(epAddr);
	pXfer->Mps = pDesc->wMaxPacketSize;
	pXfer->Started = false;
	pXfer->DataReceived = false;

	if (USB_ENDPADDR_IS_IN(epAddr))
	{
		NRF_USBD->EVENTS_ENDEPIN[epNum] = 0;
		NRF_USBD->EPINEN |= (1UL << epNum);
	}
	else
	{
		NRF_USBD->EVENTS_ENDEPOUT[epNum] = 0;
		NRF_USBD->INTENSET = (1UL << (USBD_INTEN_ENDEPOUT0_Pos + epNum));
		NRF_USBD->EPOUTEN |= (1UL << epNum);
		NRF_USBD->SIZE.EPOUT[epNum] = 0;
	}

	NRF_USBD->EPSTALL =
		(USBD_EPSTALL_STALL_UnStall << USBD_EPSTALL_STALL_Pos) | epAddr;
	NRF_USBD->DTOGGLE =
		(USBD_DTOGGLE_VALUE_Data0 << USBD_DTOGGLE_VALUE_Pos) | epAddr;
	__ISB();
	__DSB();
	return true;
}

static void nRFUsbRegEpClose(uint8_t EpAddr)
{
	const uint8_t epNum = USB_ENDPADDR_NUM(EpAddr);
	if (epNum == 0 || epNum >= NRFX_USBD_EP_COUNT)
	{
		return;
	}
	if ((uint8_t)atomic_load(&s_DmaEpAddr) == EpAddr)
	{
		nRFUsbdDmaWait();
	}
	nRFUsbdXfer_t *pXfer = nRFUsbdGetXfer(EpAddr);
	if (USB_ENDPADDR_IS_IN(EpAddr))
	{
		NRF_USBD->INTENCLR = (1UL << (USBD_INTEN_ENDEPIN0_Pos + epNum));
		NRF_USBD->EPINEN &= ~(1UL << epNum);
		NRF_USBD->EVENTS_ENDEPIN[epNum] = 0;
		NRF_USBD->EPDATASTATUS = (1UL << epNum);
	}
	else
	{
		NRF_USBD->INTENCLR = (1UL << (USBD_INTEN_ENDEPOUT0_Pos + epNum));
		NRF_USBD->EPOUTEN &= ~(1UL << epNum);
		NRF_USBD->EVENTS_ENDEPOUT[epNum] = 0;
		NRF_USBD->EPDATASTATUS = (1UL << (16U + epNum));
		NRF_USBD->SIZE.EPOUT[epNum] = 0;
	}
	pXfer->Started = false;
	pXfer->DataReceived = false;
	pXfer->ActualLen = 0;
	pXfer->TotalLen = 0;
	pXfer->pBuffer = NULL;
	pXfer->Mps = 0;
	__ISB();
	__DSB();
}

static void nRFUsbRegEpCloseAll(void)
{
	nRFUsbdDmaWait();
	for (uint8_t epNum = 1; epNum < NRFX_USBD_EP_COUNT; epNum++)
	{
		nRFUsbRegEpClose(epNum);
		nRFUsbRegEpClose((uint8_t)(epNum | USB_ENDPADDR_DIR_IN));
	}
	NRF_USBD->EPOUTEN = 1UL;
	NRF_USBD->EPINEN = 1UL;
}

static bool nRFUsbRegEpXfer(uint8_t EpAddr, uint8_t *pBuffer, uint16_t TotalBytes)
{
	const uint8_t epNum = USB_ENDPADDR_NUM(EpAddr);
	if (epNum >= NRFX_USBD_EP_COUNT ||
		(TotalBytes > 0 && pBuffer == NULL) || atomic_load(&s_BusSuspended))
	{
		return false;
	}

	const uint32_t state = DisableInterrupt();
	nRFUsbdXfer_t *pXfer = nRFUsbdGetXfer(EpAddr);
	if (pXfer->Started || pXfer->Mps == 0)
	{
		EnableInterrupt(state);
		return false;
	}

	pXfer->pBuffer = pBuffer;
	pXfer->TotalLen = TotalBytes;
	pXfer->ActualLen = 0;
	pXfer->Started = true;

	const bool controlStatus =
		epNum == 0 && TotalBytes == 0 &&
		USB_ENDPADDR_IS_IN(EpAddr) != s_Ctrlr.SetupDirIn;
	if (controlStatus)
	{
		nRFUsbdQueueEp0Status();
	}
	else if (USB_ENDPADDR_IS_IN(EpAddr))
	{
		nRFUsbdQueueIn(epNum);
	}
	else if (epNum == 0)
	{
		nRFUsbdQueueEp0RcvOut();
	}
	else if (pXfer->DataReceived)
	{
		pXfer->DataReceived = false;
		nRFUsbdQueueOut(epNum);
	}

	EnableInterrupt(state);
	return true;
}

static inline __attribute__((always_inline))
bool nRFUsbRegDataEpXfer(uint8_t EpAddr, uint16_t Length)
{
	nRFUsbEpReg_t *pReg = nRFUsbGetEpReg(EpAddr);
	return nRFUsbRegEpXfer(EpAddr, pReg->pBuffer, Length);
}

static uint16_t nRFUsbRegEpMps(uint8_t EpAddr)
{
	const uint8_t epNum = USB_ENDPADDR_NUM(EpAddr);
	return epNum < NRFX_USBD_EP_COUNT ? nRFUsbdGetXfer(EpAddr)->Mps : 0U;
}

static void nRFUsbRegEpStall(uint8_t EpAddr)
{
	const uint8_t epNum = USB_ENDPADDR_NUM(EpAddr);
	if (epNum >= NRFX_USBD_EP_COUNT)
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

static void nRFUsbRegEpClearStall(uint8_t EpAddr)
{
	const uint8_t epNum = USB_ENDPADDR_NUM(EpAddr);
	if (epNum == 0 || epNum >= NRFX_USBD_EP_COUNT)
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

static uint32_t nRFUsbdCollectEvents(void)
{
	uint32_t enabled = NRF_USBD->INTEN & NRFUSBD_IRQ_MASK;
	uint32_t intStatus = 0;
	volatile uint32_t *pEvent = &NRF_USBD->EVENTS_USBRESET;
	while (enabled != 0U)
	{
		const uint32_t index = nRFUsbdLowestBit(enabled);
		enabled &= enabled - 1U;
		if (pEvent[index] == 0U)
		{
			continue;
		}
		intStatus |= (1UL << index);
		pEvent[index] = 0;
		__ISB();
		__DSB();
	}
	return intStatus;
}

static void nRFUsbdBusReset(void)
{
	if ((uint8_t)atomic_load(&s_DmaEpAddr) != NRFX_USBD_DMA_EP_NONE)
	{
		nRFUsbdDmaRelease();
	}
	NRF_USBD->EPOUTEN = 1UL;
	NRF_USBD->EPINEN = 1UL;
	for (uint8_t epNum = 0; epNum < NRFX_USBD_EP_COUNT; epNum++)
	{
		NRF_USBD->TASKS_STARTEPIN[epNum] = 0;
		NRF_USBD->TASKS_STARTEPOUT[epNum] = 0;
	}
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
		USBD_INTEN_ENDEPIN0_Msk |
		USBD_INTEN_ENDEPOUT0_Msk;
	nRFUsbdResetState();
}

static void nRFUsbdSetupEvent(void)
{
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
	s_Ctrlr.SetupDirIn =
		(evt.Setup.bmRequestType & USB_REQTYPE_MASK_DIR) != 0;
	const bool setAddress =
		(evt.Setup.bmRequestType &
		 (USB_REQTYPE_MASK_RECEIPT | USB_REQTYPE_MASK_TYPE)) == 0 &&
		evt.Setup.bRequest == USB_REQ_SET_ADDRESS;
	if (setAddress)
	{
		UsbCtrlrEvt_t addrEvt = {};
		addrEvt.Type = USB_CTRLR_EVT_ADDRESS;
		addrEvt.Address = (uint8_t)(evt.Setup.wValue & 0x7FU);
		nRFUsbdEmit(&addrEvt);
		return;
	}
	nRFUsbdEmit(&evt);
}

static void nRFUsbdHandleOutEnd(uint8_t EpNum)
{
	nRFUsbdXfer_t *pXfer = &s_Ctrlr.Xfer[EpNum][0];
	if (!pXfer->Started)
	{
		return;
	}
	const uint16_t transferLen = (uint16_t)NRF_USBD->EPOUT[EpNum].AMOUNT;
	if (EpNum == 0U && pXfer->pBuffer != NULL)
	{
		if (transferLen > 0U)
		{
			memcpy(pXfer->pBuffer, s_Ep0Bounce, transferLen);
		}
		pXfer->pBuffer += transferLen;
	}
	pXfer->ActualLen += transferLen;
	if (transferLen == pXfer->Mps && pXfer->ActualLen < pXfer->TotalLen)
	{
		if (EpNum == 0)
		{
			nRFUsbdQueueEp0RcvOut();
		}
	}
	else
	{
		pXfer->Started = false;
		nRFUsbdEmitXfer(EpNum, pXfer->ActualLen, USB_CTRLR_XFER_SUCCESS);
	}
}

static void nRFUsbdHandleOutData(uint8_t EpNum)
{
	if (EpNum != 0U)
	{
		nRFUsbEpReg_t *pReg = nRFUsbGetEpReg(EpNum);
		if (pReg->bBlocking)
		{
			nRFUsbEpRegisteredEvent(EpNum, USB_CTRLR_EVT_DRDY, 0U,
				USB_CTRLR_XFER_SUCCESS);
		}
		else
		{
			(void)nRFUsbRegDataEpXfer(EpNum, nRFUsbdGetXfer(EpNum)->Mps);
		}
		return;
	}

	nRFUsbdXfer_t *pXfer = &s_Ctrlr.Xfer[0][0];
	if (pXfer->Started &&
		(pXfer->ActualLen < pXfer->TotalLen || pXfer->TotalLen == 0))
	{
		pXfer->DataReceived = false;
		nRFUsbdQueueOut(0);
	}
	else
	{
		pXfer->DataReceived = true;
	}
}

static void nRFUsbdHandleInData(uint8_t EpNum)
{
	nRFUsbdXfer_t *pXfer = &s_Ctrlr.Xfer[EpNum][1];
	const uint8_t epAddr = (uint8_t)(EpNum | USB_ENDPADDR_DIR_IN);
	if (!pXfer->Started)
	{
		return;
	}
	const uint16_t transferLen = (uint16_t)NRF_USBD->EPIN[EpNum].AMOUNT;
	if (EpNum == 0U && pXfer->pBuffer != NULL)
	{
		pXfer->pBuffer += transferLen;
	}
	pXfer->ActualLen += transferLen;
	if (pXfer->ActualLen < pXfer->TotalLen)
	{
		nRFUsbdQueueIn(EpNum);
	}
	else
	{
		pXfer->Started = false;
		nRFUsbdEmitXfer(epAddr, pXfer->ActualLen, USB_CTRLR_XFER_SUCCESS);
	}
}

extern "C" void USBD_IRQHandler(void)
{
	const uint8_t activeDma = (uint8_t)atomic_load(&s_DmaEpAddr);
	if (activeDma != NRFX_USBD_DMA_EP_NONE)
	{
		const bool reset = NRF_USBD->EVENTS_USBRESET != 0U;
		volatile uint32_t *pEndEvent = nRFUsbdDmaEndEvent(activeDma);
		if (*pEndEvent == 0U && !reset)
		{
			return;
		}
		if (*pEndEvent != 0U && USB_ENDPADDR_IS_IN(activeDma) &&
			USB_ENDPADDR_NUM(activeDma) != 0U)
		{
			*pEndEvent = 0;
			__ISB();
			__DSB();
		}
		nRFUsbdDmaRelease();
	}

	const uint32_t intStatus = nRFUsbdCollectEvents();
	if (intStatus == 0)
	{
		nRFUsbdServicePending();
		return;
	}

	uint32_t eventCause = 0;
	if ((intStatus & USBD_INTEN_USBEVENT_Msk) != 0)
	{
		eventCause = NRF_USBD->EVENTCAUSE;
		NRF_USBD->EVENTCAUSE = eventCause;
		__ISB();
		__DSB();
	}

	if ((intStatus & USBD_INTEN_USBRESET_Msk) != 0)
	{
		nRFUsbdBusReset();
		nRFUsbdEmitSimple(USB_CTRLR_EVT_RESET);
		return;
	}

	if ((intStatus & USBD_INTEN_USBEVENT_Msk) != 0)
	{
		if ((eventCause & USBD_EVENTCAUSE_SUSPEND_Msk) != 0 &&
			!atomic_exchange(&s_BusSuspended, true))
		{
			atomic_store(&s_SuspendPending, s_UsbdCfg.bLowPowerSuspend);
			atomic_store(&s_RemoteWakePending, false);
			atomic_store(&s_HostResumePending, false);
			if ((NRF_USBD->INTEN & USBD_INTEN_SOF_Msk) == 0U)
			{
				NRF_USBD->EVENTS_SOF = 0;
			}
			NRF_USBD->INTENSET = USBD_INTENSET_SOF_Msk;
			nRFUsbdEmitSimple(USB_CTRLR_EVT_SUSPEND);
		}
		if ((eventCause & USBD_EVENTCAUSE_RESUME_Msk) != 0)
		{
			nRFUsbdHostResumeDetected();
		}
		if ((eventCause & USBD_EVENTCAUSE_USBWUALLOWED_Msk) != 0)
		{
			nRFUsbdWakeAllowed();
		}
	}

	uint32_t outEnd = (intStatus >> USBD_INTEN_ENDEPOUT0_Pos) &
		(uint32_t)(((1UL << NRFX_USBD_EP_COUNT) - 1UL) & ~1UL);
	while (outEnd != 0U)
	{
		const uint32_t epNum = nRFUsbdLowestBit(outEnd);
		outEnd &= outEnd - 1U;
		nRFUsbdHandleOutEnd((uint8_t)epNum);
	}

	if ((intStatus & (USBD_INTEN_EPDATA_Msk | USBD_INTEN_EP0DATADONE_Msk)) != 0)
	{
		const uint32_t dataStatus = NRF_USBD->EPDATASTATUS;
		NRF_USBD->EPDATASTATUS = dataStatus;
		__ISB();
		__DSB();
		const uint32_t epMask =
			(uint32_t)(((1UL << NRFX_USBD_EP_COUNT) - 1UL) & ~1UL);
		uint32_t outData = (dataStatus >> 16U) & epMask;
		uint32_t inData = dataStatus & epMask;
		while (outData != 0U)
		{
			const uint32_t epNum = nRFUsbdLowestBit(outData);
			outData &= outData - 1U;
			nRFUsbdHandleOutData((uint8_t)epNum);
		}
		while (inData != 0U)
		{
			const uint32_t epNum = nRFUsbdLowestBit(inData);
			inData &= inData - 1U;
			nRFUsbdHandleInData((uint8_t)epNum);
		}
	}

	const bool setupPending = (intStatus & USBD_INTEN_EP0SETUP_Msk) != 0;
	if (setupPending)
	{
		nRFUsbdHostResumeDetected();
		nRFUsbdAbortEp0();
		nRFUsbdSetupEvent();
	}
	else
	{
		if ((intStatus & USBD_INTEN_ENDEPOUT0_Msk) != 0)
		{
			nRFUsbdHandleOutEnd(0);
		}
		if ((intStatus & USBD_INTEN_EP0DATADONE_Msk) != 0)
		{
			if (s_Ctrlr.SetupDirIn)
			{
				nRFUsbdHandleInData(0);
			}
			else
			{
				nRFUsbdHandleOutData(0);
			}
		}
	}

	if ((intStatus & USBD_INTEN_SOF_Msk) != 0)
	{
		nRFUsbdHostResumeDetected();
		UsbCtrlrEvt_t evt = {};
		evt.Type = USB_CTRLR_EVT_SOF;
		evt.FrameNo = (uint16_t)NRF_USBD->FRAMECNTR;
		nRFUsbdEmit(&evt);
		if (!s_Ctrlr.SofEnabled && !atomic_load(&s_BusSuspended))
		{
			NRF_USBD->INTENCLR = USBD_INTENCLR_SOF_Msk;
		}
	}

	nRFUsbdTryRemoteWake();
	nRFUsbdTryEnterLowPower();
	nRFUsbdServicePending();
}

static bool nRFUsbRegStart(void) { return true; }
static bool nRFUsbRegHighSpeed(void) { return false; }

bool UsbCtrlrInit(int DevNo, const UsbCtrlrCfg_t *pCfg)
{
	if (!nRFUsbValidDevNo(DevNo) || pCfg == NULL)
	{
		return false;
	}
	memset(s_EpReg, 0, sizeof(s_EpReg));
	if (!nRFUsbPowerInit(pCfg))
	{
		return false;
	}
	return nRFUsbRegInit(pCfg->EvtHandler, pCfg->pContext);
}

bool UsbCtrlrStart(int DevNo)
{
	if (!nRFUsbValidDevNo(DevNo) || !nRFUsbPowerStart())
	{
		return false;
	}
	if (!nRFUsbRegStart())
	{
		nRFUsbPowerStop();
		return false;
	}
	return true;
}

void UsbCtrlrStop(int DevNo)
{
	if (!nRFUsbValidDevNo(DevNo))
	{
		return;
	}
	nRFUsbRegStop();
	nRFUsbPowerStop();
}

void UsbCtrlrProcess(int DevNo)
{
	if (nRFUsbValidDevNo(DevNo))
	{
		nRFUsbPowerProcess();
	}
}

bool UsbCtrlrVbusDetected(int DevNo)
{
	return nRFUsbValidDevNo(DevNo) && nRFUsbVbusDetected();
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
	if (nRFUsbValidDevNo(DevNo)) nRFUsbRegIntEnable();
}
void UsbCtrlrIntDisable(int DevNo)
{
	if (nRFUsbValidDevNo(DevNo)) nRFUsbRegIntDisable();
}
void UsbCtrlrConnect(int DevNo)
{
	if (nRFUsbValidDevNo(DevNo)) nRFUsbRegConnect();
}
void UsbCtrlrDisconnect(int DevNo)
{
	if (nRFUsbValidDevNo(DevNo)) nRFUsbRegDisconnect();
}
void UsbCtrlrRemoteWakeup(int DevNo)
{
	if (nRFUsbValidDevNo(DevNo)) nRFUsbRegRemoteWakeup();
}
void UsbCtrlrSofEnable(int DevNo, bool Enable)
{
	if (nRFUsbValidDevNo(DevNo)) nRFUsbRegSofEnable(Enable);
}
void UsbCtrlrSetAddress(int DevNo, uint8_t Address)
{
	if (nRFUsbValidDevNo(DevNo)) nRFUsbRegSetAddress(Address);
}

bool UsbCtrlrEpOpen(int DevNo, const UsbEndPointDesc_t *pDesc)
{
	return nRFUsbValidDevNo(DevNo) && nRFUsbRegEpOpen(pDesc);
}
void UsbCtrlrEpClose(int DevNo, uint8_t EpAddr)
{
	if (nRFUsbValidDevNo(DevNo)) nRFUsbRegEpClose(EpAddr);
}
void UsbCtrlrEpCloseAll(int DevNo)
{
	if (nRFUsbValidDevNo(DevNo)) nRFUsbRegEpCloseAll();
}

bool UsbCtrlrEpRegister(int DevNo, uint8_t EpAddr, uint8_t *pBuffer,
		bool bBlocking, UsbCtrlrEpHandler_t Handler, void *pContext)
{
	const uint8_t epNum = USB_ENDPADDR_NUM(EpAddr);
	if (!nRFUsbValidDevNo(DevNo) || epNum == 0U ||
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
	return nRFUsbRegDataEpXfer(EpAddr, Length);
}

bool UsbCtrlrEp0Xfer(int DevNo, uint8_t EpAddr, uint8_t *pBuffer,
		uint16_t Length)
{
	return nRFUsbValidDevNo(DevNo) && USB_ENDPADDR_NUM(EpAddr) == 0U &&
		nRFUsbRegEpXfer(EpAddr, pBuffer, Length);
}

void UsbCtrlrEpStall(int DevNo, uint8_t EpAddr)
{
	if (nRFUsbValidDevNo(DevNo)) nRFUsbRegEpStall(EpAddr);
}
void UsbCtrlrEpClearStall(int DevNo, uint8_t EpAddr)
{
	if (nRFUsbValidDevNo(DevNo)) nRFUsbRegEpClearStall(EpAddr);
}

#else
#error "usb_ctrlr_nrfx: this part has no USB controller"
#endif
