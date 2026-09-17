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
static bool s_UsbdXtalHeld = false;
static bool s_UsbdVbusLast = false;

static bool s_LowPowerExitPending = false;
static inline __attribute__((always_inline)) bool nRFUsbdDmaActive(void);


//
// nRF52 USBD constants and types.
//

// Errata 199's hardware-visible EasyDMA busy register is also the shared DMA
// ownership flag: 0x82 before STARTEP and zero after ENDEP.
#define NRFX_USBD_EASYDMA_BUSY_REG			(*((volatile uint32_t *)0x40027C1CUL))
#define NRFX_USBD_EASYDMA_BUSY_REG_BUSY		0x82UL
#define NRFX_USBD_EASYDMA_BUSY_REG_CLEAR	0UL

#define NRFUSBD_QUE_DEPTH			(NRFX_USBD_EP_COUNT * 2)
#define NRFUSBD_EP0_QUE_DEPTH		4U

enum
{
	NRFX_USBD_DATA_EP_COUNT = 8,
	NRFX_USBD_EP_COUNT = 9,
	NRFX_USBD_ISO_EP_NO = 8,
	NRFX_USBD_MAX_PACKET_SIZE = 64,
	NRFX_USBD_ISO_MAX_PACKET_SIZE = 512,
	NRFX_USBD_DMA_EP_NONE = 0xFFU,
	NRFX_USBD_XFER_EVT_OUT = 0x80U,
	NRFX_USBD_EP0_IDLE = 0U,
	NRFX_USBD_EP0_PENDING,
	NRFX_USBD_EP0_ACTIVE,
	NRFX_USBD_ISO_OUT_OPEN = 1U,
	NRFX_USBD_ISO_IN_OPEN = 2U,
};

#pragma pack(push, 4)

typedef struct __nRF_Usbd_Xfer
{
	uint8_t *pBuffer;
	uint16_t TotalLen;
	volatile uint16_t ActualLen;
	volatile bool DataReceived;
} nRFUsbdXfer_t;

typedef struct __nRF_Usbd_Ctrlr
{
	nRFUsbdXfer_t Xfer[NRFX_USBD_EP_COUNT][2];
	UsbCtrlrEvt_t SetupEvent;
	atomic_uint_fast8_t Ep0State;
	bool SofEnabled;
	bool SetupDirIn;
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

static atomic_bool s_PendingEp0Status;
static atomic_bool s_PendingEp0RcvOut;
static atomic_bool s_BusSuspended;
static atomic_bool s_SuspendPending;
static atomic_bool s_RemoteWakePending;
static atomic_bool s_HostResumePending;
static atomic_bool s_MacAwake;
static atomic_uint_fast8_t s_IsoOpen;
static atomic_bool s_IsoInReady;
static atomic_bool s_IsoOutReady;
static uint16_t s_IsoOutSize;

static void nRFUsbdHostResumeDetected(void);


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

/**
 * The part does not leave USB low power on its own. The native controller
 * writes LOWPOWER on suspend and signals resume, but no driver anywhere
 * writes ForceNormal except the remote wakeup path, which is the device
 * asking the host to wake, not the other way round. Without this the board is
 * gone after the host sleeps and needs a power cycle.
 */
/**
 * Leave USBD low power without waiting.
 *
 * Requesting the exit and observing READY are separate steps here. Spinning
 * for READY put a bound of NRFX_USBD_READY_WAIT_LOOPS iterations directly
 * under the application main loop, which is around a hundred milliseconds at
 * 64 MHz if the bit is slow to arrive. Nothing needs the exit to have finished
 * by the time this returns, so the request is raised and the next call
 * finishes it.
 */
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
	// Retire a request raised by an earlier call before looking at anything
	// else. Only this clears the errata register the request set.
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

static UsbSpeed_t nRFUsbMaxSpeed(void)
{
	return USB_SPEED_FULL;
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

	if (nRFUsbVbusDetected() == false)
	{
		// No cable. Not a failure: the poll in UsbdProcess reports the attach
		// and the caller comes back.
		return false;
	}

	if (UsbdXtalRequest() == false)
	{
		return false;
	}

	s_UsbdXtalHeld = true;

	NVIC_SetPriority(USBD_IRQn, s_UsbdIntPrio);

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

	// Stop the controller interrupt before powering down the wrapper.
	NVIC_DisableIRQ(USBD_IRQn);

	UsbdStopCtrlr();

	if (s_UsbdXtalHeld)
	{
		UsbdXtalRelease();
		s_UsbdXtalHeld = false;
	}

	s_UsbdStarted = false;
}

/**
 * Called from the application main loop through UsbCtrlrProcess. It must cost
 * nothing when there is nothing to do: a pending low power exit to retire, or
 * a VBUS edge. Neither waits.
 */
static void nRFUsbPowerProcess(void)
{
	if (s_UsbdInitialized == false)
	{
		return;
	}

	// POWER may be read while USBD EasyDMA is active, but most USBD registers
	// may not. Leave the low-power state untouched until the DMA END event.
	const bool dmaActive = nRFUsbdDmaActive();
	if (!dmaActive && s_LowPowerExitPending)
	{
		UsbdLowPowerExitFinish();
	}
	else if (!dmaActive && s_UsbdStarted &&
			 s_UsbdLowPowerSuspend == false &&
			 NRF_USBD->LOWPOWER != USBD_LOWPOWER_LOWPOWER_ForceNormal)
	{
		UsbdLowPowerExit();
	}

	const bool vbus = nRFUsbVbusDetected();

	if (vbus == s_UsbdVbusLast)
	{
		return;
	}

	// The port reports bus power state through UsbCtrlrVbusDetected(). Turning
	// an edge into USB_EVT_ATTACHED or USB_EVT_DETACHED for the application is
	// the generic layer's job, so there is no cable callback here. The edge is
	// still tracked because the low power path above needs it.
	s_UsbdVbusLast = vbus;
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
uint16_t nRFUsbdMps(uint8_t EpAddr)
{
	return USB_ENDPADDR_NUM(EpAddr) == 0U ?
		NRFX_USBD_MAX_PACKET_SIZE : nRFUsbGetEpReg(EpAddr)->Mps;
}

/**
 * Lowest set bit position. Walking a mask this way visits only the bits that
 * are set, in ascending order, instead of testing every position. CLZ is one
 * cycle on Cortex-M, and this is on the interrupt entry path.
 */
static inline __attribute__((always_inline))
uint32_t nRFUsbdLowestBit(uint32_t Mask)
{
	return 31U - (uint32_t)__CLZ(Mask & (uint32_t)(0U - Mask));
}


static inline __attribute__((always_inline))
volatile uint32_t *nRFUsbdDmaEndEvent(uint8_t EpNum, bool In)
{
	if (EpNum == NRFX_USBD_ISO_EP_NO)
	{
		return In ?
			&NRF_USBD->EVENTS_ENDISOIN : &NRF_USBD->EVENTS_ENDISOOUT;
	}

	return In ? &NRF_USBD->EVENTS_ENDEPIN[EpNum] :
		&NRF_USBD->EVENTS_ENDEPOUT[EpNum];
}

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
void nRFUsbdDmaStartLocked(volatile uint32_t *pTask, uint8_t EpNum, bool In)
{
	*nRFUsbdDmaEndEvent(EpNum, In) = 0;
	__DSB();

	NRFX_USBD_EASYDMA_BUSY_REG = NRFX_USBD_EASYDMA_BUSY_REG_BUSY;
	*pTask = 1;
	__DSB();
}


/**
 * Finish the one active EasyDMA transaction.
 *
 * EPSTATUS plus the matching END event identifies an ordinary completion.
 * Regular queue entries are consumed when DMA starts; EP0 retains its queue
 * head for its separate packet path. ISO has dedicated END events.
 */
static inline __attribute__((always_inline))
uint8_t nRFUsbdDmaFinishLocked(uint32_t DmaStatus,
	bool PreserveIsoEvent)
{
	uint8_t epAddr = NRFX_USBD_DMA_EP_NONE;
	volatile uint32_t *pEndEvent = NULL;
	uint32_t epStatusMask = DmaStatus & 0x00FF00FFUL;

	// EPSTATUS is a write-one-to-clear bitmap. EPIN[0..7] occupy bits 0..7
	// and EPOUT[0..7] occupy bits 16..23. The selected bit identifies both
	// endpoint and direction; its matching END event confirms completion.
	if (epStatusMask != 0U)
	{
		const uint8_t statusBit =
			(uint8_t)(31U - (uint32_t)__CLZ(epStatusMask));
		const bool out = statusBit >= 16U;
		const uint8_t epNum = out ?
			(uint8_t)(statusBit - 16U) : statusBit;
		epStatusMask = 1UL << statusBit;

		volatile uint32_t *pEvent = out ?
			&NRF_USBD->EVENTS_ENDEPOUT[epNum] :
			&NRF_USBD->EVENTS_ENDEPIN[epNum];
		if (*pEvent != 0U)
		{
			epAddr = out ? epNum :
				(uint8_t)(epNum | USB_ENDPADDR_DIR_IN);
			pEndEvent = pEvent;
		}
	}
	else if (NRF_USBD->EVENTS_ENDISOIN != 0U)
	{
		epAddr = USB_ENDPADDR_DIRIN(NRFX_USBD_ISO_EP_NO);
		pEndEvent = &NRF_USBD->EVENTS_ENDISOIN;
		epStatusMask = DmaStatus & (1UL << 8U);
	}
	else if (NRF_USBD->EVENTS_ENDISOOUT != 0U)
	{
		epAddr = NRFX_USBD_ISO_EP_NO;
		pEndEvent = &NRF_USBD->EVENTS_ENDISOOUT;
		epStatusMask = DmaStatus & (1UL << 24U);
	}

	if (epAddr == NRFX_USBD_DMA_EP_NONE)
	{
		return epAddr;
	}

	const uint8_t epNum = USB_ENDPADDR_NUM(epAddr);
	if (!PreserveIsoEvent || epNum != NRFX_USBD_ISO_EP_NO)
	{
		*pEndEvent = 0U;
	}

	// Retire the exact captured endpoint after its END event. This includes
	// ISO so EPSTATUS == 0 reliably means no EasyDMA capture remains.
	if (epStatusMask != 0U)
	{
		NRF_USBD->EPSTATUS = epStatusMask;
	}
	// Device-memory writes are ordered. One completion barrier retires both
	// W1C stores before the DMA ownership lock is released.
	__DSB();

	// EP0 keeps its entry until DMA completion because its separate packet
	// path may use queue-backed storage. Regular entries were consumed when
	// their DMA started, and ISO has no queue entry.
	if (epNum == 0U)
	{
		(void)CFifoGet(s_hEp0Que);
	}

	// EP0 retains the errata lock between its data packets and status stage.
	if (epNum != 0U ||
		atomic_load(&s_Ctrlr.Ep0State) != NRFX_USBD_EP0_ACTIVE)
	{
		nRFUsbdDmaUnlock();
	}

	return epAddr;
}

/** Foreground wrapper for forced-stop/wait paths. */
static uint8_t nRFUsbdDmaFinish(uint32_t DmaStatus,
	bool PreserveIsoEvent)
{
	const uint32_t primask = __get_PRIMASK();
	__disable_irq();
	const uint8_t epAddr =
		nRFUsbdDmaFinishLocked(DmaStatus, PreserveIsoEvent);
	__set_PRIMASK(primask);
	return epAddr;
}

static void nRFUsbdDmaWait(void)
{
	while (nRFUsbdDmaActive())
	{
		// EP0 keeps the errata register locked after ENDEP while it waits for
		// the host acknowledgement. A forced stop may release that ownership.
		if (atomic_load(&s_Ctrlr.Ep0State) ==
			NRFX_USBD_EP0_ACTIVE && CFifoUsed(s_hEp0Que) == 0)
		{
			nRFUsbdDmaUnlock();
			return;
		}

		if (NRF_USBD->EVENTS_USBRESET != 0U)
		{
			nRFUsbdDmaUnlock();
			return;
		}

		if (nRFUsbdDmaFinish(NRF_USBD->EPSTATUS, false) ==
			NRFX_USBD_DMA_EP_NONE)
		{
			continue;
		}
	}
}


static void nRFUsbdNoDmaTask(volatile uint32_t *pTask)
{
	*pTask = 1;
	__ISB();
	__DSB();
}

static uint8_t nRFUsbdEp0StatusNow(void)
{
	const uint8_t epAddr = s_Ctrlr.SetupDirIn ?
		USB_ENDPADDR_DIR_OUT : USB_ENDPADDR_DIR_IN;

	NRF_USBD->TASKS_EP0STATUS = 1;
	__ISB();
	__DSB();

	return epAddr;
}

static bool nRFUsbdStartIsoNow(void)
{
	const uint8_t inAddr = USB_ENDPADDR_DIRIN(NRFX_USBD_ISO_EP_NO);
	nRFUsbdXfer_t *pIn = nRFUsbdGetXfer(inAddr);
	const uint_fast8_t isoOpen = atomic_load(&s_IsoOpen);
	if ((isoOpen & NRFX_USBD_ISO_IN_OPEN) != 0U &&
		atomic_load(&s_IsoInReady))
	{
		atomic_store(&s_IsoInReady, false);
		NRF_USBD->ISOIN.PTR = (uint32_t)(uintptr_t)nRFUsbGetEpReg(inAddr)->pBuffer;
		NRF_USBD->ISOIN.MAXCNT = pIn->TotalLen;
		nRFUsbdDmaStartLocked(&NRF_USBD->TASKS_STARTISOIN,
			NRFX_USBD_ISO_EP_NO, true);
		return true;
	}

	nRFUsbdXfer_t *pOut = nRFUsbdGetXfer(NRFX_USBD_ISO_EP_NO);
	if ((isoOpen & NRFX_USBD_ISO_OUT_OPEN) != 0U &&
		atomic_load(&s_IsoOutReady))
	{
		atomic_store(&s_IsoOutReady, false);
		const uint16_t len = s_IsoOutSize < pOut->TotalLen ?
			s_IsoOutSize : pOut->TotalLen;
		NRF_USBD->ISOOUT.PTR = (uint32_t)(uintptr_t)
			nRFUsbGetEpReg(NRFX_USBD_ISO_EP_NO)->pBuffer;
		NRF_USBD->ISOOUT.MAXCNT = len;
		nRFUsbdDmaStartLocked(&NRF_USBD->TASKS_STARTISOOUT,
			NRFX_USBD_ISO_EP_NO, false);
		return true;
	}

	return false;
}

static inline __attribute__((always_inline))
bool nRFUsbdIsoPending(void)
{
	return atomic_load(&s_IsoInReady) || atomic_load(&s_IsoOutReady);
}

static void nRFUsbdServiceIso(void)
{
	if (!nRFUsbdIsoPending())
	{
		return;
	}

	const uint32_t state = DisableInterrupt();
	if (atomic_load(&s_Ctrlr.Ep0State) == NRFX_USBD_EP0_IDLE &&
		!atomic_load(&s_HostResumePending) &&
		(!atomic_load(&s_BusSuspended) || atomic_load(&s_SuspendPending)) &&
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
	uint8_t *pBuffer = epNum == 0U ? s_Ep0Bounce :
		s_EpReg[epNum][isIn ? 1 : 0].pBuffer;

	if (isIn)
	{
		NRF_USBD->EPIN[epNum].PTR = (uint32_t)(uintptr_t)pBuffer;
		NRF_USBD->EPIN[epNum].MAXCNT = pQue->Len;
		nRFUsbdDmaStartLocked(&NRF_USBD->TASKS_STARTEPIN[epNum],
			epNum, true);
	}
	else
	{
		const uint16_t received = (uint16_t)NRF_USBD->SIZE.EPOUT[epNum];
		const uint16_t len = received < pQue->Len ? received : pQue->Len;

		NRF_USBD->EPOUT[epNum].PTR = (uint32_t)(uintptr_t)pBuffer;
		NRF_USBD->EPOUT[epNum].MAXCNT = len;
		nRFUsbdDmaStartLocked(&NRF_USBD->TASKS_STARTEPOUT[epNum],
			epNum, false);
	}
}

static inline __attribute__((always_inline))
void nRFUsbdStartQueuedDma(void)
{
	nRFUsbdQue_t *pQue = (nRFUsbdQue_t *)CFifoGet(s_hQue);
	if (pQue != NULL)
		nRFUsbdStartDmaNow(pQue);
}


static void nRFUsbdServiceEp0(void)
{
	const uint32_t state = DisableInterrupt();
	if (atomic_load(&s_Ctrlr.Ep0State) != NRFX_USBD_EP0_ACTIVE)
	{
		EnableInterrupt(state);
		return;
	}

	if (atomic_exchange(&s_PendingEp0Status, false))
	{
		// The data stage is complete. Release the retained errata lock only
		// now, immediately before starting the control status stage.
		if (nRFUsbdDmaActive())
		{
			nRFUsbdDmaUnlock();
		}

		const uint8_t epAddr = nRFUsbdEp0StatusNow();
		EnableInterrupt(state);
		nRFUsbdEmitXfer(epAddr, 0U, USB_CTRLR_XFER_SUCCESS);

		uint_fast8_t expected = NRFX_USBD_EP0_ACTIVE;
		(void)atomic_compare_exchange_strong(&s_Ctrlr.Ep0State, &expected,
			NRFX_USBD_EP0_IDLE);
		return;
	}

	if (atomic_exchange(&s_PendingEp0RcvOut, false))
	{
		nRFUsbdNoDmaTask(&NRF_USBD->TASKS_EP0RCVOUT);
		EnableInterrupt(state);
		return;
	}

	// The EP0 FIFO is separate from ordinary endpoint work. The first packet
	// acquires EasyDMA; later packets restart EP0 while the errata lock remains
	// held, so another endpoint can never run between control packets.
	nRFUsbdQue_t *pHead = (nRFUsbdQue_t *)CFifoPeek(s_hEp0Que);
	if (pHead != NULL)
	{
		nRFUsbdStartDmaNow(pHead);
		EnableInterrupt(state);
		return;
	}

	// No EP0 operation remains. End EP0 ownership so ordinary endpoint work
	// may continue.
	if (nRFUsbdDmaActive())
	{
		nRFUsbdDmaUnlock();
	}
	uint_fast8_t expected = NRFX_USBD_EP0_ACTIVE;
	(void)atomic_compare_exchange_strong(&s_Ctrlr.Ep0State, &expected,
		NRFX_USBD_EP0_IDLE);

	EnableInterrupt(state);
}

static inline __attribute__((always_inline)) void nRFUsbdResumeQueuedDmaLocked(void)
{
	if (nRFUsbdDmaActive() ||
		atomic_load(&s_Ctrlr.Ep0State) != NRFX_USBD_EP0_IDLE ||
		atomic_load(&s_HostResumePending) ||
		nRFUsbdIsoPending() ||
		(atomic_load(&s_BusSuspended) &&
		 !atomic_load(&s_SuspendPending)))
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

	if (EpNum != 0U && !nRFUsbdDmaActive() &&
		atomic_load(&s_Ctrlr.Ep0State) == NRFX_USBD_EP0_IDLE &&
		!atomic_load(&s_HostResumePending) && !nRFUsbdIsoPending() &&
		(!atomic_load(&s_BusSuspended) || atomic_load(&s_SuspendPending)))
	{
		nRFUsbdStartQueuedDma();
	}

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

static void nRFUsbdQueueOut(uint8_t EpNum)
{
	nRFUsbdXfer_t *pXfer = &s_Ctrlr.Xfer[EpNum][0];

	nRFUsbdQueXferDir(EpNum, false,
				 (uint16_t)(pXfer->TotalLen - pXfer->ActualLen));
}

static void nRFUsbdQueueIn(uint8_t EpNum)
{
	nRFUsbdXfer_t *pXfer = &s_Ctrlr.Xfer[EpNum][1];
	const uint16_t remaining =
		(uint16_t)(pXfer->TotalLen - pXfer->ActualLen);
	const uint16_t mps =
		nRFUsbdMps((uint8_t)(EpNum | USB_ENDPADDR_DIR_IN));
	const uint16_t length = remaining < mps ? remaining : mps;

	if (EpNum == 0U && length > 0U)
	{
		memcpy(s_Ep0Bounce, pXfer->pBuffer, length);
	}

	nRFUsbdQueXferDir(EpNum, true, length);
}

static void nRFUsbdQueueEp0Status(void)
{
	atomic_store(&s_PendingEp0Status, true);
}

static void nRFUsbdQueueEp0RcvOut(void)
{
	atomic_store(&s_PendingEp0RcvOut, true);
}

static void nRFUsbdResetState(void)
{
	memset(s_Ctrlr.Xfer, 0, sizeof(s_Ctrlr.Xfer));
	memset(&s_Ctrlr.SetupEvent, 0, sizeof(s_Ctrlr.SetupEvent));
	atomic_store(&s_Ctrlr.Ep0State, NRFX_USBD_EP0_IDLE);
	s_Ctrlr.SofEnabled = false;
	s_Ctrlr.SetupDirIn = false;

	CFifoFlush(s_hQue);
	CFifoFlush(s_hEp0Que);
	atomic_store(&s_PendingEp0Status, false);
	atomic_store(&s_PendingEp0RcvOut, false);
	atomic_store(&s_BusSuspended, false);
	atomic_store(&s_SuspendPending, false);
	atomic_store(&s_RemoteWakePending, false);
	atomic_store(&s_HostResumePending, false);
	atomic_store(&s_MacAwake, true);
	atomic_store(&s_IsoOpen, 0U);
	atomic_store(&s_IsoInReady, false);
	atomic_store(&s_IsoOutReady, false);
	s_IsoOutSize = 0U;
	NRFX_USBD_EASYDMA_BUSY_REG = NRFX_USBD_EASYDMA_BUSY_REG_CLEAR;
}

static void nRFUsbdAbortEp0(void)
{
	// The SETUP processor enters only after the previous DMA is complete.
	// Ep0State then owns the scheduler through the entire control transaction.
	nRFUsbdQueRemoveEp(0U);
	atomic_store(&s_PendingEp0Status, false);
	atomic_store(&s_PendingEp0RcvOut, false);

	for (uint8_t dir = 0; dir < 2U; dir++)
	{
		nRFUsbdXfer_t *pXfer = &s_Ctrlr.Xfer[0][dir];
		pXfer->pBuffer = NULL;
		pXfer->TotalLen = 0;
		pXfer->ActualLen = 0;
		pXfer->DataReceived = false;
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
		!atomic_load(&s_BusSuspended) ||
		!atomic_load(&s_SuspendPending) ||
		atomic_load(&s_RemoteWakePending) ||
		atomic_load(&s_HostResumePending) ||
		atomic_load(&s_Ctrlr.Ep0State) != NRFX_USBD_EP0_IDLE ||
		nRFUsbdDmaActive() ||
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

	const uint32_t irqState = DisableInterrupt();
	if (!atomic_load(&s_BusSuspended) ||
		!atomic_load(&s_SuspendPending) ||
		atomic_load(&s_RemoteWakePending) ||
		atomic_load(&s_HostResumePending) ||
		atomic_load(&s_Ctrlr.Ep0State) != NRFX_USBD_EP0_IDLE ||
		nRFUsbdDmaActive() ||
		CFifoUsed(s_hQue) > 0 ||
		atomic_load(&s_PendingEp0Status) ||
		atomic_load(&s_PendingEp0RcvOut))
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

	atomic_store(&s_MacAwake, false);
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

	if (atomic_load(&s_RemoteWakePending) || !atomic_load(&s_SuspendPending))
	{
		atomic_store(&s_SuspendPending, false);
		NRF_USBD->LOWPOWER =
			USBD_LOWPOWER_LOWPOWER_ForceNormal << USBD_LOWPOWER_LOWPOWER_Pos;
		__ISB();
		__DSB();
		EnableInterrupt(irqState);
		return;
	}

	atomic_store(&s_SuspendPending, false);
	EnableInterrupt(irqState);
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

	const uint32_t irqState = DisableInterrupt();
	if (!atomic_load(&s_RemoteWakePending) ||
		!atomic_load(&s_BusSuspended) ||
		atomic_load(&s_HostResumePending) ||
		!atomic_load(&s_MacAwake) ||
		nRFUsbdDmaActive())
	{
		EnableInterrupt(irqState);
		return;
	}

	atomic_store(&s_RemoteWakePending, false);
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

	if (atomic_load(&s_HostResumePending))
	{
		atomic_store(&s_HostResumePending, false);
		nRFUsbdEmitSimple(USB_CTRLR_EVT_RESUME);
		return;
	}

	nRFUsbdTryRemoteWake();
}

static bool nRFUsbRegInit(void)
{
	s_hQue = CFifoInit(s_QueMem, sizeof(s_QueMem), sizeof(nRFUsbdQue_t),
					   false);
	s_hEp0Que = CFifoInit(s_Ep0QueMem, sizeof(s_Ep0QueMem),
						  sizeof(nRFUsbdQue_t), true);
	if (s_hQue == NULL || s_hEp0Que == NULL)
	{
		return false;
	}

	nRFUsbdResetState();
	return true;
}

static void nRFUsbRegStop(void)
{
	nRFUsbdDmaWait();
	nRFUsbdResetState();
}

static void nRFUsbRegIntEnable(void)
{
	NVIC_EnableIRQ(USBD_IRQn);
}

static void nRFUsbRegIntDisable(void)
{
	NVIC_DisableIRQ(USBD_IRQn);
}

static void nRFUsbRegConnect(void)
{
	NRF_USBD->USBPULLUP = 1;
}

static void nRFUsbRegDisconnect(void)
{
	NRF_USBD->USBPULLUP = 0;
}

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
	else
	{
		if (atomic_load(&s_IsoOpen) == 0U &&
			!atomic_load(&s_BusSuspended))
		{
			NRF_USBD->INTENCLR = USBD_INTENCLR_SOF_Msk;
		}
	}
}

static void nRFUsbRegSetAddress(uint8_t Address)
{
	(void)Address;
}

static bool nRFUsbRegEpOpen(const UsbEndPointDesc_t *pDesc)
{
	if (pDesc == NULL)
	{
		return false;
	}

	const uint8_t epAddr = pDesc->bEndpointAddress;
	const uint8_t epNum = USB_ENDPADDR_NUM(epAddr);
	const uint8_t type = pDesc->bmAttributes & 0x03U;

	const bool iso = epNum == NRFX_USBD_ISO_EP_NO;
	if (epNum == 0 || epNum >= NRFX_USBD_EP_COUNT ||
		(iso ? type != USB_ENDPATT_TRANS_ISO :
		 (type != USB_ENDPATT_TRANS_BULK && type != USB_ENDPATT_TRANS_INT)) ||
		pDesc->wMaxPacketSize == 0 ||
		pDesc->wMaxPacketSize > (iso ? NRFX_USBD_ISO_MAX_PACKET_SIZE :
			NRFX_USBD_MAX_PACKET_SIZE))
	{
		return false;
	}

	nRFUsbdXfer_t *pXfer = nRFUsbdGetXfer(epAddr);
	nRFUsbGetEpReg(epAddr)->Mps = pDesc->wMaxPacketSize;
	pXfer->DataReceived = false;

	if (iso)
	{
		// Both directions share the 1024-byte ISO buffer. HalfIN gives each
		// direction 512 bytes, which is more than the Bluetooth SCO maximum.
		NRF_USBD->ISOSPLIT =
			USBD_ISOSPLIT_SPLIT_HalfIN << USBD_ISOSPLIT_SPLIT_Pos;
		NRF_USBD->ISOINCONFIG =
			USBD_ISOINCONFIG_RESPONSE_ZeroData <<
			USBD_ISOINCONFIG_RESPONSE_Pos;
		if (USB_ENDPADDR_IS_IN(epAddr))
		{
			NRF_USBD->EVENTS_ENDISOIN = 0;
			NRF_USBD->INTENSET = USBD_INTEN_ENDISOIN_Msk;
			NRF_USBD->EPINEN |= (1UL << NRFX_USBD_ISO_EP_NO);
			atomic_fetch_or(&s_IsoOpen, NRFX_USBD_ISO_IN_OPEN);
			atomic_store(&s_IsoInReady, false);
		}
		else
		{
			NRF_USBD->EVENTS_ENDISOOUT = 0;
			NRF_USBD->INTENSET = USBD_INTEN_ENDISOOUT_Msk;
			NRF_USBD->EPOUTEN |= (1UL << NRFX_USBD_ISO_EP_NO);
			atomic_fetch_or(&s_IsoOpen, NRFX_USBD_ISO_OUT_OPEN);
			atomic_store(&s_IsoOutReady, false);
		}
		NRF_USBD->EVENTS_SOF = 0;
		NRF_USBD->INTENSET = USBD_INTEN_SOF_Msk;
		__ISB();
		__DSB();
		return true;
	}

	if (USB_ENDPADDR_IS_IN(epAddr))
	{
		NRF_USBD->EVENTS_ENDEPIN[epNum] = 0;
		// Data IN completes its short DMA copy in the controller interrupt.
		// EPDATA later reports when the host consumed the copied packet.
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

	if (nRFUsbdDmaActive())
	{
		nRFUsbdDmaWait();
	}

	nRFUsbdXfer_t *pXfer = nRFUsbdGetXfer(EpAddr);
	if (epNum == NRFX_USBD_ISO_EP_NO)
	{
		if (USB_ENDPADDR_IS_IN(EpAddr))
		{
			atomic_fetch_and(&s_IsoOpen,
				(uint_fast8_t)~NRFX_USBD_ISO_IN_OPEN);
			atomic_store(&s_IsoInReady, false);
			NRF_USBD->INTENCLR = USBD_INTEN_ENDISOIN_Msk;
			NRF_USBD->EPINEN &= ~(1UL << NRFX_USBD_ISO_EP_NO);
			NRF_USBD->EVENTS_ENDISOIN = 0;
		}
		else
		{
			atomic_fetch_and(&s_IsoOpen,
				(uint_fast8_t)~NRFX_USBD_ISO_OUT_OPEN);
			atomic_store(&s_IsoOutReady, false);
			NRF_USBD->INTENCLR = USBD_INTEN_ENDISOOUT_Msk;
			NRF_USBD->EPOUTEN &= ~(1UL << NRFX_USBD_ISO_EP_NO);
			NRF_USBD->EVENTS_ENDISOOUT = 0;
		}
		if (!s_Ctrlr.SofEnabled && atomic_load(&s_IsoOpen) == 0U &&
			!atomic_load(&s_BusSuspended))
		{
			NRF_USBD->INTENCLR = USBD_INTEN_SOF_Msk;
		}
	}
	else if (USB_ENDPADDR_IS_IN(EpAddr))
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

	pXfer->DataReceived = false;
	pXfer->ActualLen = 0;
	pXfer->TotalLen = 0;
	pXfer->pBuffer = NULL;
	nRFUsbGetEpReg(EpAddr)->Mps = 0U;
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
	uint8_t *pDmaBuffer = epNum == 0U ? pBuffer :
		nRFUsbGetEpReg(EpAddr)->pBuffer;

	const uint32_t state = DisableInterrupt();
	nRFUsbdXfer_t *pXfer = nRFUsbdGetXfer(EpAddr);

	pXfer->pBuffer = epNum == 0U ? pBuffer : NULL;
	pXfer->TotalLen = TotalBytes;
	pXfer->ActualLen = 0U;
	if (epNum == NRFX_USBD_ISO_EP_NO)
	{
		if (TotalBytes > nRFUsbdMps(EpAddr))
		{
			EnableInterrupt(state);
			return false;
		}
		EnableInterrupt(state);
		nRFUsbdServiceIso();
		return true;
	}

	const bool controlStatus =
		epNum == 0U && TotalBytes == 0U &&
		USB_ENDPADDR_IS_IN(EpAddr) != s_Ctrlr.SetupDirIn;

	if (controlStatus)
	{
		nRFUsbdQueueEp0Status();
	}
	else if (USB_ENDPADDR_IS_IN(EpAddr))
	{
		nRFUsbdQueueIn(epNum);
	}
	else if (epNum == 0U)
	{
		nRFUsbdQueueEp0RcvOut();
	}
	else
	{
		nRFUsbdQueueOut(epNum);
	}

	EnableInterrupt(state);
	return true;
}

static inline __attribute__((always_inline))
bool nRFUsbRegDataEpXferDir(uint8_t EpNum, bool In, uint16_t Length)
{
	if (EpNum == NRFX_USBD_ISO_EP_NO)
	{
		const uint8_t epAddr = In ? USB_ENDPADDR_DIRIN(EpNum) : EpNum;
		return nRFUsbRegEpXfer(epAddr, NULL, Length);
	}

	nRFUsbdQueXferDir(EpNum, In, Length);
	return true;
}

static inline __attribute__((always_inline))
bool nRFUsbRegDataEpXfer(uint8_t EpAddr, uint16_t Length)
{
	return nRFUsbRegDataEpXferDir(USB_ENDPADDR_NUM(EpAddr),
		USB_ENDPADDR_IS_IN(EpAddr), Length);
}

static uint16_t nRFUsbRegEpMps(uint8_t EpAddr)
{
	const uint8_t epNum = USB_ENDPADDR_NUM(EpAddr);
	return epNum < NRFX_USBD_EP_COUNT ? nRFUsbdMps(EpAddr) : 0U;
}

static void nRFUsbRegEpStall(uint8_t EpAddr)
{
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

static void nRFUsbRegEpClearStall(uint8_t EpAddr)
{
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

static void nRFUsbdProcessEp0SetupEx(uint32_t Evt, void *pContext)
{
	(void)Evt;
	(void)pContext;

	UsbCtrlrEvt_t evt;
	for (;;)
	{
		// Interrupts remain enabled while the active DMA finishes. Only its
		// completion interrupt releases the shared EasyDMA engine.
		while (nRFUsbdDmaActive())
		{
		}

		const uint32_t irqState = DisableInterrupt();
		if (nRFUsbdDmaActive())
		{
			EnableInterrupt(irqState);
			continue;
		}

		uint_fast8_t expected = NRFX_USBD_EP0_PENDING;
		if (!atomic_compare_exchange_strong(&s_Ctrlr.Ep0State, &expected,
			NRFX_USBD_EP0_ACTIVE))
		{
			EnableInterrupt(irqState);
			return;
		}

		// The ISR may capture a later SETUP while this one is processed. Work
		// from a private copy so the later AppEvt can abort this transaction.
		evt = s_Ctrlr.SetupEvent;
		s_Ctrlr.SetupDirIn =
			(evt.Setup.bmRequestType & USB_REQTYPE_MASK_DIR) != 0U;
		EnableInterrupt(irqState);
		break;
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

	// UsbDevProcessEvent queued the EP0 data or status stage while EP0 owned
	// the scheduler. Start only EP0 here and retain ownership until its final
	// packet completes.
	nRFUsbdServiceEp0();
	nRFUsbdServiceIso();
	nRFUsbdResumeQueuedDma();
}

static void nRFUsbdHandleEp0OutEnd(uint16_t TransferLen)
{
	nRFUsbdXfer_t *pXfer = &s_Ctrlr.Xfer[0][0];

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
		nRFUsbdQueueEp0RcvOut();
	}
	else
	{
		nRFUsbdEmitXfer(0U, pXfer->ActualLen, USB_CTRLR_XFER_SUCCESS);
	}
}

static void nRFUsbdHandleEp0OutData(void)
{
	nRFUsbdXfer_t *pXfer = &s_Ctrlr.Xfer[0][0];
	if (pXfer->ActualLen < pXfer->TotalLen || pXfer->TotalLen == 0U)
	{
		pXfer->DataReceived = false;
		nRFUsbdQueueOut(0U);
	}
	else
	{
		pXfer->DataReceived = true;
	}
}

static void nRFUsbdHandleEp0InData(uint16_t TransferLen)
{
	nRFUsbdXfer_t *pXfer = &s_Ctrlr.Xfer[0][1];

	if (pXfer->pBuffer != NULL)
	{
		pXfer->pBuffer += TransferLen;
	}
	pXfer->ActualLen += TransferLen;

	if (pXfer->ActualLen < pXfer->TotalLen)
	{
		nRFUsbdQueueIn(0U);
	}
	else
	{
		nRFUsbdEmitXfer(USB_ENDPADDR_DIR_IN, pXfer->ActualLen,
			USB_CTRLR_XFER_SUCCESS);
	}
}

static void nRFUsbdHandleIsoInEnd(uint16_t TransferLen)
{
	const uint8_t epAddr = USB_ENDPADDR_DIRIN(NRFX_USBD_ISO_EP_NO);
	nRFUsbdXfer_t *pXfer = nRFUsbdGetXfer(epAddr);

	pXfer->ActualLen = TransferLen;
	nRFUsbdEmitXfer(epAddr, pXfer->ActualLen, USB_CTRLR_XFER_SUCCESS);
}

static void nRFUsbdHandleIsoOutEnd(uint16_t TransferLen)
{
	nRFUsbdXfer_t *pXfer = nRFUsbdGetXfer(NRFX_USBD_ISO_EP_NO);

	pXfer->ActualLen = TransferLen;
	nRFUsbdEmitXfer(NRFX_USBD_ISO_EP_NO, pXfer->ActualLen,
		USB_CTRLR_XFER_SUCCESS);
}

static void nRFUsbdProcessIsoComplete(uint32_t Evt, void *pContext)
{
	const uint16_t amount = (uint16_t)(Evt >> 8U);

	(void)pContext;

	if ((Evt & NRFX_USBD_XFER_EVT_OUT) != 0U)
	{
		// OUT data remains in the endpoint DMA buffer until its
		// completion callback consumes it.
		nRFUsbdHandleIsoOutEnd(amount);
	}
	else
	{
		// IN data has already been consumed by the controller.
		nRFUsbdHandleIsoInEnd(amount);
	}

	nRFUsbdServiceIso();
	nRFUsbdResumeQueuedDma();
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
		nRFUsbdHandleEp0InData(amount);
	}

	nRFUsbdServiceEp0();
	nRFUsbdServiceIso();
	nRFUsbdResumeQueuedDma();
}

static void nRFUsbdProcessOutComplete(uint32_t Evt, void *pContext)
{
	const uint8_t epNum = (uint8_t)Evt;
	const uint16_t amount = (uint16_t)(Evt >> 8U);

	(void)pContext;

	// UsbIntrf packetizes regular endpoints. One queued request is one DMA
	// transaction, so only EP0 needs multi-packet completion processing.
	nRFUsbEpReg_t *pReg = &s_EpReg[epNum][0];
	pReg->Handler(epNum, USB_CTRLR_EVT_XFER_CMPL, amount,
		USB_CTRLR_XFER_SUCCESS, pReg->pContext);
}

static void nRFUsbdProcessInComplete(uint32_t Evt, void *pContext)
{
	const uint8_t epNum = (uint8_t)Evt;
	const uint16_t amount = (uint16_t)(Evt >> 8U);

	(void)pContext;

	nRFUsbEpReg_t *pReg = &s_EpReg[epNum][1];
	pReg->Handler((uint8_t)(epNum | USB_ENDPADDR_DIR_IN),
		USB_CTRLR_EVT_XFER_CMPL, amount, USB_CTRLR_XFER_SUCCESS,
		pReg->pContext);
}

static void nRFUsbdProcessEp0OutData(uint32_t Evt, void *pContext)
{
	(void)Evt;
	(void)pContext;

	nRFUsbdHandleEp0OutData();
	nRFUsbdServiceEp0();
	nRFUsbdServiceIso();
	nRFUsbdResumeQueuedDma();
}

static void nRFUsbdProcessOutData(uint32_t Evt, void *pContext)
{
	const uint8_t epNum = (uint8_t)Evt;

	(void)pContext;

	// Only blocking endpoints reach AppEvt. Nonblocking OUT is queued for
	// EasyDMA directly from EPDATASTATUS in the ISR.
	nRFUsbEpReg_t *pReg = &s_EpReg[epNum][0];
	pReg->Handler(epNum, USB_CTRLR_EVT_DRDY, 0U,
		USB_CTRLR_XFER_SUCCESS, pReg->pContext);
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

static void nRFUsbdQueueIsoComplete(bool Out, uint16_t Amount)
{
	const uint32_t evt = ((uint32_t)Amount << 8U) | (Out ? NRFX_USBD_XFER_EVT_OUT : 0U);
	(void)AppEvtHandlerQue(evt, NULL, nRFUsbdProcessIsoComplete);
}

static void nRFUsbdHandleBusEvent(uint32_t EventCause)
{
	if ((EventCause & USBD_EVENTCAUSE_SUSPEND_Msk) != 0U &&
		!atomic_load(&s_BusSuspended))
	{
		atomic_store(&s_BusSuspended, true);
		// A bus suspend and a peripheral low-power transition are separate.
		// When low-power suspend is disabled, retain all endpoint state and
		// wait for RESUME or SOF without touching USBD LOWPOWER.
		atomic_store(&s_SuspendPending, s_UsbdLowPowerSuspend);
		atomic_store(&s_RemoteWakePending, false);
		atomic_store(&s_HostResumePending, false);
		atomic_store(&s_IsoInReady, false);
		atomic_store(&s_IsoOutReady, false);
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

	const uint_fast8_t isoOpen = atomic_load(&s_IsoOpen);
	if ((isoOpen & NRFX_USBD_ISO_IN_OPEN) != 0U)
	{
		atomic_store(&s_IsoInReady, true);
	}

	if ((isoOpen & NRFX_USBD_ISO_OUT_OPEN) != 0U)
	{
		const uint32_t size = NRF_USBD->SIZE.ISOOUT;
		if (size != 0U)
		{
			s_IsoOutSize = (size & USBD_SIZE_ISOOUT_ZERO_Msk) != 0U ?
				0U : (uint16_t)size;
			atomic_store(&s_IsoOutReady, true);

			nRFUsbEpReg_t *pReg = nRFUsbGetEpReg(NRFX_USBD_ISO_EP_NO);
			if (pReg->bBlocking)
			{
				nRFUsbEpRegisteredEvent(NRFX_USBD_ISO_EP_NO,
					USB_CTRLR_EVT_DRDY, 0U, USB_CTRLR_XFER_SUCCESS);
			}
			else
			{
				(void)nRFUsbRegDataEpXfer(NRFX_USBD_ISO_EP_NO,
					nRFUsbdMps(NRFX_USBD_ISO_EP_NO));
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
		!atomic_load(&s_BusSuspended))
	{
		NRF_USBD->INTENCLR = USBD_INTENCLR_SOF_Msk;
	}

	// ISO is bound to this service interval. Start it here when EasyDMA is
	// free; otherwise the current DMA completion retries it before bulk work.
	nRFUsbdServiceIso();
}

static void nRFUsbdProcessEP0Setup(uint32_t Evt, void *pContext)
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

	// A new SETUP terminates the previous control transaction. If EP0 was
	// retaining the errata lock between packets, release that old ownership
	// before the new setup processor waits for EasyDMA.
	if (atomic_load(&s_Ctrlr.Ep0State) ==
		NRFX_USBD_EP0_ACTIVE && nRFUsbdDmaActive() &&
		CFifoUsed(s_hEp0Que) == 0)
	{
		nRFUsbdDmaUnlock();
	}

	s_Ctrlr.SetupEvent = evt;
	atomic_store(&s_Ctrlr.Ep0State, NRFX_USBD_EP0_PENDING);

	nRFUsbdProcessEp0SetupEx(Evt, pContext);
}


extern "C" void USBD_IRQHandler(void)
{
	uint32_t endep0in = NRF_USBD->EVENTS_ENDEPIN[0];
	uint32_t dmastatus = NRF_USBD->EPSTATUS;
	uint32_t outData = 0U;
	uint8_t completedDma = NRFX_USBD_DMA_EP_NONE;
	uint16_t completedOutAmount = 0U;
	const uint_fast8_t isoOpen = atomic_load(&s_IsoOpen);

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

	// A captured endpoint is an active EasyDMA transaction. Its matching
	// ENDEP confirms completion; otherwise leave all events latched and wait.
	// EPSTATUS == 0 means there is no captured DMA, even if the errata busy
	// register is intentionally retaining EP0 ownership.
	if (dmastatus != 0U)
	{
		completedDma = nRFUsbdDmaFinishLocked(dmastatus, true);
		if (completedDma == NRFX_USBD_DMA_EP_NONE)
		{
			return;
		}

		const uint8_t completedEp = USB_ENDPADDR_NUM(completedDma);
		const bool regularComplete = completedEp > 0U &&
			completedEp < NRFX_USBD_ISO_EP_NO;
		if (regularComplete && !USB_ENDPADDR_IS_IN(completedDma))
		{
			// Snapshot before another OUT DMA can update this register.
			completedOutAmount =
				(uint16_t)NRF_USBD->EPOUT[completedEp].AMOUNT;
		}

		// A regular completion proves EP0 was idle and the bus was running
		// when this DMA started. With no newly-latched control or bus event,
		// start the next queued regular DMA now. Completion notification and
		// the remaining ISR work proceed while that DMA is active.
		if (regularComplete && isoOpen == 0U &&
			NRF_USBD->EVENTS_EP0SETUP == 0U &&
			NRF_USBD->EVENTS_USBEVENT == 0U)
		{
			nRFUsbdStartQueuedDma();
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

	// Regular OUT endpoint addresses are 1..7. Endpoint zero is handled
	// further down with the control-transfer sequence.
	if (completedDma > 0U &&
		completedDma < NRFX_USBD_ISO_EP_NO)
	{
		nRFUsbdQueueOutComplete(completedDma, completedOutAmount);
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

	if (NRF_USBD->EVENTS_EP0DATADONE != 0U)
	{

		NRF_USBD->EVENTS_EP0DATADONE = 0U;
		NRF_USBD->EVENTS_ENDEPIN[0] = 0;
		__ISB();
		__DSB();

		if (endep0in)
		//if (s_Ctrlr.SetupDirIn)
		{
			//nRFUsbdQueueXferComplete(0U,
			//	(uint16_t)NRF_USBD->EPIN[0].AMOUNT);
			nRFUsbdQueueEp0Complete(false,
				(uint16_t)NRF_USBD->EPIN[0].AMOUNT);
		}
		else
		{
//			nRFUsbdQueueOutData(0U);
			(void)AppEvtHandlerQue(0U, NULL, nRFUsbdProcessEp0OutData);
		}
	}
	else if (NRF_USBD->EVENTS_EPDATA != 0U ||
		(NRF_USBD->EPDATASTATUS & 0x00FE00FEUL) != 0U)
	{
		// Clear the event first so a new endpoint event remains observable.
		// Service at most one endpoint per direction in this interrupt. Any
		// remaining status bits are retained and serviced by a pending IRQ.
		NRF_USBD->EVENTS_EPDATA = 0U;
		const uint32_t dataStatus = NRF_USBD->EPDATASTATUS;
		uint32_t servicedStatus = dataStatus & 0x00010001UL;

		outData = (dataStatus >> 16U) & 0xFEU;
		if (outData != 0U)
		{
			const uint32_t epNum = 31U - (uint32_t)__CLZ(outData);
			servicedStatus |= 1UL << (epNum + 16U);

			nRFUsbEpReg_t *pReg = nRFUsbGetEpReg((uint8_t)epNum);
			if (pReg->bBlocking)
			{
				(void)AppEvtHandlerQue(epNum, NULL,
					nRFUsbdProcessOutData);
			}
			else
			{
				// EPDATASTATUS already identifies the ready OUT endpoint.
				// Publish its DMA request here instead of waiting for AppEvt;
				// the ISR tail starts it after all USBD status is consumed.
				nRFUsbdQue_t *pQue =
					(nRFUsbdQue_t *)CFifoPut(s_hQue);
				pQue->EpNum = (uint8_t)epNum;
				pQue->Dir = 0U;
				pQue->Len = pReg->Mps;
			}
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
			NVIC_SetPendingIRQ(USBD_IRQn);
		}
	}

	if (isoOpen != 0U)
	{
		if (NRF_USBD->EVENTS_ENDISOIN != 0U)
		{
			NRF_USBD->EVENTS_ENDISOIN = 0U;
			nRFUsbdQueueIsoComplete(false,
				(uint16_t)NRF_USBD->ISOIN.AMOUNT);
		}

		if (NRF_USBD->EVENTS_ENDISOOUT != 0U)
		{
			NRF_USBD->EVENTS_ENDISOOUT = 0U;
			nRFUsbdQueueIsoComplete(true,
				(uint16_t)NRF_USBD->ISOOUT.AMOUNT);
		}
	}

	if (completedDma == 0U)
	{
		nRFUsbdQueueEp0Complete(true,
			(uint16_t)NRF_USBD->EPOUT[0].AMOUNT);
	}

	if (NRF_USBD->EVENTS_SOF != 0U)
	{
		NRF_USBD->EVENTS_SOF = 0U;
		__ISB();
		__DSB();

		nRFUsbdHandleSof();
	}

	if (atomic_load(&s_RemoteWakePending))
	{
		nRFUsbdTryRemoteWake();
	}

	// ENDEP released the shared EasyDMA channel above. ISO has priority when
	// it is open; ordinary CDC traffic avoids the ISO service path entirely.
	if (completedDma != NRFX_USBD_DMA_EP_NONE || outData != 0U)
	{
		if (isoOpen != 0U)
		{
			nRFUsbdServiceIso();
		}
		nRFUsbdResumeQueuedDmaLocked();
	}

	if (s_UsbdLowPowerSuspend && atomic_load(&s_BusSuspended))
	{
		nRFUsbdTryEnterLowPower();
	}

}

/**
 * USBD has no second register stage. The power stage leaves the peripheral
 * enabled and endpoint zero is prepared when the bus reset arrives, so there
 * is nothing to do between the two. It was a weak default in usbd_ctrlr.cpp
 * before, which hid the fact that this backend never implemented it.
 */
static bool nRFUsbRegStart(void)
{
	return true;
}

/** USBD is full speed only, USB_HIGHSPEED_CAPABLE(0) is 0 for these parts. */
static bool nRFUsbRegHighSpeed(void)
{
	return false;
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

	memset(s_EpReg, 0, sizeof(s_EpReg));

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

	// One call where usbd.h and usbd_ctrlr.h used to need two. Power, clock
	// and PHY come up, then endpoint zero is prepared.
	if (!nRFUsbPowerStart())
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
		AppEvtHandlerExec();
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
	if (nRFUsbValidDevNo(DevNo))
	{
		nRFUsbRegIntEnable();
	}
}

void UsbCtrlrIntDisable(int DevNo)
{
	if (nRFUsbValidDevNo(DevNo))
	{
		nRFUsbRegIntDisable();
	}
}

void UsbCtrlrConnect(int DevNo)
{
	if (nRFUsbValidDevNo(DevNo))
	{
		nRFUsbRegConnect();
	}
}

void UsbCtrlrDisconnect(int DevNo)
{
	if (nRFUsbValidDevNo(DevNo))
	{
		nRFUsbRegDisconnect();
	}
}

void UsbCtrlrRemoteWakeup(int DevNo)
{
	if (nRFUsbValidDevNo(DevNo))
	{
		nRFUsbRegRemoteWakeup();
	}
}

void UsbCtrlrSofEnable(int DevNo, bool Enable)
{
	if (nRFUsbValidDevNo(DevNo))
	{
		nRFUsbRegSofEnable(Enable);
	}
}

void UsbCtrlrSetAddress(int DevNo, uint8_t Address)
{
	if (nRFUsbValidDevNo(DevNo))
	{
		nRFUsbRegSetAddress(Address);
	}
}

bool UsbCtrlrEpOpen(int DevNo, const UsbEndPointDesc_t *pDesc)
{
	return nRFUsbValidDevNo(DevNo) && nRFUsbRegEpOpen(pDesc);
}

void UsbCtrlrEpClose(int DevNo, uint8_t EpAddr)
{
	if (nRFUsbValidDevNo(DevNo))
	{
		nRFUsbRegEpClose(EpAddr);
	}
}

void UsbCtrlrEpCloseAll(int DevNo)
{
	if (nRFUsbValidDevNo(DevNo))
	{
		nRFUsbRegEpCloseAll();
	}
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

bool UsbCtrlrEpOutXfer(int DevNo, uint8_t EpNum, uint16_t Length)
{
	(void)DevNo;
	return nRFUsbRegDataEpXferDir(EpNum, false, Length);
}

bool UsbCtrlrEpInXfer(int DevNo, uint8_t EpNum, uint16_t Length)
{
	(void)DevNo;
	return nRFUsbRegDataEpXferDir(EpNum, true, Length);
}


bool UsbCtrlrEp0Xfer(int DevNo, uint8_t EpAddr, uint8_t *pBuffer,
						 uint16_t Length)
{
	return nRFUsbValidDevNo(DevNo) && USB_ENDPADDR_NUM(EpAddr) == 0U &&
		nRFUsbRegEpXfer(EpAddr, pBuffer, Length);
}

int UsbCtrlrEp0Send(int DevNo, uint8_t *pBuffer, int Length)
{
	int cnt = 0;

	do
	{
		int l = min(Length, NRFX_USBD_MAX_PACKET_SIZE);

		nRFEPPkt_t *p = (nRFEPPkt_t*)CFifoPut(s_hEp0Que);

		if (p == nullptr)
		{
			break;
		}

		if (l > 0 && pBuffer != nullptr)
		{
			memcpy(p->Payload, pBuffer, l);
		}
		p->Hdr.EpNum = 0U;
		p->Hdr.Dir = 1U;
		p->Hdr.Len = l;
		pBuffer += l;
		cnt +=l;
		Length -= l;
	} while  (Length > 0);

	if (NRFX_USBD_EASYDMA_BUSY_REG == NRFX_USBD_EASYDMA_BUSY_REG_CLEAR)
	{
		NRFX_USBD_EASYDMA_BUSY_REG = NRFX_USBD_EASYDMA_BUSY_REG_BUSY;
		nRFEPPkt_t *p = (nRFEPPkt_t*)CFifoPeek(s_hEp0Que);

		if (p)
		{
			s_Ctrlr.SetupDirIn = true;

			NRF_USBD->EVENTS_EP0DATADONE = 0U;
			NRF_USBD->EVENTS_ENDEPIN[0] = 0;

			NRF_USBD->EPIN[0].MAXCNT = p->Hdr.Len;
			NRF_USBD->EPIN[0].PTR = (uint32_t)p;

			if (p->Hdr.Len < NRFX_USBD_MAX_PACKET_SIZE)
			{
				NRF_USBD->SHORTS = USBD_SHORTS_EP0DATADONE_EP0STATUS_Msk;
			}
			else
			{
				NRF_USBD->SHORTS = 0;
			}
			NRF_USBD->TASKS_STARTEPIN[0] = 1U;
		}
	}

	return cnt;
}

void UsbCtrlrEpStall(int DevNo, uint8_t EpAddr)
{
	if (nRFUsbValidDevNo(DevNo))
	{
		nRFUsbRegEpStall(EpAddr);
	}
}

void UsbCtrlrEpClearStall(int DevNo, uint8_t EpAddr)
{
	if (nRFUsbValidDevNo(DevNo))
	{
		nRFUsbRegEpClearStall(EpAddr);
	}
}
