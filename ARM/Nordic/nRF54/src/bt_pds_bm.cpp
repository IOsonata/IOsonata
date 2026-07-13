/**-------------------------------------------------------------------------
@file	bt_pds_bm.cpp

@brief	BtPds NVM implementation for the nRF54L bm S145 path.

		RRAM is memory mapped for reads. Writes are submitted through
		sd_flash_write because the SoftDevice arbitrates NVM access against radio
		activity. The implementation presents a synchronous interface to BtPds.

		A write can be requested while the BLE observer chain is running. In that
		case the normal SoC poller cannot run until the caller returns, so this
		module drains pending SoC events itself. Every drained event is forwarded
		to the complete registered SoC observer list in the same order as
		nrf_sdh_soc.c; no unrelated event is consumed privately.

@author	Hoang Nguyen Hoan
@date	Jul 2026

@license MIT, (c) 2016-2026 I-SYST inc.
----------------------------------------------------------------------------*/
#include <string.h>
#include <errno.h>
#include <stdint.h>

#include "nrf_soc.h"
#include "nrf_error.h"
#include "nrf_sdh_soc.h"
#include "nrf.h"

#include "bluetooth/bt_pds.h"
#include "coredev/system_core_clock.h"

#ifndef BT_PDS_BM_REGION_ADDR
#define BT_PDS_BM_REGION_ADDR		0x00158800UL
#endif
#ifndef BT_PDS_BM_REGION_SIZE
#define BT_PDS_BM_REGION_SIZE		0x00001000UL	// 4 KB
#endif
#ifndef BT_PDS_BM_SECTOR_SIZE
#define BT_PDS_BM_SECTOR_SIZE		0x00000400UL	// 1 KB
#endif

#define BT_PDS_BM_WRITE_GRAN		4U

#ifndef BT_PDS_BM_BUSY_TIMEOUT_MS
#define BT_PDS_BM_BUSY_TIMEOUT_MS	250U
#endif
#ifndef BT_PDS_BM_WRITE_TIMEOUT_MS
#define BT_PDS_BM_WRITE_TIMEOUT_MS	2000U
#endif
#ifndef BT_PDS_BM_FALLBACK_POLLS
#define BT_PDS_BM_FALLBACK_POLLS	2000000UL
#endif

typedef struct __Bt_Pds_Bm_Deadline {
	bool		UseCycles;
	uint32_t	Start;
	uint32_t	Limit;
	uint32_t	Polls;
} BtPdsBmDeadline_t;

static volatile bool s_FlashOpDone;
static volatile bool s_FlashOpOk;
static bool s_FlashOpActive;
static int8_t s_CycleCounterAvailable = -1;

static void BtPdsBmSocEvtHandler(uint32_t SysEvt, void *pCtx)
{
	(void)pCtx;

	switch (SysEvt)
	{
		case NRF_EVT_FLASH_OPERATION_SUCCESS:
			s_FlashOpOk = true;
			s_FlashOpDone = true;
			break;

		case NRF_EVT_FLASH_OPERATION_ERROR:
			s_FlashOpOk = false;
			s_FlashOpDone = true;
			break;

		default:
			break;
	}
}

NRF_SDH_SOC_OBSERVER(s_BtPdsBmSocObs, BtPdsBmSocEvtHandler, NULL, HIGH);

static bool CycleCounterEnable(void)
{
	if (s_CycleCounterAvailable >= 0)
	{
		return s_CycleCounterAvailable != 0;
	}

	CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
	DWT->CYCCNT = 0U;
	DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
	__NOP();
	__NOP();

	s_CycleCounterAvailable = DWT->CYCCNT != 0U ? 1 : 0;
	return s_CycleCounterAvailable != 0;
}

static BtPdsBmDeadline_t DeadlineStart(uint32_t TimeoutMs)
{
	BtPdsBmDeadline_t deadline = {};
	uint32_t freq = SystemCoreClockGet();

	if (freq != 0U && CycleCounterEnable())
	{
		uint64_t cycles = ((uint64_t)freq * TimeoutMs) / 1000U;
		if (cycles == 0U)
		{
			cycles = 1U;
		}
		if (cycles > 0x7FFFFFFFUL)
		{
			cycles = 0x7FFFFFFFUL;
		}

		deadline.UseCycles = true;
		deadline.Start = DWT->CYCCNT;
		deadline.Limit = (uint32_t)cycles;
	}
	else
	{
		deadline.Polls = BT_PDS_BM_FALLBACK_POLLS;
	}

	return deadline;
}

static bool DeadlineExpired(BtPdsBmDeadline_t *pDeadline)
{
	if (pDeadline->UseCycles)
	{
		return (uint32_t)(DWT->CYCCNT - pDeadline->Start) >=
			pDeadline->Limit;
	}

	if (pDeadline->Polls == 0U)
	{
		return true;
	}
	pDeadline->Polls--;
	return false;
}

// Drain the SoftDevice SoC queue and forward every event exactly as the normal
// sdk-nrf-bm soc_evt_poll does. This function is used only while a synchronous
// write is blocking the outer event dispatcher.
static int DispatchSocEvents(void)
{
	uint32_t evtId;
	uint32_t status;

	while ((status = sd_evt_get(&evtId)) == NRF_SUCCESS)
	{
		TYPE_SECTION_FOREACH(
			struct nrf_sdh_soc_evt_observer,
			nrf_sdh_soc_evt_observers,
			observer)
		{
			observer->handler(evtId, observer->context);
		}
	}

	return status == NRF_ERROR_NOT_FOUND ? 0 : -EIO;
}

static int FlashWriteWords(uint32_t *pDst, const uint32_t *pSrc,
						   uint32_t WordCount)
{
	if (WordCount == 0U)
	{
		return 0;
	}
	if (s_FlashOpActive)
	{
		return -EBUSY;
	}

	s_FlashOpActive = true;
	int result = 0;
	uint32_t status;
	BtPdsBmDeadline_t busyDeadline = DeadlineStart(BT_PDS_BM_BUSY_TIMEOUT_MS);

	while (true)
	{
		s_FlashOpDone = false;
		s_FlashOpOk = false;
		__DMB();

		status = sd_flash_write(pDst, pSrc, WordCount);
		if (status == NRF_ERROR_BUSY)
		{
			if (DispatchSocEvents() != 0)
			{
				result = -EIO;
				goto exit;
			}
			if (DeadlineExpired(&busyDeadline))
			{
				result = -ETIMEDOUT;
				goto exit;
			}
			__NOP();
			continue;
		}
		if (status != NRF_SUCCESS)
		{
			result = -EIO;
			goto exit;
		}
		break;
	}

	{
		BtPdsBmDeadline_t writeDeadline =
			DeadlineStart(BT_PDS_BM_WRITE_TIMEOUT_MS);

		while (!s_FlashOpDone)
		{
			if (DispatchSocEvents() != 0)
			{
				result = -EIO;
				goto exit;
			}
			if (DeadlineExpired(&writeDeadline))
			{
				result = -ETIMEDOUT;
				goto exit;
			}
			__NOP();
		}
	}

	__DMB();
	if (!s_FlashOpOk)
	{
		result = -EIO;
	}

exit:
	s_FlashOpActive = false;
	return result;
}

static bool RangeValid(uint32_t Off, uint32_t Len)
{
	return Off <= BT_PDS_BM_REGION_SIZE &&
		Len <= BT_PDS_BM_REGION_SIZE - Off;
}

static int BtPdsBmRead(uint32_t Off, void *pBuf, uint32_t Len)
{
	if (!RangeValid(Off, Len) || (Len > 0U && pBuf == nullptr))
	{
		return -EINVAL;
	}

	memcpy(pBuf, (const void *)(BT_PDS_BM_REGION_ADDR + Off), Len);
	return 0;
}

static int BtPdsBmWrite(uint32_t Off, const void *pData, uint32_t Len)
{
	if (!RangeValid(Off, Len) || (Off & 3U) != 0U || (Len & 3U) != 0U ||
		(Len > 0U && (pData == nullptr ||
		 ((uintptr_t)pData & 3U) != 0U)))
	{
		return -EINVAL;
	}

	uint32_t *pDst = (uint32_t *)(BT_PDS_BM_REGION_ADDR + Off);
	const uint32_t *pSrc = (const uint32_t *)pData;
	return FlashWriteWords(pDst, pSrc, Len / 4U);
}

static int BtPdsBmErase(uint32_t Off)
{
	if ((Off % BT_PDS_BM_SECTOR_SIZE) != 0U ||
		!RangeValid(Off, BT_PDS_BM_SECTOR_SIZE))
	{
		return -EINVAL;
	}

	uint32_t erased[32];
	memset(erased, 0xFF, sizeof(erased));

	uint32_t *pDst = (uint32_t *)(BT_PDS_BM_REGION_ADDR + Off);
	uint32_t words = BT_PDS_BM_SECTOR_SIZE / 4U;
	uint32_t index = 0U;

	while (index < words)
	{
		uint32_t count = words - index;
		if (count > sizeof(erased) / sizeof(erased[0]))
		{
			count = sizeof(erased) / sizeof(erased[0]);
		}

		int result = FlashWriteWords(&pDst[index], erased, count);
		if (result != 0)
		{
			return result;
		}
		index += count;
	}
	return 0;
}

static const BtPdsNvm_t s_BtPdsBmNvm = {
	.RegionOffset = BT_PDS_BM_REGION_ADDR,
	.RegionSize   = BT_PDS_BM_REGION_SIZE,
	.SectorSize   = BT_PDS_BM_SECTOR_SIZE,
	.WriteGran    = BT_PDS_BM_WRITE_GRAN,
	.Read         = BtPdsBmRead,
	.Write        = BtPdsBmWrite,
	.Erase        = BtPdsBmErase,
};

extern "C" int BtPdsBmInit(void)
{
	return BtPdsInit(&s_BtPdsBmNvm);
}
