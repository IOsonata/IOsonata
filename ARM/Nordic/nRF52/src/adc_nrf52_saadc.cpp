/*--------------------------------------------------------------------------
File   : adc_nrf52_saadc.cpp

Author : Hoang Nguyen Hoan          June 16, 2017

Desc   : ADC implementation for Nordic nRF52 SAADC, also used on nRF5340
         and nRF91. See adc_nrf52_saadc.h for the conversion modes.

Copyright (c) 2017, I-SYST inc., all rights reserved

Permission to use, copy, modify, and distribute this software for any purpose
with or without fee is hereby granted, provided that the above copyright
notice and this permission notice appear in all copies, and none of the
names : I-SYST or its contributors may be used to endorse or
promote products derived from this software without specific prior written
permission.

For info or contributing contact : hnhoan at i-syst dot com

THIS SOFTWARE IS PROVIDED BY THE REGENTS AND CONTRIBUTORS ``AS IS'' AND ANY
EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE REGENTS OR CONTRIBUTORS BE LIABLE FOR ANY
DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
(INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

----------------------------------------------------------------------------
Modified by          Date              Description

----------------------------------------------------------------------------*/
#include <stdint.h>
#include <string.h>

#ifdef __ICCARM__
#include "intrinsics.h"
#endif

#include "adc_nrf52_saadc.h"
#include "coredev/pei.h"

// PEI is optional. These weak calls cannot extract the PEI matrix
// into a normal ADC-only firmware's static HAL link.
#if defined(__GNUC__)
#pragma weak PeiBindEvent
#pragma weak PeiGetEventEndpoint
#pragma weak PeiUnbindEvent
#pragma weak PeiConnectTask
#pragma weak PeiDisconnectTask
#endif

// The nRF91 names its peripherals by security state and the application runs
// secure
#if !defined(NRF_SAADC) && defined(NRF_SAADC_S)
#define NRF_SAADC		NRF_SAADC_S
#endif

// Internal timer clock, SAMPLERATE.CC range
#define SAADC_NRF52_TIMER_CLK		16000000ULL
#define SAADC_NRF52_CC_MIN			80
#define SAADC_NRF52_CC_MAX			2047

// CH[n].LIMIT reset value, HIGH 0x7FFF and LOW 0x8000: no limit event
#define SAADC_NRF52_LIMIT_OFF		0x7FFF8000UL

// Driver result memory, two blocks of one round of all channels
#define SAADC_NRF52_DEF_MEM			(2 * SAADC_NRF52_MAX_CHAN)

#pragma pack(push, 4)
typedef struct __ADC_nRF52_Data {
	AdcnRF52 *pDevObj;
	uint32_t OvrSample;			// OVERSAMPLE register value from the configuration
	int NbChanAct;				// Enabled channels
	uint32_t RoundCnt;			// Rounds converted, time stamp without Timer
	uint16_t ChanState[SAADC_NRF52_MAX_CHAN];	// PSELP | PSELN << 8, 0 when closed
	uint8_t	RefVoltIdx[SAADC_NRF52_MAX_CHAN];
	bool Burst[SAADC_NRF52_MAX_CHAN];			// BurstMode of the channel configuration
	float GainFactor[SAADC_NRF52_MAX_CHAN];		// Volts per count, pre-calculated
	int16_t Last[SAADC_NRF52_MAX_CHAN];			// Result of the last round
	hCFifo_t hFifo[SAADC_NRF52_MAX_CHAN];
	Timer *pTimer;				// Time stamp source, NULL for the round count
	int16_t ResData[SAADC_NRF52_DEF_MEM];		// Driver block memory
	int16_t *pMem;				// Block memory in use
	int MemLen;					// Results that fit in pMem
	int BlkRounds;				// Rounds per block asked
	int BlkUsed;				// Rounds per block of the running conversion
	int BlkChan;				// Channels of the running conversion
	int16_t *pBlk[2];			// The two blocks
	int Cur;					// Block being filled
	uint32_t Cc;				// SAMPLERATE.CC of the rate set, 0 for none
	uint32_t LimitMask;			// INTEN bits of the limits set
	AdcnRF52LimitHandler_t LimitHandler;
	volatile bool bRunning;		// Continuous conversion started
	volatile bool bNew;			// Polling continuous mode, a round not read yet
	bool bIntTimer;				// Running conversion uses the internal timer
} ADCNRF52_DATA;

#pragma pack(pop)

alignas(4) static ADCNRF52_DATA s_AdcnRF52DevData;

// The SAADC writes one result per enabled channel, in channel order, so the
// result of a channel is at the count of enabled channels below it
static int nRF52ADCResIdx(int Chan)
{
	int idx = 0;

	for (int i = 0; i < Chan; i++)
	{
		if (s_AdcnRF52DevData.ChanState[i] != 0)
		{
			idx++;
		}
	}

	return idx;
}

// Enabled channels, the results of one conversion
static int nRF52ADCNbEnabled(void)
{
	return nRF52ADCResIdx(SAADC_NRF52_MAX_CHAN);
}

// INTEN bits of the LIMITH and LIMITL events of a channel
static uint32_t nRF52ADCLimitMask(int Chan)
{
	return 3UL << (SAADC_INTEN_CH0LIMITH_Pos + 2 * Chan);
}

// Oversampling works with one channel only. Without BURST a SAMPLE task takes
// one of the 2^OVERSAMPLE samples and the result comes after all of them, so
// BURST is set on the channel while oversampling: one SAMPLE task, one result.
static void nRF52ADCApplyOversample(void)
{
	int nb = nRF52ADCNbEnabled();
	uint32_t ovr = nb == 1 ? s_AdcnRF52DevData.OvrSample : 0;

	NRF_SAADC->OVERSAMPLE = ovr;

	for (int i = 0; i < SAADC_NRF52_MAX_CHAN; i++)
	{
		if (s_AdcnRF52DevData.ChanState[i] != 0)
		{
			uint32_t chconfig = NRF_SAADC->CH[i].CONFIG & ~SAADC_CH_CONFIG_BURST_Msk;

			if (s_AdcnRF52DevData.Burst[i] || ovr != 0)
			{
				chconfig |= SAADC_CH_CONFIG_BURST_Enabled << SAADC_CH_CONFIG_BURST_Pos;
			}
			NRF_SAADC->CH[i].CONFIG = chconfig;
		}
	}

	s_AdcnRF52DevData.NbChanAct = nb;
}

static void nRF52ADCClearEvents(void)
{
	NRF_SAADC->EVENTS_STARTED = 0;
	NRF_SAADC->EVENTS_END = 0;
	NRF_SAADC->EVENTS_DONE = 0;
	NRF_SAADC->EVENTS_RESULTDONE = 0;
	NRF_SAADC->EVENTS_CALIBRATEDONE = 0;
	NRF_SAADC->EVENTS_STOPPED = 0;
}

static uint32_t nRF52ADCIntMask(void)
{
	return SAADC_INTEN_END_Msk | s_AdcnRF52DevData.LimitMask;
}

// Store NbRound rounds of results, BlkChan results each. The last round is
// kept for the channels read without FIFO. bFifo false leaves the FIFOs out,
// polling single mode returns the results directly.
static void nRF52ADCStore(const int16_t *pRes, int NbRound, bool bFifo)
{
	ADCNRF52_DATA *dev = &s_AdcnRF52DevData;
	int nb = dev->BlkChan;
	uint32_t tsend = 0;

	if (NbRound <= 0 || nb <= 0)
	{
		return;
	}

	if (dev->pTimer != NULL)
	{
		tsend = (uint32_t)dev->pTimer->uSecond();
	}

	for (int k = 0; k < NbRound; k++)
	{
		const int16_t *res = &pRes[k * nb];
		uint32_t ts;

		dev->RoundCnt++;

		if (dev->pTimer == NULL)
		{
			ts = dev->RoundCnt;
		}
		else if (dev->bIntTimer)
		{
			// Rounds of the internal timer are CC / 16 us apart
			ts = tsend - (uint32_t)(((NbRound - 1 - k) * dev->Cc) >> 4);
		}
		else
		{
			ts = tsend;
		}

		int idx = 0;

		for (int i = 0; i < SAADC_NRF52_MAX_CHAN && idx < nb; i++)
		{
			if (dev->ChanState[i] == 0)
			{
				continue;
			}

			if (k == NbRound - 1)
			{
				dev->Last[i] = res[idx];
			}

			if (bFifo && dev->hFifo[i] != NULL)
			{
				AdcData_t *p = (AdcData_t *)CFifoPut(dev->hFifo[i]);

				if (p != NULL)
				{
					p->Chan = i;
					//
					// *** Factor calculation
					// Vin = ADCresult * Reference / (Resolution * Gain)
					// => GainFactor = Reference / (Resolution * Gain)
					// => Vin = ADCresult * GainFactor
					p->Data = (float)res[idx] * dev->GainFactor[i];
					p->Timestamp = ts;
				}
			}
			idx++;
		}
	}
}

// End of a continuous block. The next block is started first so that DMA
// moves on to the other buffer, then the full one is stored.
// Returns true when a block was stored.
static bool nRF52ADCBlockEnd(void)
{
	ADCNRF52_DATA *dev = &s_AdcnRF52DevData;

	if (NRF_SAADC->EVENTS_END == 0)
	{
		return false;
	}

	NRF_SAADC->EVENTS_END = 0;

	if (dev->bRunning == false)
	{
		return false;
	}

	int amount = (int)NRF_SAADC->RESULT.AMOUNT;
	const int16_t *full = dev->pBlk[dev->Cur];

	dev->Cur ^= 1;
	NRF_SAADC->RESULT.PTR = (uint32_t)dev->pBlk[dev->Cur];
	NRF_SAADC->TASKS_START = 1;

	nRF52ADCStore(full, amount / dev->BlkChan, true);

	return true;
}

static void nRF52ADCLimitEvents(void)
{
	ADCNRF52_DATA *dev = &s_AdcnRF52DevData;

	for (int i = 0; i < SAADC_NRF52_MAX_CHAN; i++)
	{
		if ((dev->LimitMask & nRF52ADCLimitMask(i)) == 0)
		{
			continue;
		}
		if (NRF_SAADC->EVENTS_CH[i].LIMITH)
		{
			NRF_SAADC->EVENTS_CH[i].LIMITH = 0;
			if (dev->LimitHandler != NULL && dev->pDevObj != NULL)
			{
				dev->LimitHandler(dev->pDevObj, i, true);
			}
		}
		if (NRF_SAADC->EVENTS_CH[i].LIMITL)
		{
			NRF_SAADC->EVENTS_CH[i].LIMITL = 0;
			if (dev->LimitHandler != NULL && dev->pDevObj != NULL)
			{
				dev->LimitHandler(dev->pDevObj, i, false);
			}
		}
	}
}

extern "C" void SAADC_IRQHandler()
{
	ADCNRF52_DATA *dev = &s_AdcnRF52DevData;

	if (dev->bRunning)
	{
		// Continuous mode
		if (nRF52ADCBlockEnd() && dev->pDevObj != NULL)
		{
			dev->pDevObj->EvtHandler(DEV_EVT_DATA_RDY);
		}
	}
	else if (NRF_SAADC->EVENTS_END)
	{
		// Single mode, one round
		NRF_SAADC->EVENTS_END = 0;

		if (dev->BlkChan > 0)
		{
			nRF52ADCStore(dev->pBlk[0], 1, true);

			if (dev->pDevObj != NULL)
			{
				dev->pDevObj->EvtHandler(DEV_EVT_DATA_RDY);
			}
		}
	}

	nRF52ADCLimitEvents();

	// Make sure the event clears are done before the interrupt returns
	(void)NRF_SAADC->EVENTS_END;
}

bool nRF52ADCWaitForEnd(int32_t Timeout)
{
	do {
		if (NRF_SAADC->EVENTS_END != 0)
		{
			NRF_SAADC->EVENTS_END = 0;
			return true;
		}
	} while (--Timeout > 0);

	return false;
}

bool nRF52ADCWaitBusy(int32_t Timeout)
{
	do {
		if (NRF_SAADC->STATUS == 0)
			return true;
	} while (--Timeout > 0);

	return false;
}

bool nRF52ADCWaitForStop(int32_t Timeout)
{
	do {
		if (NRF_SAADC->EVENTS_STOPPED == 1)
		{
			NRF_SAADC->EVENTS_STOPPED = 0;

			return true;
		}
	} while (--Timeout > 0);

	return false;
}

static bool nRF52ADCWaitStarted(int32_t Timeout)
{
	do {
		if (NRF_SAADC->EVENTS_STARTED != 0)
		{
			NRF_SAADC->EVENTS_STARTED = 0;
			return true;
		}
	} while (--Timeout > 0);

	return false;
}

AdcnRF52::AdcnRF52()
{
	memset(&s_AdcnRF52DevData, 0, sizeof(s_AdcnRF52DevData));
}

AdcnRF52::~AdcnRF52()
{
	s_AdcnRF52DevData.pDevObj = NULL;
}

/**
 * @brief	Execute auto calibration
 *
 * @return	true - success
 */
bool AdcnRF52::Calibrate()
{
	// Polled in both modes. The interrupt is held off so that it does not
	// take the CALIBRATEDONE event.
	if (vbInterrupt)
	{
		NVIC_DisableIRQ(SAADC_IRQn);
	}

	s_AdcnRF52DevData.bRunning = false;

	nRF52ADCWaitBusy(1000000);

	NRF_SAADC->TASKS_STOP = 1;
	nRF52ADCWaitForStop(100000);

	NRF_SAADC->RESULT.MAXCNT = 0;
	NRF_SAADC->SAMPLERATE = SAADC_SAMPLERATE_MODE_Task << SAADC_SAMPLERATE_MODE_Pos;

	NRF_SAADC->EVENTS_CALIBRATEDONE = 0;
	NRF_SAADC->TASKS_CALIBRATEOFFSET = 1;

	bool done = false;
	int32_t timeout = 1000000;

	do {
		if (NRF_SAADC->EVENTS_CALIBRATEDONE != 0)
		{
			done = true;
			break;
		}
	} while (timeout-- > 0);

	// Anomaly 86: a START after the calibration may write a sample to RAM,
	// stop the SAADC first
	NRF_SAADC->TASKS_STOP = 1;
	nRF52ADCWaitForStop(100000);

	nRF52ADCClearEvents();

	if (vbInterrupt)
	{
		NVIC_ClearPendingIRQ(SAADC_IRQn);
		NVIC_EnableIRQ(SAADC_IRQn);
	}

	return done;
}

bool AdcnRF52::Init(const AdcCfg_t &Cfg, Timer *pTimer, DeviceIntrf *pIntrf)
{
	if (s_AdcnRF52DevData.pDevObj != NULL && s_AdcnRF52DevData.pDevObj != this)
		return false;

	// Undocumented : Forced power on just in case that it was turned off.
	*(volatile uint32_t *)((uint32_t)NRF_SAADC + 0xFFC);
	*(volatile uint32_t *)((uint32_t)NRF_SAADC + 0xFFC) = 1;

	memset(&s_AdcnRF52DevData, 0, sizeof(s_AdcnRF52DevData));

	// Stop current process
    NRF_SAADC->TASKS_STOP = 1;
    nRF52ADCWaitForStop(10000);

    // Flush all interrupt
    NRF_SAADC->INTENCLR = 0xFFFFFFFF;
    NVIC_ClearPendingIRQ(SAADC_IRQn);

    NRF_SAADC->RESULT.PTR = 0;
    NRF_SAADC->RESULT.MAXCNT = 0;

	// Clear all events. The channel state is cleared above, disconnect the
	// channels too: a channel left connected would be converted with the
	// others.
	nRF52ADCClearEvents();
	for (int i = 0; i < SAADC_NRF52_MAX_CHAN; i++)
	{
		NRF_SAADC->CH[i].PSELP = 0;
		NRF_SAADC->CH[i].PSELN = 0;
		NRF_SAADC->CH[i].CONFIG = 0;
		NRF_SAADC->CH[i].LIMIT = SAADC_NRF52_LIMIT_OFF;
		NRF_SAADC->EVENTS_CH[i].LIMITH = 0;
		NRF_SAADC->EVENTS_CH[i].LIMITL = 0;
	}

	s_AdcnRF52DevData.pMem = s_AdcnRF52DevData.ResData;
	s_AdcnRF52DevData.MemLen = SAADC_NRF52_DEF_MEM;
	s_AdcnRF52DevData.BlkRounds = 1;
	s_AdcnRF52DevData.pBlk[0] = s_AdcnRF52DevData.ResData;
	s_AdcnRF52DevData.pBlk[1] = s_AdcnRF52DevData.ResData;

	Resolution(Cfg.Resolution);

	/// NOTE: ARM GCC 11 and above generate bad branch on if (ov>0)
	/// when using __builtin_clzl although both __CLZ & __builtin..
	/// generate same value
//#ifdef __ICCARM__
	int ov = 31 - __CLZ(Cfg.OvrSample);
//#else
//	int ov = 31 - __builtin_clzl(Cfg.OvrSample);
//#endif

	if (ov > 0)
	{
		s_AdcnRF52DevData.OvrSample = (ov << SAADC_OVERSAMPLE_OVERSAMPLE_Pos) & SAADC_OVERSAMPLE_OVERSAMPLE_Msk;
	}
	else
	{
		// none
		s_AdcnRF52DevData.OvrSample = SAADC_OVERSAMPLE_OVERSAMPLE_Bypass << SAADC_OVERSAMPLE_OVERSAMPLE_Pos;
	}

	NRF_SAADC->OVERSAMPLE = s_AdcnRF52DevData.OvrSample;
	NRF_SAADC->SAMPLERATE = SAADC_SAMPLERATE_MODE_Task << SAADC_SAMPLERATE_MODE_Pos;

	vMode = Cfg.Mode;
	Rate(Cfg.Rate);

	vbInterrupt = Cfg.bInterrupt;

	s_AdcnRF52DevData.pDevObj = this;

	s_AdcnRF52DevData.pTimer = pTimer;

	SetRefVoltage(Cfg.pRefVolt, Cfg.NbRefVolt);
	SetEvtHandler(Cfg.EvtHandler);

	NVIC_ClearPendingIRQ(SAADC_IRQn);

	if (Cfg.bInterrupt == true)
	{
		vIntPrio = Cfg.IntPrio;

		NRF_SAADC->INTENSET = nRF52ADCIntMask();

		NVIC_SetPriority(SAADC_IRQn, Cfg.IntPrio);
		NVIC_EnableIRQ(SAADC_IRQn);
	}

	NRF_SAADC->ENABLE = (SAADC_ENABLE_ENABLE_Enabled << SAADC_ENABLE_ENABLE_Pos);

	return true;
}

bool AdcnRF52::Enable()
{
	s_AdcnRF52DevData.bRunning = false;

    NRF_SAADC->RESULT.PTR = 0;
    NRF_SAADC->RESULT.MAXCNT = 0;

    nRF52ADCClearEvents();

	// Reconnect the open channels, Disable disconnected them
	for (int i = 0; i < SAADC_NRF52_MAX_CHAN; i++)
	{
		NRF_SAADC->CH[i].PSELP = s_AdcnRF52DevData.ChanState[i] & 0xFF;
		NRF_SAADC->CH[i].PSELN = s_AdcnRF52DevData.ChanState[i] >> 8;
	}

	NVIC_ClearPendingIRQ(SAADC_IRQn);

	if (vbInterrupt == true)
	{
		NRF_SAADC->INTENSET = nRF52ADCIntMask();

		NVIC_SetPriority(SAADC_IRQn, vIntPrio);
		NVIC_EnableIRQ(SAADC_IRQn);
	}

	s_AdcnRF52DevData.RoundCnt = 0;
	NRF_SAADC->ENABLE = (SAADC_ENABLE_ENABLE_Enabled << SAADC_ENABLE_ENABLE_Pos);

	return true;
}

void AdcnRF52::Disable()
{
	s_AdcnRF52DevData.bRunning = false;

    NRF_SAADC->TASKS_STOP = 1;
    nRF52ADCWaitForStop(10000);

    NRF_SAADC->SAMPLERATE = SAADC_SAMPLERATE_MODE_Task << SAADC_SAMPLERATE_MODE_Pos;

    // Flush all interrupt
    NRF_SAADC->INTENCLR = 0xFFFFFFFF;
    NVIC_ClearPendingIRQ(SAADC_IRQn);
	NVIC_DisableIRQ(SAADC_IRQn);

    NRF_SAADC->RESULT.PTR = 0;
    NRF_SAADC->RESULT.MAXCNT = 0;

    // Clear all events
    nRF52ADCClearEvents();

	// Disconnect all channels
	for (int i = 0; i < SAADC_NRF52_MAX_CHAN; i++)
	{
		NRF_SAADC->CH[i].PSELP = 0;
		NRF_SAADC->CH[i].PSELN = 0;
	}

	NRF_SAADC->ENABLE = 0;
}

void AdcnRF52::Reset()
{
	StopConversion();

	Disable();
	Enable();
}

void AdcnRF52::PowerOff()
{
	StopConversion();

	// Undocumented : Turn power off. All registers go back to their reset
	// value, Init is needed again.
	*(volatile uint32_t *)((uint32_t)NRF_SAADC + 0xFFC);
	*(volatile uint32_t *)((uint32_t)NRF_SAADC + 0xFFC) = 1;
	*(volatile uint32_t *)((uint32_t)NRF_SAADC + 0xFFC) = 0;
}

uint32_t AdcnRF52::Rate(uint32_t Val)
{
	// Val in mHz. The internal timer gives 16 MHz / CC, CC 80 to 2047,
	// 200 kHz to 7.8 kHz. The closest is taken.
	if (Val == 0)
	{
		s_AdcnRF52DevData.Cc = 0;
		vRate = 0;

		return vRate;
	}

	uint64_t cc = (SAADC_NRF52_TIMER_CLK * 1000ULL + (Val >> 1)) / Val;

	if (cc < SAADC_NRF52_CC_MIN)
	{
		cc = SAADC_NRF52_CC_MIN;
	}
	else if (cc > SAADC_NRF52_CC_MAX)
	{
		cc = SAADC_NRF52_CC_MAX;
	}

	s_AdcnRF52DevData.Cc = (uint32_t)cc;
	vRate = (uint32_t)((SAADC_NRF52_TIMER_CLK * 1000ULL + (cc >> 1)) / cc);

	return vRate;
}

uint16_t AdcnRF52::Resolution(uint16_t Val)
{
	if (Val < 10)
	{
		// 8 bits
		vResolution = 8;
		NRF_SAADC->RESOLUTION = (SAADC_RESOLUTION_VAL_8bit << SAADC_RESOLUTION_VAL_Pos);
	}
	else if (Val < 12)
	{
		// 10 bits
		vResolution = 10;
		NRF_SAADC->RESOLUTION = (SAADC_RESOLUTION_VAL_10bit << SAADC_RESOLUTION_VAL_Pos);
	}
	else if (Val < 14)
	{
		// 12 bits
		vResolution = 12;
		NRF_SAADC->RESOLUTION = (SAADC_RESOLUTION_VAL_12bit << SAADC_RESOLUTION_VAL_Pos);
	}
	else
	{
		// 14 bits
		vResolution = 14;
		NRF_SAADC->RESOLUTION = (SAADC_RESOLUTION_VAL_14bit << SAADC_RESOLUTION_VAL_Pos);
	}

	return vResolution;
}

bool AdcnRF52::OpenChannel(const AdcChanCfg_t *pChanCfg, int NbChan)
{
	if (pChanCfg == NULL || NbChan <= 0)
		return false;

	for (int i = 0; i < NbChan; i++)
	{
		if (pChanCfg[i].Chan < 0 || pChanCfg[i].Chan >= SAADC_NRF52_MAX_CHAN)
			return false;
	}

	if (s_AdcnRF52DevData.bRunning)
	{
		// Disabling the SAADC below would stop the DMA of the running blocks
		return false;
	}

	NRF_SAADC->ENABLE = 0;

	for (int i = 0; i < NbChan; i++)
	{
		uint32_t chconfig = 0;	// CH[].CONFIG register value

		// The FIFO is looked up by channel number
		if (pChanCfg[i].pFifoMem != NULL && pChanCfg[i].FifoMemSize > CFIFO_TOTAL_MEMSIZE(2, sizeof(AdcData_t)))
		{
			s_AdcnRF52DevData.hFifo[pChanCfg[i].Chan] = CFifoInit(pChanCfg[i].pFifoMem, pChanCfg[i].FifoMemSize, sizeof(AdcData_t), false);
		}
		else
		{
			s_AdcnRF52DevData.hFifo[pChanCfg[i].Chan] = NULL;
		}

		NRF_SAADC->CH[pChanCfg[i].Chan].PSELP = pChanCfg[i].PinP.PinNo + 1;
		s_AdcnRF52DevData.ChanState[pChanCfg[i].Chan] = (pChanCfg[i].PinP.PinNo + 1) & 0xFF;

		switch (pChanCfg[i].PinP.Conn)
		{
			case ADC_PIN_CONN_PULLUP:
				chconfig |= (SAADC_CH_CONFIG_RESP_Pullup << SAADC_CH_CONFIG_RESP_Pos);
				break;
			case ADC_PIN_CONN_PULLDOWN:
				chconfig |= (SAADC_CH_CONFIG_RESP_Pulldown << SAADC_CH_CONFIG_RESP_Pos);
				break;
			case ADC_PIN_CONN_VDD:
				chconfig |= (SAADC_CH_CONFIG_RESP_VDD1_2 << SAADC_CH_CONFIG_RESP_Pos);
				break;
		}

		uint32_t resdiv;

		if (pChanCfg[i].Type == ADC_CHAN_TYPE_DIFFERENTIAL)
		{
			resdiv = 1 << (vResolution - 1);
			NRF_SAADC->CH[pChanCfg[i].Chan].PSELN = pChanCfg[i].PinN.PinNo + 1;

			// In case of differential, Negative channel is kept
			s_AdcnRF52DevData.ChanState[pChanCfg[i].Chan] |= (pChanCfg[i].PinN.PinNo + 1) << 8;

			chconfig |= SAADC_CH_CONFIG_MODE_Diff << SAADC_CH_CONFIG_MODE_Pos;

			switch (pChanCfg[i].PinN.Conn)
			{
				case ADC_PIN_CONN_PULLUP:
					chconfig |= (SAADC_CH_CONFIG_RESN_Pullup << SAADC_CH_CONFIG_RESN_Pos);
					break;
				case ADC_PIN_CONN_PULLDOWN:
					chconfig |= (SAADC_CH_CONFIG_RESN_Pulldown << SAADC_CH_CONFIG_RESN_Pos);
					break;
				case ADC_PIN_CONN_VDD:
					chconfig |= (SAADC_CH_CONFIG_RESN_VDD1_2 << SAADC_CH_CONFIG_RESN_Pos);
					break;
				default:
					;
			}
		}
		else
		{
			// Single ended
			resdiv = 1 << vResolution;
			NRF_SAADC->CH[pChanCfg[i].Chan].PSELN = 0;
		}

		//
		// *** Factor calculation
		// Vin = ADCresult * Reference / (Resolution * Gain)
		// => GainFactor = Reference / (Resolution * Gain)
		//

		// Reference voltage
		s_AdcnRF52DevData.RefVoltIdx[pChanCfg[i].Chan] = pChanCfg[i].RefVoltIdx;

		if (vpRefVolt[pChanCfg[i].RefVoltIdx].Type == ADC_REFVOLT_TYPE_INTERNAL)
		{
			chconfig |= SAADC_CH_CONFIG_REFSEL_Internal << SAADC_CH_CONFIG_REFSEL_Pos;
			s_AdcnRF52DevData.GainFactor[pChanCfg[i].Chan] = 0.6;
		}
		else
		{
			chconfig |= SAADC_CH_CONFIG_REFSEL_VDD1_4 << SAADC_CH_CONFIG_REFSEL_Pos;
			s_AdcnRF52DevData.GainFactor[pChanCfg[i].Chan] = vpRefVolt[s_AdcnRF52DevData.RefVoltIdx[pChanCfg[i].Chan]].Voltage;
		}

		if (pChanCfg[i].Gain & 0xFF)
		{
			// Factional gain
			s_AdcnRF52DevData.GainFactor[pChanCfg[i].Chan] /= (float)resdiv / (float)(pChanCfg[i].Gain & 0xFF);
			chconfig |= ((6 - (pChanCfg[i].Gain & 0xFF)) << SAADC_CH_CONFIG_GAIN_Pos) & SAADC_CH_CONFIG_GAIN_Msk;
		}
		else
		{
			s_AdcnRF52DevData.GainFactor[pChanCfg[i].Chan] /= (float)resdiv * (float)((pChanCfg[i].Gain >> 8)& 0xFF);
//#ifdef __ICCARM__
			chconfig |= ((5 + (31 - __CLZ(pChanCfg[i].Gain >> 8))) << SAADC_CH_CONFIG_GAIN_Pos) & SAADC_CH_CONFIG_GAIN_Msk;
//#else
//			chconfig |= ((5 + (31 - __builtin_clzl(pChanCfg[i].Gain >> 8))) << SAADC_CH_CONFIG_GAIN_Pos) & SAADC_CH_CONFIG_GAIN_Msk;
//#endif
		}

		if (pChanCfg[i].PinP.Conn == ADC_PIN_CONN_VDD)
		{
			s_AdcnRF52DevData.GainFactor[pChanCfg[i].Chan] *= 2.0;
		}

		if (pChanCfg[i].AcqTime < 5)
		{
			// Acquisition time 3us not need to set.
		}
		else if (pChanCfg[i].AcqTime < 10)
		{
			// 5 us
			chconfig |= SAADC_CH_CONFIG_TACQ_5us << SAADC_CH_CONFIG_TACQ_Pos;
		}
		else if (pChanCfg[i].AcqTime < 15)
		{
			// 10 us
			chconfig |= SAADC_CH_CONFIG_TACQ_10us << SAADC_CH_CONFIG_TACQ_Pos;
		}
		else if (pChanCfg[i].AcqTime < 20)
		{
			// 15 us
			chconfig |= SAADC_CH_CONFIG_TACQ_15us << SAADC_CH_CONFIG_TACQ_Pos;
		}
		else if (pChanCfg[i].AcqTime < 40)
		{
			// 15 us
			chconfig |= SAADC_CH_CONFIG_TACQ_20us << SAADC_CH_CONFIG_TACQ_Pos;
		}
		else
		{
			chconfig |= SAADC_CH_CONFIG_TACQ_40us << SAADC_CH_CONFIG_TACQ_Pos;
		}

		// BURST is set by nRF52ADCApplyOversample
		s_AdcnRF52DevData.Burst[pChanCfg[i].Chan] = pChanCfg[i].BurstMode;

		NRF_SAADC->CH[pChanCfg[i].Chan].CONFIG = chconfig;

		// The limits were in counts of the previous configuration
		LimitOff(pChanCfg[i].Chan);
	}

	// Channels opened before stay enabled: a conversion covers all of them.
	// Oversampling only with one channel, scan mode excludes it.
	nRF52ADCApplyOversample();

	NRF_SAADC->ENABLE = (SAADC_ENABLE_ENABLE_Enabled << SAADC_ENABLE_ENABLE_Pos);

	return true;
}

/**
 * @brief	Close ADC channel
 * @param 	Chan : Channel number
 */
void AdcnRF52::CloseChannel(int Chan)
{
	if (Chan < 0 || Chan >= SAADC_NRF52_MAX_CHAN)
		return;

	LimitOff(Chan);

	NRF_SAADC->CH[Chan].PSELP = 0;
	NRF_SAADC->CH[Chan].PSELN = 0;
	NRF_SAADC->CH[Chan].CONFIG = 0;

	s_AdcnRF52DevData.ChanState[Chan] = 0;
	s_AdcnRF52DevData.Burst[Chan] = false;

	// One channel left may oversample again
	nRF52ADCApplyOversample();
}

bool AdcnRF52::StartConversion()
{
	ADCNRF52_DATA *dev = &s_AdcnRF52DevData;
	int nb = dev->NbChanAct;

	if (dev->bRunning)
	{
		// Continuous conversion already running
		return true;
	}

	if (nb <= 0 || nRF52ADCWaitBusy(10000) == false)
		return false;

	dev->BlkChan = nb;
	nRF52ADCClearEvents();

	if (vMode == ADC_CONV_MODE_CONTINUOUS)
	{
		// Two blocks, each one starting on a word. The closest number of
		// rounds that fits the memory is taken.
		int rounds = dev->BlkRounds;

		while (rounds > 1 && 2 * ((rounds * nb + 1) & ~1) > dev->MemLen)
		{
			rounds--;
		}

		int blklen = rounds * nb;

		if (2 * ((blklen + 1) & ~1) > dev->MemLen)
		{
			return false;
		}

		dev->BlkUsed = rounds;
		dev->pBlk[0] = dev->pMem;
		dev->pBlk[1] = dev->pMem + ((blklen + 1) & ~1);
		dev->Cur = 0;
		dev->bNew = false;

		// The internal timer works with one channel only
		dev->bIntTimer = dev->Cc != 0 && nb == 1;

		if (dev->bIntTimer)
		{
			NRF_SAADC->SAMPLERATE = (SAADC_SAMPLERATE_MODE_Timers << SAADC_SAMPLERATE_MODE_Pos) |
									((dev->Cc << SAADC_SAMPLERATE_CC_Pos) & SAADC_SAMPLERATE_CC_Msk);
		}
		else
		{
			NRF_SAADC->SAMPLERATE = SAADC_SAMPLERATE_MODE_Task << SAADC_SAMPLERATE_MODE_Pos;
		}

		NRF_SAADC->RESULT.PTR = (uint32_t)dev->pBlk[0];
		NRF_SAADC->RESULT.MAXCNT = blklen;

		dev->bRunning = true;
		NRF_SAADC->TASKS_START = 1;

		if (nRF52ADCWaitStarted(10000) == false)
		{
			StopConversion();

			return false;
		}

		if (dev->bIntTimer)
		{
			// One SAMPLE task starts the internal timer, STOP stops it
			NRF_SAADC->TASKS_SAMPLE = 1;
		}

		return true;
	}

	// Single mode, one round in the driver memory
	dev->pBlk[0] = dev->ResData;
	dev->bIntTimer = false;

	NRF_SAADC->SAMPLERATE = SAADC_SAMPLERATE_MODE_Task << SAADC_SAMPLERATE_MODE_Pos;
	NRF_SAADC->RESULT.PTR = (uint32_t)dev->ResData;
	NRF_SAADC->RESULT.MAXCNT = nb;
	NRF_SAADC->TASKS_START = 1;

	if (nRF52ADCWaitStarted(10000) == false)
	{
		return false;
	}

	NRF_SAADC->TASKS_SAMPLE = 1;

	return true;
}

void AdcnRF52::StopConversion()
{
	// The interrupt is held off so that it does not start a new block
	bool irq = NVIC_GetEnableIRQ(SAADC_IRQn) != 0;

	if (irq)
	{
		NVIC_DisableIRQ(SAADC_IRQn);
	}

	s_AdcnRF52DevData.bRunning = false;
	s_AdcnRF52DevData.bIntTimer = false;

	NRF_SAADC->TASKS_STOP = 1;

	nRF52ADCWaitForStop(10000);

	NRF_SAADC->SAMPLERATE = SAADC_SAMPLERATE_MODE_Task << SAADC_SAMPLERATE_MODE_Pos;
	NRF_SAADC->RESULT.MAXCNT = 0;

	NRF_SAADC->EVENTS_STARTED = 0;
	NRF_SAADC->EVENTS_END = 0;
	NRF_SAADC->EVENTS_DONE = 0;
	NRF_SAADC->EVENTS_RESULTDONE = 0;

	if (irq)
	{
		NVIC_ClearPendingIRQ(SAADC_IRQn);
		NVIC_EnableIRQ(SAADC_IRQn);
	}
}

// Fill one AdcData_t with the last result of a channel
static void nRF52ADCLast(int Chan, AdcData_t *pBuff)
{
	ADCNRF52_DATA *dev = &s_AdcnRF52DevData;

	pBuff->Chan = Chan;
	//
	// *** Factor calculation
	// Vin = ADCresult * Reference / (Resolution * Gain)
	// => GainFactor = Reference / (Resolution * Gain)
	// => Vin = ADCresult * GainFactor
	pBuff->Data = (float)dev->Last[Chan] * dev->GainFactor[Chan];
	if (dev->pTimer == NULL)
		pBuff->Timestamp = dev->RoundCnt;
	else
		pBuff->Timestamp = (uint32_t)dev->pTimer->uSecond();
}

// Polling mode: take the result of the round started by StartConversion
// (single mode) or the full blocks (continuous mode)
static bool nRF52ADCPoll(ADC_CONV_MODE Mode)
{
	ADCNRF52_DATA *dev = &s_AdcnRF52DevData;

	if (Mode == ADC_CONV_MODE_SINGLE)
	{
		if (dev->BlkChan <= 0 || nRF52ADCWaitForEnd(100000) == false)
		{
			return false;
		}
		nRF52ADCStore(dev->ResData, 1, false);
		dev->bNew = true;
	}
	else if (dev->bRunning && nRF52ADCBlockEnd())
	{
		dev->bNew = true;
	}

	return dev->bNew;
}

int AdcnRF52::Read(AdcData_t *pBuff, int Len)
{
	ADCNRF52_DATA *dev = &s_AdcnRF52DevData;
	// Polling single mode returns the round just converted, no FIFO
	bool direct = vbInterrupt == false && vMode == ADC_CONV_MODE_SINGLE;
	int cnt = 0;

	if (pBuff == NULL || Len <= 0)
		return 0;

	if (vbInterrupt == false)
	{
		nRF52ADCPoll(vMode);
	}

	for (int i = 0; i < SAADC_NRF52_MAX_CHAN && cnt < Len; i++)
	{
		if (dev->ChanState[i] == 0)
		{
			continue;
		}

		if (dev->hFifo[i] != NULL && direct == false)
		{
			AdcData_t *p = (AdcData_t *)CFifoGet(dev->hFifo[i]);
			if (p != NULL)
			{
				memcpy(pBuff, p, sizeof(AdcData_t));
				pBuff++;
				cnt++;
			}
		}
		else if (vbInterrupt || dev->bNew)
		{
			nRF52ADCLast(i, pBuff);
			pBuff++;
			cnt++;
		}
	}

	if (vbInterrupt == false)
	{
		dev->bNew = false;
	}

	return cnt;
}

/**
 * @brief	Read ADC data of one channel
 *
 * @param 	Chan : Channel number
 *  		pBuff : Pointer to buffer for returning data
 *
 * @return	true - data available
 */
bool AdcnRF52::Read(int Chan, AdcData_t *pBuff)
{
	ADCNRF52_DATA *dev = &s_AdcnRF52DevData;
	bool direct = vbInterrupt == false && vMode == ADC_CONV_MODE_SINGLE;

	if (pBuff == NULL || Chan < 0 || Chan >= SAADC_NRF52_MAX_CHAN ||
		dev->ChanState[Chan] == 0)
		return false;

	if (vbInterrupt == false)
	{
		if (nRF52ADCPoll(vMode) == false && direct)
		{
			// No conversion ended
			return false;
		}
	}

	if (dev->hFifo[Chan] != NULL && direct == false)
	{
		AdcData_t *p = (AdcData_t *)CFifoGet(dev->hFifo[Chan]);

		if (p == NULL)
		{
			pBuff->Chan = -1;
			return false;
		}
		memcpy(pBuff, p, sizeof(AdcData_t));

		return true;
	}

	if (dev->RoundCnt == 0)
	{
		// Nothing converted yet
		return false;
	}

	nRF52ADCLast(Chan, pBuff);

	return true;
}

bool AdcnRF52::UpdateData()
{
	if (vbInterrupt == false && vMode == ADC_CONV_MODE_CONTINUOUS)
	{
		return nRF52ADCPoll(vMode);
	}

	return true;
}

int AdcnRF52::BlockRounds(int NbRound, int16_t *pMem, int MemSize)
{
	ADCNRF52_DATA *dev = &s_AdcnRF52DevData;

	if (dev->bRunning)
	{
		return dev->BlkUsed;
	}

	if (NbRound < 1)
	{
		NbRound = 1;
	}

	if (pMem != NULL && ((uintptr_t)pMem & 3) == 0 && MemSize >= (int)(2 * sizeof(int16_t)))
	{
		dev->pMem = pMem;
		dev->MemLen = MemSize / (int)sizeof(int16_t);
	}
	else
	{
		dev->pMem = dev->ResData;
		dev->MemLen = SAADC_NRF52_DEF_MEM;
	}

	dev->BlkRounds = NbRound;

	return NbRound;
}

int AdcnRF52::BlockRounds()
{
	return s_AdcnRF52DevData.bRunning ? s_AdcnRF52DevData.BlkUsed : s_AdcnRF52DevData.BlkRounds;
}

bool AdcnRF52::Limit(int Chan, float LowVolt, float HighVolt)
{
	ADCNRF52_DATA *dev = &s_AdcnRF52DevData;

	if (Chan < 0 || Chan >= SAADC_NRF52_MAX_CHAN || dev->ChanState[Chan] == 0 ||
		vbInterrupt == false || LowVolt > HighVolt || dev->GainFactor[Chan] <= 0.0f)
	{
		return false;
	}

	// Counts of the result, closest value, within the 16 bit fields
	float lim[2] = { LowVolt / dev->GainFactor[Chan], HighVolt / dev->GainFactor[Chan] };
	int32_t cnt[2];

	for (int i = 0; i < 2; i++)
	{
		if (lim[i] >= 32767.0f)
		{
			cnt[i] = 32767;
		}
		else if (lim[i] <= -32768.0f)
		{
			cnt[i] = -32768;
		}
		else
		{
			cnt[i] = (int32_t)(lim[i] + (lim[i] >= 0.0f ? 0.5f : -0.5f));
		}
	}

	uint32_t mask = nRF52ADCLimitMask(Chan);

	NRF_SAADC->INTENCLR = mask;
	NRF_SAADC->CH[Chan].LIMIT = (((uint32_t)(uint16_t)cnt[1] << SAADC_CH_LIMIT_HIGH_Pos) & SAADC_CH_LIMIT_HIGH_Msk) |
								(((uint32_t)(uint16_t)cnt[0] << SAADC_CH_LIMIT_LOW_Pos) & SAADC_CH_LIMIT_LOW_Msk);
	NRF_SAADC->EVENTS_CH[Chan].LIMITH = 0;
	NRF_SAADC->EVENTS_CH[Chan].LIMITL = 0;

	dev->LimitMask |= mask;
	NRF_SAADC->INTENSET = mask;

	return true;
}

void AdcnRF52::LimitOff(int Chan)
{
	ADCNRF52_DATA *dev = &s_AdcnRF52DevData;

	if (Chan < 0 || Chan >= SAADC_NRF52_MAX_CHAN)
	{
		return;
	}

	uint32_t mask = nRF52ADCLimitMask(Chan);

	NRF_SAADC->INTENCLR = mask;
	dev->LimitMask &= ~mask;
	NRF_SAADC->CH[Chan].LIMIT = SAADC_NRF52_LIMIT_OFF;
	NRF_SAADC->EVENTS_CH[Chan].LIMITH = 0;
	NRF_SAADC->EVENTS_CH[Chan].LIMITL = 0;
}

void AdcnRF52::LimitHandler(AdcnRF52LimitHandler_t Handler)
{
	s_AdcnRF52DevData.LimitHandler = Handler;
}


// A prepared ADC owns its channel setup and fixed internal result buffer.
// No new buffers are allocated for PEI. The caller must disconnect all PPI
// routes before changing channels, stopping or disabling the peripheral.
bool AdcnRF52::PeiSamplingReady() const
{
	// Continuous mode: the driver blocks are set while the conversion runs
	// and the END interrupt moves on to the next block. The rounds come from
	// SAMPLE tasks, not from the internal timer.
	if (s_AdcnRF52DevData.pDevObj == this && vMode == ADC_CONV_MODE_CONTINUOUS)
	{
		return s_AdcnRF52DevData.bRunning && s_AdcnRF52DevData.bIntTimer == false &&
			NRF_SAADC->ENABLE ==
				(SAADC_ENABLE_ENABLE_Enabled << SAADC_ENABLE_ENABLE_Pos);
	}

	if (s_AdcnRF52DevData.pDevObj != this ||
		vMode != ADC_CONV_MODE_SINGLE || vbInterrupt ||
		s_AdcnRF52DevData.NbChanAct <= 0 ||
		s_AdcnRF52DevData.NbChanAct > SAADC_NRF52_MAX_CHAN ||
		NRF_SAADC->ENABLE !=
			(SAADC_ENABLE_ENABLE_Enabled << SAADC_ENABLE_ENABLE_Pos) ||
		NRF_SAADC->RESULT.PTR !=
			(uint32_t)(uintptr_t)s_AdcnRF52DevData.ResData ||
		NRF_SAADC->RESULT.MAXCNT !=
			(uint32_t)s_AdcnRF52DevData.NbChanAct)
	{
		return false;
	}
	return true;
}

bool AdcnRF52::PreparePeiSampling()
{
	int i;
	int timeout;

	if (s_AdcnRF52DevData.pDevObj != this ||
		vMode != ADC_CONV_MODE_SINGLE || vbInterrupt ||
		s_AdcnRF52DevData.NbChanAct <= 0 ||
		s_AdcnRF52DevData.NbChanAct > SAADC_NRF52_MAX_CHAN ||
		NRF_SAADC->ENABLE !=
			(SAADC_ENABLE_ENABLE_Enabled << SAADC_ENABLE_ENABLE_Pos) ||
		NRF_SAADC->STATUS != 0U)
	{
		return false;
	}

#if defined(NRF52832_XXAA)
	// nRF52832 erratum 150: PPI-driven START may fail to raise
	// STARTED with TACQ of 5 us or less. END->START uses PPI too.
	for (i = 0; i < SAADC_NRF52_MAX_CHAN; i++)
	{
		if (s_AdcnRF52DevData.ChanState[i] != 0U &&
			((NRF_SAADC->CH[i].CONFIG & SAADC_CH_CONFIG_TACQ_Msk) >>
				SAADC_CH_CONFIG_TACQ_Pos) <= SAADC_CH_CONFIG_TACQ_5us)
		{
			return false;
		}
	}
#else
	(void)i;
#endif

	// Read takes the results of this round buffer
	s_AdcnRF52DevData.BlkChan = s_AdcnRF52DevData.NbChanAct;
	s_AdcnRF52DevData.pBlk[0] = s_AdcnRF52DevData.ResData;
	s_AdcnRF52DevData.bIntTimer = false;

	NRF_SAADC->SAMPLERATE =
		(SAADC_SAMPLERATE_MODE_Task << SAADC_SAMPLERATE_MODE_Pos);
	NRF_SAADC->RESULT.PTR = (uint32_t)(uintptr_t)s_AdcnRF52DevData.ResData;
	NRF_SAADC->RESULT.MAXCNT = s_AdcnRF52DevData.NbChanAct;
	NRF_SAADC->EVENTS_STARTED = 0U;
	NRF_SAADC->EVENTS_END = 0U;
	NRF_SAADC->EVENTS_DONE = 0U;
	NRF_SAADC->EVENTS_RESULTDONE = 0U;
	NRF_SAADC->EVENTS_STOPPED = 0U;
	NRF_SAADC->TASKS_START = 1U;

	timeout = 100000;
	while (NRF_SAADC->EVENTS_STARTED == 0U && timeout-- > 0)
	{
	}
	if (NRF_SAADC->EVENTS_STARTED == 0U)
	{
		NRF_SAADC->TASKS_STOP = 1U;
		nRF52ADCWaitForStop(10000);
		NRF_SAADC->RESULT.PTR = 0U;
		NRF_SAADC->RESULT.MAXCNT = 0U;
		return false;
	}
	NRF_SAADC->EVENTS_STARTED = 0U;
	return PeiSamplingReady();
}

static PeiEndpoint_t AdcNrf52PeiEventEndpoint(ADC_NRF52_PEI_EVENT Event)
{
	switch (Event)
	{
		case ADC_NRF52_PEI_EVENT_STARTED:
			return (PeiEndpoint_t)&NRF_SAADC->EVENTS_STARTED;
		case ADC_NRF52_PEI_EVENT_END:
			return (PeiEndpoint_t)&NRF_SAADC->EVENTS_END;
		case ADC_NRF52_PEI_EVENT_DONE:
			return (PeiEndpoint_t)&NRF_SAADC->EVENTS_DONE;
		case ADC_NRF52_PEI_EVENT_RESULTDONE:
			return (PeiEndpoint_t)&NRF_SAADC->EVENTS_RESULTDONE;
		case ADC_NRF52_PEI_EVENT_STOPPED:
			return (PeiEndpoint_t)&NRF_SAADC->EVENTS_STOPPED;
		default:
			return 0U;
	}
}

static PeiEndpoint_t AdcNrf52PeiTaskEndpoint(ADC_NRF52_PEI_TASK Task)
{
	switch (Task)
	{
		case ADC_NRF52_PEI_TASK_START:
			return (PeiEndpoint_t)&NRF_SAADC->TASKS_START;
		case ADC_NRF52_PEI_TASK_SAMPLE:
			return (PeiEndpoint_t)&NRF_SAADC->TASKS_SAMPLE;
		case ADC_NRF52_PEI_TASK_STOP:
			return (PeiEndpoint_t)&NRF_SAADC->TASKS_STOP;
		default:
			return 0U;
	}
}

extern "C" bool AdcNrf52PreparePeiSampling(void)
{
	return s_AdcnRF52DevData.pDevObj != NULL &&
		s_AdcnRF52DevData.pDevObj->PreparePeiSampling();
}

extern "C" bool AdcNrf52PeiBindEvent(PeiDev_t *pPei, PeiEvent_t *pEvt,
	ADC_NRF52_PEI_EVENT Event)
{
	PeiEndpoint_t endpoint;

	if (pPei == NULL || pEvt == NULL || PeiBindEvent == NULL ||
		s_AdcnRF52DevData.pDevObj == NULL ||
		!s_AdcnRF52DevData.pDevObj->PeiSamplingReady())
	{
		return false;
	}
	endpoint = AdcNrf52PeiEventEndpoint(Event);
	if (endpoint == 0U)
	{
		return false;
	}
	return PeiBindEvent(pPei, pEvt, endpoint);
}

extern "C" bool AdcNrf52PeiUnbindEvent(PeiDev_t *pPei, PeiEvent_t Evt)
{
	PeiEndpoint_t endpoint;
	int i;

	if (pPei == NULL || PeiGetEventEndpoint == NULL ||
		PeiUnbindEvent == NULL)
	{
		return false;
	}
	endpoint = PeiGetEventEndpoint(pPei, Evt);
	for (i = ADC_NRF52_PEI_EVENT_STARTED;
		i <= ADC_NRF52_PEI_EVENT_STOPPED; i++)
	{
		if (endpoint == AdcNrf52PeiEventEndpoint(
			(ADC_NRF52_PEI_EVENT)i))
		{
			return PeiUnbindEvent(pPei, Evt, endpoint);
		}
	}
	return false;
}

extern "C" bool AdcNrf52PeiConnectTask(PeiDev_t *pPei, PeiEvent_t Evt,
	ADC_NRF52_PEI_TASK Task)
{
	PeiEndpoint_t endpoint;

	if (pPei == NULL || PeiConnectTask == NULL ||
		s_AdcnRF52DevData.pDevObj == NULL ||
		!s_AdcnRF52DevData.pDevObj->PeiSamplingReady())
	{
		return false;
	}
	// Continuous mode: the END interrupt starts the next block, START and
	// STOP from PEI would work against it. SAMPLE only.
	if (s_AdcnRF52DevData.pDevObj->Mode() == ADC_CONV_MODE_CONTINUOUS &&
		Task != ADC_NRF52_PEI_TASK_SAMPLE)
	{
		return false;
	}
	endpoint = AdcNrf52PeiTaskEndpoint(Task);
	if (endpoint == 0U)
	{
		return false;
	}
	return PeiConnectTask(pPei, Evt, endpoint);
}

extern "C" bool AdcNrf52PeiDisconnectTask(PeiDev_t *pPei, PeiEvent_t Evt,
	ADC_NRF52_PEI_TASK Task)
{
	PeiEndpoint_t endpoint;

	if (pPei == NULL || PeiDisconnectTask == NULL)
	{
		return false;
	}
	endpoint = AdcNrf52PeiTaskEndpoint(Task);
	if (endpoint == 0U)
	{
		return false;
	}
	return PeiDisconnectTask(pPei, Evt, endpoint);
}
