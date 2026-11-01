/**-------------------------------------------------------------------------
@file	i2s_nrfx.cpp

@brief	I2S (Inter-IC sound interface) implementation on Nordic nRF series


@author	Nguyen Hoan Hoang
@date	Apr. 4, 2020

@license

MIT License

Copyright (c) 2020 I-SYST inc. All rights reserved.

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

#if defined(NRF54LM20A_XXAA) || defined(NRF54LM20B_XXAA)
#include "nrf_tdm.h"
#endif

#include "istddef.h"
#include "idelay.h"
#include "coredev/i2s.h"

// nRF54LM20 replaces the I2S peripheral with TDM. Keep the generic I2S
// interface and map only the hardware backend for these parts.
#if defined(NRF54LM20A_XXAA) || defined(NRF54LM20B_XXAA)
#define IOSONATA_NRF_TDM_AUDIO	1
#define NRF_AUDIO				NRF_TDM
#define AUDIO_IRQn				TDM_IRQn
#elif defined(NRF53_SERIES)
#define IOSONATA_NRF_TDM_AUDIO	0
#define NRF_AUDIO				NRF_I2S0_S
#define NRF_AUDIO_BASE			NRF_I2S0_S_BASE
#define AUDIO_IRQn				I2S0_IRQn
#elif defined(NRF52_SERIES)
#define IOSONATA_NRF_TDM_AUDIO	0
#define NRF_AUDIO				NRF_I2S
#define NRF_AUDIO_BASE			NRF_I2S_BASE
#define AUDIO_IRQn				I2S_IRQn
#else
#define IOSONATA_NRF_TDM_AUDIO	0
#define NRF_AUDIO				NRF_I2S_S
#define NRF_AUDIO_BASE			NRF_I2S_S_BASE
#define AUDIO_IRQn				I2S_IRQn
#endif

#pragma pack(push, 4)
typedef struct
{
	uint32_t Freq;		//!< Clock frequency in Hz
#if IOSONATA_NRF_TDM_AUDIO
	nrf_tdm_mck_div_t MckDiv;
	nrf_tdm_sck_div_t SckDiv;
#else
	uint32_t RegVal;	//!< Register value
#endif
} MCLK_FREQ;
#pragma pack(pop)

static const MCLK_FREQ s_MClkFreqTable[] = {
#if IOSONATA_NRF_TDM_AUDIO
	{256000, NRF_TDM_MCK_DIV_125, NRF_TDM_SCK_DIV_125},
	{507937, NRF_TDM_MCK_DIV_63, NRF_TDM_SCK_DIV_63},
	{761905, NRF_TDM_MCK_DIV_42, NRF_TDM_SCK_DIV_42},
	{1000000, NRF_TDM_MCK_DIV_32, NRF_TDM_SCK_DIV_32},
	{1032258, NRF_TDM_MCK_DIV_31, NRF_TDM_SCK_DIV_31},
	{1066667, NRF_TDM_MCK_DIV_30, NRF_TDM_SCK_DIV_30},
	{1391304, NRF_TDM_MCK_DIV_23, NRF_TDM_SCK_DIV_23},
	{1523810, NRF_TDM_MCK_DIV_21, NRF_TDM_SCK_DIV_21},
	{2000000, NRF_TDM_MCK_DIV_16, NRF_TDM_SCK_DIV_16},
	{2133333, NRF_TDM_MCK_DIV_15, NRF_TDM_SCK_DIV_15},
	{2909091, NRF_TDM_MCK_DIV_11, NRF_TDM_SCK_DIV_11},
	{3200000, NRF_TDM_MCK_DIV_10, NRF_TDM_SCK_DIV_10},
	{4000000, NRF_TDM_MCK_DIV_8, NRF_TDM_SCK_DIV_8},
	{5333333, NRF_TDM_MCK_DIV_6, NRF_TDM_SCK_DIV_6},
	{6400000, NRF_TDM_MCK_DIV_5, NRF_TDM_SCK_DIV_5},
	{8000000, NRF_TDM_MCK_DIV_4, NRF_TDM_SCK_DIV_4},
	{10666667, NRF_TDM_MCK_DIV_3, NRF_TDM_SCK_DIV_3},
	{16000000, NRF_TDM_MCK_DIV_2, NRF_TDM_SCK_DIV_2},
#else
	{256000, I2S_CONFIG_MCKFREQ_MCKFREQ_32MDIV125},	// 256 KHz
	{507937, I2S_CONFIG_MCKFREQ_MCKFREQ_32MDIV63},	// 0.5079365 MHz
	{761905, I2S_CONFIG_MCKFREQ_MCKFREQ_32MDIV42},	// 0.7619048 MHz
	{1000000, I2S_CONFIG_MCKFREQ_MCKFREQ_32MDIV32},	// 1.0 MHz
	{1032258, I2S_CONFIG_MCKFREQ_MCKFREQ_32MDIV31},	// 1.0322581 MHz
	{1066667, I2S_CONFIG_MCKFREQ_MCKFREQ_32MDIV30},	// 1.0666667 MHz
	{1391304, I2S_CONFIG_MCKFREQ_MCKFREQ_32MDIV23},	// 1.3913043 MHz
	{1523810, I2S_CONFIG_MCKFREQ_MCKFREQ_32MDIV21},	// 1.5238095 MHz
	{2000000, I2S_CONFIG_MCKFREQ_MCKFREQ_32MDIV16},	// 2.0 MHz
	{2133333, I2S_CONFIG_MCKFREQ_MCKFREQ_32MDIV15},	// 2.1333333 MHz
	{2909091, I2S_CONFIG_MCKFREQ_MCKFREQ_32MDIV11},	// 2.9090909 MHz
	{3200000, I2S_CONFIG_MCKFREQ_MCKFREQ_32MDIV10},	// 3.2 MHz
	{4000000, I2S_CONFIG_MCKFREQ_MCKFREQ_32MDIV8},	// 4.0 MHz
#if defined(I2S_CONFIG_MCKFREQ_MCKFREQ_32MDIV6)
	{5333333, I2S_CONFIG_MCKFREQ_MCKFREQ_32MDIV6},	// 5.3333333 MHz
	{6400000, I2S_CONFIG_MCKFREQ_MCKFREQ_32MDIV5},	// 6.4 MHz
	{8000000, I2S_CONFIG_MCKFREQ_MCKFREQ_32MDIV4},	// 8.0 MHz
	{10666667, I2S_CONFIG_MCKFREQ_MCKFREQ_32MDIV3},	// 10.6666667 MHz
	{16000000, I2S_CONFIG_MCKFREQ_MCKFREQ_32MDIV2},	// 16.0 MHz
#endif
#endif
};

static const int s_NbFreq = sizeof(s_MClkFreqTable) / sizeof(MCLK_FREQ);

#if !IOSONATA_NRF_TDM_AUDIO
static const uint32_t s_Ratio[] = {
	32, 48, 64, 96, 128, 192, 256, 384, 512
};

alignas(4) static const int s_NbRatio = sizeof(s_Ratio) / sizeof(uint32_t);
#endif

alignas(4) static I2SDev_t *s_pI2SDev = nullptr;

#define I2S_BUFF_MAX		128

alignas(4) static uint32_t s_I2S_RxBuffer[2][I2S_BUFF_MAX];
alignas(4) static uint32_t s_I2S_TxBuffer[2][I2S_BUFF_MAX];
alignas(4) static uint32_t s_I2S_RxReady[I2S_BUFF_MAX];
alignas(4) static uint32_t s_I2S_TxPending[I2S_BUFF_MAX];
static volatile uint32_t s_RxEventCount;
static volatile uint32_t s_TxEventCount;
static volatile bool s_RxReady;
static volatile bool s_TxPending;
static uint32_t s_BlockWords;
static bool s_Running;
static bool s_IntEnabled;

#if IOSONATA_NRF_TDM_AUDIO
static nrf_tdm_config_t s_TdmCfg;
#define NRF_AUDIO_INT_MASK	(NRF_TDM_INT_RXPTRUPD_MASK_MASK | \
	NRF_TDM_INT_TXPTRUPD_MASK_MASK | NRF_TDM_INT_STOPPED_MASK_MASK)
#endif

static void nRFxAudioInterruptsDisable(void)
{
#if IOSONATA_NRF_TDM_AUDIO
	nrf_tdm_int_disable(NRF_AUDIO, NRF_AUDIO_INT_MASK);
#else
	NRF_AUDIO->INTEN = 0;
#endif
}

static void nRFxAudioInterruptsEnable(void)
{
#if IOSONATA_NRF_TDM_AUDIO
	nrf_tdm_int_enable(NRF_AUDIO, NRF_AUDIO_INT_MASK);
#else
	NRF_AUDIO->INTEN = I2S_INTEN_RXPTRUPD_Msk |
		I2S_INTEN_TXPTRUPD_Msk | I2S_INTEN_STOPPED_Msk;
#endif
}

static bool nRFxAudioRxInterruptEnabled(void)
{
#if IOSONATA_NRF_TDM_AUDIO
	return nrf_tdm_int_enable_check(NRF_AUDIO,
		NRF_TDM_INT_RXPTRUPD_MASK_MASK) != 0;
#else
	return (NRF_AUDIO->INTEN & I2S_INTEN_RXPTRUPD_Msk) != 0;
#endif
}

#if defined(NRF53_SERIES)
// HFCLKAUDIO synthesizes exact integer-family and 44.1-kHz-family sample
// clocks. The 32 MHz PCLK dividers do not provide exact 48 kHz.
typedef struct
{
	uint32_t Rate;
	uint32_t Mclk;
	uint32_t MckReg;
	uint32_t RatioReg;
	uint32_t AudioReg;
} I2SAudioRate_t;

static const I2SAudioRate_t s_AudioRates[] = {
	// 48-kHz codec reference: ACLK 12.288 MHz, MCK 6.144 MHz,
	// MCK/LRCK = 128. The CS47L63 FLL uses MCLK1 with N = 8.
	// RATIO register indices: 32,48,64,96,128,192,256,384,512.
	{ 8000,  3072000, I2S_CONFIG_MCKFREQ_MCKFREQ_32MDIV4, 7, 39846 },
	{ 12000, 3072000, I2S_CONFIG_MCKFREQ_MCKFREQ_32MDIV4, 6, 39846 },
	{ 16000, 3072000, I2S_CONFIG_MCKFREQ_MCKFREQ_32MDIV4, 5, 39846 },
	{ 24000, 3072000, I2S_CONFIG_MCKFREQ_MCKFREQ_32MDIV4, 4, 39846 },
	{ 32000, 3072000, I2S_CONFIG_MCKFREQ_MCKFREQ_32MDIV4, 3, 39846 },
	{ 48000, 6144000, 0x66666000U, 4, 39846 },
	{ 96000, 3072000, I2S_CONFIG_MCKFREQ_MCKFREQ_32MDIV4, 0, 39846 },
};

static bool s_AudioClockStarted;
static uint32_t s_AudioClockReg;

static bool nRF53AudioClockStart(void)
{
	uint32_t retry;

	if (s_AudioClockStarted)
	{
		return true;
	}
	NRF_CLOCK_S->HFCLKAUDIO.FREQUENCY = s_AudioClockReg;
	NRF_CLOCK_S->EVENTS_HFCLKAUDIOSTARTED = 0;
	NRF_CLOCK_S->TASKS_HFCLKAUDIOSTART = 1;
	retry = 1000000U;
	while (NRF_CLOCK_S->EVENTS_HFCLKAUDIOSTARTED == 0 && retry-- > 0)
	{
	}
	if (NRF_CLOCK_S->EVENTS_HFCLKAUDIOSTARTED == 0)
	{
		return false;
	}
	NRF_CLOCK_S->EVENTS_HFCLKAUDIOSTARTED = 0;
	s_AudioClockStarted = true;
	return true;
}

static void nRF53AudioClockStop(void)
{
	if (s_AudioClockStarted)
	{
		NRF_CLOCK_S->TASKS_HFCLKAUDIOSTOP = 1;
		s_AudioClockStarted = false;
	}
}
#endif

bool nRFxI2SWaitStop(I2SDev_t * const pDev, int32_t Timeout)
{
	do {
		if (NRF_AUDIO->EVENTS_STOPPED)
		{
			NRF_AUDIO->EVENTS_STOPPED = 0;

			return true;
		}
	} while(Timeout-- > 0);

	return false;
}

bool nRFxI2SWaitRx(I2SDev_t * const pDev, int32_t Timeout)
{
	do {
		if (NRF_AUDIO->EVENTS_RXPTRUPD)
		{
			NRF_AUDIO->EVENTS_RXPTRUPD = 0;

			return true;
		}
	} while(Timeout-- > 0);

	return false;
}

bool nRFxI2SWaitTx(I2SDev_t * const pDev, int32_t Timeout)
{
	do {
		if (NRF_AUDIO->EVENTS_TXPTRUPD)
		{
			NRF_AUDIO->EVENTS_TXPTRUPD = 0;

			return true;
		}
	} while(Timeout-- > 0);

	return false;
}

static void nRFxI2SDisable(DevIntrf_t * const pDev)
{
	NVIC_DisableIRQ(AUDIO_IRQn);
	nRFxAudioInterruptsDisable();
	if (s_Running)
	{
		s_Running = false;
		NRF_AUDIO->EVENTS_STOPPED = 0;
		NRF_AUDIO->TASKS_STOP = 1;
		nRFxI2SWaitStop(s_pI2SDev, 100000);
	}
#if IOSONATA_NRF_TDM_AUDIO
	nrf_tdm_disable(NRF_AUDIO);
#else
	NRF_AUDIO->ENABLE = 0;
#endif
	NRF_AUDIO->EVENTS_RXPTRUPD = 0;
	NRF_AUDIO->EVENTS_TXPTRUPD = 0;
	NRF_AUDIO->EVENTS_STOPPED = 0;
	NVIC_ClearPendingIRQ(AUDIO_IRQn);
#if defined(NRF53_SERIES)
	nRF53AudioClockStop();
#endif
}

static void nRFxI2SEnable(DevIntrf_t * const pDev)
{
#if IOSONATA_NRF_TDM_AUDIO
	bool rxEnabled;
	bool txEnabled;
#endif

#if defined(NRF53_SERIES)
	if (!nRF53AudioClockStart())
	{
		s_Running = false;
		return;
	}
#endif
	if (s_pI2SDev->hRxFifo != NULL)
	{
		CFifoFlush(s_pI2SDev->hRxFifo);
	}
	if (s_pI2SDev->hTxFifo != NULL)
	{
		CFifoFlush(s_pI2SDev->hTxFifo);
	}
#if IOSONATA_NRF_TDM_AUDIO
	rxEnabled = NRF_AUDIO->PSEL.SDIN != UINT32_MAX;
	txEnabled = NRF_AUDIO->PSEL.SDOUT != UINT32_MAX;
	if (rxEnabled && txEnabled)
	{
		nrf_tdm_transfer_direction_set(NRF_AUDIO, NRF_TDM_RXTXEN_DUPLEX);
	}
	else if (rxEnabled)
	{
		nrf_tdm_transfer_direction_set(NRF_AUDIO, NRF_TDM_RXTXEN_RX);
	}
	else if (txEnabled)
	{
		nrf_tdm_transfer_direction_set(NRF_AUDIO, NRF_TDM_RXTXEN_TX);
	}
	else
	{
		s_Running = false;
		return;
	}
	nrf_tdm_mck_set(NRF_AUDIO, NRF_AUDIO->PSEL.MCK != UINT32_MAX);
	nrf_tdm_rx_buffer_set(NRF_AUDIO, s_I2S_RxBuffer[0]);
	nrf_tdm_tx_buffer_set(NRF_AUDIO, s_I2S_TxBuffer[0]);
	nrf_tdm_rx_count_set(NRF_AUDIO, s_BlockWords);
	nrf_tdm_tx_count_set(NRF_AUDIO, s_BlockWords);
#else
	NRF_AUDIO->CONFIG.RXEN = (NRF_AUDIO->PSEL.SDIN != 0xFFFFFFFFUL) ? 1 : 0;
	NRF_AUDIO->CONFIG.TXEN = (NRF_AUDIO->PSEL.SDOUT != 0xFFFFFFFFUL) ? 1 : 0;
	NRF_AUDIO->CONFIG.MCKEN = (NRF_AUDIO->PSEL.MCK != 0xFFFFFFFFUL) ? 1 : 0;
	NRF_AUDIO->RXD.PTR = (uint32_t)s_I2S_RxBuffer[0];
	NRF_AUDIO->TXD.PTR = (uint32_t)s_I2S_TxBuffer[0];
	NRF_AUDIO->RXTXD.MAXCNT = s_BlockWords;
#endif
	s_RxEventCount = 0;
	s_TxEventCount = 0;
	s_RxReady = false;
	s_TxPending = false;
	memset(s_I2S_RxBuffer, 0, sizeof(s_I2S_RxBuffer));
	memset(s_I2S_TxBuffer, 0, sizeof(s_I2S_TxBuffer));
	NRF_AUDIO->EVENTS_RXPTRUPD = 0;
	NRF_AUDIO->EVENTS_TXPTRUPD = 0;
	NRF_AUDIO->EVENTS_STOPPED = 0;
#if IOSONATA_NRF_TDM_AUDIO
	nrf_tdm_enable(NRF_AUDIO);
#else
	NRF_AUDIO->ENABLE = I2S_ENABLE_ENABLE_Msk;
#endif
	if (s_IntEnabled)
	{
		nRFxAudioInterruptsEnable();
		NVIC_ClearPendingIRQ(AUDIO_IRQn);
		NVIC_EnableIRQ(AUDIO_IRQn);
	}
	s_Running = true;
	NRF_AUDIO->TASKS_START = 1;
}

static void nRFxI2SPowerOff(DevIntrf_t * const pDev)
{
	nRFxI2SDisable(pDev);
#if !IOSONATA_NRF_TDM_AUDIO
	*(volatile uint32_t *)((uint32_t)NRF_AUDIO_BASE + 0xFFC);
	*(volatile uint32_t *)((uint32_t)NRF_AUDIO_BASE + 0xFFC) = 1;
	*(volatile uint32_t *)((uint32_t)NRF_AUDIO_BASE + 0xFFC) = 0;
#endif
}

static uint32_t nRFxI2SGetRate(DevIntrf_t * const pDev)
{
	I2SDev_t *dev = (I2SDev_t*)pDev->pDevData;

	return dev->Freq;
}

static uint32_t nRFxI2SSetRate(DevIntrf_t * const pDev, uint32_t Rate)
{
	I2SDev_t *dev;
#if IOSONATA_NRF_TDM_AUDIO
	uint32_t diff;
	uint32_t target;
	uint32_t width;
	int idx;
#endif

	dev = (I2SDev_t *)pDev->pDevData;
	if (dev == nullptr || Rate == 0 || s_Running)
	{
		return 0;
	}
#if IOSONATA_NRF_TDM_AUDIO
	diff = UINT32_MAX;
	idx = 0;
	width = dev->SampleWidth << 4;
	target = Rate * width;
	for (int i = 0; i < s_NbFreq; i++)
	{
		uint32_t actual;
		uint32_t d;

		actual = s_MClkFreqTable[i].Freq;
		d = actual > target ? actual - target : target - actual;
		if (d < diff)
		{
			idx = i;
			diff = d;
		}
	}
	s_TdmCfg.mck_setup = s_MClkFreqTable[idx].MckDiv;
	s_TdmCfg.sck_setup = s_MClkFreqTable[idx].SckDiv;
	nrf_tdm_configure(NRF_AUDIO, &s_TdmCfg);
	nrf_tdm_mck_configure(NRF_AUDIO, NRF_TDM_SRC_PCLK32M, false);
	nrf_tdm_sck_configure(NRF_AUDIO, NRF_TDM_SRC_PCLK32M, false);
	dev->MClkFreq = s_MClkFreqTable[idx].Freq;
	dev->Freq = s_MClkFreqTable[idx].Freq / width;
	return dev->Freq;
#elif defined(NRF53_SERIES)
	unsigned idx = 0;
	uint32_t diff = UINT32_MAX;

	for (unsigned i = 0; i < sizeof(s_AudioRates) / sizeof(s_AudioRates[0]); i++)
	{
		uint32_t actual = s_AudioRates[i].Rate;
		uint32_t d = actual > Rate ? actual - Rate : Rate - actual;

		if (d < diff)
		{
			idx = i;
			diff = d;
		}
	}
	if (s_AudioClockStarted &&
		s_AudioClockReg != s_AudioRates[idx].AudioReg)
	{
		nRF53AudioClockStop();
	}
	s_AudioClockReg = s_AudioRates[idx].AudioReg;
	if (!nRF53AudioClockStart())
	{
		return 0;
	}
	NRF_AUDIO->CONFIG.CLKCONFIG = I2S_CONFIG_CLKCONFIG_CLKSRC_ACLK;
	NRF_AUDIO->CONFIG.MCKFREQ = s_AudioRates[idx].MckReg;
	NRF_AUDIO->CONFIG.RATIO = s_AudioRates[idx].RatioReg;
	dev->MClkFreq = s_AudioRates[idx].Mclk;
	dev->Freq = s_AudioRates[idx].Rate;
	return dev->Freq;
#else
	uint32_t diff;
	int idx;
	int ratio;
	uint32_t width;

	diff = UINT32_MAX;
	idx = 0;
	ratio = 0;
	width = dev->SampleWidth << 4;
	for (int i = 0; i < s_NbFreq; i++)
	{
		uint32_t r = s_MClkFreqTable[i].Freq / Rate;
		if (r > width)
		{
			r = ((r + (width >> 1)) / width) * width;
		}
		else
		{
			r = width;
		}
		bool allowed = false;
		for (int j = 0; j < s_NbRatio; j++)
		{
			if (s_Ratio[j] == r)
			{
				allowed = true;
				break;
			}
		}
		if (!allowed)
		{
			continue;
		}
		uint32_t freq = s_MClkFreqTable[i].Freq / r;
		uint32_t d = freq > Rate ? freq - Rate : Rate - freq;
		if (d < diff)
		{
			idx = i;
			diff = d;
			ratio = r;
		}
	}
	if (ratio == 0)
	{
		return 0;
	}
	dev->MClkFreq = s_MClkFreqTable[idx].Freq;
	NRF_AUDIO->CONFIG.MCKFREQ = s_MClkFreqTable[idx].RegVal;
	for (int i = 0; i < s_NbRatio; i++)
	{
		if (s_Ratio[i] == (uint32_t)ratio)
		{
			NRF_AUDIO->CONFIG.RATIO = i;
			break;
		}
	}
	dev->Freq = dev->MClkFreq / ratio;
	return dev->Freq;
#endif
}

static void nRFxI2SService(void)
{
	uint32_t event;
	uint32_t complete;
	uint32_t next;
	uint8_t *pFifo;
	uint32_t byteCount;

	if (s_pI2SDev == nullptr || !s_Running)
	{
		return;
	}

	byteCount = s_BlockWords * sizeof(uint32_t);
	event = NRF_AUDIO->EVENTS_RXPTRUPD;
	if (event != 0)
	{
		NRF_AUDIO->EVENTS_RXPTRUPD = 0;
		complete = s_RxEventCount;
		next = (complete + 1U) & 1U;
		NRF_AUDIO->RXD.PTR = (uint32_t)s_I2S_RxBuffer[next];
		s_RxEventCount = s_RxEventCount + 1U;
		if (complete > 0)
		{
			complete = (complete - 1U) & 1U;
			memcpy(s_I2S_RxReady, s_I2S_RxBuffer[complete], byteCount);
			s_RxReady = true;
			if (s_pI2SDev->hRxFifo != NULL &&
				CFifoBlockSize(s_pI2SDev->hRxFifo) == byteCount)
			{
				pFifo = CFifoResv(s_pI2SDev->hRxFifo);
				if (pFifo != nullptr)
				{
					memcpy(pFifo, s_I2S_RxBuffer[complete], byteCount);
					CFifoPut(s_pI2SDev->hRxFifo);
				}
			}
			if (s_pI2SDev->DevIntrf.EvtCB != NULL)
			{
				s_pI2SDev->DevIntrf.EvtCB(&s_pI2SDev->DevIntrf,
					DEVINTRF_EVT_RX_DATA,
					(uint8_t *)s_I2S_RxBuffer[complete], byteCount);
			}
		}
	}

	event = NRF_AUDIO->EVENTS_TXPTRUPD;
	if (event != 0)
	{
		NRF_AUDIO->EVENTS_TXPTRUPD = 0;
		next = (s_TxEventCount + 1U) & 1U;
		pFifo = nullptr;
		if (s_pI2SDev->hTxFifo != NULL &&
			CFifoBlockSize(s_pI2SDev->hTxFifo) == byteCount)
		{
			pFifo = CFifoGet(s_pI2SDev->hTxFifo);
		}
		if (pFifo != NULL)
		{
			memcpy(s_I2S_TxBuffer[next], pFifo, byteCount);
		}
		else if (s_TxPending)
		{
			memcpy(s_I2S_TxBuffer[next], s_I2S_TxPending, byteCount);
			s_TxPending = false;
		}
		else
		{
			memset(s_I2S_TxBuffer[next], 0, byteCount);
		}
		NRF_AUDIO->TXD.PTR = (uint32_t)s_I2S_TxBuffer[next];
		s_TxEventCount = s_TxEventCount + 1U;
	}

	if (NRF_AUDIO->EVENTS_STOPPED != 0)
	{
		NRF_AUDIO->EVENTS_STOPPED = 0;
		s_Running = false;
	}
}

bool nRFxI2SStartRx(DevIntrf_t * const pDev, uint32_t DevAddr)
{
	return s_Running;
}

int nRFxI2SRxData(DevIntrf_t * const pDev, uint8_t *pBuff, int BuffLen)
{
	uint32_t remaining;
	uint32_t blockBytes;
	uint8_t *pFifo;

	if (pBuff == nullptr || BuffLen <= 0 || !s_Running)
	{
		return 0;
	}
	blockBytes = s_BlockWords * 4U;
	if ((uint32_t)BuffLen < blockBytes)
	{
		return 0;
	}

	pFifo = NULL;
	remaining = 1000000;
	while (remaining-- != 0)
	{
		if (s_pI2SDev->hRxFifo != NULL)
		{
			pFifo = CFifoGet(s_pI2SDev->hRxFifo);
			if (pFifo != NULL)
			{
				memcpy(pBuff, pFifo, blockBytes);
				return blockBytes;
			}
		}
		else if (s_RxReady)
		{
			s_RxReady = false;
			memcpy(pBuff, s_I2S_RxReady, blockBytes);
			return blockBytes;
		}
		if (!nRFxAudioRxInterruptEnabled())
		{
			nRFxI2SService();
		}
	}

	return 0;
}

void nRFxI2SStopRx(DevIntrf_t * const pDev)
{
}

bool nRFxI2SStartTx(DevIntrf_t * const pDev, uint32_t DevAddr)
{
	return s_Running;
}

int nRFxI2STxData(DevIntrf_t * const pDev, const uint8_t *pBuff, int BuffLen)
{
	uint32_t blockBytes;
	uint8_t *pFifo;

	if (pBuff == NULL || !s_Running)
	{
		return 0;
	}
	blockBytes = s_BlockWords * 4U;
	if (BuffLen < 0 || (uint32_t)BuffLen != blockBytes)
	{
		return 0;
	}

	if (s_pI2SDev->hTxFifo != NULL)
	{
		if (CFifoBlockSize(s_pI2SDev->hTxFifo) != blockBytes)
		{
			return 0;
		}
		pFifo = CFifoResv(s_pI2SDev->hTxFifo);
		if (pFifo == nullptr)
		{
			return 0;
		}
		memcpy(pFifo, pBuff, blockBytes);
		CFifoPut(s_pI2SDev->hTxFifo);
		return blockBytes;
	}
	if (s_TxPending)
	{
		return 0;
	}
	memcpy(s_I2S_TxPending, pBuff, blockBytes);
	s_TxPending = true;
	return blockBytes;
}

void nRFxI2SStopTx(DevIntrf_t * const pDev)
{
}

void nRFxI2SReset(DevIntrf_t * const pDev)
{
	nRFxI2SDisable(pDev);
	msDelay(1);
	nRFxI2SEnable(pDev);
}

/**
 * @brief	Initialize I2S hardware interface
 *
 * This is a required implementation.
 *
 * @param	pDev : Pointer to device private data to be initialized
 * @param	pCfg : Pointer to configuration data
 *
 * @return	true - Initialization success
 */
bool I2SInit(I2SDev_t * const pDev, const I2SCfg_t * const pCfgData)
{
#if IOSONATA_NRF_TDM_AUDIO
	nrf_tdm_pins_t pins;
#endif

	if (pDev == nullptr || pCfgData == nullptr ||
		pCfgData->pIOPinMap == nullptr || pCfgData->NbIOPins < 3 ||
		pCfgData->NbIOPins > 5 || pCfgData->Freq == 0 ||
		pCfgData->DevNo != 0 ||
		(pCfgData->SampleWidth != I2SSWIDTH_16BITS &&
		 pCfgData->SampleWidth != I2SSWIDTH_24BITS))
	{
		return false;
	}

	s_pI2SDev = pDev;

	pDev->DevIntrf.pDevData = pDev;

#if !IOSONATA_NRF_TDM_AUDIO
	// Force power on in case it was powered off previously
	*(volatile uint32_t *)((uint32_t)NRF_AUDIO_BASE + 0xFFC);
	*(volatile uint32_t *)((uint32_t)NRF_AUDIO_BASE + 0xFFC) = 1;
#endif

	pDev->hRxFifo = NULL;
	pDev->hTxFifo = NULL;
	if (pCfgData->pRxFifoMem != NULL)
	{
		pDev->hRxFifo = CFifoInit(pCfgData->pRxFifoMem,
			pCfgData->RxFifoMemSize, I2S_BUFF_MAX * sizeof(uint32_t), true);
		if (pDev->hRxFifo == NULL)
		{
			return false;
		}
	}
	if (pCfgData->pTxFifoMem != NULL)
	{
		pDev->hTxFifo = CFifoInit(pCfgData->pTxFifoMem,
			pCfgData->TxFifoMemSize, I2S_BUFF_MAX * sizeof(uint32_t), true);
		if (pDev->hTxFifo == NULL)
		{
			return false;
		}
	}

	pDev->Mode = pCfgData->Mode;
	pDev->Format = pCfgData->Format;
	pDev->SampleWidth = pCfgData->SampleWidth / 8;
	pDev->Chan = pCfgData->Chan;
	pDev->pIOPinMap = pCfgData->pIOPinMap;
	pDev->NbIOPins = pCfgData->NbIOPins;

	IOPinCfg(pCfgData->pIOPinMap, pCfgData->NbIOPins);

#if IOSONATA_NRF_TDM_AUDIO
	pins.sck_pin = (pDev->pIOPinMap[I2S_SCK_IOPIN_IDX].PortNo << 5) |
		pDev->pIOPinMap[I2S_SCK_IOPIN_IDX].PinNo;
	pins.fsync_pin = (pDev->pIOPinMap[I2S_WS_IOPIN_IDX].PortNo << 5) |
		pDev->pIOPinMap[I2S_WS_IOPIN_IDX].PinNo;
	pins.sdin_pin = pDev->pIOPinMap[I2S_SDI_IOPIN_IDX].PinNo >= 0 ?
		((pDev->pIOPinMap[I2S_SDI_IOPIN_IDX].PortNo << 5) |
		 pDev->pIOPinMap[I2S_SDI_IOPIN_IDX].PinNo) :
		NRF_TDM_PIN_NOT_CONNECTED;
	pins.sdout_pin = NRF_TDM_PIN_NOT_CONNECTED;
	pins.mck_pin = NRF_TDM_PIN_NOT_CONNECTED;
	if (pDev->NbIOPins > 3 && pDev->pIOPinMap[I2S_SDO_IOPIN_IDX].PinNo >= 0)
	{
		pins.sdout_pin = (pDev->pIOPinMap[I2S_SDO_IOPIN_IDX].PortNo << 5) |
			pDev->pIOPinMap[I2S_SDO_IOPIN_IDX].PinNo;
	}
	if (pDev->NbIOPins > 4 && pDev->pIOPinMap[I2S_MCK_IOPIN_IDX].PinNo >= 0)
	{
		pins.mck_pin = (pDev->pIOPinMap[I2S_MCK_IOPIN_IDX].PortNo << 5) |
			pDev->pIOPinMap[I2S_MCK_IOPIN_IDX].PinNo;
	}
	nrf_tdm_pins_set(NRF_AUDIO, &pins);

	memset(&s_TdmCfg, 0, sizeof(s_TdmCfg));
	s_TdmCfg.mode = pCfgData->Mode == I2SMODE_SLAVE ?
		NRF_TDM_MODE_SLAVE : NRF_TDM_MODE_MASTER;
	s_TdmCfg.sample_width = pDev->SampleWidth == 2 ?
		NRF_TDM_SWIDTH_16BIT : NRF_TDM_SWIDTH_24BIT;
	s_TdmCfg.num_of_channels = NRF_TDM_CHANNELS_COUNT_2;
	s_TdmCfg.sck_polarity = NRF_TDM_POLARITY_POSEDGE;
#if NRF_TDM_HAS_CONFIG_FSYNC_DURATION_ENUM
	s_TdmCfg.fsync_duration = NRF_TDM_FSYNC_DURATION_CHANNEL;
#else
	s_TdmCfg.fsync_duration = pCfgData->SampleWidth;
#endif
	if (pCfgData->Format == I2SFMT_NORMAL)
	{
		s_TdmCfg.alignment = NRF_TDM_ALIGN_LEFT;
		s_TdmCfg.fsync_polarity = NRF_TDM_POLARITY_NEGEDGE;
		s_TdmCfg.channel_delay = NRF_TDM_CHANNEL_DELAY_1CK;
	}
	else
	{
		s_TdmCfg.alignment = pCfgData->Format == I2SFMT_ALIGN_RIGHT ?
			NRF_TDM_ALIGN_RIGHT : NRF_TDM_ALIGN_LEFT;
		s_TdmCfg.fsync_polarity = NRF_TDM_POLARITY_POSEDGE;
		s_TdmCfg.channel_delay = NRF_TDM_CHANNEL_DELAY_NONE;
	}
	if (pCfgData->Chan == I2SCHAN_LEFT)
	{
		s_TdmCfg.channels = (nrf_tdm_channel_mask_t)(
			NRF_TDM_CHANNEL_RX0_MASK | NRF_TDM_CHANNEL_TX0_MASK);
	}
	else if (pCfgData->Chan == I2SCHAN_RIGHT)
	{
		s_TdmCfg.channels = (nrf_tdm_channel_mask_t)(
			NRF_TDM_CHANNEL_RX1_MASK | NRF_TDM_CHANNEL_TX1_MASK);
	}
	else
	{
		s_TdmCfg.channels = (nrf_tdm_channel_mask_t)(
			NRF_TDM_CHANNEL_RX0_MASK | NRF_TDM_CHANNEL_RX1_MASK |
			NRF_TDM_CHANNEL_TX0_MASK | NRF_TDM_CHANNEL_TX1_MASK);
	}
#else
	NRF_AUDIO->PSEL.SCK = (pDev->pIOPinMap[I2S_SCK_IOPIN_IDX].PortNo << 5) | pDev->pIOPinMap[I2S_SCK_IOPIN_IDX].PinNo;
	NRF_AUDIO->PSEL.LRCK = (pDev->pIOPinMap[I2S_WS_IOPIN_IDX].PortNo << 5) | pDev->pIOPinMap[I2S_WS_IOPIN_IDX].PinNo;
	NRF_AUDIO->PSEL.SDIN = pDev->pIOPinMap[I2S_SDI_IOPIN_IDX].PinNo >= 0 ?
		((pDev->pIOPinMap[I2S_SDI_IOPIN_IDX].PortNo << 5) | pDev->pIOPinMap[I2S_SDI_IOPIN_IDX].PinNo) :
		0xFFFFFFFFUL;
	NRF_AUDIO->PSEL.SDOUT = -1;
	NRF_AUDIO->PSEL.MCK = -1;

	if (pDev->NbIOPins > 3)
	{
		NRF_AUDIO->PSEL.SDOUT = pDev->pIOPinMap[I2S_SDO_IOPIN_IDX].PinNo >= 0 ?
			((pDev->pIOPinMap[I2S_SDO_IOPIN_IDX].PortNo << 5) | pDev->pIOPinMap[I2S_SDO_IOPIN_IDX].PinNo) :
			0xFFFFFFFFUL;
	}

	if (pDev->NbIOPins > 4)
	{
		NRF_AUDIO->PSEL.MCK = pDev->pIOPinMap[I2S_MCK_IOPIN_IDX].PinNo >= 0 ?
			((pDev->pIOPinMap[I2S_MCK_IOPIN_IDX].PortNo << 5) | pDev->pIOPinMap[I2S_MCK_IOPIN_IDX].PinNo) :
			0xFFFFFFFFUL;
	}

	if (pCfgData->Mode == I2SMODE_SLAVE)
	{
		NRF_AUDIO->CONFIG.MODE = I2S_CONFIG_MODE_MODE_Slave;
	}
	else
	{
		NRF_AUDIO->CONFIG.MODE = I2S_CONFIG_MODE_MODE_Master;
	}

	NRF_AUDIO->CONFIG.FORMAT = 0;
	NRF_AUDIO->CONFIG.SWIDTH = pDev->SampleWidth == 2 ? 1U : 2U;
	NRF_AUDIO->CONFIG.CHANNELS = pCfgData->Chan;

	if (pCfgData->Format != I2SFMT_NORMAL)
	{
		NRF_AUDIO->CONFIG.FORMAT = 1;
		NRF_AUDIO->CONFIG.ALIGN = pCfgData->Format == I2SFMT_ALIGN_RIGHT ? 1 : 0;
	}
#endif

	if (nRFxI2SSetRate(&pDev->DevIntrf, pCfgData->Freq) == 0)
	{
		return false;
	}


	pDev->DevIntrf.EnCnt = 1;
	pDev->DevIntrf.Type = DEVINTRF_TYPE_I2S;
	pDev->DevIntrf.bDma = pCfgData->bDmaEn;
	pDev->DevIntrf.Disable = nRFxI2SDisable;
	pDev->DevIntrf.Enable = nRFxI2SEnable;
	pDev->DevIntrf.PowerOff = nRFxI2SPowerOff;
	pDev->DevIntrf.GetRate = nRFxI2SGetRate;
	pDev->DevIntrf.SetRate = nRFxI2SSetRate;
	pDev->DevIntrf.StartRx = nRFxI2SStartRx;
	pDev->DevIntrf.StopRx = nRFxI2SStopRx;
	pDev->DevIntrf.StartTx = nRFxI2SStartTx;
	pDev->DevIntrf.RxData = nRFxI2SRxData;
	pDev->DevIntrf.TxData = nRFxI2STxData;
	pDev->DevIntrf.StopTx = nRFxI2SStopTx;
	pDev->DevIntrf.Reset = nRFxI2SReset;
	pDev->DevIntrf.IntPrio = pCfgData->IntPrio;
	pDev->DevIntrf.EvtCB = pCfgData->EvtCB;
	pDev->DevIntrf.MaxRetry = pCfgData->MaxRetry;
	atomic_flag_clear(&pDev->DevIntrf.bBusy);

	s_BlockWords = I2S_BUFF_MAX;
#if IOSONATA_NRF_TDM_AUDIO
	nrf_tdm_rx_buffer_set(NRF_AUDIO, s_I2S_RxBuffer[0]);
	nrf_tdm_tx_buffer_set(NRF_AUDIO, s_I2S_TxBuffer[0]);
	nrf_tdm_rx_count_set(NRF_AUDIO, s_BlockWords);
	nrf_tdm_tx_count_set(NRF_AUDIO, s_BlockWords);
#else
	NRF_AUDIO->RXD.PTR = (uint32_t)s_I2S_RxBuffer[0];
	NRF_AUDIO->TXD.PTR = (uint32_t)s_I2S_TxBuffer[0];
	NRF_AUDIO->RXTXD.MAXCNT = s_BlockWords;
#endif
	nRFxAudioInterruptsDisable();
	NRF_AUDIO->EVENTS_RXPTRUPD = 0;
	NRF_AUDIO->EVENTS_STOPPED = 0;
	NRF_AUDIO->EVENTS_TXPTRUPD = 0;

	s_IntEnabled = pCfgData->bIntEn;
	NVIC_DisableIRQ(AUDIO_IRQn);
	NVIC_ClearPendingIRQ(AUDIO_IRQn);
	if (s_IntEnabled)
	{
		NVIC_SetPriority(AUDIO_IRQn, pCfgData->IntPrio);
	}

	nRFxI2SEnable(&pDev->DevIntrf);
	if (!s_Running)
	{
		// Audio clock startup failed. Do not report a usable I2S device.
		nRFxAudioInterruptsDisable();
		NVIC_DisableIRQ(AUDIO_IRQn);
		NVIC_ClearPendingIRQ(AUDIO_IRQn);
		nRFxI2SDisable(&pDev->DevIntrf);
		return false;
	}

	return true;
}

#if IOSONATA_NRF_TDM_AUDIO
extern "C" void TDM_IRQHandler()
#elif defined(NRF53_SERIES)
extern "C" void I2S0_IRQHandler()
#else
extern "C" void I2S_IRQHandler()
#endif
{
	nRFxI2SService();
}
