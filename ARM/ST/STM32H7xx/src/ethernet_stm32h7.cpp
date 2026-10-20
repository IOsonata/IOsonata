/**-------------------------------------------------------------------------
@file	ethernet_stm32h7.cpp

@brief	STM32H753 Ethernet MAC/DMA with a LAN8742A RMII PHY.

@author	Hoang Nguyen Hoan
@date	Oct. 20, 2026

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
#include "stm32h7xx.h"
#include "coredev/ethernet.h"
#include "coredev/interrupt.h"
#include "idelay.h"

#define ETH_RING_COUNT		4
#define ETH_BUFFER_SIZE		1536
#define ETH_DESC_OWN		0x80000000UL
#define ETH_DESC_FD			0x20000000UL
#define ETH_DESC_LD			0x10000000UL
#define ETH_RX_IOC			0x40000000UL
#define ETH_RX_BUF1V		0x01000000UL
#define ETH_RX_ERROR		0x00008000UL
#define ETH_RX_LENGTH		0x00007FFFUL
#define ETH_TX_IOC			0x80000000UL
#define ETH_POLL_US			2000

// Each descriptor occupies its own cache line. DSL skips the four padding
// words. No cache maintenance can discard CPU changes in an adjacent entry.
typedef struct {
	volatile uint32_t Word[4];
	uint32_t Pad[4];
} EthH7Desc_t;

typedef struct {
	alignas(32) EthH7Desc_t Rx[ETH_RING_COUNT];
	alignas(32) EthH7Desc_t Tx[ETH_RING_COUNT];
	alignas(32) uint8_t RxData[ETH_RING_COUNT][ETH_BUFFER_SIZE];
	alignas(32) uint8_t TxData[ETH_RING_COUNT][ETH_BUFFER_SIZE];
} EthH7Mem_t;

static EthH7Mem_t s_EthMem __attribute__((section(".eth_dma"), aligned(32)));
static EthernetDev_t *s_pEthDev;
static EthernetCfg_t s_EthCfg;
static uint32_t s_EthMdioClock;
static volatile uint32_t s_EthRate;
static int s_EthRx;
static int s_EthTx;
static bool s_EthEnabled;
static volatile bool s_EthFault;

static void EthClean(void *pMem, int Len)
{
	if (SCB->CCR & SCB_CCR_DC_Msk)
	{
		SCB_CleanDCache_by_Addr((uint32_t *)pMem, Len);
	}
	__DSB();
}

static void EthInvalidate(void *pMem, int Len)
{
	if (SCB->CCR & SCB_CCR_DC_Msk)
	{
		SCB_InvalidateDCache_by_Addr((uint32_t *)pMem, Len);
	}
	__DSB();
}

static bool EthWaitClear(volatile uint32_t *pReg, uint32_t Mask)
{
	for (int n = 0; n < ETH_POLL_US; n++)
	{
		if ((*pReg & Mask) == 0)
		{
			return true;
		}
		usDelay(1);
	}

	return false;
}

static bool EthMdio(int Reg, uint16_t *pValue, bool bWrite)
{
	if (EthWaitClear(&ETH->MACMDIOAR, ETH_MACMDIOAR_MB) == false)
	{
		return false;
	}

	if (bWrite)
	{
		ETH->MACMDIODR = *pValue;
	}
	ETH->MACMDIOAR = s_EthMdioClock |
		((uint32_t)s_EthCfg.PhyAddr << ETH_MACMDIOAR_PA_Pos) |
		((uint32_t)Reg << ETH_MACMDIOAR_RDA_Pos) |
		(bWrite ? ETH_MACMDIOAR_MOC_WR : ETH_MACMDIOAR_MOC_RD) | ETH_MACMDIOAR_MB;
	if (EthWaitClear(&ETH->MACMDIOAR, ETH_MACMDIOAR_MB) == false)
	{
		return false;
	}

	if (bWrite == false)
	{
		*pValue = (uint16_t)ETH->MACMDIODR;
	}

	return true;
}

static bool EthOwned(DevIntrf_t *pDev)
{
	return s_pEthDev != nullptr && pDev == &s_pEthDev->DevIntrf;
}

static void EthStop(void)
{
	ETH->DMACIER = 0;
	ETH->DMACTCR &= ~ETH_DMACTCR_ST;
	ETH->DMACRCR &= ~ETH_DMACRCR_SR;
	ETH->MACCR &= ~(ETH_MACCR_RE | ETH_MACCR_TE);
	NVIC_DisableIRQ(ETH_IRQn);
}

static void EthStart(void)
{
	if (s_EthEnabled == false || s_EthRate == 0 || s_EthFault)
	{
		return;
	}
	ETH->MACCR |= ETH_MACCR_RE | ETH_MACCR_TE;
	ETH->MTLTQOMR |= ETH_MTLTQOMR_FTQ;
	ETH->DMACTCR |= ETH_DMACTCR_ST;
	ETH->DMACRCR |= ETH_DMACRCR_SR;
	ETH->DMACSR = ETH_DMACSR_TPS | ETH_DMACSR_RPS;
	if (s_EthCfg.bIntEn)
	{
		ETH->DMACIER = ETH_DMACIER_RIE | ETH_DMACIER_TIE |
			ETH_DMACIER_NIE | ETH_DMACIER_AIE | ETH_DMACIER_FBEE;
		NVIC_ClearPendingIRQ(ETH_IRQn);
		NVIC_EnableIRQ(ETH_IRQn);
	}
}

static void EthDisable(DevIntrf_t *pDev)
{
	if (EthOwned(pDev))
	{
		s_EthEnabled = false;
		EthStop();
	}
}

static void EthEnable(DevIntrf_t *pDev)
{
	if (EthOwned(pDev))
	{
		s_EthEnabled = true;
		EthStart();
	}
}

static uint32_t EthGetRate(DevIntrf_t *pDev)
{
	return EthOwned(pDev) && s_EthEnabled && s_EthFault == false ? s_EthRate : 0;
}

static uint32_t EthSetRate(DevIntrf_t *pDev, uint32_t Rate)
{
	(void)Rate;

	return EthGetRate(pDev);
}

static bool EthStartRx(DevIntrf_t *pDev, uint32_t Addr)
{
	(void)Addr;

	return EthOwned(pDev) && s_EthEnabled && s_EthFault == false;
}

static bool EthStartTx(DevIntrf_t *pDev, uint32_t Addr)
{
	(void)Addr;

	return EthGetRate(pDev) != 0;
}

static int EthRx(DevIntrf_t *pDev, uint8_t *pData, int Len)
{
	(void)pDev;

	for (int n = 0; n < ETH_RING_COUNT; n++)
	{
		EthH7Desc_t *desc = &s_EthMem.Rx[s_EthRx];
		uint32_t status;
		int count;
		bool valid;

		EthInvalidate(desc, sizeof(*desc));
		status = desc->Word[3];
		if (status & ETH_DESC_OWN)
		{
			return 0;
		}
		__DMB();
		count = (int)(status & ETH_RX_LENGTH);
		valid = (status & (ETH_DESC_FD | ETH_DESC_LD | ETH_RX_ERROR | 0x40000000UL)) ==
			(ETH_DESC_FD | ETH_DESC_LD) && count >= 14 && count <= ETHERNET_FRAME_SIZE && count <= Len;
		if (valid)
		{
			EthInvalidate(s_EthMem.RxData[s_EthRx], ETH_BUFFER_SIZE);
			memcpy(pData, s_EthMem.RxData[s_EthRx], count);
		}

		// Write-back descriptors replace buffer addresses with status words.
		desc->Word[0] = (uint32_t)(uintptr_t)s_EthMem.RxData[s_EthRx];
		desc->Word[1] = 0;
		desc->Word[2] = 0;
		__DMB();
		desc->Word[3] = ETH_DESC_OWN | ETH_RX_BUF1V | (s_EthCfg.bIntEn ? ETH_RX_IOC : 0);
		EthClean(desc, sizeof(*desc));
		ETH->DMACRDTPR = (uint32_t)(uintptr_t)desc;
		s_EthRx = (s_EthRx + 1) % ETH_RING_COUNT;
		if (valid)
		{
			return count;
		}
	}

	return 0;
}

static int EthTx(DevIntrf_t *pDev, const uint8_t *pData, int Len)
{
	EthH7Desc_t *desc = &s_EthMem.Tx[s_EthTx];

	if (pData == nullptr || Len < 14 || Len > ETHERNET_FRAME_SIZE || EthGetRate(pDev) == 0)
	{
		return 0;
	}
	EthInvalidate(desc, sizeof(*desc));
	if (desc->Word[3] & ETH_DESC_OWN)
	{
		return 0;
	}

	memcpy(s_EthMem.TxData[s_EthTx], pData, Len);
	EthClean(s_EthMem.TxData[s_EthTx], ETH_BUFFER_SIZE);
	desc->Word[0] = (uint32_t)(uintptr_t)s_EthMem.TxData[s_EthTx];
	desc->Word[1] = 0;
	desc->Word[2] = (uint32_t)Len | (s_EthCfg.bIntEn ? ETH_TX_IOC : 0);
	__DMB();
	desc->Word[3] = ETH_DESC_OWN | ETH_DESC_FD | ETH_DESC_LD | (uint32_t)Len;
	EthClean(desc, sizeof(*desc));
	s_EthTx = (s_EthTx + 1) % ETH_RING_COUNT;
	ETH->DMACTDTPR = (uint32_t)(uintptr_t)&s_EthMem.Tx[s_EthTx];

	return Len;
}

static void EthNoOp(DevIntrf_t *pDev)
{
	(void)pDev;
}

static void *EthHandle(DevIntrf_t *pDev)
{
	return EthOwned(pDev) ? s_pEthDev : nullptr;
}

static void EthPowerOff(DevIntrf_t *pDev)
{
	if (EthOwned(pDev))
	{
		EthernetClose(s_pEthDev);
	}
}

static void EthReset(DevIntrf_t *pDev)
{
	EthernetDev_t *dev = s_pEthDev;
	EthernetCfg_t cfg = s_EthCfg;

	if (EthOwned(pDev))
	{
		cfg.EvtCB = pDev->EvtCB;
		EthernetClose(dev);
		EthernetInit(dev, &cfg);
	}
}

void EthernetPollLink(EthernetDev_t *pDev)
{
	uint16_t status;
	uint16_t mode;
	uint32_t rate = 0;
	uint32_t mac;
	bool duplex = false;

	if (pDev != s_pEthDev || s_pEthDev == nullptr || s_EthFault)
	{
		return;
	}

	// BMSR link is latched low; the second read is the current state.
	if (EthMdio(1, &status, false) && EthMdio(1, &status, false) &&
		(status & 0x24U) == 0x24U && EthMdio(31, &mode, false))
	{
		mode = (mode >> 2) & 7U;
		if (mode == 1 || mode == 2 || mode == 5 || mode == 6)
		{
			rate = mode == 2 || mode == 6 ? 100000000UL : 10000000UL;
			duplex = mode == 5 || mode == 6;
		}
	}
	mac = ETH->MACCR;
	if (rate == s_EthRate && ((mac & ETH_MACCR_DM) != 0) == duplex)
	{
		return;
	}

	EthStop();
	s_EthRate = rate;
	mac &= ~(ETH_MACCR_RE | ETH_MACCR_TE | ETH_MACCR_FES | ETH_MACCR_DM);
	ETH->MACCR = mac | (rate == 100000000UL ? ETH_MACCR_FES : 0) | (duplex ? ETH_MACCR_DM : 0);
	EthStart();
	if (pDev->DevIntrf.EvtCB != nullptr)
	{
		pDev->DevIntrf.EvtCB(&pDev->DevIntrf, DEVINTRF_EVT_STATECHG, nullptr, 0);
	}
}

void EthernetClose(EthernetDev_t *pDev)
{
	if (pDev != nullptr && pDev == s_pEthDev)
	{
		EthStop();
		// Reset the peripheral before its memory can be reused by another owner.
		RCC->AHB1RSTR |= RCC_AHB1RSTR_ETH1MACRST;
		(void)RCC->AHB1RSTR;
		RCC->AHB1RSTR &= ~RCC_AHB1RSTR_ETH1MACRST;
		IOPinDis(s_EthCfg.pIOPinMap, s_EthCfg.NbIOPins);
		s_pEthDev = nullptr;
		s_EthRate = 0;
		s_EthEnabled = false;
	}
}

bool EthernetInit(EthernetDev_t *pDev, const EthernetCfg_t *pCfg)
{
	DevIntrf_t *dev;
	uint16_t id1;
	uint16_t id2;
	uint16_t control = 0x8000;
	uint8_t mac = 0;
	uint32_t clock = SystemD2Clock;

	if (pDev == nullptr || pCfg == nullptr || pCfg->DevNo != 0 || pCfg->pIOPinMap == nullptr ||
		pCfg->NbIOPins != 9 || pCfg->PhyAddr < 0 || pCfg->PhyAddr > 31 ||
		pCfg->IntPrio < 0 || pCfg->IntPrio >= (1 << __NVIC_PRIO_BITS) ||
		(pCfg->MacAddr[0] & 1U) || clock < 20000000UL || clock > 300000000UL ||
		(s_pEthDev != nullptr && s_pEthDev != pDev))
	{
		return false;
	}
	for (int i = 0; i < ETHERNET_ADDR_SIZE; i++)
	{
		mac |= pCfg->MacAddr[i];
	}
	if (mac == 0)
	{
		return false;
	}

	EthernetClose(pDev);
	s_EthCfg = *pCfg;
	s_pEthDev = pDev;
	s_EthFault = false;
	s_EthEnabled = true;
	s_EthRate = 0;
	RCC->APB4ENR |= RCC_APB4ENR_SYSCFGEN;
	SYSCFG->PMCR = (SYSCFG->PMCR & ~SYSCFG_PMCR_EPIS_SEL) | SYSCFG_PMCR_EPIS_SEL_2;
	RCC->AHB1ENR |= RCC_AHB1ENR_ETH1MACEN | RCC_AHB1ENR_ETH1TXEN | RCC_AHB1ENR_ETH1RXEN;
	(void)RCC->AHB1ENR;
	IOPinCfg(pCfg->pIOPinMap, pCfg->NbIOPins);
	ETH->DMAMR |= ETH_DMAMR_SWR;
	if (EthWaitClear(&ETH->DMAMR, ETH_DMAMR_SWR) == false)
	{
		EthernetClose(pDev);

		return false;
	}
	s_EthMdioClock = clock < 35000000UL ? ETH_MACMDIOAR_CR_DIV16 :
		clock < 60000000UL ? ETH_MACMDIOAR_CR_DIV26 : clock < 100000000UL ? ETH_MACMDIOAR_CR_DIV42 :
		clock < 150000000UL ? ETH_MACMDIOAR_CR_DIV62 : clock < 250000000UL ? ETH_MACMDIOAR_CR_DIV102 :
		ETH_MACMDIOAR_CR_DIV124;
	if (EthMdio(2, &id1, false) == false || EthMdio(3, &id2, false) == false ||
		id1 != 0x0007 || (id2 & 0xFFF0U) != 0xC130U || EthMdio(0, &control, true) == false)
	{
		EthernetClose(pDev);

		return false;
	}
	for (int n = 0; n < 500; n++)
	{
		if (EthMdio(0, &control, false) == false)
		{
			break;
		}
		if ((control & 0x8000U) == 0)
		{
			break;
		}
		usDelay(1000);
	}
	if (control & 0x8000U)
	{
		EthernetClose(pDev);

		return false;
	}
	control = 0x1200;
	if (EthMdio(0, &control, true) == false)
	{
		EthernetClose(pDev);

		return false;
	}

	memset(&s_EthMem, 0, sizeof(s_EthMem));
	s_EthRx = 0;
	s_EthTx = 0;
	for (int i = 0; i < ETH_RING_COUNT; i++)
	{
		s_EthMem.Rx[i].Word[0] = (uint32_t)(uintptr_t)s_EthMem.RxData[i];
		s_EthMem.Rx[i].Word[3] = ETH_DESC_OWN | ETH_RX_BUF1V | (pCfg->bIntEn ? ETH_RX_IOC : 0);
	}
	EthClean(&s_EthMem, sizeof(s_EthMem));
	ETH->DMASBMR = ETH_DMASBMR_AAL | ETH_DMASBMR_FB;
	ETH->DMACCR = ETH_DMACCR_DSL_128BIT;
	ETH->DMACTCR = ETH_DMACTCR_TPBL_32PBL;
	ETH->DMACRCR = ETH_DMACRCR_RPBL_32PBL | (ETH_BUFFER_SIZE << ETH_DMACRCR_RBSZ_Pos);
	ETH->DMACTDLAR = (uint32_t)(uintptr_t)s_EthMem.Tx;
	ETH->DMACRDLAR = (uint32_t)(uintptr_t)s_EthMem.Rx;
	ETH->DMACTDRLR = ETH_RING_COUNT - 1;
	ETH->DMACRDRLR = ETH_RING_COUNT - 1;
	ETH->DMACTDTPR = (uint32_t)(uintptr_t)s_EthMem.Tx;
	ETH->DMACRDTPR = (uint32_t)(uintptr_t)&s_EthMem.Rx[ETH_RING_COUNT - 1];
	ETH->MACA0LR = pCfg->MacAddr[0] | ((uint32_t)pCfg->MacAddr[1] << 8) |
		((uint32_t)pCfg->MacAddr[2] << 16) | ((uint32_t)pCfg->MacAddr[3] << 24);
	ETH->MACA0HR = pCfg->MacAddr[4] | ((uint32_t)pCfg->MacAddr[5] << 8);
	ETH->MACPFR = 0;
	ETH->MACCR = ETH_MACCR_ACS | ETH_MACCR_CST;
	ETH->MTLTQOMR = ETH_MTLTQOMR_TSF;
	ETH->MTLRQOMR = ETH_MTLRQOMR_RSF;
	ETH->MACIER = 0;
	ETH->MMCRIMR = 0xFFFFFFFFUL;
	ETH->MMCTIMR = 0xFFFFFFFFUL;
	NVIC_SetPriority(ETH_IRQn, pCfg->IntPrio);

	memcpy(pDev->MacAddr, pCfg->MacAddr, ETHERNET_ADDR_SIZE);
	dev = &pDev->DevIntrf;
	dev->pDevData = pDev;
	dev->Type = DEVINTRF_TYPE_ETH;
	dev->IntPrio = pCfg->IntPrio;
	dev->EvtCB = pCfg->EvtCB;
	dev->Disable = EthDisable;
	dev->Enable = EthEnable;
	dev->GetRate = EthGetRate;
	dev->SetRate = EthSetRate;
	dev->StartRx = EthStartRx;
	dev->RxData = EthRx;
	dev->StopRx = EthNoOp;
	dev->StartTx = EthStartTx;
	dev->TxData = EthTx;
	dev->TxSrData = EthTx;
	dev->StopTx = EthNoOp;
	dev->Reset = EthReset;
	dev->PowerOff = EthPowerOff;
	dev->GetHandle = EthHandle;
	dev->MaxRetry = 0;
	dev->bDma = true;
	dev->bIntEn = pCfg->bIntEn;
	dev->EnCnt = 1;
	dev->bTxReady = true;
	dev->bNoStop = false;
	atomic_flag_clear(&dev->bBusy);

	return true;
}

extern "C" void ETH_IRQHandler(void)
{
	EthernetDev_t *dev = s_pEthDev;
	uint32_t status = ETH->DMACSR;

	ETH->DMACSR = status;
	if (dev == nullptr)
	{
		return;
	}
	if (status & ETH_DMACSR_FBE)
	{
		s_EthFault = true;
		s_EthRate = 0;
		ETH->DMACIER = 0;
	}
	if (dev->DevIntrf.EvtCB != nullptr)
	{
		if (status & ETH_DMACSR_RI)
		{
			dev->DevIntrf.EvtCB(&dev->DevIntrf, DEVINTRF_EVT_RX_DATA, nullptr, 0);
		}
		if (status & ETH_DMACSR_TI)
		{
			dev->DevIntrf.EvtCB(&dev->DevIntrf, DEVINTRF_EVT_TX_READY, nullptr, 0);
		}
		if (status & ETH_DMACSR_FBE)
		{
			dev->DevIntrf.EvtCB(&dev->DevIntrf, DEVINTRF_EVT_STATECHG, nullptr, 0);
		}
	}
}
