/**-------------------------------------------------------------------------
@file	ethernet_lpc546xx.cpp

@brief	LPC546xx Ethernet MAC/DMA with a LAN8720A or LAN8742A RMII PHY

EthernetInit (net/ethernet.h) on the ENET of the LPC546xx, the same MAC and
DMA design as the STM32H7 Ethernet (ethernet_stm32h7.cpp): 4 receive and 4
transmit descriptors, 1536 byte buffers in static memory, one frame per
descriptor, the frame copied to and from the buffers, all in the .ram2
section (SRAM1 to SRAM3, not cleared by the startup). Store and forward in
the MTL queues of 2 KB, CRC stripped on receive and added on transmit,
padding added. Registers as the NXP SDK fsl_enet.c: ETHPHYSEL for RMII, the
ENET clock and reset, DMA software reset, burst length 1, the MTL transmit
queue enabled (TXQEN 2), receive queue 0 enabled (RXQ0EN 1). The M4 has no
data cache.

Tail pointers as the NXP SDK examples: the receive tail one past the last
descriptor, the 4 descriptors given to the DMA, written again after each one
given back to resume a DMA stopped on a descriptor it does not own; the
transmit tail the next free descriptor, one past the last when it is the first.

Pins, in this order: REF_CLK (ENET_RX_CLK in RMII), MDIO, MDC, CRS_DV
(ENET_RX_DV), RXD0, RXD1, TX_EN, TXD0, TXD1. The PHY reset pin is left as
the board has it. The PHY is identified (LAN8720A or LAN8742A, OUI 0x0007),
reset and set to auto negotiation; EthernetPollLink reads the result (BMSR,
PHY special control and status register 31) and sets the MAC speed and
duplex.

@author	Hoang Nguyen Hoan
@date	Oct. 27, 2026

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
#include <stddef.h>
#include <stdint.h>
#include <string.h>

#include "LPC546xx.h"

#include "net/ethernet.h"
#include "coredev/interrupt.h"
#include "coredev/system_core_clock.h"
#include "idelay.h"

#ifndef LPC546XX_ETH_POLL_US
// Waits of the DMA software reset and of an MDIO transfer, in us
#define LPC546XX_ETH_POLL_US	2000
#endif

#define ETH_RING_COUNT			4
#define ETH_BUFFER_SIZE			1536
#define ETH_NB_PINS				9
#define ETH_DESC_OWN			0x80000000UL
#define ETH_DESC_FD				0x20000000UL
#define ETH_DESC_LD				0x10000000UL
#define ETH_RX_IOC				0x40000000UL
#define ETH_RX_BUF1V			0x01000000UL
#define ETH_RX_CTXT				0x40000000UL	// Write back: context descriptor
#define ETH_RX_ERROR			0x00008000UL	// Write back: error summary
#define ETH_RX_LENGTH			0x00007FFFUL
#define ETH_TX_IOC				0x80000000UL
#define ETH_PHYSEL_RMII			SYSCON_ETHPHYSEL_PHY_SEL_MASK
#define ETH_AHBCLK_IDX			2				// AHBCLKCTRL2 and PRESETCTRL2
#define ETH_MTL_QUEUE_SIZE		7U				// (2048 / 256) - 1
#define ETH_MDIO_WRITE			1U				// MOC
#define ETH_MDIO_READ			3U

// LAN8720A and LAN8742A: OUI 0x0007, model in PHYID2 bits 15:4
#define ETH_PHY_ID1				0x0007U
#define ETH_PHY_ID2_LAN8720A	0xC0F0U
#define ETH_PHY_ID2_LAN8742A	0xC130U

typedef struct {
	volatile uint32_t Word[4];
} EthLpcDesc_t;

typedef struct {
	alignas(16) EthLpcDesc_t Rx[ETH_RING_COUNT];
	alignas(16) EthLpcDesc_t Tx[ETH_RING_COUNT];
	alignas(4) uint8_t RxData[ETH_RING_COUNT][ETH_BUFFER_SIZE];
	alignas(4) uint8_t TxData[ETH_RING_COUNT][ETH_BUFFER_SIZE];
} EthLpcMem_t;

// In SRAM1 to SRAM3 (gcc_lpc54628.ld), set up by EthernetInit
static EthLpcMem_t s_EthMem __attribute__((section(".ram2")));
static EthernetDev_t *s_pEthDev;
static EthernetCfg_t s_EthCfg;
static uint32_t s_EthMdioClock;
static volatile uint32_t s_EthRate;
static int s_EthRx;
static int s_EthTx;
static bool s_EthEnabled;
static volatile bool s_EthFault;

static bool EthWaitClear(volatile uint32_t *pReg, uint32_t Mask)
{
	for (int n = 0; n < LPC546XX_ETH_POLL_US; n++)
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
	if (EthWaitClear(&ENET->MAC_MDIO_ADDR, ENET_MAC_MDIO_ADDR_MB_MASK) == false)
	{
		return false;
	}

	if (bWrite)
	{
		ENET->MAC_MDIO_DATA = *pValue;
	}
	ENET->MAC_MDIO_ADDR = s_EthMdioClock | ENET_MAC_MDIO_ADDR_PA((uint32_t)s_EthCfg.PhyAddr) |
						  ENET_MAC_MDIO_ADDR_RDA((uint32_t)Reg) |
						  ENET_MAC_MDIO_ADDR_MOC(bWrite ? ETH_MDIO_WRITE : ETH_MDIO_READ) |
						  ENET_MAC_MDIO_ADDR_MB_MASK;
	if (EthWaitClear(&ENET->MAC_MDIO_ADDR, ENET_MAC_MDIO_ADDR_MB_MASK) == false)
	{
		return false;
	}

	if (bWrite == false)
	{
		*pValue = (uint16_t)ENET->MAC_MDIO_DATA;
	}

	return true;
}

static bool EthOwned(DevIntrf_t *pDev)
{
	return s_pEthDev != nullptr && pDev == &s_pEthDev->DevIntrf;
}

static void EthStop(void)
{
	ENET->DMA_CH[0].DMA_CHX_INT_EN = 0;
	ENET->DMA_CH[0].DMA_CHX_TX_CTRL &= ~ENET_DMA_CH_DMA_CHX_TX_CTRL_ST_MASK;
	ENET->DMA_CH[0].DMA_CHX_RX_CTRL &= ~ENET_DMA_CH_DMA_CHX_RX_CTRL_SR_MASK;
	ENET->MAC_CONFIG &= ~(ENET_MAC_CONFIG_RE_MASK | ENET_MAC_CONFIG_TE_MASK);
	NVIC_DisableIRQ(ETHERNET_IRQn);
}

static void EthStart(void)
{
	if (s_EthEnabled == false || s_EthRate == 0 || s_EthFault)
	{
		return;
	}
	ENET->MAC_CONFIG |= ENET_MAC_CONFIG_RE_MASK | ENET_MAC_CONFIG_TE_MASK;
	ENET->MTL_QUEUE[0].MTL_TXQX_OP_MODE |= ENET_MTL_QUEUE_MTL_TXQX_OP_MODE_FTQ_MASK;
	ENET->DMA_CH[0].DMA_CHX_TX_CTRL |= ENET_DMA_CH_DMA_CHX_TX_CTRL_ST_MASK;
	ENET->DMA_CH[0].DMA_CHX_RX_CTRL |= ENET_DMA_CH_DMA_CHX_RX_CTRL_SR_MASK;
	ENET->DMA_CH[0].DMA_CHX_STAT = ENET_DMA_CH_DMA_CHX_STAT_TPS_MASK | ENET_DMA_CH_DMA_CHX_STAT_RPS_MASK;
	if (s_EthCfg.bIntEn)
	{
		ENET->DMA_CH[0].DMA_CHX_INT_EN = ENET_DMA_CH_DMA_CHX_INT_EN_RIE_MASK | ENET_DMA_CH_DMA_CHX_INT_EN_TIE_MASK |
										 ENET_DMA_CH_DMA_CHX_INT_EN_NIE_MASK | ENET_DMA_CH_DMA_CHX_INT_EN_AIE_MASK |
										 ENET_DMA_CH_DMA_CHX_INT_EN_FBEE_MASK;
		NVIC_ClearPendingIRQ(ETHERNET_IRQn);
		NVIC_EnableIRQ(ETHERNET_IRQn);
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

// The link sets the rate
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

// One complete frame, or 0. A frame in error or too long for the buffer is
// dropped and the next one is looked at.
static int EthRx(DevIntrf_t *pDev, uint8_t *pData, int Len)
{
	if (EthOwned(pDev) == false || pData == nullptr)
	{
		return 0;
	}

	for (int n = 0; n < ETH_RING_COUNT; n++)
	{
		EthLpcDesc_t *desc = &s_EthMem.Rx[s_EthRx];
		uint32_t status = desc->Word[3];

		if (status & ETH_DESC_OWN)
		{
			return 0;
		}
		__DMB();

		int count = (int)(status & ETH_RX_LENGTH);
		bool valid = (status & (ETH_DESC_FD | ETH_DESC_LD | ETH_RX_ERROR | ETH_RX_CTXT)) ==
					 (ETH_DESC_FD | ETH_DESC_LD) && count >= 14 && count <= ETHERNET_FRAME_SIZE && count <= Len;

		if (valid)
		{
			memcpy(pData, s_EthMem.RxData[s_EthRx], count);
		}

		// The write back format replaced the buffer address: give it again
		desc->Word[0] = (uint32_t)(uintptr_t)s_EthMem.RxData[s_EthRx];
		desc->Word[1] = 0;
		desc->Word[2] = 0;
		__DMB();
		desc->Word[3] = ETH_DESC_OWN | ETH_RX_BUF1V | (s_EthCfg.bIntEn ? ETH_RX_IOC : 0);
		ENET->DMA_CH[0].DMA_CHX_RXDESC_TAIL_PTR = (uint32_t)(uintptr_t)(s_EthMem.Rx + ETH_RING_COUNT);
		s_EthRx = (s_EthRx + 1) % ETH_RING_COUNT;
		if (valid)
		{
			return count;
		}
	}

	return 0;
}

// One frame of 14 to 1514 bytes, without FCS
static int EthTx(DevIntrf_t *pDev, const uint8_t *pData, int Len)
{
	EthLpcDesc_t *desc = &s_EthMem.Tx[s_EthTx];

	if (pData == nullptr || Len < 14 || Len > ETHERNET_FRAME_SIZE || EthGetRate(pDev) == 0)
	{
		return 0;
	}
	if (desc->Word[3] & ETH_DESC_OWN)
	{
		return 0;
	}

	memcpy(s_EthMem.TxData[s_EthTx], pData, Len);
	desc->Word[0] = (uint32_t)(uintptr_t)s_EthMem.TxData[s_EthTx];
	desc->Word[1] = 0;
	desc->Word[2] = (uint32_t)Len | (s_EthCfg.bIntEn ? ETH_TX_IOC : 0);
	__DMB();
	desc->Word[3] = ETH_DESC_OWN | ETH_DESC_FD | ETH_DESC_LD | (uint32_t)Len;
	s_EthTx = (s_EthTx + 1) % ETH_RING_COUNT;
	ENET->DMA_CH[0].DMA_CHX_TXDESC_TAIL_PTR =
		(uint32_t)(uintptr_t)(s_EthMem.Tx + (s_EthTx == 0 ? ETH_RING_COUNT : s_EthTx));

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
	bool duplex = false;

	if (pDev == nullptr || pDev != s_pEthDev || s_EthFault)
	{
		return;
	}

	// BMSR link status is latched low: the second read is the current state.
	// Link up and auto negotiation complete, then the speed indication of
	// register 31: 1 10BASE-T half, 5 full, 2 100BASE-TX half, 6 full.
	if (EthMdio(1, &status, false) && EthMdio(1, &status, false) && (status & 0x24U) == 0x24U &&
		EthMdio(31, &mode, false))
	{
		mode = (mode >> 2) & 7U;
		if (mode == 1 || mode == 2 || mode == 5 || mode == 6)
		{
			rate = mode == 2 || mode == 6 ? 100000000UL : 10000000UL;
			duplex = mode == 5 || mode == 6;
		}
	}

	uint32_t mac = ENET->MAC_CONFIG;

	if (rate == s_EthRate && ((mac & ENET_MAC_CONFIG_DM_MASK) != 0) == duplex)
	{
		return;
	}

	EthStop();
	s_EthRate = rate;
	mac &= ~(ENET_MAC_CONFIG_RE_MASK | ENET_MAC_CONFIG_TE_MASK | ENET_MAC_CONFIG_FES_MASK | ENET_MAC_CONFIG_DM_MASK);
	ENET->MAC_CONFIG = mac | (rate == 100000000UL ? ENET_MAC_CONFIG_FES_MASK : 0) | (duplex ? ENET_MAC_CONFIG_DM_MASK : 0);
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
		// Reset before the memory of the rings can be used by another owner
		SYSCON->PRESETCTRLSET[ETH_AHBCLK_IDX] = SYSCON_PRESETCTRL_ETH_RST_MASK;
		SYSCON->PRESETCTRLCLR[ETH_AHBCLK_IDX] = SYSCON_PRESETCTRL_ETH_RST_MASK;
		SYSCON->AHBCLKCTRLCLR[ETH_AHBCLK_IDX] = SYSCON_AHBCLKCTRL_ETH_MASK;
		IOPinDis(s_EthCfg.pIOPinMap, s_EthCfg.NbIOPins);
		s_pEthDev = nullptr;
	}
}

// MDC from the AHB clock, 2.5 MHz or less (CR of the MAC_MDIO_ADDR register)
static uint32_t EthMdioClock(uint32_t Clk)
{
	uint32_t cr = Clk < 35000000UL ? 2U : Clk < 60000000UL ? 3U : Clk < 100000000UL ? 0U :
				  Clk < 150000000UL ? 1U : Clk < 250000000UL ? 4U : 5U;

	return ENET_MAC_MDIO_ADDR_CR(cr);
}

bool EthernetInit(EthernetDev_t *pDev, const EthernetCfg_t *pCfg)
{
	DevIntrf_t *dev;
	uint16_t id1;
	uint16_t id2;
	uint16_t control = 0x8000;
	uint8_t mac = 0;
	uint32_t clock = SystemCoreClockGet();

	if (pDev == nullptr || pCfg == nullptr || pCfg->DevNo != 0 || pCfg->pIOPinMap == nullptr ||
		pCfg->NbIOPins != ETH_NB_PINS || pCfg->PhyAddr < 0 || pCfg->PhyAddr > 31 || pCfg->IntPrio < 0 ||
		pCfg->IntPrio >= (1 << __NVIC_PRIO_BITS) || (pCfg->MacAddr[0] & 1U) || clock < 20000000UL ||
		clock > 300000000UL || (s_pEthDev != nullptr && s_pEthDev != pDev))
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

	// RMII selected while the ENET is in reset, then the DMA software reset,
	// which needs the reference clock of the PHY
	IOPinCfg(pCfg->pIOPinMap, pCfg->NbIOPins);
	SYSCON->AHBCLKCTRLSET[ETH_AHBCLK_IDX] = SYSCON_AHBCLKCTRL_ETH_MASK;
	SYSCON->PRESETCTRLSET[ETH_AHBCLK_IDX] = SYSCON_PRESETCTRL_ETH_RST_MASK;
	SYSCON->ETHPHYSEL = ETH_PHYSEL_RMII;
	SYSCON->PRESETCTRLCLR[ETH_AHBCLK_IDX] = SYSCON_PRESETCTRL_ETH_RST_MASK;
	ENET->DMA_MODE |= ENET_DMA_MODE_SWR_MASK;
	if (EthWaitClear(&ENET->DMA_MODE, ENET_DMA_MODE_SWR_MASK) == false)
	{
		EthernetClose(pDev);

		return false;
	}

	s_EthMdioClock = EthMdioClock(clock);
	if (EthMdio(2, &id1, false) == false || EthMdio(3, &id2, false) == false || id1 != ETH_PHY_ID1 ||
		((id2 & 0xFFF0U) != ETH_PHY_ID2_LAN8720A && (id2 & 0xFFF0U) != ETH_PHY_ID2_LAN8742A) ||
		EthMdio(0, &control, true) == false)
	{
		EthernetClose(pDev);

		return false;
	}
	// The PHY reset clears BMCR bit 15 when it ends
	for (int n = 0; n < 500; n++)
	{
		if (EthMdio(0, &control, false) == false || (control & 0x8000U) == 0)
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
	// Auto negotiation on and restarted
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

	// DMA: burst of 1, 16 byte descriptors next to each other
	ENET->DMA_CH[0].DMA_CHX_CTRL = 0;
	ENET->DMA_CH[0].DMA_CHX_TX_CTRL = ENET_DMA_CH_DMA_CHX_TX_CTRL_TxPBL(1);
	ENET->DMA_CH[0].DMA_CHX_RX_CTRL = ENET_DMA_CH_DMA_CHX_RX_CTRL_RxPBL(1) |
									  ENET_DMA_CH_DMA_CHX_RX_CTRL_RBSZ(ETH_BUFFER_SIZE >> 2);
	ENET->DMA_CH[0].DMA_CHX_TXDESC_LIST_ADDR = (uint32_t)(uintptr_t)s_EthMem.Tx;
	ENET->DMA_CH[0].DMA_CHX_RXDESC_LIST_ADDR = (uint32_t)(uintptr_t)s_EthMem.Rx;
	ENET->DMA_CH[0].DMA_CHX_TXDESC_RING_LENGTH = ETH_RING_COUNT - 1;
	ENET->DMA_CH[0].DMA_CHX_RXDESC_RING_LENGTH = ETH_RING_COUNT - 1;
	ENET->DMA_CH[0].DMA_CHX_TXDESC_TAIL_PTR = (uint32_t)(uintptr_t)s_EthMem.Tx;
	ENET->DMA_CH[0].DMA_CHX_RXDESC_TAIL_PTR = (uint32_t)(uintptr_t)(s_EthMem.Rx + ETH_RING_COUNT);

	// MTL: store and forward, the queues of 2 KB, transmit queue 0 enabled
	ENET->MTL_QUEUE[0].MTL_TXQX_OP_MODE = ENET_MTL_QUEUE_MTL_TXQX_OP_MODE_TSF_MASK |
										  ENET_MTL_QUEUE_MTL_TXQX_OP_MODE_TXQEN(2) |
										  ENET_MTL_QUEUE_MTL_TXQX_OP_MODE_TQS(ETH_MTL_QUEUE_SIZE);
	ENET->MTL_QUEUE[0].MTL_RXQX_OP_MODE = ENET_MTL_QUEUE_MTL_RXQX_OP_MODE_RSF_MASK |
										  ENET_MTL_QUEUE_MTL_RXQX_OP_MODE_RQS(ETH_MTL_QUEUE_SIZE);

	// MAC: address 0, broadcast and own address, MII port (10/100), CRC
	// stripped, speed and duplex set by EthernetPollLink
	ENET->MAC_ADDR_LOW = pCfg->MacAddr[0] | ((uint32_t)pCfg->MacAddr[1] << 8) | ((uint32_t)pCfg->MacAddr[2] << 16) |
						 ((uint32_t)pCfg->MacAddr[3] << 24);
	ENET->MAC_ADDR_HIGH = pCfg->MacAddr[4] | ((uint32_t)pCfg->MacAddr[5] << 8);
	ENET->MAC_FRAME_FILTER = 0;
	ENET->MAC_1US_TIC_COUNTR = ENET_MAC_1US_TIC_COUNTR_TIC_1US_CNTR(clock / 1000000UL - 1U);
	ENET->MAC_CONFIG = ENET_MAC_CONFIG_PS_MASK | ENET_MAC_CONFIG_ACS_MASK | ENET_MAC_CONFIG_CST_MASK;
	ENET->MAC_RXQ_CTRL[0] = ENET_MAC_RXQ_CTRL_RXQ0EN(1);
	ENET->MAC_INTR_EN = 0;
	NVIC_SetPriority(ETHERNET_IRQn, pCfg->IntPrio);

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

extern "C" void ETHERNET_IRQHandler(void)
{
	EthernetDev_t *dev = s_pEthDev;
	uint32_t status = ENET->DMA_CH[0].DMA_CHX_STAT;

	ENET->DMA_CH[0].DMA_CHX_STAT = status;
	if (dev == nullptr)
	{
		return;
	}
	if (status & ENET_DMA_CH_DMA_CHX_STAT_FBE_MASK)
	{
		s_EthFault = true;
		ENET->DMA_CH[0].DMA_CHX_INT_EN = 0;
	}
	if (dev->DevIntrf.EvtCB != nullptr)
	{
		if (status & ENET_DMA_CH_DMA_CHX_STAT_RI_MASK)
		{
			dev->DevIntrf.EvtCB(&dev->DevIntrf, DEVINTRF_EVT_RX_DATA, nullptr, 0);
		}
		if (status & ENET_DMA_CH_DMA_CHX_STAT_TI_MASK)
		{
			dev->DevIntrf.EvtCB(&dev->DevIntrf, DEVINTRF_EVT_TX_READY, nullptr, 0);
		}
		if (status & ENET_DMA_CH_DMA_CHX_STAT_FBE_MASK)
		{
			dev->DevIntrf.EvtCB(&dev->DevIntrf, DEVINTRF_EVT_STATECHG, nullptr, 0);
		}
	}
}
