/**-------------------------------------------------------------------------
@file	qspi_nrfx.cpp

@brief	Quad SPI implementation on nRFx series MCU

Note: SPI device are shared with other device such as I2C therefore be careful
not to use the same device number on an other device.

For 3 wire support : Set both MISO & MOSI to the same pin.
For QSPI support where available is the last device of the list.

@author	Hoang Nguyen Hoan
@date	Apr. 11, 2021

@license

Copyright (c) 2021, I-SYST inc., all rights reserved

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

----------------------------------------------------------------------------*/
#include "nrf.h"

#include "istddef.h"
#include "coredev/spi.h"
#include "iopinctrl.h"
#include "coredev/shared_irq.h"
#include "diskio_flash.h"
#include "convutil.h"

#include "spi_nrfx.h"

extern NrfSpiDev_t s_nRFxSPIDev[NRFX_SPI_MAXDEV];

#if defined(NRF52840_XXAA) || defined(NRF5340_XXAA_APPLICATION)

static bool nRFxQSPIWaitReady(NrfSpiDev_t * const pDev, uint32_t Timeout)
{
	uint32_t val = 0;

	do {
		if (pDev->pQSpiReg->STATUS & QSPI_STATUS_READY_Msk)
		{
			return true;
		}
	} while (Timeout-- > 0);

	return false;
}

static bool nRFxQSPIWaitEventReady(NrfSpiDev_t * const pDev, uint32_t Timeout)
{
	uint32_t val = 0;

	do {
		if (pDev->pQSpiReg->EVENTS_READY)
		{
			pDev->pQSpiReg->EVENTS_READY = 0;

			return true;
		}
	} while (Timeout-- > 0);

	return false;
}

static void nRFxQSPIDisable(DevIntrf_t * const pDev)
{
	NrfSpiDev_t *dev = (NrfSpiDev_t *)pDev->pDevData;

	dev->pQSpiReg->TASKS_DEACTIVATE = 1;
	dev->pQSpiReg->ENABLE = (QSPI_ENABLE_ENABLE_Disabled << QSPI_ENABLE_ENABLE_Pos);
}

static void nRFxQSPIEnable(DevIntrf_t * const pDev)
{
	NrfSpiDev_t *dev = (NrfSpiDev_t *)pDev->pDevData;

	dev->pQSpiReg->ENABLE = (QSPI_ENABLE_ENABLE_Enabled << QSPI_ENABLE_ENABLE_Pos);
	dev->pQSpiReg->EVENTS_READY = 0;
	dev->pQSpiReg->TASKS_ACTIVATE = 1;
	nRFxQSPIWaitEventReady(dev, 10000);
}

static void nRFxQSPIPowerOff(DevIntrf_t * const pDev)
{
	NrfSpiDev_t *dev = (NrfSpiDev_t *)pDev->pDevData;

	// Undocumented Power down.  Nordic Bug with DMA causing high current consumption
	*(volatile uint32_t *)((uint32_t)dev->pQSpiReg + 0xFFC);
	*(volatile uint32_t *)((uint32_t)dev->pQSpiReg + 0xFFC) = 1;
	*(volatile uint32_t *)((uint32_t)dev->pQSpiReg + 0xFFC) = 0;

	for (int i = 0; i < dev->pSpiDev->Cfg.NbIOPins; i++)
	{
		if (dev->pSpiDev->Cfg.pIOPinMap[i].PortNo != -1 && dev->pSpiDev->Cfg.pIOPinMap[i].PinNo != -1)
		{
			IOPinDisable(dev->pSpiDev->Cfg.pIOPinMap[i].PortNo, dev->pSpiDev->Cfg.pIOPinMap[i].PinNo);
		}
	}
}

static uint32_t nRFxQSPIGetRate(DevIntrf_t * const pDev)
{
	int rate = 0;

	if (pDev && pDev->pDevData)
		rate = ((NrfSpiDev_t*)pDev->pDevData)->pSpiDev->Cfg.Rate;

	return rate;
}

static uint32_t nRFxQSPISetRate(DevIntrf_t * const pDev, uint32_t DataRate)
{
	uint32_t rate = 0;
	uint32_t sckfreq = (32000000 + (DataRate >> 1)) / DataRate - 1;
	NrfSpiDev_t *dev = (NrfSpiDev_t *)pDev->pDevData;

	dev->pQSpiReg->IFCONFIG1 &= ~QSPI_IFCONFIG1_SCKFREQ_Msk;
	dev->pQSpiReg->IFCONFIG1 |= (sckfreq & 0xFFFF) << QSPI_IFCONFIG1_SCKFREQ_Pos;

	rate = (32000000 + ((sckfreq + 1) >> 1))/ (sckfreq + 1);

	dev->pSpiDev->Cfg.Rate = rate;

	return rate;
}

// Initial receive
bool nRFxQSPIStartRx(DevIntrf_t * const pDev, uint32_t DevCs)
{
	NrfSpiDev_t *dev = (NrfSpiDev_t *)pDev->pDevData;

	if (dev->pSpiDev->Cfg.ChipSel == SPICSEL_MAN)
		return true;

	if (DevCs < 0 || DevCs >= dev->pSpiDev->Cfg.NbIOPins - QSPI_CS_IOPIN_IDX)
		return false;

	if (dev->pSpiDev->Cfg.ChipSel == SPICSEL_DRIVER)
	{
		dev->pSpiDev->CurDevCs = DevCs;

		IOPinClear(dev->pSpiDev->Cfg.pIOPinMap[DevCs + QSPI_CS_IOPIN_IDX].PortNo,
				   dev->pSpiDev->Cfg.pIOPinMap[DevCs + QSPI_CS_IOPIN_IDX].PinNo);
	}

	return true;
}

// Receive Data only, no Start/Stop condition
int nRFxQSPIRxData(DevIntrf_t * const pDev, uint8_t *pBuff, int BuffLen)
{
	NrfSpiDev_t *dev = (NrfSpiDev_t *)pDev-> pDevData;
    int cnt = 0;

	nRFxQSPIWaitReady(dev, 10000);

	dev->pQSpiReg->EVENTS_READY = 0;

	if (dev->Cmd != 0)
	{
		uint32_t cinstrconf = ((uint32_t)dev->Cmd << QSPI_CINSTRCONF_OPCODE_Pos) |
							  ((dev->ParamLen + BuffLen + 1) << QSPI_CINSTRCONF_LENGTH_Pos) |
							  QSPI_CINSTRCONF_LIO2_Msk | QSPI_CINSTRCONF_LIO3_Msk;

		dev->pQSpiReg->CINSTRCONF = cinstrconf;

		if (nRFxQSPIWaitEventReady(dev, 10000) == true)
		{
			uint32_t buf[2];
			buf[0] = dev->pQSpiReg->CINSTRDAT0;
			if (BuffLen > 4)
			{
				buf[1] = dev->pQSpiReg->CINSTRDAT1;
			}

			int l = min(BuffLen, 8);
			memcpy(pBuff, buf, BuffLen);
			cnt = l;
		}

		dev->Cmd = 0;
		dev->ParamLen = 0;
	}
	else
	{
		dev->pQSpiReg->READ.DST = (uint32_t)pBuff;
		dev->pQSpiReg->READ.CNT = BuffLen;
		dev->pQSpiReg->TASKS_READSTART = 1;

		cnt = BuffLen;
	}

    return cnt;
}

// Stop receive
void nRFxQSPIStopRx(DevIntrf_t * const pDev)
{
	NrfSpiDev_t *dev = (NrfSpiDev_t *)pDev-> pDevData;

	nRFxQSPIWaitEventReady(dev, 10000);

	nRFxQSPIWaitReady(dev, 10000);
	dev->pQSpiReg->EVENTS_READY = 0;

	if (dev->pSpiDev->Cfg.ChipSel == SPICSEL_DRIVER)
	{
		IOPinSet(dev->pSpiDev->Cfg.pIOPinMap[dev->pSpiDev->CurDevCs + QSPI_CS_IOPIN_IDX].PortNo,
				dev->pSpiDev->Cfg.pIOPinMap[dev->pSpiDev->CurDevCs + QSPI_CS_IOPIN_IDX].PinNo);
	}
}

// Initiate transmit
bool nRFxQSPIStartTx(DevIntrf_t * const pDev, uint32_t DevCs)
{
	NrfSpiDev_t *dev = (NrfSpiDev_t *)pDev-> pDevData;

	if (dev->pSpiDev->Cfg.ChipSel == SPICSEL_MAN)
		return true;

	if (DevCs < 0 || DevCs >= dev->pSpiDev->Cfg.NbIOPins - QSPI_CS_IOPIN_IDX)
		return false;

	if (dev->pSpiDev->Cfg.ChipSel == SPICSEL_DRIVER)
	{
		dev->pSpiDev->CurDevCs = DevCs;

		IOPinClear(dev->pSpiDev->Cfg.pIOPinMap[DevCs + QSPI_CS_IOPIN_IDX].PortNo,
				   dev->pSpiDev->Cfg.pIOPinMap[DevCs + QSPI_CS_IOPIN_IDX].PinNo);
	}

	return true;
}

// Transmit Data only, no Start/Stop condition
int nRFxQSPITxData(DevIntrf_t * const pDev, uint8_t *pData, int DataLen)
{
	NrfSpiDev_t *dev = (NrfSpiDev_t *)pDev-> pDevData;
	int cnt = 0;

	nRFxQSPIWaitReady(dev, 10000);
	dev->pQSpiReg->EVENTS_READY = 0;

	if (dev->Cmd)
	{
		if (DataLen > 0)
		{
			volatile uint32_t *p = &dev->pQSpiReg->CINSTRDAT0;

			memcpy(&((uint8_t*)dev->Param)[dev->ParamLen], pData, DataLen);

			dev->ParamLen += DataLen;

			dev->pQSpiReg->CINSTRDAT0 = dev->Param[0];
			if (dev->ParamLen > 3)
			{
				dev->pQSpiReg->CINSTRDAT1 = dev->Param[1];
			}

			uint32_t cinstrconf = ((uint32_t)dev->Cmd << QSPI_CINSTRCONF_OPCODE_Pos) |
								  ((dev->ParamLen + 1) << QSPI_CINSTRCONF_LENGTH_Pos) |
								  QSPI_CINSTRCONF_LIO2_Msk | QSPI_CINSTRCONF_LIO3_Msk;

			dev->pQSpiReg->CINSTRCONF = cinstrconf;
			dev->Cmd = 0;
			dev->ParamLen = 0;
		}

		cnt = DataLen;
	}
	else
	{
		dev->pQSpiReg->WRITE.SRC = (uint32_t)pData;
		dev->pQSpiReg->WRITE.CNT = DataLen;
		dev->pQSpiReg->TASKS_WRITESTART = 1;

		cnt = DataLen;
	}

	return cnt;
}

// Stop transmit
void nRFxQSPIStopTx(DevIntrf_t * const pDev)
{
	NrfSpiDev_t *dev = (NrfSpiDev_t *)pDev-> pDevData;

	nRFxQSPIWaitEventReady(dev, 10000);
	nRFxQSPIWaitReady(dev, 10000);
	dev->pQSpiReg->EVENTS_READY = 0;

	if (dev->pSpiDev->Cfg.ChipSel == SPICSEL_DRIVER)
	{
		IOPinSet(dev->pSpiDev->Cfg.pIOPinMap[dev->pSpiDev->CurDevCs + QSPI_CS_IOPIN_IDX].PortNo,
				dev->pSpiDev->Cfg.pIOPinMap[dev->pSpiDev->CurDevCs + QSPI_CS_IOPIN_IDX].PinNo);
	}
}


bool nRFxQSPISendCmd(DevIntrf_t * const pDev, uint8_t Cmd, uint32_t Addr, uint8_t AddrLen, uint32_t DataLen, uint8_t DummyCycle)
{
	NrfSpiDev_t *dev = (NrfSpiDev_t *)pDev-> pDevData;
	uint32_t ifcfg0 = dev->pQSpiReg->IFCONFIG0;

	if (AddrLen > 3)
	{
		// Set 4 byte address mode
		ifcfg0 = (ifcfg0 & ~QSPI_IFCONFIG0_ADDRMODE_Msk) | (1 << QSPI_IFCONFIG0_ADDRMODE_Pos);
	}

	dev->Cmd = 0;

	switch (Cmd)
	{
		case FLASH_CMD_DREAD:
			ifcfg0 = (ifcfg0 & ~QSPI_IFCONFIG0_READOC_Msk) | (1 << QSPI_IFCONFIG0_READOC_Pos);
			break;
		case FLASH_CMD_QREAD:
			ifcfg0 = (ifcfg0 & ~QSPI_IFCONFIG0_READOC_Msk) | (3 << QSPI_IFCONFIG0_READOC_Pos);
			break;
		case FLASH_CMD_2READ:
			ifcfg0 = (ifcfg0 & ~QSPI_IFCONFIG0_READOC_Msk) | (2 << QSPI_IFCONFIG0_READOC_Pos);
			break;
		case FLASH_CMD_QWRITE:
			ifcfg0 = (ifcfg0 & ~QSPI_IFCONFIG0_WRITEOC_Msk) | (2 << QSPI_IFCONFIG0_READOC_Pos);
			break;
		case FLASH_CMD_4READ:
			ifcfg0 = (ifcfg0 & ~QSPI_IFCONFIG0_READOC_Msk) | (4 << QSPI_IFCONFIG0_READOC_Pos);
			break;
		case FLASH_CMD_4WRITE:
		case FLASH_CMD_E4WRITE:
			ifcfg0 = (ifcfg0 & ~QSPI_IFCONFIG0_WRITEOC_Msk) | (3 << QSPI_IFCONFIG0_WRITEOC_Pos);
			break;
		case FLASH_CMD_DWRITE:
			ifcfg0 = (ifcfg0 & ~QSPI_IFCONFIG0_WRITEOC_Msk) | (1 << QSPI_IFCONFIG0_WRITEOC_Pos);
			break;
		default: // Custom cmd
		{
			// Reset ADDRMODE to 3 byte for other commands
			ifcfg0 &= ~QSPI_IFCONFIG0_ADDRMODE_Msk;
			dev->Cmd = Cmd;
			dev->ParamLen = 0;

			if (AddrLen > 0)
			{
				dev->ParamLen += AddrLen;
				dev->Param[0] = EndianCvt32(Addr);
				if (AddrLen < 4)
				{
					dev->Param[0] >>= 8;
				}
			}

			if (DataLen <= 0)
			{
				dev->pQSpiReg->CINSTRDAT0 = dev->Param[0];
				uint32_t cinstrconf = ((uint32_t)Cmd << QSPI_CINSTRCONF_OPCODE_Pos) |
									  ((AddrLen + 1) << QSPI_CINSTRCONF_LENGTH_Pos) |
									  QSPI_CINSTRCONF_LIO2_Msk | QSPI_CINSTRCONF_LIO3_Msk;

				dev->pQSpiReg->CINSTRCONF = cinstrconf;
				dev->ParamLen = 0;
			}
			return true;
		}
	}

	dev->pQSpiReg->IFCONFIG0 = ifcfg0;
	dev->pQSpiReg->READ.SRC = Addr;
	dev->pQSpiReg->WRITE.DST = Addr;

	return true;
}


bool nRFxQSPIInit(SPIDev_t * const pDev)
{
	NRF_QSPI_Type *reg = reg = s_nRFxSPIDev[pDev->Cfg.DevNo].pQSpiReg;

	// Force power on in case it was powered off previously
	*(volatile uint32_t *)((uint32_t)reg + 0xFFC);
	*(volatile uint32_t *)((uint32_t)reg + 0xFFC) = 1;

	// Configure I/O pins
	IOPinCfg(pDev->Cfg.pIOPinMap, pDev->Cfg.NbIOPins);

	for (int i = QSPI_CS_IOPIN_IDX; i < pDev->Cfg.NbIOPins; i++)
	{
		IOPinSet(pDev->Cfg.pIOPinMap[i].PortNo, pDev->Cfg.pIOPinMap[i].PinNo);
	}

	uint32_t ifconfig1 = reg->IFCONFIG1 & ~QSPI_IFCONFIG1_SPIMODE_Msk;

	if (pDev->Cfg.BitOrder == SPIDATABIT_LSB)
	{
	}

	if (pDev->Cfg.ClkPol == SPICLKPOL_LOW && pDev->Cfg.DataPhase == SPIDATAPHASE_SECOND_CLK)
	{
		ifconfig1 |= QSPI_IFCONFIG1_SPIMODE_Msk;
	}

	reg->PSEL.SCK = (pDev->Cfg.pIOPinMap[QSPI_SCK_IOPIN_IDX].PinNo & 0x1f) | (pDev->Cfg.pIOPinMap[QSPI_SCK_IOPIN_IDX].PortNo << 5);
	reg->PSEL.IO0 = (pDev->Cfg.pIOPinMap[QSPI_D0_IOPIN_IDX].PinNo & 0x1f) | (pDev->Cfg.pIOPinMap[QSPI_D0_IOPIN_IDX].PortNo << 5);
	reg->PSEL.IO1 = (pDev->Cfg.pIOPinMap[QSPI_D1_IOPIN_IDX].PinNo & 0x1f) | (pDev->Cfg.pIOPinMap[QSPI_D1_IOPIN_IDX].PortNo << 5);
	reg->PSEL.IO2 = (pDev->Cfg.pIOPinMap[QSPI_D2_IOPIN_IDX].PinNo & 0x1f) | (pDev->Cfg.pIOPinMap[QSPI_D2_IOPIN_IDX].PortNo << 5);
	reg->PSEL.IO3 = (pDev->Cfg.pIOPinMap[QSPI_D3_IOPIN_IDX].PinNo & 0x1f) | (pDev->Cfg.pIOPinMap[QSPI_D3_IOPIN_IDX].PortNo << 5);

	if (pDev->Cfg.ChipSel == SPICSEL_AUTO)
	{
		if (pDev->Cfg.NbIOPins == 6)
		{
			reg->PSEL.CSN = (pDev->Cfg.pIOPinMap[QSPI_CS_IOPIN_IDX].PinNo & 0x1f) | (pDev->Cfg.pIOPinMap[QSPI_CS_IOPIN_IDX].PortNo << 5);
		}
		else
		{
			pDev->Cfg.ChipSel = SPICSEL_DRIVER;
		}
	}

	reg->IFCONFIG0 &= ~QSPI_IFCONFIG0_ADDRMODE_Msk;

	nRFxQSPISetRate(&pDev->DevIntrf, pDev->Cfg.Rate);

	switch (pDev->Cfg.Phy)
	{
		case SPIPHY_DUAL:
			break;
		case SPIPHY_QUAD_DDR:
			// nRF does not support dual data rate
			// revert to SDR only
			pDev->Cfg.Phy = SPIPHY_QUAD_SDR;
		case SPIPHY_QUAD_SDR:
			break;
	}

	pDev->DevIntrf.Type = DEVINTRF_TYPE_QSPI;
	pDev->DevIntrf.Disable = nRFxQSPIDisable;
	pDev->DevIntrf.Enable = nRFxQSPIEnable;
	pDev->DevIntrf.GetRate = nRFxQSPIGetRate;
	pDev->DevIntrf.SetRate = nRFxQSPISetRate;
	pDev->DevIntrf.StartRx = nRFxQSPIStartRx;
	pDev->DevIntrf.RxData = nRFxQSPIRxData;
	pDev->DevIntrf.StopRx = nRFxQSPIStopRx;
	pDev->DevIntrf.StartTx = nRFxQSPIStartTx;
	pDev->DevIntrf.TxData = nRFxQSPITxData;
	pDev->DevIntrf.StopTx = nRFxQSPIStopTx;
	pDev->DevIntrf.IntPrio = pDev->Cfg.IntPrio;
	pDev->DevIntrf.EvtCB = pDev->Cfg.EvtCB;
	pDev->DevIntrf.MaxRetry = pDev->Cfg.MaxRetry;
	pDev->DevIntrf.bDma = pDev->Cfg.bDmaEn;
	pDev->DevIntrf.PowerOff = nRFxQSPIPowerOff;
	pDev->DevIntrf.EnCnt = 1;

	atomic_flag_clear(&pDev->DevIntrf.bBusy);

#if 0
	reg->EVENTS_READY = 0;
	reg->ENABLE = 1;
	reg->TASKS_ACTIVATE = 1;
#else
	nRFxQSPIEnable(&pDev->DevIntrf);
#endif
	return true;
}

extern "C" void QSPI_IRQHandler()
{

}

SPIPHY SPISetPhy(SPIDev_t * const pDev, SPIPHY Phy)
{
	pDev->Cfg.Phy = Phy;

	return pDev->Cfg.Phy;
}

/**
 * @brief	Set Quad SPI Flash size
 */
void QuadSPISetMemSize(SPIDEV * const pDev, uint32_t Size)
{
	if (pDev->Cfg.DevNo == (NRFX_SPI_MAXDEV - 1))
	{

	}
}

bool QuadSPISendCmd(SPIDEV * const pDev, uint8_t Cmd, uint32_t Addr, uint8_t AddrLen, uint32_t DataLen, uint8_t DummyCycle)
{
	if (pDev->Cfg.DevNo == NRFX_SPI_MAXDEV - 1)
	{
		return nRFxQSPISendCmd(&pDev->DevIntrf, Cmd, Addr, AddrLen, DataLen, DummyCycle);
	}

	return false;
}


#endif
