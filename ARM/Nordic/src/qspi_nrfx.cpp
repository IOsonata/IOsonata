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
#ifdef NRF5340_XXAA_APPLICATION
#include "nrf_erratas.h"
#endif

#include "istddef.h"
#include "coredev/spi.h"
#include "iopinctrl.h"
#include "coredev/shared_intrf.h"
#include "storage/flash.h"
#include "convutil.h"

#include "spi_nrfx.h"

extern nRFSpiDev_t g_nRFxSPIDev[NRFX_SPI_MAXDEV];

#if defined(NRF52840_XXAA) || defined(NRF5340_XXAA_APPLICATION)

static bool nRFxQSPIWaitReady(nRFSpiDev_t * const pDev, uint32_t Timeout)
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

static bool nRFxQSPIWaitEventReady(nRFSpiDev_t * const pDev, uint32_t Timeout)
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
	nRFSpiDev_t *dev = (nRFSpiDev_t *)pDev->pDevData;

	dev->pQSpiReg->TASKS_DEACTIVATE = 1;
	dev->pQSpiReg->ENABLE = (QSPI_ENABLE_ENABLE_Disabled << QSPI_ENABLE_ENABLE_Pos);
}

static void nRFxQSPIEnable(DevIntrf_t * const pDev)
{
	nRFSpiDev_t *dev = (nRFSpiDev_t *)pDev->pDevData;

	dev->pQSpiReg->ENABLE = (QSPI_ENABLE_ENABLE_Enabled << QSPI_ENABLE_ENABLE_Pos);
	dev->pQSpiReg->EVENTS_READY = 0;
	dev->pQSpiReg->TASKS_ACTIVATE = 1;
	nRFxQSPIWaitEventReady(dev, 10000);
}

#ifdef NRF5340_XXAA_APPLICATION
// Only signals actually routed to the QSPI peripheral change ownership.
// Software-controlled chip selects remain GPIOs.
static void nRFxQSPIPinControl(const SPICfg_t &Cfg, bool Enable)
{
	int count = Cfg.ChipSel == SPICSEL_AUTO && Cfg.NbIOPins == 6 ? 6 : QSPI_CS_IOPIN_IDX;
	for (int i = 0; i < count; i++)
	{
		const IOPinCfg_t &pin = Cfg.pIOPinMap[i];
		if (pin.PortNo == 0 && pin.PinNo >= 13 && pin.PinNo <= 18)
		{
			uint32_t cnf = NRF_P0_S->PIN_CNF[pin.PinNo] & ~GPIO_PIN_CNF_MCUSEL_Msk;
			if (Enable)
			{
				cnf = (cnf & ~GPIO_PIN_CNF_DRIVE_Msk) |
					  (GPIO_PIN_CNF_DRIVE_H0H1 << GPIO_PIN_CNF_DRIVE_Pos) |
					  (GPIO_PIN_CNF_MCUSEL_Peripheral << GPIO_PIN_CNF_MCUSEL_Pos);
			}
			NRF_P0_S->PIN_CNF[pin.PinNo] = cnf;
		}
	}
}
#endif

static void nRFxQSPIPowerOff(DevIntrf_t * const pDev)
{
	nRFSpiDev_t *dev = (nRFSpiDev_t *)pDev->pDevData;

#ifdef NRF5340_XXAA_APPLICATION
	nRFxQSPIDisable(pDev);
	nRFxQSPIPinControl(dev->pSpiDev->Cfg, false);
#else
	// Undocumented Power down.  Nordic Bug with DMA causing high current consumption
	*(volatile uint32_t *)((uint32_t)dev->pQSpiReg + 0xFFC);
	*(volatile uint32_t *)((uint32_t)dev->pQSpiReg + 0xFFC) = 1;
	*(volatile uint32_t *)((uint32_t)dev->pQSpiReg + 0xFFC) = 0;
#endif

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
		rate = ((nRFSpiDev_t*)pDev->pDevData)->pSpiDev->Cfg.Rate;

	return rate;
}

static uint32_t nRFxQSPISetRate(DevIntrf_t * const pDev, uint32_t DataRate)
{
	nRFSpiDev_t *dev = (nRFSpiDev_t *)pDev->pDevData;
	uint32_t base = 32000000U;
	if (DataRate == 0)
	{
		return 0;
	}
#ifdef NRF5340_XXAA_APPLICATION
	uint32_t div = (NRF_CLOCK_S->HFCLK192MCTRL & CLOCK_HFCLK192MCTRL_HCLK192M_Msk) >>
				   CLOCK_HFCLK192MCTRL_HCLK192M_Pos;
	base = 96000000U >> div;
#endif
	uint32_t divider = 1U + (base - 1U) / DataRate;
	if (divider > 16)
	{
		divider = 16;
	}
	uint32_t rate = base / divider;
	uint32_t cfg = dev->pQSpiReg->IFCONFIG1 & ~QSPI_IFCONFIG1_SCKFREQ_Msk;
	dev->pQSpiReg->IFCONFIG1 = cfg | ((divider - 1U) << QSPI_IFCONFIG1_SCKFREQ_Pos);
	dev->pSpiDev->Cfg.Rate = rate;

	return rate;
}

// Initial receive
bool nRFxQSPIStartRx(DevIntrf_t * const pDev, uint32_t DevCs)
{
	nRFSpiDev_t *dev = (nRFSpiDev_t *)pDev->pDevData;

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
	nRFSpiDev_t *dev = (nRFSpiDev_t *)pDev->pDevData;
    int cnt = 0;

	nRFxQSPIWaitReady(dev, 10000);

	dev->pQSpiReg->EVENTS_READY = 0;

	if (dev->Cmd != 0)
	{
		// Nordic custom instructions carry at most eight data bytes.
		// Reject unsupported framing instead of truncating the result
		// while still reporting a successful transfer.
		if (BuffLen < 0 || dev->ParamLen > sizeof(dev->Param) ||
			(uint32_t)BuffLen > sizeof(dev->Param) - dev->ParamLen)
		{
			return 0;
		}
		uint32_t cinstrconf = ((uint32_t)dev->Cmd << QSPI_CINSTRCONF_OPCODE_Pos) |
							  ((dev->ParamLen + BuffLen + 1) << QSPI_CINSTRCONF_LENGTH_Pos) |
							  QSPI_CINSTRCONF_LIO2_Msk | QSPI_CINSTRCONF_LIO3_Msk;

		dev->pQSpiReg->CINSTRCONF = cinstrconf;

		if (nRFxQSPIWaitEventReady(dev, 10000) == true)
		{
			uint32_t buf[2] = { 0, 0 };
			buf[0] = dev->pQSpiReg->CINSTRDAT0;
			if (BuffLen > 4)
			{
				buf[1] = dev->pQSpiReg->CINSTRDAT1;
			}

			int l = min(BuffLen, 8);
			memcpy(pBuff, buf, (size_t)l);
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
	nRFSpiDev_t *dev = (nRFSpiDev_t *)pDev->pDevData;

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
	nRFSpiDev_t *dev = (nRFSpiDev_t *)pDev->pDevData;

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
int nRFxQSPITxData(DevIntrf_t * const pDev, const uint8_t *pData, int DataLen)
{
	nRFSpiDev_t *dev = (nRFSpiDev_t *)pDev->pDevData;
	int cnt = 0;

	nRFxQSPIWaitReady(dev, 10000);
	dev->pQSpiReg->EVENTS_READY = 0;

	if (dev->Cmd)
	{
		if (DataLen < 0 || dev->ParamLen > sizeof(dev->Param) ||
			(uint32_t)DataLen > sizeof(dev->Param) - dev->ParamLen)
		{
			return 0;
		}
		if (DataLen > 0)
		{
			memcpy(&((uint8_t*)dev->Param)[dev->ParamLen], pData,
				(size_t)DataLen);

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
	nRFSpiDev_t *dev = (nRFSpiDev_t *)pDev->pDevData;

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
	nRFSpiDev_t *dev = (nRFSpiDev_t *)pDev-> pDevData;
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
			ifcfg0 = (ifcfg0 & ~QSPI_IFCONFIG0_WRITEOC_Msk) | (2 << QSPI_IFCONFIG0_WRITEOC_Pos);
			break;
		case FLASH_CMD_4READ:
			ifcfg0 = (ifcfg0 & ~QSPI_IFCONFIG0_READOC_Msk) | (4 << QSPI_IFCONFIG0_READOC_Pos);
			break;
		case FLASH_CMD_4WRITE:
		case FLASH_CMD_E4WRITE:
			ifcfg0 = (ifcfg0 & ~QSPI_IFCONFIG0_WRITEOC_Msk) | (3 << QSPI_IFCONFIG0_WRITEOC_Pos);
			break;
		case FLASH_CMD_WRITE:
			ifcfg0 &= ~QSPI_IFCONFIG0_WRITEOC_Msk;
			break;
		case FLASH_CMD_DWRITE:
			ifcfg0 = (ifcfg0 & ~QSPI_IFCONFIG0_WRITEOC_Msk) | (1 << QSPI_IFCONFIG0_WRITEOC_Pos);
			break;
		case FLASH_CMD_EN4B:
			// Nordic handles 4 bytes address internally
			// ignore it
			return true;
			break;
		default: // Custom cmd
		{
			// Reset ADDRMODE to 3 byte for other commands
			ifcfg0 &= ~QSPI_IFCONFIG0_ADDRMODE_Msk;
			dev->Cmd = Cmd;
			dev->ParamLen = 0;
			dev->Param[0] = 0;
			dev->Param[1] = 0;

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
				// Already submitted. Do not replay the custom opcode when
				// a subsequent transfer starts.
				dev->Cmd = 0;
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

void *nRFxQSPIGetHandle(DevIntrf_t * const pDev)
{
	nRFSpiDev_t *dev = (nRFSpiDev_t *)pDev-> pDevData;

	return dev->pSpiDev;
}

bool nRFxQSPIInit(SPIDev_t * const pDev)
{
	NRF_QSPI_Type *reg = g_nRFxSPIDev[pDev->Cfg.DevNo].pQSpiReg;

	if (pDev->Cfg.pIOPinMap == NULL || pDev->Cfg.NbIOPins < QSPI_CS_IOPIN_IDX ||
		(pDev->Cfg.ChipSel != SPICSEL_MAN && pDev->Cfg.NbIOPins == QSPI_CS_IOPIN_IDX) ||
		pDev->Cfg.Rate == 0)
	{
		return false;
	}
#ifdef NRF5340_XXAA_APPLICATION
	// Anomaly 159 requires HCLK64M and undivided HFCLK192M for QSPI.
	if (nrf53_errata_159())
	{
		if ((NRF_CLOCK_S->HFCLKCTRL & CLOCK_HFCLKCTRL_HCLK_Msk) != CLOCK_HFCLKCTRL_HCLK_Div2)
		{
			return false;
		}
		NRF_CLOCK_S->HFCLK192MCTRL = CLOCK_HFCLK192MCTRL_HCLK192M_Div1;
	}
	// Anomaly 43: activate with pins disconnected before writing IFCONFIG1.
	if (nrf53_errata_43())
	{
		reg->PSEL.SCK = UINT32_MAX;
		reg->PSEL.CSN = UINT32_MAX;
		reg->PSEL.IO0 = UINT32_MAX;
		reg->PSEL.IO1 = UINT32_MAX;
		reg->PSEL.IO2 = UINT32_MAX;
		reg->PSEL.IO3 = UINT32_MAX;
		reg->ENABLE = QSPI_ENABLE_ENABLE_Enabled;
		reg->EVENTS_READY = 0;
		reg->TASKS_ACTIVATE = 1;
		if (!nRFxQSPIWaitEventReady(&g_nRFxSPIDev[pDev->Cfg.DevNo], 100000))
		{
			reg->ENABLE = QSPI_ENABLE_ENABLE_Disabled;
			return false;
		}
	}
#else
	// Force power on in case it was powered off previously
	*(volatile uint32_t *)((uint32_t)reg + 0xFFC);
	*(volatile uint32_t *)((uint32_t)reg + 0xFFC) = 1;
#endif

	// Configure I/O pins
	IOPinCfg(pDev->Cfg.pIOPinMap, pDev->Cfg.NbIOPins);

	for (int i = QSPI_CS_IOPIN_IDX; i < pDev->Cfg.NbIOPins; i++)
	{
		IOPinSet(pDev->Cfg.pIOPinMap[i].PortNo, pDev->Cfg.pIOPinMap[i].PinNo);
	}

#ifdef NRF5340_XXAA_APPLICATION
	nRFxQSPIPinControl(pDev->Cfg, true);
#endif

	reg->PSEL.SCK = nRFxSPIPinSelect(pDev->Cfg.pIOPinMap[QSPI_SCK_IOPIN_IDX]);
	reg->PSEL.IO0 = nRFxSPIPinSelect(pDev->Cfg.pIOPinMap[QSPI_D0_IOPIN_IDX]);
	reg->PSEL.IO1 = nRFxSPIPinSelect(pDev->Cfg.pIOPinMap[QSPI_D1_IOPIN_IDX]);
	reg->PSEL.IO2 = nRFxSPIPinSelect(pDev->Cfg.pIOPinMap[QSPI_D2_IOPIN_IDX]);
	reg->PSEL.IO3 = nRFxSPIPinSelect(pDev->Cfg.pIOPinMap[QSPI_D3_IOPIN_IDX]);

	if (pDev->Cfg.ChipSel == SPICSEL_AUTO)
	{
		if (pDev->Cfg.NbIOPins == 6)
		{
			reg->PSEL.CSN = nRFxSPIPinSelect(pDev->Cfg.pIOPinMap[QSPI_CS_IOPIN_IDX]);
		}
		else
		{
			pDev->Cfg.ChipSel = SPICSEL_DRIVER;
		}
	}

	reg->IFCONFIG0 &= ~QSPI_IFCONFIG0_ADDRMODE_Msk;

	nRFxQSPISetRate(&pDev->DevIntrf, pDev->Cfg.Rate);

#ifdef NRF5340_XXAA_APPLICATION
	if (nrf53_errata_121())
	{
		uint32_t timing = reg->IFCONFIG0 | (1UL << 16);
		if (reg->IFCONFIG1 & QSPI_IFCONFIG1_SCKFREQ_Msk)
		{
			timing &= ~(1UL << 17);
		}
		else
		{
			timing |= 1UL << 17;
		}
		reg->IFCONFIG0 = timing;
		reg->IFTIMING = 6U << QSPI_IFTIMING_RXDELAY_Pos;
	}
#endif

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

	pDev->DevIntrf.bTxReady = true;
	pDev->DevIntrf.bNoStop = false;
	pDev->DevIntrf.Type = DEVINTRF_TYPE_QSPI;
	pDev->QuadSetMemSize = QuadSPISetMemSize;
	pDev->QuadSendCmd = QuadSPISendCmd;
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
	pDev->DevIntrf.GetHandle = nRFxQSPIGetHandle;
	pDev->DevIntrf.EnCnt = 1;

	atomic_flag_clear(&pDev->DevIntrf.bBusy);

	reg->ENABLE = QSPI_ENABLE_ENABLE_Enabled;
	reg->EVENTS_READY = 0;
	reg->TASKS_ACTIVATE = 1;
	if (!nRFxQSPIWaitEventReady(&g_nRFxSPIDev[pDev->Cfg.DevNo], 100000))
	{
		nRFxQSPIPowerOff(&pDev->DevIntrf);
		return false;
	}
	return true;
}

extern "C" void QSPI_IRQHandler()
{
	NVIC_ClearPendingIRQ(QSPI_IRQn);
}

SPIPHY SPISetPhy(SPIDev_t * const pDev, SPIPHY Phy)
{
	pDev->Cfg.Phy = Phy;

	return pDev->Cfg.Phy;
}

// Legacy API: Nordic QSPI has no memory-size register to configure.
void QuadSPISetMemSize(SPIDEV * const pDev, uint32_t Size)
{
	(void)pDev;
	(void)Size;
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
