/**-------------------------------------------------------------------------
@file	eink_inrf.cpp

@brief	Implementation of E-Ink display serial interface

This driver operates in BSI low mode.  BSI can be hardware pulled down or
connected MCU. If connected to MCU BSI will be force to low by this driver.
This mode allows to support both SPI interface or bit banging

@author	Hoang Nguyen Hoan
@date	Apr. 5, 2020

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
#include "idelay.h"
#include "iopinctrl.h"
#include "display/eink_intrf.h"

static bool EInkIntrfWaitBusy(EInkIntrf_t * const pDev, uint32_t Timeout = 0)
{
	while ((--Timeout > 0) &&
		   (IOPinRead(pDev->pIOPinMap[EIINTRF_BUSY_PIN_IDX].PortNo, pDev->pIOPinMap[EIINTRF_BUSY_PIN_IDX].PinNo) == 0));

	return Timeout > 0;
}

static void EInkIntrfDisable(DevIntrf_t * const pDev)
{

}

static void EInkIntrfEnable(DevIntrf_t * const pDev)
{

}

static uint32_t EInkIntrfGetRate(DevIntrf_t * const pDev)
{
	return 1;
}

static uint32_t EInkIntrfSetRate(DevIntrf_t * const pDev, uint32_t Rate)
{
	return 1;
}

static bool EInkIntrfStartRx(DevIntrf_t * const pDev, uint32_t DevAddr)
{
	EInkIntrf_t *dev = (EInkIntrf_t*)pDev->pDevData;

	EInkIntrfSetDataMode(dev, true);

	if (dev->Type == EIINTRF_TYPE_BITBANG)
	{
		IOPinClear(dev->pIOPinMap[EIINTRF_CS_PIN_IDX].PortNo, dev->pIOPinMap[EIINTRF_CS_PIN_IDX].PinNo);
	    IOPinSetDir(dev->pIOPinMap[EIINTRF_SDA_PIN_IDX].PortNo, dev->pIOPinMap[EIINTRF_SDA_PIN_IDX].PinNo, IOPINDIR_INPUT);
	}
	else
	{
		SPIStartRx(dev->pSpiDev, dev->SpiCsIdx);
	}

	return true;
}

static int EInkIntrfRxData(DevIntrf_t * const pDev, uint8_t *pBuff, int BuffLen)
{
	EInkIntrf_t *dev = (EInkIntrf_t*)pDev->pDevData;
	int cnt = 0;

	if (dev->Type == EIINTRF_TYPE_SPI)
	{
		return SPIRxData(dev->pSpiDev, pBuff, BuffLen);
	}

	while (BuffLen > 0)
	{
		*pBuff = 0;
		for (int i = 0; i < 8; i++)
		{
			*pBuff <<=1;
			IOPinSet(dev->pIOPinMap[EIINTRF_SCL_PIN_IDX].PortNo, dev->pIOPinMap[EIINTRF_SCL_PIN_IDX].PinNo);
			IOPinClear(dev->pIOPinMap[EIINTRF_SCL_PIN_IDX].PortNo, dev->pIOPinMap[EIINTRF_SCL_PIN_IDX].PinNo);
			*pBuff |= IOPinRead(dev->pIOPinMap[EIINTRF_SDA_PIN_IDX].PortNo, dev->pIOPinMap[EIINTRF_SDA_PIN_IDX].PinNo);
		}
		BuffLen--;
		pBuff++;
	}

	return cnt;
}

static void EInkIntrfStopRx(DevIntrf_t * const pDev)
{
	EInkIntrf_t *dev = (EInkIntrf_t*)pDev->pDevData;

	if (dev->Type == EIINTRF_TYPE_BITBANG)
	{
	    IOPinSetDir(dev->pIOPinMap[EIINTRF_SDA_PIN_IDX].PortNo, dev->pIOPinMap[EIINTRF_SDA_PIN_IDX].PinNo, IOPINDIR_OUTPUT);
		IOPinSet(dev->pIOPinMap[EIINTRF_CS_PIN_IDX].PortNo, dev->pIOPinMap[EIINTRF_CS_PIN_IDX].PinNo);
	}
	else
	{
		SPIStopRx(dev->pSpiDev);
	}
}

static bool EInkIntrfStartTx(DevIntrf_t * const pDev, uint32_t DevAddr)
{
	EInkIntrf_t *dev = (EInkIntrf_t*)pDev->pDevData;

	if (dev->Type == EIINTRF_TYPE_BITBANG)
	{
		IOPinClear(dev->pIOPinMap[EIINTRF_CS_PIN_IDX].PortNo, dev->pIOPinMap[EIINTRF_CS_PIN_IDX].PinNo);
	}
	else
	{
		SPIStartTx(dev->pSpiDev, dev->SpiCsIdx);
	}

	return true;
}

static int EInkIntrfTxData(DevIntrf_t * const pDev, uint8_t *pData, int DataLen)
{
	EInkIntrf_t *dev = (EInkIntrf_t*)pDev->pDevData;
	int cnt = 0;

	if (dev->Type == EIINTRF_TYPE_SPI)
	{
		return SPITxData(dev->pSpiDev, pData, DataLen);
	}

	while (DataLen > 0)
	{
		uint8_t bit = 0x80;

		while (bit != 0)
		{
			if (bit & *pData)
			{
				IOPinSet(dev->pIOPinMap[EIINTRF_SDA_PIN_IDX].PortNo, dev->pIOPinMap[EIINTRF_SDA_PIN_IDX].PinNo);
			}
			else
			{
				IOPinClear(dev->pIOPinMap[EIINTRF_SDA_PIN_IDX].PortNo, dev->pIOPinMap[EIINTRF_SDA_PIN_IDX].PinNo);
			}
			IOPinSet(dev->pIOPinMap[EIINTRF_SCL_PIN_IDX].PortNo, dev->pIOPinMap[EIINTRF_SCL_PIN_IDX].PinNo);
			//usDelay(1);
			IOPinClear(dev->pIOPinMap[EIINTRF_SCL_PIN_IDX].PortNo, dev->pIOPinMap[EIINTRF_SCL_PIN_IDX].PinNo);

			bit >>= 1;
		}

		DataLen--;
		pData++;
		cnt++;
	}

	return cnt;
}

static void EInkIntrfStopTx(DevIntrf_t * const pDev)
{
	EInkIntrf_t *dev = (EInkIntrf_t*)pDev->pDevData;

	if (dev->Type == EIINTRF_TYPE_BITBANG)
	{
		IOPinSet(dev->pIOPinMap[EIINTRF_CS_PIN_IDX].PortNo, dev->pIOPinMap[EIINTRF_CS_PIN_IDX].PinNo);
	}
	else
	{
		SPIStopTx(dev->pSpiDev);
	}
}

static void EInkIntrfReset(DevIntrf_t * const pDev)
{
	EInkIntrf_t *dev = (EInkIntrf_t*)pDev->pDevData;

	IOPinClear(dev->pIOPinMap[EIINTRF_RST_PIN_IDX].PortNo, dev->pIOPinMap[EIINTRF_RST_PIN_IDX].PinNo);
	usDelay(500);
	IOPinSet(dev->pIOPinMap[EIINTRF_RST_PIN_IDX].PortNo, dev->pIOPinMap[EIINTRF_RST_PIN_IDX].PinNo);
}

static void EInkIntrfPowerOff(DevIntrf_t * const pDev)
{

}

void EInkIntrfSetDataMode(EInkIntrf_t *pDev, bool bDataMode)
{
	if (bDataMode)
	{
		IOPinSet(pDev->pIOPinMap[EIINTRF_DC_PIN_IDX].PortNo, pDev->pIOPinMap[EIINTRF_DC_PIN_IDX].PinNo);
	}
	else
	{
		IOPinClear(pDev->pIOPinMap[EIINTRF_DC_PIN_IDX].PortNo, pDev->pIOPinMap[EIINTRF_DC_PIN_IDX].PinNo);
	}
}

int EInkIntrfWrite(EInkIntrf_t * const pDev, uint8_t *pCmd, int CmdLen, uint8_t *pData, int DataLen)
{
    int count = 0;

    if ((CmdLen + DataLen) <= 0)
        return 0;

    EInkIntrfSetDataMode(pDev, false);
    if (DeviceIntrfStartTx(&pDev->DevIntrf, pDev->SpiCsIdx))
    {
    	if (CmdLen > 0)
    	{
    		count += EInkIntrfTxData(&pDev->DevIntrf, pCmd, CmdLen);
    	}

    	if (DataLen > 0)
    	{
    		EInkIntrfSetDataMode(pDev, true);
    		count += EInkIntrfTxData(&pDev->DevIntrf, pData, DataLen);
    	}
		DeviceIntrfStopTx(&pDev->DevIntrf);
    }

    return count;
}

bool EInkIntrfInit(EInkIntrf_t * const pDev, const EInkIntrfCfg_t *pCfgData)
{
	IOPinCfg(pCfgData->pIOPinMap, pCfgData->NbIOPins);
	IOPinSet(pDev->pIOPinMap[EIINTRF_RST_PIN_IDX].PortNo, pDev->pIOPinMap[EIINTRF_RST_PIN_IDX].PinNo);

	if (pCfgData->Type == EIINTRF_TYPE_SPI)
	{
		if (pCfgData->pSpiDev == NULL)
		{
			return false;
		}
		pDev->pSpiDev = pCfgData->pSpiDev;
		pDev->SpiCsIdx = pCfgData->SpiCsIdx;
	}
	else
	{
		if (pCfgData->NbIOPins <= EIINTRF_CS_PIN_IDX)
		{
			return false;
		}
		IOPinClear(pDev->pIOPinMap[EIINTRF_SCL_PIN_IDX].PortNo, pDev->pIOPinMap[EIINTRF_SCL_PIN_IDX].PinNo);
		IOPinSet(pDev->pIOPinMap[EIINTRF_SDA_PIN_IDX].PortNo, pDev->pIOPinMap[EIINTRF_SDA_PIN_IDX].PinNo);
		IOPinSet(pDev->pIOPinMap[EIINTRF_CS_PIN_IDX].PortNo, pDev->pIOPinMap[EIINTRF_CS_PIN_IDX].PinNo);
	}


	if (pCfgData->NbIOPins > EIINTRF_BSI_PIN_IDX)
	{
		IOPinClear(pDev->pIOPinMap[EIINTRF_BSI_PIN_IDX].PortNo, pDev->pIOPinMap[EIINTRF_BSI_PIN_IDX].PinNo);
	}

	pDev->Type = pCfgData->Type;
	pDev->pIOPinMap = pCfgData->pIOPinMap;
	pDev->NbIOPins = pCfgData->NbIOPins;
	pDev->DevIntrf.pDevData = (void*)pDev;

	pDev->DevIntrf.Type = DEVINTRF_TYPE_UNKOWN;
	pDev->DevIntrf.Disable = EInkIntrfDisable;
	pDev->DevIntrf.Enable = EInkIntrfEnable;
	pDev->DevIntrf.GetRate = EInkIntrfGetRate;
	pDev->DevIntrf.SetRate = EInkIntrfSetRate;
	pDev->DevIntrf.StartRx = EInkIntrfStartRx;
	pDev->DevIntrf.RxData = EInkIntrfRxData;
	pDev->DevIntrf.StopRx = EInkIntrfStopRx;
	pDev->DevIntrf.StartTx = EInkIntrfStartTx;
	pDev->DevIntrf.TxData = EInkIntrfTxData;
	pDev->DevIntrf.StopTx = EInkIntrfStopTx;
	pDev->DevIntrf.IntPrio = 6;
	pDev->DevIntrf.EvtCB = nullptr;
	pDev->DevIntrf.EnCnt = 1;
	pDev->DevIntrf.MaxRetry = 1;
	pDev->DevIntrf.bDma = false;
	pDev->DevIntrf.PowerOff = EInkIntrfPowerOff;
	pDev->DevIntrf.Reset = EInkIntrfReset;

	atomic_flag_clear(&pDev->DevIntrf.bBusy);

	return true;
}

bool EInkIntrf::Init(const EInkIntrfCfg_t &Cfg)
{
	return EInkIntrfInit(&vDevData, &Cfg);
}
