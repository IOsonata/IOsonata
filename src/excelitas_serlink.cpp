/**-------------------------------------------------------------------------
@file	excelitas_serlink.cpp

@brief	Implementation of Excelitas single wire interface

Implementation of both Serial In & Direct Link

@author	Hoang Nguyen Hoan
@date	Nov. 16, 2019

@license

MIT License

Copyright (c) 2019 I-SYST inc. All rights reserved.

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

#include "idelay.h"
#include "interrupt.h"

#include "excelitas_serlink.h"

bool DirectLinkStartRx(DEVINTRF * const pDevIntrf, int DevAddr)
{
	EXCELSERDEV *dev = (EXCELSERDEV*)pDevIntrf->pDevData;

	IOPinSet(dev->Pins[EXCELITAS_DL_PIN_IDX].PortNo, dev->Pins[EXCELITAS_DL_PIN_IDX].PinNo);
	IOPinSetDir(dev->Pins[EXCELITAS_DL_PIN_IDX].PortNo, dev->Pins[EXCELITAS_DL_PIN_IDX].PinNo, IOPINDIR_OUTPUT);
	usDelay(110);

	dev->NbBits = DevAddr;

	return true;
}

int DirectLinkRxData(DEVINTRF * const pDevIntrf, uint8_t *pBuff, int BuffLen)
{
	EXCELSERDEV *dev = (EXCELSERDEV*)pDevIntrf->pDevData;
	uint64_t val = 0;
	int cnt = 0;

	for (int i = 0; i < dev->NbBits; i++)
	{
		IOPinClear(dev->Pins[EXCELITAS_DL_PIN_IDX].PortNo, dev->Pins[EXCELITAS_DL_PIN_IDX].PinNo);
		IOPinSetDir(dev->Pins[EXCELITAS_DL_PIN_IDX].PortNo, dev->Pins[EXCELITAS_DL_PIN_IDX].PinNo, IOPINDIR_OUTPUT);
		IOPinSet(dev->Pins[EXCELITAS_DL_PIN_IDX].PortNo, dev->Pins[EXCELITAS_DL_PIN_IDX].PinNo);
		usDelay(1);
		IOPinSetDir(dev->Pins[EXCELITAS_DL_PIN_IDX].PortNo, dev->Pins[EXCELITAS_DL_PIN_IDX].PinNo, IOPINDIR_INPUT);
		uint32_t d = IOPinRead(dev->Pins[EXCELITAS_DL_PIN_IDX].PortNo, dev->Pins[EXCELITAS_DL_PIN_IDX].PinNo);
		usDelay(15);
		val <<= 1;
		val |= IOPinRead(dev->Pins[EXCELITAS_DL_PIN_IDX].PortNo, dev->Pins[EXCELITAS_DL_PIN_IDX].PinNo) & d;;
	}

	return cnt;
}

bool ExcelitasIntrfInit(EXCELSERDEV * const pDev, EXCELSER_CFG * const pCfg)
{
	if (pDev == NULL || pCfg == NULL)
	{
		return false;
	}

	memcpy(pDev->Pins, pCfg->Pins, EXCELITAS_PIN_MAX);

	pDev->DevIntrf.pDevData = pDev;
	pDev->DevIntrf.EvtCB = pCfg->EvtHandler;

	return true;
}

bool DirectLinkInit(DIRECTLINK_DEV *pDev, int PortNo, int PinNo, int PinOp)
{
	if (pDev == NULL)
	{
		return false;
	}

	pDev->Pin.PortNo = PortNo;
	pDev->Pin.PinNo = PinNo;
	pDev->Pin.PinOp = PinOp;
	pDev->Pin.PinDir = IOPINDIR_INPUT;
	pDev->Pin.Res = IOPINRES_NONE;
	pDev->Pin.Type = IOPINTYPE_NORMAL;

	IOPinCfg(&pDev->Pin, 1);

	IOPinClear(pDev->Pin.PortNo, pDev->Pin.PinNo);
	IOPinSetDir(pDev->Pin.PortNo, pDev->Pin.PinNo, IOPINDIR_OUTPUT);
	usDelay(2);
	IOPinSetDir(pDev->Pin.PortNo, pDev->Pin.PinNo, IOPINDIR_INPUT);

	return true;
}

int DirectLinkRead(DIRECTLINK_DEV *pDev, int NbBits, uint8_t *pBuf)
{
	uint64_t val = 0;

	if (pBuf == NULL || pDev == NULL)
	{
		return 0;
	}

	IOPinSet(pDev->Pin.PortNo, pDev->Pin.PinNo);
	IOPinSetDir(pDev->Pin.PortNo, pDev->Pin.PinNo, IOPINDIR_OUTPUT);
	usDelay(110);

	for (int i = 0; i < NbBits; i++)
	{
		IOPinClear(pDev->Pin.PortNo, pDev->Pin.PinNo);
		IOPinSetDir(pDev->Pin.PortNo, pDev->Pin.PinNo, IOPINDIR_OUTPUT);
		usDelay(1);
		IOPinSet(pDev->Pin.PortNo, pDev->Pin.PinNo);
		usDelay(1);
		IOPinSetDir(pDev->Pin.PortNo, pDev->Pin.PinNo, IOPINDIR_INPUT);
		uint32_t d = IOPinRead(pDev->Pin.PortNo, pDev->Pin.PinNo);
		usDelay(13);
		val <<= 1;
		val |= IOPinRead(pDev->Pin.PortNo, pDev->Pin.PinNo) & d;
	}

	IOPinClear(pDev->Pin.PortNo, pDev->Pin.PinNo);
	IOPinSetDir(pDev->Pin.PortNo, pDev->Pin.PinNo, IOPINDIR_OUTPUT);
	usDelay(2);
	IOPinSetDir(pDev->Pin.PortNo, pDev->Pin.PinNo, IOPINDIR_INPUT);

	memcpy(pBuf, &val, (NbBits + 7) >> 3);

	return (NbBits + 7) >> 3;
}

uint64_t DirectLinkRead2(DIRECTLINK_DEV *pDev, int NbBits)
{
	uint64_t val = 0;

	IOPinSet(pDev->Pin.PortNo, pDev->Pin.PinNo);
	IOPinSetDir(pDev->Pin.PortNo, pDev->Pin.PinNo, IOPINDIR_OUTPUT);
	usDelay(110);

	for (int i = 0; i < 15; i++)
	{
		IOPinClear(pDev->Pin.PortNo, pDev->Pin.PinNo);
		IOPinSetDir(pDev->Pin.PortNo, pDev->Pin.PinNo, IOPINDIR_OUTPUT);
		IOPinSet(pDev->Pin.PortNo, pDev->Pin.PinNo);
		usDelay(2);
		IOPinSetDir(pDev->Pin.PortNo, pDev->Pin.PinNo, IOPINDIR_INPUT);
		uint32_t d = IOPinRead(pDev->Pin.PortNo, pDev->Pin.PinNo);
		usDelay(10);
		val <<= 1ULL;
		val |= IOPinRead(pDev->Pin.PortNo, pDev->Pin.PinNo) & d;;
	}

	uint32_t cfg = 0;

	for (int i = 0; i < 25; i++)
	{
		IOPinClear(pDev->Pin.PortNo, pDev->Pin.PinNo);
		IOPinSetDir(pDev->Pin.PortNo, pDev->Pin.PinNo, IOPINDIR_OUTPUT);
		IOPinSet(pDev->Pin.PortNo, pDev->Pin.PinNo);
		usDelay(1);
		IOPinSetDir(pDev->Pin.PortNo, pDev->Pin.PinNo, IOPINDIR_INPUT);
		uint32_t d = IOPinRead(pDev->Pin.PortNo, pDev->Pin.PinNo);
		usDelay(15);
		cfg <<= 1;
		cfg |= IOPinRead(pDev->Pin.PortNo, pDev->Pin.PinNo) & d;;
	}

	IOPinClear(pDev->Pin.PortNo, pDev->Pin.PinNo);
	IOPinSetDir(pDev->Pin.PortNo, pDev->Pin.PinNo, IOPINDIR_OUTPUT);
	usDelay(2);
	IOPinSetDir(pDev->Pin.PortNo, pDev->Pin.PinNo, IOPINDIR_INPUT);

	return val;
}

bool SerialInInit(SERIALIN_DEV *pDev, int PortNo, int PinNo, int PinOp)
{
	if (pDev == NULL)
	{
		return false;
	}

	IOPinClear(pDev->Pin.PortNo, pDev->Pin.PinNo);

	pDev->Pin.PortNo = PortNo;
	pDev->Pin.PinNo = PinNo;
	pDev->Pin.PinOp = PinOp;
	pDev->Pin.PinDir = IOPINDIR_OUTPUT;
	pDev->Pin.Res = IOPINRES_NONE;
	pDev->Pin.Type = IOPINTYPE_NORMAL;

	IOPinCfg(&pDev->Pin, 1);

	//IOPinSetDir(pDev->Pin.PortNo, pDev->Pin.PinNo, IOPINDIR_OUTPUT);

	return true;
}

void SerialIn(SERIALIN_DEV *pDev, uint32_t Data, int NbBits)
{
	uint32_t mask = 1 << (NbBits - 1);

	uint32_t state = DisableInterrupt();

	while (mask != 0)
	{
		IOPinClear(pDev->Pin.PortNo, pDev->Pin.PinNo);
		usDelay(1);		// DO NOT remove.
		IOPinSet(pDev->Pin.PortNo, pDev->Pin.PinNo);
		usDelay(1);		// DO NOT remove.
		if ((Data & mask) == 0)
		{
			IOPinClear(pDev->Pin.PortNo, pDev->Pin.PinNo);
		}
		usDelay(100);	// DO NOT remove.
		mask >>= 1;
	}
	IOPinClear(pDev->Pin.PortNo, pDev->Pin.PinNo);

	EnableInterrupt(state);

	msDelay(1);
}
