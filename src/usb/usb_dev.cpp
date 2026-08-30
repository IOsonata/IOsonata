/**-------------------------------------------------------------------------
@file	usb_dev_native.cpp

@brief	Native IOsonata USB device runtime.

Owns USB device identity, hardware start/stop sequencing and the native
UsbdCore lifecycle. This file is the replacement for usb_dev.cpp when the
native device stack is linked.

@author	Hoang Nguyen Hoan
@date	Aug. 29, 2026

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

#include "usb/usb_dev.h"
#include "usb/usbd_core.h"
#include "usb/usbd_ctrlr.h"
#include "usbd_cdc_priv.h"

#define USBDEV_FUNC_MAXCNT			USBD_CDC_FUNC_MAXCNT

typedef struct __Usb_Dev_Function {
	void (*Pump)(void *pCtx);
	void *pCtx;
} UsbDevFunc_t;

static UsbDevCfg_t s_UsbDevCfg;
static char s_UsbDevSerial[USBDEV_SERIAL_MAXLEN];
static UsbDevFunc_t s_UsbDevFunc[USBDEV_FUNC_MAXCNT];
static int s_UsbDevFuncCnt;
static bool s_UsbDevInitialized;
static bool s_UsbDevStarted;
static bool s_UsbDevAttachPending;

static int UsbDevMaxCdcCount(const UsbdCaps_t *pCaps)
{
	if (pCaps == nullptr || pCaps->EpInCnt < 3U || pCaps->EpOutCnt < 3U)
	{
		return 0;
	}

	// The native CDC topology uses endpoint 1 for notification and endpoint 2
	// for bulk on the first function, then advances both endpoint numbers by
	// two. Endpoint counts include EP0, so the smaller direction determines
	// how many complete CDC functions fit on this controller instance.
	const int maxIn = ((int)pCaps->EpInCnt - 1) / 2;
	const int maxOut = ((int)pCaps->EpOutCnt - 1) / 2;
	const int maxCdc = maxIn < maxOut ? maxIn : maxOut;

	return maxCdc < USBD_CDC_FUNC_MAXCNT ? maxCdc : USBD_CDC_FUNC_MAXCNT;
}

static const uint8_t *UsbDevDescHandler(uint8_t DescType,
									uint8_t DescIndex,
									uint16_t LangId,
									UsbdSpeed_t,
									uint16_t *pLength,
									void *pContext)
{
	const UsbdSpeed_t speed = UsbdCtrlrHighSpeed() ?
		USBD_SPEED_HIGH : USBD_SPEED_FULL;

	return UsbdCdcDescHandler(DescType, DescIndex, LangId, speed,
							 pLength, pContext);
}

static void UsbDevEvtHandler(UsbdEvt_t Evt)
{
	switch (Evt)
	{
		case USBD_EVT_ATTACHED:
			if (!s_UsbDevStarted)
			{
				s_UsbDevAttachPending = true;
			}
			break;

		case USBD_EVT_DETACHED:
			if (s_UsbDevStarted)
			{
				UsbDevDisable();
			}
			else
			{
				s_UsbDevAttachPending = false;
			}
			break;
	}
}

bool UsbDevInit(const UsbDevCfg_t *pCfg)
{
	if (pCfg == nullptr || pCfg->Vid == 0 || pCfg->Pid == 0)
	{
		return false;
	}

	const UsbdCaps_t *pCaps = UsbdGetCaps();
	const int maxCdc = UsbDevMaxCdcCount(pCaps);
	if (maxCdc < 1 || pCaps->Ep0Mps == 0U)
	{
		return false;
	}

	s_UsbDevInitialized = false;
	s_UsbDevStarted = false;
	s_UsbDevAttachPending = false;
	s_UsbDevFuncCnt = 0;
	memset(s_UsbDevFunc, 0, sizeof(s_UsbDevFunc));
	memcpy(&s_UsbDevCfg, pCfg, sizeof(s_UsbDevCfg));

	if (s_UsbDevCfg.NbCdc < 1)
	{
		s_UsbDevCfg.NbCdc = 1;
	}

	if (s_UsbDevCfg.NbCdc > maxCdc)
	{
		s_UsbDevCfg.NbCdc = maxCdc;
	}

	if (s_UsbDevCfg.MaxPower == 0)
	{
		s_UsbDevCfg.MaxPower = 100;
	}

	if (s_UsbDevCfg.pSerial != nullptr)
	{
		strncpy(s_UsbDevSerial, s_UsbDevCfg.pSerial,
				sizeof(s_UsbDevSerial) - 1U);
		s_UsbDevSerial[sizeof(s_UsbDevSerial) - 1U] = '\0';
	}
	else
	{
		UsbdGetSerial(s_UsbDevSerial, sizeof(s_UsbDevSerial));
	}
	s_UsbDevCfg.pSerial = s_UsbDevSerial;

	UsbdCfg_t hwCfg = {};
	hwCfg.IntPrio = s_UsbDevCfg.IntPrio;
	hwCfg.bLowPowerSuspend = s_UsbDevCfg.bLowPowerSuspend;
	hwCfg.EvtHandler = UsbDevEvtHandler;

	if (!UsbdInit(&hwCfg))
	{
		return false;
	}

	UsbdCoreCfg_t coreCfg = {};
	coreCfg.DescHandler = UsbDevDescHandler;
	coreCfg.pDescContext = nullptr;
	coreCfg.Speed = pCaps->MaxSpeed;
	coreCfg.Ep0Mps = pCaps->Ep0Mps;

	if (!UsbdCoreInit(&coreCfg))
	{
		return false;
	}

	s_UsbDevInitialized = true;
	return true;
}

bool UsbDevRegisterFunc(void (*Pump)(void *pCtx), void *pCtx)
{
	if (Pump == nullptr || s_UsbDevFuncCnt >= USBDEV_FUNC_MAXCNT)
	{
		return false;
	}

	s_UsbDevFunc[s_UsbDevFuncCnt].Pump = Pump;
	s_UsbDevFunc[s_UsbDevFuncCnt].pCtx = pCtx;
	s_UsbDevFuncCnt++;

	return true;
}

bool UsbDevEnable(void)
{
	if (!s_UsbDevInitialized)
	{
		return false;
	}

	if (s_UsbDevStarted)
	{
		return true;
	}

	if (!UsbdStart())
	{
		return false;
	}

	if (!UsbdCtrlrStart())
	{
		UsbdStop();
		return false;
	}

	UsbdCoreStart();
	s_UsbDevStarted = true;
	s_UsbDevAttachPending = false;

	return true;
}

void UsbDevDisable(void)
{
	if (!s_UsbDevStarted)
	{
		return;
	}

	UsbdCoreStop();
	UsbdCtrlrStop();
	UsbdStop();
	s_UsbDevStarted = false;
	s_UsbDevAttachPending = false;
}

void UsbDevProcess(void)
{
	if (!s_UsbDevInitialized)
	{
		return;
	}

	UsbdProcess();

	if (!s_UsbDevStarted)
	{
		if (s_UsbDevAttachPending)
		{
			s_UsbDevAttachPending = false;
			(void)UsbDevEnable();
		}
		return;
	}

	for (int i = 0; i < s_UsbDevFuncCnt; i++)
	{
		s_UsbDevFunc[i].Pump(s_UsbDevFunc[i].pCtx);
	}
}

bool UsbDevMounted(void)
{
	return s_UsbDevStarted && UsbdCoreConfigured();
}

bool UsbDevSuspended(void)
{
	return s_UsbDevStarted && UsbdCoreSuspended();
}

const UsbDevCfg_t *UsbDevGetCfg(void)
{
	return s_UsbDevInitialized ? &s_UsbDevCfg : nullptr;
}

const char *UsbDevGetSerial(void)
{
	return s_UsbDevSerial;
}
