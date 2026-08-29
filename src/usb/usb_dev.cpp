/**-------------------------------------------------------------------------
@file	usb_dev.cpp

@brief	Generic USB device.

Sits between the application and the device stack. Owns the identity, the
start and stop order, and the pump. Knows nothing about which controller is
underneath : that is the port layer in usbd.h, and one file per MCU
family answers it.

@author	Hoang Nguyen Hoan
@date	Aug. 28, 2026

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

#include "tusb.h"
#include "device/dcd.h"

#include "usb/usb_dev.h"
#include "usb/usbd.h"

//
// The root hub port. Neither the nRF52 USBD nor the nRF54 USBHS has more than
// one, and no part in the tree does.
//
#define USBDEV_RHPORT				0

//
// How many function interfaces may register. Two CDC functions is the common
// case and this leaves room for a third without being generous with BSS.
//
#define USBDEV_FUNC_MAXCNT			4

typedef struct {
	void (*Pump)(void *pCtx);
	void *pCtx;
} UsbDevFunc_t;

//
// One USB device controller per MCU, so this state is file scope rather than
// behind a handle. Nothing here is reachable from an interrupt : the cable
// event handler below is called from UsbdProcess, which runs in the same
// context as the pump.
//
static UsbDevCfg_t s_UsbDevCfg;
static char s_UsbDevSerial[USBDEV_SERIAL_MAXLEN];
static UsbDevFunc_t s_UsbDevFunc[USBDEV_FUNC_MAXCNT];
static int s_UsbDevFuncCnt = 0;
static bool s_UsbDevInitialized = false;
static bool s_UsbDevStarted = false;
static bool s_UsbDevAttachPending = false;

/**
 * Cable events arrive here from UsbdProcess, already out of interrupt
 * context. Attaching is deferred to the next pump rather than done here so
 * that the start sequence, which waits on the part, never runs from inside
 * the port layer's own housekeeping.
 */
static void UsbDevEvtHandler(UsbdEvt_t Evt)
{
	switch (Evt)
	{
		case USBD_EVT_ATTACHED:
			if (s_UsbDevStarted)
			{
				dcd_connect(USBDEV_RHPORT);
			}
			else
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

	memcpy(&s_UsbDevCfg, pCfg, sizeof(UsbDevCfg_t));

	if (s_UsbDevCfg.NbCdc < 1)
	{
		s_UsbDevCfg.NbCdc = 1;
	}
	if (s_UsbDevCfg.NbCdc > CFG_TUD_CDC)
	{
		s_UsbDevCfg.NbCdc = CFG_TUD_CDC;
	}
	if (s_UsbDevCfg.MaxPower == 0)
	{
		s_UsbDevCfg.MaxPower = 100;
	}

	//
	// The serial string is resolved once, here, so the descriptor callback
	// stays a lookup. It runs in whatever context the host asks from and has
	// no business reading device registers.
	//
	if (s_UsbDevCfg.pSerial != nullptr)
	{
		strncpy(s_UsbDevSerial, s_UsbDevCfg.pSerial, sizeof(s_UsbDevSerial) - 1);
		s_UsbDevSerial[sizeof(s_UsbDevSerial) - 1] = '\0';
	}
	else
	{
		UsbdGetSerial(s_UsbDevSerial, sizeof(s_UsbDevSerial));
	}
	s_UsbDevCfg.pSerial = s_UsbDevSerial;

	UsbdCfg_t hwcfg;

	hwcfg.IntPrio = s_UsbDevCfg.IntPrio;
	hwcfg.bLowPowerSuspend = s_UsbDevCfg.bLowPowerSuspend;
	hwcfg.EvtHandler = UsbDevEvtHandler;

	if (UsbdInit(&hwcfg) == false)
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
	if (s_UsbDevInitialized == false)
	{
		return false;
	}

	if (s_UsbDevStarted)
	{
		return true;
	}

	//
	// No bus power is the ordinary state of a board on a battery, not a
	// failure. The attach event sets the pending flag and the next pass
	// tries again.
	//
	if (UsbdStart() == false)
	{
		return false;
	}

	tusb_rhport_init_t rhinit;

	memset(&rhinit, 0, sizeof(rhinit));
	rhinit.role = TUSB_ROLE_DEVICE;
	rhinit.speed = UsbdMaxSpeed() == USBD_SPEED_HIGH ?
					TUSB_SPEED_HIGH : TUSB_SPEED_FULL;

	//
	// tusb_rhport_init and not tud_init. tud_init reaches the device layer
	// without going through tusb.c, so the root hub port role stays invalid
	// and the shared interrupt handler dispatches to nothing.
	//
	if (tusb_rhport_init(USBDEV_RHPORT, &rhinit) == false)
	{
		UsbdStop();
		return false;
	}

	//
	// tud_rhport_init is a no-op when the stack is already initialised. That
	// is the normal reattach path, and UsbdStop disabled the controller IRQ,
	// so explicitly enable it again before reconnecting.
	//
	dcd_int_enable(USBDEV_RHPORT);

	s_UsbDevStarted = true;
	s_UsbDevAttachPending = false;

	//
	// The controller start sequence deliberately leaves the pull-up off until
	// the device stack is ready to answer the first bus reset.
	//
	dcd_connect(USBDEV_RHPORT);

	return true;
}

void UsbDevDisable(void)
{
	if (s_UsbDevStarted == false)
	{
		return;
	}

	//
	// Keep the TinyUSB software stack initialised. A reconnect begins with a
	// host bus reset, which resets its device/class state. Power down only the
	// controller and its clock here; this also avoids depending on a DCD
	// deinit hook that the nRF5x driver does not provide.
	//
	dcd_disconnect(USBDEV_RHPORT);
	UsbdStop();

	s_UsbDevStarted = false;
	s_UsbDevAttachPending = false;
}

void UsbDevProcess(void)
{
	if (s_UsbDevInitialized == false)
	{
		return;
	}

	UsbdProcess();

	if (s_UsbDevStarted == false)
	{
		if (s_UsbDevAttachPending)
		{
			s_UsbDevAttachPending = false;
			UsbDevEnable();
		}
		return;
	}

	//
	// Zero timeout. The pump does not block, whether it is called from a main
	// loop or from a thread that has other work.
	//
	tud_task_ext(0, false);

	for (int i = 0; i < s_UsbDevFuncCnt; i++)
	{
		s_UsbDevFunc[i].Pump(s_UsbDevFunc[i].pCtx);
	}
}

bool UsbDevMounted(void)
{
	return s_UsbDevStarted && tud_mounted();
}

bool UsbDevSuspended(void)
{
	return s_UsbDevStarted && tud_suspended();
}

const UsbDevCfg_t *UsbDevGetCfg(void)
{
	return s_UsbDevInitialized ? &s_UsbDevCfg : nullptr;
}

const char *UsbDevGetSerial(void)
{
	return s_UsbDevSerial;
}
