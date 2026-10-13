/**-------------------------------------------------------------------------
@file usb_lpc546xx.c
@brief LPC546xx USB0 full-speed lifecycle and interrupt handling.

@author Hoang Nguyen Hoan
@date Oct. 13, 2026
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
#include "coredev/interrupt.h"
#include "usb_control_lpc546xx.h"
#include "usb_ep0_lpc_fs.h"
#include "usb_hw_lpc546xx.h"
#include "usb_data_lpc_fs.h"
#include "usb/usb.h"
#include "iopinctrl.h"
#include "lpc_iap.h"

#define LPCUSB_DEVICE (1UL << 31)
#define LPCUSB_RESET (1UL << 26)
#define LPCUSB_CONNECT_CHANGE (1UL << 24)
#define LPCUSB_SUSPEND_CHANGE (1UL << 25)
#define LPCUSB_SETUP (1UL << 8)
#define LPCUSB_ENABLE (1UL << 7)
#define LPCUSB_CONNECT (1UL << 16)
#define LPCUSB_NAK_CONTROL (3UL << 14)

static bool Lpc546xxUsbControlReady(void);
static void Lpc546xxUsbControlFault(void);
static void Lpc546xxUsbWakeCancel(void);
static void Lpc546xxUsbWakeFrame(void);

static bool s_Prepared;
static bool s_Closing;
static bool s_Releasing;
static bool s_Detached;
static bool s_Initialized;
static bool s_SofEnabled;
static bool s_WakeAddedForce;
// 0 idle, 1..3 waiting for bus idle, 4 waiting for host resume.
static int s_WakeFrames;
static int s_VbusInt = -1;
static int s_IntPrio;
static LpcUsbFsEp0_t s_Ep0 = {
	.pReg = USB0,
	.Ready = Lpc546xxUsbControlReady,
	.Fault = Lpc546xxUsbControlFault,
};

static LpcUsbFsData_t s_Data = {
	.pEp0 = &s_Ep0,
	.pPrepared = &s_Prepared,
	.pClosing = &s_Closing,
	.Irq = USB0_IRQn,
};

static void Lpc546xxUsbBusReset(uint32_t Status, bool Detached);

LpcUsbFsData_t *LpcUsbFsDataContext(void)
{
	return &s_Data;
}

static bool Lpc546xxUsbControlReady(void)
{
	return s_Prepared && !s_Closing && !s_Detached;
}

static void Lpc546xxUsbControlFault(void)
{
	s_SofEnabled = false;
	Lpc546xxUsbWakeCancel();
	s_Ep0.Enabled = false;
	++s_Ep0.Generation;
	USB0->INTEN = 0U;
	NVIC_DisableIRQ(USB0_IRQn);
	LpcUsbFsEp0Command(&s_Ep0, LPCUSB_CONNECT, 0U, 0U);
	// Keep storage and clocks owned until Release can retire every bank.
}

bool Lpc546xxUsbControlPrepare(int IntPrio, void (*Event)(const UsbCtrlrEvt_t *))
{
	const uint32_t state = DisableInterrupt();
	if (s_Prepared || s_Releasing || Event == NULL)
	{
		EnableInterrupt(state);
		return false;
	}
	LpcUsbFsMemory_t *memory = Lpc546xxUsbHwPrepare(IntPrio);
	if (memory == NULL)
	{
		EnableInterrupt(state);
		return false;
	}
	s_Ep0.pMemory = memory;
	s_Ep0.Event = Event;
	s_Prepared = true;
	s_Closing = false;
	s_Detached = false;
	s_SofEnabled = false;
	LpcUsbFsEp0Reset(&s_Ep0);
	USB0->INTSTAT = 0xC00003FFU;
	LpcUsbFsEp0Command(&s_Ep0, 0x7FU | LPCUSB_NAK_CONTROL,
		LPCUSB_ENABLE, LPCUSB_SETUP | (7U << 24));
	USB0->INTEN = LPCUSB_DEVICE | 3U;
	EnableInterrupt(state);
	return true;
}

bool Lpc546xxUsbControlRelease(void)
{
	const uint32_t state = DisableInterrupt();
	if (s_Releasing || (s_Closing && s_Ep0.Enabled))
	{
		EnableInterrupt(state);
		return false;
	}
	if (!s_Prepared)
	{
		EnableInterrupt(state);
		return true;
	}
	s_Releasing = true;
	s_Closing = true;
	s_SofEnabled = false;
	Lpc546xxUsbControlFault();
	if (!LpcUsbFsEp0Retire(&s_Ep0, 0x3FFU))
	{
		s_Releasing = false;
		EnableInterrupt(state);
		return false;
	}
	LpcUsbFsDataForget(0x3FCU);
	LpcUsbFsEp0Command(&s_Ep0, LPCUSB_ENABLE | LPCUSB_NAK_CONTROL, 0U, 0U);
	const bool released = Lpc546xxUsbHwRelease();
	if (released)
	{
		s_Prepared = false;
		s_Closing = false;
		s_Ep0.pMemory = NULL;
		s_Ep0.Event = NULL;
	}
	s_Releasing = false;
	EnableInterrupt(state);
	return released;
}

int UsbCtrlrEp0Send(int DevNo, uint8_t *pBuffer, int Length)
{
	return LpcUsbFsEp0Send(&s_Ep0, DevNo, pBuffer, Length);
}

bool UsbCtrlrEp0Status(int DevNo, uint8_t EpAddr)
{
	return LpcUsbFsEp0Status(&s_Ep0, DevNo, EpAddr);
}

void UsbCtrlrSetAddress(int DevNo, uint8_t Address)
{
	LpcUsbFsEp0SetAddress(&s_Ep0, DevNo, Address);
}

static void Lpc546xxUsbBusReset(uint32_t Status, bool Detached)
{
	s_Detached = Detached;
	Lpc546xxUsbWakeCancel();
	if (!LpcUsbFsEp0Retire(&s_Ep0, 0x3FFU))
	{
		Lpc546xxUsbControlFault();
		return;
	}
	LpcUsbFsDataForget(0x3FCU);
	USB0->INTEN &= ~0x3FCU;
	LpcUsbFsEp0Reset(&s_Ep0);
	USB0->INTSTAT = 0x3FFU;
	// A SETUP following bus reset must survive this interrupt. Cable loss
	// instead discards the old request, even if VBUS has returned already.
	const bool clearSetup = Detached || (Status & LPCUSB_RESET) == 0U;
	LpcUsbFsEp0Command(&s_Ep0, 0x7FU | LPCUSB_NAK_CONTROL, 0U,
		(Status & (LPCUSB_RESET | LPCUSB_CONNECT_CHANGE | LPCUSB_SUSPEND_CHANGE)) |
		(clearSetup ? LPCUSB_SETUP : 0U));
	if (!clearSetup && (USB0->DEVCMDSTAT & LPCUSB_SETUP) != 0U)
		USB0->INTSETSTAT = 1U;
	UsbCtrlrEvt_t event = {0};
	event.Type = USB_CTRLR_EVT_RESET;
	s_Ep0.Event(&event);
}

void USB0_IRQHandler(void)
{
	const uint32_t state = DisableInterrupt();
	if (!s_Prepared || !s_Ep0.Enabled || s_Closing)
	{
		EnableInterrupt(state);
		return;
	}
	const uint32_t generation = s_Ep0.Generation;
	const uint32_t pending = USB0->INTSTAT & USB0->INTEN;
	const uint32_t status = USB0->DEVCMDSTAT;
	if ((pending & LPCUSB_DEVICE) != 0U)
	{
		USB0->INTSTAT = LPCUSB_DEVICE;
		if ((status & (LPCUSB_RESET | LPCUSB_CONNECT_CHANGE)) != 0U)
		{
			Lpc546xxUsbBusReset(status, s_Initialized && !UsbCtrlrVbusDetected(0));
			if (s_Initialized && (status & LPCUSB_CONNECT_CHANGE) != 0U)
				UsbProcessQue(0);
			__DSB();
			EnableInterrupt(state);
			return;
		}
		LpcUsbFsEp0Command(&s_Ep0, 0U, 0U, status & LPCUSB_SUSPEND_CHANGE);
		if ((status & LPCUSB_SUSPEND_CHANGE) != 0U)
			Lpc546xxUsbWakeCancel();
		if (!s_Detached && (status & LPCUSB_SUSPEND_CHANGE) != 0U)
		{
			UsbCtrlrEvt_t event = {0};
			event.Type = (status & (1U << 17)) != 0U ?
				USB_CTRLR_EVT_SUSPEND : USB_CTRLR_EVT_RESUME;
			s_Ep0.Event(&event);
		}
	}
	if (Lpc546xxUsbControlReady() && s_Ep0.Enabled && s_Ep0.Generation == generation &&
		(USB0->DEVCMDSTAT & LPCUSB_RESET) == 0U && (pending & 3U) != 0U)
		LpcUsbFsEp0Interrupt(&s_Ep0, pending);
	for (unsigned physical = 2U; physical < 10U && LpcUsbFsDataCurrent(); ++physical)
		LpcUsbFsDataInterrupt(physical);
	if (Lpc546xxUsbControlReady() && s_Ep0.Enabled && s_Ep0.Generation == generation &&
		(USB0->DEVCMDSTAT & LPCUSB_RESET) == 0U &&
		(USB0->INTSTAT & USB0->INTEN & (1U << 30)) != 0U)
	{
		USB0->INTSTAT = 1U << 30;
		Lpc546xxUsbWakeFrame();
		if (s_SofEnabled)
		{
			UsbCtrlrEvt_t event = {0};
			event.Type = USB_CTRLR_EVT_SOF;
			event.FrameNo = USB0->INFO & 0x7FFU;
			s_Ep0.Event(&event);
		}
	}
	__DSB();
	EnableInterrupt(state);
}

static void Lpc546xxUsbControlEvent(const UsbCtrlrEvt_t *pEvent)
{
	UsbDevProcessEvent(0, pEvent);
}

static void Lpc546xxUsbVbusEvent(int IntNo, void *pContext)
{
	(void)IntNo;
	(void)pContext;
	// GPIO detection stays live while USB is stopped. DCON_C alone only
	// reports loss of VBUS and cannot start a board powered without a cable.
	UsbProcessQue(0);
}

bool UsbCtrlrInit(int DevNo, const UsbCtrlrCfg_t *pCfg)
{
	if (DevNo != 0 || pCfg == NULL || pCfg->bLowPowerSuspend ||
		pCfg->pIOPinMap == NULL || pCfg->NbIOPins != 1)
		return false;
	const IOPinCfg_t *vbus = &pCfg->pIOPinMap[USB_VBUS_PIN_IDX];
	if (vbus->PortNo != 0 || vbus->PinNo != 22 ||
		vbus->PinOp != IOPINOP_FUNC7 || vbus->PinDir != IOPINDIR_INPUT ||
		vbus->Res != IOPINRES_NONE || vbus->Type != IOPINTYPE_NORMAL)
		return false;
	const uint32_t state = DisableInterrupt();
	// A failed retirement still owns SRAM and must not be reinitialized.
	if (s_Prepared || s_Closing || s_Releasing)
	{
		EnableInterrupt(state);
		return false;
	}
	s_Initialized = false;
	LpcUsbFsDataInit();
	if (s_VbusInt >= 0)
		IOPinDisableInterrupt(s_VbusInt);
	const int maxprio = (1 << __NVIC_PRIO_BITS) - 1;
	s_IntPrio = pCfg->IntPrio < 0 ? 0 :
		(pCfg->IntPrio > maxprio ? maxprio : pCfg->IntPrio);
	IOPinCfg(vbus, 1);
	s_VbusInt = IOPinAllocateInterrupt(s_IntPrio, 0, 22, IOPINSENSE_TOGGLE,
		Lpc546xxUsbVbusEvent, NULL);
	s_Initialized = s_VbusInt >= 0;
	EnableInterrupt(state);
	return s_Initialized;
}

bool UsbCtrlrVbusDetected(int DevNo)
{
	// GPIO reads the pad while USB clocks are disabled.
	// No USB register access is needed while its clocks are disabled.
	return DevNo == 0 && s_Initialized && IOPinRead(0, 22) != 0;
}

bool UsbCtrlrStart(int DevNo)
{
	if (DevNo != 0 || s_Closing)
		return false;
	const uint32_t state = DisableInterrupt();
	bool ready = false;
	if (UsbCtrlrVbusDetected(0))
	{
		if (s_Prepared)
			ready = s_Ep0.Enabled;
		else
			ready = Lpc546xxUsbControlPrepare(s_IntPrio, Lpc546xxUsbControlEvent);
	}
	EnableInterrupt(state);
	return ready;
}

void UsbCtrlrStop(int DevNo)
{
	if (DevNo != 0)
		return;
	const uint32_t state = DisableInterrupt();
	// Retry is allowed after a timed-out skip. Release leaves the controller
	// disconnected and disabled to callers until retirement is confirmed.
	(void)Lpc546xxUsbControlRelease();
	EnableInterrupt(state);
}

void UsbCtrlrProcess(int DevNo)
{
	if (DevNo != 0)
		return;
	const uint32_t state = DisableInterrupt();
	if (s_Prepared && s_Ep0.Enabled && !s_Closing && s_Initialized)
	{
		if (UsbCtrlrVbusDetected(0))
			s_Detached = false;
		else if (!s_Detached)
			Lpc546xxUsbBusReset(USB0->DEVCMDSTAT, true);
	}
	EnableInterrupt(state);
}

bool UsbCtrlrHighSpeed(int DevNo)
{
	(void)DevNo;
	return false;
}

void UsbCtrlrIntEnable(int DevNo)
{
	const uint32_t state = DisableInterrupt();
	if (DevNo == 0 && s_Prepared && s_Ep0.Enabled)
		NVIC_EnableIRQ(USB0_IRQn);
	EnableInterrupt(state);
}

void UsbCtrlrIntDisable(int DevNo)
{
	if (DevNo == 0)
		NVIC_DisableIRQ(USB0_IRQn);
}

void UsbCtrlrConnect(int DevNo)
{
	const uint32_t state = DisableInterrupt();
	if (DevNo == 0 && s_Prepared && s_Ep0.Enabled && UsbCtrlrVbusDetected(0))
		LpcUsbFsEp0Command(&s_Ep0, 0U, LPCUSB_CONNECT, 0U);
	EnableInterrupt(state);
}

void UsbCtrlrDisconnect(int DevNo)
{
	const uint32_t state = DisableInterrupt();
	if (DevNo == 0 && s_Prepared)
	{
		Lpc546xxUsbWakeCancel();
		LpcUsbFsEp0Command(&s_Ep0, LPCUSB_CONNECT, 0U, 0U);
	}
	EnableInterrupt(state);
}


static void Lpc546xxUsbFrameEnable(void)
{
	if (s_SofEnabled || (s_WakeFrames > 0 && s_WakeFrames < 4))
		USB0->INTEN |= USB_INTEN_FRAME_INT_EN_MASK;
	else
		USB0->INTEN &= ~USB_INTEN_FRAME_INT_EN_MASK;
}

static void Lpc546xxUsbWakeCancel(void)
{
	s_WakeFrames = 0;
	if (s_WakeAddedForce)
		LpcUsbFsEp0Command(&s_Ep0, USB_DEVCMDSTAT_FORCE_NEEDCLK_MASK, 0U, 0U);
	s_WakeAddedForce = false;
	Lpc546xxUsbFrameEnable();
}

void UsbCtrlrSofEnable(int DevNo, bool Enable)
{
	const uint32_t state = DisableInterrupt();
	if (DevNo == 0 && s_Prepared && s_Ep0.Enabled)
	{
		s_SofEnabled = Enable;
		USB0->INTSTAT = USB_INTSTAT_FRAME_INT_MASK;
		Lpc546xxUsbFrameEnable();
	}
	EnableInterrupt(state);
}

void UsbCtrlrRemoteWakeup(int DevNo)
{
	const uint32_t state = DisableInterrupt();
	if (DevNo == 0 && Lpc546xxUsbControlReady() && s_Ep0.Enabled &&
		s_WakeFrames == 0 && UsbCtrlrVbusDetected(0))
	{
		const uint32_t status = USB0->DEVCMDSTAT;
		if ((status & (LPCUSB_CONNECT | USB_DEVCMDSTAT_DSUS_MASK)) ==
			(LPCUSB_CONNECT | USB_DEVCMDSTAT_DSUS_MASK) &&
			(status & (LPCUSB_RESET | LPCUSB_CONNECT_CHANGE | LPCUSB_SUSPEND_CHANGE)) == 0U)
		{
			// The generic core checks host permission. DSUS proves 3 ms idle;
			// three fresh 1 ms timer edges add at least 2 ms before signalling.
			// UM10912 table 817: FRAME_INT runs while VBUS and DCON are set.
			USB0->INTSTAT = USB_INTSTAT_FRAME_INT_MASK;
			s_WakeFrames = 3;
			Lpc546xxUsbFrameEnable();
		}
	}
	EnableInterrupt(state);
}

static void Lpc546xxUsbWakeFrame(void)
{
	if (s_WakeFrames == 0 || s_WakeFrames == 4)
		return;
	const uint32_t status = USB0->DEVCMDSTAT;
	if (!UsbCtrlrVbusDetected(0) ||
		(status & (LPCUSB_CONNECT | USB_DEVCMDSTAT_DSUS_MASK)) !=
		(LPCUSB_CONNECT | USB_DEVCMDSTAT_DSUS_MASK) ||
		(status & (LPCUSB_RESET | LPCUSB_CONNECT_CHANGE | LPCUSB_SUSPEND_CHANGE)) != 0U)
	{
		Lpc546xxUsbWakeCancel();
		return;
	}
	if (--s_WakeFrames != 0)
		return;
	// Hardware preparation keeps FRO, USB0 clocks and PHY powered during
	// suspend. UM10912 34.7.6.2 requires FORCE_NEEDCLK before clearing DSUS.
	s_WakeAddedForce = (status & USB_DEVCMDSTAT_FORCE_NEEDCLK_MASK) == 0U;
	LpcUsbFsEp0Command(&s_Ep0, 0U, USB_DEVCMDSTAT_FORCE_NEEDCLK_MASK, 0U);
	__DSB();
	s_WakeFrames = 4;
	// Preserve command bits, never acknowledge W1C flags, and leave LPM_SUS
	// set. Clearing DSUS triggers the controller's timed resume signalling.
	USB0->DEVCMDSTAT = (USB0->DEVCMDSTAT &
		(0xFFU | USB_DEVCMDSTAT_FORCE_NEEDCLK_MASK | (1U << 11) | (0x1FU << 12))) |
		USB_DEVCMDSTAT_LPM_SUS_MASK;
	Lpc546xxUsbFrameEnable();
}

size_t UsbCtrlrGetSerial(int DevNo, char *pBuff, size_t BuffLen)
{
	if (pBuff == NULL || BuffLen == 0U)
		return 0U;
	pBuff[0] = '\0';
	if (DevNo != 0 || BuffLen == 1U)
		return 0U;
	IAP_CMD_STATUS result;
	const uint32_t state = DisableInterrupt();
	const uint32_t powerMask = SYSCON_PDRUNCFG_PDEN_ROM_MASK | SYSCON_PDRUNCFG_PDEN_VD6_MASK;
	const uint32_t poweredDown = SYSCON->PDRUNCFG[0] & powerMask;
	const uint32_t addedClock = SYSCON_AHBCLKCTRL_ROM_MASK & ~SYSCON->AHBCLKCTRL[0];
	SYSCON->PDRUNCFGCLR[0] = powerMask;
	SYSCON->AHBCLKCTRLSET[0] = addedClock;
	__DSB();
	const IAPSTATUS status = IAPReadUID(&result);
	SYSCON->AHBCLKCTRLCLR[0] = addedClock;
	SYSCON->PDRUNCFGSET[0] = poweredDown;
	__DSB();
	EnableInterrupt(state);
	if (status != IAPSTATUS_SUCCESS)
		return 0U;
	// Four ROM UID words, eight uppercase hexadecimal digits per word.
	// Usable before Init: the generic core asks for its serial first.
	const size_t count = BuffLen > 32U ? 32U : BuffLen - 1U;
	for (size_t i = 0U; i < count; ++i)
	{
		const uint8_t digit = (result.Data[i / 8U] >> (28U - (i % 8U) * 4U)) & 15U;
		pBuff[i] = digit < 10U ? '0' + digit : 'A' + digit - 10;
	}
	pBuff[count] = '\0';
	return count;
}

