/**-------------------------------------------------------------------------
@file usb_lpc546xx.c
@brief LPC546xx USB0 full-speed and USB1 high-speed lifecycle and interrupt
handling.

One context per controller. The shared LPC IP3511 code moves the transfers;
this file owns VBUS, the clocks through usb_hw_lpc546xx.c, the bus events, the
bus speed of USB1 and its errata USB.6.

VBUS of USB1 without a pin map: the controller stays on from UsbCtrlrStart
and reads USB1_VBUS (VBUS_DEBOUNCED). The connection stays requested; the
controller applies the pull-up only with VBUS, so the first bus reset of the
host is the attachment and DCON_C the detachment, as in NXP's IP3511 device
driver. A GPIO in the pin map (USB_VBUS_PIN_IDX) instead keeps the controller
off until VBUS appears on it, as USB0 does with P0.22.

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
#include "idelay.h"
#include "lpc_iap.h"

#define LPCUSB_DEVICE (1UL << 31)
#define LPCUSB_FRAME (1UL << 30)
#define LPCUSB_RESET (1UL << 26)
#define LPCUSB_CONNECT_CHANGE (1UL << 24)
#define LPCUSB_SUSPEND_CHANGE (1UL << 25)
#define LPCUSB_SETUP (1UL << 8)
#define LPCUSB_ENABLE (1UL << 7)
#define LPCUSB_FORCE_NEEDCLK (1UL << 9)
#define LPCUSB_CONNECT (1UL << 16)
#define LPCUSB_SUSPEND (1UL << 17)
#define LPCUSB_LPM_SUSPEND (1UL << 19)
#define LPCUSB_NAK_CONTROL (3UL << 14)
#define LPCUSB_VBUS_DEBOUNCED (1UL << 28)
// DEVCMDSTAT command bits a direct write keeps
#define LPCUSB_COMMANDS (0xFFUL | LPCUSB_FORCE_NEEDCLK | (1UL << 11) | (0x1FUL << 12))
// USB1 bus speed after the reset handshake, DEVCMDSTAT bits 23:22
#define LPCUSB_SPEED_POS 22
#define LPCUSB_SPEED_MASK 3UL
#define LPCUSB_SPEED_FULL 1UL
#define LPCUSB_SPEED_HIGH 2UL
// Errata USB.6 (ES_LPC546XX), timings of NXP's LPC54628 SDK
// USB_DeviceHsPhyChirpIssueWorkaround: PHY test mode Force Enable, then a
// disconnection.
#define LPCUSB_TEST_FORCE_ENABLE (5UL << 29)
#define LPCUSB_CHIRP_TEST_MS 100U
#define LPCUSB_CHIRP_DISCONNECT_MS 510U

// USB.6 state of an attachment
typedef enum __Lpc546xx_Usb_Chirp {
	LPC546XX_USB_CHIRP_CHECK,		//!< Check the speed at the next bus reset
	LPC546XX_USB_CHIRP_PENDING,		//!< Full speed seen, UsbCtrlrProcess runs it
	LPC546XX_USB_CHIRP_DONE,		//!< Not again until detached
} Lpc546xxUsbChirp_t;

typedef struct __Lpc546xx_Usb_Dev {
	LpcUsbFsEp0_t Ep0;
	LpcUsbFsData_t Data;
	int DevNo;
	IRQn_Type Irq;
	bool Prepared;
	bool Closing;
	bool Releasing;
	bool Detached;
	bool Initialized;
	bool VbusGpio;			//!< VBUS on a GPIO, else read from the controller
	bool ConnectWanted;		//!< Connection requested by UsbCtrlrConnect
	bool SofEnabled;
	bool WakeAddedForce;
	bool ChirpBusy;			//!< USB.6 sequence running, Release refused
	Lpc546xxUsbChirp_t Chirp;
	// 0 idle, 1..3 waiting for bus idle, 4 waiting for host resume.
	int WakeFrames;
	int VbusInt;
	int IntPrio;
	int VbusPort;
	int VbusPin;
} Lpc546xxUsbDev_t;

static bool Lpc546xxUsb0Ready(void);
static bool Lpc546xxUsb1Ready(void);
static void Lpc546xxUsb0Fault(void);
static void Lpc546xxUsb1Fault(void);
static void Lpc546xxUsbWakeCancel(Lpc546xxUsbDev_t *pDev);
static void Lpc546xxUsbWakeFrame(Lpc546xxUsbDev_t *pDev);

// USB0: ten physical endpoints, full-speed list. USB1: twelve, high-speed
// list and the high-speed device errata.
static Lpc546xxUsbDev_t s_Usb[2] = {
	{
		.Ep0 = {
			.pReg = USB0,
			.BufSize = sizeof(((LpcUsbFsMemory_t *)0)->Packet[0]),
			.NbEp = 5,
			.bHsList = false,
			.bHsErrata = false,
			.Ready = Lpc546xxUsb0Ready,
			.Fault = Lpc546xxUsb0Fault,
		},
		.Data = {
			.pEp0 = &s_Usb[0].Ep0,
			.pPrepared = &s_Usb[0].Prepared,
			.pClosing = &s_Usb[0].Closing,
			.Irq = USB0_IRQn,
		},
		.DevNo = 0,
		.Irq = USB0_IRQn,
		.VbusInt = -1,
	},
	{
		.Ep0 = {
			.pReg = (LpcUsbFsReg_t *)USBHSD,
			.BufSize = sizeof(((Lpc546xxUsbHsMemory_t *)0)->Packet[0]),
			.NbEp = 6,
			.bHsList = true,
			.bHsErrata = true,
			.Ready = Lpc546xxUsb1Ready,
			.Fault = Lpc546xxUsb1Fault,
		},
		.Data = {
			.pEp0 = &s_Usb[1].Ep0,
			.pPrepared = &s_Usb[1].Prepared,
			.pClosing = &s_Usb[1].Closing,
			.Irq = USB1_IRQn,
		},
		.DevNo = 1,
		.Irq = USB1_IRQn,
		.VbusInt = -1,
	},
};

static void Lpc546xxUsbBusReset(Lpc546xxUsbDev_t *pDev, uint32_t Status, bool Detached);

static Lpc546xxUsbDev_t *Lpc546xxUsbDev(int DevNo)
{
	return DevNo == 0 || DevNo == 1 ? &s_Usb[DevNo] : NULL;
}

LpcUsbFsData_t *LpcUsbFsDataContext(int DevNo)
{
	Lpc546xxUsbDev_t *dev = Lpc546xxUsbDev(DevNo);
	return dev != NULL ? &dev->Data : NULL;
}

// Every physical endpoint of the controller
static uint32_t Lpc546xxUsbAll(Lpc546xxUsbDev_t *pDev)
{
	return LpcUsbFsDataMask(&pDev->Data) | 3U;
}

static bool Lpc546xxUsbReady(Lpc546xxUsbDev_t *pDev)
{
	return pDev->Prepared && !pDev->Closing && !pDev->Detached;
}

static bool Lpc546xxUsb0Ready(void)
{
	return Lpc546xxUsbReady(&s_Usb[0]);
}

static bool Lpc546xxUsb1Ready(void)
{
	return Lpc546xxUsbReady(&s_Usb[1]);
}

static void Lpc546xxUsbFault(Lpc546xxUsbDev_t *pDev)
{
	pDev->SofEnabled = false;
	Lpc546xxUsbWakeCancel(pDev);
	pDev->Ep0.Enabled = false;
	++pDev->Ep0.Generation;
	pDev->Ep0.pReg->INTEN = 0U;
	NVIC_DisableIRQ(pDev->Irq);
	LpcUsbFsEp0Command(&pDev->Ep0, LPCUSB_CONNECT, 0U, 0U);
	// Keep storage and clocks owned until Release can retire every bank.
}

static void Lpc546xxUsb0Fault(void)
{
	Lpc546xxUsbFault(&s_Usb[0]);
}

static void Lpc546xxUsb1Fault(void)
{
	Lpc546xxUsbFault(&s_Usb[1]);
}

bool Lpc546xxUsbControlPrepare(int DevNo, int IntPrio, void (*Event)(const UsbCtrlrEvt_t *))
{
	Lpc546xxUsbDev_t *dev = Lpc546xxUsbDev(DevNo);
	if (dev == NULL || Event == NULL)
		return false;
	const uint32_t state = DisableInterrupt();
	if (dev->Prepared || dev->Releasing)
	{
		EnableInterrupt(state);
		return false;
	}
	if (DevNo == 0)
	{
		LpcUsbFsMemory_t *memory = Lpc546xxUsbHwPrepare(IntPrio);
		if (memory == NULL)
		{
			EnableInterrupt(state);
			return false;
		}
		dev->Ep0.pList = memory->Command;
		dev->Ep0.pSetup = memory->Setup;
		dev->Ep0.pBuf = memory->Packet[0];
	}
	else
	{
		Lpc546xxUsbHsMemory_t *memory = Lpc546xxUsbHsHwPrepare(IntPrio);
		if (memory == NULL)
		{
			EnableInterrupt(state);
			return false;
		}
		dev->Ep0.pList = memory->Command;
		dev->Ep0.pSetup = memory->Setup;
		dev->Ep0.pBuf = memory->Packet0[0];
	}
	dev->Ep0.Event = Event;
	dev->Ep0.bHighSpeed = false;
	dev->ConnectWanted = false;
	dev->Prepared = true;
	dev->Closing = false;
	dev->Detached = false;
	dev->SofEnabled = false;
	// A powered up PHY attaches as after a power-on reset (USB.6)
	dev->Chirp = LPC546XX_USB_CHIRP_CHECK;
	LpcUsbFsEp0Reset(&dev->Ep0);
	dev->Ep0.pReg->INTSTAT = LPCUSB_DEVICE | LPCUSB_FRAME | Lpc546xxUsbAll(dev);
	LpcUsbFsEp0Command(&dev->Ep0, 0x7FU | LPCUSB_NAK_CONTROL,
		LPCUSB_ENABLE, LPCUSB_SETUP | (7U << 24));
	dev->Ep0.pReg->INTEN = LPCUSB_DEVICE | 3U;
	EnableInterrupt(state);
	return true;
}

bool Lpc546xxUsbControlRelease(int DevNo)
{
	Lpc546xxUsbDev_t *dev = Lpc546xxUsbDev(DevNo);
	if (dev == NULL)
		return false;
	const uint32_t state = DisableInterrupt();
	if (dev->Releasing || dev->ChirpBusy || (dev->Closing && dev->Ep0.Enabled))
	{
		EnableInterrupt(state);
		return false;
	}
	if (!dev->Prepared)
	{
		EnableInterrupt(state);
		return true;
	}
	dev->Releasing = true;
	dev->Closing = true;
	dev->SofEnabled = false;
	Lpc546xxUsbFault(dev);
	if (!LpcUsbFsEp0Retire(&dev->Ep0, Lpc546xxUsbAll(dev)))
	{
		dev->Releasing = false;
		EnableInterrupt(state);
		return false;
	}
	LpcUsbFsDataForget(&dev->Data, LpcUsbFsDataMask(&dev->Data));
	LpcUsbFsEp0Command(&dev->Ep0, LPCUSB_ENABLE | LPCUSB_NAK_CONTROL, 0U, 0U);
	const bool released = DevNo == 0 ? Lpc546xxUsbHwRelease() : Lpc546xxUsbHsHwRelease();
	if (released)
	{
		dev->Prepared = false;
		dev->Closing = false;
		dev->Ep0.pList = NULL;
		dev->Ep0.pSetup = NULL;
		dev->Ep0.pBuf = NULL;
		dev->Ep0.Event = NULL;
	}
	dev->Releasing = false;
	EnableInterrupt(state);
	return released;
}

int UsbCtrlrEp0Send(int DevNo, uint8_t *pBuffer, int Length)
{
	Lpc546xxUsbDev_t *dev = Lpc546xxUsbDev(DevNo);
	return dev != NULL ? LpcUsbFsEp0Send(&dev->Ep0, pBuffer, Length) : -1;
}

bool UsbCtrlrEp0Status(int DevNo, uint8_t EpAddr)
{
	Lpc546xxUsbDev_t *dev = Lpc546xxUsbDev(DevNo);
	return dev != NULL && LpcUsbFsEp0Status(&dev->Ep0, EpAddr);
}

void UsbCtrlrSetAddress(int DevNo, uint8_t Address)
{
	Lpc546xxUsbDev_t *dev = Lpc546xxUsbDev(DevNo);
	if (dev != NULL)
		LpcUsbFsEp0SetAddress(&dev->Ep0, Address);
}

static void Lpc546xxUsbBusReset(Lpc546xxUsbDev_t *pDev, uint32_t Status, bool Detached)
{
	pDev->Detached = Detached;
	Lpc546xxUsbWakeCancel(pDev);
	if (!LpcUsbFsEp0Retire(&pDev->Ep0, Lpc546xxUsbAll(pDev)))
	{
		Lpc546xxUsbFault(pDev);
		return;
	}
	LpcUsbFsDataForget(&pDev->Data, LpcUsbFsDataMask(&pDev->Data));
	pDev->Ep0.pReg->INTEN &= ~LpcUsbFsDataMask(&pDev->Data);
	LpcUsbFsEp0Reset(&pDev->Ep0);
	pDev->Ep0.pReg->INTSTAT = Lpc546xxUsbAll(pDev);
	// The bus speed of USB1 is known at the end of the reset handshake. A
	// full-speed bus at the first reset of an attachment may be USB.6.
	const uint32_t speed = (Status >> LPCUSB_SPEED_POS) & LPCUSB_SPEED_MASK;
	pDev->Ep0.bHighSpeed = pDev->DevNo == 1 && !Detached && speed == LPCUSB_SPEED_HIGH;
	if (Detached)
		pDev->Chirp = LPC546XX_USB_CHIRP_CHECK;
	else if (pDev->DevNo == 1 && (Status & LPCUSB_RESET) != 0U && speed == LPCUSB_SPEED_FULL &&
		pDev->Chirp == LPC546XX_USB_CHIRP_CHECK)
		pDev->Chirp = LPC546XX_USB_CHIRP_PENDING;
	// A SETUP following bus reset must survive this interrupt. Cable loss
	// instead discards the old request, even if VBUS has returned already.
	const bool clearSetup = Detached || (Status & LPCUSB_RESET) == 0U;
	LpcUsbFsEp0Command(&pDev->Ep0, 0x7FU | LPCUSB_NAK_CONTROL, 0U,
		(Status & (LPCUSB_RESET | LPCUSB_CONNECT_CHANGE | LPCUSB_SUSPEND_CHANGE)) |
		(clearSetup ? LPCUSB_SETUP : 0U));
	if (!clearSetup && (pDev->Ep0.pReg->DEVCMDSTAT & LPCUSB_SETUP) != 0U)
		pDev->Ep0.pReg->INTSETSTAT = 1U;
	// Without a VBUS pin, the next attachment shows only as a bus reset: the
	// connection must stay requested while detached.
	if (Detached && !pDev->VbusGpio && pDev->ConnectWanted)
		LpcUsbFsEp0Command(&pDev->Ep0, 0U, LPCUSB_CONNECT, 0U);
	UsbCtrlrEvt_t event = {0};
	event.Type = USB_CTRLR_EVT_RESET;
	pDev->Ep0.Event(&event);
}

static void Lpc546xxUsbInterrupt(Lpc546xxUsbDev_t *pDev)
{
	const uint32_t state = DisableInterrupt();
	if (!pDev->Prepared || !pDev->Ep0.Enabled || pDev->Closing)
	{
		EnableInterrupt(state);
		return;
	}
	const uint32_t generation = pDev->Ep0.Generation;
	const uint32_t pending = pDev->Ep0.pReg->INTSTAT & pDev->Ep0.pReg->INTEN;
	const uint32_t status = pDev->Ep0.pReg->DEVCMDSTAT;
	if ((pending & LPCUSB_DEVICE) != 0U)
	{
		pDev->Ep0.pReg->INTSTAT = LPCUSB_DEVICE;
		if ((status & (LPCUSB_RESET | LPCUSB_CONNECT_CHANGE)) != 0U)
		{
			Lpc546xxUsbBusReset(pDev, status, pDev->Initialized && !UsbCtrlrVbusDetected(pDev->DevNo));
			if (pDev->Initialized && (status & LPCUSB_CONNECT_CHANGE) != 0U)
				UsbProcessQue(pDev->DevNo);
			__DSB();
			EnableInterrupt(state);
			return;
		}
		LpcUsbFsEp0Command(&pDev->Ep0, 0U, 0U, status & LPCUSB_SUSPEND_CHANGE);
		if ((status & LPCUSB_SUSPEND_CHANGE) != 0U)
			Lpc546xxUsbWakeCancel(pDev);
		if (!pDev->Detached && (status & LPCUSB_SUSPEND_CHANGE) != 0U)
		{
			UsbCtrlrEvt_t event = {0};
			event.Type = (status & LPCUSB_SUSPEND) != 0U ?
				USB_CTRLR_EVT_SUSPEND : USB_CTRLR_EVT_RESUME;
			pDev->Ep0.Event(&event);
		}
	}
	if (Lpc546xxUsbReady(pDev) && pDev->Ep0.Enabled && pDev->Ep0.Generation == generation &&
		(pDev->Ep0.pReg->DEVCMDSTAT & LPCUSB_RESET) == 0U && (pending & 3U) != 0U)
		LpcUsbFsEp0Interrupt(&pDev->Ep0, pending);
	const unsigned count = 2U * pDev->Ep0.NbEp;
	for (unsigned physical = 2U; physical < count && LpcUsbFsDataCurrent(&pDev->Data); ++physical)
		LpcUsbFsDataInterrupt(&pDev->Data, physical);
	if (Lpc546xxUsbReady(pDev) && pDev->Ep0.Enabled && pDev->Ep0.Generation == generation &&
		(pDev->Ep0.pReg->DEVCMDSTAT & LPCUSB_RESET) == 0U &&
		(pDev->Ep0.pReg->INTSTAT & pDev->Ep0.pReg->INTEN & LPCUSB_FRAME) != 0U)
	{
		pDev->Ep0.pReg->INTSTAT = LPCUSB_FRAME;
		Lpc546xxUsbWakeFrame(pDev);
		if (pDev->SofEnabled)
		{
			UsbCtrlrEvt_t event = {0};
			event.Type = USB_CTRLR_EVT_SOF;
			event.FrameNo = pDev->Ep0.pReg->INFO & 0x7FFU;
			pDev->Ep0.Event(&event);
		}
	}
	__DSB();
	EnableInterrupt(state);
}

void USB0_IRQHandler(void)
{
	Lpc546xxUsbInterrupt(&s_Usb[0]);
}

void USB1_IRQHandler(void)
{
	Lpc546xxUsbInterrupt(&s_Usb[1]);
}

static void Lpc546xxUsb0Event(const UsbCtrlrEvt_t *pEvent)
{
	UsbDevProcessEvent(0, pEvent);
}

static void Lpc546xxUsb1Event(const UsbCtrlrEvt_t *pEvent)
{
	UsbDevProcessEvent(1, pEvent);
}

static void Lpc546xxUsbVbusEvent(int IntNo, void *pContext)
{
	(void)IntNo;
	// GPIO detection stays live while USB is stopped. DCON_C alone only
	// reports loss of VBUS and cannot start a board powered without a cable.
	UsbProcessQue(((Lpc546xxUsbDev_t *)pContext)->DevNo);
}

// USB0 VBUS: P0.22 in its USB0_VBUS function, read as a GPIO as well. USB1
// VBUS: by default no pin map, the controller reads USB1_VBUS while it runs;
// or a GPIO input as the override, with its interrupt. USB1_VBUS still gives
// the bus power to the PHY in both cases.
bool UsbCtrlrInit(int DevNo, const UsbCtrlrCfg_t *pCfg)
{
	Lpc546xxUsbDev_t *dev = Lpc546xxUsbDev(DevNo);
	if (dev == NULL || pCfg == NULL || pCfg->bLowPowerSuspend)
		return false;
	const bool gpio = pCfg->NbIOPins != 0;
	if (gpio ? pCfg->pIOPinMap == NULL || pCfg->NbIOPins != 1 : DevNo == 0)
		return false;
	const IOPinCfg_t *vbus = gpio ? &pCfg->pIOPinMap[USB_VBUS_PIN_IDX] : NULL;
	if (gpio && (vbus->PortNo < 0 || vbus->PinNo < 0 || vbus->PinDir != IOPINDIR_INPUT ||
		(DevNo == 0 && (vbus->PortNo != 0 || vbus->PinNo != 22 ||
		 vbus->PinOp != IOPINOP_FUNC7 || vbus->Res != IOPINRES_NONE ||
		 vbus->Type != IOPINTYPE_NORMAL))))
		return false;
	const uint32_t state = DisableInterrupt();
	// A failed retirement still owns SRAM and must not be reinitialized.
	if (dev->Prepared || dev->Closing || dev->Releasing)
	{
		EnableInterrupt(state);
		return false;
	}
	dev->Initialized = false;
	LpcUsbFsDataInit(&dev->Data);
	if (dev->VbusInt >= 0)
	{
		IOPinDisableInterrupt(dev->VbusInt);
		dev->VbusInt = -1;
	}
	const int maxprio = (1 << __NVIC_PRIO_BITS) - 1;
	dev->IntPrio = pCfg->IntPrio < 0 ? 0 :
		(pCfg->IntPrio > maxprio ? maxprio : pCfg->IntPrio);
	dev->VbusGpio = gpio;
	if (gpio)
	{
		IOPinCfg(vbus, 1);
		dev->VbusPort = vbus->PortNo;
		dev->VbusPin = vbus->PinNo;
		dev->VbusInt = IOPinAllocateInterrupt(dev->IntPrio, vbus->PortNo, vbus->PinNo,
			IOPINSENSE_TOGGLE, Lpc546xxUsbVbusEvent, dev);
	}
	dev->Initialized = !gpio || dev->VbusInt >= 0;
	EnableInterrupt(state);
	return dev->Initialized;
}

bool UsbCtrlrVbusDetected(int DevNo)
{
	Lpc546xxUsbDev_t *dev = Lpc546xxUsbDev(DevNo);
	if (dev == NULL || !dev->Initialized)
		return false;
	// GPIO reads the pad while USB clocks are disabled. Without a VBUS pin,
	// VBUS_DEBOUNCED is readable only while the controller runs.
	return dev->VbusGpio ? IOPinRead(dev->VbusPort, dev->VbusPin) != 0 :
		dev->Prepared && (dev->Ep0.pReg->DEVCMDSTAT & LPCUSB_VBUS_DEBOUNCED) != 0U;
}

// The connection can be requested: VBUS on the GPIO, or no VBUS pin, where
// the controller applies the pull-up only with VBUS.
static bool Lpc546xxUsbConnectAllowed(Lpc546xxUsbDev_t *pDev)
{
	return !pDev->VbusGpio || UsbCtrlrVbusDetected(pDev->DevNo);
}

bool UsbCtrlrStart(int DevNo)
{
	Lpc546xxUsbDev_t *dev = Lpc546xxUsbDev(DevNo);
	if (dev == NULL || dev->Closing)
		return false;
	const uint32_t state = DisableInterrupt();
	bool ready = false;
	// Without a VBUS pin the controller starts at once to watch VBUS
	if (dev->Initialized && Lpc546xxUsbConnectAllowed(dev))
	{
		if (dev->Prepared)
			ready = dev->Ep0.Enabled;
		else
			ready = Lpc546xxUsbControlPrepare(DevNo, dev->IntPrio,
				DevNo == 0 ? Lpc546xxUsb0Event : Lpc546xxUsb1Event);
	}
	EnableInterrupt(state);
	return ready;
}

void UsbCtrlrStop(int DevNo)
{
	if (Lpc546xxUsbDev(DevNo) == NULL)
		return;
	const uint32_t state = DisableInterrupt();
	// Retry is allowed after a timed-out skip. Release leaves the controller
	// disconnected and disabled to callers until retirement is confirmed.
	(void)Lpc546xxUsbControlRelease(DevNo);
	EnableInterrupt(state);
}

// Errata USB.6 (ES_LPC546XX): at an attachment the device can miss the
// high-speed handshake of the host and run at full speed. NXP's workaround
// puts the PHY in test mode Force Enable for 100 ms, disconnects for 510 ms
// and connects again; the host then resets the bus at high speed. NXP also
// reads the frame number to tell the host speed, both results end the same
// way. The USB interrupt stays off meanwhile.
static void Lpc546xxUsbChirp(Lpc546xxUsbDev_t *pDev)
{
	const uint32_t enabled = NVIC_GetEnableIRQ(pDev->Irq);
	NVIC_DisableIRQ(pDev->Irq);
	LpcUsbFsEp0Command(&pDev->Ep0, 0U, LPCUSB_TEST_FORCE_ENABLE, 0U);
	msDelay(LPCUSB_CHIRP_TEST_MS);
	// A command write leaves the test mode
	LpcUsbFsEp0Command(&pDev->Ep0, 0U, 0U, 0U);
	LpcUsbFsEp0Command(&pDev->Ep0, LPCUSB_CONNECT, 0U, 0U);
	msDelay(LPCUSB_CHIRP_DISCONNECT_MS);
	const uint32_t state = DisableInterrupt();
	pDev->ChirpBusy = false;
	pDev->Chirp = LPC546XX_USB_CHIRP_DONE;
	if (pDev->Prepared && pDev->Ep0.Enabled && !pDev->Closing)
	{
		if (Lpc546xxUsbConnectAllowed(pDev))
		{
			LpcUsbFsEp0Command(&pDev->Ep0, 0U, 0U, LPCUSB_CONNECT_CHANGE);
			LpcUsbFsEp0Command(&pDev->Ep0, 0U, LPCUSB_CONNECT, LPCUSB_RESET);
		}
		if (enabled != 0U)
			NVIC_EnableIRQ(pDev->Irq);
	}
	EnableInterrupt(state);
}

void UsbCtrlrProcess(int DevNo)
{
	Lpc546xxUsbDev_t *dev = Lpc546xxUsbDev(DevNo);
	if (dev == NULL)
		return;
	const uint32_t state = DisableInterrupt();
	bool chirp = false;
	if (dev->Prepared && dev->Ep0.Enabled && !dev->Closing && dev->Initialized)
	{
		if (UsbCtrlrVbusDetected(DevNo))
		{
			dev->Detached = false;
			chirp = dev->Chirp == LPC546XX_USB_CHIRP_PENDING && !dev->ChirpBusy;
			dev->ChirpBusy = dev->ChirpBusy || chirp;
		}
		else if (!dev->Detached)
			Lpc546xxUsbBusReset(dev, dev->Ep0.pReg->DEVCMDSTAT, true);
	}
	EnableInterrupt(state);
	if (chirp)
		Lpc546xxUsbChirp(dev);
}

bool UsbCtrlrHighSpeed(int DevNo)
{
	Lpc546xxUsbDev_t *dev = Lpc546xxUsbDev(DevNo);
	return dev != NULL && dev->Prepared && dev->Ep0.bHighSpeed;
}

void UsbCtrlrIntEnable(int DevNo)
{
	Lpc546xxUsbDev_t *dev = Lpc546xxUsbDev(DevNo);
	const uint32_t state = DisableInterrupt();
	if (dev != NULL && dev->Prepared && dev->Ep0.Enabled)
		NVIC_EnableIRQ(dev->Irq);
	EnableInterrupt(state);
}

void UsbCtrlrIntDisable(int DevNo)
{
	Lpc546xxUsbDev_t *dev = Lpc546xxUsbDev(DevNo);
	if (dev != NULL)
		NVIC_DisableIRQ(dev->Irq);
}

void UsbCtrlrConnect(int DevNo)
{
	Lpc546xxUsbDev_t *dev = Lpc546xxUsbDev(DevNo);
	const uint32_t state = DisableInterrupt();
	if (dev != NULL && dev->Prepared && dev->Ep0.Enabled && Lpc546xxUsbConnectAllowed(dev))
	{
		dev->ConnectWanted = true;
		LpcUsbFsEp0Command(&dev->Ep0, 0U, LPCUSB_CONNECT, 0U);
	}
	EnableInterrupt(state);
}

void UsbCtrlrDisconnect(int DevNo)
{
	Lpc546xxUsbDev_t *dev = Lpc546xxUsbDev(DevNo);
	const uint32_t state = DisableInterrupt();
	if (dev != NULL && dev->Prepared)
	{
		dev->ConnectWanted = false;
		Lpc546xxUsbWakeCancel(dev);
		LpcUsbFsEp0Command(&dev->Ep0, LPCUSB_CONNECT, 0U, 0U);
	}
	EnableInterrupt(state);
}


static void Lpc546xxUsbFrameEnable(Lpc546xxUsbDev_t *pDev)
{
	if (pDev->SofEnabled || (pDev->WakeFrames > 0 && pDev->WakeFrames < 4))
		pDev->Ep0.pReg->INTEN |= LPCUSB_FRAME;
	else
		pDev->Ep0.pReg->INTEN &= ~LPCUSB_FRAME;
}

static void Lpc546xxUsbWakeCancel(Lpc546xxUsbDev_t *pDev)
{
	pDev->WakeFrames = 0;
	if (pDev->WakeAddedForce)
		LpcUsbFsEp0Command(&pDev->Ep0, LPCUSB_FORCE_NEEDCLK, 0U, 0U);
	pDev->WakeAddedForce = false;
	Lpc546xxUsbFrameEnable(pDev);
}

void UsbCtrlrSofEnable(int DevNo, bool Enable)
{
	Lpc546xxUsbDev_t *dev = Lpc546xxUsbDev(DevNo);
	const uint32_t state = DisableInterrupt();
	if (dev != NULL && dev->Prepared && dev->Ep0.Enabled)
	{
		dev->SofEnabled = Enable;
		dev->Ep0.pReg->INTSTAT = LPCUSB_FRAME;
		Lpc546xxUsbFrameEnable(dev);
	}
	EnableInterrupt(state);
}

// USB1: errata USB.3 and USB.4 (ES_LPC546XX), the host can see a
// disconnection at the end of the resume signalling and enumerate again.
void UsbCtrlrRemoteWakeup(int DevNo)
{
	Lpc546xxUsbDev_t *dev = Lpc546xxUsbDev(DevNo);
	const uint32_t state = DisableInterrupt();
	if (dev != NULL && Lpc546xxUsbReady(dev) && dev->Ep0.Enabled &&
		dev->WakeFrames == 0 && UsbCtrlrVbusDetected(DevNo))
	{
		const uint32_t status = dev->Ep0.pReg->DEVCMDSTAT;
		if ((status & (LPCUSB_CONNECT | LPCUSB_SUSPEND)) == (LPCUSB_CONNECT | LPCUSB_SUSPEND) &&
			(status & (LPCUSB_RESET | LPCUSB_CONNECT_CHANGE | LPCUSB_SUSPEND_CHANGE)) == 0U)
		{
			// The generic core checks host permission. DSUS proves 3 ms idle;
			// three fresh 1 ms timer edges add at least 2 ms before signalling.
			// UM10912 table 817: FRAME_INT runs while VBUS and DCON are set.
			dev->Ep0.pReg->INTSTAT = LPCUSB_FRAME;
			dev->WakeFrames = 3;
			Lpc546xxUsbFrameEnable(dev);
		}
	}
	EnableInterrupt(state);
}

static void Lpc546xxUsbWakeFrame(Lpc546xxUsbDev_t *pDev)
{
	if (pDev->WakeFrames == 0 || pDev->WakeFrames == 4)
		return;
	const uint32_t status = pDev->Ep0.pReg->DEVCMDSTAT;
	if (!UsbCtrlrVbusDetected(pDev->DevNo) ||
		(status & (LPCUSB_CONNECT | LPCUSB_SUSPEND)) != (LPCUSB_CONNECT | LPCUSB_SUSPEND) ||
		(status & (LPCUSB_RESET | LPCUSB_CONNECT_CHANGE | LPCUSB_SUSPEND_CHANGE)) != 0U)
	{
		Lpc546xxUsbWakeCancel(pDev);
		return;
	}
	if (--pDev->WakeFrames != 0)
		return;
	// Hardware preparation keeps the USB clocks and PHY powered during
	// suspend. UM10912 34.7.6.2 requires FORCE_NEEDCLK before clearing DSUS.
	pDev->WakeAddedForce = (status & LPCUSB_FORCE_NEEDCLK) == 0U;
	LpcUsbFsEp0Command(&pDev->Ep0, 0U, LPCUSB_FORCE_NEEDCLK, 0U);
	__DSB();
	pDev->WakeFrames = 4;
	// Preserve command bits, never acknowledge W1C flags, and leave LPM_SUS
	// set. Clearing DSUS triggers the controller's timed resume signalling.
	pDev->Ep0.pReg->DEVCMDSTAT = (pDev->Ep0.pReg->DEVCMDSTAT & LPCUSB_COMMANDS) |
		LPCUSB_LPM_SUSPEND;
	Lpc546xxUsbFrameEnable(pDev);
}

// Both controllers report the same ROM UID
size_t UsbCtrlrGetSerial(int DevNo, char *pBuff, size_t BuffLen)
{
	if (pBuff == NULL || BuffLen == 0U)
		return 0U;
	pBuff[0] = '\0';
	if (Lpc546xxUsbDev(DevNo) == NULL || BuffLen == 1U)
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

