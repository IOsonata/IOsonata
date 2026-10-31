/**-------------------------------------------------------------------------
@file	usb_ctrlr_samd.cpp

@brief	SAM D11/D21 USB full speed device controller.

The endpoint descriptors are in RAM (DESCADD), one OUT bank (bank 0) and one
IN bank (bank 1) per endpoint number. Non-control payload storage belongs to
UsbIntrf, the controller DMA reads and writes it in place. EP0 copies its
data into its own buffers.

An OUT bank marked ready (EPSTATUS.BK0RDY) is full: the device NAKs OUT
tokens until a destination is given. An IN bank not ready (BK1RDY) is
empty: the device NAKs IN tokens until data is given. The OUT bank of EP0 is
armed for the stage expecting it, a SETUP is received whatever its state.

The USB clock is generator 0 at 48 MHz, g_McuOsc.bUSBClk on, see
system_samd.c. The D- and D+ pins are PA24 and PA25, function G. The
optional VBUS pin is entry USB_VBUS_PIN_IDX of the pin map, on its EIC line.

The suspend interrupt is on once the device has an address and off after a
bus reset: the SUSPEND flag is also set while the bus is not driven, before
the host has settled the reset, as in TinyUSB.

@author	Hoang Nguyen Hoan
@date	Oct. 30, 2026

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

#include "samdxxx.h"
#include "coredev/interrupt.h"
#include "coredev/iopincfg.h"
#include "coredev/system_core_clock.h"
#include "iopinctrl.h"
#include "clock_samd.h"
#include "usb/usb.h"
#include "usb_ctrlr.h"

// The SAM D21 USB also has a host mode, its registers are a union
#if defined(USB_HOST_PIPE_NUMBER)
#define SAMD_USB					(&USB_REGS->DEVICE)
#else
#define SAMD_USB					USB_REGS
#endif

enum {
	SAMD_USB_EP_COUNT = USB_EPT_NUM,
	SAMD_USB_OUT = 0,					// Bank 0
	SAMD_USB_IN = 1,					// Bank 1
	SAMD_USB_EP0_MPS = 64,
	SAMD_USB_SIZE_64 = 3,				// PCKSIZE.SIZE code of 64 bytes
	SAMD_USB_SIZE_MAX = 7,				// 1023 bytes, isochronous only
	SAMD_USB_SYNC_POLL = 100000,
	SAMD_USB_D_PORT = 0,				// PA24 D-, PA25 D+, function G
	SAMD_USB_DM_PIN = 24,
	SAMD_USB_DP_PIN = 25,
	SAMD_USB_SERIAL_LEN = 32,			// 128 bits in hexadecimal
};

// Pad calibration when the NVM software calibration area is erased, as ASF
enum {
	SAMD_USB_TRANSN_DEFAULT = 5,
	SAMD_USB_TRANSP_DEFAULT = 29,
	SAMD_USB_TRIM_DEFAULT = 3,
};

#define SAMD_USB_OTP4_WORD_1		(*(volatile const uint32_t *)(OTP4_ADDR + 4UL))

// Full speed bit rate tolerance, 0.25 % of 48 MHz
#define SAMD_USB_CLK_MIN			47880000UL
#define SAMD_USB_CLK_MAX			48120000UL

static const uint16_t SAMD_USB_RESUME_EVENTS = USB_DEVICE_INTFLAG_WAKEUP_Msk |
	USB_DEVICE_INTFLAG_EORSM_Msk | USB_DEVICE_INTFLAG_UPRSM_Msk;

// Endpoint interrupt flags of each bank
static const uint8_t SAMD_USB_EP_FLAGS[2] = {
	USB_DEVICE_EPINTFLAG_TRCPT0_Msk | USB_DEVICE_EPINTFLAG_TRFAIL0_Msk | USB_DEVICE_EPINTFLAG_STALL0_Msk,
	USB_DEVICE_EPINTFLAG_TRCPT1_Msk | USB_DEVICE_EPINTFLAG_TRFAIL1_Msk | USB_DEVICE_EPINTFLAG_STALL1_Msk,
};

// Serial number words, the first one apart from the other three
static const uint32_t s_SamdUsbSerialAddr[4] = {
	0x0080A00CUL, 0x0080A040UL, 0x0080A044UL, 0x0080A048UL,
};

typedef struct __Samd_Usb_Registration {
	UsbCtrlrEpHandler_t Handler;
	void *pContext;
} SamdUsbReg_t;

typedef struct __Samd_Usb_Endpoint {
	uint32_t Generation;				// Changes on each open and close
	uint16_t Mps;						// 0 - closed
	uint16_t Length;					// IN bytes submitted
	uint8_t Type;
	uint8_t Size;						// PCKSIZE.SIZE code
	bool Busy;							// A buffer is owned by the bank
	bool RxReady;						// DRDY to deliver from the interrupt
} SamdUsbEp_t;

typedef struct __Samd_Usb_State {
	SamdUsbEp_t Ep[SAMD_USB_EP_COUNT][2];
	SamdUsbReg_t Reg[SAMD_USB_EP_COUNT - 1][2];
	int VbusPort;						// -1 - no VBUS pin
	int VbusPin;
	int VbusInt;
	uint16_t Ep0Length;
	uint16_t Ep0OutLeft;				// OUT data stage bytes still expected
	uint8_t PendingAddress;
	bool Initialized;
	bool Started;
	bool Closing;
	bool Suspended;
	bool SofEnabled;
	bool AddressPending;
	bool Ep0Busy;
	bool Ep0Halted;
	bool Ep0In;
	bool Ep0StatusOut;
} SamdUsbState_t;

static SamdUsbState_t s_SamdUsb;
alignas(4) static usb_device_desc_bank_registers_t s_SamdUsbDesc[SAMD_USB_EP_COUNT][2];
alignas(4) static uint8_t s_SamdUsbEp0Out[SAMD_USB_EP0_MPS];
alignas(4) static uint8_t s_SamdUsbEp0In[SAMD_USB_EP0_MPS];
// Leading bytes of an IN source that is not word aligned
alignas(4) static uint8_t s_SamdUsbInWord[SAMD_USB_EP_COUNT][4];

static_assert(sizeof(usb_device_desc_bank_registers_t) == 16, "USB bank descriptor stride");

static inline usb_device_endpoint_registers_t *SamdUsbEpReg(uint8_t EpNo)
{
	return &SAMD_USB->DEVICE_ENDPOINT[EpNo];
}

static inline bool SamdUsbSetupPending(void)
{
	return (SamdUsbEpReg(0)->USB_EPINTFLAG & USB_DEVICE_EPINTFLAG_RXSTP_Msk) != 0U;
}

// Smallest bank size code holding Mps: 8 << Size, 1023 for 7
static uint8_t SamdUsbSize(uint16_t Mps)
{
	uint8_t size = 0U;

	while (size < SAMD_USB_SIZE_MAX && (8U << size) < Mps)
	{
		size++;
	}

	return size;
}

static inline uint16_t SamdUsbBankSize(uint8_t Size)
{
	return Size >= SAMD_USB_SIZE_MAX ? 1023U : (uint16_t)(8U << Size);
}

static uint16_t SamdUsbPadcal(void)
{
	const uint32_t cal = SAMD_USB_OTP4_WORD_1;
	uint32_t transn = (cal & FUSES_OTP4_WORD_1_USB_TRANSN_Msk) >> FUSES_OTP4_WORD_1_USB_TRANSN_Pos;
	uint32_t transp = (cal & FUSES_OTP4_WORD_1_USB_TRANSP_Msk) >> FUSES_OTP4_WORD_1_USB_TRANSP_Pos;
	uint32_t trim = (cal & FUSES_OTP4_WORD_1_USB_TRIM_Msk) >> FUSES_OTP4_WORD_1_USB_TRIM_Pos;

	if (transn == (FUSES_OTP4_WORD_1_USB_TRANSN_Msk >> FUSES_OTP4_WORD_1_USB_TRANSN_Pos))
	{
		transn = SAMD_USB_TRANSN_DEFAULT;
	}
	if (transp == (FUSES_OTP4_WORD_1_USB_TRANSP_Msk >> FUSES_OTP4_WORD_1_USB_TRANSP_Pos))
	{
		transp = SAMD_USB_TRANSP_DEFAULT;
	}
	if (trim == (FUSES_OTP4_WORD_1_USB_TRIM_Msk >> FUSES_OTP4_WORD_1_USB_TRIM_Pos))
	{
		trim = SAMD_USB_TRIM_DEFAULT;
	}

	return (uint16_t)(USB_PADCAL_TRANSN(transn) | USB_PADCAL_TRANSP(transp) | USB_PADCAL_TRIM(trim));
}

// CTRLA is write synchronized. CTRLA.SWRST and SYNCBUSY.SWRST both clear
// at the end of the software reset.
static bool SamdUsbSync(uint8_t Mask)
{
	for (uint32_t i = 0U; i < SAMD_USB_SYNC_POLL; i++)
	{
		if ((SAMD_USB->USB_SYNCBUSY & Mask) == 0U &&
			(Mask != USB_SYNCBUSY_SWRST_Msk || (SAMD_USB->USB_CTRLA & USB_CTRLA_SWRST_Msk) == 0U))
		{
			return true;
		}
	}

	return false;
}

// CTRLB bits cleared and set. UPRSM, cleared by the hardware once the
// upstream resume is sent, is written 0: a 1 written back starts another.
static void SamdUsbCtrlb(uint16_t Clear, uint16_t Set)
{
	SAMD_USB->USB_CTRLB = (uint16_t)((SAMD_USB->USB_CTRLB & ~(Clear | USB_DEVICE_CTRLB_UPRSM_Msk)) | Set);
}

static void SamdUsbNotify(uint8_t EpNo, uint8_t Dir, UsbCtrlrEvtType_t Event, uint16_t Length)
{
	const uint32_t state = DisableInterrupt();
	const SamdUsbReg_t reg = s_SamdUsb.Reg[EpNo - 1U][Dir];
	EnableInterrupt(state);

	if (reg.Handler != nullptr)
	{
		reg.Handler(Event, Length, reg.pContext);
	}
}

static void SamdUsbBusEvent(UsbCtrlrEvtType_t Type)
{
	UsbCtrlrEvt_t event = {};

	event.Type = Type;
	UsbDevProcessEvent(0, &event);
}

static void SamdUsbEp0Result(bool bIn, uint16_t Length, const uint8_t *pBuffer, UsbCtrlrXferResult_t Result)
{
	UsbCtrlrEvt_t event = {};

	event.Type = USB_CTRLR_EVT_XFER_CMPL;
	event.Xfer.EpAddr = bIn ? USB_ENDPADDR_DIR_IN : 0U;
	event.Xfer.Length = Length;
	event.Xfer.pBuffer = pBuffer;
	event.Xfer.Result = Result;
	UsbDevProcessEvent(0, &event);
}

static void SamdUsbVbusEvent(int IntNo, void *pCtx)
{
	(void)IntNo;
	(void)pCtx;
	UsbProcessQue(0);
}

// A valid RAM address and the bank size, no transfer
static void SamdUsbBankInit(uint8_t EpNo, uint8_t Dir, uint8_t Size)
{
	s_SamdUsbDesc[EpNo][Dir].USB_ADDR = (uint32_t)(uintptr_t)s_SamdUsbInWord[EpNo];
	s_SamdUsbDesc[EpNo][Dir].USB_PCKSIZE = USB_DEVICE_PCKSIZE_SIZE(Size);
	s_SamdUsbDesc[EpNo][Dir].USB_EXTREG = 0U;
	s_SamdUsbDesc[EpNo][Dir].USB_STATUS_BK = 0U;
}

// The software state of a closed endpoint direction. The hardware is stopped
// by the caller before the owner gets its buffer back.
static void SamdUsbForget(uint8_t EpNo, uint8_t Dir)
{
	SamdUsbEp_t *ep = &s_SamdUsb.Ep[EpNo][Dir];
	const bool busy = ep->Busy && ep->Mps != 0U;

	ep->Mps = 0U;
	ep->Length = 0U;
	ep->Type = CONTROL;
	ep->Busy = false;
	ep->RxReady = false;
	ep->Generation++;

	if (busy)
	{
		SamdUsbNotify(EpNo, Dir, USB_CTRLR_EVT_CANCEL, 0U);
	}
}

static void SamdUsbEpDisable(uint8_t EpNo, uint8_t Dir)
{
	usb_device_endpoint_registers_t *reg = SamdUsbEpReg(EpNo);

	reg->USB_EPINTENCLR = SAMD_USB_EP_FLAGS[Dir];
	if (Dir == SAMD_USB_IN)
	{
		reg->USB_EPCFG = (uint8_t)(reg->USB_EPCFG & ~USB_DEVICE_EPCFG_EPTYPE1_Msk);
		reg->USB_EPSTATUSCLR = USB_DEVICE_EPSTATUSCLR_BK1RDY_Msk | USB_DEVICE_EPSTATUSCLR_STALLRQ1_Msk;
	}
	else
	{
		reg->USB_EPCFG = (uint8_t)(reg->USB_EPCFG & ~USB_DEVICE_EPCFG_EPTYPE0_Msk);
		reg->USB_EPSTATUSCLR = USB_DEVICE_EPSTATUSCLR_STALLRQ0_Msk;
		reg->USB_EPSTATUSSET = USB_DEVICE_EPSTATUSSET_BK0RDY_Msk;
	}
	reg->USB_EPINTFLAG = SAMD_USB_EP_FLAGS[Dir];
	__DSB();
}

// EP0 OUT bank ready for one data or status packet of up to 64 bytes. The
// bank is armed only for the stage expecting it: a SETUP is written whatever
// BK0RDY. A SETUP that came in just before the arm has set BK0RDY: the bank
// is held again, its bytes stay until the interrupt copies them.
static void SamdUsbEp0ArmOut(void)
{
	usb_device_endpoint_registers_t *reg = SamdUsbEpReg(0);

	s_SamdUsbDesc[0][SAMD_USB_OUT].USB_PCKSIZE = USB_DEVICE_PCKSIZE_SIZE(SAMD_USB_SIZE_64) |
		USB_DEVICE_PCKSIZE_MULTI_PACKET_SIZE(SAMD_USB_EP0_MPS);
	__DMB();
	reg->USB_EPSTATUSCLR = USB_DEVICE_EPSTATUSCLR_BK0RDY_Msk;
	if (SamdUsbSetupPending())
	{
		reg->USB_EPSTATUSSET = USB_DEVICE_EPSTATUSSET_BK0RDY_Msk;
	}
}

static void SamdUsbEp0Init(void)
{
	usb_device_endpoint_registers_t *reg = SamdUsbEpReg(0);

	reg->USB_EPINTENCLR = USB_DEVICE_EPINTFLAG_Msk;
	reg->USB_EPCFG = 0U;
	__DSB();

	s_SamdUsb.Ep0Busy = false;
	s_SamdUsb.Ep0Halted = false;
	s_SamdUsb.Ep0Length = 0U;
	s_SamdUsb.Ep0OutLeft = 0U;
	s_SamdUsb.AddressPending = false;
	s_SamdUsb.Ep0In = false;
	s_SamdUsb.Ep0StatusOut = false;

	s_SamdUsbDesc[0][SAMD_USB_OUT].USB_ADDR = (uint32_t)(uintptr_t)s_SamdUsbEp0Out;
	s_SamdUsbDesc[0][SAMD_USB_OUT].USB_PCKSIZE = USB_DEVICE_PCKSIZE_SIZE(SAMD_USB_SIZE_64) |
		USB_DEVICE_PCKSIZE_MULTI_PACKET_SIZE(SAMD_USB_EP0_MPS);
	s_SamdUsbDesc[0][SAMD_USB_OUT].USB_STATUS_BK = 0U;
	s_SamdUsbDesc[0][SAMD_USB_IN].USB_ADDR = (uint32_t)(uintptr_t)s_SamdUsbEp0In;
	s_SamdUsbDesc[0][SAMD_USB_IN].USB_PCKSIZE = USB_DEVICE_PCKSIZE_SIZE(SAMD_USB_SIZE_64);
	s_SamdUsbDesc[0][SAMD_USB_IN].USB_STATUS_BK = 0U;
	__DMB();

	reg->USB_EPCFG = USB_DEVICE_EPCFG_EPTYPE0(1U) | USB_DEVICE_EPCFG_EPTYPE1(1U);
	reg->USB_EPSTATUSCLR = USB_DEVICE_EPSTATUSCLR_BK1RDY_Msk | USB_DEVICE_EPSTATUSCLR_STALLRQ0_Msk |
		USB_DEVICE_EPSTATUSCLR_STALLRQ1_Msk;
	reg->USB_EPSTATUSSET = USB_DEVICE_EPSTATUSSET_BK0RDY_Msk;
	reg->USB_EPINTFLAG = USB_DEVICE_EPINTFLAG_Msk;
	reg->USB_EPINTENSET = USB_DEVICE_EPINTENSET_RXSTP_Msk | USB_DEVICE_EPINTENSET_TRCPT0_Msk |
		USB_DEVICE_EPINTENSET_TRCPT1_Msk;
}

bool UsbCtrlrInit(int DevNo, const UsbCtrlrCfg_t *pCfg)
{
	if (DevNo != 0 || pCfg == nullptr || pCfg->IntPrio < 0 || pCfg->bLowPowerSuspend ||
		(pCfg->pIOPinMap == nullptr && pCfg->NbIOPins != 0) ||
		(pCfg->pIOPinMap != nullptr && pCfg->NbIOPins <= 0))
	{
		return false;
	}

	const IOPinCfg_t *vbus = pCfg->NbIOPins > USB_VBUS_PIN_IDX ? &pCfg->pIOPinMap[USB_VBUS_PIN_IDX] : nullptr;

	if (vbus != nullptr && vbus->PortNo < 0)
	{
		vbus = nullptr;
	}

	const uint32_t state = DisableInterrupt();

	// Reinitialization requires the port to be stopped
	if (s_SamdUsb.Started || s_SamdUsb.Closing)
	{
		EnableInterrupt(state);

		return false;
	}

	NVIC_DisableIRQ(USB_IRQn);
	if (s_SamdUsb.Initialized && s_SamdUsb.VbusInt >= 0)
	{
		IOPinDisableInterrupt(s_SamdUsb.VbusInt);
	}

	memset(&s_SamdUsb, 0, sizeof(s_SamdUsb));
	s_SamdUsb.VbusPort = -1;
	s_SamdUsb.VbusPin = -1;
	s_SamdUsb.VbusInt = -1;

	NVIC_SetPriority(USB_IRQn, SamdIrqPrio(pCfg->IntPrio));

	if (pCfg->NbIOPins > 0)
	{
		IOPinCfg(pCfg->pIOPinMap, pCfg->NbIOPins);
	}

	if (vbus != nullptr)
	{
		const int intno = IOPinAllocateInterrupt(pCfg->IntPrio, vbus->PortNo, vbus->PinNo,
												 IOPINSENSE_TOGGLE, SamdUsbVbusEvent, nullptr);

		if (intno < 0)
		{
			EnableInterrupt(state);

			return false;
		}

		s_SamdUsb.VbusPort = vbus->PortNo;
		s_SamdUsb.VbusPin = vbus->PinNo;
		s_SamdUsb.VbusInt = intno;
	}

	// The device USB pins, not board wiring
	IOPinConfig(SAMD_USB_D_PORT, SAMD_USB_DM_PIN, IOPINOP_FUNC6, IOPINDIR_INPUT, IOPINRES_NONE, IOPINTYPE_NORMAL);
	IOPinConfig(SAMD_USB_D_PORT, SAMD_USB_DP_PIN, IOPINOP_FUNC6, IOPINDIR_INPUT, IOPINRES_NONE, IOPINTYPE_NORMAL);

	s_SamdUsb.Initialized = true;

	EnableInterrupt(state);

	return true;
}

bool UsbCtrlrVbusDetected(int DevNo)
{
	if (DevNo != 0 || !s_SamdUsb.Initialized)
	{
		return false;
	}

	if (s_SamdUsb.VbusPort < 0)
	{
		return true;
	}

	return IOPinRead(s_SamdUsb.VbusPort, s_SamdUsb.VbusPin) != 0;
}

bool UsbCtrlrStart(int DevNo)
{
	if (DevNo != 0 || !s_SamdUsb.Initialized || s_SamdUsb.Closing || !UsbCtrlrVbusDetected(0))
	{
		return false;
	}

	if (s_SamdUsb.Started)
	{
		return true;
	}

	// 48 MHz main clock set for USB by SystemInit, the clock layer owns it.
	// The DFLL48M on the 32 kHz crystal runs at 47.97 MHz, within the
	// 0.25 % of full speed.
	const uint32_t freq = SystemCoreClockGet();

	if (!g_McuOsc.bUSBClk || freq < SAMD_USB_CLK_MIN || freq > SAMD_USB_CLK_MAX)
	{
		return false;
	}

	PM_REGS->PM_AHBMASK |= PM_AHBMASK_USB_Msk;
	PM_REGS->PM_APBBMASK |= PM_APBBMASK_USB_Msk;

	if (!SamdGclkChannelEnable(USB_GCLK_ID, SAMD_GCLK_MAIN))
	{
		return false;
	}

	const uint32_t state = DisableInterrupt();

	NVIC_DisableIRQ(USB_IRQn);

	SAMD_USB->USB_CTRLA = USB_CTRLA_SWRST_Msk;
	if (!SamdUsbSync(USB_SYNCBUSY_SWRST_Msk))
	{
		EnableInterrupt(state);

		return false;
	}

	SAMD_USB->USB_PADCAL = SamdUsbPadcal();
	SAMD_USB->USB_QOSCTRL = USB_QOSCTRL_CQOS(3U) | USB_QOSCTRL_DQOS(3U);

	for (uint8_t ep = 0U; ep < SAMD_USB_EP_COUNT; ep++)
	{
		SamdUsbBankInit(ep, SAMD_USB_OUT, 0U);
		SamdUsbBankInit(ep, SAMD_USB_IN, 0U);
	}
	__DMB();

	SAMD_USB->USB_DESCADD = (uint32_t)(uintptr_t)s_SamdUsbDesc;
	SAMD_USB->USB_CTRLB = USB_DEVICE_CTRLB_SPDCONF_FS | USB_DEVICE_CTRLB_DETACH_Msk;
	SAMD_USB->USB_CTRLA = USB_CTRLA_MODE_DEVICE | USB_CTRLA_ENABLE_Msk;
	if (!SamdUsbSync(USB_SYNCBUSY_ENABLE_Msk))
	{
		SAMD_USB->USB_CTRLA = 0U;
		EnableInterrupt(state);

		return false;
	}

	SAMD_USB->USB_INTENCLR = USB_DEVICE_INTENCLR_Msk;
	SAMD_USB->USB_INTFLAG = USB_DEVICE_INTFLAG_Msk;
	SAMD_USB->USB_INTENSET = USB_DEVICE_INTENSET_EORST_Msk;

	s_SamdUsb.Suspended = false;
	s_SamdUsb.SofEnabled = false;
	s_SamdUsb.Started = true;

	SamdUsbEp0Init();

	NVIC_ClearPendingIRQ(USB_IRQn);

	EnableInterrupt(state);

	return true;
}

void UsbCtrlrStop(int DevNo)
{
	if (DevNo != 0 || !s_SamdUsb.Started || s_SamdUsb.Closing)
	{
		return;
	}

	const uint32_t state = DisableInterrupt();

	NVIC_DisableIRQ(USB_IRQn);

	// Bus released and DMA stopped before the owners get their buffers back
	SamdUsbCtrlb(0U, USB_DEVICE_CTRLB_DETACH_Msk);
	SAMD_USB->USB_CTRLA = 0U;
	(void)SamdUsbSync(USB_SYNCBUSY_ENABLE_Msk);
	__DSB();

	s_SamdUsb.Started = false;
	s_SamdUsb.Closing = true;
	for (uint8_t ep = 1U; ep < SAMD_USB_EP_COUNT; ep++)
	{
		SamdUsbForget(ep, SAMD_USB_OUT);
		SamdUsbForget(ep, SAMD_USB_IN);
	}
	s_SamdUsb.Closing = false;
	s_SamdUsb.Ep0Busy = false;
	s_SamdUsb.AddressPending = false;
	s_SamdUsb.Suspended = false;
	s_SamdUsb.SofEnabled = false;

	NVIC_ClearPendingIRQ(USB_IRQn);

	EnableInterrupt(state);

	(void)SamdGclkChannelDisable(USB_GCLK_ID);
}

void UsbCtrlrProcess(int DevNo)
{
	(void)DevNo;
}

bool UsbCtrlrHighSpeed(int DevNo)
{
	(void)DevNo;

	return false;
}

void UsbCtrlrIntEnable(int DevNo)
{
	if (DevNo == 0 && s_SamdUsb.Started)
	{
		NVIC_EnableIRQ(USB_IRQn);
	}
}

void UsbCtrlrIntDisable(int DevNo)
{
	if (DevNo == 0)
	{
		NVIC_DisableIRQ(USB_IRQn);
	}
}

void UsbCtrlrConnect(int DevNo)
{
	if (DevNo != 0 || !s_SamdUsb.Started || s_SamdUsb.Closing || !UsbCtrlrVbusDetected(0))
	{
		return;
	}

	const uint32_t state = DisableInterrupt();
	SamdUsbCtrlb(USB_DEVICE_CTRLB_DETACH_Msk, 0U);
	EnableInterrupt(state);
}

void UsbCtrlrDisconnect(int DevNo)
{
	if (DevNo != 0 || !s_SamdUsb.Started)
	{
		return;
	}

	const uint32_t state = DisableInterrupt();

	SamdUsbCtrlb(0U, USB_DEVICE_CTRLB_DETACH_Msk);
	SAMD_USB->USB_INTENCLR = USB_DEVICE_INTENCLR_SUSPEND_Msk | SAMD_USB_RESUME_EVENTS;
	SAMD_USB->USB_INTFLAG = USB_DEVICE_INTFLAG_SUSPEND_Msk | SAMD_USB_RESUME_EVENTS;
	s_SamdUsb.Suspended = false;

	EnableInterrupt(state);
}

void UsbCtrlrRemoteWakeup(int DevNo)
{
	// The generic USB core checks that the host allows it
	if (DevNo != 0 || !s_SamdUsb.Started || !s_SamdUsb.Suspended || !UsbCtrlrVbusDetected(0))
	{
		return;
	}

	const uint32_t state = DisableInterrupt();
	SAMD_USB->USB_CTRLB |= USB_DEVICE_CTRLB_UPRSM_Msk;
	EnableInterrupt(state);
}

void UsbCtrlrSofEnable(int DevNo, bool Enable)
{
	if (DevNo != 0 || !s_SamdUsb.Started)
	{
		return;
	}

	const uint32_t state = DisableInterrupt();

	s_SamdUsb.SofEnabled = Enable;
	SAMD_USB->USB_INTFLAG = USB_DEVICE_INTFLAG_SOF_Msk;
	if (Enable)
	{
		SAMD_USB->USB_INTENSET = USB_DEVICE_INTENSET_SOF_Msk;
	}
	else
	{
		SAMD_USB->USB_INTENCLR = USB_DEVICE_INTENCLR_SOF_Msk;
	}

	EnableInterrupt(state);
}

void UsbCtrlrSetAddress(int DevNo, uint8_t Address)
{
	if (DevNo != 0 || !s_SamdUsb.Started || Address > 127U)
	{
		return;
	}

	// DADD is written once the status stage is sent, the host addresses the
	// device by its old address until then
	const uint32_t state = DisableInterrupt();
	s_SamdUsb.PendingAddress = Address;
	s_SamdUsb.AddressPending = true;
	EnableInterrupt(state);
}

void UsbCtrlrEpBind(int DevNo, uint8_t EpNo, bool bIn, bool bBlocking,
					UsbCtrlrEpHandler_t Handler, void *pContext)
{
	(void)bBlocking;

	if (DevNo != 0 || EpNo == 0U || EpNo >= SAMD_USB_EP_COUNT || s_SamdUsb.Closing)
	{
		return;
	}

	const uint32_t state = DisableInterrupt();
	s_SamdUsb.Reg[EpNo - 1U][bIn ? SAMD_USB_IN : SAMD_USB_OUT].Handler = Handler;
	s_SamdUsb.Reg[EpNo - 1U][bIn ? SAMD_USB_IN : SAMD_USB_OUT].pContext = pContext;
	EnableInterrupt(state);
}

bool UsbCtrlrEpOpenData(int DevNo, uint8_t EpNo, bool bIn, uint8_t Type,
						uint16_t MaxPacketSize)
{
	if (DevNo != 0 || !s_SamdUsb.Started || s_SamdUsb.Closing || EpNo == 0U ||
		EpNo >= SAMD_USB_EP_COUNT ||
		(Type != ISO && Type != BULK && Type != INT) ||
		MaxPacketSize == 0U ||
		MaxPacketSize > (Type == ISO ? USB_CTRLR0_ISO_PKT_LEN_MAX : USB_CTRLR0_BULK_PKT_LEN_MAX) ||
		(Type == BULK && MaxPacketSize != 8U && MaxPacketSize != 16U &&
		 MaxPacketSize != 32U && MaxPacketSize != 64U))
	{
		return false;
	}

	const uint8_t dir = bIn ? SAMD_USB_IN : SAMD_USB_OUT;
	const uint32_t state = DisableInterrupt();
	SamdUsbEp_t *ep = &s_SamdUsb.Ep[EpNo][dir];

	if (ep->Mps != 0U)
	{
		EnableInterrupt(state);

		return false;
	}

	usb_device_endpoint_registers_t *reg = SamdUsbEpReg(EpNo);
	const uint8_t size = SamdUsbSize(MaxPacketSize);

	ep->Mps = MaxPacketSize;
	ep->Length = 0U;
	ep->Type = Type;
	ep->Size = size;
	ep->Busy = false;
	ep->RxReady = false;
	ep->Generation++;

	SamdUsbBankInit(EpNo, dir, size);
	__DMB();

	// EPTYPE is the transfer type plus one. The tokens are NAKed meanwhile:
	// an isochronous interface can be opened again while the host sends.
	const bool gnak = (SAMD_USB->USB_CTRLB & USB_DEVICE_CTRLB_GNAK_Msk) != 0U;

	SamdUsbCtrlb(0U, USB_DEVICE_CTRLB_GNAK_Msk);

	if (bIn)
	{
		reg->USB_EPCFG = (uint8_t)((reg->USB_EPCFG & ~USB_DEVICE_EPCFG_EPTYPE1_Msk) |
								   USB_DEVICE_EPCFG_EPTYPE1(Type + 1U));
		reg->USB_EPSTATUSCLR = USB_DEVICE_EPSTATUSCLR_BK1RDY_Msk | USB_DEVICE_EPSTATUSCLR_STALLRQ1_Msk |
			USB_DEVICE_EPSTATUSCLR_DTGLIN_Msk;
		reg->USB_EPINTFLAG = SAMD_USB_EP_FLAGS[SAMD_USB_IN];
		reg->USB_EPINTENSET = USB_DEVICE_EPINTENSET_TRCPT1_Msk;
	}
	else
	{
		// Bank full until the owner gives a destination
		reg->USB_EPSTATUSSET = USB_DEVICE_EPSTATUSSET_BK0RDY_Msk;
		reg->USB_EPCFG = (uint8_t)((reg->USB_EPCFG & ~USB_DEVICE_EPCFG_EPTYPE0_Msk) |
								   USB_DEVICE_EPCFG_EPTYPE0(Type + 1U));
		reg->USB_EPSTATUSSET = USB_DEVICE_EPSTATUSSET_BK0RDY_Msk;
		reg->USB_EPSTATUSCLR = USB_DEVICE_EPSTATUSCLR_STALLRQ0_Msk | USB_DEVICE_EPSTATUSCLR_DTGLOUT_Msk;
		reg->USB_EPINTFLAG = SAMD_USB_EP_FLAGS[SAMD_USB_OUT];
		// An isochronous frame with a CRC error sets TRFAIL0
		reg->USB_EPINTENSET = USB_DEVICE_EPINTENSET_TRCPT0_Msk |
			(Type == ISO ? USB_DEVICE_EPINTENSET_TRFAIL0_Msk : 0U);

		// The owner is asked for a destination from the interrupt
		ep->RxReady = true;
		NVIC_SetPendingIRQ(USB_IRQn);
	}

	if (!gnak)
	{
		SamdUsbCtrlb(USB_DEVICE_CTRLB_GNAK_Msk, 0U);
	}

	EnableInterrupt(state);

	return true;
}

bool UsbCtrlrEpOpen(int DevNo, const UsbEndPointDesc_t *pDesc)
{
	if (pDesc == nullptr || (pDesc->bEndpointAddress & 0x70U) != 0U)
	{
		return false;
	}

	return UsbCtrlrEpOpenData(DevNo, pDesc->bEndpointAddress & 0x0FU,
							  (pDesc->bEndpointAddress & USB_ENDPADDR_DIR_IN) != 0U,
							  pDesc->bmAttributes & 3U, pDesc->wMaxPacketSize);
}

void UsbCtrlrEpClose(int DevNo, uint8_t EpNo, bool bIn)
{
	if (DevNo != 0 || EpNo == 0U || EpNo >= SAMD_USB_EP_COUNT || s_SamdUsb.Closing)
	{
		return;
	}

	const uint8_t dir = bIn ? SAMD_USB_IN : SAMD_USB_OUT;
	const uint32_t state = DisableInterrupt();

	if (s_SamdUsb.Ep[EpNo][dir].Mps != 0U && s_SamdUsb.Started)
	{
		s_SamdUsb.Closing = true;
		SamdUsbEpDisable(EpNo, dir);
		SamdUsbForget(EpNo, dir);
		s_SamdUsb.Closing = false;
	}

	EnableInterrupt(state);
}

void UsbCtrlrEpCloseAll(int DevNo)
{
	if (DevNo != 0 || !s_SamdUsb.Started || s_SamdUsb.Closing)
	{
		return;
	}

	const uint32_t state = DisableInterrupt();

	s_SamdUsb.Closing = true;
	for (uint8_t ep = 1U; ep < SAMD_USB_EP_COUNT; ep++)
	{
		SamdUsbEpDisable(ep, SAMD_USB_OUT);
		SamdUsbEpDisable(ep, SAMD_USB_IN);
	}
	for (uint8_t ep = 1U; ep < SAMD_USB_EP_COUNT; ep++)
	{
		SamdUsbForget(ep, SAMD_USB_OUT);
		SamdUsbForget(ep, SAMD_USB_IN);
	}
	s_SamdUsb.Closing = false;

	EnableInterrupt(state);
}

bool UsbCtrlrEpReceive(int DevNo, uint8_t EpNo, uint8_t *pBuffer, uint16_t Capacity)
{
	if (DevNo != 0 || EpNo == 0U || EpNo >= SAMD_USB_EP_COUNT || pBuffer == nullptr ||
		((uintptr_t)pBuffer & 3U) != 0U)
	{
		return false;
	}

	const uint32_t state = DisableInterrupt();
	SamdUsbEp_t *ep = &s_SamdUsb.Ep[EpNo][SAMD_USB_OUT];

	if (!s_SamdUsb.Started || s_SamdUsb.Closing || ep->Mps == 0U || ep->Busy ||
		Capacity < SamdUsbRxCapacity(ep->Mps))
	{
		EnableInterrupt(state);

		return false;
	}

	// One packet: MULTI_PACKET_SIZE is a multiple of the bank size, a packet
	// of Mps bytes below the bank size is a short packet
	s_SamdUsbDesc[EpNo][SAMD_USB_OUT].USB_ADDR = (uint32_t)(uintptr_t)pBuffer;
	s_SamdUsbDesc[EpNo][SAMD_USB_OUT].USB_PCKSIZE = USB_DEVICE_PCKSIZE_SIZE(ep->Size) |
		USB_DEVICE_PCKSIZE_MULTI_PACKET_SIZE(SamdUsbBankSize(ep->Size));
	s_SamdUsbDesc[EpNo][SAMD_USB_OUT].USB_STATUS_BK = 0U;
	ep->Busy = true;
	ep->RxReady = false;
	__DMB();

	usb_device_endpoint_registers_t *reg = SamdUsbEpReg(EpNo);

	reg->USB_EPINTFLAG = USB_DEVICE_EPINTFLAG_TRCPT0_Msk | USB_DEVICE_EPINTFLAG_TRFAIL0_Msk;
	reg->USB_EPSTATUSCLR = USB_DEVICE_EPSTATUSCLR_BK0RDY_Msk;

	EnableInterrupt(state);

	return true;
}

bool UsbCtrlrEpSend(int DevNo, uint8_t EpNum, uint8_t *pBuffer, uint16_t Length)
{
	if (DevNo != 0 || EpNum == 0U || EpNum >= SAMD_USB_EP_COUNT || (Length != 0U && pBuffer == nullptr))
	{
		return false;
	}

	const uint32_t state = DisableInterrupt();
	SamdUsbEp_t *ep = &s_SamdUsb.Ep[EpNum][SAMD_USB_IN];
	usb_device_endpoint_registers_t *reg = SamdUsbEpReg(EpNum);

	if (!s_SamdUsb.Started || s_SamdUsb.Closing || ep->Mps == 0U || ep->Busy || Length > ep->Mps ||
		(reg->USB_EPSTATUS & USB_DEVICE_EPSTATUS_BK1RDY_Msk) != 0U)
	{
		EnableInterrupt(state);

		return false;
	}

	// A halted endpoint keeps the packet until the host clears the halt
	uintptr_t addr = (uintptr_t)(pBuffer != nullptr ? pBuffer : s_SamdUsbInWord[EpNum]);
	const uint32_t misalign = (uint32_t)(addr & 3U);

	if (Length != 0U && misalign != 0U)
	{
		// The bank address is word aligned: the bytes up to the next word
		// boundary go first, from a copy. The owner advances by the length
		// of the completion, its next source is aligned.
		const uint16_t lead = (uint16_t)(4U - misalign);

		if (Length > lead)
		{
			Length = lead;
		}
		memcpy(s_SamdUsbInWord[EpNum], pBuffer, Length);
		addr = (uintptr_t)s_SamdUsbInWord[EpNum];
	}

	s_SamdUsbDesc[EpNum][SAMD_USB_IN].USB_ADDR = (uint32_t)addr;
	s_SamdUsbDesc[EpNum][SAMD_USB_IN].USB_PCKSIZE = USB_DEVICE_PCKSIZE_SIZE(ep->Size) |
		USB_DEVICE_PCKSIZE_BYTE_COUNT(Length);
	s_SamdUsbDesc[EpNum][SAMD_USB_IN].USB_STATUS_BK = 0U;
	ep->Length = Length;
	ep->Busy = true;
	__DMB();

	reg->USB_EPINTFLAG = USB_DEVICE_EPINTFLAG_TRCPT1_Msk | USB_DEVICE_EPINTFLAG_TRFAIL1_Msk;
	reg->USB_EPSTATUSSET = USB_DEVICE_EPSTATUSSET_BK1RDY_Msk;

	EnableInterrupt(state);

	return true;
}

void UsbCtrlrEpProcessEvent(int DevNo, uint8_t EpNo, bool bIn,
							UsbCtrlrEvtType_t Event, uint16_t Value)
{
	if (DevNo != 0 || !s_SamdUsb.Started || EpNo == 0U || EpNo >= SAMD_USB_EP_COUNT)
	{
		return;
	}

	const uint8_t dir = bIn ? SAMD_USB_IN : SAMD_USB_OUT;

	if (s_SamdUsb.Ep[EpNo][dir].Mps != 0U)
	{
		SamdUsbNotify(EpNo, dir, Event, Value);
	}
}

int UsbCtrlrEp0Send(int DevNo, uint8_t *pBuffer, int Length)
{
	if (DevNo != 0 || Length < 0 || (Length != 0 && pBuffer == nullptr))
	{
		return -1;
	}

	const uint32_t state = DisableInterrupt();
	usb_device_endpoint_registers_t *reg = SamdUsbEpReg(0);

	// A new SETUP supersedes this IN, it stays pending for the interrupt
	if (!s_SamdUsb.Started || s_SamdUsb.Closing || s_SamdUsb.Ep0Busy || s_SamdUsb.Ep0Halted ||
		SamdUsbSetupPending() || (reg->USB_EPSTATUS & USB_DEVICE_EPSTATUS_BK1RDY_Msk) != 0U)
	{
		EnableInterrupt(state);

		return -1;
	}

	const uint16_t length = Length < SAMD_USB_EP0_MPS ? (uint16_t)Length : (uint16_t)SAMD_USB_EP0_MPS;

	if (length != 0U)
	{
		memcpy(s_SamdUsbEp0In, pBuffer, length);
	}

	// A SETUP during the copy has superseded this IN
	if (SamdUsbSetupPending())
	{
		EnableInterrupt(state);

		return -1;
	}

	s_SamdUsbDesc[0][SAMD_USB_IN].USB_PCKSIZE = USB_DEVICE_PCKSIZE_SIZE(SAMD_USB_SIZE_64) |
		USB_DEVICE_PCKSIZE_BYTE_COUNT(length);
	s_SamdUsb.Ep0Length = length;
	s_SamdUsb.Ep0Busy = true;
	s_SamdUsb.Ep0StatusOut = false;
	__DMB();

	reg->USB_EPINTFLAG = USB_DEVICE_EPINTFLAG_TRCPT1_Msk;
	reg->USB_EPSTATUSSET = USB_DEVICE_EPSTATUSSET_BK1RDY_Msk;

	// A SETUP just before BK1RDY was set has had its BK1RDY clear undone:
	// the stale packet is taken back
	if (SamdUsbSetupPending())
	{
		reg->USB_EPSTATUSCLR = USB_DEVICE_EPSTATUSCLR_BK1RDY_Msk;
		s_SamdUsb.Ep0Busy = false;
		EnableInterrupt(state);

		return -1;
	}

	EnableInterrupt(state);

	return length;
}

bool UsbCtrlrEp0Status(int DevNo, uint8_t EpAddr)
{
	if (EpAddr == USB_ENDPADDR_DIR_IN)
	{
		return UsbCtrlrEp0Send(DevNo, nullptr, 0) == 0;
	}

	if (DevNo != 0 || EpAddr != 0U || !s_SamdUsb.Started || s_SamdUsb.Closing)
	{
		return false;
	}

	// The OUT bank is armed for the status OUT, completed from the interrupt
	const uint32_t state = DisableInterrupt();
	const bool accepted = !s_SamdUsb.Ep0Busy && !s_SamdUsb.Ep0Halted;

	if (accepted)
	{
		s_SamdUsb.Ep0StatusOut = true;
		if (!SamdUsbSetupPending())
		{
			SamdUsbEp0ArmOut();
		}
	}

	EnableInterrupt(state);

	return accepted;
}

void UsbCtrlrEpStall(int DevNo, uint8_t EpNo, bool bIn)
{
	if (DevNo != 0 || !s_SamdUsb.Started || s_SamdUsb.Closing || EpNo >= SAMD_USB_EP_COUNT)
	{
		return;
	}

	const uint32_t state = DisableInterrupt();
	usb_device_endpoint_registers_t *reg = SamdUsbEpReg(EpNo);

	if (EpNo == 0U)
	{
		// A new SETUP is never stalled on behalf of the previous request.
		// Both directions of the control endpoint stall.
		if (!SamdUsbSetupPending())
		{
			s_SamdUsb.Ep0Halted = true;
			s_SamdUsb.Ep0Busy = false;
			s_SamdUsb.AddressPending = false;
			s_SamdUsb.Ep0StatusOut = false;
			reg->USB_EPSTATUSCLR = USB_DEVICE_EPSTATUSCLR_BK1RDY_Msk;
			reg->USB_EPSTATUSSET = USB_DEVICE_EPSTATUSSET_STALLRQ0_Msk | USB_DEVICE_EPSTATUSSET_STALLRQ1_Msk;
		}
	}
	else if (s_SamdUsb.Ep[EpNo][bIn ? SAMD_USB_IN : SAMD_USB_OUT].Mps != 0U)
	{
		reg->USB_EPSTATUSSET = bIn ? USB_DEVICE_EPSTATUSSET_STALLRQ1_Msk : USB_DEVICE_EPSTATUSSET_STALLRQ0_Msk;
	}

	EnableInterrupt(state);
}

void UsbCtrlrEpClearStall(int DevNo, uint8_t EpNo, bool bIn)
{
	if (DevNo != 0 || !s_SamdUsb.Started || s_SamdUsb.Closing || EpNo >= SAMD_USB_EP_COUNT)
	{
		return;
	}

	const uint32_t state = DisableInterrupt();
	usb_device_endpoint_registers_t *reg = SamdUsbEpReg(EpNo);

	if (EpNo == 0U)
	{
		if (!SamdUsbSetupPending())
		{
			reg->USB_EPSTATUSCLR = USB_DEVICE_EPSTATUSCLR_STALLRQ0_Msk | USB_DEVICE_EPSTATUSCLR_STALLRQ1_Msk;
			reg->USB_EPINTFLAG = USB_DEVICE_EPINTFLAG_STALL0_Msk | USB_DEVICE_EPINTFLAG_STALL1_Msk;
			s_SamdUsb.Ep0Halted = false;
		}
	}
	else if (s_SamdUsb.Ep[EpNo][bIn ? SAMD_USB_IN : SAMD_USB_OUT].Mps != 0U)
	{
		// The data toggle restarts at DATA0 with the halt cleared
		if (bIn)
		{
			reg->USB_EPSTATUSCLR = USB_DEVICE_EPSTATUSCLR_STALLRQ1_Msk | USB_DEVICE_EPSTATUSCLR_DTGLIN_Msk;
			reg->USB_EPINTFLAG = USB_DEVICE_EPINTFLAG_STALL1_Msk;
		}
		else
		{
			reg->USB_EPSTATUSCLR = USB_DEVICE_EPSTATUSCLR_STALLRQ0_Msk | USB_DEVICE_EPSTATUSCLR_DTGLOUT_Msk;
			reg->USB_EPINTFLAG = USB_DEVICE_EPINTFLAG_STALL0_Msk;
		}
	}

	EnableInterrupt(state);
}

// EP0 IN acknowledged by the host. The address of SET_ADDRESS is in effect
// once its status stage is sent.
static void SamdUsbEp0InDone(void)
{
	const uint16_t txLength = s_SamdUsb.Ep0Length;

	s_SamdUsb.Ep0Busy = false;

	if (s_SamdUsb.AddressPending && txLength == 0U)
	{
		// The bus is settled once the device is addressed: suspend detection on
		SAMD_USB->USB_DADD = USB_DEVICE_DADD_DADD(s_SamdUsb.PendingAddress) | USB_DEVICE_DADD_ADDEN_Msk;
		s_SamdUsb.AddressPending = false;
		SAMD_USB->USB_INTFLAG = USB_DEVICE_INTFLAG_SUSPEND_Msk;
		SAMD_USB->USB_INTENSET = USB_DEVICE_INTENSET_SUSPEND_Msk;
	}

	SamdUsbEp0Result(true, txLength, nullptr, USB_CTRLR_XFER_SUCCESS);
}

static void SamdUsbEp0Interrupt(void)
{
	usb_device_endpoint_registers_t *reg = SamdUsbEpReg(0);
	const uint8_t flags = reg->USB_EPINTFLAG;

	if ((flags & USB_DEVICE_EPINTFLAG_RXSTP_Msk) != 0U)
	{
		UsbCtrlrEvt_t event = {};

		event.Type = USB_CTRLR_EVT_SETUP;
		memcpy(&event.Setup, s_SamdUsbEp0Out, sizeof(event.Setup));

		// The host may send this SETUP before the interrupt has seen the
		// status stage of the previous request: that completion goes to the
		// core first, the class completes the request (CDC line state) and a
		// SET_ADDRESS takes effect
		if ((flags & USB_DEVICE_EPINTFLAG_TRCPT1_Msk) != 0U && s_SamdUsb.Ep0Busy)
		{
			SamdUsbEp0InDone();
			if (!s_SamdUsb.Started)
			{
				return;
			}
		}
		if ((flags & USB_DEVICE_EPINTFLAG_TRCPT0_Msk) != 0U && s_SamdUsb.Ep0In && s_SamdUsb.Ep0StatusOut &&
			!s_SamdUsb.Ep0Halted)
		{
			s_SamdUsb.Ep0StatusOut = false;
			SamdUsbEp0Result(false, 0U, nullptr, USB_CTRLR_XFER_SUCCESS);
			if (!s_SamdUsb.Started)
			{
				return;
			}
		}

		s_SamdUsb.Ep0Busy = false;
		s_SamdUsb.Ep0Halted = false;
		s_SamdUsb.AddressPending = false;
		s_SamdUsb.Ep0StatusOut = false;
		s_SamdUsb.Ep0In = (event.Setup.bmRequestType & USB_ENDPADDR_DIR_IN) != 0U;

		// The SETUP has cleared BK1RDY and both stalls in hardware, the
		// previous request is over. RXSTP is cleared once the request is
		// copied: the next SETUP is written only while it is clear.
		reg->USB_EPSTATUSCLR = USB_DEVICE_EPSTATUSCLR_BK1RDY_Msk | USB_DEVICE_EPSTATUSCLR_STALLRQ0_Msk |
			USB_DEVICE_EPSTATUSCLR_STALLRQ1_Msk;
		reg->USB_EPINTFLAG = USB_DEVICE_EPINTFLAG_Msk;

		s_SamdUsb.Ep0OutLeft = s_SamdUsb.Ep0In ? 0U : event.Setup.wLength;
		if (s_SamdUsb.Ep0OutLeft != 0U)
		{
			SamdUsbEp0ArmOut();
		}

		UsbDevProcessEvent(0, &event);

		return;
	}

	// The host may have sent the status OUT by the time the IN completion is
	// seen. Its bytes are copied before the IN callback can arm a new stage.
	const bool out = (flags & USB_DEVICE_EPINTFLAG_TRCPT0_Msk) != 0U;
	const uint16_t rxLength = out ? (uint16_t)(s_SamdUsbDesc[0][SAMD_USB_OUT].USB_PCKSIZE &
										USB_DEVICE_PCKSIZE_BYTE_COUNT_Msk) : 0U;
	uint8_t data[SAMD_USB_EP0_MPS];

	if (out && rxLength != 0U && rxLength <= sizeof(data))
	{
		memcpy(data, s_SamdUsbEp0Out, rxLength);
	}

	if ((flags & USB_DEVICE_EPINTFLAG_TRCPT1_Msk) != 0U)
	{
		reg->USB_EPINTFLAG = USB_DEVICE_EPINTFLAG_TRCPT1_Msk;

		if (s_SamdUsb.Ep0Busy)
		{
			SamdUsbEp0InDone();

			if (!s_SamdUsb.Started || SamdUsbSetupPending())
			{
				return;
			}
		}
	}

	if (out)
	{
		if (!s_SamdUsb.Started || SamdUsbSetupPending())
		{
			return;
		}

		// An IN still queued is superseded by the OUT
		reg->USB_EPSTATUSCLR = USB_DEVICE_EPSTATUSCLR_BK1RDY_Msk;
		s_SamdUsb.Ep0Busy = false;
		s_SamdUsb.AddressPending = false;

		const bool halted = s_SamdUsb.Ep0Halted;
		const bool valid = rxLength <= sizeof(data) &&
			(!s_SamdUsb.Ep0In || (s_SamdUsb.Ep0StatusOut && rxLength == 0U));

		reg->USB_EPINTFLAG = USB_DEVICE_EPINTFLAG_TRCPT0_Msk;

		// The bank stays full after a status OUT or the last data packet
		const uint16_t left = s_SamdUsb.Ep0OutLeft;

		s_SamdUsb.Ep0OutLeft = (!s_SamdUsb.Ep0In && valid && rxLength == SAMD_USB_EP0_MPS && left > rxLength) ?
			(uint16_t)(left - rxLength) : 0U;
		if (s_SamdUsb.Ep0OutLeft != 0U)
		{
			SamdUsbEp0ArmOut();
		}

		// A request the core has already stalled gets no completion
		if (!halted)
		{
			SamdUsbEp0Result(false, valid ? rxLength : 0U, valid ? data : nullptr,
							 valid ? USB_CTRLR_XFER_SUCCESS : USB_CTRLR_XFER_CANCELLED);
		}
	}
}

static void SamdUsbDataInterrupt(uint8_t EpNo)
{
	usb_device_endpoint_registers_t *reg = SamdUsbEpReg(EpNo);
	const uint8_t flags = reg->USB_EPINTFLAG & reg->USB_EPINTENSET;

	if ((flags & USB_DEVICE_EPINTFLAG_TRCPT1_Msk) != 0U)
	{
		SamdUsbEp_t *ep = &s_SamdUsb.Ep[EpNo][SAMD_USB_IN];

		reg->USB_EPINTFLAG = USB_DEVICE_EPINTFLAG_TRCPT1_Msk;

		if (ep->Busy)
		{
			const uint16_t length = ep->Length;

			ep->Busy = false;
			SamdUsbNotify(EpNo, SAMD_USB_IN, USB_CTRLR_EVT_XFER_CMPL, length);

			if (!s_SamdUsb.Started)
			{
				return;
			}
		}
	}

	if ((flags & USB_DEVICE_EPINTFLAG_TRCPT0_Msk) != 0U)
	{
		SamdUsbEp_t *ep = &s_SamdUsb.Ep[EpNo][SAMD_USB_OUT];

		reg->USB_EPINTFLAG = USB_DEVICE_EPINTFLAG_TRCPT0_Msk;

		// The bank stays full (BK0RDY) until the next destination is given
		if (ep->Busy)
		{
			const uint16_t length = (uint16_t)(s_SamdUsbDesc[EpNo][SAMD_USB_OUT].USB_PCKSIZE &
											   USB_DEVICE_PCKSIZE_BYTE_COUNT_Msk);
			const uint8_t bank = s_SamdUsbDesc[EpNo][SAMD_USB_OUT].USB_STATUS_BK;
			// ERRORFLOW on an isochronous bank is a later frame dropped on the
			// full bank, the data of this one is good
			const bool failed = length > ep->Mps ||
				(ep->Type == ISO && (bank & USB_DEVICE_STATUS_BK_CRCERR_Msk) != 0U);
			const uint32_t generation = ep->Generation;

			s_SamdUsbDesc[EpNo][SAMD_USB_OUT].USB_STATUS_BK = 0U;
			ep->Busy = false;
			SamdUsbNotify(EpNo, SAMD_USB_OUT, failed ? USB_CTRLR_EVT_XFER_FAILED : USB_CTRLR_EVT_XFER_CMPL,
						  failed ? 0U : length);

			// The completion may close or reopen the endpoint or stop the device
			if (s_SamdUsb.Started && ep->Generation == generation && ep->Mps != 0U && !ep->Busy)
			{
				SamdUsbNotify(EpNo, SAMD_USB_OUT, USB_CTRLR_EVT_DRDY, 0U);
			}
			if (!s_SamdUsb.Started)
			{
				return;
			}
		}
	}

	if ((flags & USB_DEVICE_EPINTFLAG_TRFAIL0_Msk) != 0U)
	{
		SamdUsbEp_t *ep = &s_SamdUsb.Ep[EpNo][SAMD_USB_OUT];
		const uint8_t bank = s_SamdUsbDesc[EpNo][SAMD_USB_OUT].USB_STATUS_BK;

		// Isochronous OUT only. An overrun on a full bank drops the frame and
		// leaves no buffer to report.
		reg->USB_EPINTFLAG = USB_DEVICE_EPINTFLAG_TRFAIL0_Msk;
		s_SamdUsbDesc[EpNo][SAMD_USB_OUT].USB_STATUS_BK = 0U;

		if (ep->Busy && (bank & USB_DEVICE_STATUS_BK_CRCERR_Msk) != 0U)
		{
			// The bank stops receiving before its buffer goes back to the owner
			const uint32_t generation = ep->Generation;

			reg->USB_EPSTATUSSET = USB_DEVICE_EPSTATUSSET_BK0RDY_Msk;
			ep->Busy = false;
			SamdUsbNotify(EpNo, SAMD_USB_OUT, USB_CTRLR_EVT_XFER_FAILED, 0U);
			if (s_SamdUsb.Started && ep->Generation == generation && ep->Mps != 0U && !ep->Busy)
			{
				SamdUsbNotify(EpNo, SAMD_USB_OUT, USB_CTRLR_EVT_DRDY, 0U);
			}
		}
	}
}

static void SamdUsbInterrupt(void)
{
	if (!s_SamdUsb.Started)
	{
		return;
	}

	uint16_t status = SAMD_USB->USB_INTFLAG & SAMD_USB->USB_INTENSET;

	if ((status & USB_DEVICE_INTFLAG_EORST_Msk) != 0U)
	{
		SAMD_USB->USB_INTFLAG = USB_DEVICE_INTFLAG_Msk;
		SAMD_USB->USB_INTENCLR = USB_DEVICE_INTENCLR_SUSPEND_Msk | SAMD_USB_RESUME_EVENTS;
		SAMD_USB->USB_DADD = 0U;
		s_SamdUsb.Suspended = false;

		UsbCtrlrEpCloseAll(0);
		if (!s_SamdUsb.Started)
		{
			return;
		}
		SamdUsbEp0Init();
		SamdUsbBusEvent(USB_CTRLR_EVT_RESET);

		// Completions from before the reset do not belong to the new bus state
		return;
	}

	if (s_SamdUsb.Suspended && (status & SAMD_USB_RESUME_EVENTS) != 0U)
	{
		SAMD_USB->USB_INTFLAG = SAMD_USB_RESUME_EVENTS | USB_DEVICE_INTFLAG_SUSPEND_Msk;
		SAMD_USB->USB_INTENCLR = SAMD_USB_RESUME_EVENTS;
		SAMD_USB->USB_INTENSET = USB_DEVICE_INTENSET_SUSPEND_Msk;
		s_SamdUsb.Suspended = false;
		SamdUsbBusEvent(USB_CTRLR_EVT_RESUME);
		if (!s_SamdUsb.Started)
		{
			return;
		}
	}

	if ((SamdUsbEpReg(0)->USB_EPINTFLAG & SamdUsbEpReg(0)->USB_EPINTENSET) != 0U)
	{
		SamdUsbEp0Interrupt();
		if (!s_SamdUsb.Started)
		{
			return;
		}
	}

	for (uint8_t ep = 1U; ep < SAMD_USB_EP_COUNT; ep++)
	{
		// Read again after each callback
		if ((SamdUsbEpReg(ep)->USB_EPINTFLAG & SamdUsbEpReg(ep)->USB_EPINTENSET) != 0U)
		{
			SamdUsbDataInterrupt(ep);
			if (!s_SamdUsb.Started)
			{
				return;
			}
		}

		SamdUsbEp_t *out = &s_SamdUsb.Ep[ep][SAMD_USB_OUT];

		if (out->RxReady)
		{
			out->RxReady = false;
			if (out->Mps != 0U && !out->Busy)
			{
				SamdUsbNotify(ep, SAMD_USB_OUT, USB_CTRLR_EVT_DRDY, 0U);
			}
			if (!s_SamdUsb.Started)
			{
				return;
			}
		}
	}

	status = SAMD_USB->USB_INTFLAG & SAMD_USB->USB_INTENSET;

	if ((status & USB_DEVICE_INTFLAG_SOF_Msk) != 0U)
	{
		const uint16_t frame = SAMD_USB->USB_FNUM;

		SAMD_USB->USB_INTFLAG = USB_DEVICE_INTFLAG_SOF_Msk;
		if (s_SamdUsb.SofEnabled && !s_SamdUsb.Suspended && (frame & USB_DEVICE_FNUM_FNCERR_Msk) == 0U)
		{
			UsbCtrlrEvt_t event = {};

			event.Type = USB_CTRLR_EVT_SOF;
			event.FrameNo = (uint16_t)((frame & USB_DEVICE_FNUM_FNUM_Msk) >> USB_DEVICE_FNUM_FNUM_Pos);
			UsbDevProcessEvent(0, &event);
			if (!s_SamdUsb.Started)
			{
				return;
			}
		}
	}

	if (!s_SamdUsb.Suspended && (status & USB_DEVICE_INTFLAG_SUSPEND_Msk) != 0U)
	{
		// A WAKEUP from the traffic before the suspend is not a resume
		SAMD_USB->USB_INTFLAG = USB_DEVICE_INTFLAG_SUSPEND_Msk | SAMD_USB_RESUME_EVENTS;
		SAMD_USB->USB_INTENCLR = USB_DEVICE_INTENCLR_SUSPEND_Msk;
		SAMD_USB->USB_INTENSET = SAMD_USB_RESUME_EVENTS;
		s_SamdUsb.Suspended = true;
		SamdUsbBusEvent(USB_CTRLR_EVT_SUSPEND);
	}
}

// The port entry points are also called from other interrupts: as on SAM4L,
// the USB interrupt runs with the other interrupts disabled
extern "C" void USB_Handler(void)
{
	const uint32_t state = DisableInterrupt();
	SamdUsbInterrupt();
	__DSB();
	EnableInterrupt(state);
}

size_t UsbCtrlrGetSerial(int DevNo, char *pBuff, size_t BuffLen)
{
	if (DevNo != 0 || pBuff == nullptr || BuffLen == 0U)
	{
		return 0U;
	}

	const size_t count = BuffLen > SAMD_USB_SERIAL_LEN ? (size_t)SAMD_USB_SERIAL_LEN : BuffLen - 1U;

	for (size_t i = 0U; i < count; i++)
	{
		const uint32_t word = *(volatile const uint32_t *)s_SamdUsbSerialAddr[i >> 3];
		const uint32_t nibble = (word >> (28U - 4U * (i & 7U))) & 15U;

		pBuff[i] = (char)(nibble < 10U ? '0' + nibble : 'A' + nibble - 10U);
	}
	pBuff[count] = '\0';

	return count;
}
