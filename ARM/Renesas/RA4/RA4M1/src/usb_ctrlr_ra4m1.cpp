/**-------------------------------------------------------------------------
@file	usb_ctrlr_ra4m1.cpp

@brief	Native Renesas RA4M1 USBFS device controller.

The generic USB protocol and class code stays in src/usb. This file owns the
RA4M1 USBFS peripheral, ICU route, CFIFO and the nine programmable data pipes.
Transfers are interrupt driven and use CFIFO only; D0FIFO/D1FIFO remain free
for a later DMA optimization.

The application supplies board pin configuration through UsbCtrlrCfg_t. The
controller does not encode development-board pin assignments.

@author	Hoang Nguyen Hoan
@date	Oct. 25, 2026

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
#include <stdint.h>
#include <stddef.h>
#include <string.h>

#include "ra4m1xxx.h"
#include "interrupt_ra4m1.h"
#include "coredev/interrupt.h"
#include "coredev/iopincfg.h"
#include "usb/usb.h"
#include "usb_ctrlr.h"

#if defined(USB_DEBUG_TRACE) && USB_DEBUG_TRACE > 0
#include "syslog.h"
#define RA4USB_TRACE(...) SysLogPrintf(SysLogGet(), "[USBFS] " __VA_ARGS__)
#else
#define RA4USB_TRACE(...) ((void)0)
#endif

enum
{
	RA4USB_PIPE_COUNT = 9,
	RA4USB_EP0_MPS = 64,
	RA4USB_WAIT_COUNT = 4096,
	RA4USB_MSTP_MASK = 1U << 11,
	RA4USB_PIPE_MASK = 0x03FFU,

	RA4USB_PID_NAK = 0U,
	RA4USB_PID_BUF = 1U,
	RA4USB_PID_STALL = 2U,
	RA4USB_PID_MASK = 3U,
	RA4USB_PBUSY = 1U << 5,
	RA4USB_SQCLR = 1U << 8,
	RA4USB_ACLRM = 1U << 9,
	RA4USB_CSCLR = 1U << 13,

	RA4USB_CFIFO_CURPIPE_MASK = 0x000FU,
	RA4USB_CFIFO_ISEL = 1U << 5,
	RA4USB_CFIFO_FRDY = 1U << 13,
	RA4USB_CFIFO_BCLR = 1U << 14,
	RA4USB_CFIFO_BVAL = 1U << 15,
	RA4USB_CFIFO_DTLN_MASK = 0x0FFFU,

	RA4USB_INT_BRDY = 1U << 8,
	RA4USB_INT_NRDY = 1U << 9,
	RA4USB_INT_BEMP = 1U << 10,
	RA4USB_INT_CTRT = 1U << 11,
	RA4USB_INT_DVST = 1U << 12,
	RA4USB_INT_SOFR = 1U << 13,
	RA4USB_INT_RESM = 1U << 14,
	RA4USB_INT_VBINT = 1U << 15,
	RA4USB_INT_VALID = 1U << 3,
	RA4USB_INT_VBSTS = 1U << 7,

	RA4USB_CTSQ_MASK = 0x0007U,
	RA4USB_CTSQ_RDDS = 0x0001U,
	RA4USB_CTSQ_RDSS = 0x0002U,
	RA4USB_CTSQ_WRDS = 0x0003U,
	RA4USB_CTSQ_WRSS = 0x0004U,
	RA4USB_CTSQ_WRND = 0x0005U,
	RA4USB_CTSQ_SQER = 0x0006U,

	RA4USB_DVSQ_MASK = 0x0070U,
	RA4USB_DVSQ_DEFAULT = 0x0010U,
	RA4USB_DVSQ_SUSPEND = 0x0040U,

	RA4USB_PIPECFG_DIR_IN = 1U << 4,
	RA4USB_PIPECFG_SHTNAK = 1U << 7,
	RA4USB_PIPECFG_TYPE_BULK = 1U << 14,
	RA4USB_PIPECFG_TYPE_INT = 2U << 14,
	RA4USB_PIPECFG_TYPE_ISO = 3U << 14,

	RA4USB_SYSCFG_USBE = 1U << 0,
	RA4USB_SYSCFG_DPRPU = 1U << 4,
	RA4USB_SYSCFG_SCKE = 1U << 10,
	RA4USB_USBMC_VDCEN = 1U << 7,
	RA4USB_DCP_CCPL = 1U << 2,
};

typedef struct __Ra4UsbRegistration
{
	UsbCtrlrEpHandler_t Handler;
	void *pContext;
} Ra4UsbRegistration_t;

typedef struct __Ra4UsbPipe
{
	Ra4UsbRegistration_t *pReg;
	uint8_t EpNo;
	uint8_t Type;
	bool In;
	bool Busy;
	bool Halted;
	uint16_t Mps;
	uint8_t *pBuffer;
	uint16_t Length;
} Ra4UsbPipe_t;

typedef struct __Ra4UsbState
{
	Ra4UsbPipe_t Pipe[RA4USB_PIPE_COUNT + 1];
	Ra4UsbRegistration_t Reg[16][2];
	uint8_t EpToPipe[16][2];
	const IOPinCfg_t *pIOPinMap;
	int NbIOPins;
	IRQn_Type Irq;
	int IntPrio;
	bool Initialized;
	bool Started;
	bool Closing;
	bool Suspended;
	bool SofEnabled;
	bool Ep0Halted;
	bool Ep0InBusy;
	bool Ep0StatusPending;
	uint8_t Ep0StatusAddr;
	uint16_t Ep0InLength;
	uint16_t Ep0OutRemaining;
} Ra4UsbState_t;

static Ra4UsbState_t s_Usb;
alignas(4) static uint8_t s_Ep0In[RA4USB_EP0_MPS];
alignas(4) static uint8_t s_Ep0Out[RA4USB_EP0_MPS];
static_assert(sizeof(UsbSetupData_t) == 8U, "USB setup layout");

static inline uint16_t Ra4UsbPipeBit(uint8_t Pipe)
{
	return (uint16_t)(1U << Pipe);
}

static inline volatile uint16_t *Ra4UsbPipeCtr(uint8_t Pipe)
{
	return Pipe == 0U ? &R_USB_FS0->DCPCTR :
		&R_USB_FS0->PIPE_CTR[Pipe - 1U];
}

static bool Ra4UsbWaitClear(volatile uint16_t *pReg, uint16_t Mask)
{
	for (uint32_t i = 0; i < RA4USB_WAIT_COUNT; ++i)
	{
		if ((*pReg & Mask) == 0U)
		{
			return true;
		}
	}
	return false;
}

static bool Ra4UsbSetPid(uint8_t Pipe, uint16_t Pid)
{
	volatile uint16_t *pCtr = Ra4UsbPipeCtr(Pipe);
	uint16_t value = *pCtr;
	value = (uint16_t)((value & (uint16_t)~RA4USB_PID_MASK) |
		(Pid & RA4USB_PID_MASK));
	*pCtr = value;
	if (Pid == RA4USB_PID_NAK)
	{
		return Ra4UsbWaitClear(pCtr, RA4USB_PBUSY);
	}
	return true;
}

static void Ra4UsbClearPipeStatus(uint8_t Pipe)
{
	const uint16_t bit = Ra4UsbPipeBit(Pipe);
	R_USB_FS0->BRDYSTS = (uint16_t)((uint16_t)~bit & RA4USB_PIPE_MASK);
	R_USB_FS0->NRDYSTS = (uint16_t)((uint16_t)~bit & RA4USB_PIPE_MASK);
	R_USB_FS0->BEMPSTS = (uint16_t)((uint16_t)~bit & RA4USB_PIPE_MASK);
}

static void Ra4UsbClearPipeBuffer(uint8_t Pipe)
{
	if (Pipe == 0U || Pipe > RA4USB_PIPE_COUNT)
	{
		return;
	}
	if ((R_USB_FS0->CFIFOSEL & RA4USB_CFIFO_CURPIPE_MASK) == Pipe)
	{
		R_USB_FS0->CFIFOSEL = 0U;
		for (uint32_t i = 0U; i < RA4USB_WAIT_COUNT; ++i)
		{
			if ((R_USB_FS0->CFIFOSEL & RA4USB_CFIFO_CURPIPE_MASK) == 0U)
			{
				break;
			}
		}
	}
	volatile uint16_t *pCtr = Ra4UsbPipeCtr(Pipe);
	*pCtr |= RA4USB_ACLRM;
	*pCtr &= (uint16_t)~RA4USB_ACLRM;
	*pCtr |= RA4USB_CSCLR;
}

static bool Ra4UsbSelectFifo(uint8_t Pipe, bool In)
{
	uint16_t select = Pipe & RA4USB_CFIFO_CURPIPE_MASK;
	if (Pipe == 0U && In)
	{
		select |= RA4USB_CFIFO_ISEL;
	}
	R_USB_FS0->CFIFOSEL = select;
	for (uint32_t i = 0; i < RA4USB_WAIT_COUNT; ++i)
	{
		const uint16_t current = R_USB_FS0->CFIFOSEL;
		if ((current & (RA4USB_CFIFO_CURPIPE_MASK | RA4USB_CFIFO_ISEL)) ==
			select && (R_USB_FS0->CFIFOCTR & RA4USB_CFIFO_FRDY) != 0U)
		{
			return true;
		}
	}
	return false;
}

static bool Ra4UsbFifoWrite(uint8_t Pipe, const uint8_t *pData,
	uint16_t Length, uint16_t Mps)
{
	if (!Ra4UsbSetPid(Pipe, RA4USB_PID_NAK) ||
		!Ra4UsbSelectFifo(Pipe, true))
	{
		return false;
	}
	R_USB_FS0->CFIFOCTR = RA4USB_CFIFO_BCLR;
	for (uint16_t i = 0; i < Length; ++i)
	{
		R_USB_FS0->CFIFOLL = pData[i];
	}
	if (Length < Mps)
	{
		R_USB_FS0->CFIFOCTR = RA4USB_CFIFO_BVAL;
	}
	return true;
}

static int Ra4UsbFifoRead(uint8_t Pipe, uint8_t *pData, uint16_t Capacity)
{
	if (!Ra4UsbSelectFifo(Pipe, false))
	{
		return -1;
	}
	const uint16_t length =
		R_USB_FS0->CFIFOCTR & RA4USB_CFIFO_DTLN_MASK;
	const uint16_t copied = length < Capacity ? length : Capacity;
	for (uint16_t i = 0; i < copied; ++i)
	{
		pData[i] = R_USB_FS0->CFIFOLL;
	}
	for (uint16_t i = copied; i < length; ++i)
	{
		(void)R_USB_FS0->CFIFOLL;
	}
	R_USB_FS0->CFIFOCTR = RA4USB_CFIFO_BCLR;
	return length <= Capacity ? (int)length : -1;
}

static void Ra4UsbBusEvent(UsbCtrlrEvtType_t Type)
{
	UsbCtrlrEvt_t event = {};
	event.Type = Type;
	UsbDevProcessEvent(0, &event);
}

static void Ra4UsbControlComplete(uint8_t EpAddr, uint8_t *pBuffer,
	uint16_t Length, UsbCtrlrXferResult_t Result)
{
	UsbCtrlrEvt_t event = {};
	event.Type = USB_CTRLR_EVT_XFER_CMPL;
	event.Xfer.EpAddr = EpAddr;
	event.Xfer.pBuffer = pBuffer;
	event.Xfer.Length = Length;
	event.Xfer.Result = Result;
	UsbDevProcessEvent(0, &event);
}

static void Ra4UsbNotify(uint8_t Pipe, UsbCtrlrEvtType_t Event, uint16_t Length)
{
	Ra4UsbRegistration_t reg = {};
	const uint32_t state = DisableInterrupt();
	if (Pipe > 0U && Pipe <= RA4USB_PIPE_COUNT &&
		s_Usb.Pipe[Pipe].pReg != nullptr)
	{
		reg = *s_Usb.Pipe[Pipe].pReg;
	}
	EnableInterrupt(state);
	if (reg.Handler != nullptr)
	{
		reg.Handler(Event, Length, reg.pContext);
	}
}

static uint8_t Ra4UsbPhysical(uint8_t EpNo, bool In)
{
	return EpNo < 16U ? s_Usb.EpToPipe[EpNo][In ? 1U : 0U] : 0U;
}

static bool Ra4UsbArmOut(uint8_t Pipe)
{
	if (!Ra4UsbSetPid(Pipe, RA4USB_PID_NAK) ||
		!Ra4UsbSelectFifo(Pipe, false))
	{
		return false;
	}
	R_USB_FS0->CFIFOCTR = RA4USB_CFIFO_BCLR;
	const uint16_t bit = Ra4UsbPipeBit(Pipe);
	R_USB_FS0->BRDYSTS = (uint16_t)((uint16_t)~bit & RA4USB_PIPE_MASK);
	R_USB_FS0->NRDYSTS = (uint16_t)((uint16_t)~bit & RA4USB_PIPE_MASK);
	R_USB_FS0->BRDYENB |= bit;
	if (Pipe == 0U || s_Usb.Pipe[Pipe].Type != ISO)
	{
		R_USB_FS0->NRDYENB |= bit;
	}
	else
	{
		R_USB_FS0->NRDYENB &= (uint16_t)~bit;
	}
	return Ra4UsbSetPid(Pipe, RA4USB_PID_BUF);
}

static void Ra4UsbCancelPipe(uint8_t Pipe, bool Notify)
{
	if (Pipe == 0U || Pipe > RA4USB_PIPE_COUNT)
	{
		return;
	}
	Ra4UsbPipe_t *ep = &s_Usb.Pipe[Pipe];
	const bool busy = ep->Busy;
	if (s_Usb.Started)
	{
		const uint16_t bit = Ra4UsbPipeBit(Pipe);
		R_USB_FS0->BRDYENB &= (uint16_t)~bit;
		R_USB_FS0->NRDYENB &= (uint16_t)~bit;
		R_USB_FS0->BEMPENB &= (uint16_t)~bit;
		(void)Ra4UsbSetPid(Pipe, RA4USB_PID_NAK);
		Ra4UsbClearPipeBuffer(Pipe);
		Ra4UsbClearPipeStatus(Pipe);
	}
	ep->Busy = false;
	ep->pBuffer = nullptr;
	ep->Length = 0U;
	if (Notify && busy)
	{
		Ra4UsbNotify(Pipe, USB_CTRLR_EVT_CANCEL, 0U);
	}
}

static void Ra4UsbReleasePipe(uint8_t Pipe, bool Notify)
{
	if (Pipe == 0U || Pipe > RA4USB_PIPE_COUNT)
	{
		return;
	}
	Ra4UsbPipe_t old = s_Usb.Pipe[Pipe];
	Ra4UsbCancelPipe(Pipe, Notify);
	if (s_Usb.Started)
	{
		(void)Ra4UsbSetPid(Pipe, RA4USB_PID_NAK);
		R_USB_FS0->PIPESEL = Pipe;
		R_USB_FS0->PIPECFG = 0U;
		R_USB_FS0->PIPEMAXP = 0U;
		R_USB_FS0->PIPEPERI = 0U;
		R_USB_FS0->PIPESEL = 0U;
		*Ra4UsbPipeCtr(Pipe) = RA4USB_SQCLR;
	}
	if (old.EpNo < 16U)
	{
		s_Usb.EpToPipe[old.EpNo][old.In ? 1U : 0U] = 0U;
	}
	memset(&s_Usb.Pipe[Pipe], 0, sizeof(s_Usb.Pipe[Pipe]));
}

static void Ra4UsbReleaseAll(bool Notify)
{
	for (uint8_t pipe = 1U; pipe <= RA4USB_PIPE_COUNT; ++pipe)
	{
		if (s_Usb.Pipe[pipe].Mps != 0U)
		{
			Ra4UsbReleasePipe(pipe, Notify);
		}
	}
}

static void Ra4UsbEp0Reset(void)
{
	R_USB_FS0->BRDYENB &= (uint16_t)~1U;
	R_USB_FS0->NRDYENB &= (uint16_t)~1U;
	R_USB_FS0->BEMPENB &= (uint16_t)~1U;
	Ra4UsbClearPipeStatus(0U);
	R_USB_FS0->DCPCFG = 0U;
	R_USB_FS0->DCPMAXP = RA4USB_EP0_MPS;
	R_USB_FS0->DCPCTR = RA4USB_SQCLR;
	(void)Ra4UsbSetPid(0U, RA4USB_PID_BUF);
	s_Usb.Ep0Halted = false;
	s_Usb.Ep0InBusy = false;
	s_Usb.Ep0StatusPending = false;
	s_Usb.Ep0StatusAddr = 0U;
	s_Usb.Ep0InLength = 0U;
	s_Usb.Ep0OutRemaining = 0U;
}

static void Ra4UsbReset(void)
{
	s_Usb.Closing = true;
	Ra4UsbReleaseAll(false);
	s_Usb.Closing = false;
	Ra4UsbEp0Reset();
	s_Usb.Suspended = false;
	Ra4UsbBusEvent(USB_CTRLR_EVT_RESET);
}

static void Ra4UsbHandleSetup(uint16_t Stage)
{
	if ((Stage != RA4USB_CTSQ_RDDS && Stage != RA4USB_CTSQ_WRDS &&
		Stage != RA4USB_CTSQ_WRND) ||
		(R_USB_FS0->INTSTS0 & RA4USB_INT_VALID) == 0U)
	{
		return;
	}

	R_USB_FS0->INTSTS0 = (uint16_t)~RA4USB_INT_VALID;
	const uint16_t request = R_USB_FS0->USBREQ;
	UsbCtrlrEvt_t event = {};
	event.Type = USB_CTRLR_EVT_SETUP;
	event.Setup.bmRequestType = (uint8_t)request;
	event.Setup.bRequest = (uint8_t)(request >> 8U);
	event.Setup.wValue = R_USB_FS0->USBVAL;
	event.Setup.wIndex = R_USB_FS0->USBINDX;
	event.Setup.wLength = R_USB_FS0->USBLENG;

	s_Usb.Ep0Halted = false;
	s_Usb.Ep0InBusy = false;
	s_Usb.Ep0StatusPending = false;
	s_Usb.Ep0InLength = 0U;
	s_Usb.Ep0OutRemaining =
		(event.Setup.bmRequestType & USB_ENDPADDR_DIR_IN) == 0U ?
		event.Setup.wLength : 0U;

	UsbDevProcessEvent(0, &event);

	if (!s_Usb.Ep0Halted && s_Usb.Ep0OutRemaining != 0U)
	{
		(void)Ra4UsbArmOut(0U);
	}
}

static void Ra4UsbHandleControlStage(uint16_t Status)
{
	const uint16_t stage = Status & RA4USB_CTSQ_MASK;
	if (stage == RA4USB_CTSQ_SQER)
	{
		R_USB_FS0->INTSTS0 = (uint16_t)~RA4USB_INT_VALID;
		s_Usb.Ep0StatusPending = false;
		s_Usb.Ep0InBusy = false;
		return;
	}

	Ra4UsbHandleSetup(stage);

	if (!s_Usb.Ep0StatusPending)
	{
		return;
	}

	const bool statusOut =
		s_Usb.Ep0StatusAddr == USB_ENDPADDR_DIR_OUT &&
		stage == RA4USB_CTSQ_RDSS;
	const bool statusIn =
		s_Usb.Ep0StatusAddr == USB_ENDPADDR_DIR_IN &&
		(stage == RA4USB_CTSQ_WRSS || stage == RA4USB_CTSQ_WRND);
	if (!statusOut && !statusIn)
	{
		return;
	}

	s_Usb.Ep0StatusPending = false;
	R_USB_FS0->DCPCTR |= RA4USB_DCP_CCPL;
	Ra4UsbControlComplete(s_Usb.Ep0StatusAddr, nullptr, 0U,
		USB_CTRLR_XFER_SUCCESS);
}

static void Ra4UsbHandleEp0Out(void)
{
	R_USB_FS0->BRDYENB &= (uint16_t)~1U;
	const uint16_t capacity =
		s_Usb.Ep0OutRemaining < (uint16_t)RA4USB_EP0_MPS ?
		s_Usb.Ep0OutRemaining : (uint16_t)RA4USB_EP0_MPS;
	const int length = Ra4UsbFifoRead(0U, s_Ep0Out, capacity);
	if (length < 0)
	{
		s_Usb.Ep0OutRemaining = 0U;
		(void)Ra4UsbSetPid(0U, RA4USB_PID_STALL);
		Ra4UsbControlComplete(USB_ENDPADDR_DIR_OUT, s_Ep0Out, 0U,
			USB_CTRLR_XFER_FAILED);
		return;
	}

	if ((uint16_t)length <= s_Usb.Ep0OutRemaining)
	{
		s_Usb.Ep0OutRemaining =
			(uint16_t)(s_Usb.Ep0OutRemaining - (uint16_t)length);
	}
	const bool more = s_Usb.Ep0OutRemaining != 0U &&
		(uint16_t)length == RA4USB_EP0_MPS;
	(void)Ra4UsbSetPid(0U, RA4USB_PID_NAK);
	R_USB_FS0->NRDYENB &= (uint16_t)~1U;

	// The generic core consumes pBuffer synchronously. Keep EP0 NAKed until
	// that callback returns so USBFS cannot overwrite this controller buffer.
	Ra4UsbControlComplete(USB_ENDPADDR_DIR_OUT, s_Ep0Out,
		(uint16_t)length, USB_CTRLR_XFER_SUCCESS);
	if (more && !s_Usb.Ep0Halted)
	{
		(void)Ra4UsbArmOut(0U);
	}
	else
	{
		s_Usb.Ep0OutRemaining = 0U;
	}
}

static void Ra4UsbHandleEp0In(void)
{
	if (!s_Usb.Ep0InBusy)
	{
		R_USB_FS0->BEMPENB &= (uint16_t)~1U;
		return;
	}
	R_USB_FS0->BEMPENB &= (uint16_t)~1U;
	s_Usb.Ep0InBusy = false;
	const uint16_t length = s_Usb.Ep0InLength;
	s_Usb.Ep0InLength = 0U;
	Ra4UsbControlComplete(USB_ENDPADDR_DIR_IN, s_Ep0In, length,
		USB_CTRLR_XFER_SUCCESS);
}

static void Ra4UsbHandleOut(uint8_t Pipe)
{
	Ra4UsbPipe_t *ep = &s_Usb.Pipe[Pipe];
	const uint16_t bit = Ra4UsbPipeBit(Pipe);
	R_USB_FS0->BRDYENB &= (uint16_t)~bit;
	R_USB_FS0->NRDYENB &= (uint16_t)~bit;
	if (!ep->Busy || ep->In || ep->pBuffer == nullptr)
	{
		(void)Ra4UsbSetPid(Pipe, RA4USB_PID_NAK);
		return;
	}

	const int length = Ra4UsbFifoRead(Pipe, ep->pBuffer, ep->Length);
	(void)Ra4UsbSetPid(Pipe, RA4USB_PID_NAK);
	ep->Busy = false;
	ep->pBuffer = nullptr;
	ep->Length = 0U;
	if (ep->Type != ISO)
	{
		R_USB_FS0->NRDYENB |= bit;
	}
	if (length < 0)
	{
		Ra4UsbNotify(Pipe, USB_CTRLR_EVT_XFER_FAILED, 0U);
		return;
	}
	Ra4UsbNotify(Pipe, USB_CTRLR_EVT_XFER_CMPL, (uint16_t)length);
}

static void Ra4UsbHandleIn(uint8_t Pipe)
{
	Ra4UsbPipe_t *ep = &s_Usb.Pipe[Pipe];
	const uint16_t bit = Ra4UsbPipeBit(Pipe);
	R_USB_FS0->BEMPENB &= (uint16_t)~bit;
	R_USB_FS0->NRDYENB &= (uint16_t)~bit;
	if (!ep->Busy || !ep->In)
	{
		(void)Ra4UsbSetPid(Pipe, RA4USB_PID_NAK);
		return;
	}
	(void)Ra4UsbSetPid(Pipe, RA4USB_PID_NAK);
	const uint16_t length = ep->Length;
	ep->Busy = false;
	ep->pBuffer = nullptr;
	ep->Length = 0U;
	Ra4UsbNotify(Pipe, USB_CTRLR_EVT_XFER_CMPL, length);
}

static void Ra4UsbHandleBrdy(uint16_t Pipes)
{
	if ((Pipes & 1U) != 0U)
	{
		Ra4UsbHandleEp0Out();
		Pipes &= (uint16_t)~1U;
	}
	for (uint8_t pipe = 1U; pipe <= RA4USB_PIPE_COUNT; ++pipe)
	{
		if ((Pipes & Ra4UsbPipeBit(pipe)) != 0U)
		{
			Ra4UsbHandleOut(pipe);
		}
	}
}

static void Ra4UsbHandleBemp(uint16_t Pipes)
{
	if ((Pipes & 1U) != 0U)
	{
		Ra4UsbHandleEp0In();
		Pipes &= (uint16_t)~1U;
	}
	for (uint8_t pipe = 1U; pipe <= RA4USB_PIPE_COUNT; ++pipe)
	{
		if ((Pipes & Ra4UsbPipeBit(pipe)) != 0U)
		{
			Ra4UsbHandleIn(pipe);
		}
	}
}

static void Ra4UsbHandleNrdy(uint16_t Pipes)
{
	// An idle OUT endpoint is intentionally NAKed until its owner supplies a
	// destination. NRDY is the retry hint required by the generic USB ABI.
	for (uint8_t pipe = 1U; pipe <= RA4USB_PIPE_COUNT; ++pipe)
	{
		const uint16_t bit = Ra4UsbPipeBit(pipe);
		if ((Pipes & bit) == 0U)
		{
			continue;
		}
		Ra4UsbPipe_t *ep = &s_Usb.Pipe[pipe];
		if (!ep->In && !ep->Busy && !ep->Halted && ep->Mps != 0U)
		{
			Ra4UsbNotify(pipe, USB_CTRLR_EVT_DRDY, 0U);
		}
	}
}

static bool Ra4UsbHandleOneInterrupt(void)
{
	const uint16_t intStatus = R_USB_FS0->INTSTS0;
	const uint16_t intEnable = R_USB_FS0->INTENB0;
	const uint16_t enabled = intStatus & intEnable;

	if ((enabled & RA4USB_INT_RESM) != 0U)
	{
		R_USB_FS0->INTSTS0 = (uint16_t)~RA4USB_INT_RESM;
		if (s_Usb.Suspended)
		{
			s_Usb.Suspended = false;
			Ra4UsbBusEvent(USB_CTRLR_EVT_RESUME);
		}
		return true;
	}
	if ((enabled & RA4USB_INT_VBINT) != 0U)
	{
		R_USB_FS0->INTSTS0 = (uint16_t)~RA4USB_INT_VBINT;
		UsbProcessQue(0);
		return true;
	}
	if ((enabled & RA4USB_INT_SOFR) != 0U)
	{
		R_USB_FS0->INTSTS0 = (uint16_t)~RA4USB_INT_SOFR;
		UsbCtrlrEvt_t event = {};
		event.Type = USB_CTRLR_EVT_SOF;
		event.FrameNo = R_USB_FS0->FRMNUM & 0x07FFU;
		UsbDevProcessEvent(0, &event);
		return true;
	}
	if ((enabled & RA4USB_INT_DVST) != 0U)
	{
		R_USB_FS0->INTSTS0 = (uint16_t)~RA4USB_INT_DVST;
		const uint16_t state = intStatus & RA4USB_DVSQ_MASK;
		if (state == RA4USB_DVSQ_DEFAULT)
		{
			Ra4UsbReset();
		}
		else if ((state & RA4USB_DVSQ_SUSPEND) != 0U && !s_Usb.Suspended)
		{
			s_Usb.Suspended = true;
			Ra4UsbBusEvent(USB_CTRLR_EVT_SUSPEND);
		}
		return true;
	}

	const uint16_t brdy =
		R_USB_FS0->BRDYSTS & R_USB_FS0->BRDYENB & RA4USB_PIPE_MASK;
	if ((enabled & RA4USB_INT_BRDY) != 0U && brdy != 0U)
	{
		R_USB_FS0->BRDYSTS =
			(uint16_t)((uint16_t)~brdy & RA4USB_PIPE_MASK);
		Ra4UsbHandleBrdy(brdy);
		return true;
	}

	const uint16_t bemp =
		R_USB_FS0->BEMPSTS & R_USB_FS0->BEMPENB & RA4USB_PIPE_MASK;
	if ((enabled & RA4USB_INT_BEMP) != 0U && bemp != 0U)
	{
		R_USB_FS0->BEMPSTS =
			(uint16_t)((uint16_t)~bemp & RA4USB_PIPE_MASK);
		Ra4UsbHandleBemp(bemp);
		return true;
	}

	const uint16_t nrdy =
		R_USB_FS0->NRDYSTS & R_USB_FS0->NRDYENB & RA4USB_PIPE_MASK;
	if ((enabled & RA4USB_INT_NRDY) != 0U && nrdy != 0U)
	{
		R_USB_FS0->NRDYSTS =
			(uint16_t)((uint16_t)~nrdy & RA4USB_PIPE_MASK);
		Ra4UsbHandleNrdy(nrdy);
		return true;
	}

	if ((enabled & RA4USB_INT_CTRT) != 0U)
	{
		const uint16_t status = R_USB_FS0->INTSTS0;
		R_USB_FS0->INTSTS0 = (uint16_t)~RA4USB_INT_CTRT;
		Ra4UsbHandleControlStage(status);
		return true;
	}
	return false;
}

static void Ra4UsbInterrupt(int IntNo, void *pContext)
{
	(void)IntNo;
	(void)pContext;
	for (uint32_t i = 0U; i < 32U && Ra4UsbHandleOneInterrupt(); ++i)
	{
	}
	(void)R_USB_FS0->INTSTS0;
	__DSB();
}

static bool Ra4UsbClockEnable(void)
{
	R_MSTP->MSTPCRB &= ~RA4USB_MSTP_MASK;
	(void)R_MSTP->MSTPCRB;
	R_USB_FS0->SYSCFG = RA4USB_SYSCFG_SCKE;
	for (uint32_t i = 0U; i < RA4USB_WAIT_COUNT; ++i)
	{
		if ((R_USB_FS0->SYSCFG & RA4USB_SYSCFG_SCKE) != 0U)
		{
			R_USB_FS0->SYSCFG =
				RA4USB_SYSCFG_SCKE | RA4USB_SYSCFG_USBE;
			R_USB_FS0->USBMC |= RA4USB_USBMC_VDCEN;
			return true;
		}
	}
	R_USB_FS0->SYSCFG = 0U;
	R_USB_FS0->USBMC &= (uint16_t)~RA4USB_USBMC_VDCEN;
	R_MSTP->MSTPCRB |= RA4USB_MSTP_MASK;
	return false;
}

static void Ra4UsbClockDisable(void)
{
	R_USB_FS0->SYSCFG = 0U;
	R_USB_FS0->USBMC &= (uint16_t)~RA4USB_USBMC_VDCEN;
	__DSB();
	R_MSTP->MSTPCRB |= RA4USB_MSTP_MASK;
	(void)R_MSTP->MSTPCRB;
}

static bool Ra4UsbPipeMpsValid(uint8_t Pipe, uint8_t Type, uint16_t Mps)
{
	if (Mps == 0U)
	{
		return false;
	}
	if (Type == ISO)
	{
		return Pipe >= 1U && Pipe <= 2U && Mps <= 256U;
	}
	if (Mps > 64U)
	{
		return false;
	}
	if (Type == BULK)
	{
		return Pipe >= 1U && Pipe <= 5U &&
			(Mps == 8U || Mps == 16U || Mps == 32U || Mps == 64U);
	}
	return Type == INT && Pipe >= 6U && Pipe <= 9U;
}

static uint8_t Ra4UsbAllocatePipe(uint8_t Type, uint16_t Mps)
{
	if (Type == BULK)
	{
		// Keep ISO-capable pipes 1 and 2 free while pipes 3..5 are available.
		static const uint8_t order[] = {3U, 4U, 5U, 1U, 2U};
		for (uint8_t pipe : order)
		{
			if (s_Usb.Pipe[pipe].Mps == 0U &&
				Ra4UsbPipeMpsValid(pipe, Type, Mps))
			{
				return pipe;
			}
		}
		return 0U;
	}

	const uint8_t first = Type == ISO ? 1U : 6U;
	const uint8_t last = Type == ISO ? 2U : 9U;
	for (uint8_t pipe = first; pipe <= last; ++pipe)
	{
		if (s_Usb.Pipe[pipe].Mps == 0U &&
			Ra4UsbPipeMpsValid(pipe, Type, Mps))
		{
			return pipe;
		}
	}
	return 0U;
}

bool UsbCtrlrInit(int DevNo, const UsbCtrlrCfg_t *pCfg)
{
	if (DevNo != 0 || pCfg == nullptr || pCfg->IntPrio < 0 ||
		pCfg->IntPrio >= (1 << __NVIC_PRIO_BITS) ||
		pCfg->bLowPowerSuspend ||
		(pCfg->pIOPinMap == nullptr && pCfg->NbIOPins != 0) ||
		(pCfg->pIOPinMap != nullptr && pCfg->NbIOPins <= 0))
	{
		return false;
	}

	const uint32_t state = DisableInterrupt();
	if (s_Usb.Started || s_Usb.Closing)
	{
		EnableInterrupt(state);
		return false;
	}
	if (s_Usb.Initialized && (int)s_Usb.Irq >= 0)
	{
		Ra4m1UnregisterIntHandler(s_Usb.Irq);
	}
	memset(&s_Usb, 0, sizeof(s_Usb));
	s_Usb.Irq = (IRQn_Type)-1;
	s_Usb.IntPrio = pCfg->IntPrio;
	s_Usb.pIOPinMap = pCfg->pIOPinMap;
	s_Usb.NbIOPins = pCfg->NbIOPins;
	if (pCfg->pIOPinMap != nullptr)
	{
		IOPinCfg(pCfg->pIOPinMap, pCfg->NbIOPins);
	}
	s_Usb.Irq = Ra4m1RegisterIntHandler(RA4M1_EVTID_USBFS_USBI,
		pCfg->IntPrio, Ra4UsbInterrupt, nullptr);
	if ((int)s_Usb.Irq < 0)
	{
		memset(&s_Usb, 0, sizeof(s_Usb));
		s_Usb.Irq = (IRQn_Type)-1;
		EnableInterrupt(state);
		return false;
	}
	NVIC_DisableIRQ(s_Usb.Irq);
	s_Usb.Initialized = true;
	EnableInterrupt(state);
	return true;
}

bool UsbCtrlrVbusDetected(int DevNo)
{
	if (DevNo != 0 || !s_Usb.Initialized)
	{
		return false;
	}
	if (s_Usb.Started)
	{
		return (R_USB_FS0->INTSTS0 & RA4USB_INT_VBSTS) != 0U;
	}
	// The generic USB stack checks VBUS before calling UsbCtrlrStart.
	// USBFS must therefore be released from module-stop before checking
	// INTSTS0.VBSTS. The deep-standby DVBSTS0 bit is not the normal
	// operating-mode cable-detect source.
	if (SystemUsbClockGet() != 48000000U)
	{
		return false;
	}
	if (!Ra4UsbClockEnable())
	{
		return false;
	}
	return (R_USB_FS0->INTSTS0 & RA4USB_INT_VBSTS) != 0U;
}

bool UsbCtrlrStart(int DevNo)
{
	if (DevNo != 0 || !s_Usb.Initialized || s_Usb.Closing)
	{
		return false;
	}
	if (s_Usb.Started)
	{
		return true;
	}
	if (SystemUsbClockGet() != 48000000U)
	{
		return false;
	}

	const uint32_t state = DisableInterrupt();
	NVIC_DisableIRQ(s_Usb.Irq);
	if (!Ra4UsbClockEnable())
	{
		EnableInterrupt(state);
		return false;
	}
	if ((R_USB_FS0->INTSTS0 & RA4USB_INT_VBSTS) == 0U)
	{
		Ra4UsbClockDisable();
		EnableInterrupt(state);
		return false;
	}

	R_USB_FS0->INTENB0 = 0U;
	R_USB_FS0->BRDYENB = 0U;
	R_USB_FS0->NRDYENB = 0U;
	R_USB_FS0->BEMPENB = 0U;
	R_USB_FS0->INTSTS0 = 0U;
	R_USB_FS0->BRDYSTS = 0U;
	R_USB_FS0->NRDYSTS = 0U;
	R_USB_FS0->BEMPSTS = 0U;
	R_USB_FS0->PIPESEL = 0U;
	R_USB_FS0->CFIFOSEL = 0U;
	R_USB_FS0->D0FIFOSEL = 0U;
	R_USB_FS0->D1FIFOSEL = 0U;
	memset(s_Usb.EpToPipe, 0, sizeof(s_Usb.EpToPipe));
	memset(s_Usb.Pipe, 0, sizeof(s_Usb.Pipe));
	s_Usb.Suspended = false;
	s_Usb.SofEnabled = false;
	s_Usb.Closing = false;
	s_Usb.Started = true;
	Ra4UsbEp0Reset();

	R_USB_FS0->INTSTS0 = 0U;
	R_USB_FS0->BRDYSTS = 0U;
	R_USB_FS0->NRDYSTS = 0U;
	R_USB_FS0->BEMPSTS = 0U;
	R_USB_FS0->INTENB0 = RA4USB_INT_BRDY | RA4USB_INT_NRDY |
		RA4USB_INT_BEMP | RA4USB_INT_CTRT | RA4USB_INT_DVST |
		RA4USB_INT_RESM | RA4USB_INT_VBINT;
	NVIC_ClearPendingIRQ(s_Usb.Irq);
	EnableInterrupt(state);
	RA4USB_TRACE("start USBCLK=%lu\r\n",
		static_cast<unsigned long>(SystemUsbClockGet()));
	return true;
}

void UsbCtrlrStop(int DevNo)
{
	if (DevNo != 0 || !s_Usb.Started || s_Usb.Closing)
	{
		return;
	}
	const uint32_t state = DisableInterrupt();
	NVIC_DisableIRQ(s_Usb.Irq);
	R_USB_FS0->SYSCFG &= (uint16_t)~RA4USB_SYSCFG_DPRPU;
	R_USB_FS0->INTENB0 = 0U;
	R_USB_FS0->BRDYENB = 0U;
	R_USB_FS0->NRDYENB = 0U;
	R_USB_FS0->BEMPENB = 0U;
	s_Usb.Closing = true;
	Ra4UsbReleaseAll(true);
	s_Usb.Closing = false;
	s_Usb.Started = false;
	s_Usb.Suspended = false;
	s_Usb.Ep0InBusy = false;
	s_Usb.Ep0StatusPending = false;
	Ra4UsbClockDisable();
	NVIC_ClearPendingIRQ(s_Usb.Irq);
	EnableInterrupt(state);
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
	if (DevNo == 0 && s_Usb.Initialized && (int)s_Usb.Irq >= 0)
	{
		NVIC_ClearPendingIRQ(s_Usb.Irq);
		NVIC_EnableIRQ(s_Usb.Irq);
	}
}

void UsbCtrlrIntDisable(int DevNo)
{
	if (DevNo == 0 && s_Usb.Initialized && (int)s_Usb.Irq >= 0)
	{
		NVIC_DisableIRQ(s_Usb.Irq);
	}
}

void UsbCtrlrConnect(int DevNo)
{
	if (DevNo == 0 && s_Usb.Started && UsbCtrlrVbusDetected(DevNo))
	{
		R_USB_FS0->SYSCFG |= RA4USB_SYSCFG_DPRPU;
	}
}

void UsbCtrlrDisconnect(int DevNo)
{
	if (DevNo == 0 && s_Usb.Started)
	{
		R_USB_FS0->SYSCFG &= (uint16_t)~RA4USB_SYSCFG_DPRPU;
	}
}

void UsbCtrlrRemoteWakeup(int DevNo)
{
	if (DevNo == 0 && s_Usb.Started && s_Usb.Suspended)
	{
		R_USB_FS0->DVSTCTR0_b.WKUP = 1U;
	}
}

void UsbCtrlrSofEnable(int DevNo, bool Enable)
{
	if (DevNo != 0 || !s_Usb.Started)
	{
		return;
	}
	const uint32_t state = DisableInterrupt();
	if (Enable)
	{
		R_USB_FS0->INTSTS0 = (uint16_t)~RA4USB_INT_SOFR;
		R_USB_FS0->INTENB0 |= RA4USB_INT_SOFR;
	}
	else
	{
		R_USB_FS0->INTENB0 &= (uint16_t)~RA4USB_INT_SOFR;
		R_USB_FS0->INTSTS0 = (uint16_t)~RA4USB_INT_SOFR;
	}
	s_Usb.SofEnabled = Enable;
	EnableInterrupt(state);
}

void UsbCtrlrSetAddress(int DevNo, uint8_t Address)
{
	(void)DevNo;
	(void)Address;
	// USBFS performs SET_ADDRESS in hardware. USBADDR is read-only in
	// peripheral mode; the generic core commits its pending address after
	// the status stage completes.
}

void UsbCtrlrEpBind(int DevNo, uint8_t EpNo, bool bIn, bool bBlocking,
	UsbCtrlrEpHandler_t Handler, void *pContext)
{
	(void)bBlocking;
	if (DevNo != 0 || EpNo == 0U || EpNo >= 16U)
	{
		return;
	}
	const uint32_t state = DisableInterrupt();
	s_Usb.Reg[EpNo][bIn ? 1U : 0U].Handler = Handler;
	s_Usb.Reg[EpNo][bIn ? 1U : 0U].pContext = pContext;
	const uint8_t pipe = Ra4UsbPhysical(EpNo, bIn);
	if (pipe != 0U)
	{
		s_Usb.Pipe[pipe].pReg = &s_Usb.Reg[EpNo][bIn ? 1U : 0U];
	}
	EnableInterrupt(state);
}

bool UsbCtrlrEpOpenData(int DevNo, uint8_t EpNo, bool bIn, uint8_t Type,
	uint16_t MaxPacketSize)
{
	if (DevNo != 0 || !s_Usb.Started || s_Usb.Closing ||
		EpNo == 0U || EpNo >= 16U ||
		(Type != BULK && Type != INT && Type != ISO) ||
		Ra4UsbPhysical(EpNo, bIn) != 0U)
	{
		return false;
	}

	const uint32_t state = DisableInterrupt();
	const uint8_t pipe = Ra4UsbAllocatePipe(Type, MaxPacketSize);
	if (pipe == 0U)
	{
		EnableInterrupt(state);
		return false;
	}
	if (!Ra4UsbSetPid(pipe, RA4USB_PID_NAK))
	{
		EnableInterrupt(state);
		return false;
	}

	uint16_t cfg = EpNo & 0x0FU;
	cfg |= Type == BULK ? RA4USB_PIPECFG_TYPE_BULK :
		Type == ISO ? RA4USB_PIPECFG_TYPE_ISO : RA4USB_PIPECFG_TYPE_INT;
	if (bIn)
	{
		cfg |= RA4USB_PIPECFG_DIR_IN;
	}
	else if (Type == BULK)
	{
		cfg |= RA4USB_PIPECFG_SHTNAK;
	}

	R_USB_FS0->PIPESEL = pipe;
	R_USB_FS0->PIPECFG = cfg;
	R_USB_FS0->PIPEMAXP = MaxPacketSize;
	R_USB_FS0->PIPEPERI = 0U;
	R_USB_FS0->PIPESEL = 0U;
	*Ra4UsbPipeCtr(pipe) = RA4USB_SQCLR;
	Ra4UsbClearPipeBuffer(pipe);
	Ra4UsbClearPipeStatus(pipe);

	Ra4UsbPipe_t *ep = &s_Usb.Pipe[pipe];
	ep->pReg = &s_Usb.Reg[EpNo][bIn ? 1U : 0U];
	ep->EpNo = EpNo;
	ep->Type = Type;
	ep->In = bIn;
	ep->Busy = false;
	ep->Halted = false;
	ep->Mps = MaxPacketSize;
	ep->pBuffer = nullptr;
	ep->Length = 0U;
	s_Usb.EpToPipe[EpNo][bIn ? 1U : 0U] = pipe;
	if (!bIn && Type != ISO)
	{
		R_USB_FS0->NRDYENB |= Ra4UsbPipeBit(pipe);
	}
	EnableInterrupt(state);
	return true;
}

bool UsbCtrlrEpOpen(int DevNo, const UsbEndPointDesc_t *pDesc)
{
	if (pDesc == nullptr)
	{
		return false;
	}
	const uint8_t type = pDesc->bmAttributes & USB_ENDPATT_TRANS_MASK;
	return UsbCtrlrEpOpenData(DevNo,
		USB_ENDPADDR_NUM(pDesc->bEndpointAddress),
		USB_ENDPADDR_IS_IN(pDesc->bEndpointAddress),
		type, pDesc->wMaxPacketSize & 0x07FFU);
}

void UsbCtrlrEpClose(int DevNo, uint8_t EpNo, bool bIn)
{
	if (DevNo != 0 || EpNo == 0U || EpNo >= 16U || s_Usb.Closing)
	{
		return;
	}
	const uint32_t state = DisableInterrupt();
	const uint8_t pipe = Ra4UsbPhysical(EpNo, bIn);
	if (pipe != 0U && s_Usb.Started)
	{
		s_Usb.Closing = true;
		Ra4UsbReleasePipe(pipe, true);
		s_Usb.Closing = false;
	}
	EnableInterrupt(state);
}

void UsbCtrlrEpCloseAll(int DevNo)
{
	if (DevNo != 0 || !s_Usb.Started || s_Usb.Closing)
	{
		return;
	}
	const uint32_t state = DisableInterrupt();
	s_Usb.Closing = true;
	Ra4UsbReleaseAll(true);
	s_Usb.Closing = false;
	EnableInterrupt(state);
}

bool UsbCtrlrEpReceive(int DevNo, uint8_t EpNo, uint8_t *pBuffer,
	uint16_t Capacity)
{
	if (DevNo != 0 || EpNo == 0U || pBuffer == nullptr)
	{
		return false;
	}
	const uint32_t state = DisableInterrupt();
	const uint8_t pipe = Ra4UsbPhysical(EpNo, false);
	if (!s_Usb.Started || s_Usb.Closing || pipe == 0U)
	{
		EnableInterrupt(state);
		return false;
	}
	Ra4UsbPipe_t *ep = &s_Usb.Pipe[pipe];
	if (ep->Busy || ep->Halted || ep->In || Capacity < ep->Mps)
	{
		EnableInterrupt(state);
		return false;
	}
	ep->pBuffer = pBuffer;
	ep->Length = Capacity;
	ep->Busy = true;
	if (!Ra4UsbArmOut(pipe))
	{
		ep->Busy = false;
		ep->pBuffer = nullptr;
		ep->Length = 0U;
		EnableInterrupt(state);
		return false;
	}
	EnableInterrupt(state);
	return true;
}

bool UsbCtrlrEpSend(int DevNo, uint8_t EpNo, uint8_t *pBuffer,
	uint16_t Length)
{
	if (DevNo != 0 || EpNo == 0U ||
		(Length != 0U && pBuffer == nullptr))
	{
		return false;
	}
	const uint32_t state = DisableInterrupt();
	const uint8_t pipe = Ra4UsbPhysical(EpNo, true);
	if (!s_Usb.Started || s_Usb.Closing || pipe == 0U)
	{
		EnableInterrupt(state);
		return false;
	}
	Ra4UsbPipe_t *ep = &s_Usb.Pipe[pipe];
	if (ep->Busy || ep->Halted || !ep->In || Length > ep->Mps)
	{
		EnableInterrupt(state);
		return false;
	}
	if (!Ra4UsbFifoWrite(pipe, pBuffer, Length, ep->Mps))
	{
		EnableInterrupt(state);
		return false;
	}
	ep->pBuffer = pBuffer;
	ep->Length = Length;
	ep->Busy = true;
	const uint16_t bit = Ra4UsbPipeBit(pipe);
	R_USB_FS0->BEMPSTS = (uint16_t)((uint16_t)~bit & RA4USB_PIPE_MASK);
	R_USB_FS0->NRDYSTS = (uint16_t)((uint16_t)~bit & RA4USB_PIPE_MASK);
	R_USB_FS0->BEMPENB |= bit;
	R_USB_FS0->NRDYENB |= bit;
	if (!Ra4UsbSetPid(pipe, RA4USB_PID_BUF))
	{
		R_USB_FS0->BEMPENB &= (uint16_t)~bit;
		R_USB_FS0->NRDYENB &= (uint16_t)~bit;
		ep->Busy = false;
		ep->pBuffer = nullptr;
		ep->Length = 0U;
		EnableInterrupt(state);
		return false;
	}
	EnableInterrupt(state);
	return true;
}

void UsbCtrlrEpProcessEvent(int DevNo, uint8_t EpNo, bool bIn,
	UsbCtrlrEvtType_t Event, uint16_t Value)
{
	if (DevNo != 0 || !s_Usb.Started)
	{
		return;
	}
	const uint8_t pipe = Ra4UsbPhysical(EpNo, bIn);
	if (pipe != 0U)
	{
		Ra4UsbNotify(pipe, Event, Value);
	}
}

int UsbCtrlrEp0Send(int DevNo, uint8_t *pBuffer, int Length)
{
	if (DevNo != 0 || Length < 0 ||
		(Length != 0 && pBuffer == nullptr))
	{
		return -1;
	}
	const uint32_t state = DisableInterrupt();
	if (!s_Usb.Started || s_Usb.Closing || s_Usb.Ep0Halted ||
		s_Usb.Ep0InBusy)
	{
		EnableInterrupt(state);
		return -1;
	}
	const uint16_t length = Length < RA4USB_EP0_MPS ?
		(uint16_t)Length : (uint16_t)RA4USB_EP0_MPS;
	if (length != 0U)
	{
		memcpy(s_Ep0In, pBuffer, length);
	}
	if (!Ra4UsbFifoWrite(0U, s_Ep0In, length, RA4USB_EP0_MPS))
	{
		EnableInterrupt(state);
		return -1;
	}
	s_Usb.Ep0InLength = length;
	s_Usb.Ep0InBusy = true;
	R_USB_FS0->BEMPSTS = (uint16_t)~1U;
	R_USB_FS0->BEMPENB |= 1U;
	if (!Ra4UsbSetPid(0U, RA4USB_PID_BUF))
	{
		R_USB_FS0->BEMPENB &= (uint16_t)~1U;
		s_Usb.Ep0InBusy = false;
		EnableInterrupt(state);
		return -1;
	}
	EnableInterrupt(state);
	return length;
}

bool UsbCtrlrEp0Status(int DevNo, uint8_t EpAddr)
{
	if (DevNo != 0 || !s_Usb.Started || s_Usb.Closing ||
		s_Usb.Ep0Halted ||
		(EpAddr != USB_ENDPADDR_DIR_IN &&
		 EpAddr != USB_ENDPADDR_DIR_OUT))
	{
		return false;
	}
	const uint32_t state = DisableInterrupt();
	if (s_Usb.Ep0StatusPending)
	{
		EnableInterrupt(state);
		return false;
	}
	s_Usb.Ep0StatusAddr = EpAddr;
	s_Usb.Ep0StatusPending = true;
	const uint16_t stage = R_USB_FS0->INTSTS0 & RA4USB_CTSQ_MASK;
	const bool ready =
		(EpAddr == USB_ENDPADDR_DIR_OUT && stage == RA4USB_CTSQ_RDSS) ||
		(EpAddr == USB_ENDPADDR_DIR_IN &&
		 (stage == RA4USB_CTSQ_WRSS || stage == RA4USB_CTSQ_WRND));
	if (ready)
	{
		s_Usb.Ep0StatusPending = false;
		R_USB_FS0->DCPCTR |= RA4USB_DCP_CCPL;
		EnableInterrupt(state);
		Ra4UsbControlComplete(EpAddr, nullptr, 0U,
			USB_CTRLR_XFER_SUCCESS);
		return true;
	}
	EnableInterrupt(state);
	return true;
}

void UsbCtrlrEpStall(int DevNo, uint8_t EpNo, bool bIn)
{
	if (DevNo != 0 || !s_Usb.Started || s_Usb.Closing)
	{
		return;
	}
	const uint32_t state = DisableInterrupt();
	if (EpNo == 0U)
	{
		s_Usb.Ep0Halted = true;
		s_Usb.Ep0InBusy = false;
		s_Usb.Ep0StatusPending = false;
		R_USB_FS0->BRDYENB &= (uint16_t)~1U;
		R_USB_FS0->BEMPENB &= (uint16_t)~1U;
		R_USB_FS0->NRDYENB &= (uint16_t)~1U;
		(void)Ra4UsbSetPid(0U, RA4USB_PID_NAK);
		(void)Ra4UsbSetPid(0U, RA4USB_PID_STALL);
		EnableInterrupt(state);
		return;
	}
	const uint8_t pipe = Ra4UsbPhysical(EpNo, bIn);
	if (pipe != 0U)
	{
		s_Usb.Pipe[pipe].Halted = true;
		(void)Ra4UsbSetPid(pipe, RA4USB_PID_NAK);
		(void)Ra4UsbSetPid(pipe, RA4USB_PID_STALL);
	}
	EnableInterrupt(state);
}

void UsbCtrlrEpClearStall(int DevNo, uint8_t EpNo, bool bIn)
{
	if (DevNo != 0 || !s_Usb.Started || s_Usb.Closing)
	{
		return;
	}
	const uint32_t state = DisableInterrupt();
	if (EpNo == 0U)
	{
		(void)Ra4UsbSetPid(0U, RA4USB_PID_NAK);
		R_USB_FS0->DCPCTR |= RA4USB_SQCLR;
		s_Usb.Ep0Halted = false;
		(void)Ra4UsbSetPid(0U, RA4USB_PID_BUF);
		EnableInterrupt(state);
		return;
	}
	const uint8_t pipe = Ra4UsbPhysical(EpNo, bIn);
	if (pipe != 0U)
	{
		Ra4UsbPipe_t *ep = &s_Usb.Pipe[pipe];
		(void)Ra4UsbSetPid(pipe, RA4USB_PID_NAK);
		*Ra4UsbPipeCtr(pipe) |= RA4USB_SQCLR;
		ep->Halted = false;
		if (ep->Busy)
		{
			if (ep->In)
			{
				if (Ra4UsbFifoWrite(pipe, ep->pBuffer, ep->Length, ep->Mps))
				{
					R_USB_FS0->BEMPENB |= Ra4UsbPipeBit(pipe);
					(void)Ra4UsbSetPid(pipe, RA4USB_PID_BUF);
				}
			}
			else
			{
				(void)Ra4UsbArmOut(pipe);
			}
		}
		else if (!ep->In && ep->Type != ISO)
		{
			R_USB_FS0->NRDYENB |= Ra4UsbPipeBit(pipe);
		}
	}
	EnableInterrupt(state);
}

size_t UsbCtrlrGetSerial(int DevNo, char *pSerial, size_t MaxLen)
{
	if (DevNo != 0 || pSerial == nullptr || MaxLen == 0U)
	{
		return 0U;
	}
	static const char hex[] = "0123456789ABCDEF";
	const uintptr_t info = *(const uint32_t *)0x407FB19CUL;
	if (info == 0U || info == UINT32_MAX)
	{
		pSerial[0] = '\0';
		return 0U;
	}
	const uint8_t *uid = reinterpret_cast<const uint8_t *>(info + 0x14U);
	const size_t bytes = MaxLen > 33U ? 16U : (MaxLen - 1U) / 2U;
	size_t pos = 0U;
	for (size_t i = 0U; i < bytes && pos + 2U < MaxLen; ++i)
	{
		pSerial[pos++] = hex[uid[i] >> 4U];
		pSerial[pos++] = hex[uid[i] & 0x0FU];
	}
	pSerial[pos] = '\0';
	return pos;
}

bool UsbCtrlrIsoInit(int DevNo)
{
	return DevNo == 0 && s_Usb.Initialized;
}

bool UsbCtrlrIsoOpen(int DevNo, uint8_t EpNo, bool bIn,
	uint16_t MaxPacketSize)
{
	return UsbCtrlrEpOpenData(DevNo, EpNo, bIn, ISO, MaxPacketSize);
}

bool UsbCtrlrIsoSend(int DevNo, uint8_t EpNum, uint8_t *pBuffer,
	uint16_t Length)
{
	if (DevNo != 0 || EpNum == 0U || EpNum >= 16U || !s_Usb.Started)
	{
		return false;
	}

	// The generic ISO scheduler calls this once per service interval even
	// when no IN frame is queued. Service OUT first so it is armed before
	// the host's isochronous OUT transaction in the current USB frame.
	const uint8_t outPipe = Ra4UsbPhysical(EpNum, false);
	if (outPipe != 0U && s_Usb.Pipe[outPipe].Type == ISO &&
		!s_Usb.Pipe[outPipe].Busy && !s_Usb.Pipe[outPipe].Halted)
	{
		Ra4UsbNotify(outPipe, USB_CTRLR_EVT_DRDY, 0U);
	}

	const uint8_t inPipe = Ra4UsbPhysical(EpNum, true);
	if (inPipe == 0U || s_Usb.Pipe[inPipe].Type != ISO)
	{
		return false;
	}
	return UsbCtrlrEpSend(DevNo, EpNum, pBuffer, Length);
}

uint16_t UsbCtrlrIsoTraceSnapshot(int DevNo, uint8_t **ppData)
{
	(void)DevNo;
	if (ppData != nullptr)
	{
		*ppData = nullptr;
	}
	return 0U;
}
