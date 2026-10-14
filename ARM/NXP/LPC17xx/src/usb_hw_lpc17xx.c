/**-------------------------------------------------------------------------
@file	usb_hw_lpc17xx.c

@brief	Private LPC17xx USB clocks, SIE, endpoint realization and CPU FIFOs.

@author	Hoang Nguyen Hoan
@date	Oct. 12, 2026

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
#include "LPC17xx.h"
#include "usb_hw_lpc17xx.h"

#define LPC17_USB_POWER (1UL << 31)
#define LPC17_USB_CLOCKS 0x12U
#define LPC17_USB_WAIT 100000
#define LPC17_USB_CCEMPTY (1U << 4)
#define LPC17_USB_CDFULL (1U << 5)

static bool s_Prepared;
static bool s_SieFault;
static bool s_Ep0Ready;

static bool Lpc17UsbClockValid(uint32_t MainHz)
{
	if (MainHz < 1000000U || MainHz > 25000000U ||
		(LPC_SC->SCS & 0x60U) != 0x60U || SystemCoreClock < 18000000U)
		return false;
	const uint32_t pll1 = LPC_SC->PLL1STAT;
	if ((pll1 & (1U << 9)) != 0U)
	{
		if ((pll1 & (7U << 8)) != (7U << 8) || MainHz < 10000000U)
			return false;
		const uint64_t output = (uint64_t)MainHz * ((pll1 & 31U) + 1U);
		const uint64_t cco = output * 2U * (1U << ((pll1 >> 5) & 3U));
		return output == 48000000U && cco >= 156000000U && cco <= 320000000U;
	}
	const uint32_t pll0 = LPC_SC->PLL0STAT;
	const uint32_t divider = (LPC_SC->USBCLKCFG & 15U) + 1U;
	if ((LPC_SC->CLKSRCSEL & 3U) != 1U ||
		(pll0 & (5U << 24)) != (5U << 24) || (divider & 1U) != 0U)
		return false;
	const uint64_t numerator = (uint64_t)MainHz * 2U * ((pll0 & 0x7FFFU) + 1U);
	const uint32_t denominator = ((pll0 >> 16) & 0xFFU) + 1U;
	return numerator == (uint64_t)48000000U * denominator * divider &&
		numerator >= (uint64_t)275000000U * denominator &&
		numerator <= (uint64_t)550000000U * denominator;
}

bool Lpc17UsbHwPrepare(uint32_t MainHz)
{
	const uint32_t state = __get_PRIMASK();
	__disable_irq();
	bool ready = false;
	if (!s_Prepared && (LPC_SC->PCONP & LPC17_USB_POWER) == 0U &&
		Lpc17UsbClockValid(MainHz))
	{
		LPC_SC->PCONP |= LPC17_USB_POWER;
		LPC_USB->USBClkCtrl = LPC17_USB_CLOCKS;
		int wait = LPC17_USB_WAIT;
		while ((LPC_USB->USBClkSt & LPC17_USB_CLOCKS) != LPC17_USB_CLOCKS && --wait > 0)
		{
		}
		ready = wait > 0;
		if (ready)
		{
			s_Prepared = true;
			s_SieFault = false;
			s_Ep0Ready = false;
		}
		else
		{
			LPC_USB->USBClkCtrl = 0U;
			LPC_SC->PCONP &= ~LPC17_USB_POWER;
		}
	}
	__set_PRIMASK(state);
	return ready;
}

bool Lpc17UsbHwRelease(void)
{
	const uint32_t state = __get_PRIMASK();
	__disable_irq();
	bool released = !s_Prepared;
	if (s_Prepared && LPC_USB->USBReEp == 0U && LPC_USB->USBEpDMASt == 0U &&
		LPC_USB->USBDevIntEn == 0U && LPC_USB->USBEpIntEn == 0U &&
		LPC_USB->USBDMAIntEn == 0U)
	{
		LPC_USB->USBClkCtrl = 0U;
		LPC_SC->PCONP &= ~LPC17_USB_POWER;
		s_Prepared = false;
		s_Ep0Ready = false;
		released = true;
	}
	__set_PRIMASK(state);
	return released;
}

// Caller holds PRIMASK for the entire command/data sequence. These completion
// flags belong to this polling path and must not be consumed by the USB ISR.
static bool Lpc17UsbSieWait(uint32_t Flag)
{
	int wait = LPC17_USB_WAIT;
	while ((LPC_USB->USBDevIntSt & Flag) == 0U && --wait > 0)
	{
	}
	if (wait == 0)
	{
		s_SieFault = true;
		return false;
	}
	return true;
}

static bool Lpc17UsbSieTransfer(uint8_t Command, uint32_t Phase, uint8_t *pData)
{
	const uint32_t state = __get_PRIMASK();
	__disable_irq();
	bool done = false;
	if (s_Prepared && !s_SieFault &&
		(LPC_SC->PCONP & LPC17_USB_POWER) != 0U &&
		(LPC_USB->USBClkSt & LPC17_USB_CLOCKS) == LPC17_USB_CLOCKS)
	{
		// UM10360 11.12: command phase first, followed by optional data.
		// Clear only the command handshake flags, leaving bus/endpoint events.
		LPC_USB->USBDevIntClr = LPC17_USB_CCEMPTY | LPC17_USB_CDFULL;
		LPC_USB->USBCmdCode = ((uint32_t)Command << 16) | 0x500U;
		done = Lpc17UsbSieWait(LPC17_USB_CCEMPTY);
		if (done && Phase != 0U)
		{
			const uint32_t flag = Phase == 0x200U ? LPC17_USB_CDFULL : LPC17_USB_CCEMPTY;
			const uint8_t value = Phase == 0x200U ? Command : *pData;
			LPC_USB->USBDevIntClr = flag;
			LPC_USB->USBCmdCode = ((uint32_t)value << 16) | Phase;
			done = Lpc17UsbSieWait(flag);
			if (done && Phase == 0x200U)
				*pData = (uint8_t)LPC_USB->USBCmdData;
		}
	}
	__set_PRIMASK(state);
	return done;
}

bool Lpc17UsbSieCommand(uint8_t Command)
{
	return Lpc17UsbSieTransfer(Command, 0U, 0);
}

bool Lpc17UsbSieWrite(uint8_t Command, uint8_t Data)
{
	return Lpc17UsbSieTransfer(Command, 0x100U, &Data);
}

bool Lpc17UsbSieRead(uint8_t Command, uint8_t *pData)
{
	if (pData == 0)
		return false;
	return Lpc17UsbSieTransfer(Command, 0x200U, pData);
}

static bool Lpc17UsbPacketReady(void)
{
	return s_Prepared && !s_SieFault &&
		(LPC_SC->PCONP & LPC17_USB_POWER) != 0U &&
		(LPC_USB->USBClkSt & LPC17_USB_CLOCKS) == LPC17_USB_CLOCKS;
}

bool Lpc17UsbEp0Realize(void)
{
	const uint32_t state = __get_PRIMASK();
	__disable_irq();
	bool ready = false;
	// Realization reallocates endpoint RAM. Only use before enabling traffic.
	if (Lpc17UsbPacketReady() && !s_Ep0Ready &&
		(LPC_USB->USBReEp & ~3U) == 0U && LPC_USB->USBDevIntEn == 0U &&
		LPC_USB->USBEpIntEn == 0U && LPC_USB->USBEpDMASt == 0U)
	{
		ready = true;
		for (unsigned ep = 0; ep < 2U && ready; ++ep)
		{
			LPC_USB->USBDevIntClr = 1U << 8;
			LPC_USB->USBReEp |= 1U << ep;
			LPC_USB->USBEpInd = ep;
			LPC_USB->USBMaxPSize = 64U;
			ready = Lpc17UsbSieWait(1U << 8);
			LPC_USB->USBDevIntClr = 1U << 8;
		}
		s_Ep0Ready = ready;
	}
	__set_PRIMASK(state);
	return ready;
}

bool Lpc17UsbDataRealize(uint8_t EpNo, bool In, uint8_t Type, uint16_t Size)
{
	// UM10360 Table 185: EP15 is bulk, unlike the other multiples of three.
	if (EpNo == 0U || EpNo > 15U || Size == 0U || Size > 64U)
		return false;
	const bool bulk = EpNo % 3U == 2U || EpNo == 15U;
	const bool interrupt = EpNo % 3U == 1U;
	if ((Type != 2U || !bulk ||
		(Size != 8U && Size != 16U && Size != 32U && Size != 64U)) &&
		(Type != 3U || !interrupt))
		return false;
	const uint32_t physical = (uint32_t)EpNo * 2U + (In ? 1U : 0U);
	const uint32_t bit = 1UL << physical;
	const uint32_t state = __get_PRIMASK();
	__disable_irq();
	bool ready = false;
	// All supported bulk/interrupt directions at 64 bytes plus EP0 and the
	// pointer table use at most 2576 of 4096 EP_RAM bytes. Exclude ISO RAM.
	if (Lpc17UsbPacketReady() && s_Ep0Ready &&
		(LPC_USB->USBReEp & 3U) == 3U &&
		(LPC_USB->USBReEp & (~0xFCF3CF3FUL | bit)) == 0U &&
		LPC_USB->USBDevIntEn == 0U && LPC_USB->USBEpIntEn == 0U &&
		LPC_USB->USBEpDMASt == 0U && LPC_USB->USBDMAIntEn == 0U &&
		LPC_USB->USBCtrl == 0U)
	{
		LPC_USB->USBDevIntClr = 1U << 8;
		LPC_USB->USBReEp |= bit;
		LPC_USB->USBEpInd = physical;
		LPC_USB->USBMaxPSize = Size;
		ready = Lpc17UsbSieWait(1U << 8);
		LPC_USB->USBDevIntClr = 1U << 8;
	}
	__set_PRIMASK(state);
	return ready;
}

// Caller holds PRIMASK and validates the endpoint. SIE allows only one
// physical endpoint clear at a time, followed by consuming its status byte.
static bool Lpc17UsbEndpointAck(unsigned Physical, uint8_t *pStatus)
{
	LPC_USB->USBDevIntClr = LPC17_USB_CDFULL;
	LPC_USB->USBEpIntClr = 1UL << Physical;
	if (!Lpc17UsbSieWait(LPC17_USB_CDFULL))
		return false;
	*pStatus = (uint8_t)LPC_USB->USBCmdData;
	return true;
}

bool Lpc17UsbEp0Ack(bool In, uint8_t *pStatus)
{
	if (pStatus == 0)
		return false;
	const uint32_t state = __get_PRIMASK();
	__disable_irq();
	bool done = false;
	if (Lpc17UsbPacketReady() && s_Ep0Ready)
	{
		// UM10360 11.10.3.3: one endpoint per clear, then consume SIE status.
		// Also permits clearing OUT STP/PO during SETUP overwrite recovery.
		done = Lpc17UsbEndpointAck(In ? 1U : 0U, pStatus);
	}
	__set_PRIMASK(state);
	return done;
}

bool Lpc17UsbDataCollect(uint32_t *pMask, uint8_t Status[32])
{
	if (pMask == 0 || Status == 0)
		return false;
	const uint32_t state = __get_PRIMASK();
	__disable_irq();
	bool done = false;
	if (Lpc17UsbPacketReady() && s_Ep0Ready &&
		LPC_USB->USBEpDMASt == 0U && LPC_USB->USBDMAIntEn == 0U &&
		LPC_USB->USBCtrl == 0U && (LPC_USB->USBDevIntSt & 8U) == 0U)
	{
		const uint32_t enabled = LPC_USB->USBEpIntEn & ~3U;
		const uint32_t pending = LPC_USB->USBEpIntSt & enabled;
		uint8_t status[32] = {0};
		done = (enabled & ~0xFCF3CF3FUL) == 0U &&
			(enabled & ~LPC_USB->USBReEp) == 0U;
		for (unsigned physical = 2; physical < 32U && done; ++physical)
		{
			if ((pending & (1UL << physical)) != 0U)
				done = (LPC_USB->USBDevIntSt & 8U) == 0U &&
					Lpc17UsbEndpointAck(physical, &status[physical]);
		}
		done = done && (LPC_USB->USBDevIntSt & 8U) == 0U;
		if (done)
		{
			for (unsigned physical = 0; physical < 32U; ++physical)
				Status[physical] = status[physical];
			*pMask = pending;
		}
	}
	__set_PRIMASK(state);
	return done;
}

bool Lpc17UsbDeviceInit(void)
{
	const uint32_t state = __get_PRIMASK();
	__disable_irq();
	bool done = false;
	if (Lpc17UsbPacketReady() && !s_Ep0Ready &&
		(LPC_USB->USBReEp & ~3U) == 0U && LPC_USB->USBEpDMASt == 0U &&
		LPC_USB->USBDevIntEn == 0U && LPC_USB->USBEpIntEn == 0U &&
		LPC_USB->USBDMAIntEn == 0U)
	{
		uint8_t status;
		// UM10360 11.13: keep SoftConnect inactive until the ISR is ready.
		done = Lpc17UsbSieWrite(0xFEU, 0U) && Lpc17UsbEp0Realize() &&
			Lpc17UsbSieWrite(0xF3U, 1U) && // always clocked, no NAK IRQs
			Lpc17UsbSieWrite(0xD8U, 0U) && // unconfigured
			Lpc17UsbSieWrite(0xD0U, 0x80U) && // address zero, device enabled
			Lpc17UsbSieWrite(0x40U, 0U) && Lpc17UsbSieWrite(0x41U, 0U) &&
			Lpc17UsbEp0Ack(false, &status) && Lpc17UsbEp0Ack(true, &status);
		if (done)
		{
			LPC_USB->USBEpIntPri = 0U;
			LPC_USB->USBDevIntPri = 0U;
			// Only EP0 is owned here. Do not consume other endpoint events.
			LPC_USB->USBDevIntClr = LPC17_USB_CCEMPTY | LPC17_USB_CDFULL | (1U << 8);
		}
		else
		{
			s_Ep0Ready = false;
			s_SieFault = true;
		}
	}
	__set_PRIMASK(state);
	return done;
}

static bool Lpc17UsbResetDataInterrupts(void)
{
	const uint32_t pending = LPC_USB->USBEpIntSt & ~3U;
	uint8_t status;
	for (unsigned physical = 2; physical < 32U; ++physical)
	{
		if ((pending & (1UL << physical)) != 0U &&
			!Lpc17UsbEndpointAck(physical, &status))
			return false;
	}
	return true;
}

bool Lpc17UsbBusReset(void)
{
	const uint32_t state = __get_PRIMASK();
	__disable_irq();
	bool done = false;
	if (s_Prepared && (LPC_SC->PCONP & LPC17_USB_POWER) != 0U &&
		(LPC_USB->USBClkSt & LPC17_USB_CLOCKS) == LPC17_USB_CLOCKS)
	{
		const uint32_t deviceEnable = LPC_USB->USBDevIntEn;
		const uint32_t endpointEnable = LPC_USB->USBEpIntEn;
		LPC_USB->USBDevIntEn = 0U;
		LPC_USB->USBEpIntEn = 0U;
		uint8_t outStatus;
		uint8_t inStatus;
		// Bus reset clears FIFO contents, but not endpoint interrupt status.
		// Retire old completions before rebuilding the software generation.
		done = !s_SieFault && s_Ep0Ready &&
			LPC_USB->USBEpDMASt == 0U && LPC_USB->USBDMAIntEn == 0U &&
			(LPC_USB->USBReEp & ~3U) == 0U && (endpointEnable & ~3U) == 0U &&
			Lpc17UsbResetDataInterrupts() &&
			Lpc17UsbEp0Ack(false, &outStatus) && Lpc17UsbEp0Ack(true, &inStatus);
		// If service was delayed past the first SETUP, do not resize live RAM
		// or silently discard that request. Force controller reinitialization.
		done = done && (outStatus & 5U) == 0U;
		s_Ep0Ready = false;
		if (done)
			done = Lpc17UsbEp0Realize() && Lpc17UsbSieWrite(0xF3U, 1U) &&
				Lpc17UsbSieWrite(0x40U, 0U) && Lpc17UsbSieWrite(0x41U, 0U);
		if (done)
		{
			// Address/configuration were reset by the SIE. Do not reconnect or
			// acknowledge endpoint events arriving during the restoration.
			LPC_USB->USBEpIntEn = endpointEnable;
			LPC_USB->USBDevIntEn = deviceEnable;
		}
		else
		{
			s_Ep0Ready = false;
			s_SieFault = true;
		}
	}
	__set_PRIMASK(state);
	return done;
}

int Lpc17UsbDeviceStatus(uint8_t *pStatus)
{
	if (pStatus == 0)
		return -1;
	const uint32_t state = __get_PRIMASK();
	__disable_irq();
	int result = -1;
	if (Lpc17UsbPacketReady())
	{
		result = 0;
		if ((LPC_USB->USBDevIntSt & (1U << 3)) != 0U)
		{
			// UM10360 11.12.7: clear DEV_STAT BEFORE Get Device Status.
			LPC_USB->USBDevIntClr = 1U << 3;
			result = Lpc17UsbSieRead(0xFEU, pStatus) ? 1 : -1;
		}
	}
	__set_PRIMASK(state);
	return result;
}

int Lpc17UsbEp0Collect(uint8_t Status[2])
{
	if (Status == 0)
		return -1;
	const uint32_t state = __get_PRIMASK();
	__disable_irq();
	int result = -1;
	if (Lpc17UsbPacketReady() && s_Ep0Ready)
	{
		const uint32_t pending = LPC_USB->USBEpIntSt & LPC_USB->USBEpIntEn & 3U;
		uint8_t status[2] = {0, 0};
		// Capture IN before OUT: a new SETUP observed by the OUT select/clear
		// must be considered before dispatching the captured IN completion.
		// Never rescan here; directions absent from the snapshot stay pending.
		bool done = true;
		if ((pending & 2U) != 0U)
			done = Lpc17UsbEp0Ack(true, &status[1]);
		if (done && (pending & 1U) != 0U)
			done = Lpc17UsbEp0Ack(false, &status[0]);
		if (done)
		{
			Status[0] = status[0];
			Status[1] = status[1];
			result = (int)pending;
		}
	}
	__set_PRIMASK(state);
	return result;
}

int Lpc17UsbEp0Read(uint8_t *pData, int Capacity)
{
	if (Capacity < 0 || (Capacity > 0 && pData == 0))
		return -1;
	const uint32_t state = __get_PRIMASK();
	__disable_irq();
	int result = -1;
	uint8_t packet[64];
	if (Lpc17UsbPacketReady() && s_Ep0Ready)
	{
		LPC_USB->USBCtrl = 1U;
		int wait = LPC17_USB_WAIT;
		while ((LPC_USB->USBRxPLen & (1U << 11)) == 0U && --wait > 0)
		{
		}
		const unsigned length = LPC_USB->USBRxPLen & 0x3FFU;
		if (wait == 0 || length > 64U)
			s_SieFault = true;
		else if (length <= (unsigned)Capacity)
		{
			for (unsigned offset = 0; offset < length; offset += 4U)
			{
				const uint32_t word = LPC_USB->USBRxData;
				for (unsigned byte = 0; byte < 4U && offset + byte < length; ++byte)
					packet[offset + byte] = (uint8_t)(word >> (8U * byte));
			}
			LPC_USB->USBCtrl = 0U;
			uint8_t status;
			if (Lpc17UsbSieCommand(0U) && Lpc17UsbSieRead(0xF2U, &status))
			{
				// SETUP can overwrite EP0 during CPU reads. Never publish that data.
				if ((status & 1U) != 0U)
					result = -2;
				else
				{
					for (unsigned i = 0; i < length; ++i)
						pData[i] = packet[i];
					result = (int)length;
				}
			}
		}
		// Includes ZLP and insufficient capacity: RD_EN must not remain set.
		LPC_USB->USBCtrl = 0U;
	}
	__set_PRIMASK(state);
	return result;
}

bool Lpc17UsbEp0Write(const uint8_t *pData, int Length)
{
	if (Length < 0 || Length > 64 || (Length > 0 && pData == 0))
		return false;
	const uint32_t state = __get_PRIMASK();
	__disable_irq();
	bool done = false;
	uint8_t status;
	if (Lpc17UsbPacketReady() && s_Ep0Ready &&
		Lpc17UsbSieRead(1U, &status) && (status & 3U) == 0U)
	{
		LPC_USB->USBCtrl = 2U;
		LPC_USB->USBTxPLen = (uint32_t)Length;
		for (int offset = 0; offset < Length; offset += 4)
		{
			uint32_t word = 0U;
			for (int byte = 0; byte < 4 && offset + byte < Length; ++byte)
				word |= (uint32_t)pData[offset + byte] << (8U * byte);
			LPC_USB->USBTxData = word;
		}
		// Explicitly clear WR_EN, also for a zero-length packet.
		LPC_USB->USBCtrl = 0U;
		done = Lpc17UsbSieCommand(1U) && Lpc17UsbSieCommand(0xFAU);
	}
	__set_PRIMASK(state);
	return done;
}

// Called with PRIMASK held. Read the indexed hardware MPS instead of keeping
// a second endpoint-size table that could survive a hardware bus reset.
static unsigned Lpc17UsbDataSize(uint8_t EpNo, bool In)
{
	if (EpNo == 0U || EpNo > 15U || (EpNo % 3U == 0U && EpNo != 15U))
		return 0U;
	const unsigned physical = (unsigned)EpNo * 2U + (In ? 1U : 0U);
	if (!Lpc17UsbPacketReady() || !s_Ep0Ready ||
		(LPC_USB->USBReEp & (1UL << physical)) == 0U ||
		LPC_USB->USBEpDMASt != 0U || LPC_USB->USBDMAIntEn != 0U ||
		LPC_USB->USBCtrl != 0U || (LPC_USB->USBDevIntSt & 8U) != 0U)
		return 0U;
	LPC_USB->USBEpInd = physical;
	const unsigned size = LPC_USB->USBMaxPSize;
	if (size == 0U || size > 64U || (EpNo % 3U != 1U &&
		(size != 8U && size != 16U && size != 32U && size != 64U)))
		return 0U;
	return size;
}

int Lpc17UsbDataRead(uint8_t EpNo, uint8_t *pData, int Capacity)
{
	if (Capacity < 0 || (Capacity > 0 && pData == 0))
		return -1;
	const uint32_t state = __get_PRIMASK();
	__disable_irq();
	int result = -1;
	const unsigned size = Lpc17UsbDataSize(EpNo, false);
	uint8_t status;
	uint8_t packet[64];
	if (size != 0U && Lpc17UsbSieRead(EpNo * 2U, &status) && (status & 2U) == 0U)
	{
		// FE is OR of both OUT buffer-full bits: consume one packet per call.
		if ((status & 1U) == 0U)
			result = -2;
		else
		{
			LPC_USB->USBCtrl = ((uint32_t)EpNo << 2) | 1U;
			int wait = LPC17_USB_WAIT;
			while ((LPC_USB->USBRxPLen & (1U << 11)) == 0U && --wait > 0)
			{
			}
			const uint32_t info = LPC_USB->USBRxPLen;
			const unsigned length = info & 0x3FFU;
			if (wait == 0 || length > size || (info & (1U << 10)) == 0U)
				s_SieFault = true;
			else if (length <= (unsigned)Capacity)
			{
				for (unsigned offset = 0; offset < length; offset += 4U)
				{
					const uint32_t word = LPC_USB->USBRxData;
					for (unsigned byte = 0; byte < 4U && offset + byte < length; ++byte)
						packet[offset + byte] = (uint8_t)(word >> (8U * byte));
				}
				LPC_USB->USBCtrl = 0U;
				// A reset invalidates the packet even if the FIFO read completed.
				if (Lpc17UsbDataSize(EpNo, false) != 0U &&
					Lpc17UsbSieCommand(EpNo * 2U) && Lpc17UsbSieCommand(0xF2U) &&
					(LPC_USB->USBDevIntSt & 8U) == 0U)
				{
					for (unsigned i = 0; i < length; ++i)
						pData[i] = packet[i];
					result = (int)length;
				}
			}
			LPC_USB->USBCtrl = 0U;
		}
	}
	__set_PRIMASK(state);
	return result;
}

bool Lpc17UsbDataWrite(uint8_t EpNo, const uint8_t *pData, int Length)
{
	if (Length < 0 || Length > 64 || (Length > 0 && pData == 0))
		return false;
	const uint32_t state = __get_PRIMASK();
	__disable_irq();
	bool done = false;
	const unsigned size = Lpc17UsbDataSize(EpNo, true);
	uint8_t status;
	const uint8_t physical = EpNo * 2U + 1U;
	// FE is AND of both IN buffer-full bits: either free buffer may be used.
	if (size != 0U && (unsigned)Length <= size &&
		Lpc17UsbSieRead(physical, &status) && (status & 3U) == 0U)
	{
		LPC_USB->USBCtrl = ((uint32_t)EpNo << 2) | 2U;
		LPC_USB->USBTxPLen = (uint32_t)Length;
		for (int offset = 0; offset < Length; offset += 4)
		{
			uint32_t word = 0U;
			for (int byte = 0; byte < 4 && offset + byte < Length; ++byte)
				word |= (uint32_t)pData[offset + byte] << (8U * byte);
			LPC_USB->USBTxData = word;
		}
		LPC_USB->USBCtrl = 0U;
		done = Lpc17UsbDataSize(EpNo, true) != 0U &&
			Lpc17UsbSieCommand(physical) && Lpc17UsbSieCommand(0xFAU) &&
			(LPC_USB->USBDevIntSt & 8U) == 0U;
	}
	__set_PRIMASK(state);
	return done;
}
