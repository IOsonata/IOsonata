/**-------------------------------------------------------------------------
@file	i2c_test.cpp

@brief	Host test of i2c_ra4m1.cpp against a model of the RA4M1 IIC.

		The driver is compiled unchanged. The IIC registers, PRCR and MSTPCRB
		are objects of tests/ra4m1/i2c_include whose accesses call the model
		below, so the driver code runs the same register sequence as on the
		MCU. The model has:

		- an IIC master engine and a bus with simulated slave devices that
		  record every start, address, byte, acknowledge and stop as a trace
		  string, so the exact bus sequence is checked, not only the data,
		- an IIC slave engine driven by a bus master of the test, with the
		  SCL hold of WAIT, of an unread ICDRR and of a master NACK,
		- the ICU as edge events, the request is cleared by the handler, and
		  a settable latency between the edge and the handler,
		- every register access is one tick, handlers may take the CPU
		  between any two accesses of the driver,
		- fault injection: a stuck SCL with the IIC timeout, arbitration
		  loss, and a slave holding SDA low for the bus recovery.

		The model checks the driver sequence: ICDRT written only when empty,
		ST only on a free bus, SP and RS only as bus master, ACKBT written
		only with ACKWP, no byte lost to an unread ICDRR, MSTPCRB written with
		PRC1, bit rate registers written with bits 7..5 set.

		Not a timing or electrical test and not the silicon: the sequence
		of the model follows the FSP r_iic_master and r_iic_slave flows.

@author	Hoang Nguyen Hoan
@date	Oct. 13, 2026

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
#include <stdarg.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "i2c_ra4m1.h"

#include <string>
#include <vector>

#include "ra4m1xxx.h"
#include "interrupt_ra4m1.h"
#include "coredev/interrupt.h"
#include "coredev/i2c.h"
#include "coredev/system_core_clock.h"

// ---------------------------------------------------------------------------
// Test bookkeeping
// ---------------------------------------------------------------------------

static int s_Checks;
static int s_Fails;

static void Check(bool Cond, const char *pFmt, ...)
{
	s_Checks++;

	if (!Cond)
	{
		s_Fails++;
		va_list ap;
		va_start(ap, pFmt);
		printf("FAIL: ");
		vprintf(pFmt, ap);
		printf("\n");
		va_end(ap);
	}
}

// ---------------------------------------------------------------------------
// Clock, pins, system registers
// ---------------------------------------------------------------------------

R_IIC0_Type g_TestIic[2];
R_PFS_Type g_TestPfs;
R_SYSTEM_Type g_TestSystem;
R_MSTP_Type g_TestMstp;
uint32_t g_TestPrimask;

static uint32_t s_Pclkb = 24000000;
static int s_Violations;
static std::string s_ViolationLog;

extern "C" uint32_t SystemCoreClockGet(void)
{
	return 48000000;
}

extern "C" uint32_t SystemPeriphClockGet(int Idx)
{
	return Idx == 1 ? s_Pclkb : 0;
}

static void Violation(const char *pFmt, ...)
{
	char buf[160];
	va_list ap;

	va_start(ap, pFmt);
	vsnprintf(buf, sizeof(buf), pFmt, ap);
	va_end(ap);
	s_Violations++;
	s_ViolationLog += buf;
	s_ViolationLog += "; ";
}

// Pin model: latches, GPIO or peripheral function, a stuck slave on SDA
static bool s_PinGpio[10][16];
static bool s_PinLatch[10][16];
static int s_SclPort = -1, s_SclPin, s_SdaPort = -1, s_SdaPin;
static int s_SdaHeldPulses;		// SDA reads low until this many SCL rises
static int s_SclRises;
static int s_PinStops;

extern "C" void IOPinDisable(int PortNo, int PinNo)
{
	s_PinGpio[PortNo][PinNo] = true;
}

extern "C" void IOPinConfig(int PortNo, int PinNo, int PinOp, IOPINDIR Dir, IOPINRES Resistor, IOPINTYPE Type)
{
	s_PinGpio[PortNo][PinNo] = PinOp == IOPINOP_GPIO;
}

extern "C" int IOPinRead(int PortNo, int PinNo)
{
	if (PortNo == s_SdaPort && PinNo == s_SdaPin && s_SdaHeldPulses > 0)
	{
		return 0;
	}

	return s_PinLatch[PortNo][PinNo] ? 1 : 0;
}

extern "C" void IOPinSet(int PortNo, int PinNo)
{
	const bool was = s_PinLatch[PortNo][PinNo];

	s_PinLatch[PortNo][PinNo] = true;

	if (PortNo == s_SclPort && PinNo == s_SclPin && !was)
	{
		s_SclRises++;

		if (s_SdaHeldPulses > 0)
		{
			s_SdaHeldPulses--;
		}
	}

	if (PortNo == s_SdaPort && PinNo == s_SdaPin && !was &&
		s_PinLatch[s_SclPort][s_SclPin])
	{
		s_PinStops++;
	}
}

extern "C" void IOPinClear(int PortNo, int PinNo)
{
	s_PinLatch[PortNo][PinNo] = false;
}

// ---------------------------------------------------------------------------
// Bus with simulated slave devices
// ---------------------------------------------------------------------------

static std::string s_Trace;

typedef struct
{
	bool bPresent;
	int Addr;
	int AckLimit;			// write bytes acknowledged before a NACK, -1 none
	std::vector<uint8_t> Written;
	std::vector<uint8_t> ReadData;
	size_t ReadPos;
	int Reads;
	bool bSelected;
} SimDev_t;

static SimDev_t s_Dev[4];

static void DevReset(void)
{
	for (int i = 0; i < 4; i++)
	{
		s_Dev[i] = SimDev_t();
		s_Dev[i].AckLimit = -1;
	}
}

static void TraceByte(char Pfx, uint8_t B, bool Ack)
{
	char buf[16];

	snprintf(buf, sizeof(buf), " %c%02X%c", Pfx, B, Ack ? '+' : '-');
	s_Trace += buf;
}

static SimDev_t *DevFind(int Addr)
{
	for (int i = 0; i < 4; i++)
	{
		if (s_Dev[i].bPresent && s_Dev[i].Addr == Addr)
		{
			return &s_Dev[i];
		}
	}

	return NULL;
}

static SimDev_t *s_pSel;

static void BusStartCond(bool bRestart)
{
	s_Trace += bRestart ? " Sr" : "S";
	s_pSel = NULL;
}

static bool BusAddress(uint8_t B)
{
	SimDev_t *d = DevFind(B >> 1);

	TraceByte((B & 1) ? 'R' : 'W', B >> 1, d != NULL);
	s_pSel = d;

	if (d != NULL)
	{
		d->bSelected = true;

		if ((B & 1) == 0)
		{
			d->Written.clear();
		}
	}

	return d != NULL;
}

static bool BusWrite(uint8_t B)
{
	bool ack = s_pSel != NULL;

	if (ack && s_pSel->AckLimit >= 0 && (int)s_pSel->Written.size() >= s_pSel->AckLimit)
	{
		ack = false;
	}

	if (ack)
	{
		s_pSel->Written.push_back(B);
	}

	TraceByte('d', B, ack);

	return ack;
}

static uint8_t BusRead(void)
{
	uint8_t b = 0xFF;

	if (s_pSel != NULL)
	{
		s_pSel->Reads++;

		if (s_pSel->ReadPos < s_pSel->ReadData.size())
		{
			b = s_pSel->ReadData[s_pSel->ReadPos++];
		}
	}

	return b;
}

static void BusReadAck(uint8_t B, bool Ack)
{
	TraceByte('r', B, Ack);
}

static void BusStopCond(void)
{
	s_Trace += " P";
	s_pSel = NULL;
}

// ---------------------------------------------------------------------------
// ICU model
// ---------------------------------------------------------------------------

typedef struct
{
	uint8_t Evt;
	Ra4m1IRQHandler_t Handler;
	void *pCtx;
	bool bIr;
	long IrDue;
	bool bAcked;
} IcuSlot_t;

static IcuSlot_t s_Slot[32];
static bool s_bInIsr;
static long s_Tick;
static long s_IsrLatency;
static int s_IsrCount;

IRQn_Type Ra4m1RegisterIntHandler(uint8_t EvtId, int Prio, Ra4m1IRQHandler_t pHandler, void *pCtx)
{
	for (int i = 0; i < 32; i++)
	{
		if (s_Slot[i].Handler && s_Slot[i].Evt == EvtId)
		{
			return (IRQn_Type)-1;
		}
	}

	for (int i = 0; i < 32; i++)
	{
		if (s_Slot[i].Handler == NULL)
		{
			s_Slot[i] = IcuSlot_t();
			s_Slot[i].Evt = EvtId;
			s_Slot[i].Handler = pHandler;
			s_Slot[i].pCtx = pCtx;
			return (IRQn_Type)i;
		}
	}

	return (IRQn_Type)-1;
}

void Ra4m1UnregisterIntHandler(IRQn_Type IrqNo)
{
	if ((unsigned)IrqNo < 32)
	{
		s_Slot[IrqNo] = IcuSlot_t();
	}
}

bool Ra4m1AcknowledgeInt(IRQn_Type IrqNo)
{
	if ((unsigned)IrqNo >= 32 || s_Slot[IrqNo].Handler == NULL)
	{
		return false;
	}

	s_Slot[IrqNo].bIr = false;
	s_Slot[IrqNo].bAcked = true;

	return true;
}

static int SlotCount(void)
{
	int n = 0;

	for (int i = 0; i < 32; i++)
	{
		n += s_Slot[i].Handler != NULL;
	}

	return n;
}

static void RaiseEvent(uint8_t Evt)
{
	for (int i = 0; i < 32; i++)
	{
		if (s_Slot[i].Handler && s_Slot[i].Evt == Evt && !s_Slot[i].bIr)
		{
			s_Slot[i].bIr = true;
			s_Slot[i].IrDue = s_Tick + s_IsrLatency;
		}
	}
}

void TestIrqPoint(void)
{
	if (s_bInIsr || g_TestPrimask != 0)
	{
		return;
	}

	for (int guard = 0; guard < 100000; guard++)
	{
		int pick = -1;

		for (int i = 0; i < 32; i++)
		{
			if (s_Slot[i].Handler && s_Slot[i].bIr && s_Slot[i].IrDue <= s_Tick)
			{
				pick = i;
				break;
			}
		}

		if (pick < 0)
		{
			return;
		}

		IcuSlot_t &slot = s_Slot[pick];

		s_bInIsr = true;
		s_IsrCount++;
		slot.bAcked = false;

		Ra4m1IRQHandler_t handler = slot.Handler;
		void *ctx = slot.pCtx;

		handler(pick, ctx);

		// Same as the dispatcher: a handler that did not acknowledge gets
		// the trailing clear.
		if (!slot.bAcked && s_Slot[pick].Handler == handler)
		{
			s_Slot[pick].bIr = false;
		}

		s_bInIsr = false;
	}

	printf("FAIL: interrupt storm\n");
	s_Fails++;
}

void TestBasePriLowered(void)
{
}

// ---------------------------------------------------------------------------
// IIC model
// ---------------------------------------------------------------------------

#define M_BBSY			0x80U
#define M_MST			0x40U
#define M_TRS			0x20U

#define SR2_TMOF		0x01U
#define SR2_AL			0x02U
#define SR2_START		0x04U
#define SR2_STOP		0x08U
#define SR2_NACKF		0x10U
#define SR2_RDRF		0x20U
#define SR2_TEND		0x40U
#define SR2_TDRE		0x80U

#define IER_TMOIE		0x01U
#define IER_ALIE		0x02U
#define IER_STIE		0x04U
#define IER_SPIE		0x08U
#define IER_NAKIE		0x10U
#define IER_RIE			0x20U
#define IER_TEIE		0x40U
#define IER_TIE			0x80U

#define MR3_ACKBT		0x08U
#define MR3_ACKWP		0x10U
#define MR3_WAIT		0x40U

#define BYTE_TICKS		100
#define WAIT_EDGE_TICKS	3
#define COND_TICKS		3

typedef enum
{
	PH_IDLE,
	PH_START,			// START or RESTART being generated
	PH_WAIT_BYTE,		// SCL low, waiting for ICDRT or SP or RS
	PH_TXBYTE,			// byte in flight
	PH_RXHOLD_DUMMY,	// address sent, waiting for the dummy read
	PH_RXBYTE,			// byte in flight
	PH_RXDONE,			// RDRF set, waiting for the WAIT edge
	PH_RXHOLD,			// SCL held until ICDRR is read
	PH_STOP				// STOP being generated
} Phase_t;

typedef struct
{
	uint8_t iccr1, iccr2, icmr1, icmr2, icmr3, icfer, icser, icier, icsr1, icsr2;
	uint8_t sarl[3], saru[3], icbrl, icbrh, icdrt, icdrr;

	Phase_t ph;
	long due;
	bool bRestart;
	bool bSp;
	bool bAddr;
	uint8_t txb;
	long tdreAt;
	long stall;

	// Slave
	bool bSlActive;
	bool bSlRead;
	bool bNackHold;
	bool bSlShift;			// transmit byte moved to the shift register

	// Previous event signals
	bool sig[8];
} Iic_t;

static Iic_t s_Iic[2];

// Fault injection
static bool s_bSclStuck;
static int s_ArbLossAtByte = -1;
static int s_MasterByteNo;
static int s_Overrun;

static inline bool Ice(const Iic_t &u)
{
	return (u.iccr1 & 0x80U) != 0 && (u.iccr1 & 0x40U) == 0;
}

static void UnitReset(Iic_t &u)
{
	u.icsr1 = 0;
	u.icsr2 = SR2_TDRE;
	u.iccr2 = 0;
	u.ph = PH_IDLE;
	u.bSp = false;
	u.bSlActive = false;
	u.bNackHold = false;
	u.tdreAt = 0;
	u.stall = 0;
}

static void UpdateEvents(int Unit)
{
	Iic_t &u = s_Iic[Unit];
	const uint8_t base = Unit == 0 ? 0x35 : 0x3A;
	bool now[8];

	now[0] = Ice(u) && (u.icsr2 & SR2_RDRF) && (u.icier & IER_RIE);
	now[1] = Ice(u) && (u.icsr2 & SR2_TDRE) && (u.icier & IER_TIE);
	now[2] = Ice(u) && (u.icsr2 & SR2_TEND) && (u.icier & IER_TEIE);
	now[3] = Ice(u) && (u.icsr2 & SR2_TMOF) && (u.icier & IER_TMOIE);
	now[4] = Ice(u) && (u.icsr2 & SR2_AL) && (u.icier & IER_ALIE);
	now[5] = Ice(u) && (u.icsr2 & SR2_START) && (u.icier & IER_STIE);
	now[6] = Ice(u) && (u.icsr2 & SR2_STOP) && (u.icier & IER_SPIE);
	now[7] = Ice(u) && (u.icsr2 & SR2_NACKF) && (u.icier & IER_NAKIE);

	for (int i = 0; i < 8; i++)
	{
		if (now[i] && !u.sig[i])
		{
			RaiseEvent((uint8_t)(base + (i < 3 ? i : 3)));
		}

		u.sig[i] = now[i];
	}
}

static void MasterStartByte(Iic_t &u)
{
	u.ph = PH_RXBYTE;
	u.due = s_Tick + BYTE_TICKS;
}

static void MasterAfterTx(Iic_t &u)
{
	u.ph = PH_WAIT_BYTE;

	if (u.bSp)
	{
		u.ph = PH_STOP;
		u.due = s_Tick + COND_TICKS;
	}
}

static void MasterRun(Iic_t &u)
{
	if (u.tdreAt != 0 && s_Tick >= u.tdreAt)
	{
		u.tdreAt = 0;
		u.icsr2 |= SR2_TDRE;
	}

	if (u.ph == PH_IDLE || u.ph == PH_WAIT_BYTE || u.ph == PH_RXHOLD ||
		u.ph == PH_RXHOLD_DUMMY || s_Tick < u.due)
	{
		return;
	}

	if (s_bSclStuck)
	{
		// SCL is held low. The IIC timeout flags it after a while.
		if (++u.stall > 3000 && (u.icfer & 0x01U))
		{
			u.icsr2 |= SR2_TMOF;
		}

		u.due = s_Tick + 1;
		return;
	}

	switch (u.ph)
	{
		case PH_START:
			BusStartCond(u.bRestart);
			u.iccr2 |= M_BBSY;
			u.icsr2 |= SR2_START;
			u.icsr2 &= (uint8_t)~SR2_TEND;
			u.icsr2 |= SR2_TDRE;
			u.bAddr = true;
			u.iccr2 |= M_TRS;
			u.ph = PH_WAIT_BYTE;
			break;

		case PH_TXBYTE:
			s_MasterByteNo++;

			if (s_ArbLossAtByte >= 0 && s_MasterByteNo == s_ArbLossAtByte)
			{
				u.icsr2 |= SR2_AL;
				u.iccr2 &= (uint8_t)~(M_MST | M_TRS);
				u.ph = PH_IDLE;
				break;
			}

			if (u.bAddr)
			{
				const bool ack = BusAddress(u.txb);

				u.bAddr = false;

				if (!ack)
				{
					u.icsr2 |= SR2_NACKF | SR2_TEND;
					MasterAfterTx(u);
				}
				else if ((u.txb & 1) != 0)
				{
					u.iccr2 &= (uint8_t)~M_TRS;
					u.icdrr = 0xAA;
					u.icsr2 |= SR2_RDRF;
					u.ph = PH_RXHOLD_DUMMY;
				}
				else
				{
					u.icsr2 |= SR2_TEND;
					MasterAfterTx(u);
				}
			}
			else
			{
				const bool ack = BusWrite(u.txb);

				u.icsr2 |= SR2_TEND;

				if (!ack)
				{
					u.icsr2 |= SR2_NACKF;
				}

				MasterAfterTx(u);
			}
			break;

		case PH_RXBYTE:
		{
			const uint8_t b = BusRead();
			const bool ack = (u.icmr3 & MR3_ACKBT) == 0;

			BusReadAck(b, ack);

			if (u.icsr2 & SR2_RDRF)
			{
				s_Overrun++;
				Violation("ICDRR overrun");
			}

			u.icdrr = b;
			u.icsr2 |= SR2_RDRF;
			u.ph = PH_RXDONE;
			u.due = s_Tick + WAIT_EDGE_TICKS;
			break;
		}

		case PH_RXDONE:
			// With WAIT, SCL stays low until ICDRR is read, and if that
			// already happened there is nothing to wait for. Without WAIT
			// the next byte is clocked at once and ICDRR is overrun when it
			// is not read before that byte ends.
			if ((u.icmr3 & MR3_WAIT) != 0 && (u.icsr2 & SR2_RDRF) != 0)
			{
				u.ph = PH_RXHOLD;
			}
			else if (u.bSp)
			{
				u.ph = PH_STOP;
				u.due = s_Tick + COND_TICKS;
			}
			else
			{
				MasterStartByte(u);
			}
			break;

		case PH_STOP:
			BusStopCond();
			u.icsr2 |= SR2_STOP;
			u.icsr2 &= (uint8_t)~SR2_TEND;
			u.iccr2 &= (uint8_t)~(M_BBSY | M_MST | M_TRS);
			u.bSp = false;
			u.ph = PH_IDLE;
			break;

		default:
			break;
	}
}

static void MasterWriteCr2(Iic_t &u, uint8_t v)
{
	if (!Ice(u))
	{
		return;
	}

	if (v & 0x02U)
	{
		if ((u.iccr2 & M_BBSY) || u.ph != PH_IDLE)
		{
			Violation("ST on a busy bus");
		}
		else
		{
			u.iccr2 |= M_MST | M_TRS;
			u.ph = PH_START;
			u.due = s_Tick + COND_TICKS;
			u.bRestart = false;
			u.bSp = false;
		}
	}

	if (v & 0x04U)
	{
		if ((u.iccr2 & (M_BBSY | M_MST)) != (M_BBSY | M_MST))
		{
			Violation("RS without bus mastership");
		}
		else if (u.ph != PH_WAIT_BYTE)
		{
			Violation("RS while a byte is in flight, phase %d", (int)u.ph);
		}
		else
		{
			u.ph = PH_START;
			u.due = s_Tick + COND_TICKS;
			u.bRestart = true;
		}
	}

	if (v & 0x08U)
	{
		if ((u.iccr2 & (M_BBSY | M_MST)) != (M_BBSY | M_MST))
		{
			Violation("SP without bus mastership");
		}
		else
		{
			u.bSp = true;

			if (u.ph == PH_WAIT_BYTE)
			{
				u.ph = PH_STOP;
				u.due = s_Tick + COND_TICKS;
			}
		}
	}
}

// ---- Slave side, driven by the bus master of the test ----

static bool SclHeld(Iic_t &u)
{
	if (u.bNackHold)
	{
		return true;
	}

	return u.bSlActive && (u.icmr3 & MR3_WAIT) != 0 && (u.icsr2 & SR2_RDRF) != 0;
}

static void Advance(void);

static bool BmWaitScl(int Unit)
{
	for (int i = 0; i < 200000; i++)
	{
		if (!SclHeld(s_Iic[Unit]))
		{
			return true;
		}

		Advance();
		TestIrqPoint();
	}

	return false;
}

static void BmStart(int Unit, bool bRestart)
{
	Iic_t &u = s_Iic[Unit];

	if (bRestart)
	{
		BmWaitScl(Unit);
		u.icsr1 = 0;
		u.bSlActive = false;
	}

	s_Trace += bRestart ? " Sr" : "S";
	u.iccr2 |= M_BBSY;
	u.icsr2 |= SR2_START;
	UpdateEvents(Unit);
	TestIrqPoint();
}

// Returns true when the slave acknowledged
static bool BmAddress(int Unit, int Addr, bool bRead)
{
	Iic_t &u = s_Iic[Unit];

	if (!BmWaitScl(Unit))
	{
		Check(false, "SCL held at the address");
		return false;
	}

	int match = -1;

	for (int i = 0; i < 3; i++)
	{
		if (Ice(u) && ((u.icser >> i) & 1U) && (u.sarl[i] >> 1) == Addr)
		{
			match = i;
		}
	}

	if (match < 0)
	{
		TraceByte(bRead ? 'R' : 'W', (uint8_t)Addr, false);
		return false;
	}

	const bool ack = (u.icmr3 & MR3_ACKBT) == 0;

	u.icsr1 |= (uint8_t)(1U << match);
	TraceByte(bRead ? 'R' : 'W', (uint8_t)Addr, ack);

	if (!ack)
	{
		return false;
	}

	u.bSlActive = true;
	u.bSlRead = bRead;
	u.icdrr = (uint8_t)((Addr << 1) | (bRead ? 1 : 0));

	if (u.icsr2 & SR2_RDRF)
	{
		s_Overrun++;
		Violation("ICDRR overrun at the address");
	}

	u.icsr2 |= SR2_RDRF;

	if (bRead)
	{
		u.iccr2 |= M_TRS;
		u.icsr2 |= SR2_TDRE;
		u.icsr2 &= (uint8_t)~SR2_TEND;
	}

	UpdateEvents(Unit);
	TestIrqPoint();

	return true;
}

static bool BmWrite(int Unit, uint8_t B)
{
	Iic_t &u = s_Iic[Unit];

	if (!BmWaitScl(Unit))
	{
		Check(false, "SCL held at a write byte");
		return false;
	}

	if (!u.bSlActive || u.bSlRead)
	{
		TraceByte('d', B, false);
		return false;
	}

	const bool ack = (u.icmr3 & MR3_ACKBT) == 0;

	TraceByte('d', B, ack);

	if (u.icsr2 & SR2_RDRF)
	{
		s_Overrun++;
		Violation("ICDRR overrun on a data byte");
	}

	u.icdrr = B;
	u.icsr2 |= SR2_RDRF;
	UpdateEvents(Unit);
	TestIrqPoint();

	return ack;
}

// Returns the byte, or -1 when the slave stalls
static int BmRead(int Unit, bool bAck)
{
	Iic_t &u = s_Iic[Unit];

	if (!BmWaitScl(Unit))
	{
		Check(false, "SCL held at a read byte");
		return -1;
	}

	// The slave must load ICDRT before the clocks of the byte
	int i = 0;

	while ((u.icsr2 & SR2_TDRE) != 0 && i++ < 200000)
	{
		Advance();
		TestIrqPoint();
	}

	if ((u.icsr2 & SR2_TDRE) != 0 || !u.bSlActive || !u.bSlRead)
	{
		return -1;
	}

	const uint8_t b = u.icdrt;

	u.icsr2 |= SR2_TDRE;
	u.icsr2 |= SR2_TEND;
	TraceByte('r', b, bAck);

	if (!bAck)
	{
		u.icsr2 |= SR2_NACKF;
		u.bNackHold = true;
	}

	UpdateEvents(Unit);
	TestIrqPoint();

	return b;
}

static void BmStop(int Unit)
{
	Iic_t &u = s_Iic[Unit];

	BmWaitScl(Unit);
	s_Trace += " P";
	u.icsr2 |= SR2_STOP;
	u.icsr2 &= (uint8_t)~SR2_TEND;
	u.iccr2 &= (uint8_t)~(M_BBSY | M_TRS);
	u.icsr1 = 0;
	u.bSlActive = false;
	u.icsr2 |= SR2_TDRE;
	UpdateEvents(Unit);
	TestIrqPoint();
}

// ---- Register access ----

static int UnitOf(const void *pReg)
{
	const char *p = (const char *)pReg;

	if (p >= (const char *)&g_TestIic[0] && p < (const char *)&g_TestIic[1])
	{
		return 0;
	}

	return 1;
}

static void Advance(void)
{
	s_Tick++;

	for (int u = 0; u < 2; u++)
	{
		MasterRun(s_Iic[u]);
		UpdateEvents(u);
	}
}

static uint32_t IicRead(Iic_t &u, int Unit, int Id)
{
	switch (Id)
	{
		case IICREG_ICCR1: return u.iccr1;
		case IICREG_ICCR2: return u.iccr2;
		case IICREG_ICMR1: return u.icmr1;
		case IICREG_ICMR2: return u.icmr2;
		case IICREG_ICMR3: return u.icmr3;
		case IICREG_ICFER: return u.icfer;
		case IICREG_ICSER: return u.icser;
		case IICREG_ICIER: return u.icier;
		case IICREG_ICSR1: return u.icsr1;
		case IICREG_ICSR2: return u.icsr2;
		case IICREG_ICBRL: return u.icbrl;
		case IICREG_ICBRH: return u.icbrh;
		case IICREG_ICDRT: return u.icdrt;
		case IICREG_ICDRR:
		{
			const uint8_t v = u.icdrr;

			u.icsr2 &= (uint8_t)~SR2_RDRF;
			u.bNackHold = false;

			if (u.ph == PH_RXHOLD_DUMMY)
			{
				MasterStartByte(u);
			}
			else if (u.ph == PH_RXHOLD)
			{
				if (u.bSp)
				{
					u.ph = PH_STOP;
					u.due = s_Tick + COND_TICKS;
				}
				else
				{
					MasterStartByte(u);
				}
			}

			return v;
		}
		default: return 0;
	}
}

static uint16_t s_Prcr;
static uint32_t s_Mstpcrb = 0xFFFFFFFFU;

static uint32_t SysRead(int Id)
{
	return Id == SYSREG_PRCR ? s_Prcr : Id == SYSREG_SCKDIVCR ? 0x01000100U : s_Mstpcrb;
}

static void SysWrite(int Id, uint32_t Val)
{
	if (Id == SYSREG_SCKDIVCR)
	{
		return;
	}

	if (Id == SYSREG_PRCR)
	{
		if ((Val & 0xFF00U) != 0xA500U)
		{
			Violation("PRCR written without the key");
			return;
		}

		s_Prcr = (uint16_t)(Val & 0xFFU);
	}
	else
	{
		if ((s_Prcr & 0x02U) == 0)
		{
			Violation("MSTPCRB written with PRC1 clear");
			return;
		}

		s_Mstpcrb = Val;
	}
}

static void IicWrite(Iic_t &u, int Unit, int Id, uint32_t Val)
{
	const uint8_t v = (uint8_t)Val;

	switch (Id)
	{
		case IICREG_ICCR1:
		{
			const bool wasIce = (u.iccr1 & 0x80U) != 0;
			const bool wasRst = (u.iccr1 & 0x40U) != 0;

			u.iccr1 = v;

			if ((v & 0x80U) == 0 || ((v & 0x40U) != 0 && !wasRst) ||
				(!wasIce && (v & 0x80U) != 0))
			{
				UnitReset(u);
			}
			break;
		}
		case IICREG_ICCR2:
			MasterWriteCr2(u, v);
			break;
		case IICREG_ICMR1: u.icmr1 = v; break;
		case IICREG_ICMR2: u.icmr2 = v; break;
		case IICREG_ICMR3:
		{
			uint8_t nv = v;

			if ((v & MR3_ACKBT) != (u.icmr3 & MR3_ACKBT) && (v & MR3_ACKWP) == 0)
			{
				Violation("ACKBT written without ACKWP");
				nv = (uint8_t)((v & ~MR3_ACKBT) | (u.icmr3 & MR3_ACKBT));
			}

			u.icmr3 = nv;
			break;
		}
		case IICREG_ICFER: u.icfer = v; break;
		case IICREG_ICSER: u.icser = v; break;
		case IICREG_ICIER: u.icier = v; break;
		case IICREG_ICSR1: u.icsr1 = (uint8_t)(u.icsr1 & v); break;
		case IICREG_ICSR2:
		{
			// Flags are cleared by writing 0, 1 leaves them as they are.
			const uint8_t clr = (uint8_t)(~v & (SR2_TMOF | SR2_AL | SR2_START | SR2_STOP | SR2_NACKF | SR2_TEND));

			u.icsr2 &= (uint8_t)~clr;
			break;
		}
		case IICREG_ICBRL:
			if ((v & 0xE0U) != 0xE0U)
			{
				Violation("ICBRL bits 7..5 not set");
			}
			u.icbrl = v;
			break;
		case IICREG_ICBRH:
			if ((v & 0xE0U) != 0xE0U)
			{
				Violation("ICBRH bits 7..5 not set");
			}
			u.icbrh = v;
			break;
		case IICREG_ICDRT:
			if ((u.icsr2 & SR2_TDRE) == 0)
			{
				Violation("ICDRT written while full");
			}

			u.icdrt = v;

			if ((u.iccr2 & M_MST) != 0)
			{
				u.txb = v;
				u.icsr2 &= (uint8_t)~(SR2_TDRE | SR2_TEND);
				u.tdreAt = s_Tick + 2;
				u.ph = PH_TXBYTE;
				u.due = s_Tick + BYTE_TICKS;
			}
			else
			{
				u.icsr2 &= (uint8_t)~(SR2_TDRE | SR2_TEND);
			}
			break;
		default:
			break;
	}

	if (Id == IICREG_ICCR1 || Id == IICREG_ICCR2 || Id == IICREG_ICIER || Id == IICREG_ICSR2 ||
		Id == IICREG_ICDRT)
	{
		UpdateEvents(Unit);
	}
}

static uint32_t RegRead(const void *pReg, int Id)
{
	if (Id >= SYSREG_PRCR)
	{
		return SysRead(Id);
	}

	const int unit = UnitOf(pReg);
	Iic_t &u = s_Iic[unit];

	if (Id == IICREG_SARL || Id == IICREG_SARU)
	{
		const R_IIC0_Type *base = &g_TestIic[unit];
		const int idx = (int)((const TestIicSar_t *)(Id == IICREG_SARL ?
			(const void *)pReg : (const void *)((const char *)pReg - 1)) - base->SAR);

		return Id == IICREG_SARL ? u.sarl[idx] : u.saru[idx];
	}

	return IicRead(u, unit, Id);
}

static void RegWrite(const void *pReg, int Id, uint32_t Val)
{
	if (Id >= SYSREG_PRCR)
	{
		SysWrite(Id, Val);
		return;
	}

	const int unit = UnitOf(pReg);
	Iic_t &u = s_Iic[unit];

	if (Id == IICREG_SARL || Id == IICREG_SARU)
	{
		const R_IIC0_Type *base = &g_TestIic[unit];
		const int idx = (int)((const TestIicSar_t *)(Id == IICREG_SARL ?
			(const void *)pReg : (const void *)((const char *)pReg - 1)) - base->SAR);

		if (Id == IICREG_SARL)
		{
			u.sarl[idx] = (uint8_t)Val;
		}
		else
		{
			u.saru[idx] = (uint8_t)Val;
		}
		return;
	}

	IicWrite(u, unit, Id, Val);
}

// Hooks used by TestReg. Each access is a tick, then the handlers may run.
static uint32_t Access(const void *pReg, int Id, bool bWrite, uint32_t Val)
{
	Advance();

	uint32_t r = 0;

	if (bWrite)
	{
		RegWrite(pReg, Id, Val);
	}
	else
	{
		r = RegRead(pReg, Id);
	}

	for (int u = 0; u < 2; u++)
	{
		UpdateEvents(u);
	}

	TestIrqPoint();

	return r;
}

uint32_t TestRegRead(const void *pReg, int Id)
{
	return Access(pReg, Id, false, 0);
}

void TestRegWrite(const void *pReg, int Id, uint32_t Val)
{
	Access(pReg, Id, true, Val);
}

// ---------------------------------------------------------------------------
// Test helpers
// ---------------------------------------------------------------------------

// The slave buffer setters of src/coredev/i2c.cpp, which needs the delay
// code of a target, so they are repeated here.
void I2CSetReadRqstData(I2CDev_t * const pDev, int SlaveIdx, uint8_t * const pData, int DataLen)
{
	if (SlaveIdx < 0 || SlaveIdx >= I2C_SLAVEMODE_MAX_ADDR || pDev == NULL)
	{
		return;
	}

	pDev->pRRData[SlaveIdx] = pData;
	pDev->RRDataLen[SlaveIdx] = DataLen;
}

void I2CSetWriteRqstBuffer(I2CDev_t * const pDev, int SlaveIdx, uint8_t * const pBuff, int BuffLen)
{
	if (SlaveIdx < 0 || SlaveIdx >= I2C_SLAVEMODE_MAX_ADDR || pDev == NULL)
	{
		return;
	}

	pDev->pTRBuff[SlaveIdx] = pBuff;
	pDev->TRBuffLen[SlaveIdx] = BuffLen;
}

typedef struct
{
	int Evt;
	int Len;
} Evt_t;

static std::vector<Evt_t> s_Evts;
static uint8_t s_SlWrBuf[16];
static uint8_t s_SlRdBuf[16];
static int s_SlWrLen;
static int s_SlRdLen;
static I2CDev_t *s_pSlave;

static int EvtHandler(DevIntrf_t * const pIntrf, DEVINTRF_EVT Evt, uint8_t *pBuf, int Len)
{
	s_Evts.push_back({ (int)Evt, Len });

	if (s_pSlave != NULL)
	{
		// The event does not say which address was used, so the buffer
		// goes to all of them.
		for (int i = 0; i < 3; i++)
		{
			if (Evt == DEVINTRF_EVT_WRITE_RQST && s_SlWrLen > 0)
			{
				I2CSetWriteRqstBuffer(s_pSlave, i, s_SlWrBuf, s_SlWrLen);
			}
			else if (Evt == DEVINTRF_EVT_READ_RQST && s_SlRdLen > 0)
			{
				I2CSetReadRqstData(s_pSlave, i, s_SlRdBuf, s_SlRdLen);
			}
		}
	}

	return 0;
}

static const IOPinCfg_t s_Pins0[2] = {
	{ 4, 1, IOPINOP_FUNC6, IOPINDIR_BI, IOPINRES_PULLUP, IOPINTYPE_OPENDRAIN },
	{ 4, 8, IOPINOP_FUNC6, IOPINDIR_BI, IOPINRES_PULLUP, IOPINTYPE_OPENDRAIN },
};

static const IOPinCfg_t s_Pins1[2] = {
	{ 2, 6, IOPINOP_FUNC6, IOPINDIR_BI, IOPINRES_PULLUP, IOPINTYPE_OPENDRAIN },
	{ 2, 5, IOPINOP_FUNC6, IOPINDIR_BI, IOPINRES_PULLUP, IOPINTYPE_OPENDRAIN },
};

static I2CCfg_t MasterCfg(int DevNo, bool bInt, uint32_t Rate)
{
	I2CCfg_t cfg = {};

	cfg.DevNo = DevNo;
	cfg.Type = I2CTYPE_STANDARD;
	cfg.Mode = I2CMODE_MASTER;
	cfg.pIOPinMap = DevNo == 0 ? s_Pins0 : s_Pins1;
	cfg.NbIOPins = 2;
	cfg.Rate = Rate;
	cfg.MaxRetry = 2;
	cfg.AddrType = I2CADDR_TYPE_NORMAL;
	cfg.bIntEn = bInt;
	cfg.IntPrio = 3;
	cfg.EvtCB = bInt ? EvtHandler : NULL;

	return cfg;
}

static I2CCfg_t SlaveCfg(int DevNo, int NbAddr, uint8_t A0, uint8_t A1, uint8_t A2)
{
	I2CCfg_t cfg = MasterCfg(DevNo, true, 100000);

	cfg.Mode = I2CMODE_SLAVE;
	cfg.NbSlaveAddr = NbAddr;
	cfg.SlaveAddr[0] = A0;
	cfg.SlaveAddr[1] = A1;
	cfg.SlaveAddr[2] = A2;

	return cfg;
}

static void WorldReset(void)
{
	memset(s_Iic, 0, sizeof(s_Iic));
	memset(s_Slot, 0, sizeof(s_Slot));
	s_Mstpcrb = 0xFFFFFFFFU;
	s_Prcr = 0;
	s_Trace.clear();
	s_Evts.clear();
	s_Tick = 1;
	s_IsrLatency = 0;
	s_bSclStuck = false;
	s_ArbLossAtByte = -1;
	s_MasterByteNo = 0;
	s_Overrun = 0;
	s_Violations = 0;
	s_ViolationLog.clear();
	s_Pclkb = 24000000;
	s_pSlave = NULL;
	g_TestPrimask = 0;
	DevReset();
	memset(s_PinGpio, 0, sizeof(s_PinGpio));
	memset(s_PinLatch, 1, sizeof(s_PinLatch));
	s_SdaHeldPulses = 0;
	s_SclRises = 0;
	s_PinStops = 0;
}

static void CheckClean(const char *pWhere)
{
	Check(s_Violations == 0, "%s: model violations: %s", pWhere, s_ViolationLog.c_str());
	Check(s_Overrun == 0, "%s: overrun %d", pWhere, s_Overrun);
	s_Violations = 0;
	s_ViolationLog.clear();
}

static void Idle(int Ticks)
{
	for (int i = 0; i < Ticks; i++)
	{
		Advance();
		TestIrqPoint();
	}
}

static bool WaitEvt(size_t N, int MaxTicks = 200000)
{
	for (int i = 0; i < MaxTicks && s_Evts.size() < N; i++)
	{
		Idle(1);
	}

	return s_Evts.size() >= N;
}

static void CheckTrace(const char *pWhat, const char *pExp)
{
	Check(s_Trace == pExp, "%s: trace \"%s\" expected \"%s\"", pWhat, s_Trace.c_str(), pExp);
	s_Trace.clear();
}

static void AddDev(int Slot, int Addr)
{
	s_Dev[Slot].bPresent = true;
	s_Dev[Slot].Addr = Addr;
}

// ---------------------------------------------------------------------------
// Tests
// ---------------------------------------------------------------------------

static void TestInit(void)
{
	WorldReset();

	I2CDev_t dev = {};
	I2CCfg_t good = MasterCfg(0, false, 100000);

	Check(!I2CInit(NULL, &good), "NULL device accepted");
	Check(!I2CInit(&dev, NULL), "NULL config accepted");

	I2CCfg_t c = good;
	c.DevNo = 2;
	Check(!I2CInit(&dev, &c), "DevNo 2 accepted");
	c = good; c.DevNo = -1;
	Check(!I2CInit(&dev, &c), "DevNo -1 accepted");
	c = good; c.Type = I2CTYPE_SMBUS;
	Check(!I2CInit(&dev, &c), "SMBus accepted");
	c = good; c.AddrType = I2CADDR_TYPE_EXT;
	Check(!I2CInit(&dev, &c), "10 bit address accepted");
	c = good; c.bDmaEn = true;
	Check(!I2CInit(&dev, &c), "DMA accepted");
	c = good; c.Rate = 0;
	Check(!I2CInit(&dev, &c), "rate 0 accepted");
	c = good; c.Rate = 1000000;
	Check(!I2CInit(&dev, &c), "1 MHz accepted");
	c = good; c.NbIOPins = 1;
	Check(!I2CInit(&dev, &c), "one pin accepted");
	c = good; c.pIOPinMap = NULL;
	Check(!I2CInit(&dev, &c), "no pin map accepted");
	c = good; c.MaxRetry = -1;
	Check(!I2CInit(&dev, &c), "negative retry accepted");
	c = good; c.pIOPinMap = s_Pins1;
	Check(!I2CInit(&dev, &c), "IIC1 pins on IIC0 accepted");

	IOPinCfg_t badpins[2] = { s_Pins0[0], s_Pins0[1] };
	c = good; c.pIOPinMap = badpins;
	badpins[0].PinOp = IOPINOP_FUNC5;
	Check(!I2CInit(&dev, &c), "wrong pin function accepted");
	badpins[0] = s_Pins0[0];
	badpins[1].Type = IOPINTYPE_NORMAL;
	Check(!I2CInit(&dev, &c), "push pull accepted");
	badpins[1] = s_Pins0[0];
	Check(!I2CInit(&dev, &c), "same pin for SDA and SCL accepted");

	c = SlaveCfg(1, 1, 0x2A, 0, 0); c.bIntEn = false;
	Check(!I2CInit(&dev, &c), "slave without interrupt accepted");
	c = SlaveCfg(1, 0, 0, 0, 0);
	Check(!I2CInit(&dev, &c), "slave without address accepted");
	c = SlaveCfg(1, 4, 0x2A, 0x2B, 0x2C); c.SlaveAddr[3] = 0x2D;
	Check(!I2CInit(&dev, &c), "four slave addresses accepted");
	c = SlaveCfg(1, 1, 0x07, 0, 0);
	Check(!I2CInit(&dev, &c), "reserved low address accepted");
	c = SlaveCfg(1, 1, 0x78, 0, 0);
	Check(!I2CInit(&dev, &c), "reserved high address accepted");
	c = SlaveCfg(1, 2, 0x2A, 0x2A, 0);
	Check(!I2CInit(&dev, &c), "duplicate slave address accepted");
	Check(SlotCount() == 0, "failed init left %d ICU slots", SlotCount());
	Check((s_Mstpcrb & 0x300U) == 0x300U, "failed init left a module running");

	Check(I2CInit(&dev, &good), "master init failed");
	Check((s_Mstpcrb & 0x200U) == 0, "IIC0 module stop not released");
	Check((s_Mstpcrb & 0x100U) != 0, "IIC1 module released by IIC0 init");
	Check(Ice(s_Iic[0]), "ICE not set after init");
	Check(SlotCount() == 0, "polling master took ICU slots");
	I2CDev_t dev2 = {};
	Check(!I2CInit(&dev2, &good), "second init of IIC0 accepted");
	CheckClean("init");

	I2CPowerOff(&dev);
	Check((s_Mstpcrb & 0x200U) != 0, "power off left the module running");
	Check(I2CInit(&dev2, &good), "init after power off failed");
	I2CPowerOff(&dev2);

	// Interrupt master takes four slots and gives them back
	I2CCfg_t ic = MasterCfg(0, true, 400000);
	Check(I2CInit(&dev, &ic), "interrupt master init failed");
	Check(SlotCount() == 4, "interrupt master used %d slots", SlotCount());
	I2CPowerOff(&dev);
	Check(SlotCount() == 0, "power off left %d slots", SlotCount());
	CheckClean("init2");
}

static void TestRate(void)
{
	static const uint32_t pclk[] = { 12000000, 24000000, 32000000, 48000000 };
	static const uint32_t rates[] = { 10000, 50000, 100000, 250000, 400000 };

	for (uint32_t clk : pclk)
	{
		for (uint32_t rate : rates)
		{
			WorldReset();
			s_Pclkb = clk;

			I2CDev_t dev = {};
			I2CCfg_t cfg = MasterCfg(0, false, rate);
			const bool ok = I2CInit(&dev, &cfg);

			if (!ok)
			{
				// Too slow for the divider range is allowed, a fast rate is not
				Check(rate <= 10000, "rate %u at %u Hz rejected", rate, clk);
				continue;
			}

			const uint32_t actual = I2CGetRate(&dev);
			const uint8_t cks = (s_Iic[0].icmr1 >> 4) & 7U;
			const uint32_t off = cks == 0 ? 3 : 2;
			const uint32_t lo = ((s_Iic[0].icbrl & 0x1FU) + off) << cks;
			const uint32_t hi = ((s_Iic[0].icbrh & 0x1FU) + off) << cks;
			const uint32_t hw = clk / (lo + hi);

			Check(actual == hw, "rate %u at %u: reported %u, registers give %u", rate, clk, actual, hw);
			Check(actual <= rate, "rate %u at %u: %u exceeds the request", rate, clk, actual);
			Check(actual * 10U >= rate * 7U, "rate %u at %u: %u far below the request", rate, clk, actual);

			const uint64_t lowNs = (uint64_t)lo * 1000000000ULL / clk;
			const uint64_t highNs = (uint64_t)hi * 1000000000ULL / clk;
			const uint32_t minLow = rate <= 100000 ? 4700 : 1300;
			const uint32_t minHigh = rate <= 100000 ? 4000 : 600;

			Check(lowNs >= minLow, "rate %u at %u: low %u ns", rate, clk, (unsigned)lowNs);
			Check(highNs >= minHigh, "rate %u at %u: high %u ns", rate, clk, (unsigned)highNs);
			CheckClean("rate");
			I2CPowerOff(&dev);
		}
	}

	WorldReset();

	I2CDev_t dev = {};
	I2CCfg_t cfg = MasterCfg(0, false, 100000);

	Check(I2CInit(&dev, &cfg), "init");

	const int r = I2CSetRate(&dev, 400000);

	Check(r > 100000 && r <= 400000, "SetRate 400k gave %d", r);
	Check(I2CGetRate(&dev) == r, "GetRate differs after SetRate");
	Check(I2CSetRate(&dev, 2000000) == 0, "SetRate above 400k accepted");
	Check(I2CGetRate(&dev) == r, "failed SetRate changed the rate");

	Check(I2CStartTx(&dev, 0x50), "StartTx");
	Check(I2CSetRate(&dev, 100000) == 0, "SetRate accepted inside a transfer");
	I2CStopTx(&dev);
	Check(I2CSetRate(&dev, 100000) > 0, "SetRate after the transfer");
	CheckClean("setrate");
	I2CPowerOff(&dev);
}

static void TestMasterPolling(void)
{
	WorldReset();
	AddDev(0, 0x50);

	I2CDev_t dev = {};
	I2CCfg_t cfg = MasterCfg(0, false, 100000);

	Check(I2CInit(&dev, &cfg), "init");

	// Plain writes
	for (int len : { 1, 2, 3, 7 })
	{
		uint8_t d[8];

		for (int i = 0; i < len; i++)
		{
			d[i] = (uint8_t)(0x10 + i);
		}

		s_Trace.clear();
		const int n = I2CTx(&dev, 0x50, d, len);
		std::string exp = "S W50+";
		char b[16];

		for (int i = 0; i < len; i++)
		{
			snprintf(b, sizeof(b), " d%02X+", d[i]);
			exp += b;
		}

		exp += " P";
		Check(n == len, "write %d returned %d", len, n);
		CheckTrace("write", exp.c_str());
		Check(s_Dev[0].Written.size() == (size_t)len, "device got %zu bytes", s_Dev[0].Written.size());
		Check((s_Iic[0].iccr2 & M_BBSY) == 0, "bus busy after write");
	}

	// Register write: address then data
	{
		const uint8_t reg[1] = { 0x20 };
		const uint8_t data[3] = { 1, 2, 3 };
		const int n = I2CWrite(&dev, 0x50, (uint8_t *)reg, 1, (uint8_t *)data, 3);

		Check(n == 3, "register write returned %d", n);
		CheckTrace("regwrite", "S W50+ d20+ d01+ d02+ d03+ P");
	}

	// Reads of every length, with a restart after the register address
	for (int len : { 1, 2, 3, 4, 5, 8, 16 })
	{
		s_Dev[0].ReadData.clear();
		s_Dev[0].ReadPos = 0;
		s_Dev[0].Reads = 0;

		for (int i = 0; i < len; i++)
		{
			s_Dev[0].ReadData.push_back((uint8_t)(0xC0 + i));
		}

		uint8_t reg = 0x05;
		uint8_t buf[16] = {};
		const int n = I2CRead(&dev, 0x50, &reg, 1, buf, len);
		std::string exp = "S W50+ d05+ Sr R50+";
		char b[16];

		for (int i = 0; i < len; i++)
		{
			snprintf(b, sizeof(b), " r%02X%c", 0xC0 + i, i == len - 1 ? '-' : '+');
			exp += b;
		}

		exp += " P";
		Check(n == len, "read %d returned %d", len, n);
		CheckTrace("read", exp.c_str());
		Check(s_Dev[0].Reads == len, "read %d clocked %d bytes", len, s_Dev[0].Reads);

		bool same = true;

		for (int i = 0; i < len; i++)
		{
			same = same && buf[i] == 0xC0 + i;
		}

		Check(same, "read %d data differs", len);
		Check((s_Iic[0].iccr2 & M_BBSY) == 0, "bus busy after read");
		Check((s_Iic[0].icmr3 & (MR3_ACKBT | MR3_WAIT)) == 0, "ACKBT or WAIT left set after read %d", len);
	}

	// Read without a register address
	{
		s_Dev[0].ReadData = { 0x11, 0x22 };
		s_Dev[0].ReadPos = 0;

		uint8_t buf[2] = {};
		const int n = I2CRx(&dev, 0x50, buf, 2);

		Check(n == 2 && buf[0] == 0x11 && buf[1] == 0x22, "plain read %d", n);
		CheckTrace("plainread", "S R50+ r11+ r22- P");
	}

	// Absent device
	{
		const uint8_t d[1] = { 1 };
		const int n = I2CTx(&dev, 0x33, (uint8_t *)d, 1);

		Check(n == 0, "write to absent device returned %d", n);
		Check((s_Iic[0].iccr2 & M_BBSY) == 0, "bus busy after address NACK");
		Check(s_Trace.find("W33-") != std::string::npos, "no NACK in trace");
		Check(s_Trace.find("d01") == std::string::npos, "data sent after address NACK");
		s_Trace.clear();

		uint8_t buf[2];
		const int r = I2CRx(&dev, 0x33, buf, 2);

		Check(r == 0, "read from absent device returned %d", r);
		Check((s_Iic[0].iccr2 & M_BBSY) == 0, "bus busy after read address NACK");
		s_Trace.clear();
	}

	// NACK on the third data byte
	{
		s_Dev[0].AckLimit = 2;

		const uint8_t d[5] = { 1, 2, 3, 4, 5 };
		const int n = I2CTx(&dev, 0x50, (uint8_t *)d, 5);

		Check(n == 2, "write refused at byte 3 returned %d", n);
		Check(s_Trace.find("d03-") != std::string::npos, "byte 3 not refused in trace");
		Check(s_Trace.find("d04") == std::string::npos, "byte 4 sent after a NACK");
		Check((s_Iic[0].iccr2 & M_BBSY) == 0, "bus busy after data NACK");
		s_Dev[0].AckLimit = -1;
		s_Trace.clear();
	}

	// BBSY left set before the first transfer is cleared by a reset
	{
		s_Iic[0].iccr2 |= M_BBSY;

		const uint8_t d[2] = { 3, 4 };
		const int n = I2CTx(&dev, 0x50, (uint8_t *)d, 2);

		Check(n == 2, "write with BBSY left set returned %d", n);
		CheckTrace("bbsy", "S W50+ d03+ d04+ P");
	}

	// Register dump text
	{
		char txt[400];
		const int n = Ra4m1I2cDump(&dev.DevIntrf, txt, sizeof(txt));

		Check(n > 0 && n == (int)strlen(txt), "dump length %d", n);
		Check(strstr(txt, " cr1=9") != NULL, "dump has no cr1: %s", txt);
		Check(strstr(txt, " fer=77") != NULL, "dump has no fer=77: %s", txt);
		Check(strstr(txt, " sdapfs=") != NULL && strstr(txt, " sclpfs=") != NULL,
			"dump has no pin registers: %s", txt);
		char small[8];
		Check(Ra4m1I2cDump(&dev.DevIntrf, small, sizeof(small)) < 8, "dump overran");
	}

	// Still usable after the errors
	{
		const uint8_t d[2] = { 9, 8 };
		const int n = I2CTx(&dev, 0x50, (uint8_t *)d, 2);

		Check(n == 2, "write after errors returned %d", n);
		CheckTrace("after", "S W50+ d09+ d08+ P");
	}

	CheckClean("master polling");
	I2CPowerOff(&dev);
}

static void TestMasterAsync(void)
{
	WorldReset();
	AddDev(0, 0x50);

	I2CDev_t dev = {};
	I2CCfg_t cfg = MasterCfg(0, true, 400000);

	Check(I2CInit(&dev, &cfg), "init");

	for (long lat : { 0L, 1L, 7L, 40L })
	{
		s_IsrLatency = lat;

		// Write
		s_Evts.clear();
		s_Trace.clear();

		uint8_t d[4] = { 0xA1, 0xA2, 0xA3, 0xA4 };
		int n = I2CTx(&dev, 0x50, d, 4);

		Check(n == -1, "async write returned %d", n);
		d[0] = d[1] = d[2] = d[3] = 0;		// the driver owns a copy
		Check(WaitEvt(1), "async write: no completion at latency %ld", lat);
		Idle(60);
		Check(s_Evts.size() == 1 && s_Evts[0].Evt == DEVINTRF_EVT_COMPLETED && s_Evts[0].Len == 4,
			"async write event count %zu len %d", s_Evts.size(), s_Evts.empty() ? -1 : s_Evts[0].Len);
		CheckTrace("async write", "S W50+ dA1+ dA2+ dA3+ dA4+ P");
		Check(dev.DevIntrf.bTxReady, "bTxReady not set after async write");
		Check(!atomic_flag_test_and_set(&dev.DevIntrf.bBusy), "bBusy held after async write");
		atomic_flag_clear(&dev.DevIntrf.bBusy);
		Check(s_Iic[0].icier == 0, "ICIER %02X left after async write", s_Iic[0].icier);

		// Read with a register address
		for (int len : { 1, 2, 3, 6 })
		{
			s_Evts.clear();
			s_Dev[0].ReadData.clear();
			s_Dev[0].ReadPos = 0;
			s_Dev[0].Reads = 0;

			for (int i = 0; i < len; i++)
			{
				s_Dev[0].ReadData.push_back((uint8_t)(0x60 + i));
			}

			uint8_t reg = 0x01;
			uint8_t buf[8] = {};

			n = I2CRead(&dev, 0x50, &reg, 1, buf, len);
			Check(n == -1, "async read returned %d", n);
			Check(WaitEvt(1), "async read %d: no completion at latency %ld", len, lat);
			Idle(60);

			std::string exp = "S W50+ d01+ Sr R50+";
			char b[16];
			bool same = true;

			for (int i = 0; i < len; i++)
			{
				snprintf(b, sizeof(b), " r%02X%c", 0x60 + i, i == len - 1 ? '-' : '+');
				exp += b;
				same = same && buf[i] == 0x60 + i;
			}

			exp += " P";
			Check(s_Evts.size() == 1 && s_Evts[0].Len == len, "async read %d event len %d", len,
				s_Evts.empty() ? -1 : s_Evts[0].Len);
			CheckTrace("async read", exp.c_str());
			Check(same, "async read %d data", len);
			Check(s_Dev[0].Reads == len, "async read %d clocked %d", len, s_Dev[0].Reads);
			Check(!atomic_flag_test_and_set(&dev.DevIntrf.bBusy), "bBusy held after async read");
			atomic_flag_clear(&dev.DevIntrf.bBusy);
		}

		// NACK on the address
		s_Evts.clear();
		n = I2CTx(&dev, 0x33, d, 2);
		Check(n == -1, "async write to absent returned %d", n);
		Check(WaitEvt(1), "async address NACK: no completion");
		Idle(60);
		Check(s_Evts.size() == 1 && s_Evts[0].Len == 0, "async address NACK len %d",
			s_Evts.empty() ? -1 : s_Evts[0].Len);
		Check((s_Iic[0].iccr2 & M_BBSY) == 0, "bus busy after async NACK");
		s_Trace.clear();

		// NACK in the data
		s_Evts.clear();
		s_Dev[0].AckLimit = 1;
		n = I2CTx(&dev, 0x50, d, 3);
		Check(n == -1, "async refused write returned %d", n);
		Check(WaitEvt(1), "async data NACK: no completion");
		Idle(60);
		Check(s_Evts.size() == 1 && s_Evts[0].Len == 1, "async data NACK len %d",
			s_Evts.empty() ? -1 : s_Evts[0].Len);
		s_Dev[0].AckLimit = -1;
		s_Trace.clear();
	}

	// A write larger than the copy buffer runs synchronously
	{
		s_Evts.clear();

		uint8_t big[100];

		for (int i = 0; i < 100; i++)
		{
			big[i] = (uint8_t)i;
		}

		const int n = I2CTx(&dev, 0x50, big, 100);

		Check(n == 100, "100 byte write returned %d", n);
		Check(s_Dev[0].Written.size() == 100, "device got %zu bytes", s_Dev[0].Written.size());
		s_Trace.clear();
	}

	CheckClean("master async");
	I2CPowerOff(&dev);
}

static void TestMasterFaults(void)
{
	WorldReset();
	AddDev(0, 0x50);

	I2CDev_t dev = {};
	I2CCfg_t cfg = MasterCfg(0, false, 400000);

	Check(I2CInit(&dev, &cfg), "init");

	// SCL stuck low: the IIC timeout fires, the driver gives up and recovers
	s_bSclStuck = true;

	const uint8_t d[3] = { 1, 2, 3 };
	int n = I2CTx(&dev, 0x50, (uint8_t *)d, 3);

	Check(n == 0, "stuck bus write returned %d", n);
	s_bSclStuck = false;
	s_Trace.clear();
	Check((s_Iic[0].iccr2 & M_BBSY) == 0, "bus busy after the timeout");

	n = I2CTx(&dev, 0x50, (uint8_t *)d, 3);
	Check(n == 3, "write after the timeout returned %d", n);
	CheckTrace("after timeout", "S W50+ d01+ d02+ d03+ P");

	// Arbitration loss during a write
	s_MasterByteNo = 0;
	s_ArbLossAtByte = 3;
	n = I2CTx(&dev, 0x50, (uint8_t *)d, 3);
	Check(n < 3, "write with arbitration loss returned %d", n);
	s_ArbLossAtByte = -1;
	s_Trace.clear();
	Check((s_Iic[0].iccr2 & M_BBSY) == 0, "bus busy after arbitration loss");

	n = I2CTx(&dev, 0x50, (uint8_t *)d, 2);
	Check(n == 2, "write after arbitration loss returned %d", n);
	s_Trace.clear();

	// A stuck read does not hang either
	s_Dev[0].ReadData = { 1, 2, 3 };
	s_Dev[0].ReadPos = 0;
	s_bSclStuck = true;

	uint8_t buf[3];

	n = I2CRx(&dev, 0x50, buf, 3);
	Check(n < 3, "stuck read returned %d", n);
	s_bSclStuck = false;
	s_Trace.clear();
	s_Dev[0].ReadPos = 0;
	n = I2CRx(&dev, 0x50, buf, 3);
	Check(n == 3, "read after the timeout returned %d", n);
	s_Trace.clear();
	CheckClean("faults");
	I2CPowerOff(&dev);
}

static void TestReset(void)
{
	WorldReset();
	AddDev(0, 0x50);

	I2CDev_t dev = {};
	I2CCfg_t cfg = MasterCfg(0, false, 100000);

	Check(I2CInit(&dev, &cfg), "init");
	s_SclPort = 4; s_SclPin = 8; s_SdaPort = 4; s_SdaPin = 1;

	// A slave holds SDA low until it has seen 5 clock pulses
	s_SdaHeldPulses = 5;
	s_SclRises = 0;
	s_PinStops = 0;
	DeviceIntrfReset(&dev.DevIntrf);

	Check(s_SdaHeldPulses == 0, "bus not freed, %d pulses left", s_SdaHeldPulses);
	Check(s_SclRises >= 5 && s_SclRises <= 10, "reset gave %d SCL pulses", s_SclRises);
	Check(s_PinStops >= 1, "reset gave no STOP");
	Check(!s_PinGpio[4][1] && !s_PinGpio[4][8], "pins not back to the IIC function");
	Check(Ice(s_Iic[0]), "ICE not set after reset");

	const uint8_t d[2] = { 5, 6 };

	Check(I2CTx(&dev, 0x50, (uint8_t *)d, 2) == 2, "write after reset");
	s_Trace.clear();

	// Bus that never frees: 9 pulses at most
	s_SdaHeldPulses = 100;
	s_SclRises = 0;
	DeviceIntrfReset(&dev.DevIntrf);
	Check(s_SclRises <= 10, "stuck bus: %d pulses", s_SclRises);
	s_SdaHeldPulses = 0;

	// Disable and Enable
	I2CDisable(&dev);
	Check(!Ice(s_Iic[0]), "ICE set after disable");
	I2CEnable(&dev);
	Check(Ice(s_Iic[0]), "ICE clear after enable");
	Check(I2CTx(&dev, 0x50, (uint8_t *)d, 2) == 2, "write after re-enable");
	s_Trace.clear();
	CheckClean("reset");
	I2CPowerOff(&dev);
}

// ---- Slave ----

static void ExpectEvents(const char *pWhat, std::initializer_list<Evt_t> Exp)
{
	bool same = s_Evts.size() == Exp.size();

	if (same)
	{
		size_t i = 0;

		for (const Evt_t &e : Exp)
		{
			same = same && s_Evts[i].Evt == e.Evt && s_Evts[i].Len == e.Len;
			i++;
		}
	}

	if (!same)
	{
		std::string got;
		char b[24];

		for (const Evt_t &e : s_Evts)
		{
			snprintf(b, sizeof(b), "[%d:%d]", e.Evt, e.Len);
			got += b;
		}

		Check(false, "%s: events %s, expected %zu", pWhat, got.c_str(), Exp.size());
	}
	else
	{
		Check(true, "%s", pWhat);
	}

	s_Evts.clear();
}

#define EV_WR		DEVINTRF_EVT_WRITE_RQST
#define EV_RD		DEVINTRF_EVT_READ_RQST
#define EV_DONE		DEVINTRF_EVT_COMPLETED

static void SlaveWrite(int Unit, int Addr, const uint8_t *pData, int Len, int *pAcked)
{
	int acked = 0;

	BmStart(Unit, false);

	if (BmAddress(Unit, Addr, false))
	{
		for (int i = 0; i < Len; i++)
		{
			if (BmWrite(Unit, pData[i]))
			{
				acked++;
			}
		}
	}

	BmStop(Unit);
	Idle(40);

	if (pAcked != NULL)
	{
		*pAcked = acked;
	}
}

static int SlaveRead(int Unit, int Addr, uint8_t *pBuf, int Len, bool bRestart)
{
	int got = 0;

	if (bRestart)
	{
		BmStart(Unit, true);
	}
	else
	{
		BmStart(Unit, false);
	}

	if (BmAddress(Unit, Addr, true))
	{
		for (int i = 0; i < Len; i++)
		{
			const int b = BmRead(Unit, i < Len - 1);

			if (b < 0)
			{
				break;
			}

			pBuf[got++] = (uint8_t)b;
		}
	}

	BmStop(Unit);
	Idle(40);

	return got;
}

static void TestSlave(long Latency)
{
	WorldReset();
	s_IsrLatency = Latency;

	I2CDev_t dev = {};
	I2CCfg_t cfg = SlaveCfg(1, 1, 0x2A, 0, 0);

	cfg.EvtCB = EvtHandler;
	Check(I2CInit(&dev, &cfg), "slave init");
	s_pSlave = &dev;
	Check(SlotCount() == 4, "slave used %d slots", SlotCount());
	Check((s_Iic[1].icser & 7U) == 1 && (s_Iic[1].sarl[0] >> 1) == 0x2A, "slave address registers");
	Check((s_Iic[1].icmr3 & MR3_WAIT) != 0, "slave WAIT not set");
	Check(s_Iic[1].icier == IER_RIE, "idle slave ICIER %02X", s_Iic[1].icier);

	// Write
	s_SlWrLen = 8;

	const uint8_t wr[12] = { 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12 };
	int acked = 0;

	memset(s_SlWrBuf, 0, sizeof(s_SlWrBuf));
	SlaveWrite(1, 0x2A, wr, 4, &acked);
	Check(acked == 4, "write 4: %d acked", acked);
	Check(memcmp(s_SlWrBuf, wr, 4) == 0, "write 4 data");
	ExpectEvents("write 4", { { EV_WR, 0 }, { EV_DONE, 4 } });
	CheckTrace("write 4", "S W2A+ d01+ d02+ d03+ d04+ P");
	Check(s_Iic[1].icier == IER_RIE, "ICIER %02X after stop", s_Iic[1].icier);
	Check((s_Iic[1].icmr3 & MR3_ACKBT) == 0, "ACKBT left set");

	// Write exactly the buffer size, then more
	memset(s_SlWrBuf, 0, sizeof(s_SlWrBuf));
	SlaveWrite(1, 0x2A, wr, 8, &acked);
	Check(acked == 8, "write 8: %d acked", acked);
	ExpectEvents("write 8", { { EV_WR, 0 }, { EV_DONE, 8 } });
	s_Trace.clear();

	memset(s_SlWrBuf, 0, sizeof(s_SlWrBuf));
	SlaveWrite(1, 0x2A, wr, 12, &acked);
	Check(acked == 8, "write 12 into 8: %d acked", acked);
	Check(memcmp(s_SlWrBuf, wr, 8) == 0, "write 12 data");
	ExpectEvents("write 12", { { EV_WR, 0 }, { EV_DONE, 8 } });
	Check(s_Trace.find("d09-") != std::string::npos, "byte 9 not refused: %s", s_Trace.c_str());
	s_Trace.clear();
	Check((s_Iic[1].icmr3 & MR3_ACKBT) == 0, "ACKBT left set after overflow");

	// Address only
	SlaveWrite(1, 0x2A, wr, 0, &acked);
	ExpectEvents("write 0", { { EV_WR, 0 }, { EV_DONE, 0 } });
	CheckTrace("write 0", "S W2A+ P");

	// No buffer: the first byte is refused
	s_SlWrLen = 0;
	SlaveWrite(1, 0x2A, wr, 3, &acked);
	Check(acked == 0, "write without buffer: %d acked", acked);
	ExpectEvents("write nobuf", { { EV_WR, 0 }, { EV_DONE, 0 } });
	s_Trace.clear();
	Check((s_Iic[1].icmr3 & MR3_ACKBT) == 0, "ACKBT left set after refusal");
	s_SlWrLen = 8;

	// Other address: no event, NACK
	SlaveWrite(1, 0x2B, wr, 2, &acked);
	Check(acked == 0, "foreign address acked");
	ExpectEvents("foreign", {});
	s_Trace.clear();

	// Read
	for (int i = 0; i < 16; i++)
	{
		s_SlRdBuf[i] = (uint8_t)(0xB0 + i);
	}

	uint8_t buf[16];
	int got;

	s_SlRdLen = 6;
	got = SlaveRead(1, 0x2A, buf, 6, false);
	Check(got == 6 && memcmp(buf, s_SlRdBuf, 6) == 0, "read 6: got %d", got);
	ExpectEvents("read 6", { { EV_RD, 0 }, { EV_DONE, 6 } });
	CheckTrace("read 6", "S R2A+ rB0+ rB1+ rB2+ rB3+ rB4+ rB5- P");

	got = SlaveRead(1, 0x2A, buf, 1, false);
	Check(got == 1 && buf[0] == 0xB0, "read 1: got %d", got);
	ExpectEvents("read 1", { { EV_RD, 0 }, { EV_DONE, 1 } });
	s_Trace.clear();

	got = SlaveRead(1, 0x2A, buf, 3, false);
	Check(got == 3, "read 3 of 6: got %d", got);
	ExpectEvents("read 3 of 6", { { EV_RD, 0 }, { EV_DONE, 3 } });
	s_Trace.clear();

	// Read past the data: filler, count stays
	got = SlaveRead(1, 0x2A, buf, 9, false);
	Check(got == 9 && buf[6] == 0xFF && buf[8] == 0xFF, "read 9 of 6: got %d", got);
	ExpectEvents("read 9 of 6", { { EV_RD, 0 }, { EV_DONE, 6 } });
	s_Trace.clear();

	// Read with no data installed
	s_SlRdLen = 0;
	got = SlaveRead(1, 0x2A, buf, 2, false);
	Check(got == 2 && buf[0] == 0xFF, "read nodata: got %d", got);
	ExpectEvents("read nodata", { { EV_RD, 0 }, { EV_DONE, 0 } });
	s_Trace.clear();
	s_SlRdLen = 6;

	// Register read: write one byte, restart, read
	{
		BmStart(1, false);
		Check(BmAddress(1, 0x2A, false), "reg read: address");
		Check(BmWrite(1, 0x07), "reg read: register byte");
		BmStart(1, true);
		Check(BmAddress(1, 0x2A, true), "reg read: read address");

		int n = 0;

		for (int i = 0; i < 4; i++)
		{
			const int b = BmRead(1, i < 3);

			if (b >= 0)
			{
				buf[n++] = (uint8_t)b;
			}
		}

		BmStop(1);
		Idle(40);
		Check(n == 4 && memcmp(buf, s_SlRdBuf, 4) == 0, "reg read: %d bytes", n);
		ExpectEvents("reg read", { { EV_WR, 0 }, { EV_RD, 1 }, { EV_DONE, 4 } });
		CheckTrace("reg read", "S W2A+ d07+ Sr R2A+ rB0+ rB1+ rB2+ rB3- P");
	}

	// Write, restart, write
	{
		s_SlWrLen = 8;
		BmStart(1, false);
		Check(BmAddress(1, 0x2A, false), "wr-wr: address");
		Check(BmWrite(1, 0x11), "wr-wr: byte");
		Check(BmWrite(1, 0x12), "wr-wr: byte");
		BmStart(1, true);
		Check(BmAddress(1, 0x2A, false), "wr-wr: second address");
		Check(BmWrite(1, 0x21), "wr-wr: byte");
		BmStop(1);
		Idle(40);
		ExpectEvents("wr-wr", { { EV_WR, 0 }, { EV_DONE, 2 }, { EV_WR, 0 }, { EV_DONE, 1 } });
		s_Trace.clear();
	}

	// Read, restart, read
	{
		BmStart(1, false);
		Check(BmAddress(1, 0x2A, true), "rd-rd: address");
		Check(BmRead(1, true) == 0xB0, "rd-rd: byte");
		Check(BmRead(1, false) == 0xB1, "rd-rd: byte");
		BmStart(1, true);
		Check(BmAddress(1, 0x2A, true), "rd-rd: second address");
		Check(BmRead(1, false) == 0xB0, "rd-rd: byte");
		BmStop(1);
		Idle(40);
		ExpectEvents("rd-rd", { { EV_RD, 0 }, { EV_DONE, 2 }, { EV_RD, 0 }, { EV_DONE, 1 } });
		s_Trace.clear();
	}

	// Many back to back
	for (int i = 0; i < 20; i++)
	{
		SlaveWrite(1, 0x2A, wr, 1 + i % 8, &acked);
		Check(acked == 1 + i % 8, "loop write %d", i);
		got = SlaveRead(1, 0x2A, buf, 1 + i % 6, false);
		Check(got == 1 + i % 6, "loop read %d", i);
	}

	s_Evts.clear();
	s_Trace.clear();
	CheckClean("slave");

	// Disable stops answering
	I2CDisable(&dev);
	Check(!Ice(s_Iic[1]), "slave ICE set after disable");
	I2CEnable(&dev);
	Check(Ice(s_Iic[1]), "slave ICE clear after enable");
	SlaveWrite(1, 0x2A, wr, 2, &acked);
	Check(acked == 2, "write after re-enable: %d", acked);
	s_Evts.clear();
	s_Trace.clear();

	I2CPowerOff(&dev);
	Check(SlotCount() == 0, "slave power off left %d slots", SlotCount());
	CheckClean("slave2");
}

static void TestSlaveMulti(void)
{
	WorldReset();

	I2CDev_t dev = {};
	I2CCfg_t cfg = SlaveCfg(1, 3, 0x2A, 0x3B, 0x4C);

	cfg.EvtCB = EvtHandler;
	Check(I2CInit(&dev, &cfg), "multi init");
	s_pSlave = &dev;
	s_SlWrLen = 8;
	s_SlRdLen = 3;
	s_SlRdBuf[0] = 7; s_SlRdBuf[1] = 8; s_SlRdBuf[2] = 9;
	Check((s_Iic[1].icser & 7U) == 7, "ICSER %02X", s_Iic[1].icser);

	const uint8_t wr[2] = { 0x5A, 0xA5 };
	uint8_t buf[4];
	int acked;

	for (int a : { 0x2A, 0x3B, 0x4C })
	{
		SlaveWrite(1, a, wr, 2, &acked);
		Check(acked == 2, "multi write %02X: %d", a, acked);
		ExpectEvents("multi write", { { EV_WR, 0 }, { EV_DONE, 2 } });
		const int got = SlaveRead(1, a, buf, 3, false);
		Check(got == 3 && buf[2] == 9, "multi read %02X: %d", a, got);
		ExpectEvents("multi read", { { EV_RD, 0 }, { EV_DONE, 3 } });
	}

	SlaveWrite(1, 0x4D, wr, 2, &acked);
	Check(acked == 0, "multi foreign acked");
	s_Trace.clear();
	CheckClean("multi");
	I2CPowerOff(&dev);
}

// Two devices at once: IIC0 master in polling mode and IIC1 slave
static void TestBothUnits(void)
{
	WorldReset();
	AddDev(0, 0x50);

	I2CDev_t m = {}, s = {};
	I2CCfg_t mc = MasterCfg(0, false, 100000);
	I2CCfg_t sc = SlaveCfg(1, 1, 0x2A, 0, 0);

	sc.EvtCB = EvtHandler;
	Check(I2CInit(&m, &mc), "master init");
	Check(I2CInit(&s, &sc), "slave init");
	s_pSlave = &s;
	s_SlWrLen = 8;

	const uint8_t d[3] = { 1, 2, 3 };
	int acked;

	Check(I2CTx(&m, 0x50, (uint8_t *)d, 3) == 3, "master write");
	s_Trace.clear();
	SlaveWrite(1, 0x2A, d, 3, &acked);
	Check(acked == 3, "slave write");
	Check(I2CTx(&m, 0x50, (uint8_t *)d, 3) == 3, "master write 2");
	s_Trace.clear();
	s_Evts.clear();
	CheckClean("both");
	I2CPowerOff(&m);
	I2CPowerOff(&s);
}

int main(void)
{
	TestInit();
	TestRate();
	TestMasterPolling();
	TestMasterAsync();
	TestMasterFaults();
	TestReset();

	for (long lat : { 0L, 1L, 5L, 30L })
	{
		TestSlave(lat);
	}

	TestSlaveMulti();
	TestBothUnits();

	printf("%d checks, %d failed, %d interrupts taken\n", s_Checks, s_Fails, s_IsrCount);
	printf(s_Fails == 0 ? "I2C host test PASS\n" : "I2C host test FAIL\n");

	return s_Fails == 0 ? 0 : 1;
}
