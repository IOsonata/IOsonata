/**-------------------------------------------------------------------------
@file	lte.cpp

@brief	Generic LTE subsystem: URC FIFO, 3GPP configuration and URC parsing.

The parts of the LTE subsystem that are the same for every modem, see
lte.h. The port reports URCs and faults from the modem interrupt with
LteUrcPut and LteFaultPut. One work event, queued through LteEvtQue, takes
them out of the interrupt: it parses the URCs, updates the status and calls
the configuration handlers.

Timer and cycle coding from 3GPP 24.008: periodic TAU is a GPRS Timer 3
(10.5.7.4a), active time a GPRS Timer 2 (10.5.7.3), both sent as 8 bit
strings, eDRX a 4 bit value (10.5.5.32).

@author	Hoang Nguyen Hoan
@date	Oct. 6, 2026

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
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "app_evt_handler.h"
#include "coredev/interrupt.h"
#include "lte/lte.h"

/** @addtogroup LTE
  * @{
  */

// Longest configuration command, an APN is up to 100 characters
#define LTE_CMD_LEN_MAX				160

// Fields of a URC or a response line kept by the parser
#define LTE_FIELD_MAX				10

// Timer value strings: 8 bits and the terminating 0
#define LTE_TIMER_STR_LEN			9

// Unit bits of a GPRS timer that mean deactivated
#define LTE_TIMER_DEACTIVATED		7U

// Access technology of +CEREG
#define LTE_CEREG_ACT_LTEM			7
#define LTE_CEREG_ACT_NBIOT			9

// AcT-type of +CEDRXS and +CEDRXP
#define LTE_EDRX_ACT_LTEM			4
#define LTE_EDRX_ACT_NBIOT			5

// RSRP and RSRQ of +CESQ: offsets of the index to dBm and to half dB
#define LTE_CESQ_RSRP_OFFSET		140
#define LTE_CESQ_RSRQ_OFFSET		39
#define LTE_CESQ_UNKNOWN			255

// State of the queued work, so that it is queued once and queued again
// when LteEvtQue refused it
enum {
	LTE_WORK_IDLE,
	LTE_WORK_PENDING,
	LTE_WORK_QUEUED,
};

#pragma pack(push, 4)

typedef struct {
	LteCfg_t Cfg;
	hCFifo_t hUrc;
	LteStatus_t Status;
	uint32_t UrcLostSeen;				// UrcLost when the state was last read back
	volatile uint8_t WorkState;
	volatile bool bFault;
} LteCore_t;

#pragma pack(pop)

// URC FIFO memory with the default size, replaced by the application
// definition of the same name when it has its own, see lte.h
alignas(4) __attribute__((weak)) uint8_t g_LteUrcMem[LTE_URC_MEMSIZE_DEFAULT];

// GPRS Timer 3 units in s, index is the unit bits, 0 for deactivated
static const uint32_t s_LteTauUnit[8] = {
	600, 3600, 36000, 2, 30, 60, 1152000, 0
};

// Periodic TAU units from the shortest
static const uint8_t s_LteTauOrder[7] = { 3, 4, 5, 0, 1, 2, 6 };

// GPRS Timer 2 units in s. 3 to 6 are read as minutes, 7 is deactivated.
static const uint32_t s_LteActiveUnit[8] = {
	2, 60, 360, 60, 60, 60, 60, 0
};

// eDRX cycle in ms of each 4 bit value, LTE-M
static const uint32_t s_LteEdrxCycle[16] = {
	5120, 10240, 20480, 40960, 61440, 81920, 102400, 122880,
	143360, 163840, 327680, 655360, 1310720, 2621440, 5242880, 10485760
};

// Values each technology accepts. NB-IoT reads the others as 20.48 s,
// LTE-M reads 1110 and 1111 as 1101.
static const uint16_t s_LteEdrxNbIot = 0xFE2C;
static const uint16_t s_LteEdrxLteM = 0x3FFF;

// Technologies with their +CEDRXS AcT-type, for the eDRX request
static const LTE_RAT s_LteEdrxRat[2] = { LTE_RAT_LTEM, LTE_RAT_NBIOT };
static const int s_LteEdrxAct[2] = { LTE_EDRX_ACT_LTEM, LTE_EDRX_ACT_NBIOT };

// PDN type names of +CGDCONT, indexed by LTE_PDN
static const char * const s_LtePdnName[3] = { "IPV4V6", "IP", "IPV6" };

// State read back after lost URCs: registration, then RRC
static const char * const s_LteResyncCmd[2] = { "AT+CEREG?", "AT+CSCON?" };
static const char * const s_LteResyncPrefix[2] = { "+CEREG:", "+CSCON:" };

static LteCore_t s_Lte;

// ---------------------------------------------------------------------------
// Timer and cycle coding
// ---------------------------------------------------------------------------

static void LteTimerStr(uint32_t Bits, char pStr[LTE_TIMER_STR_LEN])
{
	for (int i = 0; i < 8; i++)
	{
		pStr[i] = (Bits & (0x80U >> i)) ? '1' : '0';
	}
	pStr[8] = 0;
}

// Value of an 8 bit string, -1 when it is not one
static int LteTimerBits(const char *pStr)
{
	uint32_t bits = 0;

	for (int i = 0; i < 8; i++)
	{
		if (pStr[i] != '0' && pStr[i] != '1')
		{
			return -1;
		}
		bits = (bits << 1) | (uint32_t)(pStr[i] - '0');
	}

	return pStr[8] == 0 ? (int)bits : -1;
}

// Smallest periodic TAU at least Sec, the longest when none is
static void LtePsmTauStr(uint32_t Sec, char pStr[LTE_TIMER_STR_LEN])
{
	uint32_t bits = (6U << 5) | 31U;

	for (int i = 0; i < (int)sizeof(s_LteTauOrder); i++)
	{
		uint32_t unit = s_LteTauUnit[s_LteTauOrder[i]];
		uint64_t n = ((uint64_t)Sec + unit - 1) / unit;

		if (n <= 31)
		{
			bits = ((uint32_t)s_LteTauOrder[i] << 5) | (uint32_t)n;
			break;
		}
	}

	LteTimerStr(bits, pStr);
}

// Smallest active time at least Sec, the longest when none is
static void LtePsmActiveStr(uint32_t Sec, char pStr[LTE_TIMER_STR_LEN])
{
	uint32_t bits = (2U << 5) | 31U;

	for (uint32_t u = 0; u < 3; u++)
	{
		uint64_t n = ((uint64_t)Sec + s_LteActiveUnit[u] - 1) / s_LteActiveUnit[u];

		if (n <= 31)
		{
			bits = (u << 5) | (uint32_t)n;
			break;
		}
	}

	LteTimerStr(bits, pStr);
}

// Seconds of a GPRS timer string, false when deactivated or not a timer
static bool LteTimerSec(const char *pStr, const uint32_t pUnit[8], uint32_t *pSec)
{
	int bits = LteTimerBits(pStr);

	if (bits < 0 || ((uint32_t)bits >> 5) == LTE_TIMER_DEACTIVATED)
	{
		return false;
	}

	*pSec = pUnit[(uint32_t)bits >> 5] * ((uint32_t)bits & 31U);

	return true;
}

// Shortest eDRX cycle at least Ms the technology accepts, the longest when
// none is
static uint32_t LteEdrxValue(uint32_t Ms, LTE_RAT Rat)
{
	uint16_t allowed = Rat == LTE_RAT_NBIOT ? s_LteEdrxNbIot : s_LteEdrxLteM;
	uint32_t last = 0;

	for (uint32_t v = 0; v < 16; v++)
	{
		if ((allowed & (1U << v)) == 0)
		{
			continue;
		}
		if (s_LteEdrxCycle[v] >= Ms)
		{
			return v;
		}
		last = v;
	}

	return last;
}

// Value of a 4 bit string, -1 when it is not one
static int LteEdrxBits(const char *pStr)
{
	if (strlen(pStr) != 4)
	{
		return -1;
	}

	int bits = 0;

	for (int i = 0; i < 4; i++)
	{
		if (pStr[i] != '0' && pStr[i] != '1')
		{
			return -1;
		}
		bits = (bits << 1) | (pStr[i] - '0');
	}

	return bits;
}

// ---------------------------------------------------------------------------
// URC parsing
// ---------------------------------------------------------------------------

// Split the arguments of a URC or response in place: fields separated by
// commas, quotes and spaces around them removed. Returns the field count.
static int LteSplit(char *pArgs, char *pField[LTE_FIELD_MAX])
{
	int n = 0;
	char *p = pArgs;

	while (n < LTE_FIELD_MAX)
	{
		while (*p == ' ')
		{
			p++;
		}

		char *start = p;
		bool quoted = *p == '"';

		if (quoted)
		{
			start = ++p;
			while (*p != 0 && *p != '"')
			{
				p++;
			}
			if (*p == '"')
			{
				*p++ = 0;
			}
		}

		while (*p != 0 && *p != ',')
		{
			p++;
		}

		bool last = *p == 0;

		*p = 0;
		if (quoted == false)
		{
			char *e = p;

			while (e > start && (e[-1] == ' ' || e[-1] == '\r' || e[-1] == '\n'))
			{
				*--e = 0;
			}
		}
		pField[n++] = start;
		if (last)
		{
			break;
		}
		p++;
	}

	return n;
}

static bool LteRegStat(LTE_REG Stat)
{
	return Stat == LTE_REG_HOME || Stat == LTE_REG_ROAMING;
}

static void LteEvt(LTE_EVT Evt)
{
	if (s_Lte.Cfg.EvtHandler != nullptr)
	{
		s_Lte.Cfg.EvtHandler(Evt, &s_Lte.Status);
	}
}

// +CEREG: <stat>[,<tac>,<ci>,<AcT>[,<cause_type>,<reject_cause>[,<Active-Time>,
// <Periodic-TAU-ext>]]]
static void LteCereg(char *pArgs)
{
	char *f[LTE_FIELD_MAX];
	int n = LteSplit(pArgs, f);

	if (n < 1 || f[0][0] == 0)
	{
		return;
	}

	LteStatus_t &st = s_Lte.Status;
	LTE_REG stat = (LTE_REG)strtol(f[0], nullptr, 10);
	bool wasreg = LteRegStat(st.RegStat);
	bool isreg = LteRegStat(stat);
	LTE_REG oldstat = st.RegStat;
	uint16_t oldtac = st.Tac;
	uint32_t oldci = st.CellId;
	bool oldpsm = st.bPsm;
	uint32_t oldtau = st.PsmTau;
	uint32_t oldactive = st.PsmActive;

	st.RegStat = stat;

	if (n >= 3 && f[1][0] != 0 && f[2][0] != 0)
	{
		st.Tac = (uint16_t)strtoul(f[1], nullptr, 16);
		st.CellId = (uint32_t)strtoul(f[2], nullptr, 16);
	}

	if (n >= 4 && f[3][0] != 0)
	{
		int act = (int)strtol(f[3], nullptr, 10);

		st.Rat = act == LTE_CEREG_ACT_LTEM ? LTE_RAT_LTEM :
				 act == LTE_CEREG_ACT_NBIOT ? LTE_RAT_NBIOT : LTE_RAT_NONE;
	}

	if (n >= 8)
	{
		uint32_t active = 0;
		uint32_t tau = 0;

		st.bPsm = LteTimerSec(f[6], s_LteActiveUnit, &active) &&
				  LteTimerSec(f[7], s_LteTauUnit, &tau);
		st.PsmActive = st.bPsm ? active : 0;
		st.PsmTau = st.bPsm ? tau : 0;
	}
	else if (isreg == false)
	{
		st.bPsm = false;
		st.PsmActive = 0;
		st.PsmTau = 0;
	}

	if (isreg && wasreg == false)
	{
		LteEvt(LTE_EVT_REGISTERED);
	}
	else if (wasreg && isreg == false)
	{
		LteEvt(LTE_EVT_UNREGISTERED);
	}
	else if (isreg == false)
	{
		if (stat != oldstat)
		{
			LteEvt(LTE_EVT_REG_STATE);
		}
	}
	else
	{
		if (st.Tac != oldtac || st.CellId != oldci)
		{
			LteEvt(LTE_EVT_CELL);
		}
		if (st.bPsm != oldpsm || st.PsmTau != oldtau || st.PsmActive != oldactive)
		{
			LteEvt(LTE_EVT_PSM);
		}
	}
}

// +CSCON: <mode>
static void LteCscon(char *pArgs)
{
	char *f[LTE_FIELD_MAX];
	int n = LteSplit(pArgs, f);

	if (n < 1 || f[0][0] == 0)
	{
		return;
	}

	bool conn = strtol(f[0], nullptr, 10) == 1;

	if (conn != s_Lte.Status.bRrcConnected)
	{
		s_Lte.Status.bRrcConnected = conn;
		LteEvt(conn ? LTE_EVT_RRC_CONNECTED : LTE_EVT_RRC_IDLE);
	}
}

// +CEDRXP: <AcT-type>,<Requested_eDRX_value>,<NW-provided_eDRX_value>,
// <Paging_time_window>
static void LteCedrxp(char *pArgs)
{
	char *f[LTE_FIELD_MAX];
	int n = LteSplit(pArgs, f);

	if (n < 1)
	{
		return;
	}

	int act = (int)strtol(f[0], nullptr, 10);
	int nw = n >= 3 ? LteEdrxBits(f[2]) : -1;
	int ptw = n >= 4 ? LteEdrxBits(f[3]) : -1;
	uint32_t cycle = 0;
	uint32_t window = 0;

	if (nw >= 0 && (act == LTE_EDRX_ACT_LTEM || act == LTE_EDRX_ACT_NBIOT))
	{
		if (act == LTE_EDRX_ACT_NBIOT && (s_LteEdrxNbIot & (1U << nw)) == 0)
		{
			nw = 2;
		}
		else if (act == LTE_EDRX_ACT_LTEM && (s_LteEdrxLteM & (1U << nw)) == 0)
		{
			nw = 13;
		}
		cycle = s_LteEdrxCycle[nw];
		if (ptw >= 0)
		{
			window = (uint32_t)(ptw + 1) * (act == LTE_EDRX_ACT_LTEM ? 1280U : 2560U);
		}
	}

	if (cycle != s_Lte.Status.EdrxCycle || window != s_Lte.Status.EdrxPtw)
	{
		s_Lte.Status.EdrxCycle = cycle;
		s_Lte.Status.EdrxPtw = window;
		LteEvt(LTE_EVT_EDRX);
	}
}

static void LteUrcProcess(char *pUrc)
{
	if (strncmp(pUrc, "+CEREG:", 7) == 0)
	{
		LteCereg(pUrc + 7);
	}
	else if (strncmp(pUrc, "+CSCON:", 7) == 0)
	{
		LteCscon(pUrc + 7);
	}
	else if (strncmp(pUrc, "+CEDRXP:", 8) == 0)
	{
		LteCedrxp(pUrc + 8);
	}
	else if (s_Lte.Cfg.UrcHandler != nullptr)
	{
		s_Lte.Cfg.UrcHandler(pUrc);
	}
}

// ---------------------------------------------------------------------------
// Work out of the interrupt
// ---------------------------------------------------------------------------

// Read the registration and the RRC state back after URCs were lost:
// the read responses carry <n> before the URC fields
static void LteResync(void)
{
	char resp[LTE_URC_LEN_MAX];

	for (int i = 0; i < 2; i++)
	{
		if (LteAtCmd(s_LteResyncCmd[i], resp, sizeof(resp)) != 0)
		{
			continue;
		}

		char *p = strstr(resp, s_LteResyncPrefix[i]);
		char *e = p != nullptr ? strpbrk(p, "\r\n") : nullptr;

		if (e != nullptr)
		{
			*e = 0;
		}
		p = p != nullptr ? strchr(p, ',') : nullptr;
		if (p == nullptr)
		{
			continue;
		}
		if (i == 0)
		{
			LteCereg(p + 1);
		}
		else
		{
			LteCscon(p + 1);
		}
	}
}

static void LteWorkEvt(uint32_t EvtId, void *pCtx)
{
	(void)EvtId;
	(void)pCtx;

	// What the interrupt reports from here on queues the next event
	s_Lte.WorkState = LTE_WORK_IDLE;

	char urc[LTE_URC_LEN_MAX];
	uint8_t *p;

	// Each URC is copied out of the FIFO before it is processed: the handlers
	// may wait for AT responses while more URCs arrive.
	while (s_Lte.hUrc != nullptr && (p = CFifoPeek(s_Lte.hUrc)) != nullptr)
	{
		memcpy(urc, p, LTE_URC_LEN_MAX);
		urc[LTE_URC_LEN_MAX - 1] = 0;
		CFifoGet(s_Lte.hUrc);
		LteUrcProcess(urc);
	}

	if (s_Lte.bFault)
	{
		// The modem stopped: nothing it reported holds any more
		uint32_t lost = s_Lte.Status.UrcLost;

		s_Lte.bFault = false;
		memset(&s_Lte.Status, 0, sizeof(s_Lte.Status));
		s_Lte.Status.UrcLost = lost;
		s_Lte.UrcLostSeen = lost;
		LteEvt(LTE_EVT_MODEM_FAULT);
	}
	else if (s_Lte.Status.UrcLost != s_Lte.UrcLostSeen)
	{
		s_Lte.UrcLostSeen = s_Lte.Status.UrcLost;
		LteResync();
	}
}

static void LteWorkQue(void)
{
	uint32_t state = DisableInterrupt();

	if (s_Lte.WorkState == LTE_WORK_IDLE)
	{
		s_Lte.WorkState = LTE_WORK_PENDING;
	}
	if (s_Lte.WorkState != LTE_WORK_PENDING)
	{
		EnableInterrupt(state);

		return;
	}
	s_Lte.WorkState = LTE_WORK_QUEUED;
	EnableInterrupt(state);

	if (LteEvtQue(0, nullptr, LteWorkEvt) == false)
	{
		s_Lte.WorkState = LTE_WORK_PENDING;
	}
}

// Default for an application without an OS, see lte.h. Interrupt context.
__attribute__((weak)) bool LteEvtQue(uint32_t EvtId, void *pCtx, LteEvtQueHandler_t Handler)
{
	return AppEvtHandlerQue(EvtId, pCtx, Handler);
}

void LteCheckStatus(void)
{
	if (s_Lte.WorkState == LTE_WORK_PENDING)
	{
		LteWorkQue();
	}
}

void LteUrcPut(const char *pUrc)
{
	if (pUrc == nullptr || s_Lte.hUrc == nullptr)
	{
		return;
	}

	char *p = (char *)CFifoPut(s_Lte.hUrc);

	if (p == nullptr)
	{
		s_Lte.Status.UrcLost++;
	}
	else
	{
		int i = 0;

		while (i < LTE_URC_LEN_MAX - 1 && pUrc[i] != 0 && pUrc[i] != '\r' && pUrc[i] != '\n')
		{
			p[i] = pUrc[i];
			i++;
		}
		p[i] = 0;
	}

	LteWorkQue();
}

void LteFaultPut(void)
{
	s_Lte.bFault = true;
	LteWorkQue();
}

// ---------------------------------------------------------------------------
// Initialization and configuration
// ---------------------------------------------------------------------------

bool LteCoreInit(const LteCfg_t * const pCfg)
{
	if (pCfg == nullptr ||
		(pCfg->Rat != LTE_RAT_LTEM && pCfg->Rat != LTE_RAT_NBIOT && pCfg->Rat != LTE_RAT_LTEM_NBIOT) ||
		(pCfg->RatPref != LTE_RAT_NONE && pCfg->RatPref != LTE_RAT_LTEM && pCfg->RatPref != LTE_RAT_NBIOT) ||
		(pCfg->RatPref & ~pCfg->Rat) != 0 ||
		pCfg->NbBand < 0 || (pCfg->NbBand > 0 && pCfg->pBand == nullptr) ||
		(unsigned)pCfg->PdnType >= sizeof(s_LtePdnName) / sizeof(s_LtePdnName[0]))
	{
		return false;
	}

	uint32_t size = pCfg->UrcMemSize != 0 ? pCfg->UrcMemSize : LTE_URC_MEMSIZE_DEFAULT;

	// No URC goes into the FIFO while it is set up again
	s_Lte.hUrc = nullptr;

	hCFifo_t fifo = CFifoInit(g_LteUrcMem, size, LTE_URC_LEN_MAX, true);

	if (fifo == nullptr)
	{
		return false;
	}

	s_Lte.Cfg = *pCfg;
	s_Lte.hUrc = fifo;
	memset(&s_Lte.Status, 0, sizeof(s_Lte.Status));
	s_Lte.UrcLostSeen = 0;
	s_Lte.WorkState = LTE_WORK_IDLE;
	s_Lte.bFault = false;

	return true;
}

static bool LteCmd(const char *pCmd)
{
	return LteAtCmd(pCmd, nullptr, 0) == 0;
}

bool LteCoreConfig(void)
{
	const LteCfg_t &c = s_Lte.Cfg;
	char cmd[LTE_CMD_LEN_MAX];
	int len;

	if (c.pApn != nullptr)
	{
		len = snprintf(cmd, sizeof(cmd), "AT+CGDCONT=0,\"%s\",\"%s\"", s_LtePdnName[c.PdnType], c.pApn);
		if (len < 0 || len >= (int)sizeof(cmd) || LteCmd(cmd) == false)
		{
			return false;
		}
	}

	if (c.bPsm && c.PsmTau != 0)
	{
		char tau[LTE_TIMER_STR_LEN];
		char active[LTE_TIMER_STR_LEN];

		LtePsmTauStr(c.PsmTau, tau);
		LtePsmActiveStr(c.PsmActive, active);
		snprintf(cmd, sizeof(cmd), "AT+CPSMS=1,,,\"%s\",\"%s\"", tau, active);
	}
	else
	{
		snprintf(cmd, sizeof(cmd), "AT+CPSMS=%d", c.bPsm ? 1 : 0);
	}
	if (LteCmd(cmd) == false)
	{
		return false;
	}

	if (c.EdrxCycle != 0)
	{
		for (int i = 0; i < 2; i++)
		{
			if ((c.Rat & s_LteEdrxRat[i]) == 0)
			{
				continue;
			}

			char bits[LTE_TIMER_STR_LEN];

			LteTimerStr(LteEdrxValue(c.EdrxCycle, s_LteEdrxRat[i]), bits);
			snprintf(cmd, sizeof(cmd), "AT+CEDRXS=2,%d,\"%s\"", s_LteEdrxAct[i], &bits[4]);
			if (LteCmd(cmd) == false)
			{
				return false;
			}
		}
	}
	else if (LteCmd("AT+CEDRXS=3") == false)
	{
		return false;
	}

	return LteCmd("AT+CEREG=5") && LteCmd("AT+CSCON=1");
}

// ---------------------------------------------------------------------------
// Status
// ---------------------------------------------------------------------------

const LteStatus_t *LteGetStatus(void)
{
	return &s_Lte.Status;
}

bool LteRegistered(void)
{
	return LteRegStat(s_Lte.Status.RegStat);
}

// +CESQ: <rxlev>,<ber>,<rscp>,<ecno>,<rsrq>,<rsrp>
bool LteGetSignal(int *pRsrp, int *pRsrq)
{
	char resp[64];

	if (pRsrp == nullptr || LteAtCmd("AT+CESQ", resp, sizeof(resp)) != 0)
	{
		return false;
	}

	char *p = strstr(resp, "+CESQ:");

	if (p == nullptr)
	{
		return false;
	}

	char *e = strpbrk(p, "\r\n");

	if (e != nullptr)
	{
		*e = 0;
	}

	char *f[LTE_FIELD_MAX];

	if (LteSplit(p + 6, f) < 6)
	{
		return false;
	}

	int rsrq = (int)strtol(f[4], nullptr, 10);
	int rsrp = (int)strtol(f[5], nullptr, 10);

	if (rsrp == LTE_CESQ_UNKNOWN || (pRsrq != nullptr && rsrq == LTE_CESQ_UNKNOWN))
	{
		return false;
	}

	*pRsrp = rsrp - LTE_CESQ_RSRP_OFFSET;
	if (pRsrq != nullptr)
	{
		*pRsrq = (rsrq - LTE_CESQ_RSRQ_OFFSET) / 2;
	}

	return true;
}

/** @} End of group LTE */
