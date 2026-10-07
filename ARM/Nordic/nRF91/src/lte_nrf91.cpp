/**-------------------------------------------------------------------------
@file	lte_nrf91.cpp

@brief	LTE subsystem, nRF91 port on the Modem library.

Implements the port part of lte.h: LteInit turns the modem on through
nRF91ModemInit when the application has not done it (see modem_nrf91.h),
sets the system mode (%XSYSTEMMODE), the band lock (%XBANDLOCK, runtime lock),
the 3GPP configuration of the generic layer and release assistance (%RAI),
then attaches. URCs and modem faults come from the Modem library interrupt
and go to LteUrcPut and LteFaultPut. AT commands go through
nrf_modem_at_cmd, which waits on the modem glue.

A fault is reported only when LteInit turned the modem on: an application
that does it itself gets the fault through its own nRF91ModemCfg_t handler.
LteInit after a fault shuts the Modem library down and starts it again.

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
#include <stdio.h>
#include <string.h>

#include "nrf.h"

#include "nrf_modem.h"
#include "nrf_modem_at.h"
#include "nrf_errno.h"
#include "lte/lte.h"
#include "modem_nrf91.h"

/** @addtogroup LTE
  * @{
  */

// Longest identity or version response read by LteGetInfo
#define LTE_NRF91_INFO_RESP_LEN		96

// Longest %XSYSTEMMODE command
#define LTE_NRF91_SYSMODE_LEN		40

// Highest band %XBANDLOCK takes, and the longest command
#define LTE_NRF91_BAND_MAX			88
#define LTE_NRF91_BANDLOCK_LEN		(LTE_NRF91_BAND_MAX + 24)

// %XSYSTEMMODE LTE preference: none, LTE-M, NB-IoT
#define LTE_NRF91_PREF_NONE			0
#define LTE_NRF91_PREF_LTEM			1
#define LTE_NRF91_PREF_NBIOT		2

// Commands of LteGetInfo, indexed by LTE_INFO, and the prefix of their
// response when there is one
static const char * const s_LteNrf91InfoCmd[] = {
	"AT+CGSN", "AT+CIMI", "AT%XICCID", "AT+CGMR"
};
static const char * const s_LteNrf91InfoPrefix[] = {
	nullptr, nullptr, "%XICCID:", nullptr
};

// Modem configuration when LteInit turns the modem on, kept by the Modem
// library while it runs
static nRF91ModemCfg_t s_LteNrf91ModemCfg;

// LteInit turned the modem on, so it restarts it after a fault
static bool s_bLteNrf91Started = false;

// Modem library interrupt
static void LteNrf91Urc(const char *pUrc)
{
	LteUrcPut(pUrc);
}

// Modem library interrupt
static void LteNrf91Fault(struct nrf_modem_fault_info *pInfo)
{
	(void)pInfo;

	LteFaultPut();
}

// Runtime band lock: a string of one bit per band, the highest band first,
// band 1 last. The modem drops it at CFUN=0 and at a reset, so LteInit sets
// it at each start, after its CFUN=0. LteDisconnect (CFUN=4) keeps it.
static bool LteNrf91BandLockCmd(const LteCfg_t * const pCfg, char pCmd[LTE_NRF91_BANDLOCK_LEN])
{
	int top = 0;

	for (int i = 0; i < pCfg->NbBand; i++)
	{
		if (pCfg->pBand[i] < 1 || pCfg->pBand[i] > LTE_NRF91_BAND_MAX)
		{
			return false;
		}
		if (pCfg->pBand[i] > top)
		{
			top = pCfg->pBand[i];
		}
	}

	int len = snprintf(pCmd, LTE_NRF91_BANDLOCK_LEN, "AT%%XBANDLOCK=2,\"");
	char *mask = &pCmd[len];

	memset(mask, '0', (size_t)top);
	for (int i = 0; i < pCfg->NbBand; i++)
	{
		mask[top - pCfg->pBand[i]] = '1';
	}
	mask[top] = '"';
	mask[top + 1] = 0;

	return true;
}

int LteAtCmd(const char *pCmd, char *pResp, int RespLen)
{
	if (pCmd == nullptr)
	{
		return -NRF_EFAULT;
	}

	if (pResp == nullptr || RespLen <= 0)
	{
		return nrf_modem_at_printf("%s", pCmd);
	}

	return nrf_modem_at_cmd(pResp, (size_t)RespLen, "%s", pCmd);
}

bool LteGetInfo(LTE_INFO Info, char *pBuf, int BufLen)
{
	char resp[LTE_NRF91_INFO_RESP_LEN];

	if ((unsigned)Info >= sizeof(s_LteNrf91InfoCmd) / sizeof(s_LteNrf91InfoCmd[0]) ||
		pBuf == nullptr || BufLen <= 0 ||
		LteAtCmd(s_LteNrf91InfoCmd[Info], resp, sizeof(resp)) != 0)
	{
		return false;
	}

	const char *p = resp;

	if (s_LteNrf91InfoPrefix[Info] != nullptr)
	{
		p = strstr(resp, s_LteNrf91InfoPrefix[Info]);
		if (p == nullptr)
		{
			return false;
		}
		p += strlen(s_LteNrf91InfoPrefix[Info]);
		while (*p == ' ')
		{
			p++;
		}
	}

	size_t n = strcspn(p, "\r\n");

	if (n == 0 || n >= (size_t)BufLen)
	{
		return false;
	}

	memcpy(pBuf, p, n);
	pBuf[n] = 0;

	return true;
}

bool LteConnect(void)
{
	return LteAtCmd("AT+CFUN=1", nullptr, 0) == 0;
}

bool LteDisconnect(void)
{
	return LteAtCmd("AT+CFUN=4", nullptr, 0) == 0;
}

bool LteInit(const LteCfg_t * const pCfg)
{
	char bandlock[LTE_NRF91_BANDLOCK_LEN];

	// Checked first: a refused configuration leaves a running LTE as it is
	if (pCfg == nullptr || pCfg->NbBand < 0 ||
		(pCfg->NbBand > 0 && (pCfg->pBand == nullptr || LteNrf91BandLockCmd(pCfg, bandlock) == false)))
	{
		return false;
	}

	// No URC while the generic layer sets up its FIFO again
	nrf_modem_at_notif_handler_set(nullptr);

	if (LteCoreInit(pCfg) == false)
	{
		// Refused before anything changed: the earlier configuration holds
		nrf_modem_at_notif_handler_set(LteNrf91Urc);

		return false;
	}

	// Not initialized: off, or stopped by a fault. After a fault the library
	// is shut down before it is started again.
	if (nrf_modem_is_initialized() == false)
	{
		if (s_bLteNrf91Started)
		{
			nrf_modem_shutdown();
			s_bLteNrf91Started = false;
		}

		s_LteNrf91ModemCfg = {};
		s_LteNrf91ModemCfg.Fw = NRF91_MODEM_FW_CELLULAR;
		s_LteNrf91ModemCfg.IntPrio = pCfg->IntPrio;
		s_LteNrf91ModemCfg.pTimer = pCfg->pTimer;
		s_LteNrf91ModemCfg.TimerTrigNo = pCfg->TimerTrigNo;
		s_LteNrf91ModemCfg.FaultHandler = LteNrf91Fault;

		if (nRF91ModemInit(&s_LteNrf91ModemCfg) != 0)
		{
			return false;
		}
		s_bLteNrf91Started = true;
	}

	int pref = LTE_NRF91_PREF_NONE;

	if (pCfg->Rat == LTE_RAT_LTEM_NBIOT)
	{
		pref = pCfg->RatPref == LTE_RAT_LTEM ? LTE_NRF91_PREF_LTEM :
			   pCfg->RatPref == LTE_RAT_NBIOT ? LTE_NRF91_PREF_NBIOT : LTE_NRF91_PREF_NONE;
	}

	char sysmode[LTE_NRF91_SYSMODE_LEN];

	snprintf(sysmode, sizeof(sysmode), "AT%%XSYSTEMMODE=%d,%d,%d,%d",
			 (pCfg->Rat & LTE_RAT_LTEM) ? 1 : 0, (pCfg->Rat & LTE_RAT_NBIOT) ? 1 : 0,
			 pCfg->bGnss ? 1 : 0, pref);

	// System mode, band lock and PDN settings are accepted with the radio off
	if (nrf_modem_at_notif_handler_set(LteNrf91Urc) != 0 ||
		LteAtCmd("AT+CFUN=0", nullptr, 0) != 0 ||
		LteAtCmd(sysmode, nullptr, 0) != 0 ||
		(pCfg->NbBand > 0 && LteAtCmd(bandlock, nullptr, 0) != 0) ||
		LteCoreConfig() == false)
	{
		return false;
	}

	// Release assistance is kept by the modem: turned off when not asked
	// for, where the modem firmware has it
	if (pCfg->bRai)
	{
		if (LteAtCmd("AT%RAI=1", nullptr, 0) != 0)
		{
			return false;
		}
	}
	else
	{
		(void)LteAtCmd("AT%RAI=0", nullptr, 0);
	}

	return LteConnect();
}

/** @} End of group LTE */
