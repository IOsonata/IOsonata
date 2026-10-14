/**-------------------------------------------------------------------------
@file	gnss_nrf91.cpp

@brief	GNSS receiver of the nRF91 modem, on ModemIpcIntrf, see gnss_nrf91.h.

@author	Hoang Nguyen Hoan
@date	Oct. 8, 2026

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
#include <stdlib.h>
#include <string.h>
#include <math.h>

#include "coredev/interrupt.h"
#include "gnss_nrf91.h"

/** @addtogroup GNSS
  * @{
  */

// Fix interval of periodic fixes and fix retry, in s
#define GNSS_NRF91_PERIOD_MIN		10U
#define GNSS_NRF91_PERIOD_MAX		65535U
#define GNSS_NRF91_TIMEOUT_MAX		65535U

// Longest response of AT%XSYSTEMMODE?, AT+CFUN? and AT+CGMR
#define GNSS_NRF91_RESP_LEN			64

// nRF9160 modem firmware: the Modem library takes GNSS requests from 1.3.4 on
#define GNSS_NRF91_FW_NRF9160		"mfw_nrf9160_"
#define GNSS_NRF91_FW_MIN			((1U << 16) | (3U << 8) | 4U)
#define GNSS_NRF91_FW_FIELDS		3
#define GNSS_NRF91_FW_FIELD_MAX		255

// Fields of %XSYSTEMMODE: LTE-M, NB-IoT, GNSS, LTE preference
#define GNSS_NRF91_SYSMODE_FIELDS	4
#define GNSS_NRF91_SYSMODE_GNSS		2

// Longest AT%XSYSTEMMODE command
#define GNSS_NRF91_SYSMODE_LEN		40

// Degrees to radians
#define GNSS_NRF91_DEG_TO_RAD		((float)M_PI / 180.0f)

// A system of the modem: its GNSS_SYS bit, signal mask bit and signal id
typedef struct {
	GNSS_SYS Sys;
	uint8_t Mask;
	uint8_t Signal;
} GnssNrf91Sys_t;

static const GnssNrf91Sys_t s_GnssNrf91Sys[] = {
	{ GNSS_SYS_GPS, NRF_MODEM_GNSS_SYSTEM_GPS_L1_CA_MASK, NRF_MODEM_GNSS_SIGNAL_GPS_L1_CA },
	{ GNSS_SYS_QZSS, NRF_MODEM_GNSS_SYSTEM_QZSS_L1_CA_MASK, NRF_MODEM_GNSS_SIGNAL_QZSS_L1_CA },
	{ GNSS_SYS_GALILEO, NRF_MODEM_GNSS_SYSTEM_GAL_E1_OS_MASK, NRF_MODEM_GNSS_SIGNAL_GAL_E1_OS },
};

static const int s_GnssNrf91NbSys = sizeof(s_GnssNrf91Sys) / sizeof(s_GnssNrf91Sys[0]);

// Dynamics mode of each GNSS_DYN
static const uint32_t s_GnssNrf91Dyn[] = {
	NRF_MODEM_GNSS_DYNAMICS_GENERAL_PURPOSE,
	NRF_MODEM_GNSS_DYNAMICS_STATIONARY,
	NRF_MODEM_GNSS_DYNAMICS_PEDESTRIAN,
	NRF_MODEM_GNSS_DYNAMICS_AUTOMOTIVE,
};

GnssNrf91::GnssNrf91()
{
	memset(vSv, 0, sizeof(vSv));
}

// A receiver command on the GNSS selector, parameter little endian
bool GnssNrf91::GnssCmd(uint8_t Cmd, uint32_t Param, int ParamLen)
{
	uint8_t d[1 + sizeof(uint32_t)];

	d[0] = Cmd;
	for (int i = 0; i < ParamLen; i++)
	{
		d[1 + i] = (uint8_t)(Param >> (8 * i));
	}

	return vpIntrf->Tx(MODEM_IPC_ADDR_GNSS, d, ParamLen + 1) == ParamLen + 1;
}

bool GnssNrf91::AtCmd(const char *pCmd)
{
	int len = (int)strlen(pCmd);

	return vpIntrf->Tx(MODEM_IPC_ADDR_AT, (const uint8_t *)pCmd, len) == len;
}

// The NbVal integers, comma separated, after pPrefix in the response to pCmd
bool GnssNrf91::AtRead(const char *pCmd, const char *pPrefix, int *pVal, int NbVal)
{
	char resp[GNSS_NRF91_RESP_LEN];

	if (vpIntrf->Read(MODEM_IPC_ADDR_AT, (const uint8_t *)pCmd, (int)strlen(pCmd), (uint8_t *)resp,
					  sizeof(resp)) <= 0)
	{
		return false;
	}

	char *p = strstr(resp, pPrefix);

	if (p == nullptr)
	{
		return false;
	}
	p += strlen(pPrefix);

	for (int i = 0; i < NbVal; i++)
	{
		char *e;

		pVal[i] = (int)strtol(p, &e, 10);
		if (e == p || (i < NbVal - 1 && *e != ','))
		{
			return false;
		}
		p = e + 1;
	}

	return true;
}

// false for an nRF9160 modem firmware older than 1.3.4. Revision 1 of the
// nRF9160 runs at most 1.2.8: the Modem library refuses its GNSS settings and
// no position data arrives. Other firmware, nRF91x1 and nRF9151, passes.
bool GnssNrf91::FwSupported()
{
	char resp[GNSS_NRF91_RESP_LEN];
	const char *cmd = "AT+CGMR";

	if (vpIntrf->Read(MODEM_IPC_ADDR_AT, (const uint8_t *)cmd, (int)strlen(cmd), (uint8_t *)resp,
					  sizeof(resp)) <= 0)
	{
		return false;
	}

	char *p = strstr(resp, GNSS_NRF91_FW_NRF9160);

	if (p == nullptr)
	{
		return true;
	}
	p += strlen(GNSS_NRF91_FW_NRF9160);

	uint32_t ver = 0;

	for (int i = 0; i < GNSS_NRF91_FW_FIELDS; i++)
	{
		char *e;
		long v = strtol(p, &e, 10);

		// A version written another way is left to the receiver commands
		if (e == p || v < 0 || v > GNSS_NRF91_FW_FIELD_MAX || (i < GNSS_NRF91_FW_FIELDS - 1 && *e != '.'))
		{
			return true;
		}
		ver = (ver << 8) | (uint32_t)v;
		p = e + 1;
	}

	return ver >= GNSS_NRF91_FW_MIN;
}

bool GnssNrf91::Init(const GnssCfg_t &Cfg, DeviceIntrf * const pIntrf, Timer * const pTimer)
{
	uint32_t sys = Cfg.SysMask;

	for (int i = 0; i < s_GnssNrf91NbSys; i++)
	{
		sys &= ~(uint32_t)s_GnssNrf91Sys[i].Sys;
	}

	// Checked first: a refused configuration leaves a running receiver as it
	// is. A system the modem does not have is refused.
	if (pIntrf == nullptr || pIntrf->Type() != DEVINTRF_TYPE_CEL || sys != 0 ||
		(Cfg.FixInterval > 1 && Cfg.FixInterval < GNSS_NRF91_PERIOD_MIN) ||
		Cfg.FixInterval > GNSS_NRF91_PERIOD_MAX || Cfg.FixTimeout > GNSS_NRF91_TIMEOUT_MAX ||
		CoreInit(Cfg, pIntrf, pTimer) == false)
	{
		return false;
	}

	Valid(false);

	if (FwSupported() == false)
	{
		return false;
	}

	// GNSS in the system mode: added while the modem is off, required when
	// it runs
	int mode[GNSS_NRF91_SYSMODE_FIELDS];

	if (AtRead("AT%XSYSTEMMODE?", "%XSYSTEMMODE:", mode, GNSS_NRF91_SYSMODE_FIELDS) == false)
	{
		return false;
	}

	if (mode[GNSS_NRF91_SYSMODE_GNSS] == 0)
	{
		int fun;
		char cmd[GNSS_NRF91_SYSMODE_LEN];

		if (AtRead("AT+CFUN?", "+CFUN:", &fun, 1) == false || fun != 0)
		{
			return false;
		}

		snprintf(cmd, sizeof(cmd), "AT%%XSYSTEMMODE=%d,%d,1,%d", mode[0], mode[1], mode[3]);
		if (AtCmd(cmd) == false)
		{
			return false;
		}
	}

	for (int i = 0; i < vCfg.NbCmd; i++)
	{
		if (AtCmd(vCfg.ppCmd[i]) == false)
		{
			return false;
		}
	}

	if (Enable() == false)
	{
		return false;
	}

	// The motion model is set with the receiver running and kept by the modem
	// in its non volatile memory. A modem without it runs general purpose.
	if (GnssCmd(MODEM_IPC_GNSS_DYN, s_GnssNrf91Dyn[vCfg.Dyn], 4) == false && vCfg.Dyn != GNSS_DYN_GENERAL)
	{
		Disable();

		return false;
	}

	Valid(true);

	return true;
}

bool GnssNrf91::Enable()
{
	uint8_t mask = 0;

	if (vpIntrf == nullptr)
	{
		return false;
	}

	for (int i = 0; i < s_GnssNrf91NbSys; i++)
	{
		if (vCfg.SysMask & s_GnssNrf91Sys[i].Sys)
		{
			mask |= s_GnssNrf91Sys[i].Mask;
		}
	}

	// Settings are accepted with the receiver stopped: a stop of a receiver
	// that does not run is refused, which changes nothing. A single fix also
	// leaves the receiver to be stopped before it starts again.
	(void)GnssCmd(MODEM_IPC_GNSS_STOP, 0, 0);

	// The fix retry limits a single or a periodic fix, it has no effect on
	// a fix every second
	if (AtCmd("AT+CFUN=31") == false ||
		(mask != 0 && GnssCmd(MODEM_IPC_GNSS_SIGNAL, mask, 1) == false) ||
		(vCfg.FixInterval != 1 && GnssCmd(MODEM_IPC_GNSS_FIX_RETRY, vCfg.FixTimeout, 2) == false) ||
		GnssCmd(MODEM_IPC_GNSS_FIX_INTERVAL, vCfg.FixInterval, 2) == false ||
		GnssCmd(MODEM_IPC_GNSS_START, 0, 0) == false)
	{
		return false;
	}

	return true;
}

void GnssNrf91::Disable()
{
	if (vpIntrf != nullptr)
	{
		(void)GnssCmd(MODEM_IPC_GNSS_STOP, 0, 0);
	}
}

void GnssNrf91::Reset()
{
	Disable();
	(void)Enable();
}

int GnssNrf91::GetSat(GnssSat_t *pSat, int MaxSat)
{
	if (pSat == nullptr || MaxSat <= 0)
	{
		return 0;
	}

	int n = 0;
	uint32_t state = DisableInterrupt();

	for (int i = 0; i < NRF_MODEM_GNSS_MAX_SATELLITES && n < MaxSat; i++)
	{
		const struct nrf_modem_gnss_sv &sv = vSv[i];
		int k = 0;

		while (k < s_GnssNrf91NbSys && s_GnssNrf91Sys[k].Signal != sv.signal)
		{
			k++;
		}

		if (sv.sv == 0 || k >= s_GnssNrf91NbSys)
		{
			continue;
		}

		pSat[n].Sys = s_GnssNrf91Sys[k].Sys;
		pSat[n].Id = sv.sv;
		pSat[n].Cn0 = sv.cn0;
		pSat[n].Elev = sv.elevation;
		pSat[n].Azim = sv.azimuth;
		pSat[n].bUsed = (sv.flags & NRF_MODEM_GNSS_SV_FLAG_USED_IN_FIX) != 0;
		n++;
	}

	EnableInterrupt(state);

	return n;
}

// Interface interrupt
int GnssNrf91::IntrfEvtHandler(DEVINTRF_EVT EvtId, uint8_t *pBuffer, int Len)
{
	if (pBuffer == nullptr || Len < (int)sizeof(ModemIpcEvt_t))
	{
		return 0;
	}

	const ModemIpcEvt_t *evt = (const ModemIpcEvt_t *)pBuffer;

	if (evt->DevAddr == MODEM_IPC_ADDR_MODEM)
	{
		EvtPut(GNSS_EVT_FAULT);

		return Len;
	}

	if (evt->DevAddr != MODEM_IPC_ADDR_GNSS)
	{
		return 0;
	}

	switch (evt->Id)
	{
		case NRF_MODEM_GNSS_EVT_PVT:
			{
				if (EvtId != DEVINTRF_EVT_RX_DATA || evt->pData == nullptr ||
					evt->DataLen < (int)sizeof(struct nrf_modem_gnss_pvt_data_frame))
				{
					return 0;
				}

				const struct nrf_modem_gnss_pvt_data_frame &pvt =
					*(const struct nrf_modem_gnss_pvt_data_frame *)evt->pData;
				int tracked = 0;
				int used = 0;

				memcpy(vSv, pvt.sv, sizeof(vSv));
				for (int i = 0; i < NRF_MODEM_GNSS_MAX_SATELLITES; i++)
				{
					if (pvt.sv[i].sv != 0)
					{
						tracked++;
						if (pvt.sv[i].flags & NRF_MODEM_GNSS_SV_FLAG_USED_IN_FIX)
						{
							used++;
						}
					}
				}

				bool valid = (pvt.flags & NRF_MODEM_GNSS_PVT_FLAG_FIX_VALID) != 0;
				GnssFix_t fix;

				memset(&fix, 0, sizeof(fix));
				if (valid)
				{
					fix.Time.Year = pvt.datetime.year;
					fix.Time.Month = pvt.datetime.month;
					fix.Time.Day = pvt.datetime.day;
					fix.Time.Hour = pvt.datetime.hour;
					fix.Time.Min = pvt.datetime.minute;
					fix.Time.Sec = pvt.datetime.seconds;
					fix.Time.Msec = pvt.datetime.ms;
					fix.Lat = pvt.latitude;
					fix.Lon = pvt.longitude;
					fix.Alt = pvt.altitude;
					fix.PosAccH = pvt.accuracy;
					fix.PosAccV = pvt.altitude_accuracy;
					fix.bVelValid = (pvt.flags & NRF_MODEM_GNSS_PVT_FLAG_VELOCITY_VALID) != 0;
					if (fix.bVelValid)
					{
						// Heading is clockwise from north, vertical speed is
						// positive up
						float h = pvt.heading * GNSS_NRF91_DEG_TO_RAD;
						float cross = pvt.speed * pvt.heading_accuracy * GNSS_NRF91_DEG_TO_RAD;
						float acc = sqrtf(pvt.speed_accuracy * pvt.speed_accuracy + cross * cross);

						fix.Vel[0] = pvt.speed * cosf(h);
						fix.Vel[1] = pvt.speed * sinf(h);
						fix.Vel[2] = -pvt.vertical_speed;
						// One std-dev for the three axes: the largest of the
						// horizontal one, heading error included, and the
						// vertical one
						fix.VelAcc = acc > pvt.vertical_speed_accuracy ? acc : pvt.vertical_speed_accuracy;
					}
					fix.Hdop = pvt.hdop;
					fix.Vdop = pvt.vdop;
					fix.NbSat = (uint8_t)used;
				}

				UpdatePut(valid, &fix, tracked, used);
			}
			break;

		case NRF_MODEM_GNSS_EVT_SLEEP_AFTER_TIMEOUT:
			EvtPut(GNSS_EVT_TIMEOUT);
			break;

		case NRF_MODEM_GNSS_EVT_BLOCKED:
			EvtPut(GNSS_EVT_BLOCKED);
			break;

		case NRF_MODEM_GNSS_EVT_UNBLOCKED:
			EvtPut(GNSS_EVT_UNBLOCKED);
			break;

		default:
			break;
	}

	return Len;
}

/** @} End of group GNSS */
