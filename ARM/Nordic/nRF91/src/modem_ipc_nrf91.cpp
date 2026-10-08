/**-------------------------------------------------------------------------
@file	modem_ipc_nrf91.cpp

@brief	DeviceIntrf over the IPC link to the nRF91 modem, see modem_ipc_nrf91.h.

The transfers call the Modem library: nrf_modem_at for MODEM_IPC_ADDR_AT,
nrf_modem_gnss for MODEM_IPC_ADDR_GNSS. The library callbacks run in the
modem interrupt and go to the configuration EvtCB.

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
#include <string.h>

#include "nrf_modem.h"
#include "nrf_modem_at.h"
#include "nrf_modem_gnss.h"
#include "nrf_errno.h"

#include "modem_ipc_nrf91.h"

/** @addtogroup LTE
  * @{
  */

// The device the library callbacks report to: one modem
static ModemIpcIntrfDev_t *s_pModemIpcDev = nullptr;

// Last PVT frame, written in the modem interrupt
static struct nrf_modem_gnss_pvt_data_frame s_ModemIpcPvt;

static void ModemIpcEvt(ModemIpcIntrfDev_t * const pDev, DEVINTRF_EVT EvtId, uint32_t DevAddr,
						int32_t Id, const void *pData, int DataLen)
{
	if (pDev->DevIntrf.EvtCB != nullptr)
	{
		ModemIpcEvt_t evt = { DevAddr, Id, pData, DataLen };

		pDev->DevIntrf.EvtCB(&pDev->DevIntrf, EvtId, (uint8_t *)&evt, sizeof(evt));
	}
}

// Modem library interrupt
static void ModemIpcGnssEvt(int Evt)
{
	ModemIpcIntrfDev_t *dev = s_pModemIpcDev;

	if (dev == nullptr)
	{
		return;
	}

	if (Evt == NRF_MODEM_GNSS_EVT_PVT)
	{
		if (nrf_modem_gnss_read(&s_ModemIpcPvt, sizeof(s_ModemIpcPvt), NRF_MODEM_GNSS_DATA_PVT) == 0)
		{
			ModemIpcEvt(dev, DEVINTRF_EVT_RX_DATA, MODEM_IPC_ADDR_GNSS, Evt, &s_ModemIpcPvt,
						sizeof(s_ModemIpcPvt));
		}
	}
	else
	{
		ModemIpcEvt(dev, DEVINTRF_EVT_STATECHG, MODEM_IPC_ADDR_GNSS, Evt, nullptr, 0);
	}
}

// Modem library interrupt
static void ModemIpcFault(struct nrf_modem_fault_info *pInfo)
{
	if (s_pModemIpcDev != nullptr)
	{
		ModemIpcEvt(s_pModemIpcDev, DEVINTRF_EVT_STATECHG, MODEM_IPC_ADDR_MODEM,
					pInfo != nullptr ? (int32_t)pInfo->reason : 0, pInfo,
					pInfo != nullptr ? (int)sizeof(*pInfo) : 0);
	}
}

// The parameter of a GNSS command, little endian
static uint32_t ModemIpcParam(const uint8_t *pData, int Len)
{
	uint32_t v = 0;

	for (int i = Len - 1; i >= 0; i--)
	{
		v = (v << 8) | pData[i];
	}

	return v;
}

// A GNSS command: the result of the library call, -NRF_EINVAL for an unknown
// command or a parameter of the wrong size
static int32_t ModemIpcGnssCmd(const uint8_t *pData, int DataLen)
{
	const uint8_t *p = pData + 1;
	int len = DataLen - 1;

	switch (pData[0])
	{
		case MODEM_IPC_GNSS_START:
			return len == 0 ? nrf_modem_gnss_start() : -NRF_EINVAL;

		case MODEM_IPC_GNSS_STOP:
			return len == 0 ? nrf_modem_gnss_stop() : -NRF_EINVAL;

		case MODEM_IPC_GNSS_SIGNAL:
			return len == 1 ? nrf_modem_gnss_signal_mask_set((uint8_t)ModemIpcParam(p, len)) : -NRF_EINVAL;

		case MODEM_IPC_GNSS_FIX_RETRY:
			return len == 2 ? nrf_modem_gnss_fix_retry_set((uint16_t)ModemIpcParam(p, len)) : -NRF_EINVAL;

		case MODEM_IPC_GNSS_FIX_INTERVAL:
			return len == 2 ? nrf_modem_gnss_fix_interval_set((uint16_t)ModemIpcParam(p, len)) : -NRF_EINVAL;

		case MODEM_IPC_GNSS_DYN:
			return len == 4 ? nrf_modem_gnss_dyn_mode_change(ModemIpcParam(p, len)) : -NRF_EINVAL;

		case MODEM_IPC_GNSS_NV_DELETE:
			return len == 4 ? nrf_modem_gnss_nv_data_delete(ModemIpcParam(p, len)) : -NRF_EINVAL;
	}

	return -NRF_EINVAL;
}

// The modem power is set by its functional mode (AT+CFUN): nothing to do for
// the interface itself
static void ModemIpcDisable(DevIntrf_t * const pDevIntrf)
{
	(void)pDevIntrf;
}

static void ModemIpcEnable(DevIntrf_t * const pDevIntrf)
{
	(void)pDevIntrf;
}

static uint32_t ModemIpcGetRate(DevIntrf_t * const pDevIntrf)
{
	(void)pDevIntrf;

	return 0;
}

static uint32_t ModemIpcSetRate(DevIntrf_t * const pDevIntrf, uint32_t Rate)
{
	(void)pDevIntrf;
	(void)Rate;

	return 0;
}

static bool ModemIpcStartRx(DevIntrf_t * const pDevIntrf, uint32_t DevAddr)
{
	ModemIpcIntrfDev_t *dev = (ModemIpcIntrfDev_t *)pDevIntrf->pDevData;

	if (DevAddr != MODEM_IPC_ADDR_AT && DevAddr != MODEM_IPC_ADDR_GNSS)
	{
		return false;
	}

	// A read opens its receive phase on the selector of its address phase:
	// the address phase stays
	if (DevAddr != dev->DevAddr)
	{
		dev->CmdLen = 0;
	}
	dev->DevAddr = DevAddr;

	return true;
}

static bool ModemIpcStartTx(DevIntrf_t * const pDevIntrf, uint32_t DevAddr)
{
	ModemIpcIntrfDev_t *dev = (ModemIpcIntrfDev_t *)pDevIntrf->pDevData;

	if (DevAddr != MODEM_IPC_ADDR_AT && DevAddr != MODEM_IPC_ADDR_GNSS)
	{
		return false;
	}

	dev->CmdLen = 0;
	dev->DevAddr = DevAddr;

	return true;
}

static int ModemIpcRxData(DevIntrf_t * const pDevIntrf, uint8_t *pBuff, int BuffLen)
{
	ModemIpcIntrfDev_t *dev = (ModemIpcIntrfDev_t *)pDevIntrf->pDevData;

	if (pBuff == nullptr || BuffLen <= 0)
	{
		return 0;
	}

	if (dev->DevAddr == MODEM_IPC_ADDR_AT)
	{
		if (dev->CmdLen <= 0 || nrf_modem_at_cmd(pBuff, (size_t)BuffLen, "%s", dev->Cmd) != 0)
		{
			return 0;
		}
		pBuff[BuffLen - 1] = 0;

		return (int)strlen((char *)pBuff);
	}

	if ((size_t)BuffLen < sizeof(struct nrf_modem_gnss_pvt_data_frame) ||
		nrf_modem_gnss_read(pBuff, BuffLen, NRF_MODEM_GNSS_DATA_PVT) != 0)
	{
		return 0;
	}

	return (int)sizeof(struct nrf_modem_gnss_pvt_data_frame);
}

static void ModemIpcStop(DevIntrf_t * const pDevIntrf)
{
	ModemIpcIntrfDev_t *dev = (ModemIpcIntrfDev_t *)pDevIntrf->pDevData;

	dev->CmdLen = 0;
}

static int ModemIpcTxData(DevIntrf_t * const pDevIntrf, const uint8_t *pData, int DataLen)
{
	ModemIpcIntrfDev_t *dev = (ModemIpcIntrfDev_t *)pDevIntrf->pDevData;

	if (pData == nullptr || DataLen <= 0)
	{
		return 0;
	}

	if (dev->DevAddr == MODEM_IPC_ADDR_GNSS)
	{
		return ModemIpcGnssCmd(pData, DataLen) == 0 ? DataLen : 0;
	}

	if (DataLen > MODEM_IPC_AT_CMD_MAX)
	{
		return 0;
	}

	memcpy(dev->Cmd, pData, DataLen);
	dev->Cmd[DataLen] = 0;

	return nrf_modem_at_printf("%s", dev->Cmd) == 0 ? DataLen : 0;
}

// Address phase of a read: kept for the receive phase
static int ModemIpcTxSrData(DevIntrf_t * const pDevIntrf, const uint8_t *pData, int DataLen)
{
	ModemIpcIntrfDev_t *dev = (ModemIpcIntrfDev_t *)pDevIntrf->pDevData;

	if (pData == nullptr || DataLen <= 0 || DataLen > MODEM_IPC_AT_CMD_MAX)
	{
		dev->CmdLen = 0;

		return 0;
	}

	memcpy(dev->Cmd, pData, DataLen);
	dev->Cmd[DataLen] = 0;
	dev->CmdLen = DataLen;

	return DataLen;
}

static void ModemIpcReset(DevIntrf_t * const pDevIntrf)
{
	(void)pDevIntrf;
}

// The modem off and the Modem library shut down when Init started it: Init
// starts it again. A modem started by another user is left to it.
static void ModemIpcPowerOff(DevIntrf_t * const pDevIntrf)
{
	ModemIpcIntrfDev_t *dev = (ModemIpcIntrfDev_t *)pDevIntrf->pDevData;

	if (dev->bStarted == false)
	{
		return;
	}

	if (nrf_modem_is_initialized())
	{
		// The modem stores its data at CFUN=0, required before the shutdown
		(void)nrf_modem_at_printf("AT+CFUN=0");
		nrf_modem_shutdown();
	}
	dev->bStarted = false;
}

static void *ModemIpcGetHandle(DevIntrf_t * const pDevIntrf)
{
	return pDevIntrf->pDevData;
}

bool ModemIpcIntrfInit(ModemIpcIntrfDev_t * const pDev, const ModemIpcIntrfCfg_t * const pCfg)
{
	if (pDev == nullptr || pCfg == nullptr)
	{
		return false;
	}

	// No event to a device being set up
	s_pModemIpcDev = nullptr;

	pDev->DevIntrf.pDevData = pDev;
	pDev->DevIntrf.IntPrio = pCfg->IntPrio;
	pDev->DevIntrf.EvtCB = pCfg->EvtCB;
	atomic_flag_clear(&pDev->DevIntrf.bBusy);
	// A refused command is an answer, not a transfer to retry
	pDev->DevIntrf.MaxRetry = 0;
	atomic_store(&pDev->DevIntrf.EnCnt, 0);
	pDev->DevIntrf.Type = DEVINTRF_TYPE_CEL;
	pDev->DevIntrf.bDma = false;
	pDev->DevIntrf.bIntEn = true;
	atomic_store(&pDev->DevIntrf.bTxReady, true);
	atomic_store(&pDev->DevIntrf.bNoStop, false);
	pDev->DevIntrf.Disable = ModemIpcDisable;
	pDev->DevIntrf.Enable = ModemIpcEnable;
	pDev->DevIntrf.GetRate = ModemIpcGetRate;
	pDev->DevIntrf.SetRate = ModemIpcSetRate;
	pDev->DevIntrf.StartRx = ModemIpcStartRx;
	pDev->DevIntrf.RxData = ModemIpcRxData;
	pDev->DevIntrf.StopRx = ModemIpcStop;
	pDev->DevIntrf.StartTx = ModemIpcStartTx;
	pDev->DevIntrf.TxData = ModemIpcTxData;
	pDev->DevIntrf.TxSrData = ModemIpcTxSrData;
	pDev->DevIntrf.StopTx = ModemIpcStop;
	pDev->DevIntrf.Reset = ModemIpcReset;
	pDev->DevIntrf.PowerOff = ModemIpcPowerOff;
	pDev->DevIntrf.GetHandle = ModemIpcGetHandle;
	pDev->DevAddr = MODEM_IPC_ADDR_AT;
	pDev->CmdLen = 0;

	// Not initialized: off, or stopped by a fault. After a fault the library
	// is shut down before it is started again.
	if (nrf_modem_is_initialized() == false)
	{
		if (pDev->bStarted)
		{
			nrf_modem_shutdown();
			pDev->bStarted = false;
		}

		memset(&pDev->ModemCfg, 0, sizeof(pDev->ModemCfg));
		pDev->ModemCfg.Fw = NRF91_MODEM_FW_CELLULAR;
		pDev->ModemCfg.IntPrio = pCfg->IntPrio;
		pDev->ModemCfg.pTimer = pCfg->pTimer;
		pDev->ModemCfg.TimerTrigNo = pCfg->TimerTrigNo;
		pDev->ModemCfg.FaultHandler = ModemIpcFault;

		if (nRF91ModemInit(&pDev->ModemCfg) != 0)
		{
			return false;
		}
		pDev->bStarted = true;
	}

	s_pModemIpcDev = pDev;

	return nrf_modem_gnss_event_handler_set(ModemIpcGnssEvt) == 0;
}

/** @} End of group LTE */
