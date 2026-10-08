/**-------------------------------------------------------------------------
@file	modem_ipc_nrf91.h

@brief	DeviceIntrf over the IPC link to the nRF91 modem.

The application CPU of the nRF91 reaches its modem through the IPC peripheral
and shared RAM, run by the Modem library. This interface presents that link
as a DeviceIntrf, so that a device inside the modem is used like a device on
a bus: the DevAddr selector chooses the modem service.

MODEM_IPC_ADDR_AT, AT commands:
	Tx		the command text, without line ending. Returns its length when
			the modem answers OK, 0 otherwise.
	Read	the command in the address phase, the response in the data phase,
			0 terminated. Returns the response length when the modem answers
			OK, 0 otherwise.

MODEM_IPC_ADDR_GNSS, the GNSS receiver:
	Tx		a MODEM_IPC_GNSS_* command byte followed by its parameter, little
			endian. Returns the length when the receiver accepts it, 0
			otherwise. Settings are accepted with the receiver stopped, the
			motion model with the receiver running.
	Rx		the last PVT frame (struct nrf_modem_gnss_pvt_data_frame).

Events go to the configuration EvtCB from the modem interrupt. pBuffer points
to a ModemIpcEvt_t and Len is its size:
	DEVINTRF_EVT_RX_DATA	an event with data, the PVT frame of
							NRF_MODEM_GNSS_EVT_PVT
	DEVINTRF_EVT_STATECHG	an event without data: the other
							NRF_MODEM_GNSS_EVT_* of the receiver, and a modem
							fault on MODEM_IPC_ADDR_MODEM

Init turns the modem on (nRF91ModemInit) when nothing did before, and then
reports its faults. When the Modem library was initialized before, the
interface uses it as it is and the faults go to the handler of that
initialization. The calls wait for the modem: from the main loop or the
handlers it runs, never from an interrupt.

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
#ifndef __MODEM_IPC_NRF91_H__
#define __MODEM_IPC_NRF91_H__

#include <stdint.h>
#include <stdbool.h>

#include "nrf_modem_gnss.h"
#include "device_intrf.h"
#include "modem_nrf91.h"

/** @addtogroup LTE
  * @{
  */

/// DevAddr selectors: the modem services
#define MODEM_IPC_ADDR_AT			0U		//!< AT commands
#define MODEM_IPC_ADDR_GNSS			1U		//!< GNSS receiver
#define MODEM_IPC_ADDR_MODEM		2U		//!< The modem itself, events only (faults)

/// Commands of MODEM_IPC_ADDR_GNSS, the parameter follows the command byte
#define MODEM_IPC_GNSS_START		1U		//!< Start, no parameter
#define MODEM_IPC_GNSS_STOP			2U		//!< Stop, no parameter
#define MODEM_IPC_GNSS_SIGNAL		3U		//!< Signals used, uint8_t NRF_MODEM_GNSS_SYSTEM_*_MASK bits
#define MODEM_IPC_GNSS_FIX_RETRY	4U		//!< Fix retry in s, uint16_t
#define MODEM_IPC_GNSS_FIX_INTERVAL	5U		//!< Fix interval in s, uint16_t
#define MODEM_IPC_GNSS_DYN			6U		//!< Motion model, uint32_t NRF_MODEM_GNSS_DYNAMICS_*
#define MODEM_IPC_GNSS_NV_DELETE	7U		//!< Delete stored data, uint32_t NRF_MODEM_GNSS_DELETE_* bits

/// Longest AT command, without the terminating 0
#define MODEM_IPC_AT_CMD_MAX		256

#pragma pack(push, 4)

/// Event to the configuration EvtCB, see the file description
typedef struct __Modem_Ipc_Evt {
	uint32_t DevAddr;				//!< Service reporting it, MODEM_IPC_ADDR_*
	int32_t Id;						//!< NRF_MODEM_GNSS_EVT_* on MODEM_IPC_ADDR_GNSS, fault reason
									//!< on MODEM_IPC_ADDR_MODEM
	const void *pData;				//!< Data of the event, NULL for none
	int DataLen;					//!< Size of pData
} ModemIpcEvt_t;

typedef struct __Modem_Ipc_Intrf_Config {
	int IntPrio;					//!< IPC interrupt priority
	TimerDev_t *pTimer;				//!< Time base of the waits for the modem, NULL for 1 ms
									//!< busy steps
	int TimerTrigNo;				//!< Trigger of pTimer kept for the waits
	DevIntrfEvtHandler_t EvtCB;		//!< Modem events, NULL for none
} ModemIpcIntrfCfg_t;

typedef struct __Modem_Ipc_Intrf_Device {
	DevIntrf_t DevIntrf;			//!< Base device interface
	uint32_t DevAddr;				//!< Selector of the transfer
	int CmdLen;						//!< Length of the address phase in Cmd, 0 for none
	char Cmd[MODEM_IPC_AT_CMD_MAX + 1];	//!< Address phase, 0 terminated
	nRF91ModemCfg_t ModemCfg;		//!< Modem configuration when Init turns the modem on, kept by
									//!< the Modem library
	bool bStarted;					//!< Init turned the modem on
} ModemIpcIntrfDev_t;

#pragma pack(pop)

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief	Initialize the interface, and the modem when nothing did before.
 *
 * One modem: a second device takes the events over from the first.
 *
 * @param	pDev	: Interface device data
 * @param	pCfg	: Configuration
 *
 * @return	true - success
 */
bool ModemIpcIntrfInit(ModemIpcIntrfDev_t * const pDev, const ModemIpcIntrfCfg_t * const pCfg);

#ifdef __cplusplus
}

class ModemIpcIntrf : public DeviceIntrf {
public:
	ModemIpcIntrf() : vDevData() {}

	bool Init(const ModemIpcIntrfCfg_t &Cfg) {
		return ModemIpcIntrfInit(&vDevData, &Cfg);
	}

	operator DevIntrf_t * () {
		return &vDevData.DevIntrf;
	}

	operator ModemIpcIntrfDev_t * () {
		return &vDevData;
	}

	virtual uint32_t Rate(uint32_t DataRate) {
		(void)DataRate;
		return 0;
	}

	virtual uint32_t Rate(void) {
		return 0;
	}

	virtual bool StartRx(uint32_t DevAddr) {
		return DeviceIntrfStartRx(&vDevData.DevIntrf, DevAddr);
	}

	virtual int RxData(uint8_t *pBuff, int BuffLen) {
		return DeviceIntrfRxData(&vDevData.DevIntrf, pBuff, BuffLen);
	}

	virtual void StopRx(void) {
		DeviceIntrfStopRx(&vDevData.DevIntrf);
	}

	virtual bool StartTx(uint32_t DevAddr) {
		return DeviceIntrfStartTx(&vDevData.DevIntrf, DevAddr);
	}

	virtual int TxData(const uint8_t *pData, int DataLen) {
		return DeviceIntrfTxData(&vDevData.DevIntrf, pData, DataLen);
	}

	virtual void StopTx(void) {
		DeviceIntrfStopTx(&vDevData.DevIntrf);
	}

protected:
	ModemIpcIntrfDev_t vDevData;
};

#endif

/** @} End of group LTE */

#endif
