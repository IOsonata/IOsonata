/**-------------------------------------------------------------------------
@file	lte.h

@brief	Generic LTE subsystem (LTE-M, NB-IoT).

One init for the modem and the network, the same shape as BtAppInit:

	LteInit(&cfg);			// modem on, mode, APN, PSM, eDRX, attach started
	...						// LTE_EVT_REGISTERED arrives through cfg.EvtHandler
	sock.Init(sockcfg);		// sockets once registered, see net/sock_intrf.h

LteInit starts the network attach as its last step. LteDisconnect leaves the
network (flight mode) and LteConnect attaches again.

Work leaves the modem interrupt through LteEvtQue, the same way UsbEvtQue and
BtEvtQue do: bare metal it runs from the application event queue (AppRun),
with an RTOS from the thread that serves LTE. The configuration EvtHandler
and UrcHandler are called from there, never from the interrupt, so they may
send AT commands.

Unsolicited result codes (URC) are copied in the interrupt into a FIFO in
g_LteUrcMem. The library has a weak definition of the default size. An
application that needs more defines its own, which replaces the library one
at link time, and passes its size in the configuration:

	alignas(4) uint8_t g_LteUrcMem[LTE_URC_MEMSIZE(8)];
	...
	.UrcMemSize = sizeof(g_LteUrcMem),

The subsystem uses +CEREG (registration, cell, PSM granted by the network),
+CSCON (RRC connection) and +CEDRXP (eDRX granted by the network). Every
other URC goes to the configuration UrcHandler. When the FIFO was full and
URCs were lost, the registration and RRC state are read back from the modem.

What is the same for every modem lives in src/lte/lte.cpp: the URC FIFO, the
3GPP configuration commands (27.007), the URC parsing, the status and the
events. A port implements LteInit, LteConnect, LteDisconnect, LteAtCmd and
LteGetInfo, and reports URCs and modem faults with LteUrcPut and LteFaultPut.
The nRF91 port is lte_nrf91.cpp, on the Modem library.

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
#ifndef __LTE_H__
#define __LTE_H__

#include <stdint.h>
#include <stdbool.h>

#include "cfifo.h"
#include "coredev/timer.h"

/** @addtogroup LTE
  * @{
  */

/// Longest URC kept, terminating 0 included. A longer one is cut.
#define LTE_URC_LEN_MAX					128

/// Memory for NbUrc URCs waiting to be processed
#define LTE_URC_MEMSIZE(NbUrc)			CFIFO_TOTAL_MEMSIZE(NbUrc, LTE_URC_LEN_MAX)

/// Size of the library default g_LteUrcMem
#define LTE_URC_MEMSIZE_DEFAULT			LTE_URC_MEMSIZE(8)

/// Radio access technologies, a bit each
typedef enum __Lte_Rat {
	LTE_RAT_NONE = 0,
	LTE_RAT_LTEM = 1,					//!< LTE-M (E-UTRAN WB-S1)
	LTE_RAT_NBIOT = 2,					//!< NB-IoT (E-UTRAN NB-S1)
	LTE_RAT_LTEM_NBIOT = 3,				//!< Both, the modem chooses
} LTE_RAT;

/// PDN type of the default bearer
typedef enum __Lte_Pdn {
	LTE_PDN_IPV4V6,
	LTE_PDN_IPV4,
	LTE_PDN_IPV6,
} LTE_PDN;

/// Registration state, 3GPP 27.007 +CEREG <stat>
typedef enum __Lte_Reg {
	LTE_REG_NONE = 0,					//!< Not registered, not searching
	LTE_REG_HOME = 1,					//!< Registered, home network
	LTE_REG_SEARCHING = 2,				//!< Not registered, searching
	LTE_REG_DENIED = 3,					//!< Registration denied
	LTE_REG_UNKNOWN = 4,				//!< Unknown, out of coverage
	LTE_REG_ROAMING = 5,				//!< Registered, roaming
	LTE_REG_UICC_FAIL = 90,				//!< Not registered, SIM failure
} LTE_REG;

/// Events to the configuration EvtHandler
typedef enum __Lte_Evt {
	LTE_EVT_REGISTERED,					//!< Registered, home or roaming: sockets can be used
	LTE_EVT_UNREGISTERED,				//!< Registration lost, RegStat tells why
	LTE_EVT_REG_STATE,					//!< RegStat changed while not registered
	LTE_EVT_CELL,						//!< Serving cell changed while registered
	LTE_EVT_RRC_CONNECTED,				//!< Radio connection up
	LTE_EVT_RRC_IDLE,					//!< Radio connection released
	LTE_EVT_PSM,						//!< PSM granted by the network changed
	LTE_EVT_EDRX,						//!< eDRX granted by the network changed
	LTE_EVT_MODEM_FAULT,				//!< The modem stopped, status cleared: LteInit restarts it
} LTE_EVT;

/// Information LteGetInfo reads from the modem
typedef enum __Lte_Info {
	LTE_INFO_IMEI,
	LTE_INFO_IMSI,
	LTE_INFO_ICCID,
	LTE_INFO_FWVER,						//!< Modem firmware version
} LTE_INFO;

#pragma pack(push, 4)

/// Network state, as the URCs report it
typedef struct __Lte_Status {
	LTE_REG RegStat;					//!< Registration state
	LTE_RAT Rat;						//!< Radio access technology in use
	uint16_t Tac;						//!< Tracking area code
	uint32_t CellId;					//!< E-UTRAN cell id
	bool bRrcConnected;					//!< Radio connection up
	bool bPsm;							//!< PSM granted by the network
	uint32_t PsmTau;					//!< Periodic TAU granted, in s
	uint32_t PsmActive;					//!< Active time granted, in s
	uint32_t EdrxCycle;					//!< eDRX cycle granted in ms, 0 for none
	uint32_t EdrxPtw;					//!< Paging time window granted in ms
	uint32_t UrcLost;					//!< URCs dropped, FIFO full
} LteStatus_t;

/**
 * @brief	Network event handler.
 *
 * Called from the queued LTE event, never from the interrupt.
 *
 * @param	Evt		: Event
 * @param	pStatus	: Network state after the event
 */
typedef void (*LteEvtHandler_t)(LTE_EVT Evt, const LteStatus_t * const pStatus);

/**
 * @brief	Handler of the URCs the subsystem does not use.
 *
 * Called from the queued LTE event, never from the interrupt.
 *
 * @param	pUrc	: The URC, without line ending
 */
typedef void (*LteUrcHandler_t)(const char *pUrc);

/// Deferred LTE work: runs outside the interrupt with the values it was
/// queued with. Same signature as AppEvtHandler_t.
typedef void (*LteEvtQueHandler_t)(uint32_t EvtId, void *pCtx);

typedef struct __Lte_Cfg {
	LTE_RAT Rat;						//!< Radio access technologies allowed
	LTE_RAT RatPref;					//!< Preferred one when both are allowed, LTE_RAT_NONE for none
	bool bGnss;							//!< Keep the GNSS receiver usable, where the modem has one
	const uint8_t *pBand;				//!< LTE bands allowed (band numbers, all supported by the
										//!< modem), NULL to keep the band setting of the modem
	int NbBand;							//!< Number of entries in pBand
	const char *pApn;					//!< Access point name, NULL for the network default
	LTE_PDN PdnType;					//!< PDN type, used with pApn
	bool bPsm;							//!< Request Power Saving Mode
	uint32_t PsmTau;					//!< Periodic TAU requested in s, 0 for the network choice
	uint32_t PsmActive;					//!< Active time requested in s, used when PsmTau is not 0
	uint32_t EdrxCycle;					//!< eDRX cycle requested in ms, 0 for no eDRX
	bool bRai;							//!< Release assistance indication, see SockIntrfRai
	int IntPrio;						//!< Modem interrupt priority
	TimerDev_t *pTimer;					//!< Time base of the waits for the modem, NULL for
										//!< busy steps
	int TimerTrigNo;					//!< Trigger of pTimer kept for those waits
	uint32_t UrcMemSize;				//!< Size of g_LteUrcMem, 0 for the default
	LteEvtHandler_t EvtHandler;			//!< Network events, may be NULL
	LteUrcHandler_t UrcHandler;			//!< URCs the subsystem does not use, may be NULL
} LteCfg_t;

#pragma pack(pop)

#ifdef __cplusplus
extern "C" {
#endif

/// URC FIFO memory. The library has a weak definition of
/// LTE_URC_MEMSIZE_DEFAULT bytes, see the file description.
extern uint8_t g_LteUrcMem[];

//
// Port. Implemented once per modem (lte_nrf91.cpp).
//

/**
 * @brief	Start the modem, configure it and start the network attach.
 *
 * Turns the modem on when the application has not done it already, sets
 * the radio access technologies, the APN, PSM, eDRX and the URCs the
 * subsystem uses, then attaches. Registration is reported later by
 * LTE_EVT_REGISTERED.
 *
 * An application that needs settings this configuration does not have turns
 * the modem on itself (on nRF91 with nRF91ModemInit), sends them with
 * LteAtCmd, then calls LteInit.
 *
 * @param	pCfg : Configuration, copied
 *
 * @return	true - attach started
 */
bool LteInit(const LteCfg_t * const pCfg);

/// Attach to the network again after LteDisconnect
bool LteConnect(void);

/// Leave the network, radio off, configuration kept (flight mode). An
/// application that turns the modem off itself (AT+CFUN=0) starts again with
/// LteInit, not LteConnect: the modem drops some settings when off.
bool LteDisconnect(void);

/**
 * @brief	Send an AT command and wait for its response.
 *
 * Not from an interrupt. Commands are run one at a time.
 *
 * @param	pCmd	: The command, without line ending
 * @param	pResp	: Receives the response, NULL when not needed
 * @param	RespLen	: Size of pResp
 *
 * @return	0 - OK
 * 			> 0 - ERROR, +CME ERROR or +CMS ERROR from the modem
 * 			< 0 - not sent, or the response did not fit
 */
int LteAtCmd(const char *pCmd, char *pResp, int RespLen);

/**
 * @brief	Read an identity or version from the modem.
 *
 * @param	Info	: What to read
 * @param	pBuf	: Receives it, 0 terminated
 * @param	BufLen	: Size of pBuf
 *
 * @return	true - read
 */
bool LteGetInfo(LTE_INFO Info, char *pBuf, int BufLen);

//
// Generic layer. Implemented once in src/lte/lte.cpp.
//

/// Network state as last reported
const LteStatus_t *LteGetStatus(void);

/// true while registered, home or roaming
bool LteRegistered(void);

/**
 * @brief	Signal of the serving cell (AT+CESQ).
 *
 * @param	pRsrp	: Receives RSRP in dBm
 * @param	pRsrq	: Receives RSRQ in dB, may be NULL
 *
 * @return	true - the modem has a measurement
 */
bool LteGetSignal(int *pRsrp, int *pRsrq);

/**
 * @brief	Queue deferred LTE work for execution outside the interrupt.
 *
 * Called from the modem interrupt, and by LteCheckStatus from the thread that
 * runs the LTE work, so an override must take both. The library has a weak
 * default for an application without an OS: it puts the work in the
 * application event queue (AppEvtHandlerQue), which the main loop runs.
 *
 * An application using an RTOS defines its own LteEvtQue. It stores the three
 * values in the queue of the thread that serves LTE, and that thread calls
 * Handler(EvtId, pCtx) for each one, and LteCheckStatus when it is idle.
 *
 * @param	EvtId	: Value to pass to Handler
 * @param	pCtx	: Value to pass to Handler
 * @param	Handler	: Function to call outside the interrupt
 *
 * @return	true - queued
 * 			false - queue full, LteCheckStatus queues it again
 */
bool LteEvtQue(uint32_t EvtId, void *pCtx, LteEvtQueHandler_t Handler);

/// Queue again the work LteEvtQue refused. AppRun calls it when idle.
void LteCheckStatus(void);

//
// Called by the port only
//

/**
 * @brief	Check and keep the configuration, set up the URC FIFO, clear the
 * 			status. First call of the port LteInit.
 *
 * @return	false - invalid configuration
 */
bool LteCoreInit(const LteCfg_t * const pCfg);

/**
 * @brief	Send the 3GPP configuration: APN, PSM, eDRX, +CEREG=5, +CSCON=1.
 *
 * Called by the port LteInit with the radio off, after its own settings.
 *
 * @return	false - a command failed
 */
bool LteCoreConfig(void);

/**
 * @brief	A URC from the modem. Interrupt safe, the line is copied.
 *
 * @param	pUrc : The URC, line ending allowed
 */
void LteUrcPut(const char *pUrc);

/// The modem reported a fault. Interrupt safe.
void LteFaultPut(void);

#ifdef __cplusplus
}
#endif

/** @} End of group LTE */

#endif
