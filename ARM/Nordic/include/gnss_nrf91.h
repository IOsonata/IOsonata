/**-------------------------------------------------------------------------
@file	gnss_nrf91.h

@brief	GNSS receiver of the nRF91 modem, on ModemIpcIntrf.

The receiver is inside the modem. The driver reaches it only through the IPC
interface of the modem (modem_ipc_nrf91.h): AT commands on MODEM_IPC_ADDR_AT,
receiver commands and position data on MODEM_IPC_ADDR_GNSS.

Init adds GNSS to the system mode of the modem while the modem is off
(AT+CFUN=0) and refuses a running modem without GNSS in its system mode. It
sends the commands of the configuration (AT commands, for example AT%XCOEX0
for the antenna LNA of the board), activates GNSS with AT+CFUN=31, which
leaves the rest of the modem as it is, then configures and starts the
receiver. Enable does the activation, the configuration and the start again.

The receiver shares the radio of the modem: it runs only while the modem
leaves it the radio, which GNSS_EVT_BLOCKED and GNSS_EVT_UNBLOCKED report. A
modem fault the interface reports (it started the modem) is GNSS_EVT_FAULT:
initialize the interface again, then the receiver.

Systems: GPS L1 C/A, QZSS L1 C/A, and Galileo E1 on the modem firmware that
has it (nRF91x1). FixInterval is 0, 1, or 10 to 65535 s, FixTimeout up to
65535 s. One receiver: a second instance takes it over.

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
#ifndef __GNSS_NRF91_H__
#define __GNSS_NRF91_H__

#include <stdint.h>
#include <stdbool.h>

#include "nrf_modem_gnss.h"
#include "gnss/gnss.h"
#include "modem_ipc_nrf91.h"

/** @addtogroup GNSS
  * @{
  */

#ifdef __cplusplus

/// GNSS receiver of the nRF91 modem
class GnssNrf91 : public Gnss {
public:
	GnssNrf91();

	/**
	 * @brief	Initialize the receiver, configure it and start the search.
	 *
	 * @param	Cfg		: Configuration, copied
	 * @param	pIntrf	: The modem interface (ModemIpcIntrf), initialized
	 * @param	pTimer	: Time base of the time stamps, nullptr for none
	 *
	 * @return	true - search started
	 */
	virtual bool Init(const GnssCfg_t &Cfg, DeviceIntrf * const pIntrf, Timer * const pTimer = nullptr);

	/// Activate GNSS in the modem, configure and start the receiver
	virtual bool Enable();

	/// Stop the receiver
	virtual void Disable();

	/// Stop the receiver and start it again
	virtual void Reset();

	virtual int GetSat(GnssSat_t *pSat, int MaxSat);

	/// Events of ModemIpcIntrf, see modem_ipc_nrf91.h
	virtual int IntrfEvtHandler(DEVINTRF_EVT EvtId, uint8_t *pBuffer, int Len);

private:
	bool GnssCmd(uint8_t Cmd, uint32_t Param, int ParamLen);
	bool AtCmd(const char *pCmd);
	bool AtRead(const char *pCmd, const char *pPrefix, int *pVal, int NbVal);

	struct nrf_modem_gnss_sv vSv[NRF_MODEM_GNSS_MAX_SATELLITES];	// Satellites of the last
																	// update, written in the
																	// interface interrupt
};

#endif

/** @} End of group GNSS */

#endif
