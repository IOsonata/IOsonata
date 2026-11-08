/**-------------------------------------------------------------------------
@file	wdt.h

@brief	Generic watchdog timer

A watchdog resets the MCU when the application stops reloading it within the
configured timeout. Its clock usually runs independently of the CPU, so it
keeps working when the application is stuck.

The watchdog can have several reload channels. Each enabled channel must be
reloaded within the timeout: one channel per task or per part of the
application that must stay alive, so that a single stuck part is enough to
reset the MCU. An MCU with a single reload register accepts one channel only.

A window watchdog also resets the MCU on a reload that comes too early. An
MCU without a window refuses a window value other than 0.

On most MCU a started watchdog cannot be stopped, only a reset does it, and
its configuration is locked while it runs. WdtInit called while the watchdog
already runs, for example after a reset that did not stop it, keeps the
running configuration and reports it in the device data.

The timeout event handler runs in interrupt context just before the reset.
On many MCU the reset follows within a few low frequency clock cycles, so it
can only record the minimum, it cannot prevent the reset.

@author	Hoang Nguyen Hoan
@date	Oct. 7, 2026

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
#ifndef __WDT_H__
#define __WDT_H__

#include <stdint.h>
#include <stdbool.h>

/** @addtogroup Coredev
  * @{
  */

typedef struct __Wdt_Device WdtDev_t;

/**
 * @brief	Watchdog timeout event handler
 *
 * Called in interrupt context when the watchdog times out, before the reset.
 *
 * @param	pDev : Watchdog device data
 */
typedef void (*WdtEvtHandler_t)(WdtDev_t * const pDev);

/// Watchdog configuration data
typedef struct __Wdt_Config {
	int DevNo;					//!< Watchdog instance number, 0 based
	uint32_t msTimeout;			//!< Time without reload before the reset, in msec, not 0
	uint32_t msWindow;			//!< Shortest time between reloads, in msec, 0 for none
	int NbChan;					//!< Reload channels, each must be reloaded within the timeout (0 means 1)
	bool bRunSleep;				//!< Keep counting while the CPU sleeps
	bool bRunHalt;				//!< Keep counting while a debugger halts the CPU
	int IntPrio;				//!< Interrupt priority of the timeout event
	WdtEvtHandler_t EvtHandler;	//!< Timeout event handler, NULL for none
} WdtCfg_t;

/// Watchdog device data
struct __Wdt_Device {
	int DevNo;					//!< Watchdog instance number
	uint32_t msTimeout;			//!< Timeout in effect, in msec, rounded up from the hardware value
	int NbChan;					//!< Reload channels in effect
	WdtEvtHandler_t EvtHandler;	//!< Timeout event handler
	void *pDevData;				//!< Implementation data
};

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief	Configure the watchdog, it does not start it
 *
 * When the watchdog already runs, its configuration is kept and reported in
 * pDev.
 *
 * @param	pDev : Watchdog device data to initialize
 * @param	pCfg : Configuration data
 *
 * @return	true - success
 */
bool WdtInit(WdtDev_t * const pDev, const WdtCfg_t * const pCfg);

/**
 * @brief	Start the watchdog
 *
 * On most MCU only a reset stops it again.
 *
 * @param	pDev : Watchdog device data
 *
 * @return	true - the watchdog runs
 */
bool WdtStart(WdtDev_t * const pDev);

/**
 * @brief	Reload one channel
 *
 * @param	pDev : Watchdog device data
 * @param	Chan : Channel number, 0 to NbChan - 1
 */
void WdtReload(WdtDev_t * const pDev, int Chan);

/**
 * @brief	Watchdog run state
 *
 * @param	pDev : Watchdog device data
 *
 * @return	true - the watchdog runs
 */
bool WdtRunning(WdtDev_t * const pDev);

#ifdef __cplusplus
}

/// Watchdog class
class Wdt {
public:
	virtual ~Wdt() {}

	/**
	 * @brief	Configure the watchdog, it does not start it
	 *
	 * @param	Cfg : Configuration data
	 *
	 * @return	true - success
	 */
	virtual bool Init(const WdtCfg_t &Cfg) { return WdtInit(&vDev, &Cfg); }

	/**
	 * @brief	Start the watchdog
	 *
	 * @return	true - the watchdog runs
	 */
	virtual bool Start() { return WdtStart(&vDev); }

	/**
	 * @brief	Reload one channel
	 *
	 * @param	Chan : Channel number, 0 to NbChan - 1
	 */
	virtual void Reload(int Chan = 0) { WdtReload(&vDev, Chan); }

	/**
	 * @brief	Watchdog run state
	 *
	 * @return	true - the watchdog runs
	 */
	virtual bool Running() { return WdtRunning(&vDev); }

	/**
	 * @brief	Timeout in effect
	 *
	 * @return	Timeout in msec
	 */
	virtual uint32_t Timeout() { return vDev.msTimeout; }

	operator WdtDev_t * () { return &vDev; }

private:
	WdtDev_t vDev;
};

#endif

/** @} */

#endif
