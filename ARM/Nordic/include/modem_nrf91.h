/**-------------------------------------------------------------------------
@file	modem_nrf91.h

@brief	nRF91 Modem library glue

The nRF91 modem is driven by Nordic's Modem library (nrfxlib nrf_modem). The
library is OS independent: the application provides its OS glue (the
nrf_modem_os functions), the IPC interrupt and the memory shared with the
modem. This is that glue for IOsonata, bare metal by default. A TaktOS
application also links modem_os_nrf91_taktos.c, which replaces the waits,
the semaphores and the mutex with TaktOS ones.

The application runs non secure (see spe_nrf91.h), linked with
nrf91xx_xxaa_ns.ld and a non secure configuration of the library, and links
the Modem library variant of its modem firmware, for the nRF91x1 cellular
firmware:

	sdk-nrfxlib/nrf_modem/lib/cellular/nrf9120/hard-float/libmodem.a

nRF91ModemInit replaces nrf_modem_init. Everything else is the Modem library
API: nrf_modem_at for AT commands, nrf_socket, nrf_modem_gnss.

Memory shared with the modem must be in the lower 128 KB of RAM. It goes in
the .modem_shm sections, which the non secure linker script puts at the start
of RAM. The library has weak definitions of the TX and RX areas and of the
library heap, with the default sizes below. An application that needs other
sizes defines its own, which replace the library ones at link time, and
passes their sizes in the configuration:

	NRF91_MODEM_SHM uint8_t g_nRF91ModemShmTx[MY_TX_SIZE];
	...
	.ShmTxSize = sizeof(g_nRF91ModemShmTx),

Bare metal, a wait with a timeout uses the timer given in the configuration,
with the trigger of the configuration that the application leaves to it, and
sleeps until the timeout or the next interrupt. Without a timer, such a wait
polls in 1 ms busy steps. A wait without a timeout always sleeps.

The bare metal glue has a single waiting context: call the Modem library
from the main loop (or the handlers it runs), not from interrupts, except
the calls the library allows from interrupts.

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
#ifndef __MODEM_NRF91_H__
#define __MODEM_NRF91_H__

#include <stdint.h>
#include <stdbool.h>

#include "nrf_modem.h"
#include "coredev/timer.h"
#include "syslog.h"

/** @addtogroup LTE
  * @{
  */

/// Placement of memory shared with the modem
#define NRF91_MODEM_SHM					__attribute__((section(".modem_shm"), aligned(8)))

#define NRF91_MODEM_SHM_TX_DEFAULT_SIZE	0x2000	//!< Default TX area, data to the modem
#define NRF91_MODEM_SHM_RX_DEFAULT_SIZE	0x2000	//!< Default RX area, data from the modem
#define NRF91_MODEM_HEAP_DEFAULT_SIZE	1024	//!< Default library heap

/// Modem firmware the linked Modem library variant is built for
typedef enum __nRF91_Modem_Firmware {
	NRF91_MODEM_FW_CELLULAR,	//!< LTE-M, NB-IoT and GNSS, library variant lib/cellular
	NRF91_MODEM_FW_DECT			//!< DECT NR+, library variants lib/dect and lib/dect_phy
} NRF91_MODEM_FW;

#pragma pack(push, 4)

typedef struct __nRF91_Modem_Config {
	NRF91_MODEM_FW Fw;			//!< Modem firmware of the linked library variant
	int IntPrio;				//!< IPC interrupt priority
	TimerDev_t *pTimer;			//!< Time base of the bare metal waits with a timeout,
								//!< NULL for 1 ms busy steps
	int TimerTrigNo;			//!< Trigger of pTimer kept for the waits
	uint32_t ShmTxSize;			//!< Size of g_nRF91ModemShmTx, 0 for the default
	uint32_t ShmRxSize;			//!< Size of g_nRF91ModemShmRx, 0 for the default
	uint32_t HeapSize;			//!< Size of g_nRF91ModemHeap, 0 for the default
	uint8_t *pShmTrace;			//!< Modem trace area in .modem_shm, NULL for no trace
	uint32_t ShmTraceSize;		//!< Size of pShmTrace
	SysLog_t *pLog;				//!< Log of the logging library variant (libmodem_log.a),
								//!< NULL to drop it
	nrf_modem_fault_handler_t FaultHandler;	//!< Modem fault, NULL to only record it
	nrf_modem_dfu_handler_t DfuHandler;		//!< Modem firmware update result at
											//!< initialization, NULL to only record it
} nRF91ModemCfg_t;

#pragma pack(pop)

#ifdef __cplusplus
extern "C" {
#endif

/// Memory shared with the modem and the library heap. The library has weak
/// definitions of the default sizes, see the file description.
extern uint8_t g_nRF91ModemShmTx[];
extern uint8_t g_nRF91ModemShmRx[];
extern uint8_t g_nRF91ModemHeap[];

/**
 * @brief	Initialize the Modem library and turn on the modem.
 *
 * Replaces nrf_modem_init: sets up the shared memory, the heap and the time
 * base from the configuration, then calls nrf_modem_init.
 *
 * @param	pCfg : Configuration, kept by the library until nrf_modem_shutdown
 *
 * @return	0 - success
 * 			-NRF_EINVAL - invalid configuration (DECT NR+ on nRF9160, timer
 * 						  not initialized, trigger out of range), shared
 * 						  memory not word aligned or out of reach of the modem
 * 			-NRF_EPERM - the library is already initialized
 * 			otherwise the nrf_modem_init result (negative NRF errno)
 */
int nRF91ModemInit(const nRF91ModemCfg_t * const pCfg);

/**
 * @brief	Last modem fault reported since nRF91ModemInit.
 *
 * Recorded whether or not the configuration has a fault handler.
 *
 * @param	pInfo : Receives the fault, may be NULL
 *
 * @return	true - a fault was reported
 */
bool nRF91ModemFault(struct nrf_modem_fault_info * const pInfo);

/**
 * @brief	Modem firmware update result reported at the last initialization.
 *
 * @return	NRF_MODEM_DFU_RESULT_* value, 0 if none was reported
 */
uint32_t nRF91ModemDfuResult(void);

#ifdef __cplusplus
}
#endif

/** @} End of group LTE */

#endif
