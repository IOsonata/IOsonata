/**-------------------------------------------------------------------------
@file	qdec.h

@brief	Generic quadrature decoder

A quadrature decoder follows the position of a rotary or linear encoder from
its two phase inputs, A and B, shifted by a quarter of a period. Each valid
transition of the inputs moves the position by one count, the sign giving the
direction. Swapping the A and B pins reverses the direction.

The driver keeps a 32 bit position, from 0 at init. It folds the hardware
counter into it in interrupt context, so it does not overflow while the
application does not read it.

When both inputs change between two samples, the direction is unknown and the
position is not changed. These are counted as errors. They mean that the
encoder turns too fast for the sampling period or that the inputs are noisy.

Optical encoders can need their light source turned on before each sample.
An optional LED pin does that, the decoder drives it around each sample.

The movement event handler runs in interrupt context once per report period
in which the position changed, with the change since the previous event.

@author	Hoang Nguyen Hoan
@date	Nov. 7, 2026

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
#ifndef __QDEC_H__
#define __QDEC_H__

#include <stdint.h>
#include <stdbool.h>

#include "coredev/iopincfg.h"
#include "coredev/pei.h"

/** @addtogroup Coredev
  * @{
  */

/// Pin indexes in the pin array of the configuration
#define QDEC_PHA_PIN_IDX		0		//!< Phase A input
#define QDEC_PHB_PIN_IDX		1		//!< Phase B input
#define QDEC_LED_PIN_IDX		2		//!< Optional LED output, PinNo -1 or NbPins 2 for none

typedef enum {
	QDEC_PEI_EVENT_SAMPLERDY,
	QDEC_PEI_EVENT_REPORTRDY,
	QDEC_PEI_EVENT_ACCOF,
	QDEC_PEI_EVENT_DBLRDY,
	QDEC_PEI_EVENT_STOPPED
} QDEC_PEI_EVENT;

typedef enum {
	QDEC_PEI_TASK_START,
	QDEC_PEI_TASK_STOP
} QDEC_PEI_TASK;

typedef struct __Qdec_Device QdecDev_t;

/**
 * @brief	Movement event handler
 *
 * Called in interrupt context at the end of a report period in which the
 * position changed.
 *
 * @param	pDev	: Decoder device data
 * @param	Count	: Position change since the previous event
 */
typedef void (*QdecEvtHandler_t)(QdecDev_t * const pDev, int32_t Count);

/// Quadrature decoder configuration data
typedef struct __Qdec_Config {
	int DevNo;					//!< Decoder instance number, 0 based
	const IOPinCfg_t *pPins;	//!< Pins, indexed by QDEC_xxx_PIN_IDX
	int NbPins;					//!< Number of pins, 2, or 3 with the LED
	uint32_t usSamplePeriod;	//!< Input sampling period in usec, the closest supported one is used
	uint32_t usReportPeriod;	//!< Movement report period in usec, the closest supported one is used,
								//!< 0 for each sample
	bool bDebounce;				//!< Filter glitches on the inputs
	bool bLedActHigh;			//!< LED pin active level, true for high
	uint32_t usLedLead;			//!< Time the LED is on before each sample in usec, the closest
								//!< supported one is used
	int IntPrio;				//!< Interrupt priority
	QdecEvtHandler_t EvtHandler;	//!< Movement event handler, NULL for none
} QdecCfg_t;

/// Quadrature decoder device data
struct __Qdec_Device {
	int DevNo;					//!< Decoder instance number
	uint32_t usSamplePeriod;	//!< Sampling period in effect, in usec
	uint32_t usReportPeriod;	//!< Report period in effect, in usec
	volatile int32_t Position;	//!< Position folded by the driver, read it with QdecPosition
	volatile uint32_t ErrCnt;	//!< Errors folded by the driver, read them with QdecErrCount
	QdecEvtHandler_t EvtHandler;	//!< Movement event handler
	void *pDevData;				//!< Implementation data
};

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief	Configure the decoder, it does not start it
 *
 * The position and the error count are cleared.
 *
 * @param	pDev : Decoder device data to initialize
 * @param	pCfg : Configuration data
 *
 * @return	true - success
 */
bool QdecInit(QdecDev_t * const pDev, const QdecCfg_t * const pCfg);

/**
 * @brief	Start sampling the inputs
 *
 * @param	pDev : Decoder device data
 *
 * @return	true - success
 */
bool QdecStart(QdecDev_t * const pDev);

// Prepare the configured QDEC to accept a hardware START task without
// starting sampling. The QDEC ISR still owns READCLRACC and position folding.
bool QdecPreparePeiStart(QdecDev_t *pDev);
bool QdecEnablePeiEvent(QdecDev_t *pDev, PeiDev_t *pPei,
	PeiEvent_t *pEvt, QDEC_PEI_EVENT Event);
bool QdecDisablePeiEvent(QdecDev_t *pDev, PeiDev_t *pPei,
	PeiEvent_t Evt);
bool QdecConnectTask(QdecDev_t *pDev, PeiDev_t *pPei,
	PeiEvent_t Evt, QDEC_PEI_TASK Task);
bool QdecDisconnectTask(QdecDev_t *pDev, PeiDev_t *pPei,
	PeiEvent_t Evt, QDEC_PEI_TASK Task);

/**
 * @brief	Stop sampling the inputs
 *
 * The position is kept, a later QdecStart continues from it. The decoder does
 * not see movements while stopped.
 *
 * @param	pDev : Decoder device data
 */
void QdecStop(QdecDev_t * const pDev);

/**
 * @brief	Current position
 *
 * Includes the counts not reported yet.
 *
 * @param	pDev : Decoder device data
 *
 * @return	Position in counts
 */
int32_t QdecPosition(QdecDev_t * const pDev);

/**
 * @brief	Set the position
 *
 * The next movement event counts from it.
 *
 * @param	pDev : Decoder device data
 * @param	Pos  : New position in counts
 */
void QdecSetPosition(QdecDev_t * const pDev, int32_t Pos);

/**
 * @brief	Number of errors since init
 *
 * Transitions where both inputs changed between two samples.
 *
 * @param	pDev : Decoder device data
 *
 * @return	Error count
 */
uint32_t QdecErrCount(QdecDev_t * const pDev);

#ifdef __cplusplus
}

/// Quadrature decoder class
class Qdec {
public:
	virtual ~Qdec() {}

	/**
	 * @brief	Configure the decoder, it does not start it
	 *
	 * @param	Cfg : Configuration data
	 *
	 * @return	true - success
	 */
	virtual bool Init(const QdecCfg_t &Cfg) { return QdecInit(&vDev, &Cfg); }

	/**
	 * @brief	Start sampling the inputs
	 *
	 * @return	true - success
	 */
	virtual bool Start() { return QdecStart(&vDev); }

	/**
	 * @brief	Stop sampling the inputs, the position is kept
	 */
	virtual void Stop() { QdecStop(&vDev); }

	/**
	 * @brief	Current position
	 *
	 * @return	Position in counts
	 */
	virtual int32_t Position() { return QdecPosition(&vDev); }

	/**
	 * @brief	Set the position
	 *
	 * @param	Pos : New position in counts
	 */
	virtual void Position(int32_t Pos) { QdecSetPosition(&vDev, Pos); }

	/**
	 * @brief	Number of errors since init
	 *
	 * @return	Error count
	 */
	virtual uint32_t ErrCount() { return QdecErrCount(&vDev); }

	bool PreparePeiStart()
	{
		return QdecPreparePeiStart(&vDev);
	}

	bool EnablePeiEvent(Pei &PeiDev, PeiEvent_t &Evt,
		QDEC_PEI_EVENT Event)
	{
		return QdecEnablePeiEvent(&vDev, PeiDev, &Evt, Event);
	}

	bool DisablePeiEvent(Pei &PeiDev, PeiEvent_t Evt)
	{
		return QdecDisablePeiEvent(&vDev, PeiDev, Evt);
	}

	bool ConnectTask(Pei &PeiDev, PeiEvent_t Evt, QDEC_PEI_TASK Task)
	{
		return QdecConnectTask(&vDev, PeiDev, Evt, Task);
	}

	bool DisconnectTask(Pei &PeiDev, PeiEvent_t Evt, QDEC_PEI_TASK Task)
	{
		return QdecDisconnectTask(&vDev, PeiDev, Evt, Task);
	}

	operator QdecDev_t * () { return &vDev; }

private:
	QdecDev_t vDev;
};

#endif

/** @} */

#endif
