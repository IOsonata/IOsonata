/**-------------------------------------------------------------------------
@file	pdm.h

@brief	Implementation of Pulse density modulation interface


@author	Hoang Nguyen Hoan
@date	May 17, 2019

@license

Copyright (c) 2019, I-SYST inc., all rights reserved

Permission to use, copy, modify, and distribute this software for any purpose
with or without fee is hereby granted, provided that the above copyright
notice and this permission notice appear in all copies, and none of the
names : I-SYST or its contributors may be used to endorse or
promote products derived from this software without specific prior written
permission.

For info or contributing contact : hnhoan at i-syst dot com

THIS SOFTWARE IS PROVIDED BY THE REGENTS AND CONTRIBUTORS ``AS IS'' AND ANY
EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE REGENTS OR CONTRIBUTORS BE LIABLE FOR ANY
DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
(INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

----------------------------------------------------------------------------*/
#ifndef __PDM_H__
#define __PDM_H__

#include "device_intrf.h"
#include "coredev/iopincfg.h"
#include "cfifo.h"
#include "coredev/pei.h"
//#include "audio/audiodev_pdm.h"

#if 1
#pragma pack(push, 1)

typedef enum __PDM_OpMode {
	PDM_OPMODE_MONO_LEFT,
	PDM_OPMODE_MONO_RIGHT,
	PDM_OPMODE_STEREO
} PDM_OPMODE;

typedef enum __PDM_SamplMode {
	PDM_SMPLMODE_FALLING,	// left
	PDM_SMPLMODE_RISING		// right
} PDM_SMPLMODE;

typedef enum {
	PDM_PEI_EVENT_STARTED,
	PDM_PEI_EVENT_END,
	PDM_PEI_EVENT_STOPPED
} PDM_PEI_EVENT;

typedef enum {
	PDM_PEI_TASK_START,
	PDM_PEI_TASK_STOP
} PDM_PEI_TASK;

typedef struct __PDM_DevInterf	PdmDev_t;

typedef void (*PDMEvtHandler)(PdmDev_t *pDev, DEVINTRF_EVT Evt);

#define PDM_CLKPIN_IDX				0
#define PDM_DINPIN_IDX				1

typedef struct __PDM_Config {
	const IOPinCfg_t *pPins;		//!< Pointer to pins configuration
	int NbPins;						//!< Nb Pins
	uint32_t Freq;					//!< PDM clock frequency
	PDM_SMPLMODE SmplMode;
	PDM_OPMODE OpMode;
	int8_t GainLeft;
	int8_t GainRight;
	bool bIntEn;					//!< Interrupt enable
	int	IntPrio;					//!< Interrupt priority
	PDMEvtHandler EvtHandler;		//!< Pointer to event handler
	uint8_t *pFifoMem;				//!< Pointer to CFifo memory
	uint32_t FifoMemSize;			//!< Total reserved memory size for CFifo
	uint32_t FifoBlkSize;			//!< Fifo block size
} PdmCfg_t;

struct __PDM_DevInterf {
	PdmCfg_t CfgData;
	hCFifo_t hFifo;					//!< CFifo handle
	int NbSamples;
};

#pragma pack(pop)
#endif

#ifdef __cplusplus
extern "C" {
#endif

bool PdmInit(PdmDev_t * const pDev, const PdmCfg_t * const pCfg);
uint16_t *PdmGetSamples(PdmDev_t *pDev);
bool PdmEnable(PdmDev_t *pDev);
void PdmDisable(PdmDev_t *pDev);
bool PdmStart(PdmDev_t *pDev);
void PdmStop(PdmDev_t *pDev);
void PdmSetMode(PdmDev_t *pDev, PDM_OPMODE Mode);
void PdmPowerOff(PdmDev_t * const pDev);

// nRF52 PEI requires the configured PDM driver as hardware owner.
// PdmPreparePeiStart reserves the first FIFO block and programs
// SAMPLE.PTR/MAXCNT without starting capture. Continuous DMA must be
// serviced by the normal PDM interrupt (CfgData.bIntEn = true).
// Disconnect all routes before changing mode, stopping, or disabling.
bool PdmPreparePeiStart(PdmDev_t *pDev);
bool PdmEnablePeiEvent(PdmDev_t *pDev, PeiDev_t *pPei,
	PeiEvent_t *pEvt, PDM_PEI_EVENT Event);
bool PdmDisablePeiEvent(PdmDev_t *pDev, PeiDev_t *pPei,
	PeiEvent_t Evt);
bool PdmConnectPeiTask(PdmDev_t *pDev, PeiDev_t *pPei,
	PeiEvent_t Evt, PDM_PEI_TASK Task);
bool PdmDisconnectPeiTask(PdmDev_t *pDev, PeiDev_t *pPei,
	PeiEvent_t Evt, PDM_PEI_TASK Task);

#ifdef __cplusplus
}
#endif

#endif // __PDM_H__
