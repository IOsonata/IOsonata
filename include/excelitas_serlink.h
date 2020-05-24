/**-------------------------------------------------------------------------
@file	excelitas_serlink.h

@brief	Implementation of excelitas direct link single wire interface


@author	Hoang Nguyen Hoan
@date	Nov. 16, 2019

@license

MIT License

Copyright (c) 2019 I-SYST inc. All rights reserved.

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

#ifndef __EXCELITAS_SERLINK_H__
#define __EXCELITAS_SERLINK_H__

#include "iopinctrl.h"
#include "device_intrf.h"

#define EXCELITAS_PYD2592		2592U
#define EXCELITAS_PYD2792		2792U

#define EXCELITAS_PYD2592_DATA_SIZE			40	//!< 2592 data size 40 bits
#define EXCELITAS_PYD2592_REG_BITPOS		0ULL
#define EXCELITAS_PYD2592_REG_MASK			0x1FFFFFFULL
#define EXCELITAS_PYD2592_ADCCOUNT_BITPOS	25ULL
#define EXCELITAS_PYD2592_ADCCOUNT_MASK		(0x3FFFULL << EXCELITAS_PYD2592_ADCCOUNT_BITPOS)
#define EXCELITAS_PYD2592_OUTRANGE_BITPOS	39ULL
#define EXCELITAS_PYD2592_OUTRANGE_MASK		(1ULL << EXCELITAS_PYD2592_OUTRANGE_BITPOS)

typedef void (*INTHANDLER)();

#pragma pack(push, 1)

typedef union __Pyro_Cfg_Reg {
	struct {
		uint32_t PulseMode:1;		//!< count with (0) or without (1) BPF sign change
		uint32_t Reserved1:1;		//!< Reserved: Must be set to dec 0
		uint32_t HPFCutOff:1;		//!< 0:0.4Hz1:0.2Hz
		uint32_t Reserved2:2;		//!< Reserved: Must be set to dec 2
		uint32_t SignalSource:2;	//!< 0: PIR (BPF) 1: PIR (LPF) 2: Reserved 3: Temperature Sensor
		uint32_t OpMode:2;			//!< 0: Forced Readout 1: Interrupt Readout 2: Wake Up 3: Reserved
		uint32_t WindowTime:2;		//!< = 2s+[RegVal]·2s
		uint32_t PulseCounter:2;	//!< = 1+[RegVal]
		uint32_t BlindTime:4;		//!< = 0.5s + [Reg Val] · 0.5s
		uint32_t Threshold:8;		//!< Detection threshold on BPF value
	};
	uint32_t Val;
} PYROCFG_REG;

typedef struct __Direct_Link_Dev {
	IOPINCFG Pin;
	INTHANDLER IntHnadler;
} DIRECTLINK_DEV;

typedef struct __Serial_In_Dev {
	IOPINCFG Pin;
} SERIALIN_DEV;

#define EXCELITAS_PIN_MAX		2

#define EXCELITAS_DL_PIN_IDX	0		// Directlink pin index
#define EXCELITAS_SI_PIN_IDX	1		// Serial in pin index


typedef struct __Excel_Serial_Cfg {
	IOPINCFG Pins[EXCELITAS_PIN_MAX];
	DEVINTRF_EVTCB EvtHandler;			// Event handler
} EXCELSER_CFG;


typedef struct __Excel_Serial {
	IOPINCFG Pins[EXCELITAS_PIN_MAX];
	DEVINTRF DevIntrf;
	int NbBits;
} EXCELSERDEV;

typedef struct __Pyd2592_Data {
	uint32_t Timestamp;				//!< usec timestamp
	union {
		uint64_t Val;					//!< Register value
		struct {
			uint32_t CfgReg:25;			//!< Register settings
			int32_t AdcCount:14;		//!< ADC counts
			uint32_t OutRange:1;		//!< 0: PIR was reset 1: Normal operation
		};
	};
} PYD2592_DATA;

#pragma pack(pop)

#ifdef __cplusplus
extern "C" {
#endif

bool ExcelitasIntrfInit(EXCELSERDEV * const pDev, EXCELSER_CFG * const pCFG);

bool DirectLinkInit(DIRECTLINK_DEV * const pDev, int PortNo, int PinNo, int PinOp);
int DirectLinkRead(DIRECTLINK_DEV * const pDev, int NbBits, uint8_t * const pBuf);
uint64_t DirectLinkRead2(DIRECTLINK_DEV *pDev, int NbBits);
bool SerialInInit(SERIALIN_DEV *const pDev, int PortNo, int PinNo, int PinOp);
void SerialIn(SERIALIN_DEV *pDev, uint32_t Data, int NbBits);

#ifdef __cplusplus
}
#endif

#endif // __EXCELITAS_SERLINK_H__

