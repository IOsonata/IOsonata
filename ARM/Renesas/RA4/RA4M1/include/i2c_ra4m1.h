/**-------------------------------------------------------------------------
@file	i2c_ra4m1.h

@brief	RA4M1 IIC driver event trace

The driver keeps the last events of each IIC unit with a snapshot of the unit
registers. It is read by the target test examples after a failure. Set
RA4M1_I2C_TRACE to 0 when building the library to remove it.

@author	Hoang Nguyen Hoan
@date	Oct. 15, 2026

@license

MIT License

Copyright (c) 2026, I-SYST inc., all rights reserved

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is furnished
to do so, subject to the following conditions:

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
#ifndef __I2C_RA4M1_H__
#define __I2C_RA4M1_H__

#include <stdint.h>
#include <stdbool.h>

#ifndef RA4M1_I2C_TRACE
#define RA4M1_I2C_TRACE				1
#endif

#define RA4M1_I2C_TRACE_DEPTH		64

// Trace codes
#define RA4M1_I2C_TR_CONFIG			1	// Configure done (registers after it)
#define RA4M1_I2C_TR_START_REFUSED	2	// StartTx/StartRx refused, Info bits below
#define RA4M1_I2C_TR_IDLE_BUSY		3	// bus busy after the wait, block reset next
#define RA4M1_I2C_TR_IDLE_FAIL		4	// bus still busy after the reset
#define RA4M1_I2C_TR_TX_REFUSED		5	// TxData refused, Info bits below
#define RA4M1_I2C_TR_RX_REFUSED		6	// RxData refused, Info bits below
#define RA4M1_I2C_TR_BEGIN			7	// START requested
#define RA4M1_I2C_TR_NACK			8	// slave refused the address or a byte
#define RA4M1_I2C_TR_ABORT			9	// aborted, Info 1 = timeout or arbitration, 2 = no progress
#define RA4M1_I2C_TR_ADDR			10	// address byte written to ICDRT, Info bit0 = read
#define RA4M1_I2C_TR_SAMPLE			11	// half of the no-progress budget used, registers then
#define RA4M1_I2C_TR_START_POST		12	// immediately after START request

// Info bits for TR_START_REFUSED
#define RA4M1_I2C_INFO_NOT_ENABLED	0x01
#define RA4M1_I2C_INFO_BAD_ADDR		0x02
#define RA4M1_I2C_INFO_NOT_MASTER	0x04
// Info bits for TR_TX_REFUSED and TR_RX_REFUSED
#define RA4M1_I2C_INFO_NOT_OPEN		0x01
#define RA4M1_I2C_INFO_WRONG_DIR	0x02
#define RA4M1_I2C_INFO_ASYNC_BUSY	0x04
#define RA4M1_I2C_INFO_BAD_ARG		0x08

typedef struct
{
	uint32_t Seq;		// event number, counts from 1 per unit
	uint8_t Code;
	uint8_t Info;
	uint8_t MState;		// master state machine at the event
	uint8_t ICCR1;		// bit 0 SDAI, bit 1 SCLI: line levels seen by the unit
	uint8_t ICCR2;		// bit 7 BBSY
	uint8_t ICSR2;
	uint8_t ICFER;
	uint8_t ICMR3;
	uint8_t ICIER;
	uint8_t ICSER;
	uint8_t PeerCR1;	// other unit ICCR1, 0 when its module is stopped
	uint8_t PeerSR1;	// other unit ICSR1
	uint8_t PeerSR2;	// other unit ICSR2
	uint8_t PeerIER;	// other unit ICIER
	uint32_t SdaPfs;	// local SDA PFS sampled only at stalled transfer
	uint32_t SclPfs;	// local SCL PFS sampled only at stalled transfer
	uint32_t PeerSdaPfs;	// other IIC unit SDA PFS, if initialized
	uint32_t PeerSclPfs;	// other IIC unit SCL PFS, if initialized
} Ra4m1I2cTrace_t;

#ifdef __cplusplus
extern "C" {
#endif

/// Number of events recorded for the unit since power up (may exceed the depth).
uint32_t Ra4m1I2cTraceCount(int DevNo);

/// Copy event Idx (0 = oldest kept). Returns false when there is none.
bool Ra4m1I2cTraceGet(int DevNo, uint32_t Idx, Ra4m1I2cTrace_t *pRec);

#ifdef __cplusplus
}
#endif

#endif
