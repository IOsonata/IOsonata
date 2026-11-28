/**-------------------------------------------------------------------------
@file	i2c_slave_fixture.cpp

@brief	SAM4L TWIS slave test firmware.

The production i2c_sam4l.cpp, built for the SAM4L and executed by
run_i2c_slave.py in an Arm emulator with a TWIS and PDCA register model that
plays the bus master. The event handler is the register memory of the
I2CMasterSlave example: a write gives the offset and the data, a read starts
at the offset and a read without a write continues after the last one.

@author	Hoang Nguyen Hoan
@date	Nov. 26, 2026

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
#include <stddef.h>
#include "../../ARM/Microchip/SAM4L/src/i2c_sam4l.cpp"

#define SLAVE_MEM_SIZE		256

uint32_t SystemPeriphClockGet(int) { return 48000000U; }
void IOPinDisable(int, int) {}
void IOPinConfig(int, int, int, IOPINDIR, IOPINRES, IOPINTYPE) {}
void I2CBusReset(I2CDev_t * const) {}

static I2CDev_t s_I2c;
static const IOPinCfg_t s_Pins[2] = {};
static uint8_t s_Mem[SLAVE_MEM_SIZE];
static uint8_t s_Wr[64];
static int s_Offset;
static bool s_bWrite;
static int s_Done[32];
static int s_NbDone;

static int Event(DevIntrf_t *, DEVINTRF_EVT Evt, uint8_t *, int Len)
{
	int n;

	switch (Evt)
	{
		case DEVINTRF_EVT_READ_RQST:
			if (s_bWrite)
			{
				if (Len > 0)
				{
					s_Offset = s_Wr[0];
				}
				s_bWrite = false;
			}
			if (s_Offset < 0 || s_Offset >= SLAVE_MEM_SIZE)
			{
				s_Offset = 0;
			}
			I2CSetReadRqstData(&s_I2c, 0, &s_Mem[s_Offset], SLAVE_MEM_SIZE - s_Offset);
			break;
		case DEVINTRF_EVT_WRITE_RQST:
			s_bWrite = true;
			I2CSetWriteRqstBuffer(&s_I2c, 0, s_Wr, sizeof(s_Wr));
			break;
		case DEVINTRF_EVT_COMPLETED:
			if (s_NbDone < (int)(sizeof(s_Done) / sizeof(s_Done[0])))
			{
				s_Done[s_NbDone++] = Len;
			}
			if (Len >= 0)
			{
				if (s_bWrite == false)
				{
					s_Offset += Len;
					if (s_Offset >= SLAVE_MEM_SIZE)
					{
						s_Offset = 0;
					}
				}
				else if (Len > 1 && s_Wr[0] < SLAVE_MEM_SIZE)
				{
					n = Len - 1;
					if (n > SLAVE_MEM_SIZE - s_Wr[0])
					{
						n = SLAVE_MEM_SIZE - s_Wr[0];
					}
					memcpy(&s_Mem[s_Wr[0]], &s_Wr[1], n);
				}
			}
			s_bWrite = false;
			break;
		default:
			break;
	}

	return 0;
}

extern "C" int init(unsigned dma)
{
	I2CCfg_t cfg = {};

	cfg.DevNo = 0;
	cfg.Type = I2CTYPE_STANDARD;
	cfg.Mode = I2CMODE_SLAVE;
	cfg.pIOPinMap = s_Pins;
	cfg.NbIOPins = 2;
	cfg.Rate = 100000;
	cfg.AddrType = I2CADDR_TYPE_NORMAL;
	cfg.NbSlaveAddr = 1;
	cfg.SlaveAddr[0] = 0x22;
	cfg.bDmaEn = dma;
	cfg.bIntEn = true;
	cfg.IntPrio = 1;
	cfg.EvtCB = Event;
	for (int i = 0; i < SLAVE_MEM_SIZE; i++)
	{
		s_Mem[i] = (uint8_t)i;
	}
	s_Offset = 0;
	s_bWrite = false;
	s_NbDone = 0;

	return I2CInit(&s_I2c, &cfg);
}

extern "C" void twis_isr(void) { TWIS0_Handler(); }
extern "C" void pdca_isr(void) { PDCA_4_Handler(); }

#define OFS(type, field) offsetof(type, field)
// Addresses and register offsets for the register model. A function, not a
// table: the firmware runs without its C++ start-up code.
extern "C" uint32_t layout(unsigned i)
{
	const uint32_t v[] = {
		(uint32_t)SAM4L_TWIS0, (uint32_t)s_Mem,								// 0 1
		OFS(Twis, TWIS_CR), OFS(Twis, TWIS_RHR), OFS(Twis, TWIS_THR),		// 2 3 4
		OFS(Twis, TWIS_SR), OFS(Twis, TWIS_IER), OFS(Twis, TWIS_IDR),		// 5 6 7
		OFS(Twis, TWIS_IMR), OFS(Twis, TWIS_SCR),							// 8 9
		(uint32_t)Sam4lI2CSlavePdcaRxChannel(&s_Sam4lI2CDev[0]),			// 10
		(uint32_t)&SAM4L_PDCA->PDCA_CHANNEL[SAM4L_I2C_PDCA_TX_CHAN(0)],		// 11
		OFS(PdcaChannel, PDCA_MAR), OFS(PdcaChannel, PDCA_PSR),				// 12 13
		OFS(PdcaChannel, PDCA_TCR), OFS(PdcaChannel, PDCA_CR),				// 14 15
		OFS(PdcaChannel, PDCA_IER), OFS(PdcaChannel, PDCA_IDR),				// 16 17
		OFS(PdcaChannel, PDCA_IMR), OFS(PdcaChannel, PDCA_ISR),				// 18 19
		(uint32_t)s_Done, (uint32_t)&s_NbDone,								// 20 21
		// PDCA peripheral IDs of the TWIS0 receive and transmit requests
		SAM4L_PDCA_PID_TWIS_RX(0), 27U,										// 22 23
	};
	return i < sizeof(v) / sizeof(v[0]) ? v[i] : 0;
}
