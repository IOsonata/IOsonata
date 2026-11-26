/**-------------------------------------------------------------------------
@file	i2c_irq_fixture.cpp

@brief	SAM4L TWIM master transfer test firmware.

The production i2c_sam4l.cpp and device_intrf.cpp, built for the SAM4L and
executed by run_i2c_irq.py in an Arm emulator with a TWIM and PDCA register
model.

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

uint32_t SystemPeriphClockGet(int) { return 48000000U; }
void IOPinDisable(int, int) {}
void IOPinConfig(int, int, int, IOPINDIR, IOPINRES, IOPINTYPE) {}
void I2CBusReset(I2CDev_t * const) {}

static I2CDev_t s_I2c;
static uint8_t s_Buf[1024];
static const IOPinCfg_t s_Pins[2] = {};
static int s_Events;

static int Event(DevIntrf_t *, DEVINTRF_EVT, uint8_t *, int)
{
	s_Events++;
	return 0;
}

extern "C" int init(unsigned dma, unsigned irq, unsigned tenbit)
{
	I2CCfg_t cfg = {};
	cfg.DevNo = 0;
	cfg.Type = I2CTYPE_STANDARD;
	cfg.Mode = I2CMODE_MASTER;
	cfg.pIOPinMap = s_Pins;
	cfg.NbIOPins = 2;
	cfg.Rate = 100000;
	cfg.MaxRetry = 0;
	cfg.AddrType = tenbit ? I2CADDR_TYPE_EXT : I2CADDR_TYPE_NORMAL;
	cfg.bDmaEn = dma;
	cfg.bIntEn = irq;
	cfg.IntPrio = 1;
	cfg.EvtCB = Event;
	s_Events = 0;
	return I2CInit(&s_I2c, &cfg);
}

static uint32_t Addr() { return s_I2c.Cfg.AddrType == I2CADDR_TYPE_EXT ? 0x322 : 0x22; }

extern "C" void isr(void) { TWIM0_Handler(); }
extern "C" int tx(unsigned length) { return DeviceIntrfTx(&s_I2c.DevIntrf, Addr(), s_Buf, length); }
extern "C" int rx(unsigned length) { return DeviceIntrfRx(&s_I2c.DevIntrf, Addr(), s_Buf, length); }
extern "C" int read_reg(unsigned length)
{
	uint8_t cmd[2] = { 0xA7, 0x5C };
	return DeviceIntrfRead(&s_I2c.DevIntrf, Addr(), cmd, 2, s_Buf, length);
}
extern "C" int write_data(unsigned totalLength)
{
	return DeviceIntrfWrite(&s_I2c.DevIntrf, Addr(), s_Buf, 1, s_Buf + 1, totalLength - 1);
}
extern "C" unsigned busy(void)
{
	bool b = atomic_flag_test_and_set(&s_I2c.DevIntrf.bBusy);
	if (!b) atomic_flag_clear(&s_I2c.DevIntrf.bBusy);
	return b;
}
extern "C" int events(void) { return s_Events; }

#define OFS(type, field) offsetof(type, field)
// Addresses and register offsets for the register model. A function, not a
// table: the firmware runs without its C++ start-up code.
extern "C" uint32_t layout(unsigned i)
{
	const uint32_t v[] = {
		(uint32_t)SAM4L_TWIM0, (uint32_t)s_Buf,								// 0 1
		OFS(Twim, TWIM_CR), OFS(Twim, TWIM_CMDR), OFS(Twim, TWIM_NCMDR),	// 2 3 4
		OFS(Twim, TWIM_RHR), OFS(Twim, TWIM_THR), OFS(Twim, TWIM_SR),		// 5 6 7
		OFS(Twim, TWIM_IER), OFS(Twim, TWIM_IDR), OFS(Twim, TWIM_IMR),		// 8 9 10
		OFS(Twim, TWIM_SCR),												// 11
		(uint32_t)Sam4lI2CPdcaRxChannel(&s_Sam4lI2CDev[0]),					// 12
		(uint32_t)Sam4lI2CPdcaTxChannel(&s_Sam4lI2CDev[0]),					// 13
		OFS(PdcaChannel, PDCA_MAR), OFS(PdcaChannel, PDCA_TCR),				// 14 15
		OFS(PdcaChannel, PDCA_CR), OFS(PdcaChannel, PDCA_ISR),				// 16 17
	};
	return i < sizeof(v) / sizeof(v[0]) ? v[i] : 0;
}
