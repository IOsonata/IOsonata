/**-------------------------------------------------------------------------
@file	master_irq_fixture.cpp

@brief	SAM D21 SERCOM I2C and SPI master transfer test firmware.

The production i2c_samd.cpp, spi_samd.cpp, dmac_samd.c and device_intrf.cpp,
built for the SAM D21 and executed by run_master_irq.py in an Arm emulator
with a SERCOM and DMAC register model. I2C master on SERCOM2, SPI master on
SERCOM0. The SERCOM pad of a pin is its pin number.

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
#include "coredev/i2c.h"
#include "coredev/spi.h"
#include "coredev/system_core_clock.h"
#include "samdxxx.h"
#include "sercom_samd.h"

extern "C" void SERCOM0_Handler(void);
extern "C" void SERCOM2_Handler(void);
extern "C" void DMAC_Handler(void);

static I2CDev_t s_I2c;
static SPIDev_t s_Spi;
static uint8_t s_Buf[1024];
static int s_Events;

// SDA pad 0, SCL pad 1
static const IOPinCfg_t s_I2cPins[2] = {
	{ 0, 0, 0, IOPINDIR_BI, IOPINRES_NONE, IOPINTYPE_OPENDRAIN },
	{ 0, 1, 0, IOPINDIR_BI, IOPINRES_NONE, IOPINTYPE_OPENDRAIN },
};

// SCK pad 1, MISO pad 3, MOSI pad 0, chip select PA2
static const IOPinCfg_t s_SpiPins[4] = {
	{ 0, 1, 0, IOPINDIR_OUTPUT, IOPINRES_NONE, IOPINTYPE_NORMAL },
	{ 0, 3, 0, IOPINDIR_INPUT, IOPINRES_NONE, IOPINTYPE_NORMAL },
	{ 0, 0, 0, IOPINDIR_OUTPUT, IOPINRES_NONE, IOPINTYPE_NORMAL },
	{ 0, 2, 0, IOPINDIR_OUTPUT, IOPINRES_NONE, IOPINTYPE_NORMAL },
};

// Stand-ins of sercom_samd.c and of the clock, pin and bus reset code
sercom_registers_t *SamdSercomGetReg(int DevNo) { return DevNo == 0 ? SERCOM0_REGS : SERCOM2_REGS; }
bool SamdSercomClockEnable(int) { return true; }
void SamdSercomClockDisable(int) {}
int SamdSercomPad(int, const IOPinCfg_t *pPin) { return pPin->PinNo; }
bool SamdSercomI2cPin(const IOPinCfg_t *) { return true; }
bool SamdSercomOwned(int, const DevIntrf_t *) { return false; }
uint32_t SystemCoreClockGet(void) { return 48000000U; }
uint32_t SystemPeriphClockGet(int) { return 48000000U; }
void IOPinConfig(int, int, int, IOPINDIR, IOPINRES, IOPINTYPE) {}
void IOPinDisable(int, int) {}
void I2CBusReset(I2CDev_t * const) {}

static int Event(DevIntrf_t *, DEVINTRF_EVT, uint8_t *, int)
{
	s_Events++;
	return 0;
}

static unsigned Busy(DevIntrf_t *pIntrf)
{
	bool b = atomic_flag_test_and_set(&pIntrf->bBusy);

	if (!b)
	{
		atomic_flag_clear(&pIntrf->bBusy);
	}

	return b;
}

extern "C" int i2c_init(unsigned dma, unsigned irq)
{
	I2CCfg_t cfg = {};

	cfg.DevNo = 2;
	cfg.Type = I2CTYPE_STANDARD;
	cfg.Mode = I2CMODE_MASTER;
	cfg.pIOPinMap = s_I2cPins;
	cfg.NbIOPins = 2;
	cfg.Rate = 100000;
	cfg.AddrType = I2CADDR_TYPE_NORMAL;
	cfg.bDmaEn = dma;
	cfg.bIntEn = irq;
	cfg.IntPrio = 1;
	cfg.EvtCB = Event;
	s_Events = 0;

	return I2CInit(&s_I2c, &cfg);
}

extern "C" int spi_init(unsigned dma, unsigned irq)
{
	SPICfg_t cfg = {};

	cfg.DevNo = 0;
	cfg.Phy = SPIPHY_NORMAL;
	cfg.Mode = SPIMODE_MASTER;
	cfg.pIOPinMap = s_SpiPins;
	cfg.NbIOPins = 4;
	cfg.Rate = 4000000;
	cfg.DataSize = 8;
	cfg.ChipSel = SPICSEL_AUTO;
	cfg.bDmaEn = dma;
	cfg.bIntEn = irq;
	cfg.IntPrio = 1;
	cfg.DummyByte = 0xFF;
	cfg.EvtCB = Event;
	s_Events = 0;

	return SPIInit(&s_Spi, &cfg);
}

extern "C" void isr(unsigned IrqNo)
{
	if (IrqNo == DMAC_IRQn)
	{
		DMAC_Handler();
	}
	else if (IrqNo == SERCOM0_IRQn)
	{
		SERCOM0_Handler();
	}
	else
	{
		SERCOM2_Handler();
	}
}

extern "C" int i2c_tx(unsigned Len) { return DeviceIntrfTx(&s_I2c.DevIntrf, 0x22, s_Buf, Len); }
extern "C" int i2c_rx(unsigned Len) { return DeviceIntrfRx(&s_I2c.DevIntrf, 0x22, s_Buf, Len); }
extern "C" int i2c_read_reg(unsigned Len)
{
	uint8_t cmd[2] = { 0xA7, 0x5C };

	return DeviceIntrfRead(&s_I2c.DevIntrf, 0x22, cmd, 2, s_Buf, Len);
}
extern "C" int i2c_write_data(unsigned TotalLen)
{
	return DeviceIntrfWrite(&s_I2c.DevIntrf, 0x22, s_Buf, 1, s_Buf + 1, TotalLen - 1);
}
extern "C" unsigned i2c_busy(void) { return Busy(&s_I2c.DevIntrf); }

extern "C" int spi_tx(unsigned Len) { return DeviceIntrfTx(&s_Spi.DevIntrf, 0, s_Buf, Len); }
extern "C" int spi_rx(unsigned Len) { return DeviceIntrfRx(&s_Spi.DevIntrf, 0, s_Buf, Len); }
extern "C" int spi_read(unsigned Len)
{
	uint8_t cmd[2] = { 0x9F, 0x03 };

	return DeviceIntrfRead(&s_Spi.DevIntrf, 0, cmd, 2, s_Buf, Len);
}
extern "C" unsigned spi_busy(void) { return Busy(&s_Spi.DevIntrf); }

extern "C" int events(void) { return s_Events; }

#define OFS(type, field) offsetof(type, field)
// Addresses and register offsets for the register model. A function, not a
// table: the firmware runs without its C++ start-up code.
extern "C" uint32_t layout(unsigned i)
{
	const uint32_t v[] = {
		(uint32_t)s_Buf, (uint32_t)SERCOM2_REGS, (uint32_t)SERCOM0_REGS,			// 0 1 2
		(uint32_t)DMAC_REGS,														// 3
		OFS(sercom_i2cm_registers_t, SERCOM_INTENCLR),								// 4
		OFS(sercom_i2cm_registers_t, SERCOM_INTENSET),								// 5
		OFS(sercom_i2cm_registers_t, SERCOM_INTFLAG),								// 6
		OFS(sercom_i2cm_registers_t, SERCOM_STATUS),								// 7
		OFS(sercom_i2cm_registers_t, SERCOM_ADDR),									// 8
		OFS(sercom_i2cm_registers_t, SERCOM_DATA),									// 9
		OFS(sercom_spim_registers_t, SERCOM_DATA),									// 10
		OFS(dmac_registers_t, DMAC_INTPEND), OFS(dmac_registers_t, DMAC_CHID),		// 11 12
		OFS(dmac_registers_t, DMAC_CHCTRLA), OFS(dmac_registers_t, DMAC_CHINTFLAG),	// 13 14
	};

	return i < sizeof(v) / sizeof(v[0]) ? v[i] : 0;
}
