/**-------------------------------------------------------------------------
@file	spi_irq_fixture.cpp

@brief	SAM4L SPI master transfer test firmware.

The production spi_sam4l.cpp and device_intrf.cpp, built for the SAM4L and
executed by run_spi_irq.py in an Arm emulator with a SPI and PDCA register
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
#include "../../ARM/Microchip/SAM4L/src/spi_sam4l.cpp"

uint32_t SystemPeriphClockGet(int) { return 48000000U; }
void IOPinDisable(int, int) {}
void IOPinConfig(int, int, int, IOPINDIR, IOPINRES, IOPINTYPE) {}

static SPIDev_t s_Spi;
static uint8_t s_Buf[1024];
static int s_Events;

// SCK, MISO, MOSI, chip select PB13
static const IOPinCfg_t s_Pins[4] = {
	{ 2, 30, 0, IOPINDIR_OUTPUT, IOPINRES_NONE, IOPINTYPE_NORMAL },
	{ 2, 4, 0, IOPINDIR_INPUT, IOPINRES_NONE, IOPINTYPE_NORMAL },
	{ 0, 22, 0, IOPINDIR_OUTPUT, IOPINRES_NONE, IOPINTYPE_NORMAL },
	{ 1, 13, 0, IOPINDIR_OUTPUT, IOPINRES_NONE, IOPINTYPE_NORMAL },
};

static int Event(DevIntrf_t *, DEVINTRF_EVT, uint8_t *, int)
{
	s_Events++;
	return 0;
}

extern "C" int init(unsigned dma, unsigned irq, unsigned wide)
{
	SPICfg_t cfg = {};
	cfg.DevNo = 0;
	cfg.Phy = SPIPHY_NORMAL;
	cfg.Mode = SPIMODE_MASTER;
	cfg.pIOPinMap = s_Pins;
	cfg.NbIOPins = 4;
	cfg.Rate = 1000000;
	cfg.DataSize = wide ? 16 : 8;
	cfg.BitOrder = SPIDATABIT_MSB;
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
	if (IrqNo == SPI_IRQn)
		SPI_Handler();
	else if (IrqNo == PDCA_12_IRQn)
		PDCA_12_Handler();
	else
		PDCA_13_Handler();
}
extern "C" int tx(unsigned length) { return DeviceIntrfTx(&s_Spi.DevIntrf, 0, s_Buf, length); }
extern "C" int rx(unsigned length) { return DeviceIntrfRx(&s_Spi.DevIntrf, 0, s_Buf, length); }
extern "C" int read(unsigned length)
{
	uint8_t cmd[2] = { 0x9F, 0x03 };
	return DeviceIntrfRead(&s_Spi.DevIntrf, 0, cmd, 2, s_Buf, length);
}
extern "C" unsigned busy(void)
{
	bool b = atomic_flag_test_and_set(&s_Spi.DevIntrf.bBusy);
	if (!b) atomic_flag_clear(&s_Spi.DevIntrf.bBusy);
	return b;
}
extern "C" int events(void) { return s_Events; }

#define OFS(type, field) offsetof(type, field)
// Addresses and register offsets for the register model. A function, not a
// table: the firmware runs without its C++ start-up code.
extern "C" uint32_t layout(unsigned i)
{
	const uint32_t v[] = {
		(uint32_t)SAM4L_SPI, (uint32_t)s_Buf,									// 0 1
		OFS(Spi, SPI_CR), OFS(Spi, SPI_RDR), OFS(Spi, SPI_TDR),					// 2 3 4
		OFS(Spi, SPI_SR), OFS(Spi, SPI_IER), OFS(Spi, SPI_IDR),					// 5 6 7
		OFS(Spi, SPI_IMR), OFS(Spi, SPI_CSR),									// 8 9
		(uint32_t)Sam4lSpiRxChannel(), (uint32_t)Sam4lSpiTxChannel(),			// 10 11
		OFS(PdcaChannel, PDCA_MAR), OFS(PdcaChannel, PDCA_TCR),					// 12 13
		OFS(PdcaChannel, PDCA_CR), OFS(PdcaChannel, PDCA_MR),					// 14 15
		OFS(PdcaChannel, PDCA_IER), OFS(PdcaChannel, PDCA_IDR),					// 16 17
		OFS(PdcaChannel, PDCA_IMR), OFS(PdcaChannel, PDCA_ISR),					// 18 19
		SPI_IRQn, PDCA_12_IRQn, PDCA_13_IRQn,									// 20 21 22
	};
	return i < sizeof(v) / sizeof(v[0]) ? v[i] : 0;
}
