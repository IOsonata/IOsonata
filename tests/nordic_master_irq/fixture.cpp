/**-------------------------------------------------------------------------
@file	fixture.cpp

@brief	Nordic SPIM and TWIM master interrupt transfer test firmware.

Real Nordic register layouts and IOsonata drivers, executed by run.py.
SPIM and TWIM master with bIntEn and EasyDMA, built for the MCU given to
build.py.

@author	Hoang Nguyen Hoan
@date	Oct. 16, 2026

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

#include "nrf.h"
#include "coredev/i2c.h"
#include "coredev/spi.h"
#include "coredev/shared_intrf.h"
#include "iopinctrl.h"
#include <stddef.h>
#include "idelay.h"
uint32_t SystemCoreClock = 64000000;
uint32_t SystemMicroSecLoopCnt = 0;
extern "C" void IOPinConfig(int, int, int, IOPINDIR, IOPINRES, IOPINTYPE) {}
extern "C" void IOPinDisable(int, int) {}
#ifdef NRF_P0_S
extern "C" NRF_GPIO_Type *nRFGpioGetReg(int) { return NRF_P0_S; }
#else
extern "C" NRF_GPIO_Type *nRFGpioGetReg(int) { return NRF_P0; }
#endif
#include "../../ARM/Nordic/src/i2c_nrfx.cpp"
#include "../../ARM/Nordic/src/spi_nrfx.cpp"
#include "../../ARM/Nordic/src/shared_intrf_nrfx.cpp"

static I2CDev_t i2c;
static SPIDev_t spi;
static uint8_t buffer[70000];
static IOPinCfg_t pins[4] = {};
static int completed, lastLength, txReady;
static uint8_t *lastBuffer;

static int Event(DevIntrf_t *, DEVINTRF_EVT evt, uint8_t *buf, int len)
{
	if (evt == DEVINTRF_EVT_COMPLETED) { ++completed; lastLength = len; lastBuffer = buf; }
	if (evt == DEVINTRF_EVT_TX_READY) ++txReady;
	return 0;
}

static DevIntrf_t *intrf(unsigned bus) { return bus ? &spi.DevIntrf : &i2c.DevIntrf; }

extern "C" int init(unsigned bus, unsigned dev, unsigned dma, unsigned irq)
{
	completed = lastLength = txReady = 0;
	lastBuffer = NULL;
	if (bus == 0)
	{
		I2CCfg_t cfg = {};
		cfg.DevNo = dev; cfg.Mode = I2CMODE_MASTER;
		cfg.pIOPinMap = pins; cfg.NbIOPins = 2; cfg.Rate = 400000;
		cfg.bDmaEn = dma; cfg.bIntEn = irq; cfg.EvtCB = Event; cfg.MaxRetry = 5;
		return I2CInit(&i2c, &cfg);
	}
	SPICfg_t cfg = {};
	cfg.DevNo = dev; cfg.Mode = SPIMODE_MASTER;
	cfg.pIOPinMap = pins; cfg.NbIOPins = 4; cfg.Rate = 1000000; cfg.DataSize = 8;
	cfg.bDmaEn = dma; cfg.bIntEn = irq; cfg.EvtCB = Event;
	cfg.DummyByte = 0xA5; cfg.ChipSel = SPICSEL_AUTO;
	return SPIInit(&spi, &cfg);
}
extern "C" void isr(unsigned bus, unsigned dev) { g_SharedIntrf[dev].Handler(dev, intrf(bus)); }
extern "C" int tx(unsigned bus, unsigned length) { return DeviceIntrfTx(intrf(bus), bus ? 0 : 0x22, buffer, length); }
extern "C" int rx(unsigned bus, unsigned length) { return DeviceIntrfRx(intrf(bus), bus ? 0 : 0x22, buffer, length); }
extern "C" int read_reg(unsigned bus, unsigned length)
{
	uint8_t command = 4;
	return DeviceIntrfRead(intrf(bus), bus ? 0 : 0x22, &command, 1, buffer, length);
}
extern "C" unsigned busy(unsigned bus)
{
	bool b = atomic_flag_test_and_set(&intrf(bus)->bBusy);
	if (!b) atomic_flag_clear(&intrf(bus)->bBusy);
	return b;
}
extern "C" unsigned tx_ready(unsigned bus) { return intrf(bus)->bTxReady; }
extern "C" unsigned buffer_address() { return (unsigned)buffer; }
extern "C" int events(unsigned what) { return what == 0 ? completed : what == 1 ? lastLength : what == 2 ? txReady : (int)(unsigned)lastBuffer; }
extern "C" unsigned address(unsigned bus, unsigned dev)
{
	return bus ? (unsigned)g_nRFxSPIDev[dev].pDmaReg : (unsigned)s_nRFxI2CDev[dev].pDmaReg;
}
extern "C" const unsigned dma_max = NRFX_SPI_DMA_MAXCNT;
extern "C" const unsigned txbuf_size[] = { NRFX_I2C_TXBUF_SIZE, NRFX_SPI_TXBUF_SIZE };
extern "C" unsigned txbuf_address(unsigned bus, unsigned dev)
{
	return bus ? (unsigned)g_nRFxSPIDev[dev].TxBuf : (unsigned)s_nRFxI2CDev[dev].TxBuf;
}

// Export offsets from the real MDK instead of hand-written register layouts.
#define OFS(type, field) offsetof(type, field)
#ifdef NRF54L_SERIES
#define SPIM_EV_ENDRX	EVENTS_DMA.RX.END
#define SPIM_EV_ENDTX	EVENTS_DMA.TX.END
#define TWIM_TK_STARTRX	TASKS_DMA.RX.START
#define TWIM_TK_STARTTX	TASKS_DMA.TX.START
#define TWIM_EV_RXSTART	EVENTS_DMA.RX.READY
#define TWIM_EV_TXSTART	EVENTS_DMA.TX.READY
#define DMA_RXPTR		DMA.RX.PTR
#define DMA_TXPTR		DMA.TX.PTR
#else
#define SPIM_EV_ENDRX	EVENTS_ENDRX
#define SPIM_EV_ENDTX	EVENTS_ENDTX
#define TWIM_TK_STARTRX	TASKS_STARTRX
#define TWIM_TK_STARTTX	TASKS_STARTTX
#define TWIM_EV_RXSTART	EVENTS_RXSTARTED
#define TWIM_EV_TXSTART	EVENTS_TXSTARTED
#define DMA_RXPTR		RXD.PTR
#define DMA_TXPTR		TXD.PTR
#endif
extern "C" const uint32_t offsets[] = {
	OFS(NRF_SPIM_Type,TASKS_START), OFS(NRF_SPIM_Type,EVENTS_END),
	OFS(NRF_SPIM_Type,SPIM_EV_ENDRX), OFS(NRF_SPIM_Type,SPIM_EV_ENDTX),
	OFS(NRF_SPIM_Type,DMA_RXPTR), OFS(NRF_SPIM_Type,DMA_TXPTR),		// 4 5, MAXCNT +4, AMOUNT +8
	OFS(NRF_TWIM_Type,TWIM_TK_STARTRX), OFS(NRF_TWIM_Type,TWIM_TK_STARTTX),
	OFS(NRF_TWIM_Type,TASKS_STOP), OFS(NRF_TWIM_Type,EVENTS_STOPPED),
	OFS(NRF_TWIM_Type,EVENTS_LASTRX), OFS(NRF_TWIM_Type,EVENTS_LASTTX),
	OFS(NRF_TWIM_Type,DMA_RXPTR), OFS(NRF_TWIM_Type,DMA_TXPTR),		// 12 13
	OFS(NRF_TWIM_Type,TWIM_EV_RXSTART), OFS(NRF_TWIM_Type,TWIM_EV_TXSTART),
	OFS(NRF_SPIM_Type,INTENSET),
	OFS(NRF_TWIM_Type,EVENTS_SUSPENDED), OFS(NRF_TWIM_Type,EVENTS_ERROR),	// 17 18
	OFS(NRF_TWIM_Type,SHORTS), OFS(NRF_TWIM_Type,TASKS_RESUME),			// 19 20
	OFS(NRF_TWIM_Type,ERRORSRC), OFS(NRF_TWIM_Type,INTENSET),			// 21 22
	OFS(NRF_SPIM_Type,EVENTS_STARTED),									// 23
	OFS(NRF_SPIM_Type,INTENCLR), OFS(NRF_TWIM_Type,INTENCLR),			// 24 25
};
extern "C" const uint32_t twim[] = {
	NRFX_I2C_DMA_MAXCNT, TWIM_SHORTS_LASTTX_SUSPEND_Msk, TWIM_SHORTS_LASTTX_STOP_Msk,
	TWIM_SHORTS_LASTRX_STOP_Msk, TWIM_INTEN_SUSPENDED_Msk | TWIM_INTEN_ERROR_Msk | TWIM_INTEN_STOPPED_Msk,
};

// Master without EasyDMA, where the MCU has the TWI and SPI peripherals
#ifdef TWI_PRESENT
extern "C" const uint32_t twi[] = {
	1, OFS(NRF_TWI_Type,TASKS_STARTRX), OFS(NRF_TWI_Type,TASKS_STARTTX),	// 0 1 2
	OFS(NRF_TWI_Type,TASKS_STOP), OFS(NRF_TWI_Type,TASKS_RESUME),			// 3 4
	OFS(NRF_TWI_Type,EVENTS_STOPPED), OFS(NRF_TWI_Type,EVENTS_RXDREADY),	// 5 6
	OFS(NRF_TWI_Type,EVENTS_TXDSENT), OFS(NRF_TWI_Type,EVENTS_ERROR),		// 7 8
	OFS(NRF_TWI_Type,RXD), OFS(NRF_TWI_Type,TXD), OFS(NRF_TWI_Type,SHORTS),	// 9 10 11
	TWI_SHORTS_BB_SUSPEND_Msk, TWI_SHORTS_BB_STOP_Msk,						// 12 13
	OFS(NRF_TWI_Type,INTENSET),												// 14
	TWI_INTENSET_TXDSENT_Msk | TWI_INTENSET_RXDREADY_Msk |
	TWI_INTENSET_ERROR_Msk | TWI_INTENSET_STOPPED_Msk,						// 15
};
#else
extern "C" const uint32_t twi[] = { 0 };
#endif
#ifdef SPI_PRESENT
extern "C" const uint32_t spi_legacy[] = {
	1, OFS(NRF_SPI_Type,EVENTS_READY), OFS(NRF_SPI_Type,RXD),				// 0 1 2
	OFS(NRF_SPI_Type,TXD), OFS(NRF_SPI_Type,INTENSET), SPI_INTENSET_READY_Msk,	// 3 4 5
	OFS(NRF_SPI_Type,ENABLE), SPI_ENABLE_ENABLE_Enabled, SPIM_ENABLE_ENABLE_Enabled,	// 6 7 8
};
#else
extern "C" const uint32_t spi_legacy[] = { 0 };
#endif
