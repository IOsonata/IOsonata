// Real Nordic register layouts and IOsonata drivers, executed by run.py.
#include "nrf.h"
#include "coredev/i2c.h"
#include "coredev/spi.h"
#include "coredev/shared_intrf.h"
#include "iopinctrl.h"
#include <stddef.h>
extern "C" {
uint32_t SystemCoreClock = 128000000;
uint32_t SystemMicroSecLoopCnt = 0;
void IOPinConfig(int, int, int, IOPINDIR, IOPINRES, IOPINTYPE) {}
void IOPinDisable(int, int) {}
NRF_GPIO_Type *nRFGpioGetReg(int) { return NRF_P1_S; }
}
#include "../../ARM/Nordic/src/i2c_nrfx.cpp"
#include "../../ARM/Nordic/src/spi_nrfx.cpp"
#include "../../ARM/Nordic/src/shared_intrf_nrfx.cpp"

static I2CDev_t i2c;
static SPIDev_t spi;
static uint8_t buffer[65540];
static IOPinCfg_t pins[4] = {};
static int completed, lastLength, armed;
static int Event(DevIntrf_t *, DEVINTRF_EVT evt, uint8_t *, int len)
{
	if (evt == DEVINTRF_EVT_COMPLETED) { ++completed; lastLength = len; }
	if (evt == DEVINTRF_EVT_STATECHG) ++armed;
	if (evt == DEVINTRF_EVT_READ_RQST)
		I2CSetReadRqstData(&i2c, 0, buffer, sizeof(buffer));
	return 0;
}
extern "C" int init(unsigned bus, unsigned dev, unsigned slave, unsigned rate)
{
	completed = lastLength = armed = 0;
	if (bus == 0)
	{
		I2CCfg_t cfg = {};
		cfg.DevNo = dev; cfg.Mode = slave ? I2CMODE_SLAVE : I2CMODE_MASTER;
		cfg.pIOPinMap = pins; cfg.NbIOPins = 2; cfg.Rate = rate;
		cfg.bDmaEn = true; cfg.bIntEn = slave; cfg.NbSlaveAddr = 1;
		cfg.SlaveAddr[0] = 0x22; cfg.EvtCB = Event;
		return I2CInit(&i2c, &cfg);
	}
	SPICfg_t cfg = {};
	cfg.DevNo = dev; cfg.Mode = slave ? SPIMODE_SLAVE : SPIMODE_MASTER;
	cfg.pIOPinMap = pins; cfg.NbIOPins = 4; cfg.Rate = rate; cfg.DataSize = 8;
	cfg.bDmaEn = true; cfg.bIntEn = slave; cfg.EvtCB = Event;
	cfg.DummyByte = 0xA5; cfg.ChipSel = SPICSEL_AUTO;
	return SPIInit(&spi, &cfg);
}
extern "C" unsigned rate(unsigned bus, unsigned value)
{
	return bus ? spi.DevIntrf.SetRate(&spi.DevIntrf, value) : i2c.DevIntrf.SetRate(&i2c.DevIntrf, value);
}
extern "C" unsigned address(unsigned bus, unsigned dev)
{
	return bus ? (unsigned)g_nRFxSPIDev[dev].pDmaReg : (unsigned)s_nRFxI2CDev[dev].pDmaReg;
}
extern "C" int slave_test(unsigned bus, unsigned dev)
{
	if (!g_SharedIntrf[dev].Handler) return 1;
	if (bus == 0)
	{
		auto r = s_nRFxI2CDev[dev].pDmaSReg;
		i2c.pTRBuff[0] = buffer; i2c.TRBuffLen[0] = sizeof(buffer);
		r->EVENTS_WRITE = 1;
		g_SharedIntrf[dev].Handler(dev, &i2c.DevIntrf);
		if (r->DMA.RX.PTR != (unsigned)buffer || r->DMA.RX.MAXCNT != 65535) return 2;
		r->EVENTS_DMA.RX.READY = 1;
		*(volatile uint32_t *)&r->DMA.RX.AMOUNT = 17;
		r->EVENTS_STOPPED = 1;
		g_SharedIntrf[dev].Handler(dev, &i2c.DevIntrf);
		if (completed != 1 || lastLength != 17) return 3;
		r->EVENTS_READ = 1;
		g_SharedIntrf[dev].Handler(dev, &i2c.DevIntrf);
		if (r->DMA.TX.PTR != (unsigned)buffer || r->DMA.TX.MAXCNT != 65535) return 4;
	}
	else
	{
		auto r = g_nRFxSPIDev[dev].pDmaSReg;
		spi.pRxBuff[0] = buffer; spi.RxBuffLen[0] = sizeof(buffer);
		spi.pTxData[0] = buffer; spi.TxDataLen[0] = sizeof(buffer);
		r->EVENTS_ACQUIRED = 1;
		g_SharedIntrf[dev].Handler(dev, &spi.DevIntrf);
		if (armed != 1 || r->DMA.RX.PTR != (unsigned)buffer || r->DMA.TX.PTR != (unsigned)buffer ||
			r->DMA.RX.MAXCNT != 65535 || r->DMA.TX.MAXCNT != 65535) return 5;
		*(volatile uint32_t *)&r->DMA.RX.AMOUNT = 17;
		r->EVENTS_END = 1;
		g_SharedIntrf[dev].Handler(dev, &spi.DevIntrf);
		if (completed != 1 || lastLength != 17) return 6;
	}
	return 0;
}
extern "C" int transfer(unsigned bus, unsigned receive, unsigned length)
{
	auto p = bus ? &spi.DevIntrf : &i2c.DevIntrf;
	return receive ? DeviceIntrfRx(p, bus ? 0 : 0x22, buffer, length) :
		DeviceIntrfTx(p, bus ? 0 : 0x22, buffer, length);
}
// Export offsets from the real MDK instead of hand-written register layouts.
#define OFS(type, field) offsetof(type, field)
extern "C" const uint32_t offsets[] = {
	OFS(NRF_TWIM_Type,TASKS_DMA.RX.START), OFS(NRF_TWIM_Type,TASKS_DMA.TX.START),
	OFS(NRF_TWIM_Type,TASKS_STOP), OFS(NRF_TWIM_Type,EVENTS_STOPPED),
	OFS(NRF_TWIM_Type,EVENTS_SUSPENDED), OFS(NRF_TWIM_Type,DMA.RX.PTR), OFS(NRF_TWIM_Type,DMA.TX.PTR),
	OFS(NRF_SPIM_Type,TASKS_START), OFS(NRF_SPIM_Type,EVENTS_END),
	OFS(NRF_SPIM_Type,EVENTS_DMA.RX.END), OFS(NRF_SPIM_Type,EVENTS_DMA.TX.END),
	OFS(NRF_SPIM_Type,DMA.RX.PTR), OFS(NRF_SPIM_Type,DMA.TX.PTR),
	OFS(NRF_SPIM_Type,PRESCALER), OFS(NRF_TWIM_Type,FREQUENCY),
	OFS(NRF_SPIM_Type,TASKS_STOP), OFS(NRF_SPIM_Type,EVENTS_STOPPED),
};
extern "C" int register_read()
{
	uint8_t command = 4;
	return DeviceIntrfRead(&i2c.DevIntrf, 0x22, &command, 1, buffer, 8);
}

extern "C" int timeout_test()
{
	g_nRFxSPIDev[0].pDmaReg->EVENTS_END = 0;
	return nRFxSPIWaitDMA(&g_nRFxSPIDev[0], 1);
}
