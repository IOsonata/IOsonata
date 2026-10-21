// H753 shared I2C/SPI register-model test. No hardware timing validation.
#include "coredev/i2c.h"
#include "coredev/spi.h"
#include "stm32h753xx.h"
#define CHECK(x) do { if (!(x)) return __LINE__; } while (0)
extern "C" {
volatile uint32_t I2cIrqTestPhase;
volatile uint32_t DmaTestActive;
volatile uint32_t DmaSlaveCase;
volatile uint32_t I2cIrqTestInside;
uint32_t SystemCoreClock = 400000000;
uint32_t SystemMicroSecLoopCnt = 1;
uint32_t SystemPeriphClockGet(int n) { return n == 3 ? 50000000 : 100000000; }
void SPI1_IRQHandler(); void SPI2_IRQHandler(); void SPI3_IRQHandler();
void SPI4_IRQHandler(); void SPI5_IRQHandler(); void SPI6_IRQHandler();
void I2C1_EV_IRQHandler(); void I2C2_EV_IRQHandler();
void I2C3_EV_IRQHandler(); void I2C4_EV_IRQHandler();
}
static void (*const spiIrq[])() = {SPI1_IRQHandler, SPI2_IRQHandler, SPI3_IRQHandler,
	SPI4_IRQHandler, SPI5_IRQHandler, SPI6_IRQHandler};
static void (*const i2cIrq[])() = {I2C1_EV_IRQHandler, I2C2_EV_IRQHandler,
	I2C3_EV_IRQHandler, I2C4_EV_IRQHandler};
static volatile bool done;
static volatile int count;
static volatile int events;
static volatile bool eventInIrq;
static int Event(DevIntrf_t *, DEVINTRF_EVT evt, uint8_t *, int len)
{
	if (evt == DEVINTRF_EVT_COMPLETED) { count = len; done = true; events = events + 1; eventInIrq = I2cIrqTestInside != 0; }
	return 0;
}
static uint8_t slaveRx[804], slaveTx[800];
static volatile int slaveEvents, slaveLength;
static int SlaveEvent(DevIntrf_t *intrf, DEVINTRF_EVT evt, uint8_t *, int len)
{
	if (evt == DEVINTRF_EVT_COMPLETED) { slaveLength = len; slaveEvents = slaveEvents + 1; }
	if (intrf->Type == DEVINTRF_TYPE_I2C)
	{
		I2CDev_t *dev = (I2CDev_t*)intrf->GetHandle(intrf);
		if (evt == DEVINTRF_EVT_WRITE_RQST) I2CSetWriteRqstBuffer(dev, 0, slaveRx + 1, 800);
		if (evt == DEVINTRF_EVT_READ_RQST) I2CSetReadRqstData(dev, 0, slaveTx, 800);
	}
	else if (evt == DEVINTRF_EVT_STATECHG)
	{
		SPIDev_t *dev = (SPIDev_t*)intrf->GetHandle(intrf);
		SPISetSlaveRxBuffer(dev, 0, slaveRx + 1, 800);
		SPISetSlaveTxData(dev, 0, slaveTx, 800);
	}
	return 0;
}
static const IOPinCfg_t spiPins[] = {
	{IOPORTA, 5, IOPINOP_FUNC5, IOPINDIR_OUTPUT, IOPINRES_NONE, IOPINTYPE_NORMAL},
	{IOPORTA, 6, IOPINOP_FUNC5, IOPINDIR_INPUT, IOPINRES_NONE, IOPINTYPE_NORMAL},
	{IOPORTA, 7, IOPINOP_FUNC5, IOPINDIR_OUTPUT, IOPINRES_NONE, IOPINTYPE_NORMAL},
	{IOPORTA, 4, IOPINOP_GPIO, IOPINDIR_OUTPUT, IOPINRES_NONE, IOPINTYPE_NORMAL},
};
static const IOPinCfg_t i2cPins[] = {
	{IOPORTB, 8, IOPINOP_FUNC4, IOPINDIR_BI, IOPINRES_PULLUP, IOPINTYPE_OPENDRAIN},
	{IOPORTB, 9, IOPINOP_FUNC4, IOPINDIR_BI, IOPINRES_PULLUP, IOPINTYPE_OPENDRAIN},
};
// Overwrite stack storage after Write returns, before the last IRQs run.
__attribute__((noinline)) static void ClobberStack()
{
	volatile uint8_t bytes[1024];
	for (unsigned i = 0; i < sizeof(bytes); ++i) bytes[i] = 0xEE;
}
// Reused pins test register programming only, not alternate-function routing.
extern "C" int main()
{
	static SPIDev_t spi[6];
	static I2CDev_t i2c[4];
	uint8_t data[300] = {};
	for (int n = 0; n < 6; ++n)
	{
		for (int async = 0; async < 2; ++async)
		{
			SPICfg_t cfg = {};
			cfg.DevNo = n; cfg.Mode = SPIMODE_MASTER; cfg.Phy = SPIPHY_NORMAL;
			cfg.pIOPinMap = spiPins; cfg.NbIOPins = 4;
			cfg.DataSize = 8; cfg.Rate = 1000000; cfg.ChipSel = SPICSEL_AUTO;
			cfg.BitOrder = SPIDATABIT_MSB; cfg.DummyByte = 0xA5;
			cfg.bIntEn = async; cfg.EvtCB = Event;
			CHECK(SPIInit(&spi[n], &cfg));
			CHECK(SPIGetRate(&spi[n]) == (n < 3 ? 1000000 : 781250));
			done = false;
			int ret = SPIRx(&spi[n], 0, data, sizeof(data));
			if (async)
			{
				CHECK(ret == -1);
				for (int k = 0; k < 100000 && !done; ++k) spiIrq[n]();
				CHECK(done && count == sizeof(data));
			}
			else CHECK(ret == sizeof(data));
			for (unsigned k = 0; k < sizeof(data); ++k) CHECK(data[k] == (uint8_t)(3 * k + 1));
		}
	}
	CHECK((RCC->D2CCIP1R & RCC_D2CCIP1R_SPI123SEL) == RCC_D2CCIP1R_SPI123SEL_2);
	for (int n = 0; n < 4; ++n)
	{
		for (int async = 0; async < 2; ++async)
		{
			I2CCfg_t cfg = {};
			cfg.DevNo = n; cfg.Type = I2CTYPE_STANDARD; cfg.Mode = I2CMODE_MASTER;
			cfg.pIOPinMap = i2cPins; cfg.NbIOPins = 2; cfg.Rate = 400000;
			cfg.AddrType = I2CADDR_TYPE_NORMAL; cfg.bIntEn = async; cfg.EvtCB = Event;
			CHECK(I2CInit(&i2c[n], &cfg));
			CHECK(I2CGetRate(&i2c[n]) > 350000 && I2CGetRate(&i2c[n]) <= 400000);
			CHECK(I2CStartTx(&i2c[n], 0x50)); I2CStopTx(&i2c[n]);
			CHECK(!I2CStartTx(&i2c[n], 0x23));
			done = false;
			int ret = I2CRx(&i2c[n], 0x50, data, sizeof(data));
			if (async)
			{
				CHECK(ret == -1);
				for (int k = 0; k < 100000 && !done; ++k) i2cIrq[n]();
				CHECK(done && count == sizeof(data));
			}
			else CHECK(ret == sizeof(data));
			for (unsigned k = 0; k < sizeof(data); ++k)
				CHECK(data[k] == (uint8_t)(((k + async * 300) & 255) * 5 + 1));
		}
		// Command + payload crossing the old staging limit and NBYTES limit.
		// The host model dispatches actual vectors only for enabled IRQ flags.
		static const int lengths[] = {1, 63, 64, 65, 255, 256, 600};
		uint8_t payload[599];
		for (unsigned k = 0; k < sizeof(payload); ++k) payload[k] = (uint8_t)(7 * k + 3);
		for (int len : lengths)
		{
			uint8_t cmd = 0x55;
			done = false; events = 0; eventInIrq = false;
			I2cIrqTestPhase = (n + 1) * 1000 + len;
			CHECK(I2CWrite(&i2c[n], 0x50, &cmd, 1, payload, len - 1) == 0);
			ClobberStack();
			for (int k = 0; k < 1000000 && !done; ++k) __NOP();
			CHECK(done && count == len && events == 1 && eventInIrq);
			I2cIrqTestPhase = 0;
			i2cIrq[n]();
			CHECK(events == 1 && i2c[n].DevIntrf.bTxReady);
			CHECK(I2CStartTx(&i2c[n], 0x50)); I2CStopTx(&i2c[n]);
		}
		// NACK before and after staging refills releases the temporary source
		// and reports exactly one short completion. The next write must work.
		static const int nackCounts[] = {2, 69, 254};
		for (int k = 0; k < 3; ++k)
		{
			uint8_t cmd = 0x55;
			done = false; events = 0; eventInIrq = false;
			I2cIrqTestPhase = (n + 1) * 1000 + 700 + k;
			CHECK(I2CWrite(&i2c[n], 0x51 + k, &cmd, 1, payload, 599) == 0);
			ClobberStack();
			for (int j = 0; j < 1000000 && !done; ++j) __NOP();
			CHECK(done && count == nackCounts[k] && events == 1 && eventInIrq);
			I2cIrqTestPhase = 0;
			i2cIrq[n]();
			CHECK(events == 1 && i2c[n].DevIntrf.bTxReady);
			done = false;
			CHECK(I2CTx(&i2c[n], 0x50, payload, 8) == -1);
			for (int j = 0; j < 100000 && !done; ++j) i2cIrq[n]();
			CHECK(done && count == 8);
		}
		// No IRQ dispatch: the source wait must abort, detach the source and
		// release busy once. A late vector must not complete a second time.
		SystemCoreClock = 4000; // Shorten the instruction-count timeout only.
		done = false; events = 0; eventInIrq = true;
		CHECK(I2CTx(&i2c[n], 0x50, payload, 255) == -1);
		CHECK(done && count == 0 && events == 1 && !eventInIrq);
		SystemCoreClock = 400000000;
		ClobberStack();
		i2cIrq[n]();
		CHECK(events == 1 && i2c[n].DevIntrf.bTxReady);
		done = false;
		CHECK(I2CTx(&i2c[n], 0x50, payload, 8) == -1);
		for (int j = 0; j < 100000 && !done; ++j) i2cIrq[n]();
		CHECK(done && count == 8);
		CHECK(I2CSetRate(&i2c[n], 1000000) > 900000);
		CHECK(SYSCFG->PMCR & (SYSCFG_PMCR_I2C1_FMP << n));
		I2CSetRate(&i2c[n], 100000);
		CHECK(!(SYSCFG->PMCR & (SYSCFG_PMCR_I2C1_FMP << n)));
	}
	// DMA must move data on every hardware instance, in both execution modes.
	// Buffers are deliberately unaligned and reside in DTCM, with guards.
	uint8_t dmaBuf[1204];
	for (int n = 0; n < 6; n++)
	{
		spi[n].DevIntrf.PowerOff(&spi[n].DevIntrf);
		for (int async = 0; async < 2; async++)
		{
			SPICfg_t cfg = {};
			cfg.DevNo = n; cfg.Mode = SPIMODE_MASTER; cfg.Phy = SPIPHY_NORMAL;
			cfg.pIOPinMap = spiPins; cfg.NbIOPins = 4; cfg.DataSize = 8;
			cfg.Rate = 1000000; cfg.ChipSel = SPICSEL_AUTO; cfg.DummyByte = 0xA5;
			cfg.bDmaEn = true; cfg.bIntEn = async; cfg.EvtCB = Event;
			CHECK(SPIInit(&spi[n], &cfg));
			CHECK(spi[n].DevIntrf.bDma);
			dmaBuf[0] = dmaBuf[1201] = 0xCC;
			done = false; events = 0; DmaTestActive = async;
			int ret = SPIRx(&spi[n], 0, dmaBuf + 1, 1200);
			if (async)
			{
				CHECK(ret == -1);
				for (int k = 0; k < 1000000 && !done; k++) __NOP();
				CHECK(done && count == 1200 && events == 1);
			}
			else CHECK(ret == 1200);
			for (int k = 0; k < 1200; k++) CHECK(dmaBuf[k+1] == (uint8_t)(3*k+1));
			CHECK(dmaBuf[0] == 0xCC && dmaBuf[1201] == 0xCC);
			done = false; events = 0;
			uint8_t cmd = 0x55;
			ret = DeviceIntrfWrite(&spi[n].DevIntrf, 0, &cmd, 1, dmaBuf + 1, 1199);
			if (async)
			{
				CHECK(ret == 0);
				ClobberStack();
				for (int k = 0; k < 1000000 && !done; k++) __NOP();
				CHECK(done && count == 1200 && events == 1);
			}
			else CHECK(ret == 1199);
			DmaTestActive = 0;
			spi[n].DevIntrf.PowerOff(&spi[n].DevIntrf);
		}
	}
	for (int n = 0; n < 4; n++)
	{
		i2c[n].DevIntrf.PowerOff(&i2c[n].DevIntrf);
		for (int async = 0; async < 2; async++)
		{
			I2CCfg_t cfg = {};
			cfg.DevNo = n; cfg.Type = I2CTYPE_STANDARD; cfg.Mode = I2CMODE_MASTER;
			cfg.pIOPinMap = i2cPins; cfg.NbIOPins = 2; cfg.Rate = 400000;
			cfg.AddrType = I2CADDR_TYPE_NORMAL; cfg.bIntEn = async; cfg.bDmaEn = true; cfg.EvtCB = Event;
			CHECK(I2CInit(&i2c[n], &cfg));
			CHECK(i2c[n].DevIntrf.bDma);
			for (int k = 0; k < 1200; k++) dmaBuf[k+1] = (uint8_t)(k*7);
			dmaBuf[0] = dmaBuf[1201] = 0xCC;
			done = false; events = 0; DmaTestActive = async;
			int ret = I2CTx(&i2c[n], 0x50, dmaBuf + 1, 1200);
			if (async)
			{
				CHECK(ret == -1);
				ClobberStack();
				for (int k = 0; k < 1000000 && !done; k++) __NOP();
				CHECK(done && count == 1200 && events == 1);
			}
			else CHECK(ret == 1200);
			done = false; events = 0;
			ret = I2CRx(&i2c[n], 0x50, dmaBuf + 1, 1200);
			if (async)
			{
				CHECK(ret == -1);
				for (int k = 0; k < 1000000 && !done; k++) __NOP();
				CHECK(done && count == 1200 && events == 1);
			}
			else CHECK(ret == 1200);
			for (int k = 0; k < 1200; k++) CHECK(dmaBuf[k+1] == (uint8_t)((1200+k)*7));
			CHECK(dmaBuf[0] == 0xCC && dmaBuf[1201] == 0xCC);
			static const int refused[] = {2, 69, 254};
			for (int bad = 0; bad < 3; bad++)
			{
				done = false; events = 0;
				ret = I2CTx(&i2c[n], 0x51+bad, dmaBuf+1, 600);
				if (async)
				{
					CHECK(ret == -1);
					for (int k = 0; k < 1000000 && !done; k++) __NOP();
					CHECK(done && count == refused[bad] && events == 1);
				}
				else CHECK(ret == refused[bad]);
			}
			DmaTestActive = 0;
			i2c[n].DevIntrf.PowerOff(&i2c[n].DevIntrf);
		}
	}
	for (unsigned i = 0; i < sizeof(slaveTx); i++) slaveTx[i] = (uint8_t)(i*11+5);
	// Slave DMA: partial frame and a frame crossing the staging boundary.
	for (int bus = 0; bus < 2; bus++)
	for (int n = 0; n < (bus ? 6 : 4); n++)
	{
		DevIntrf_t *intrf;
		if (!bus)
		{
			I2CCfg_t cfg = {};
			cfg.DevNo = n; cfg.Type = I2CTYPE_STANDARD; cfg.Mode = I2CMODE_SLAVE;
			cfg.pIOPinMap = i2cPins; cfg.NbIOPins = 2; cfg.Rate = 400000;
			cfg.AddrType = I2CADDR_TYPE_NORMAL; cfg.bIntEn = true; cfg.bDmaEn = true; cfg.EvtCB = SlaveEvent;
			cfg.NbSlaveAddr = 1; cfg.SlaveAddr[0] = 0x30;
			CHECK(I2CInit(&i2c[n], &cfg)); intrf = &i2c[n].DevIntrf;
		}
		else
		{
			SPICfg_t cfg = {};
			cfg.DevNo = n; cfg.Mode = SPIMODE_SLAVE; cfg.Phy = SPIPHY_NORMAL;
			cfg.pIOPinMap = spiPins; cfg.NbIOPins = 4; cfg.DataSize = 8; cfg.Rate = 1000000;
			cfg.bIntEn = true; cfg.bDmaEn = true; cfg.EvtCB = SlaveEvent;
			CHECK(SPIInit(&spi[n], &cfg)); intrf = &spi[n].DevIntrf;
		}
		slaveRx[0] = slaveRx[801] = 0xCC;
		for (int part = 0; part < 2; part++)
		{
			slaveEvents = 0; slaveLength = 0;
			DmaSlaveCase = 1000 + bus*100 + n*2 + part;
			DmaTestActive = 1;
			for (int k = 0; k < 2000000 && slaveEvents < (bus ? 1 : 2); k++) __NOP();
			CHECK(slaveEvents == (bus ? 1 : 2));
			CHECK(slaveLength == (part ? 600 : 31));
			for (int k = 0; k < (part ? 600 : 31); k++) CHECK(slaveRx[k+1] == (uint8_t)(k*3+9));
			CHECK(slaveRx[0] == 0xCC && slaveRx[801] == 0xCC);
			DmaTestActive = 0;
		}
		intrf->PowerOff(intrf);
	}
	DmaSlaveCase = 0;
	// QSPI follows SPI6 and uses HCLK, with the H7's non-linear HPRE table.
	static SPIDev_t qspi;
	SPICfg_t qcfg = {};
	qcfg.DevNo = 6; qcfg.Mode = SPIMODE_MASTER; qcfg.Phy = SPIPHY_QUAD_SDR;
	qcfg.pIOPinMap = spiPins; qcfg.NbIOPins = 4; qcfg.Rate = 10000000;
	RCC->D1CFGR = RCC_D1CFGR_HPRE_DIV2;
	RCC->D1CCIPR |= RCC_D1CCIPR_QSPISEL;
	CHECK(SPIInit(&qspi, &qcfg));
	CHECK(SPIGetRate(&qspi) == 10000000);
	CHECK(!(RCC->D1CCIPR & RCC_D1CCIPR_QSPISEL));
	CHECK((QUADSPI->CR & QUADSPI_CR_PRESCALER) == (19U << QUADSPI_CR_PRESCALER_Pos));
	RCC->D1CFGR = RCC_D1CFGR_HPRE_DIV64;
	CHECK(SPISetRate(&qspi, 1000000) == 1041666);
	qcfg.bDmaEn = true;
	CHECK(!SPIInit(&qspi, &qcfg));
	return 0;
}
