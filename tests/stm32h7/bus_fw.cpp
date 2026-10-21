// H753 shared I2C/SPI register-model test. No hardware timing validation.
#include "coredev/i2c.h"
#include "coredev/spi.h"
#include "stm32h753xx.h"
#define CHECK(x) do { if (!(x)) return __LINE__; } while (0)
extern "C" {
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
static int Event(DevIntrf_t *, DEVINTRF_EVT evt, uint8_t *, int len)
{
	if (evt == DEVINTRF_EVT_COMPLETED) { count = len; done = true; }
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
		CHECK(I2CSetRate(&i2c[n], 1000000) > 900000);
		CHECK(SYSCFG->PMCR & (SYSCFG_PMCR_I2C1_FMP << n));
		I2CSetRate(&i2c[n], 100000);
		CHECK(!(SYSCFG->PMCR & (SYSCFG_PMCR_I2C1_FMP << n)));
	}
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
