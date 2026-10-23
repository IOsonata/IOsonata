/* STM32 I2C v1 polling master: STM32F1/F2/F4/L1 register architecture. */
#include <string.h>
#include "stm32.h"
#include "coredev/i2c.h"
#include "coredev/iopincfg.h"
#include "coredev/system_core_clock.h"

#if defined(STM32F401xE) || defined(STM32F401xC) || defined(STM32L152xE)
#if !defined(I2C_SR1_SB) || !defined(I2C_CCR_FS)
#error "I2C v1 peripheral register definitions are required"
#endif
#if defined(STM32L152xE)
#define STM32_I2CV1_COUNT 2
#else
#define STM32_I2CV1_COUNT 3
#endif

typedef struct {
	I2CDev_t *pDev;
	I2C_TypeDef *pReg;
	uint32_t Addr;
	uint32_t Rate;
	bool bActive;
	bool bRead;
} Stm32I2cv1_t;

static Stm32I2cv1_t s_I2c[STM32_I2CV1_COUNT];
static I2C_TypeDef * const s_Regs[STM32_I2CV1_COUNT] = {
	I2C1, I2C2,
#if STM32_I2CV1_COUNT == 3
	I2C3,
#endif
};
static const uint32_t s_En[STM32_I2CV1_COUNT] = {
	RCC_APB1ENR_I2C1EN, RCC_APB1ENR_I2C2EN,
#if STM32_I2CV1_COUNT == 3
	RCC_APB1ENR_I2C3EN,
#endif
};
static const uint32_t s_Rst[STM32_I2CV1_COUNT] = {
	RCC_APB1RSTR_I2C1RST, RCC_APB1RSTR_I2C2RST,
#if STM32_I2CV1_COUNT == 3
	RCC_APB1RSTR_I2C3RST,
#endif
};

static Stm32I2cv1_t *Stm32V1Handle(DevIntrf_t *p)
{
	Stm32I2cv1_t *d = (Stm32I2cv1_t *)p->pDevData;
	return d && d->pDev && &d->pDev->DevIntrf == p ? d : NULL;
}

static bool Stm32V1Wait(I2C_TypeDef *r, uint32_t mask)
{
	for (uint32_t i = 0; i < 200000U; i++)
	{
		uint32_t sr = r->SR1;
		if (sr & (I2C_SR1_BERR | I2C_SR1_ARLO | I2C_SR1_AF | I2C_SR1_OVR))
		{
			return false;
		}
		if ((sr & mask) == mask)
		{
			return true;
		}
	}
	return false;
}

static void Stm32V1Stop(Stm32I2cv1_t *d)
{
	d->pReg->CR1 |= I2C_CR1_STOP;
	d->bActive = false;
	d->pReg->CR1 |= I2C_CR1_ACK;
}

static uint32_t Stm32V1Rate(DevIntrf_t *p, uint32_t rate)
{
	Stm32I2cv1_t *d = Stm32V1Handle(p);
	if (!d || rate > 400000U)
	{
		return 0;
	}
	if (!rate)
	{
		rate = 100000U;
	}
	uint32_t clk = SystemPeriphClockGet(0);
	uint32_t mhz = clk / 1000000U;
	if (mhz < 2U || mhz > 42U)
	{
		return 0;
	}
	I2C_TypeDef *r = d->pReg;
	r->CR1 &= ~I2C_CR1_PE;
	r->CR2 = (r->CR2 & ~I2C_CR2_FREQ) | mhz;
	uint32_t ccr;
	if (rate <= 100000U)
	{
		ccr = clk / (rate * 2U);
		if (ccr < 4U) ccr = 4U;
		r->CCR = ccr;
		r->TRISE = mhz + 1U;
		d->Rate = clk / (ccr * 2U);
	}
	else
	{
		ccr = clk / (rate * 3U);
		if (ccr < 1U) ccr = 1U;
		r->CCR = I2C_CCR_FS | ccr;
		r->TRISE = (mhz * 300U) / 1000U + 1U;
		d->Rate = clk / (ccr * 3U);
	}
	r->CR1 |= I2C_CR1_PE | I2C_CR1_ACK;
	return d->Rate;
}

static uint32_t Stm32V1GetRate(DevIntrf_t *p)
{
	Stm32I2cv1_t *d = Stm32V1Handle(p);
	return d ? d->Rate : 0;
}

static void *Stm32V1GetHandle(DevIntrf_t *p)
{
	Stm32I2cv1_t *d = Stm32V1Handle(p);
	return d ? d->pDev : NULL;
}

static bool Stm32V1Start(DevIntrf_t *p, uint32_t addr, bool read)
{
	Stm32I2cv1_t *d = Stm32V1Handle(p);
	if (!d || addr > 0x7fU)
	{
		return false;
	}
	I2C_TypeDef *r = d->pReg;
	if (!d->bActive)
	{
		for (uint32_t i = 0; (r->SR2 & I2C_SR2_BUSY) != 0U; i++)
		{
			if (i >= 200000U) return false;
		}
	}
	r->CR1 |= I2C_CR1_START | I2C_CR1_ACK;
	if (!Stm32V1Wait(r, I2C_SR1_SB))
	{
		Stm32V1Stop(d);
		return false;
	}
	r->DR = (addr << 1) | (read ? 1U : 0U);
	if (!Stm32V1Wait(r, I2C_SR1_ADDR))
	{
		Stm32V1Stop(d);
		r->SR1 &= ~I2C_SR1_AF;
		return false;
	}
	d->Addr = addr;
	d->bActive = true;
	d->bRead = read;
	if (!read)
	{
		(void)r->SR1;
		(void)r->SR2;
	}
	return true;
}

static bool Stm32V1StartTx(DevIntrf_t *p, uint32_t addr)
{
	return Stm32V1Start(p, addr, false);
}

static bool Stm32V1StartRx(DevIntrf_t *p, uint32_t addr)
{
	return Stm32V1Start(p, addr, true);
}

static int Stm32V1TxData(DevIntrf_t *p, const uint8_t *data, int len)
{
	Stm32I2cv1_t *d = Stm32V1Handle(p);
	if (!d || !d->bActive || d->bRead || len < 0 || (len && !data)) return 0;
	I2C_TypeDef *r = d->pReg;
	int i = 0;
	for (; i < len; i++)
	{
		if (!Stm32V1Wait(r, I2C_SR1_TXE)) break;
		r->DR = data[i];
	}
	if (i == len && !Stm32V1Wait(r, I2C_SR1_BTF)) i = 0;
	if (!p->bNoStop || i != len) Stm32V1Stop(d);
	return i;
}

static int Stm32V1RxData(DevIntrf_t *p, uint8_t *data, int len)
{
	Stm32I2cv1_t *d = Stm32V1Handle(p);
	if (!d || !d->bActive || !d->bRead || len <= 0 || !data) return 0;
	I2C_TypeDef *r = d->pReg;
	int i = 0;
	if (len == 1)
	{
		r->CR1 &= ~I2C_CR1_ACK;
		uint32_t primask = __get_PRIMASK();
		__disable_irq();
		(void)r->SR1;
		(void)r->SR2;
		r->CR1 |= I2C_CR1_STOP;
		__set_PRIMASK(primask);
		if (Stm32V1Wait(r, I2C_SR1_RXNE)) data[i++] = (uint8_t)r->DR;
	}
	else if (len == 2)
	{
		r->CR1 |= I2C_CR1_POS;
		r->CR1 &= ~I2C_CR1_ACK;
		(void)r->SR1;
		(void)r->SR2;
		if (Stm32V1Wait(r, I2C_SR1_BTF))
		{
			uint32_t primask = __get_PRIMASK();
			__disable_irq();
			r->CR1 |= I2C_CR1_STOP;
			data[i++] = (uint8_t)r->DR;
			__set_PRIMASK(primask);
			data[i++] = (uint8_t)r->DR;
		}
		r->CR1 &= ~I2C_CR1_POS;
	}
	else
	{
		(void)r->SR1;
		(void)r->SR2;
		while (len - i > 3 && Stm32V1Wait(r, I2C_SR1_RXNE))
		{
			data[i++] = (uint8_t)r->DR;
		}
		if (len - i == 3 && Stm32V1Wait(r, I2C_SR1_BTF))
		{
			r->CR1 &= ~I2C_CR1_ACK;
			uint32_t primask = __get_PRIMASK();
			__disable_irq();
			data[i++] = (uint8_t)r->DR;
			if (Stm32V1Wait(r, I2C_SR1_BTF))
			{
				r->CR1 |= I2C_CR1_STOP;
				data[i++] = (uint8_t)r->DR;
			}
			__set_PRIMASK(primask);
			if (i == len - 1) data[i++] = (uint8_t)r->DR;
		}
	}
	d->bActive = false;
	r->CR1 |= I2C_CR1_ACK;
	return i;
}

static void Stm32V1StopTx(DevIntrf_t *p)
{
	Stm32I2cv1_t *d = Stm32V1Handle(p);
	if (d && d->bActive) Stm32V1Stop(d);
}

static void Stm32V1Enable(DevIntrf_t *p)
{
	Stm32I2cv1_t *d = Stm32V1Handle(p);
	if (d) d->pReg->CR1 |= I2C_CR1_PE;
}

static void Stm32V1Disable(DevIntrf_t *p)
{
	Stm32I2cv1_t *d = Stm32V1Handle(p);
	if (d) d->pReg->CR1 &= ~I2C_CR1_PE;
}

static void Stm32V1Reset(DevIntrf_t *p)
{
	Stm32I2cv1_t *d = Stm32V1Handle(p);
	if (d)
	{
		Stm32V1Stop(d);
		d->pReg->CR1 |= I2C_CR1_SWRST;
		d->pReg->CR1 &= ~I2C_CR1_SWRST;
		Stm32V1Rate(p, d->Rate);
	}
}

extern "C" bool I2CInit(I2CDev_t * const pDev, const I2CCfg_t *cfg)
{
	if (!pDev || !cfg || cfg->DevNo < 0 || cfg->DevNo >= STM32_I2CV1_COUNT ||
		cfg->Mode != I2CMODE_MASTER || cfg->bIntEn || cfg->bDmaEn ||
		cfg->AddrType != I2CADDR_TYPE_NORMAL || !cfg->pIOPinMap || cfg->NbIOPins < 2)
	{
		return false;
	}
	Stm32I2cv1_t *d = &s_I2c[cfg->DevNo];
	if (d->pDev && d->pDev != pDev) return false;
	d->pDev = pDev;
	d->pReg = s_Regs[cfg->DevNo];
	d->bActive = false;
	memcpy(&pDev->Cfg, cfg, sizeof(*cfg));
	DevIntrf_t *intrf = &pDev->DevIntrf;
	intrf->pDevData = d;
	intrf->Type = DEVINTRF_TYPE_I2C;
	intrf->Reset = Stm32V1Reset;
	intrf->Enable = Stm32V1Enable;
	intrf->Disable = Stm32V1Disable;
	intrf->PowerOff = Stm32V1Disable;
	intrf->GetHandle = Stm32V1GetHandle;
	intrf->GetRate = Stm32V1GetRate;
	intrf->SetRate = Stm32V1Rate;
	intrf->StartRx = Stm32V1StartRx;
	intrf->RxData = Stm32V1RxData;
	intrf->StopRx = Stm32V1StopTx;
	intrf->StartTx = Stm32V1StartTx;
	intrf->TxData = Stm32V1TxData;
	intrf->TxSrData = Stm32V1TxData;
	intrf->StopTx = Stm32V1StopTx;
	intrf->bDma = false;
	intrf->bIntEn = false;
	intrf->bNoStop = false;
	intrf->EnCnt = 1;
	RCC->APB1ENR |= s_En[cfg->DevNo];
	RCC->APB1RSTR |= s_Rst[cfg->DevNo];
	RCC->APB1RSTR &= ~s_Rst[cfg->DevNo];
	IOPinCfg(cfg->pIOPinMap, cfg->NbIOPins);
	return Stm32V1Rate(intrf, cfg->Rate) != 0U;
}

extern "C" void I2CBusReset(I2CDev_t *pDev)
{
	if (pDev) Stm32V1Reset(&pDev->DevIntrf);
}

extern "C" void I2CSetReadRqstData(I2CDev_t *, int, uint8_t *, int) {}
extern "C" void I2CSetWriteRqstBuffer(I2CDev_t *, int, uint8_t *, int) {}

#endif
