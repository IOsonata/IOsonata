/**-------------------------------------------------------------------------
@file	i2c_master_slave_diag.cpp

@brief	Diagnostics of the STM32WBA65 I2C master and slave example

Called by i2c_master_slave.cpp after a failed run. It prints the I2C1 and
I2C2 registers and the level of the four bus pins, then checks the two
jumpers by driving each master pin low and reading the slave pin.

@author	Hoang Nguyen Hoan
@date	Oct. 17, 2026

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

#include <stdio.h>

#include "stm32wbaxx.h"
#include "coredev/iopincfg.h"
#include "iopinctrl.h"
#include "board.h"

static void DumpUnit(const char *pName, I2C_TypeDef *pReg)
{
	printf("%s: CR1=%08lx CR2=%08lx OAR1=%08lx OAR2=%08lx TIMINGR=%08lx ISR=%08lx\r\n",
		pName, (unsigned long)pReg->CR1, (unsigned long)pReg->CR2,
		(unsigned long)pReg->OAR1, (unsigned long)pReg->OAR2,
		(unsigned long)pReg->TIMINGR, (unsigned long)pReg->ISR);
}

static void DumpPin(const char *pName, int Port, int Pin)
{
	GPIO_TypeDef *reg = Port == IOPORTB ? GPIOB : GPIOA;
	uint32_t af = reg->AFR[Pin >> 3] >> ((Pin & 7) * 4) & 0xFU;

	printf("%s: MODER=%lu OTYPER=%lu PUPDR=%lu AF=%lu IDR=%lu\r\n", pName,
		(unsigned long)(reg->MODER >> (Pin * 2) & 3U),
		(unsigned long)(reg->OTYPER >> Pin & 1U),
		(unsigned long)(reg->PUPDR >> (Pin * 2) & 3U),
		(unsigned long)af, (unsigned long)(reg->IDR >> Pin & 1U));
}

static void Jumper(const char *pName, int OutPort, int OutPin, int InPort, int InPin)
{
	int high;
	int low;

	IOPinConfig(InPort, InPin, IOPINOP_GPIO, IOPINDIR_INPUT, IOPINRES_NONE, IOPINTYPE_NORMAL);
	IOPinConfig(OutPort, OutPin, IOPINOP_GPIO, IOPINDIR_OUTPUT, IOPINRES_NONE, IOPINTYPE_NORMAL);
	IOPinSet(OutPort, OutPin);
	for (volatile int i = 0; i < 1000; ++i)
		__NOP();
	high = IOPinRead(InPort, InPin);
	IOPinClear(OutPort, OutPin);
	for (volatile int i = 0; i < 1000; ++i)
		__NOP();
	low = IOPinRead(InPort, InPin);
	printf("%s: out high -> in %d, out low -> in %d (expect 1 then 0)\r\n", pName, high, low);
}

static void DumpGpdma(void)
{
	DMA_Channel_TypeDef * const ch[8] = {
		GPDMA1_Channel0, GPDMA1_Channel1, GPDMA1_Channel2, GPDMA1_Channel3,
		GPDMA1_Channel4, GPDMA1_Channel5, GPDMA1_Channel6, GPDMA1_Channel7,
	};

	for (int i = 0; i < 8; ++i)
	{
		if (ch[i]->CCR == 0U && ch[i]->CSR == 0U)
			continue;
		printf("GPDMA%d: CCR=%08lx CSR=%08lx CBR1=%08lx CTR2=%08lx NVIC en=%lu pend=%lu\r\n", i,
			(unsigned long)ch[i]->CCR, (unsigned long)ch[i]->CSR,
			(unsigned long)ch[i]->CBR1, (unsigned long)ch[i]->CTR2,
			(unsigned long)NVIC_GetEnableIRQ((IRQn_Type)(GPDMA1_Channel0_IRQn + i)),
			(unsigned long)NVIC_GetPendingIRQ((IRQn_Type)(GPDMA1_Channel0_IRQn + i)));
	}
	printf("I2C1 NVIC ev en=%lu pend=%lu, I2C2 ev en=%lu pend=%lu\r\n",
		(unsigned long)NVIC_GetEnableIRQ(I2C1_EV_IRQn), (unsigned long)NVIC_GetPendingIRQ(I2C1_EV_IRQn),
		(unsigned long)NVIC_GetEnableIRQ(I2C2_EV_IRQn), (unsigned long)NVIC_GetPendingIRQ(I2C2_EV_IRQn));
}

void I2cMasterSlaveDiagnostics(const char *pWhen)
{
	printf("Diagnostics %s\r\n", pWhen);
	DumpUnit("I2C1", I2C1);
	DumpUnit("I2C2", I2C2);
	DumpGpdma();
	DumpPin("master SDA", I2C_MASTER_SDA_PORT, I2C_MASTER_SDA_PIN);
	DumpPin("master SCL", I2C_MASTER_SCL_PORT, I2C_MASTER_SCL_PIN);
	DumpPin("slave  SDA", I2C_SLAVE_SDA_PORT, I2C_SLAVE_SDA_PIN);
	DumpPin("slave  SCL", I2C_SLAVE_SCL_PORT, I2C_SLAVE_SCL_PIN);
	printf("Jumper test, the I2C pins are left as GPIO\r\n");
	Jumper("SDA", I2C_MASTER_SDA_PORT, I2C_MASTER_SDA_PIN, I2C_SLAVE_SDA_PORT, I2C_SLAVE_SDA_PIN);
	Jumper("SCL", I2C_MASTER_SCL_PORT, I2C_MASTER_SCL_PIN, I2C_SLAVE_SCL_PORT, I2C_SLAVE_SCL_PIN);
}
