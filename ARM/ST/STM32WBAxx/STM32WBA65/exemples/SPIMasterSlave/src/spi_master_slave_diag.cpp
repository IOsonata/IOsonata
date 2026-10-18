/**-------------------------------------------------------------------------
@file	spi_master_slave_diag.cpp

@brief	Diagnostics of the STM32WBA65 SPI master and slave example

Called by spi_master_slave.cpp when a frame does not complete. It prints the
SPI2 registers, the counters of the slave handler and the NVIC state of the
SPI2 interrupt.

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
#include <stdint.h>

#include "stm32wba65xx.h"

extern volatile uint32_t g_Stm32SpiSlaveTrace[8];

void SpiMasterSlaveDiagnostics(void)
{
	SPI_TypeDef *reg = SPI2;
	const uint32_t bit = 1UL << ((uint32_t)SPI2_IRQn & 31U);
	const uint32_t idx = (uint32_t)SPI2_IRQn >> 5;

	printf("Diagnostics\r\n");
	printf("SPI2: CR1=%08x CR2=%08x CFG1=%08x CFG2=%08x IER=%08x SR=%08x\r\n",
		(unsigned)reg->CR1, (unsigned)reg->CR2, (unsigned)reg->CFG1,
		(unsigned)reg->CFG2, (unsigned)reg->IER, (unsigned)reg->SR);
	printf("Handler: entries=%u status seen=%08x last status=%08x last IER=%08x\r\n",
		(unsigned)g_Stm32SpiSlaveTrace[0], (unsigned)g_Stm32SpiSlaveTrace[1],
		(unsigned)g_Stm32SpiSlaveTrace[2], (unsigned)g_Stm32SpiSlaveTrace[3]);
	printf("Handler: no pending enabled flag=%u first such status=%08x arms=%u ends=%u\r\n",
		(unsigned)g_Stm32SpiSlaveTrace[4], (unsigned)g_Stm32SpiSlaveTrace[7],
		(unsigned)g_Stm32SpiSlaveTrace[5], (unsigned)g_Stm32SpiSlaveTrace[6]);
	printf("NVIC SPI2: enabled=%d pending=%d active=%d\r\n",
		(NVIC->ISER[idx] & bit) != 0U, (NVIC->ISPR[idx] & bit) != 0U,
		(NVIC->IABR[idx] & bit) != 0U);
}
