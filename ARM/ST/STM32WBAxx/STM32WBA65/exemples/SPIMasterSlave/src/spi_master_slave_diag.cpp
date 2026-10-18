/**-------------------------------------------------------------------------
@file	spi_master_slave_diag.cpp

@brief	Diagnostics of the STM32WBA65 SPI master and slave example

Called by spi_master_slave.cpp when a frame does not complete. It prints the
SPI2 registers and the NVIC state of the SPI2 interrupt, then checks the four
jumpers.

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
#include "coredev/iopincfg.h"
#include "iopinctrl.h"
#include "board.h"

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

void SpiMasterSlaveDiagnostics(void)
{
	SPI_TypeDef *reg = SPI2;
	const uint32_t bit = 1UL << ((uint32_t)SPI2_IRQn & 31U);
	const uint32_t idx = (uint32_t)SPI2_IRQn >> 5;

	printf("Diagnostics\r\n");
	printf("SPI2: CR1=%08x CR2=%08x CFG1=%08x CFG2=%08x IER=%08x SR=%08x\r\n",
		(unsigned)reg->CR1, (unsigned)reg->CR2, (unsigned)reg->CFG1,
		(unsigned)reg->CFG2, (unsigned)reg->IER, (unsigned)reg->SR);
	printf("NVIC SPI2: enabled=%d pending=%d active=%d\r\n",
		(NVIC->ISER[idx] & bit) != 0U, (NVIC->ISPR[idx] & bit) != 0U,
		(NVIC->IABR[idx] & bit) != 0U);
	printf("Jumper test, the SPI pins are left as GPIO\r\n");
	Jumper("SCK  D15-D13", SPI_MASTER_SCK_PORT, SPI_MASTER_SCK_PIN, SPI_SLAVE_SCK_PORT, SPI_SLAVE_SCK_PIN);
	Jumper("MOSI D6-D11 ", SPI_MASTER_MOSI_PORT, SPI_MASTER_MOSI_PIN, SPI_SLAVE_MOSI_PORT, SPI_SLAVE_MOSI_PIN);
	Jumper("CS   D8-D10 ", SPI_MASTER_CS_PORT, SPI_MASTER_CS_PIN, SPI_SLAVE_CS_PORT, SPI_SLAVE_CS_PIN);
	Jumper("MISO D12-D4 ", SPI_SLAVE_MISO_PORT, SPI_SLAVE_MISO_PIN, SPI_MASTER_MISO_PORT, SPI_MASTER_MISO_PIN);
}
