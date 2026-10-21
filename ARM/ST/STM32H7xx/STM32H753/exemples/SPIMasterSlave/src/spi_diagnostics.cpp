/**-------------------------------------------------------------------------
@file	spi_diagnostics.cpp

@brief	Read-only failure diagnostics for H753 SPIMasterSlave.

@author	Hoang Nguyen Hoan
@date	Oct. 18, 2026

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

// Read-only failure diagnostics for the H753 SPI master/slave example.
// The shared application calls this hook only after a failed frame.
#include "stddev.h"
#include "stm32.h"

static void SpiRegisters(const char *Name, SPI_TypeDef *Reg)
{
	printf("%s CR1=%08lx CR2=%08lx CFG1=%08lx CFG2=%08lx IER=%08lx SR=%08lx\r\n",
		Name, (unsigned long)Reg->CR1, (unsigned long)Reg->CR2,
		(unsigned long)Reg->CFG1, (unsigned long)Reg->CFG2,
		(unsigned long)Reg->IER, (unsigned long)Reg->SR);
}

static void PinRegisters(const char *Name, GPIO_TypeDef *Reg)
{
	printf("%s MODER=%08lx AFRL=%08lx AFRH=%08lx IDR=%04lx ODR=%04lx\r\n",
		Name, (unsigned long)Reg->MODER, (unsigned long)Reg->AFR[0],
		(unsigned long)Reg->AFR[1], (unsigned long)Reg->IDR,
		(unsigned long)Reg->ODR);
	printf("%s OSPEEDR=%08lx OTYPER=%04lx PUPDR=%08lx\r\n",
		Name, (unsigned long)Reg->OSPEEDR, (unsigned long)Reg->OTYPER,
		(unsigned long)Reg->PUPDR);
}

void SpiMasterSlaveDiagnostics(void)
{
	// Never read RXDR: doing so would consume received data.
	SpiRegisters("SPI1", SPI1);
	SpiRegisters("SPI3", SPI3);
	PinRegisters("GPIOA", GPIOA);
	PinRegisters("GPIOB", GPIOB);
	PinRegisters("GPIOC", GPIOC);
	PinRegisters("GPIOD", GPIOD);
	printf("RCC CR=%08lx D1CCIPR=%08lx D2CCIP1R=%08lx APB1LENR=%08lx APB2ENR=%08lx\r\n",
		(unsigned long)RCC->CR, (unsigned long)RCC->D1CCIPR,
		(unsigned long)RCC->D2CCIP1R, (unsigned long)RCC->APB1LENR,
		(unsigned long)RCC->APB2ENR);
	printf("NSS EXTI EXTICR4=%08lx RTSR1=%08lx FTSR1=%08lx IMR1=%08lx PR1=%08lx\r\n",
		(unsigned long)SYSCFG->EXTICR[3], (unsigned long)EXTI->RTSR1,
		(unsigned long)EXTI->FTSR1, (unsigned long)EXTI_D1->IMR1,
		(unsigned long)EXTI_D1->PR1);
	printf("NVIC ISER1=%08lx ISPR1=%08lx IABR1=%08lx\r\n",
		(unsigned long)NVIC->ISER[1], (unsigned long)NVIC->ISPR[1],
		(unsigned long)NVIC->IABR[1]);
}
