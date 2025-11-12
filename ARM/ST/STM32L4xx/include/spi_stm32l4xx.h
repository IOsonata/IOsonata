/**-------------------------------------------------------------------------
@file	spi_stm32l4xx.h

@brief	SPI implementation specific to STM32L4xx

@author	Hoang Nguyen Hoan
@date	June. 3, 2019

@license

MIT

Copyright (c) 2019, I-SYST inc., all rights reserved

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

#ifndef __SPI_STM32L4XX_H__
#define __SPI_STM32L4XX_H__

#include <stdint.h>

#include "stm32l4xx.h"
#include "coredev/spi.h"

#ifdef STM32L4S9xx
#define STM32L4XX_SPI_MAXDEV		5
#define STM32L4XX_OSPI_DEVNO_START	3
#else
#define STM32L4XX_SPI_MAXDEV		4
#define STM32L4XX_QSPI_DEVNO_START	3
#endif

#pragma pack(push, 4)

typedef struct {
	int DevNo;
	SPIDEV *pSpiDev;
	union {
		SPI_TypeDef	*pReg;
#ifdef STM32L4S9xx
		OCTOSPI_TypeDef	*pOReg;
#else
		QUADSPI_TypeDef	*pQReg;
#endif
	};
	QOSPI_PHASE QOPhase;
	int AdSize;
	uint32_t CcrReg;	// used by QuadSPI only
} STM32L4XX_SPIDev_t;
#pragma pack(pop)

uint32_t STM32L4xxSPIGetRate(DevIntrf_t * const pDev);

#endif // __SPI_STM32L4XX_H__

