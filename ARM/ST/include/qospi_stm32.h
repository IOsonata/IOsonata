/**-------------------------------------------------------------------------
@file	qospi_stm32.h

@brief	STM32 QUADSPI and OCTOSPI, the SPI devices after the SPI instances

		SPIInit of spi_stm32.cpp hands a DevNo from STM32_QOSPI_DEVNO_START
		to Stm32QospiInit. quadspi_stm32.cpp implements it on the parts that
		have QUADSPI (DevNo 3), octospi_stm32.cpp on the parts that have
		OCTOSPI1 and OCTOSPI2 (DevNo 3 and 4). A library links the one of its
		part. Master in polling mode only.

@author	Hoang Nguyen Hoan
@date	Oct. 19, 2026

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
#ifndef __QOSPI_STM32_H__
#define __QOSPI_STM32_H__

#include "stm32.h"
#include "coredev/spi.h"

#if defined(QUADSPI) || defined(OCTOSPI1)

/// First DevNo of the QUADSPI or OCTOSPI instances, after SPI1 to SPI3
#define STM32_QOSPI_DEVNO_START		3

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief	Initialize the QUADSPI or OCTOSPI instance of pCfgData->DevNo.
 *
 * Master in polling mode only: slave, interrupt and DMA configurations, and
 * a DevNo that is not a QUADSPI or OCTOSPI instance, return false before
 * the device or the hardware is changed.
 *
 * @return	true - success
 */
bool Stm32QospiInit(SPIDev_t * const pDev, const SPICfg_t *pCfgData);

#ifdef __cplusplus
}
#endif

#endif // QUADSPI || OCTOSPI1

#endif // __QOSPI_STM32_H__
