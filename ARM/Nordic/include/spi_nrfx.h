/**-------------------------------------------------------------------------
@file	spi_nrfx.h

@brief	SPI implementation on nRFx series MCU

Note: SPI device are shared with other device such as I2C therefore be careful
not to use the same device number on an other device.

For 3 wire support : Set both MISO & MOSI to the same pin.
For QSPI support where available is the last device of the list.

@author	Hoang Nguyen Hoan
@date	Apr. 11, 2021

@license

Copyright (c) 2021, I-SYST inc., all rights reserved

Permission to use, copy, modify, and distribute this software for any purpose
with or without fee is hereby granted, provided that the above copyright
notice and this permission notice appear in all copies, and none of the
names : I-SYST or its contributors may be used to endorse or
promote products derived from this software without specific prior written
permission.

For info or contributing contact : hnhoan at i-syst dot com

THIS SOFTWARE IS PROVIDED BY THE REGENTS AND CONTRIBUTORS ``AS IS'' AND ANY
EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE REGENTS OR CONTRIBUTORS BE LIABLE FOR ANY
DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
(INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

----------------------------------------------------------------------------*/
#ifndef __SPI_NRFX_H__
#define __SPI_NRFX_H__

#include "nrf_peripherals.h"

#ifdef SPIM_PRESENT
#if defined(NRF52840_XXAA) || defined(NRF5340_XXAA_APPLICATION)
#define NRFX_SPI_MAXDEV			(SPIM_COUNT + 1)
#else
#define NRFX_SPI_MAXDEV			SPIM_COUNT
#endif
#else
#define NRFX_SPI_MAXDEV			SPI_COUNT
#endif

#define NRFX_SPISLAVE_MAXDEV	SPIS_COUNT
#define NRFX_SPI_DMA_MAXCNT		((1<<SPIS0_EASYDMA_MAXCNT_SIZE)-1)

#pragma pack(push, 4)
typedef struct {
	int DevNo;
	SPIDev_t *pSpiDev;
	union {
#ifdef SPI_PRESENT
		NRF_SPI_Type  *pReg;	// Master I/O register map
#endif
#ifdef SPIM_PRESENT
		NRF_SPIM_Type *pDmaReg;	// Master DMA register map
#endif
#ifdef SPIS_COUNT
		NRF_SPIS_Type *pDmaSReg;// Slave DMA register map
#endif
#if defined(NRF52840_XXAA) || defined(NRF5340_XXAA_APPLICATION)
		NRF_QSPI_Type *pQSpiReg;// QSPI Register map
#endif
	};
	uint8_t Cmd;	// Current QSPI command code
	uint32_t ParamLen;	// Command parameter length in bytes
	uint32_t Param[2];	// Command Parameter data
} NrfSpiDev_t;

//typedef NrfSpiDev_t		NRFX_SPIDEV;

typedef struct {
	uint32_t Freq;
	uint32_t RegVal;
} NrfSpiFreq_t;

//typedef NrfSpiFreq_t	NRFX_SPIFREQ;

#pragma pack(pop)

#endif	// __SPI_NRFX_H__
