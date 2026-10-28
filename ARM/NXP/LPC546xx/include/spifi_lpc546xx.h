/**-------------------------------------------------------------------------
@file	spifi_lpc546xx.h

@brief	LPC546xx SPIFI, the quad SPI flash interface, as a QSPI device

SPIInit of spi_lpc546xx.cpp gives SPI DevNo LPC546XX_SPIFI_DEVNO, after the
ten Flexcomm SPI, to Lpc546xxSpifiInit. Master in polling mode, command mode
only: the flash is not memory mapped. Flash commands go through
QuadSPISendCmd, then RxData or TxData for their data, as on the other QSPI
ports.

Configuration:
	Mode		SPIMODE_MASTER
	Phy			SPIPHY_NORMAL, SPIPHY_DUAL or SPIPHY_QUAD_SDR. The commands
				that use more data lines than the Phy are refused.
	pIOPinMap	SCK, IO0 to IO3, CS (QSPI_CS_IOPIN_IDX), at least these 6
				pins with their SPIFI functions. CS is driven by the SPIFI,
				only DevCs 0 exists.
	ChipSel		SPICSEL_AUTO
	DataSize	8, MSB first
	ClkPol, DataPhase	mode 0 or mode 3
	bIntEn, bDmaEn	false

Clock: the main clock divided by 1 to 256, the closest rate to the request.

@author	Hoang Nguyen Hoan
@date	Oct. 27, 2026

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
#ifndef __SPIFI_LPC546XX_H__
#define __SPIFI_LPC546XX_H__

#include "coredev/spi.h"

/// SPI DevNo of the SPIFI, after the Flexcomm SPI 0 to 9
#define LPC546XX_SPIFI_DEVNO		10

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief	Initialize the SPIFI as the QSPI device pDev.
 *
 * Called by SPIInit for DevNo LPC546XX_SPIFI_DEVNO.
 *
 * @param	pDev		: SPI device data
 * @param	pCfgData	: Configuration, see the file header
 *
 * @return	true on success, false when the configuration is not supported
 */
bool Lpc546xxSpifiInit(SPIDev_t * const pDev, const SPICfg_t *pCfgData);

#ifdef __cplusplus
}
#endif

#endif // __SPIFI_LPC546XX_H__
