/**-------------------------------------------------------------------------
@file	i2c_r9a02.h

@brief	R9A02G021 I2C, IICA master and slave, SAU simplified I2C master

coredev/i2c.h implementation. I2CCfg_t.DevNo selects the hardware:

	DevNo 0 and 1 : IICA0 and IICA1 (i2c_r9a02.cpp)
	DevNo 2 to 7  : SAU simplified I2C IIC00, IIC01, IIC10, IIC11, IIC20,
					IIC21 (i2c_sau_r9a02.cpp)

7-bit addresses. The IICA is master or slave and follows the I2C bus
specification, clock stretching and arbitration included; the slave works in
its interrupt. The simplified I2C is master only, without clock stretching
during a byte, arbitration and bus busy detection: one master, slaves that do
not stretch SCL.

With bIntEn the data of a transfer go in the background, the call returns -1
and DEVINTRF_EVT_COMPLETED marks the end. A transmission is copied first, up
to R9A02_I2C_TX_COPY_SIZE bytes, a longer one is done in the call and ends
the same way. On IICA0/1, bDmaEn uses software-triggered DTC transfers for
payload bytes, in master and slave modes. The CPU still services each byte
for ACK/NACK and protocol decisions. Completion semantics are unchanged.
IICA0 reserves ELC_SWEVT0 and IICA1 ELC_SWEVT1, plus one ICU slot each;
these software events must not be linked elsewhere while DMA is enabled.
The SAU simplified I2C still rejects bDmaEn.

@author	Hoang Nguyen Hoan
@date	Oct. 13, 2026

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
#ifndef __I2C_R9A02_H__
#define __I2C_R9A02_H__

#include <stdint.h>
#include <stdbool.h>

#include "coredev/i2c.h"

/// Largest transmission done in the background with bIntEn, per instance.
/// Overridable in the library build.
#ifndef R9A02_I2C_TX_COPY_SIZE
#define R9A02_I2C_TX_COPY_SIZE		32
#endif

#define R9A02_I2C_DEVNO_IIC00		2			//!< First simplified I2C DevNo

#ifdef __cplusplus

/// @brief	Simplified I2C initialization, called by I2CInit for DevNo 2 to 7.
///
/// @param	pDev	: I2C device data
/// @param	pCfg	: Configuration
///
/// @return	true on success
bool R9a02I2cSauInit(I2CDev_t * const pDev, const I2CCfg_t *pCfg);

/// @brief	Check whether a simplified I2C instance uses an I2C object.
///
/// @param	pDev	: I2C device data
/// @param	DevNo	: Instance not checked, -1 checks all
///
/// @return	true if another instance has pDev
bool R9a02I2cSauUses(const I2CDev_t *pDev, int DevNo);

/// @brief	Check whether an IICA instance uses an I2C object.
///
/// @param	pDev	: I2C device data
/// @param	DevNo	: Instance not checked, -1 checks all
///
/// @return	true if another instance has pDev
bool R9a02I2cIicaUses(const I2CDev_t *pDev, int DevNo);

#endif

#endif
