/**-------------------------------------------------------------------------
@file	device.cpp

@brief	Generic device base class

This is the base class to implement all sort devices, hardware or software.
For example a sensor device or a software audio decoder.  The device can transfer
data via it's DeviceIntrf object.

@author	Hoang Nguyen Hoan
@date	Feb. 12, 2017

@license

MIT License

Copyright (c) 2017 I-SYST inc. All rights reserved.

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

#include "device.h"

Device::Device()
{
	vDevAddr = 0;
	vpIntrf = NULL;
	vpTimer = NULL;
	vbValid = false;
	vDevId = -1;
	vbIntEn = false;
}

/**
 * @brief	Read device's register/memory block.
 *
 * This default implementation sets bit 7 of the Cmd/Addr byte for SPI read access as most
 * devices work this way on SPI interface. Overwrite this implementation if SPI access is different
 *
 * @param 	pCmdAddr 	: Buffer containing command or address to be written
 * 						  prior reading data back
 * @param	CmdAddrLen 	: Command buffer size
 * @param	pBuff		: Data buffer container
 * @param	BuffLen		: Data buffer size
 *
 * @return	Actual number of bytes read
 */
int Device::Read(uint8_t *pCmdAddr, int CmdAddrLen, uint8_t *pBuff, int BuffLen)
{
	uint8_t cmd = *pCmdAddr;

	if (vpIntrf->Type() == DEVINTRF_TYPE_SPI)
	{
		// Most sensor that supports SPI have this for reading registers
		// overload this function if different
		*pCmdAddr |= 0x80;
	}

	int cnt = vpIntrf->Read(vDevAddr, pCmdAddr, CmdAddrLen, pBuff, BuffLen);

	*pCmdAddr = cmd;

	return cnt;
}

/**
 * @brief	Write to device's register/memory block
 *
 * This default implementation clears bit 7 of the Cmd/Addr byte for SPI write access as most
 * devices work this way on SPI interface.  Overwrite this implementation if SPI access is different
 *
 * @param 	pCmdAddr 	: Buffer containing command or address to be written
 * 						  prior writing data back
 * @param	CmdAddrLen 	: Command buffer size
 * @param	pData		: Data buffer to be written to the device
 * @param	DataLen		: Size of data
 *
 * @return	Actual number of bytes written
 */
int Device::Write(uint8_t *pCmdAddr, int CmdAddrLen, const uint8_t *pData, int DataLen)
{
	if (vpIntrf->Type() == DEVINTRF_TYPE_SPI)
	{
		// Most sensor that supports SPI have this for writing registers
		// overload this function if different
		//*pCmdAddr &= 0x7F;
	}

	return vpIntrf->Write(vDevAddr, pCmdAddr, CmdAddrLen, pData, DataLen);
}
