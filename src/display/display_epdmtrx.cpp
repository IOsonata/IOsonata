/**-------------------------------------------------------------------------
@file	display_epdmtrx.cpp

@brief	Generic e-paper matrix display controller implementation


@author	Hoang Nguyen Hoan
@date	Apr. 23, 2022

@license

MIT License

Copyright (c) 2022, I-SYST inc., all rights reserved

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
#include "iopinctrl.h"
#include "display/display_epdmtrx.h"

bool EpdMatrix::Init(const DisplayCfg_t &Cfg, DeviceIntrf * const pIntrf)
{
	if (pIntrf == nullptr)
	{
		return false;
	}

	Interface(pIntrf);

	vCfg = Cfg;

	DeviceAddress(vCfg.DevAddr);
	Type(DISPL_TYPE_MATRIX);

	IOPinCfg(vCfg.pPins, vCfg.NbPins);
	IOPinClear(vCfg.pPins[DISPL_CTRL_DCX_PINIDX].PortNo, vCfg.pPins[DISPL_CTRL_DCX_PINIDX].PinNo);

	Reset();

	return true;
}

/**
 * @brief	Power on or wake up device
 *
 * @return	true - If success
 */
bool EpdMatrix::Enable()
{
	return true;
}

/**
 * @brief	Put device in power down or power saving sleep mode
 *
 * This function is used to put the device in lowest power mode
 * possible so that the Enable function can wake up without full
 * initialization.
 */
void EpdMatrix::Disable()
{

}

/**
 * @brief	Clear screen
 *
 * @return	None
 */
void EpdMatrix::Clear()
{
	memset(vpFrameBuffer, 0xFF, vFrameBuffSize);
	Refresh();
}

/**
 * @brief	Render a null terminated string to the screen.
 *
 * Display driver must implement this to emulate a console line print
 *
 * @param 	pStr	: Null terminated string to print
 * @param 	Color	: Color to render
 */
void EpdMatrix::Print(char const *pStr, uint32_t Color)
{

}

/**
 * @brief	Fill region with color
 *
 * @param	StartX 	: Region start X coordinate in pixel
 * @param 	StartY 	: Region start Y coordinate in pixel
 * @param 	Width	: Region width in pixel
 * @param 	Height	: Region height in pixel
 * @param 	Color	: Color to fill
 *
 * @return	None
 */
void EpdMatrix::Fill(uint16_t StartX, uint16_t StartY, uint16_t Width, uint16_t Height, uint32_t Color)
{

}

/**
 * @brief	Set pixel color
 *
 * @param 	X		: Pixel X coordinate
 * @param 	Y		: Pixel Y coordinate
 * @param 	Color	: Color to set
 *
 * @return	None
 */
void EpdMatrix::SetPixel(uint16_t X, uint16_t Y, uint32_t Color)
{

}

/**
 * @brief	Transfer local graphic memory to display region
 *
 * @param 	StartX	: Region start X in pixel
 * @param 	StartY	: Region start Y in pixel
 * @param 	Width	: Region width in pixel
 * @param 	Height	: Region height in pixel
 * @param 	pMem	: Pointer to local graphics memory
 *
 * @return	None
 */
void EpdMatrix::BitBlt(uint16_t StartX, uint16_t StartY, uint16_t Width, uint16_t Height, uint8_t *pMem)
{

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
int EpdMatrix::Cmd(uint8_t Cmd, uint8_t *pData, int DataLen)
{
	int cnt = 0;

	IOPinClear(vCfg.pPins[DISPL_CTRL_DCX_PINIDX].PortNo, vCfg.pPins[DISPL_CTRL_DCX_PINIDX].PinNo);
	cnt += vpIntrf->Tx(DeviceAddress(), &Cmd, 1);

	if (pData != NULL && DataLen > 0)
	{
		IOPinSet(vCfg.pPins[DISPL_CTRL_DCX_PINIDX].PortNo, vCfg.pPins[DISPL_CTRL_DCX_PINIDX].PinNo);
		cnt += vpIntrf->Tx(DeviceAddress(), pData, DataLen);

		IOPinClear(vCfg.pPins[DISPL_CTRL_DCX_PINIDX].PortNo, vCfg.pPins[DISPL_CTRL_DCX_PINIDX].PinNo);
	}

	return cnt;
}

bool EpdMatrix::WaitBusy(int Timeout)
{
	do {
		Cmd(0x71, 0, 0);
		if (IOPinRead(vCfg.pPins[EPD_CTRL_BUSY_PINIDX].PortNo, vCfg.pPins[EPD_CTRL_BUSY_PINIDX].PinNo))
		{
			return true;
		}
	} while (--Timeout > 0);

	return false;
}

