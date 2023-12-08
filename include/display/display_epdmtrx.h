/**-------------------------------------------------------------------------
@file	display_epdmtrx.h

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
#ifndef __DISPLAY_EPDMTRX_H__
#define __DISPLAY_EPDMTRX_H__

#include "display/display.h"

#define EINKMTRX_CMD_PSR				0
#define EINKMTRX_CMD_PSR_RST_MASK				(1<<0)
#define EINKMTRX_CMD_PSR_RST					(0<<0)
#define EINKMTRX_CMD_PSR_SHD					(1<<1)	// Booster switch on
#define EINKMTRX_CMD_PSR_SHR					(1<<2)	// Source shift right
#define EINKMTRX_CMD_PSR_SHL					(0<<2)	// Source shift left
#define EINKMTRX_CMD_PSR_SCAN_DWN				(0<<3)	// Gate scan direction down
#define EINKMTRX_CMD_PSR_SCAN_UP				(1<<3)	// Gate scan direction up
#define EINKMTRX_CMD_PSR_KW_R					(0<<4)	// Pixel Black/White/Red mode
#define EINKMTRX_CMD_PSR_KW						(1<<4)	// Pixel Black/White mode
#define EINKMTRX_CMD_PSR_LUT_OTP				(0<<5)	// LUT from OTP
#define EINKMTRX_CMD_PSR_LUT_REG				(1<<5)	// LUT from register
#define EINKMTRX_CMD_PSR_RES_MASK				(3<<6)	// Resolution mask
#define EINKMTRX_CMD_PSR_RES_0					(0<<6)	// Resolution 0 (low) controller dependent
#define EINKMTRX_CMD_PSR_RES_1					(1<<6)	// Resolution 1 controller dependent
#define EINKMTRX_CMD_PSR_RES_2					(2<<6)	// Resolution 2 controller dependent
#define EINKMTRX_CMD_PSR_RES_3					(3<<6)	// Resolution 3 (high) controller dependent

#define EINKMTRX_CMD_PWR				1
#define EINKMTRX_CMD_POF				2
#define EINKMTRX_CMD_PFS				3
#define EINKMTRX_CMD_PON				4
#define EINKMTRX_CMD_PMES				5
#define EINKMTRX_CMD_BTST				6
#define EINKMTRX_CMD_DSLP				7
#define EINKMTRX_CMD_DTM1				0x10
#define EINKMTRX_CMD_DSP				0x11
#define EINKMTRX_CMD_DRF				0x12
#define EINKMTRX_CMD_DTM2				0x13
#define EINKMTRX_CMD_AUTO				0x17
#define EINKMTRX_CMD_LUTOPT				0x2A
#define EINKMTRX_CMD_PLL				0x30
#define EINKMTRX_CMD_TSC				0x40
#define EINKMTRX_CMD_TSE				0x41
#define EINKMTRX_CMD_TSW				0x42
#define EINKMTRX_CMD_TSR				0x43
#define EINKMTRX_CMD_PBC				0x44
#define EINKMTRX_CMD_CDI				0x50
#define EINKMTRX_CMD_LPD				0x51
#define EINKMTRX_CMD_TCON				0x60
#define EINKMTRX_CMD_TRES				0x61
#define EINKMTRX_CMD_GSST				0x65
#define EINKMTRX_CMD_REV				0x70
#define EINKMTRX_CMD_FLG				0x71
#define EINKMTRX_CMD_AMV				0x80
#define EINKMTRX_CMD_VV					0x81
#define EINKMTRX_CMD_VDCS				0x82
#define EINKMTRX_CMD_PTL				0x90
#define EINKMTRX_CMD_PTIN				0x91
#define EINKMTRX_CMD_PTOUT				0x92
#define EINKMTRX_CMD_PGM				0xA0
#define EINKMTRX_CMD_APG				0xA1
#define EINKMTRX_CMD_ROTP				0xA2
#define EINKMTRX_CMD_CCSET				0xE0
#define EINKMTRX_CMD_PWS				0xE3
#define EINKMTRX_CMD_LVSEL				0xE4
#define EINKMTRX_CMD_TSSET				0xE5


#define EPD_CTRL_BUSY_PINIDX		(DISPL_CTRL_RST_PINIDX + 1)	//!< Input display busy flag, 0 : busy
#define EPD_CTRL_BSI_PINIDX		(EINK_CTRL_BUSY_PINIDX + 1)	//!< serial interface select 0 : 4 wire, 1 : 3 wire

class EpdMatrix : public DisplayDotMatrix {
public:
	bool Init(const DisplayCfg_t &, DeviceIntrf * const pIntrf);

	/**
	 * @brief	Power on or wake up device
	 *
	 * @return	true - If success
	 */
	virtual bool Enable();

	/**
	 * @brief	Put device in power down or power saving sleep mode
	 *
	 * This function is used to put the device in lowest power mode
	 * possible so that the Enable function can wake up without full
	 * initialization.
	 */
	virtual void Disable();

	/**
	 * @brief	Clear screen
	 *
	 * @return	None
	 */
	virtual void Clear();

	/**
	 * @brief	Render a null terminated string to the screen.
	 *
	 * Display driver must implement this to emulate a console line print
	 *
	 * @param 	pStr	: Null terminated string to print
	 * @param 	Color	: Color to render
	 */
	virtual void Print(char const *pStr, uint32_t Color);

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
	virtual void Fill(uint16_t StartX, uint16_t StartY, uint16_t Width, uint16_t Height, uint32_t Color);

	/**
	 * @brief	Set pixel color
	 *
	 * @param 	X		: Pixel X coordinate
	 * @param 	Y		: Pixel Y coordinate
	 * @param 	Color	: Color to set
	 *
	 * @return	None
	 */
	virtual void SetPixel(uint16_t X, uint16_t Y, uint32_t Color);

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
	virtual void BitBlt(uint16_t StartX, uint16_t StartY, uint16_t Width, uint16_t Height, uint8_t *pMem);

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
	int Cmd(uint8_t Cmd, uint8_t *pData, int DataLen);

	bool WaitBusy(int Timeout);

protected:
	void SetFrameBuffer(uint8_t * const pBuff, int Size) { vpFrameBuffer = pBuff; vFrameBuffSize = Size; }

	int vFrameBuffSize;

private:
	virtual void Refresh() = 0;

	uint8_t *vpFrameBuffer;
};


#endif // __DISPLAY_EPDMTRX_H__
