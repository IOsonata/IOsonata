/**-------------------------------------------------------------------------
@file	display_uc8151x.h

@brief	UC8151X EInk display controller implementation


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
#ifndef __DISPLAY_UC8151X_H__
#define __DISPLAY_UC8151X_H__

#include "display/display_epdmtrx.h"

// UC8151X default portrait
#define UC8151X_WIDTH_MAX					160
#define UC8151X_HEIGHT_MAX					296

#define UC8151X_CMD_PSR				0
#define UC8151X_CMD_PSR_RST_MASK			(1<<0)
#define UC8151X_CMD_PSR_RST					(0<<0)
#define UC8151X_CMD_PSR_SHD					(1<<1)	// Booster switch on
#define UC8151X_CMD_PSR_SHR					(1<<2)	// Source shift right
#define UC8151X_CMD_PSR_SHL					(0<<2)	// Source shift left
#define UC8151X_CMD_PSR_SCAN_DWN			(0<<3)	// Gate scan direction down
#define UC8151X_CMD_PSR_SCAN_UP				(1<<3)	// Gate scan direction up
#define UC8151X_CMD_PSR_KW_R				(0<<4)	// Pixel Black/White/Red mode
#define UC8151X_CMD_PSR_KW					(1<<4)	// Pixel Black/White mode
#define UC8151X_CMD_PSR_LUT_OTP				(0<<5)	// LUT from OTP
#define UC8151X_CMD_PSR_LUT_REG				(1<<5)	// LUT from register
#define UC8151X_CMD_PSR_RES_MASK			(3<<6)	// Resolution mask
#define UC8151X_CMD_PSR_RES_96_230			(0<<6)	// Resolution 96 x 230
#define UC8151X_CMD_PSR_RES_96_252			(1<<6)	// Resolution 96 x 252
#define UC8151X_CMD_PSR_RES_128_296			(2<<6)	// Resolution 128 x 296
#define UC8151X_CMD_PSR_RES_160_296			(3<<6)	// Resolution 160 x 296

#define UC8151X_CMD_PWR				1
#define UC8151X_CMD_POF				2
#define UC8151X_CMD_PFS				3
#define UC8151X_CMD_PON				4
#define UC8151X_CMD_PMES			5
#define UC8151X_CMD_BTST			6
#define UC8151X_CMD_DSLP			7
#define UC8151X_CMD_DTM1			0x10
#define UC8151X_CMD_DSP				0x11
#define UC8151X_CMD_DRF				0x12
#define UC8151X_CMD_DTM2			0x13
#define UC8151X_CMD_AUTO			0x17
#define UC8151X_CMD_LUTOPT			0x2A
#define UC8151X_CMD_PLL				0x30
#define UC8151X_CMD_TSC				0x40
#define UC8151X_CMD_TSE				0x41
#define UC8151X_CMD_TSW				0x42
#define UC8151X_CMD_TSR				0x43
#define UC8151X_CMD_PBC				0x44
#define UC8151X_CMD_CDI				0x50
#define UC8151X_CMD_LPD				0x51
#define UC8151X_CMD_TCON			0x60
#define UC8151X_CMD_TRES			0x61
#define UC8151X_CMD_TRES_HRES_MASK			(0x1F<<3)
#define UC8151X_CMD_TRES_VRES_MASK			(0x1FF)

#define UC8151X_CMD_GSST			0x65
#define UC8151X_CMD_REV				0x70
#define UC8151X_CMD_FLG				0x71
#define UC8151X_CMD_AMV				0x80
#define UC8151X_CMD_VV				0x81
#define UC8151X_CMD_VDCS			0x82
#define UC8151X_CMD_PTL				0x90
#define UC8151X_CMD_PTIN			0x91
#define UC8151X_CMD_PTOUT			0x92
#define UC8151X_CMD_PGM				0xA0
#define UC8151X_CMD_APG				0xA1
#define UC8151X_CMD_ROTP			0xA2
#define UC8151X_CMD_CCSET			0xE0
#define UC8151X_CMD_PWS				0xE3
#define UC8151X_CMD_LVSEL			0xE4
#define UC8151X_CMD_TSSET			0xE5

#define UC8151X_LINE_BUFFLEN					(UC8151X_HEIGHT_MAX >> 2)

class EpdUC8151x : public EpdMatrix {
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
	//virtual void Clear();

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


private:
	void Refresh();

	uint8_t vFrameBuff[(UC8151X_WIDTH_MAX >> 2) * UC8151X_HEIGHT_MAX];
	//int vFrameBuffSize;
};


#endif // __DISPLAY_UC8151X_H__
