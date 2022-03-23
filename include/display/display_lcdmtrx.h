/**-------------------------------------------------------------------------
@file	display_lcdmtrx.h

@brief	Generic LCD matrix display controller definitions


@author	Hoang Nguyen Hoan
@date	Mar. 18, 2022

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
#ifndef __DISPLAY_LCD_H__
#define __DISPLAY_LCD_H__

#include "display.h"

// Standard LCD commands 
#define LCDMTRX_CMD_NOP							0		// NOP
#define LCDMTRX_CMD_SWRESET						1		// Software reset
#define LCDMTRX_CMD_RD_DID						4		// Read display ID
#define LCDMTRX_CMD_RD_DSI_NBERR				5		// Read number of errors on DSI
#define LCDMTRX_CMD_RD_RED						6		// Read Red color
#define LCDMTRX_CMD_RD_GREEN					7		// Read Green color
#define LCDMTRX_CMD_RD_BLUE						8		// Read blue color
#define LCDMTRX_CMD_RD_STATUS					9		// Read display status
#define LCDMTRX_CMD_RD_PWRMODE					0xA		// Read display power mode
#define LCDMTRX_CMD_RD_MADCTL					0xB		// Read display
#define LCDMTRX_CMD_RD_COLMOD					0xC		// Read display pixel format
#define LCDMTRX_CMD_RD_IMAGEMODE				0xD		// Read display image mode
#define LCDMTRX_CMD_RD_SIGNALMODE				0xE		// Read display signal mode
#define LCDMTRX_CMD_RD_DIAGRES					0xF		// Read display self-diagnostic result
#define LCDMTRX_CMD_SLEEPIN						0x10	// Enter sleep mode
#define LCDMTRX_CMD_SLEEPOUT					0x11	// Wake up
#define LCDMTRX_CMD_PARTIALON					0x12	// Partial mode on
#define LCDMTRX_CMD_NORMALON					0x13	// Normal mode on (Partial mode off)
#define LCDMTRX_CMD_INVOFF						0x20	// Display inversion off
#define LCDMTRX_CMD_INVON						0x21	// Display inversion on
#define LCDMTRX_CMD_ALLPOFF						0x22	// All pixels off
#define LCDMTRX_CMD_ALLPON						0x23	// All pixels on
#define LCDMTRX_CMD_GAMSET						0x26	// Gamma set
#define LCDMTRX_CMD_DISPOFF						0x28	// Display off
#define LCDMTRX_CMD_DISPON						0x29	// Display on
#define LCDMTRX_CMD_CASET						0x2A	// Column address set
#define LCDMTRX_CMD_RASET						0x2B	// Row address set
#define LCDMTRX_CMD_RAMWR						0x2C	// Memory write
#define LCDMTRX_CMD_COLOR_SET					0x2D	// Set color LUT
#define LCDMTRX_CMD_RAMRD						0x2E	// Memory read
#define LCDMTRX_CMD_PARTIAL_REGION				0x30	// Set partial region start/end addresses
#define LCDMTRX_CMD_VSCROLLDEF					0x33	// Vertical scrolling definition
#define LCDMTRX_CMD_TEAR_OFF					0x34	// Tearing effect line off
#define LCDMTRX_CMD_TEAR_ON						0x35	// Tearing effect line on

#define LCDMTRX_CMD_MADCTL						0x36	// Memory data access control
#define LCDMTRX_CMD_MADCTL_MH							(1<<2)	// Display data latch order right to left
#define LCDMTRX_CMD_MADCTL_RGB							(0<<3)	// Display RGB order
#define LCDMTRX_CMD_MADCTL_BGR							(1<<3)	// Display BGR order
#define LCDMTRX_CMD_MADCTL_ML							(1<<4)	// Line address order, refresh bottom to top
#define LCDMTRX_CMD_MADCTL_MV							(1<<5)	// Page/Column order reverse
#define LCDMTRX_CMD_MADCTL_MX							(1<<6)	// Column address order, right to left
#define LCDMTRX_CMD_MADCTL_MY							(1<<7)	// Page address order bottom to top



#define LCDMTRX_CMD_VSCROLL_START_ADDR			0x37	// Vertical scrolling start address
#define LCDMTRX_CMD_IDLEMODE_OFF				0x38	// Idle mode off
#define LCDMTRX_CMD_IDMODE_ON					0x39	// Idle mode on

#define LCDMTRX_CMD_COLMOD						0x3A	// Interface pixel format
#define LCDMTRX_CMD_COLMOD_COLOR_FMT_MASK				(7<<0)	// Control interface color format mask
#define LCDMTRX_CMD_COLMOD_COLOR_FMT_12					(3<<0)	// 	12 bits/pixel
#define LCDMTRX_CMD_COLMOD_COLOR_FMT_16					(5<<0)	// 	16 bits/pixel
#define LCDMTRX_CMD_COLMOD_COLOR_FMT_18					(6<<0)	// 	18 bits/pixel
#define LCDMTRX_CMD_COLMOD_COLOR_FMT_24					(7<<0)	// 	24 bits/pixel
#define LCDMTRX_CMD_COLMOD_RGB_INTRF_MASK				(7<<4)	//
#define LCDMTRX_CMD_COLMOD_RGB_INTRF_4K					(3<<4)	// 	4K
#define LCDMTRX_CMD_COLMOD_RGB_INTRF_65K				(5<<4)	// 	65K
#define LCDMTRX_CMD_COLMOD_RGB_INTRF_262K				(6<<4)	// 	262K
#define LCDMTRX_CMD_COLMOD_RGB_INTRF_16M				(6<<4)	// 	16M


#define LCDMTRX_CMD_RAMWRC						0x3C	// Memory write continue
#define LCDMTRX_CMD_RAMRDC						0x3E	// Memory read continue
#define LCDMTRX_CMD_TEAR_SCANLINE				0x44	// Set tearing scanline
#define LCDMTRX_CMD_RD_SCANLINE					0x45	// Get scanline
#define LCDMTRX_CMD_WR_DISBV					0x51	// Write display brightness value
#define LCDMTRX_CMD_RD_DISBV					0x52	// Read display brightness value

#define LCDMTRX_CMD_WR_CTRLD					0x53	// Write control display
#define LCDMTRX_CMD_WR_CTRLD_BL_ON						(1<<2)	// Backlight on
#define LCDMTRX_CMD_WR_CTRLD_DD_ON						(1<<3)	// Dimming on
#define LCDMTRX_CMD_WR_CTRLD_BCTRL_ON					(1<<5)	// Brightness control on


#define LCDMTRX_CMD_RD_CTRLD					0x54	// Read control display
#define LCDMTRX_CMD_WR_CACE						0x55	// Write content adaptive brightness control & color enhancement
#define LCDMTRX_CMD_RD_CABC						0x56	// Read content adaptive brightness control
#define LCDMTRX_CMD_WR_CABCMB					0x5E	// Write CABC minimum brightness
#define LCDMTRX_CMD_RD_CABCMB					0x5F	// Read CABC minimum brightness
#define LCDMTRX_CMD_RD_ABCSDR					0x68	// Read automatic brightness control self-diagnostic result
#define LCDMTRX_RD_BWLB							0x70	// Read black/white low bits
#define LCDMTRX_RD_BKX							0x71	// Read Black X
#define LCDMTRX_RD_BKY							0x72	// Read black Y
#define LCDMTRX_CMD_RD_WX						0x73	// Read white X
#define LCDMTRX_CMD_RD_WY						0x74	// Read white Y
#define LCDMTRX_RD_RGLB							0x75	// Read read green low bits
#define LCDMTRX_RD_REDX							0x76	// Read red X
#define LCDMTRX_RD_REDY							0x77	// Read red Y
#define LCDMTRX_RD_GREENX						0x78	// Read green X
#define LCDMTRX_RD_GREENY						0x79	// Read green Y
#define LCDMTRX_RD_BALB							0x7A	// Read blue alpha low bits
#define LCDMTRX_RD_BLUEX						0x7B	// Read blue X
#define LCDMTRX_RD_BLUEY						0x7C	// Read blue Y
#define LCDMTRX_RD_AX							0x7D	// Read alpha X
#define LCDMTRX_RD_AY							0x7E	// Read alpha Y
#define LCDMTRX_RD_DDB							0xA1	// Read DDB at set location
#define LCDMTRX_RD_DDB_CONT						0xA8	// Read DDB continue
#define LCDMTRX_RD_FCS							0xAA	// Read first checksum
#define LCDMTRX_RD_CCS							0xAF	// Read continue checksum
#define LCDMTRX_CMD_RD_ID1						0xDA	// Read ID1
#define LCDMTRX_CMD_RD_ID2						0xDB	// Read ID2
#define LCDMTRX_CMD_RD_ID3						0xDC	// Read ID3


class LCDMatrix : public Display {
public:
	virtual bool Init(DisplayCfg_t &, DeviceIntrf *pIntrf);

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
	 * @brief	Set display orientation
	 *
	 * @param	Orient	: Orientation to set
	 *
	 * @return	None
	 */
	virtual void Orientation(DISPL_ORIENT Orient);

	/**
	 * @brief	Get current orientation
	 *
	 * @return	Current orientation
	 */
	virtual DISPL_ORIENT Orientation() { return Display::Orientation(); }

	/**
	 * @brief	Reset device to it initial default state
	 */
	virtual void Reset();

	/**
	 * @brief	Clear screen
	 *
	 * @return	None
	 */
	virtual void Clear();

	/**
	 * @brief	Set current rendering location
	 *
	 * @param 	Col	:
	 * @param 	Row	:
	 */
	virtual void SetCurrent(uint16_t Col, uint16_t Row) {
		SetRamWrRegion(Col, Row, vWidth - Col + 1, vHeight - Row + 1);
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
	virtual void Fill(uint16_t X, uint16_t Y, uint16_t Width, uint16_t Height, uint32_t Color);

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
	virtual void BitBlt(uint16_t X, uint16_t Y, uint16_t Width, uint16_t Height, uint8_t *pBuffer);

	/**
	 * @brief	Draw line on the screen
	 *
	 * Option function to draw a line on matrix display
	 *
	 * @param 	StartX	: Start X coordinate
	 * @param 	StartY	: Start Y coordinate
	 * @param 	EndX	: End X coordinate
	 * @param 	EndY	: End Y coordinate
	 * @param	Color	: Pixel color
	 */
	virtual void Line(uint16_t StartX, uint16_t StartY, uint16_t EndX, uint16_t EndY, uint32_t Color);

	/**
	 * @brief	Display text string at location
	 *
	 * Print a zero terminated string to the screen using the current font
	 *
	 * @param 	Col		: X coordinate
	 * @param 	Row		: Y coordinate
	 * @param 	pStr	: Zero terminated string
	 */
	virtual void Text(uint16_t Col, uint16_t Row, char *pStr);

	/**
	 * @brief	Scroll display (optional)
	 *
	 * Scroll the screen in the specified direction by n steps.
	 * Step	:	1 char/line for character based display
	 * 		   	1 pixel for graphics based display
	 *
	 * @param 	Dir		: Direction of scroll
	 * @param 	Count	: Number of step to scroll
	 *
	 * @return	None
	 */
	virtual void Scroll(DISPL_SCROLL_DIR Dir, uint16_t Count);

	virtual void Print(char *pStr, uint32_t Color);

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
	virtual int Read(uint8_t *pCmdAddr, int CmdAddrLen, uint8_t *pBuff, int BuffLen);

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
	virtual int Write(uint8_t *pCmdAddr, int CmdAddrLen, uint8_t *pData, int DataLen);

protected:
	void SetRamWrRegion(uint16_t X, uint16_t Y, uint16_t With, uint16_t Height);

	uint8_t *vpLineBuff;

private:
	int vPixelLen;
	uint8_t vMadCtl;
	uint16_t vCurScrollLine;	//!< Keep track of scrolling (rotation)
};


#endif // __DISPLAY_LCD_H__
