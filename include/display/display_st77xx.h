/**-------------------------------------------------------------------------
@file	display_st77xx.h

@brief	ST77xx LCD display controller implementation


@author	Hoang Nguyen Hoan
@date	Mar. 9, 2022

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
#ifndef __DISPLAY_ST77XX_H__
#define __DISPLAY_ST77XX_H__

#include "display.h"

// ST7789 default portrait
#define ST7789_WIDTH_MAX					240
#define ST7789_HEIGHT_MAX					320

#define ST77XX_CMD_NOP						0		// NOP
#define ST77XX_CMD_SWRESET					1		// Software reset
#define ST77XX_CMD_RDDID					4		// Read display ID
#define ST77XX_CMD_RDDST					9		// Read display status
#define ST77XX_CMD_RDDPM					0xA		// Read display power
#define ST77XX_CMD_RDD_MADCTL				0xB		// Read display
#define ST77XX_CMD_RDD_COLMOD				0xC		// Read display pixel
#define ST77XX_CMD_RDDIM					0xD		// Read display image
#define ST77XX_CMD_RDDSM					0xE		// Read display signal
#define ST77XX_CMD_RDDSDR					0xF		// Read display self-diagnostic result
#define ST77XX_CMD_SLPIN					0x10	// Sleep in
#define ST77XX_CMD_SLPOUT					0x11	// Sleep out
#define ST77XX_CMD_PTLON					0x12	// Partial mode on
#define ST77XX_CMD_NORON					0x13	// Partial mode off
#define ST77XX_CMD_INVOFF					0x20	// Display inversion off
#define ST77XX_CMD_INVON					0x21	// Display inversion on
#define ST77XX_CMD_GAMSET					0x26	// Gamma set
#define ST77XX_CMD_DISPOFF					0x28	// Display off
#define ST77XX_CMD_DISPON					0x29	// Display on
#define ST77XX_CMD_CASET					0x2A	// Column address set
#define ST77XX_CMD_RASET					0x2B	// Row address set
#define ST77XX_CMD_RAMWR					0x2C	// Memory write
#define ST77XX_CMD_RAMRD					0x2E	// Memory read
#define ST77XX_CMD_PTLAR					0x30	// Partial start/end address set
#define ST77XX_CMD_VSCRDEF					0x33	// Vertical scrolling definition
#define ST77XX_CMD_TEOFF					0x34	// Tearing effect line off
#define ST77XX_CMD_TEON						0x35	// Tearing effect line on

#define ST77XX_CMD_MADCTL					0x36	// Memory data access control
#define ST77XX_CMD_MADCTL_MH						(1<<2)	// Display data latch order right to left
#define ST77XX_CMD_MADCTL_RGB						(0<<3)	// Display RGB order
#define ST77XX_CMD_MADCTL_BGR						(1<<3)	// Display BGR order
#define ST77XX_CMD_MADCTL_ML						(1<<4)	// Line address order, refresh bottom to top
#define ST77XX_CMD_MADCTL_MV						(1<<5)	// Page/Column order reverse
#define ST77XX_CMD_MADCTL_MX						(1<<6)	// Column address order, right to left
#define ST77XX_CMD_MADCTL_MY						(1<<7)	// Page address order bottom to top



#define ST77XX_CMD_VSCRSADD					0x37	// Vertical scrolling start address
#define ST77XX_CMD_IDMOFF					0x38	// Idle mode off
#define ST77XX_CMD_IDMON					0x39	// Idle mode on

#define ST77XX_CMD_COLMOD					0x3A	// Interface pixel format
#define ST77XX_CMD_COLMOD_COLOR_FMT_MASK			(7<<0)	// Control interface color format mask
#define ST77XX_CMD_COLMOD_COLOR_FMT_12				(3<<0)	// 	12 bits/pixel
#define ST77XX_CMD_COLMOD_COLOR_FMT_16				(5<<0)	// 	16 bits/pixel
#define ST77XX_CMD_COLMOD_COLOR_FMT_18				(6<<0)	// 	18 bits/pixel
#define ST77XX_CMD_COLMOD_COLOR_FMT_16M				(7<<0)	// 	16M truncated
#define ST77XX_CMD_COLMOD_RGB_INTRF_MASK			(7<<4)	//
#define ST77XX_CMD_COLMOD_RGB_INTRF_4K				(3<<4)	// 	4K
#define ST77XX_CMD_COLMOD_RGB_INTRF_65K				(5<<4)	// 	65K
#define ST77XX_CMD_COLMOD_RGB_INTRF_262K			(6<<4)	// 	262K


#define ST77XX_CMD_RAMWRC					0x3C	// Memory write continue
#define ST77XX_CMD_RAMRDC					0x3E	// Memory read continue
#define ST77XX_CMD_TESCAN					0x44	// Set tearing scanline
#define ST77XX_CMD_RDTESCAN					0x45	// Get scanline
#define ST77XX_CMD_WRDISBV					0x51	// Write display brightness value
#define ST77XX_CMD_RDDISBV					0x52	// Read display brightness value

#define ST77XX_CMD_WRCTRLD					0x53	// Write control display
#define ST77XX_CMD_WRCTRLD_BL_ON					(1<<2)	// Backlight on
#define ST77XX_CMD_WRCTRLD_DD_ON					(1<<3)	// Dimming on
#define ST77XX_CMD_WRCTRLD_BCTRL_ON					(1<<5)	// Brightness control on


#define ST77XX_CMD_RDCTRLD					0x54	// Read control display
#define ST77XX_CMD_WRCACE					0x55	// Write content adaptive brightness control & color enhancement
#define ST77XX_CMD_RDCABC					0x56	// Read content adaptive brightness control
#define ST77XX_CMD_WRCABCMB					0x5E	// Write CABC minimum brightness
#define ST77XX_CMD_RDCABCMB					0x5F	// Read CABC minimum brightness
#define ST77XX_CMD_RDABCSDR					0x68	// Read automatic brightness control self-diagnostic result
#define ST77XX_CMD_RDID1					0xDA	// Read ID1
#define ST77XX_CMD_RDID2					0xDB	// Read ID2
#define ST77XX_CMD_RDID3					0xDC	// Read ID3

#define ST77XX_CMD_RAMCTRL					0xB0	// RAM control
#define ST77XX_CMD_RAMCTRL_DM_MASK					(3<<0)
#define ST77XX_CMD_RAMCTRL_RM_RGB					(1<<4)	// RAM access RBG
#define ST77XX_CMD_RAMCTRL_ENDIAN_LITTLE			(1<<11)	// Little endian

#define ST77XX_CMD_RGBCTRL					0xB1	// RGB control
#define ST77XX_CMD_PORCTRL					0xB2	// Porch control
#define ST77XX_CMD_FRCTRL1					0xB3	// Frame rate control1
#define ST77XX_CMD_PARCTRL					0xB5	// Partial control
#define ST77XX_CMD_GCTRL					0xB7	// Gate control
#define ST77XX_CMD_GTADJ					0xB8	// Gate on timing adjustment
#define ST77XX_CMD_DGMEN					0xBA	// Digital gamma enable
#define ST77XX_CMD_VCOMS					0xBB	// VCOM settings
#define ST77XX_CMD_POWSAVE					0xBC	// Power saving mode

#define ST77XX_CMD_DLPOFFSAVE				0xBD	// Display off power save
#define ST77XX_CMD_LCMCTRL					0xC0	// LCM control
#define ST77XX_CMD_IDSET					0xC1	// ID setting
#define ST77XX_CMD_VDVVRHEN					0xC2	// VDV and VRH command enable
#define ST77XX_CMD_VRHS						0xC3	// VRH set
#define ST77XX_CMD_VDVSET					0xC4	// VDV setting
#define ST77XX_CMD_VCOMFSET					0xC5	// VCOM offset set
#define ST77XX_CMD_FRCTRL2					0xC6	// FR control 2
#define ST77XX_CMD_CABCCTRL					0xC7	// CABC control
#define ST77XX_CMD_REGSEL1					0xC8	// Register value selection1
#define ST77XX_CMD_REGSEL2					0xCA	// Register value selection2
#define ST77XX_CMD_PWMFRSEL					0xCC	// PWM frequency selection
#define ST77XX_CMD_PWCTRL1					0xD0	// Power control1
#define ST77XX_CMD_VAPVANEN					0xD2	// Enable VAP/VAN signal output
#define ST77XX_CMD_CMD2EN					0xDF	// Command 2 enable
#define ST77XX_CMD_PVGAMCTRL				0xE0	// Positive voltage gamma control
#define ST77XX_CMD_NVGAMCTRL				0xE1	// Negative voltage gamma control
#define ST77XX_CMD_DGMLUTR					0xE2	// Digital gamma lookup table for red
#define ST77XX_CMD_DGMLUTB					0xE3	// Digital gamma lookup table for blue
#define ST77XX_CMD_GATECTRL					0xE4	// Gate control
#define ST77XX_CMD_SPI2EN					0xE7	// SPI2 enable
#define ST77XX_CMD_PWCTRL2					0xE8	// Power control 2
#define ST77XX_CMD_EQCTRL					0xE9	// Equalize time control
#define ST77XX_CMD_PROMCTRL					0xEC	// Program control
#define ST77XX_CMD_PROMEN					0xFA	// Program mode
#define ST77XX_CMD_NVMSET					0xFC	// NVM setting
#define ST77XX_CMD_PROMACT					0xFE	// Program action

#define ST7789_LINE_BUFFLEN					(ST7789_HEIGHT_MAX << 2)

class DisplST77xx : public Display {
public:
	bool Init(DisplayCfg_t &, DeviceIntrf *pIntrf);

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
	 * @brief	Reset device to it initial default state
	 */
	virtual void Reset();

	virtual void Clear();
	virtual void Fill(uint16_t X, uint16_t Y, uint16_t Width, uint16_t Height, uint32_t Color);

	virtual void Backlight(bool bOn);

	virtual void SetPixel(uint16_t X, uint16_t Y, uint32_t Color);
	virtual void BitBlt(uint16_t X, uint16_t Y, uint16_t Width, uint16_t Height, uint8_t *pBuffer);

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

private:
	uint8_t vLineBuff[ST7789_LINE_BUFFLEN];
	int vPixelLen;
};


#endif // __DISPLAY_ST77XX_H__
