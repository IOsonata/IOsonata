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

#include "display/display_lcdmtrx.h"

// ST7789 default portrait
#define ST7789_WIDTH_MAX					240
#define ST7789_HEIGHT_MAX					320

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

class LcdST77xx : public LCDMatrix {
public:
	bool Init(DisplayCfg_t &, DeviceIntrf *pIntrf);


private:
	uint8_t vLineBuff[ST7789_LINE_BUFFLEN];
};


#endif // __DISPLAY_ST77XX_H__
