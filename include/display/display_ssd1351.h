/**-------------------------------------------------------------------------
@file	display_ssd1351.h

@brief	SSD1351 LCD display controller implementation

128 RGB x 128 Dot Matrix OLED/PLED Segment/Common Driver with Controller

@author	Hoang Nguyen Hoan
@date	Jul. 5, 2023

@license

MIT License

Copyright (c) 2023, I-SYST inc., all rights reserved

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
#ifndef __DISPLAY_SSD1351_H__
#define __DISPLAY_SSD1351_H__

#include "display/display_lcdmtrx.h"

// SSD1351 default portrait
#define SSD1351_WIDTH_MAX					128
#define SSD1351_HEIGHT_MAX					128

#define SSD1351_CMD_RAMCTRL					0xB0	// RAM control
#define SSD1351_CMD_RAMCTRL_DM_MASK					(3<<0)
#define SSD1351_CMD_RAMCTRL_RM_RGB					(1<<4)	// RAM access RBG
#define SSD1351_CMD_RAMCTRL_ENDIAN_LITTLE			(1<<11)	// Little endian

#define SSD1351_CMD_RGBCTRL					0xB1	// RGB control
#define SSD1351_CMD_PORCTRL					0xB2	// Porch control
#define SSD1351_CMD_FRCTRL1					0xB3	// Frame rate control1
#define SSD1351_CMD_PARCTRL					0xB5	// Partial control
#define SSD1351_CMD_GCTRL					0xB7	// Gate control
#define SSD1351_CMD_GTADJ					0xB8	// Gate on timing adjustment
#define SSD1351_CMD_DGMEN					0xBA	// Digital gamma enable
#define SSD1351_CMD_VCOMS					0xBB	// VCOM settings
#define SSD1351_CMD_POWSAVE					0xBC	// Power saving mode

#define SSD1351_CMD_DLPOFFSAVE				0xBD	// Display off power save
#define SSD1351_CMD_LCMCTRL					0xC0	// LCM control
#define SSD1351_CMD_IDSET					0xC1	// ID setting
#define SSD1351_CMD_VDVVRHEN					0xC2	// VDV and VRH command enable
#define SSD1351_CMD_VRHS						0xC3	// VRH set
#define SSD1351_CMD_VDVSET					0xC4	// VDV setting
#define SSD1351_CMD_VCOMFSET					0xC5	// VCOM offset set
#define SSD1351_CMD_FRCTRL2					0xC6	// FR control 2
#define SSD1351_CMD_CABCCTRL					0xC7	// CABC control
#define SSD1351_CMD_REGSEL1					0xC8	// Register value selection1
#define SSD1351_CMD_REGSEL2					0xCA	// Register value selection2
#define SSD1351_CMD_PWMFRSEL					0xCC	// PWM frequency selection
#define SSD1351_CMD_PWCTRL1					0xD0	// Power control1
#define SSD1351_CMD_VAPVANEN					0xD2	// Enable VAP/VAN signal output
#define SSD1351_CMD_CMD2EN					0xDF	// Command 2 enable
#define SSD1351_CMD_PVGAMCTRL				0xE0	// Positive voltage gamma control
#define SSD1351_CMD_NVGAMCTRL				0xE1	// Negative voltage gamma control
#define SSD1351_CMD_DGMLUTR					0xE2	// Digital gamma lookup table for red
#define SSD1351_CMD_DGMLUTB					0xE3	// Digital gamma lookup table for blue
#define SSD1351_CMD_GATECTRL					0xE4	// Gate control
#define SSD1351_CMD_SPI2EN					0xE7	// SPI2 enable
#define SSD1351_CMD_PWCTRL2					0xE8	// Power control 2
#define SSD1351_CMD_EQCTRL					0xE9	// Equalize time control
#define SSD1351_CMD_PROMCTRL					0xEC	// Program control
#define SSD1351_CMD_PROMEN					0xFA	// Program mode
#define SSD1351_CMD_NVMSET					0xFC	// NVM setting
#define SSD1351_CMD_PROMACT					0xFE	// Program action

#define SSD1351_LINE_BUFFLEN				(SSD1351_HEIGHT_MAX << 2)

class LcdSSD1351 : public LCDMatrix {
public:
	bool Init(DisplayCfg_t &, DeviceIntrf *pIntrf);


private:
	uint8_t vLineBuff[SSD1351_LINE_BUFFLEN];
};


#endif // __DISPLAY_SSD1351_H__
