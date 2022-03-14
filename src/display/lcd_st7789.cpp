/**-------------------------------------------------------------------------
@file	lcd_st7789.h

@brief	ST7789 LCD display controller definitions


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
#include "idelay.h"
#include "convutil.h"
#include "iopinctrl.h"
#include "display/lcd_st7789.h"

bool LcdST77xx::Init(LcdDisplayCfg_t &Cfg, DeviceIntrf *pIntrf)
{
	Interface(pIntrf);

	vCfg = Cfg;

	DeviceAddress(vCfg.DevAddr);

	IOPinCfg(vCfg.pPins, vCfg.NbPins);

	uint8_t cmd = ST7789_CMD_SWRESET;
	uint32_t d = 6;
	Write(&cmd, 1, NULL, 0);

	msDelay(200);

	cmd = ST7789_CMD_SLPOUT;
	Write(&cmd, 1, NULL, 0);

	uint16_t ramctrl = 0x0d000;// | ST77XX_CMD_RAMCTRL_ENDIAN_LITTLE;
	uint8_t madctl = 0;

	cmd = ST7789_CMD_COLMOD;
	switch (vCfg.PixelSize)
	{
		case 12:
			d = ST7789_CMD_COLMOD_COLOR_FMT_12 | ST7789_CMD_COLMOD_RGB_INTRF_4K;
			vPixelLen = 2;
			break;
		case 16:
			ramctrl |= ST77XX_CMD_RAMCTRL_ENDIAN_LITTLE;
			madctl |= ST7789_CMD_MADCTL_BGR;
			d = ST7789_CMD_COLMOD_COLOR_FMT_16 | ST7789_CMD_COLMOD_RGB_INTRF_65K;
			vPixelLen = 2;
			break;
		case 18:
			d = ST7789_CMD_COLMOD_COLOR_FMT_18 | ST7789_CMD_COLMOD_RGB_INTRF_262K;
			vPixelLen = 3;
			break;
	}
	Write(&cmd, 1, (uint8_t*)&d, 1);

	if (vCfg.Orient == DISPL_ORIENT_LANDSCAPE)
	{
		madctl |= ST7789_CMD_MADCTL_MV;// | ST7789_CMD_MADCTL_MY;
	}
	else
	{
		madctl |= 	ST7789_CMD_MADCTL_MX;
	}
	cmd = ST7789_CMD_MADCTL;
	Write(&cmd, 1, (uint8_t*)&madctl, 1);

#if 1
	cmd = ST77XX_CMD_RAMCTRL;
//	d = ST77XX_CMD_RAMCTRL_ENDIAN_LITTLE | ;
	Write(&cmd, 1, (uint8_t*)&ramctrl, 2);

//	cmd = ST77XX_CMD_RGBCTRL;
//	d = 0x140200;
//	Write(&cmd, 1, (uint8_t*)&d, 3);
	cmd = ST77XX_CMD_LCMCTRL;
	d = 0;
	Write(&cmd, 1, (uint8_t*)&d, 1);
#endif
/*
	cmd = ST7789_CMD_CASET;
	d = (EndianCvt16(vCfg.HLen - 1) << 16);

	Write(&cmd, 1, (uint8_t*)&d, 4);

	cmd = ST7789_CMD_RASET;
	d = (EndianCvt16(vCfg.VLen - 1) << 16);
	Write(&cmd, 1, (uint8_t*)&d, 4);
*/
//	SetRamWrRegion(0, 0, vCfg.HLen, vCfg.VLen);

	cmd = ST7789_CMD_INVON;
	Write(&cmd, 1, NULL, 0);

	cmd = ST7789_CMD_NORON;
	Write(&cmd, 1, NULL, 0);

	cmd = ST7789_CMD_DISPON;
	Write(&cmd, 1, NULL, 0);

	return true;
}

/**
 * @brief	Power on or wake up device
 *
 * @return	true - If success
 */
bool LcdST77xx::Enable()
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
void LcdST77xx::Disable()
{

}

/**
 * @brief	Reset device to it initial default state
 */
void LcdST77xx::Reset()
{

}

void LcdST77xx::Clear()
{
	int line = vCfg.HLen * vPixelLen;

	memset(vLineBuff, 0, line);
#if 0
	uint16_t *p = (uint16_t*)vLineBuff;

	for (int i = 0; i < vCfg.HLen; i++)
	{
		p[i] = (0xf800);
	}
#endif
#if 1
	SetRamWrRegion(0, 0, vCfg.HLen, vCfg.VLen);

//	uint32_t d = EndianCvt16(0xf800);
	uint8_t cmd = ST7789_CMD_RAMWR;
	Write(&cmd, 1, vLineBuff, line);

	//cmd = ST7789_CMD_RAMWRC;
	for (int i = 1; i < vCfg.VLen; i++)
	{
		Write(0, 0, vLineBuff, line);
	}
#else
	uint32_t d = (0xf800);//EndianCvt16(0xf800) | 0xf80000;
	uint8_t cmd = ST7789_CMD_RAMWR;

	//cmd = ST7789_CMD_RAMWRC;
	for (int j = 0; j < vCfg.VLen; j++)
	{
		//Write(&cmd, 1, 0, 0);//(uint8_t*)&d, 2);
		for (int i = 0; i <vCfg.HLen; i++)
		{
			SetRamWrRegion(i, j, 1, 1);
			Write(&cmd, 1, (uint8_t*)&d, 2);
		}
	}
#endif
}

void LcdST77xx::SetPixel(uint16_t X, uint16_t Y, uint32_t Color)
{
	SetRamWrRegion(X, Y, 1, 1);
	uint8_t cmd = ST7789_CMD_RAMWR;
	Write(&cmd, 1, (uint8_t*)&Color, vPixelLen);
}

void LcdST77xx::BitBlt(uint16_t X, uint16_t Y, uint16_t Width, uint16_t Height, uint8_t *pBuffer)
{
	SetRamWrRegion(X, Y, Width, Height);
	uint8_t cmd = ST7789_CMD_RAMWR;
	uint32_t l = Width * Height * vPixelLen;
	Write(&cmd, 1, pBuffer, Width * Height * vPixelLen);
}

void LcdST77xx::Backlight(bool bOn)
{
	uint8_t cmd = ST7789_CMD_WRCTRLD;
	uint8_t d = ST7789_CMD_WRCTRLD_BCTRL_ON;
	if (bOn == true)
	{
		d |= ST7789_CMD_WRCTRLD_BL_ON | ST7789_CMD_WRCTRLD_DD_ON;
	}
	Write(&cmd, 1, &d, 1);
}

void LcdST77xx::SetRamWrRegion(uint16_t X, uint16_t Y, uint16_t Width, uint16_t Height)
{
	uint8_t cmd = ST7789_CMD_CASET;
	uint32_t d = (EndianCvt16(X) & 0xFFFF) | (EndianCvt16(X + Width - 1) << 16);
	Write(&cmd, 1, (uint8_t*)&d, 4);

	cmd = ST7789_CMD_RASET;
	d = (EndianCvt16(Y) & 0xFFFF) | (EndianCvt16(Y + Height - 1) << 16);
	Write(&cmd, 1, (uint8_t*)&d, 4);
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
int LcdST77xx::Write(uint8_t *pCmdAddr, int CmdAddrLen, uint8_t *pData, int DataLen)
{
	int cnt = 0;
	vpIntrf->StartTx(DeviceAddress());

	if (pCmdAddr && CmdAddrLen > 0)
	{
		IOPinClear(vCfg.pPins[LCD_CTRL_DCX_PINIDX].PortNo, vCfg.pPins[LCD_CTRL_DCX_PINIDX].PinNo);

		cnt += vpIntrf->TxData(pCmdAddr, CmdAddrLen);
	}
	if (pData != NULL && DataLen > 0)
	{
		IOPinSet(vCfg.pPins[LCD_CTRL_DCX_PINIDX].PortNo, vCfg.pPins[LCD_CTRL_DCX_PINIDX].PinNo);
		cnt += vpIntrf->TxData(pData, DataLen);
	}
	vpIntrf->StopTx();

	return cnt;
}
