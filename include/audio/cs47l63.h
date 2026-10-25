/**-------------------------------------------------------------------------
@file	cs47l63.h
@brief	Cirrus Logic CS47L63 control-interface driver
@author	Hoang Nguyen Hoan
@date	Oct. 24, 2026
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
#ifndef __AUDIO_CS47L63_H__
#define __AUDIO_CS47L63_H__

#include <stddef.h>
#include <stdint.h>
#include "device.h"

#define CS47L63_DEVICE_ID			0x047A63U
#define CS47L63_REG_DEVID			0x00000000U
#define CS47L63_REG_REVID			0x00000004U
#define CS47L63_SPI_READ			0x80000000U
#define CS47L63_SPI_ADDR_MASK		0x7FFFFFFFU

typedef struct __Cs47l63_Config
{
	uint32_t DevAddr;
	IOPinCfg_t ResetPin;
	IOPinCfg_t IrqPin;
	uint32_t BootTimeoutMs;
} Cs47l63Cfg_t;

typedef struct __Cs47l63_Reg_Val
{
	uint32_t Reg;
	uint32_t Value;
	uint16_t DelayMs;
} Cs47l63RegVal_t;

class Cs47l63 : public Device
{
public:
	bool Init(const Cs47l63Cfg_t &Cfg, DeviceIntrf *pIntrf);
	bool Enable();
	void Disable();
	void Reset();
	void PowerOff();

	bool ReadReg(uint32_t Reg, uint32_t &Value);
	bool WriteReg(uint32_t Reg, uint32_t Value);
	bool UpdateReg(uint32_t Reg, uint32_t Mask, uint32_t Value);
	bool ReadBlock(uint32_t Reg, uint8_t *pData, size_t DataLen);
	bool WriteBlock(uint32_t Reg, const uint8_t *pData, size_t DataLen);
	bool Apply(const Cs47l63RegVal_t *pConfig, size_t Count);

	uint8_t Revision() const { return vRevision; }
	bool IrqActive() const;

private:
	bool Boot();
	bool WaitBoot();
	bool PinValid(const IOPinCfg_t &Pin) const;

	Cs47l63Cfg_t vCfg{};
	uint8_t vRevision = 0;
	bool vbEnabled = false;
};

#endif
